"""Read-only replay of recorded paired poses; never imports ROS or an SDK client."""
from collections import Counter
from pathlib import Path
import copy
import hashlib
import json

import numpy as np
from fr3c_target_follow.follow_geometry import rotation_from_quaternion
from fr3c_target_follow.pose_preview import rotation_exp, rotation_vector as rotation_log
from fr3c_target_follow.marker_pose import candidate_orientation_quality
from fr3c_target_follow.orientation_quality import OrientationGate, measurement_problem, stamp_ns


ROOT = Path(__file__).resolve().parents[4]
RUNS = ('20261010_125147_157700', '20261010_125324_158990', '20261010_125428_159911')


def replay(rows, tolerance):
    gate = OrientationGate(confirmation_frames=2, timeout_s=.35,
                           max_angular_speed_deg_s=60., jitter_allowance_deg=6.)
    statuses = Counter()
    paired = 0
    last_accepted = 0
    longest_gap = 0
    accepted_stamps = []
    for row in rows:
        if row['kind'] != 'observation':
            continue
        observation = row['observation']
        quality = observation.get('quality')
        if not quality or observation.get('position_m') is None:
            continue
        ns = stamp_ns(observation['stamp'])
        if ns <= gate.last_ns:
            continue
        paired += 1
        quality = copy.deepcopy(quality)
        quality.update(candidate_orientation_quality(quality['solver'],
            quality.get('candidate_rotation_distance_deg'), quality.get('candidate_margin_px'),
            quality['selected_candidate_index'], .10, tolerance))
        rotation = rotation_from_quaternion(observation['quaternion_xyzw'])
        result = gate.observe(ns, rotation, measurement_problem(quality))
        statuses[result['status']] += 1
        if result['valid_orientation']:
            if last_accepted:
                longest_gap = max(longest_gap, ns-last_accepted)
            last_accepted = ns
            accepted_stamps.append(ns)
    return dict(paired=paired, statuses=dict(statuses), accepted=statuses['valid'],
                accepted_fraction=statuses['valid']/paired if paired else 0,
                longest_gap_between_accepted_s=longest_gap/1e9,
                first_accepted_ns=accepted_stamps[0] if accepted_stamps else None)


def range_event(rows):
    initial = next(row['state'] for row in rows if row['kind'] == 'state' and row['state']['status'] == 'following')
    paused = next(row['state'] for row in rows if row['kind'] == 'state' and row['state']['status'] == 'paused_reteach')
    offset = np.asarray(next(row['offset_matrix'] for row in rows if row['kind'] == 'start'))
    observation = next(row['observation'] for row in rows if row['kind'] == 'observation' and
                       row['observation'].get('status') == 'valid' and
                       row['observation'].get('stamp') == paused['trusted_image_stamp'])
    r0 = rotation_from_quaternion(initial['target_base_quaternion_xyzw'])
    raw = rotation_from_quaternion(observation['quaternion_xyzw'])
    mapped = rotation_exp(.5*rotation_log(raw @ r0.T)) @ r0
    p = np.asarray(observation['position_m']) + mapped @ offset[:3, 3]
    angle = np.linalg.norm(rotation_log(mapped @ offset[:3, :3] @
        rotation_from_quaternion(initial['actual_tcp_quaternion_xyzw']).T))*180/np.pi
    distance = np.linalg.norm(p-initial['actual_tcp_m'])*1000
    return dict(stamp_s=paused['stamp_s'], raw_mapped_tcp_displacement_mm=float(distance),
                raw_mapped_tcp_rotation_deg=float(angle), prior_radius_mm=100., updated_radius_mm=200.,
                would_pass_updated_range=bool(distance <= 200 and angle <= 20))


def main():
    results = []
    for name in RUNS:
        directory = ROOT/'evidence/target_follow/stage9'/name
        path = directory/'samples.jsonl'
        rows = [json.loads(line) for line in path.open()]
        report = json.loads((directory/'report.json').read_text())
        item = dict(run=name, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    hardware_servo_commands_sent=report['hardware_servo_commands_sent'],
                    sdk_stop_codes={key: report.get(key) for key in ('stop_motion_code', 'servo_end_code', 'close_code')},
                    prior_policy_replay=replay(rows, 5.), updated_policy_replay=replay(rows, 35.))
        if report['starts']:
            item['first_range_pause'] = range_event(rows)
        results.append(item)
    output = dict(kind='offline_quality_policy_replay', hardware_commands_sent_by_analysis=0,
                  candidate_tolerance_deg=35., follow_radius_m=.20, runs=results,
                  limitations=['Uses recorded selected poses, not regenerated images or candidate selection.',
                               'New closed-loop motion changes the viewpoint; replay does not validate live behavior.',
                               'Range comparison covers the first recorded limit event only.'])
    path = Path(__file__).with_name('diagnosis_replay.json')
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
