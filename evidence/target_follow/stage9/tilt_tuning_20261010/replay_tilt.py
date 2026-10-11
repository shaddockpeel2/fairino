"""Read-only replay of tilt quality; no ROS or SDK connection."""
from collections import Counter
from pathlib import Path
import hashlib
import json

import numpy as np
from fr3c_target_follow.follow_geometry import rotation_from_quaternion
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.orientation_quality import OrientationGate, measurement_problem, stamp_ns
from fr3c_target_follow.pose_preview import rotation_exp, rotation_vector


ROOT = Path(__file__).resolve().parents[4]
RUN = '20261010_134555_196814'


def replay(observations, jitter):
    gate = OrientationGate(2, .35, 60., jitter)
    counts = Counter()
    for o in observations:
        ns = stamp_ns(o['stamp'])
        if ns <= gate.last_ns:
            continue
        result = gate.observe(ns, rotation_from_quaternion(o['quaternion_xyzw']), measurement_problem(o['quality']))
        counts[result['status']] += 1
    return dict(jitter_allowance_deg=jitter, statuses=dict(counts), paired=sum(counts.values()),
                accepted_fraction=counts['valid']/sum(counts.values()))


def main():
    directory = ROOT/'evidence/target_follow/stage9'/RUN
    source = directory/'samples.jsonl'
    rows = [json.loads(line) for line in source.open()]
    states = [r['state'] for r in rows if r['kind'] == 'state']
    observations = [r['observation'] for r in rows if r['kind'] == 'observation' and r['observation'].get('quality')]
    initial = next(s for s in states if s['status'] == 'following')
    last = next(s for s in reversed(states) if s['status'] == 'following')
    relocated = next(s for s in states if s['status'] == 'paused_reteach')
    r0 = rotation_from_quaternion(last['target_base_quaternion_xyzw'])
    r1 = rotation_from_quaternion(relocated['target_base_quaternion_xyzw'])
    anchor = rotation_from_quaternion(initial['target_base_quaternion_xyzw'])
    mapped0 = rotation_exp(.5*rotation_vector(r0@anchor.T))@anchor
    mapped1 = rotation_exp(.5*rotation_vector(r1@anchor.T))@anchor
    report = json.loads((directory/'report.json').read_text())
    codes = Counter(r['sdk_return_code'] for r in rows if r['kind'] == 'servo')
    output = dict(run=RUN, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        hardware_commands_sent_by_this_analysis=0, live_report=report, live_servo_return_codes=dict(codes),
        recovery_event=dict(raw_angle_gap_deg=rotation_distance_deg(r0,r1),
            raw_position_gap_mm=float(np.linalg.norm(np.asarray(last['target_base_m'])-relocated['target_base_m'])*1000),
            mapped_angle_gap_deg=rotation_distance_deg(mapped0,mapped1),
            prior_raw_angle_threshold_deg=20., updated_raw_angle_threshold_deg=40.),
        prior_policy_replay=replay(observations,6.), updated_policy_replay=replay(observations,25.),
        limitations=['Uses recorded poses and selected candidates, not regenerated camera images.',
                     'Gate replay does not simulate the changed live robot viewpoint.',
                     'Updated live tilt following has not yet been validated.'])
    path = Path(__file__).with_name('diagnosis_replay.json')
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({key: output[key] for key in ('live_servo_return_codes','recovery_event','prior_policy_replay','updated_policy_replay')}, indent=2))


if __name__ == '__main__':
    main()
