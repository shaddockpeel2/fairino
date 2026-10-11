"""Read recorded geometry and SDK returns only; no ROS or hardware connection."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from fr3c_target_follow.follow_geometry import rotation_from_quaternion, transform_rpy
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.pose_preview import rotation_exp, rotation_vector


ROOT = Path(__file__).resolve().parents[4]
RUN = '20261010_140121_208977'


def main():
    source = ROOT/'evidence/target_follow/stage9'/RUN
    rows = [json.loads(line) for line in (source/'samples.jsonl').open()]
    states = [r['state'] for r in rows if r['kind'] == 'state']
    servos = [r for r in rows if r['kind'] == 'servo']
    session = json.loads((source/'session.json').read_text())
    params = session['driver_parameters']
    first = next(s for s in states if s['status'] == 'following')
    paused = next(s for s in states if s['status'] == 'paused_reteach')
    tool = transform_rpy(params['tcp_xyz_m'], params['tcp_rpy_rad'])
    flange = servos[0]['actual_flange_m_rad']
    center = transform_rpy(flange[:3], flange[3:]) @ tool
    offset = np.asarray(first['offset_matrix'])
    taught = center @ np.linalg.inv(offset)
    target_r = rotation_from_quaternion(paused['target_base_quaternion_xyzw'])
    mapped = rotation_exp(.5*rotation_vector(target_r @ taught[:3,:3].T)) @ taught[:3,:3]
    raw_goal_position = mapped @ offset[:3,3] + paused['target_base_m']
    angle = rotation_distance_deg(mapped @ offset[:3,:3], center[:3,:3])
    distance = float(np.linalg.norm(raw_goal_position-center[:3,3])*1000)
    transitions, previous = [], None
    for s in states:
        key = (s['status'], s['reason'])
        if key != previous:
            transitions.append({k: s[k] for k in ('stamp_s', 'status', 'reason', 'servo_commands_sent')})
            previous = key
    # This quiet-looking interval is evidence of small desired changes, not proof
    # that the physical target was stationary (there is no ground-truth sensor).
    quiet = [s for s in servos if first['stamp_s']+60 <= s['stamp_s'] < first['stamp_s']+70]
    p = np.asarray([s['desired_tcp_m'] for s in quiet])
    v = np.asarray([s['velocity_m_s'] for s in quiet])
    w = np.asarray([s['angular_velocity_deg_s'] for s in quiet])
    result = dict(run=RUN, source_sha256=hashlib.sha256((source/'samples.jsonl').read_bytes()).hexdigest(),
        hardware_commands_sent_by_analysis=0,
        servo_return_codes=dict(Counter(s['sdk_return_code'] for s in servos)),
        live_report=json.loads((source/'report.json').read_text()), transitions=transitions,
        range_event=dict(raw_target_change_deg=rotation_distance_deg(target_r,taught[:3,:3]),
                         mapped_tcp_change_deg=angle, raw_goal_distance_mm=distance,
                         old_radius_mm=200, old_angle_deg=20, new_radius_mm=500, new_angle_deg=90,
                         accepted_by_new_range=distance<=500 and angle<=90),
        small_change_interval=dict(seconds_after_start=[60,70], samples=len(quiet),
            desired_position_span_per_axis_mm=(np.ptp(p,axis=0)*1000).tolist(),
            nonzero_correction_fraction=float(np.mean((np.linalg.norm(v,axis=1)>1e-7) |
                                                      (np.linalg.norm(w,axis=1)>1e-4)))),
        limitations=['Recorded poses, no camera-image re-estimation or changed viewpoint simulation.',
                     'Small-change interval has no independent stationary-target ground truth.',
                     'New stationary hold and enlarged range require live confirmation.'])
    Path(__file__).with_name('diagnosis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('servo_return_codes','range_event','small_change_interval')},indent=2))


if __name__ == '__main__':
    main()
