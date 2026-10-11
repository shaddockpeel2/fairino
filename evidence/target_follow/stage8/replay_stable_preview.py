"""Compare old/stable Demo targets on recorded static data, without ROS/SDK."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml

from fr3c_target_follow.follow_geometry import transform, rotation_from_quaternion, transform_rpy
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.orientation_quality import OrientationGate, measurement_problem, stamp_ns
from fr3c_target_follow.pose_preview import PosePreview

ROOT = Path(__file__).resolve().parents[3]
INPUT = ROOT/'evidence/target_follow/stage6/manual_20261009_101620'
OUT = Path(__file__).resolve().parent/'stable_preview_replay'


def statistics(values):
    return dict(p95=float(np.percentile(values, 95)), max=float(np.max(values)))


def replay(rows, config, jitter, reclassify):
    parameters = yaml.safe_load(config.read_text())['pose_preview']['ros__parameters']
    preview = PosePreview(**{k: v for k, v in parameters.items()
                             if k not in ('auto_teach', 'target_id', 'publish_hz', 'feedback_timeout_s')})
    tcp = yaml.safe_load((ROOT/'src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml').read_text())['ydi_j22_tcp']
    first_flange = next(row for row in rows if row['kind'] == 'flange')
    actual = transform(first_flange['position_m'], first_flange['quaternion_xyzw']) @ transform_rpy(tcp['xyz_m'], tcp['rpy_rad'])
    gate = OrientationGate(confirmation_frames=2, jitter_allowance_deg=jitter)
    qualities = {stamp_ns(row['stamp']): row for row in rows if row['kind'] == 'quality' and row.get('stamp')}
    counts = Counter()
    positions, angles = [], []
    for row in rows:
        if row['kind'] != 'observation':
            continue
        if row.get('position_m') is None:
            if row['status'] in ('orientation_ambiguous', 'orientation_measurement_rejected', 'target_not_found'):
                quality = qualities[stamp_ns(row['stamp'])]
                if measurement_problem(reclassify(quality, .10)) is not None:
                    gate.confirmed = 0
            else:
                gate.reset()
            continue
        ns = stamp_ns(row['stamp'])
        result = gate.observe(ns, rotation_from_quaternion(row['quaternion_xyzw']),
                              measurement_problem(reclassify(row['quality'], .10)))
        counts[result['status']] += 1
        if not result['valid_orientation']:
            continue
        target = transform(row['position_m'], row['quaternion_xyzw'])
        if preview.offset is None:
            preview.teach(ns, target, actual)
        else:
            preview.observe(ns, target)
        goal = preview.goal()
        positions.append(np.linalg.norm(goal[:3, 3]-actual[:3, 3])*1000)
        angles.append(rotation_distance_deg(goal[:3, :3], actual[:3, :3]))
    return dict(parameters=parameters, jitter_allowance_deg=jitter, quality_counts=dict(counts),
                accepted=len(positions), position_from_recorded_tcp_mm=statistics(positions),
                rotation_from_recorded_tcp_deg=statistics(angles))


def main():
    spec = importlib.util.spec_from_file_location('recorded_quality_replay', INPUT/'replay_demo_thresholds.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = ROOT/'src/fr3c_target_follow/config'
    report = dict(hardware_servo_commands_sent=0, absolute_pose_accuracy_verified=False, cases={},
                  note='Offline relative targets on user-confirmed stationary recordings. Uses a fixed initial '
                       'flange as TCP feedback; target displacement includes record bias, not absolute accuracy. '
                       'No ROS holding/timing, camera-motion compensation test or robot response is simulated.')
    for case in ('static', 'static_more_pixels'):
        source = INPUT/case/'samples.jsonl'
        rows = [json.loads(line) for line in source.read_text().splitlines()]
        report['cases'][case] = dict(input_path=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            previous=replay(rows,config/'pose_preview.yaml',3.,module.reclassify),
            stable=replay(rows,config/'pose_preview_demo_stable.yaml',6.,module.reclassify))
    OUT.mkdir(exist_ok=True)
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for case, result in report['cases'].items():
        print(case, {name: {field: data[field] for field in ('position_from_recorded_tcp_mm', 'rotation_from_recorded_tcp_deg')}
                     for name, data in result.items() if name in ('previous', 'stable')})


if __name__ == '__main__':
    main()
