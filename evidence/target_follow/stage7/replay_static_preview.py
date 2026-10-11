"""Offline Stage 7 Demo preview on the user's stationary-target recordings."""
from collections import Counter
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from fr3c_target_follow.follow_geometry import transform, rotation_from_quaternion, transform_rpy
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.orientation_quality import OrientationGate, measurement_problem, stamp_ns
from fr3c_target_follow.pose_preview import PosePreview


ROOT = Path(__file__).resolve().parents[3]
INPUT = ROOT/'evidence/target_follow/stage6/manual_20261009_101620'
OUTPUT = Path(__file__).resolve().parent/'static_replay'


def metrics(values):
    return dict(median=float(np.median(values)), p95=float(np.percentile(values, 95)), max=float(np.max(values)))


def main():
    OUTPUT.mkdir(exist_ok=True)
    spec = importlib.util.spec_from_file_location('threshold_replay', INPUT/'replay_demo_thresholds.py')
    replay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(replay)
    tcp = yaml.safe_load((ROOT/'src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml').read_text())['ydi_j22_tcp']
    flange_tcp = transform_rpy(tcp['xyz_m'], tcp['rpy_rad'])
    figure, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    results = {}
    for column, case in enumerate(('static', 'static_more_pixels')):
        rows = [json.loads(line) for line in (INPUT/case/'samples.jsonl').read_text().splitlines()]
        qualities = {stamp_ns(row['stamp']): row for row in rows if row['kind'] == 'quality' and row.get('stamp')}
        robot = next(row for row in rows if row['kind'] == 'flange')
        actual = transform(robot['position_m'], robot['quaternion_xyzw']) @ flange_tcp
        gate, preview = OrientationGate(confirmation_frames=2), PosePreview()
        times, raw_position, goal_position, raw_angle, goal_angle = [], [], [], [], []
        status_counts = Counter()
        for row in rows:
            if row['kind'] != 'observation':
                continue
            if row.get('position_m') is None:
                if row['status'] in ('orientation_ambiguous', 'orientation_measurement_rejected', 'target_not_found'):
                    if measurement_problem(replay.reclassify(qualities[stamp_ns(row['stamp'])], .10)) is not None:
                        gate.confirmed = 0
                else:
                    gate.reset()
                continue
            ns = stamp_ns(row['stamp'])
            result = gate.observe(ns, rotation_from_quaternion(row['quaternion_xyzw']),
                                  measurement_problem(replay.reclassify(row['quality'], .10)))
            status_counts[result['status']] += 1
            if not result['valid_orientation']:
                continue
            target = transform(row['position_m'], row['quaternion_xyzw'])
            if preview.offset is None:
                preview.teach(ns, target, actual)
            else:
                preview.observe(ns, target)
            raw_goal, goal = target @ preview.offset, preview.goal()
            times.append(row['audit_receive_s'])
            raw_position.append(np.linalg.norm(raw_goal[:3, 3]-actual[:3, 3])*1000)
            goal_position.append(np.linalg.norm(goal[:3, 3]-actual[:3, 3])*1000)
            raw_angle.append(rotation_distance_deg(raw_goal[:3, :3], actual[:3, :3]))
            goal_angle.append(rotation_distance_deg(goal[:3, :3], actual[:3, :3]))
        baseline = json.loads((INPUT/'demo_threshold_analysis/comparison.json').read_text())['cases'][case][2]
        assert dict(status_counts) == baseline['status_counts']
        results[case] = dict(accepted=len(times), raw_tcp_position_from_recorded_mm=metrics(raw_position),
                            preview_tcp_position_from_recorded_mm=metrics(goal_position),
                            raw_tcp_rotation_from_recorded_deg=metrics(raw_angle),
                            preview_tcp_rotation_from_recorded_deg=metrics(goal_angle),
                            offset_length_mm=float(np.linalg.norm(preview.offset[:3, 3])*1000))
        relative_times = np.array(times)-times[0]
        for axis, raw, filtered, ylabel in ((axes[0, column], raw_position, goal_position, 'TCP displacement (mm)'),
                                             (axes[1, column], raw_angle, goal_angle, 'TCP rotation (deg)')):
            gaps = np.flatnonzero(np.diff(relative_times) > .35)+1
            plot_times = np.insert(relative_times, gaps, np.nan)
            axis.plot(plot_times, np.insert(raw, gaps, np.nan), label='Unfiltered relative goal', lw=.8, alpha=.7)
            axis.plot(plot_times, np.insert(filtered, gaps, np.nan), label='Demo filtered + deadband goal', lw=1.3)
            axis.set_title(case)
            axis.set_xlabel('Seconds from record point')
            axis.set_ylabel(ylabel)
            axis.grid(alpha=.25); axis.legend(fontsize=8)
    figure.suptitle('Offline Stage 7 on stationary-target measurements: predicted TCP changes\n'
                   'Only accepted observations; fixed first flange approximates actual TCP. No robot execution.')
    figure.savefig(OUTPUT/'comparison.png', dpi=150)
    figure.savefig(OUTPUT/'comparison.svg')
    plt.close(figure)
    report = dict(cases=results, configuration=dict(quality_margin_px=.10, confirmation_frames=2,
                  position_tau_s=.15, rotation_tau_s=.4, deadband_enter_deg=3., deadband_exit_deg=2., rotation_mapping_gain=1.),
                  hardware_commands_sent=0,
                  note='Predicted movement from the recorded TCP under stationary-target noise, not actual motion or '
                       'absolute accuracy. Replays accepted measurements only, not ROS timing/holding/reference velocities. '
                       'Initial flange approximates stationary robot feedback; range/servo execution is not implemented here.')
    (OUTPUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
