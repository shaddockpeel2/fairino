"""Offline analysis of the user's stage 7 preview. No ROS/SDK calls."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from fr3c_target_follow.follow_geometry import rotation_from_quaternion, transform
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.pose_preview import rotation_exp, rotation_vector


ROOT = Path(__file__).resolve().parent
OUT = ROOT/'analysis'


def stats(values):
    values = np.asarray(values, float)
    return dict(count=int(values.size), median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                max=float(values.max()))


def matrix(row, name):
    return transform(row[name+'_m'], row[name+'_quaternion_xyzw'])


def scatter(rows, name):
    poses = [matrix(row, name) for row in rows]
    centre = np.median([pose[:3, 3] for pose in poses], axis=0)
    u, _, vh = np.linalg.svd(np.mean([pose[:3, :3] for pose in poses], axis=0))
    mean = u @ np.diag([1., 1., np.linalg.det(u @ vh)]) @ vh
    return dict(position_deviation_from_median_mm=stats([np.linalg.norm(pose[:3, 3]-centre)*1000 for pose in poses]),
                position_axis_span_mm=(np.ptp([pose[:3, 3] for pose in poses], axis=0)*1000).tolist(),
                rotation_deviation_from_mean_deg=stats([rotation_distance_deg(mean, pose[:3, :3]) for pose in poses]))


def main():
    OUT.mkdir(exist_ok=True)
    rows = [json.loads(line) for line in (ROOT/'samples.jsonl').read_text().splitlines()]
    report = json.loads((ROOT/'report.json').read_text())
    states = [row for row in rows if row['kind'] == 'preview']
    pairs = [row for row in rows if row['kind'] == 'observation' and row.get('position_m') is not None]
    active = [row for row in states if row['status'] in ('preview', 'holding_quality')]
    recorded = [row for row in states if row['offset_recorded']]
    first = active[0]
    t0 = rows[0]['audit_receive_s']
    offset = np.asarray(first['offset_matrix'])
    initial_tcp = matrix(first, 'actual_tcp')
    initial_tag = initial_tcp @ np.linalg.inv(offset)
    times = np.array([row['audit_receive_s']-t0 for row in active])
    gaps = np.flatnonzero(np.diff(times) > .1)+1
    plot_times = np.insert(times, gaps, np.nan)
    full_goals = [matrix(row, 'desired_tcp') for row in active]
    half_goals = []
    errors = []
    for row, goal in zip(active, full_goals):
        mapped = matrix(row, 'mapped_target')
        errors.append(np.linalg.norm((mapped @ offset)[:3, 3]-goal[:3, 3])*1000)
        mapped[:3, :3] = rotation_exp(.5*rotation_vector(mapped[:3, :3] @ initial_tag[:3, :3].T)) @ initial_tag[:3, :3]
        half_goals.append(mapped @ offset)
    full_displacements = [np.linalg.norm(goal[:3, 3]-initial_tcp[:3, 3])*1000 for goal in full_goals]
    half_displacements = [np.linalg.norm(goal[:3, 3]-initial_tcp[:3, 3])*1000 for goal in half_goals]
    low_motion = [row for row in active if 3 <= row['audit_receive_s']-t0 < 30]
    robot = [row for row in states if 'actual_tcp_m' in row]
    feedback = [row for row in rows if row['kind'] == 'feedback']
    bins = []
    for start in range(0, 90, 5):
        sample = [row for row in states if start <= row['audit_receive_s']-t0 < start+5]
        valid = [row for row in sample if row['status'] in ('preview', 'holding_quality')]
        bins.append(dict(start_s=start, status_counts=dict(Counter(row['status'] for row in sample)),
                         max_tcp_angle_gap_deg=max((row['rotation_error_deg'] for row in valid), default=None)))
    last = active[-1]
    logs = '\n'.join(path.read_text() for path in (ROOT/'ros_logs').glob('*/launch.log'))
    analysis = dict(duration_s=report['duration_s'], paired=len(pairs), trusted=report['trusted_observations'],
                    paired_status_counts=dict(Counter(row['status'] for row in pairs)),
                    trusted_pair_fraction=report['trusted_observations']/len(pairs),
                    recorded_states=len(recorded), active_recorded_states=len(active),
                    active_after_record_fraction=len(active)/len(recorded),
                    after_record_status_counts=dict(Counter(row['status'] for row in recorded)),
                    startup_until_record_s=first['audit_receive_s']-t0,
                    stale_feedback_only_before_record=all(not row['offset_recorded'] for row in states
                                                         if row['status'] == 'stale_robot_feedback'),
                    low_motion_window=dict(start_s=3, end_s=30, selected_from_measured_trajectory=True,
                        raw=scatter(low_motion, 'raw_target'), filtered=scatter(low_motion, 'filtered_target'),
                        desired=scatter(low_motion, 'desired_tcp')),
                    actual_tcp=scatter(robot, 'actual_tcp'), offset_length_mm=float(np.linalg.norm(offset[:3, 3])*1000),
                    max_tcp_angle_gap_deg=report['max_rotation_error_deg'],
                    max_full_goal_displacement_from_record_mm=max(full_displacements),
                    hypothetical_half_mapping=dict(max_displacement_from_record_mm=max(half_displacements),
                        max_tcp_angle_gap_deg=max(rotation_distance_deg(goal[:3, :3], initial_tcp[:3, :3]) for goal in half_goals),
                        note='Offline remapping of accepted rotations; not a new live run or robot execution.'),
                    max_relative_transform_position_disagreement_mm=max(errors),
                    initial_goal_position_error_mm=float(np.linalg.norm(matrix(first, 'desired_tcp')[:3, 3]-initial_tcp[:3, 3])*1000),
                    user_says_roughly_returned_to_initial_pose=True,
                    end=dict(time_s=last['audit_receive_s']-t0,
                        raw_angle_from_record_deg=rotation_distance_deg(matrix(last, 'raw_target')[:3, :3], initial_tag[:3, :3]),
                        filtered_angle_from_record_deg=rotation_distance_deg(matrix(last, 'filtered_target')[:3, :3], initial_tag[:3, :3]),
                        tcp_angle_gap_deg=last['rotation_error_deg'],
                        last_3s_tcp_angle_gap_range_deg=[min(row['rotation_error_deg'] for row in active
                            if row['audit_receive_s'] >= last['audit_receive_s']-3),
                            max(row['rotation_error_deg'] for row in active if row['audit_receive_s'] >= last['audit_receive_s']-3)]),
                    feedback_status_counts=dict(Counter(row['status'] for row in feedback)),
                    controller_coordinate_contexts=sorted(set((row.get('tool_id'), row.get('work_object_id')) for row in feedback)),
                    all_recorded_tool_workobject_transforms_zero=all(
                        row['tool_coord_mm_deg'] == [0.]*6 and row['work_object_coord_mm_deg'] == [0.]*6 for row in feedback),
                    owned_processes_clean_exit=logs.count('process has finished cleanly'),
                    process_error='process has died' in logs or 'Traceback' in logs,
                    bins_5s=bins, hardware_commands_sent=0,
                    source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                   for name in ('samples.jsonl', 'report.json', 'session.json')},
                    note='TCP angle gap is desired versus actual robot orientation, not camera accuracy. '
                         'Robot remained stationary; low-motion interval is inferred, not externally labelled. '
                         'Approximate return and unsettled endpoint do not establish return accuracy or physical axis correctness.')
    (OUT/'analysis.json').write_text(json.dumps(analysis, ensure_ascii=False, indent=2)+'\n')
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), constrained_layout=True)
    for name, label in (('raw_target', 'Raw target'), ('filtered_target', 'Filtered target'),
                        ('mapped_target', 'Target after deadband')):
        values = [rotation_distance_deg(matrix(row, name)[:3, :3], initial_tag[:3, :3]) for row in active]
        axes[0].plot(plot_times, np.insert(values, gaps, np.nan), label=label, lw=.9)
    axes[0].set_ylabel('Angle from record (deg)'); axes[0].legend(ncol=3)
    axes[0].axvspan(3, 30, color='#e5f5e0', alpha=.5)
    axes[1].plot(plot_times, np.insert(full_displacements, gaps, np.nan), label='Recorded mapping 1.0')
    axes[1].plot(plot_times, np.insert(half_displacements, gaps, np.nan), label='Offline mapping 0.5', ls='--')
    axes[1].set_ylabel('TCP goal displacement (mm)'); axes[1].legend()
    names = list(dict.fromkeys(row['status'] for row in states))
    for i, name in enumerate(names):
        sample = [row['audit_receive_s']-t0 for row in states if row['status'] == name]
        axes[2].scatter(sample, np.full(len(sample), i), s=3)
    axes[2].set_yticks(range(len(names)), names); axes[2].invert_yaxis()
    axes[2].set_xlabel('Seconds from first recorded row (startup included)')
    for axis in axes:
        axis.set_xlim(0, 90); axis.grid(alpha=.25)
    figure.suptitle('User stage 7 read-only preview: initial jitter suppressed; deliberate motion changes TCP goal\n'
                   'Actual robot stationary. Angular gap is not camera accuracy; half mapping is an offline comparison.')
    figure.savefig(OUT/'preview_analysis.png', dpi=150)
    figure.savefig(OUT/'preview_analysis.svg')
    plt.close(figure)
    print(json.dumps({key: analysis[key] for key in ('active_after_record_fraction', 'low_motion_window',
          'max_full_goal_displacement_from_record_mm', 'hypothetical_half_mapping', 'end',
          'owned_processes_clean_exit', 'process_error')}, indent=2))


if __name__ == '__main__':
    main()
