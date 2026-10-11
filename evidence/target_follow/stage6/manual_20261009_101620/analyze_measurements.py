"""Offline analysis of the user's two sessions. Reads data; sends no ROS/SDK calls."""
from collections import Counter
import csv
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from fr3c_target_follow.follow_geometry import rotation_from_quaternion, transform, transform_rpy
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.orientation_quality import OrientationGate, measurement_problem, stamp_ns

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'analysis'
OUT.mkdir(exist_ok=True)


def statistics(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return None
    return dict(count=int(a.size), min=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


def runs(rows, predicate):
    lengths, spans, current = [], [], []
    for row in rows:
        if predicate(row):
            current.append(row)
        elif current:
            lengths.append(len(current))
            spans.append(current[-1]['audit_receive_s']-current[0]['audit_receive_s'])
            current = []
    if current:
        lengths.append(len(current))
        spans.append(current[-1]['audit_receive_s']-current[0]['audit_receive_s'])
    return dict(frame_counts=statistics(lengths), span_s=statistics(spans),
                span_note='first-to-last sample in each run; single frame has zero span')


def broken_line(times, values):
    """Do not draw an interpolated measurement across missing image intervals."""
    points_t, points_v = [], []
    for i, (t, v) in enumerate(zip(times, values)):
        if i and t-times[i-1] > .3:
            points_t.append(np.nan)
            points_v.append(np.nan)
        points_t.append(t)
        points_v.append(v)
    return points_t, points_v


fig, axes = plt.subplots(4, 2, figsize=(14, 12), constrained_layout=True)
results = {}
for column, case in enumerate(('static', 'target_rotation')):
    folder = ROOT/case
    report = json.loads((folder/'report.json').read_text())
    rows = [json.loads(line) for line in (folder/'samples.jsonl').read_text().splitlines()]
    pairs = [r for r in rows if r['kind'] == 'observation' and r.get('position_m') is not None]
    observations = [r for r in rows if r['kind'] == 'observation']
    qualities = [r for r in rows if r['kind'] == 'quality']
    valid_quality = [r for r in qualities if r['status'] == 'valid']
    states = [r for r in rows if r['kind'] == 'state']
    t0 = min(r['audit_receive_s'] for r in rows)
    t = np.array([r['audit_receive_s']-t0 for r in pairs])
    rpy = np.array([r['rpy_deg'] for r in pairs])
    rpy[:, 2] = np.degrees(np.unwrap(np.radians(rpy[:, 2])))
    rotations = [rotation_from_quaternion(r['quaternion_xyzw']) for r in pairs]
    u, _, vh = np.linalg.svd(np.mean(rotations, axis=0))
    mean_rotation = u @ np.diag([1., 1., np.linalg.det(u @ vh)]) @ vh
    deviations = [rotation_distance_deg(mean_rotation, r) for r in rotations]
    view_tilts = [np.degrees(np.arccos(np.clip(-rotation_from_quaternion(
        r['quality']['quaternion_xyzw'])[2, 2], -1., 1.))) for r in pairs]
    time_by_state = Counter()
    for i, row in enumerate(states[:-1]):
        time_by_state[row['status']] += min(.15, states[i+1]['audit_receive_s']-row['audit_receive_s'])
    duration_sum = sum(time_by_state.values())
    bins = []
    for start in range(0, int(np.ceil(report['duration_s'])), 5):
        selection = [r for r in pairs if start <= r['audit_receive_s']-t0 < start+5]
        selected_quality = [r for r in qualities if start <= r['audit_receive_s']-t0 < start+5]
        counts = Counter(r['status'] for r in selection)
        bins.append(dict(start_s=start, paired=len(selection), valid=counts['valid'],
                         ambiguous=counts['orientation_ambiguous'], acquiring=counts['acquiring_orientation'],
                         jump=counts['orientation_jump'], rejected=counts['orientation_measurement_rejected'],
                         missing=sum(r['status'] == 'target_not_found' for r in selected_quality)))
    # Parameter sensitivity only: replay each recorded pair, retaining measured
    # poses and quality. It does not recreate ROS pairing, and is not accuracy.
    sensitivity = {}
    for confirmation in (1, 3, 5):
        gate = OrientationGate(confirmation_frames=confirmation)
        counter = Counter()
        for row in observations:
            if row.get('position_m') is None:
                if row['status'] in ('orientation_ambiguous', 'orientation_measurement_rejected', 'target_not_found'):
                    gate.confirmed = 0
                else:
                    gate.reset()
                continue
            result = gate.observe(stamp_ns(row['stamp']), rotation_from_quaternion(row['quaternion_xyzw']),
                                  measurement_problem(row['quality']))
            counter[result['status']] += 1
        if confirmation == 5:
            assert dict(counter) == report['paired_status_counts'], (case, counter)
        sensitivity[str(confirmation)] = dict(status_counts=dict(counter),
                                              available_fraction=counter['valid']/len(pairs))
    logs = list((folder/'ros_logs').glob('*/launch.log'))
    log = '\n'.join(p.read_text() for p in logs)
    results[case] = dict(
        scenario=case, duration_s=report['duration_s'], paired=len(pairs),
        trusted=report['trusted_orientation_observations'],
        paired_status_counts=report['paired_status_counts'],
        trusted_fraction=report['orientation_available_fraction'],
        valid_quality_measurement_fraction=sum(r['orientation_measurement_valid'] for r in valid_quality)/len(valid_quality),
        rpy_axis_span_deg=np.ptp(rpy, axis=0).tolist(),
        rpy_axis_std_deg=np.std(rpy, axis=0).tolist(),
        rotation_deviation_from_mean_deg=statistics(deviations),
        measured_camera_normal_tilt_deg=statistics(view_tilts),
        image_age_ms=statistics([r['image_age_ms'] for r in valid_quality]),
        selected_candidate_counts=dict(Counter(r['selected_candidate_index'] for r in valid_quality)),
        valid_orientation_runs=runs(pairs, lambda r: r['valid_orientation']),
        good_measurement_runs=runs(pairs, lambda r: r['orientation_measurement_valid']),
        monitor_status_time_s=dict(time_by_state),
        monitor_valid_time_fraction=time_by_state['valid']/duration_sum,
        confirmation_sensitivity=sensitivity,
        sensitivity_note='Offline replay reproduces baseline paired status counts; only confirmation length changed. Higher availability is not improved accuracy.',
        quality_status_counts=dict(Counter(r['status'] for r in qualities)),
        bins_5s=bins,
        all_owned_processes_clean_exit='process has died' not in log and 'Traceback' not in log
            and log.count('process has finished cleanly') == 7,
        hardware_motion_commands_sent=0,
        ground_truth_orientation_available=False,
    )
    if case == 'static':
        # Hypothetical unfiltered full-relative-pose input, to quantify noise
        # coupling. This is an offline calculation, not a stage 7 controller.
        workspace = ROOT.parents[3]
        tcp_file = workspace/'src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml'
        tcp_cfg = yaml.safe_load(tcp_file.read_text())['ydi_j22_tcp']
        robot = next(r for r in rows if r['kind'] == 'flange')
        tcp = transform(robot['position_m'], robot['quaternion_xyzw']) @ transform_rpy(
            tcp_cfg['xyz_m'], tcp_cfg['rpy_rad'])
        initial = next(r for r in pairs if r['valid_orientation'])
        offset = np.linalg.inv(transform(initial['position_m'], initial['quaternion_xyzw'])) @ tcp
        goals = np.array([(transform(r['position_m'], r['quaternion_xyzw']) @ offset)[:3, 3] for r in pairs])
        rotated_offsets = np.array([rotation @ offset[:3, 3] for rotation in rotations])
        results[case]['hypothetical_unfiltered_relative_pose'] = dict(
            offset_length_mm=float(np.linalg.norm(offset[:3, 3])*1000),
            initial_stamp=initial['stamp'], tcp_file=str(tcp_file),
            goal_axis_span_mm=(np.ptp(goals, axis=0)*1000).tolist(),
            goal_deviation_from_coordinate_median_mm=statistics(
                np.linalg.norm(goals-np.median(goals, axis=0), axis=1)*1000),
            orientation_only_goal_deviation_from_coordinate_median_mm=statistics(
                np.linalg.norm(rotated_offsets-np.median(rotated_offsets, axis=0), axis=1)*1000),
            note='All recorded poses, including rejected frames, projected without filter/limits. Robot was stationary; first recorded flange approximates fixed TCP. No actual control output or stage 7 implementation.')
    with (OUT/(case+'_bins.csv')).open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(bins[0]))
        writer.writeheader()
        writer.writerows(bins)
    for axis, label, colour in zip(range(3), ('Roll', 'Pitch', 'Yaw'), ('#d95f02', '#7570b3', '#1b9e77')):
        axes[0, column].plot(*broken_line(t, rpy[:, axis]), label=label, color=colour, lw=1)
    axes[0, column].set_title(f'{case}: {report["trusted_orientation_observations"]}/{len(pairs)} usable ({report["orientation_available_fraction"]:.1%})')
    axes[0, column].set_ylabel('Measured base RPY (deg)')
    axes[0, column].legend(loc='best', ncol=3)
    tq = [r['audit_receive_s']-t0 for r in valid_quality]
    margin = [r['candidate_margin_px'] for r in valid_quality]
    axes[1, column].plot(*broken_line(tq, margin), color='#377eb8', lw=.8)
    axes[1, column].axhline(.15, color='#e41a1c', ls='--', label='ambiguity margin 0.15 px')
    axes[1, column].set_ylabel('Two-candidate error gap (px)')
    axes[1, column].legend(loc='upper right')
    statuses = ['valid', 'acquiring_orientation', 'orientation_ambiguous', 'orientation_jump',
                'orientation_measurement_rejected', 'target_not_found', 'no_fresh_image', 'waiting_for_paired_pose']
    colours = ['#4daf4a', '#ffbf00', '#e41a1c', '#984ea3', '#a65628', '#555555', '#999999', '#cccccc']
    for index, (status, colour) in enumerate(zip(statuses, colours)):
        ts = [r['audit_receive_s']-t0 for r in states if r['status'] == status]
        if ts:
            axes[2, column].scatter(ts, np.full(len(ts), index), s=5, color=colour)
    axes[2, column].set_yticks(range(len(statuses)))
    axes[2, column].set_yticklabels(['valid', 'confirming', 'ambiguous', 'jump', 'branch rejected', 'missing tag', 'no image', 'waiting'])
    axes[2, column].invert_yaxis()
    positions = np.array([r['position_m'] for r in pairs])
    centred = (positions-np.median(positions, axis=0))*1000
    for axis, label in enumerate(('X', 'Y', 'Z')):
        axes[3, column].plot(*broken_line(t, centred[:, axis]), label=label, lw=1)
    axes[3, column].set_ylabel('Measured position from median (mm)')
    axes[3, column].set_xlabel('Seconds from first recorded row (startup included)')
    axes[3, column].legend(loc='best', ncol=3)
    for row in range(4):
        axes[row, column].grid(alpha=.25)
        axes[row, column].set_xlim(0, report['duration_s'])
fig.suptitle('Read-only stage 6: user confirmed static target; rotation included large real motions\nRaw measurements and gate status; no absolute orientation ground truth', fontsize=12)
fig.savefig(OUT/'orientation_measurements.png', dpi=150)
fig.savefig(OUT/'orientation_measurements.svg')
plt.close(fig)
(OUT/'analysis.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
print(json.dumps({case: {key: result[key] for key in ('trusted_fraction', 'rpy_axis_span_deg',
    'rotation_deviation_from_mean_deg', 'valid_orientation_runs', 'confirmation_sensitivity',
    'all_owned_processes_clean_exit')} for case, result in results.items()}, indent=2))
