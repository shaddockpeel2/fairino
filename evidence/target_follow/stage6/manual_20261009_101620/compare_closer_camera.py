"""Compare recorded views offline; never connect to ROS, the camera or SDK."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from fr3c_target_follow.follow_geometry import PoseHistory, rotation_from_quaternion, transform
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.orientation_quality import stamp_ns


ROOT = Path(__file__).resolve().parent
OUT = ROOT/'closer_camera_analysis'


def stats(values):
    values = np.asarray(values, float)
    return dict(count=int(values.size), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(values.max()))


def mean_rotation(rows):
    matrices = [rotation_from_quaternion(row['quaternion_xyzw']) for row in rows]
    u, _, vh = np.linalg.svd(np.mean(matrices, axis=0))
    return u @ np.diag([1., 1., np.linalg.det(u @ vh)]) @ vh


def centre(rows):
    return np.median([row['position_m'] for row in rows], axis=0)


def load(case):
    folder = ROOT/case
    rows = [json.loads(line) for line in (folder/'samples.jsonl').read_text().splitlines()]
    pairs = [row for row in rows if row['kind'] == 'observation' and row.get('position_m') is not None]
    report = json.loads((folder/'report.json').read_text())
    return rows, pairs, report


def describe(rows, pairs, report):
    average = mean_rotation(pairs)
    rpy = np.array([row['rpy_deg'] for row in pairs])
    rpy = np.degrees(np.unwrap(np.radians(rpy), axis=0))
    quality = [row for row in rows if row['kind'] == 'quality' and row['status'] == 'valid']
    logs = '\n'.join(path.read_text() for path in (ROOT/report['case']/'ros_logs').glob('*/launch.log'))
    return dict(
        paired=len(pairs), trusted=report['trusted_orientation_observations'],
        trusted_fraction=report['orientation_available_fraction'],
        paired_status_counts=report['paired_status_counts'],
        quality_status_counts=dict(Counter(row['status'] for row in rows if row['kind'] == 'quality')),
        all_rotation_deviation_from_own_mean_deg=stats([
            rotation_distance_deg(average, rotation_from_quaternion(row['quaternion_xyzw'])) for row in pairs]),
        position_deviation_from_median_mm=report['all_base']['position_deviation_from_median_mm'],
        rpy_axis_span_deg=np.ptp(rpy, axis=0).tolist(),
        min_edge_px=report['min_edge_px'], candidate_margin_px=report['candidate_margin_px'],
        candidate_rotation_distance_deg=report['candidate_rotation_distance_deg'],
        reprojection_error_px=report['reprojection_error_px'],
        inferred_range_mm=stats([row['distance_m']*1000 for row in quality]),
        inferred_normal_tilt_deg=stats([np.degrees(np.arccos(np.clip(
            -rotation_from_quaternion(row['quaternion_xyzw'])[2, 2], -1., 1.))) for row in quality]),
        image_age_ms=stats([row['image_age_ms'] for row in quality]),
        selected_candidate_counts=dict(Counter(row['selected_candidate_index'] for row in quality)),
        robot_flange=report['robot_flange'],
        all_owned_processes_clean_exit='process has died' not in logs and 'Traceback' not in logs
            and logs.count('process has finished cleanly') == 7)


def applied_extrinsic(rows, pairs):
    """Recover the transform used by the published coordinate chain, not a calibration."""
    history = PoseHistory(duration_s=1000.)
    for row in rows:
        if row['kind'] == 'flange':
            history.add(stamp_ns(row['stamp'])/1e9, row['position_m'], row['quaternion_xyzw'])
    recovered = []
    for row in pairs:
        flange, _ = history.at(stamp_ns(row['stamp'])/1e9)
        camera_tag = transform(row['quality']['position_m'], row['quality']['quaternion_xyzw'])
        base_tag = transform(row['position_m'], row['quaternion_xyzw'])
        recovered.append(np.linalg.inv(flange) @ base_tag @ np.linalg.inv(camera_tag))
    reference = recovered[0]
    return dict(sample_count=len(recovered), flange_to_optical_matrix=reference.tolist(),
                translation_spread_max_mm=max(np.linalg.norm(value[:3, 3]-reference[:3, 3])*1000
                                              for value in recovered),
                rotation_spread_max_deg=max(rotation_distance_deg(value[:3, :3], reference[:3, :3])
                                            for value in recovered))


def difference(before, after):
    delta = (centre(after)-centre(before))*1000
    return dict(before_count=len(before), after_count=len(after),
                before_median_mm=(centre(before)*1000).tolist(),
                after_median_mm=(centre(after)*1000).tolist(),
                delta_mm=delta.tolist(), translation_norm_mm=float(np.linalg.norm(delta)),
                mean_rotation_change_deg=rotation_distance_deg(mean_rotation(before), mean_rotation(after)))


def main():
    OUT.mkdir(exist_ok=True)
    sessions = {case: load(case) for case in ('static', 'target_rotation', 'static_more_pixels')}
    results = {}
    hashes = {}
    for case, (rows, pairs, report) in sessions.items():
        results[case] = describe(rows, pairs, dict(report, case=case))
        for filename in ('samples.jsonl', 'report.json', 'session.json'):
            path = ROOT/case/filename
            hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()

    old_rows, old_pairs, _ = sessions['target_rotation']
    new_rows, new_pairs, _ = sessions['static_more_pixels']
    end = max(row['audit_receive_s'] for row in old_rows)
    before = [row for row in old_pairs if row['audit_receive_s'] >= end-3.]
    old_flange = [row for row in old_rows if row['kind'] == 'flange' and row['audit_receive_s'] >= end-3.]
    new_flange = [row for row in new_rows if row['kind'] == 'flange']
    comparison = dict(
        user_confirmed_target_unchanged=True, user_confirmed_camera_mount_unchanged=True,
        before_window='final 3 seconds relative to final recorded row of target_rotation',
        before_status_counts=dict(Counter(row['status'] for row in before)),
        all_paired=difference(before, new_pairs),
        trusted_only=difference([row for row in before if row['valid_orientation']],
                                [row for row in new_pairs if row['valid_orientation']]),
        robot_flange=difference(old_flange, new_flange),
        applied_extrinsics={case: applied_extrinsic(rows, pairs) for case, (rows, pairs, _) in sessions.items()
                            if case in ('target_rotation', 'static_more_pixels')},
        note='Two separate sessions; target stillness during the final 3-second window is not independently confirmed. '
             'Raw and trusted orientation means do not establish absolute accuracy or uniquely identify the cause. '
             'A constant recovered extrinsic verifies algebraic application, not physical calibration validity.')
    results.update(cross_view=comparison, source_sha256=hashes,
                   hardware_motion_commands_sent=0, absolute_orientation_ground_truth_available=False)
    (OUT/'comparison.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for case, colour, label in (('static', '#377eb8', 'Original static'),
                                ('static_more_pixels', '#d95f02', 'Closer static')):
        rows, pairs, _ = sessions[case]
        t0 = min(row['audit_receive_s'] for row in rows)
        times = [row['audit_receive_s']-t0 for row in pairs]
        mean = mean_rotation(pairs)
        axes[0, 0].plot(times, [row['quality']['min_edge_px'] for row in pairs], color=colour, label=label)
        axes[0, 1].plot(times, [rotation_distance_deg(mean, rotation_from_quaternion(row['quaternion_xyzw']))
                               for row in pairs], color=colour, label=label, lw=.8)
        axes[1, 0].plot(times, [row['quality']['candidate_margin_px'] for row in pairs], color=colour, label=label, lw=.8)
    axes[1, 0].axhline(.15, color='#555555', ls='--', label='Ambiguity threshold')
    for selection, colour, label in ((before, '#377eb8', 'Rotation session: final 3 s'),
                                      (new_pairs, '#d95f02', 'Closer static: separate session')):
        t0 = selection[0]['audit_receive_s']
        axes[1, 1].plot([row['audit_receive_s']-t0 for row in selection],
                        [row['position_m'][0]*1000 for row in selection], color=colour, label=label, lw=.8)
    for axis, title, ylabel in zip(axes.flat,
            ('Tag image size', 'Stationary orientation scatter', 'Candidate separation', 'Fixed target: base X discrepancy'),
            ('Shortest edge (px)', 'Angle from own mean (deg)', 'Reprojection error gap (px)', 'Base X (mm)')):
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.set_xlabel('Seconds within each plotted recording window')
        axis.grid(alpha=.25)
        axis.legend(fontsize=8)
    figure.suptitle('Stage 6 read-only comparison: larger tag image and higher availability; tilt noise remains\n'
                   'Different views and separate sessions; not an absolute-accuracy measurement')
    figure.savefig(OUT/'comparison.png', dpi=150)
    figure.savefig(OUT/'comparison.svg')
    plt.close(figure)
    print(json.dumps(dict(
        closer=results['static_more_pixels'], cross_view_all=comparison['all_paired'],
        cross_view_trusted=comparison['trusted_only'], robot_motion=comparison['robot_flange']), indent=2))


if __name__ == '__main__':
    main()
