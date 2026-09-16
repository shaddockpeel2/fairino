"""Offline validation and solving for a captured hand-eye session."""

import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np
import yaml

from .geometry import (
    handeye_residuals,
    matrix_to_rpy,
    rotation_distance_deg,
    transform_spread,
    transform_matrix,
)


METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _split_positions(sample_indices, training_samples, minimum_validation_samples,
                     random_validation_samples=None, random_seed=0):
    """Return training/validation positions without changing the captured dataset."""
    sample_indices = list(sample_indices)
    if random_validation_samples is None:
        if len(sample_indices) < training_samples + minimum_validation_samples:
            raise ValueError(
                f'need {training_samples} training and {minimum_validation_samples} '
                f'validation samples; got {len(sample_indices)} total')
        return (
            list(range(training_samples)),
            list(range(training_samples, len(sample_indices))),
            {'type': 'ordered_tail'},
        )

    if random_validation_samples < minimum_validation_samples:
        raise ValueError(
            f'random validation count {random_validation_samples} is smaller than '
            f'minimum {minimum_validation_samples}')
    expected_total = training_samples + random_validation_samples
    if len(sample_indices) != expected_total:
        raise ValueError(
            f'random split requires exactly {training_samples} training and '
            f'{random_validation_samples} validation samples; got '
            f'{len(sample_indices)} total')
    selected_indices = sorted(random.Random(random_seed).sample(
        sample_indices, random_validation_samples))
    selected_set = set(selected_indices)
    training_positions = [
        position for position, index in enumerate(sample_indices)
        if index not in selected_set
    ]
    validation_positions = [
        position for position, index in enumerate(sample_indices)
        if index in selected_set
    ]
    return training_positions, validation_positions, {
        'type': 'random_holdout',
        'seed': random_seed,
        'requested_validation_samples': random_validation_samples,
    }


def _percentiles(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
    }


def _rotation_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _motion_invariant_report(base, camera):
    rows = []
    for first in range(len(base)):
        for second in range(first + 1, len(base)):
            wrist_angle = _rotation_angle(
                base[first][:3, :3].T @ base[second][:3, :3])
            camera_angle = _rotation_angle(
                camera[first][:3, :3] @ camera[second][:3, :3].T)
            rows.append({
                'first_sample': first + 1,
                'second_sample': second + 1,
                'wrist_rotation_deg': wrist_angle,
                'camera_rotation_deg': camera_angle,
                'absolute_mismatch_deg': abs(wrist_angle - camera_angle),
            })
    mismatches = [row['absolute_mismatch_deg'] for row in rows]
    worst = max(rows, key=lambda row: row['absolute_mismatch_deg'])
    return {
        'pair_count': len(rows),
        'absolute_rotation_angle_mismatch_deg': _percentiles(mismatches),
        'worst_pair': worst,
        'warning': bool(np.percentile(mismatches, 95) > 1.0),
        'explanation': (
            'For a rigid eye-in-hand setup, corresponding wrist and camera relative '
            'rotations must have the same angle, independent of the unknown hand-eye transform.'
        ),
    }


def _quality_report(samples, session_dir):
    fields = [
        'corners_min',
        'markers_min',
        'reprojection_rms_px_median',
        'reprojection_rms_px_max',
        'wrist_translation_spread_max_mm',
        'wrist_rotation_spread_max_deg',
        'board_translation_spread_max_mm',
        'board_rotation_spread_max_deg',
    ]
    summary = {}
    for field in fields:
        values = np.asarray([sample['quality'][field] for sample in samples], dtype=float)
        summary[field] = {
            'min': float(np.min(values)),
            'median': float(np.median(values)),
            'max': float(np.max(values)),
        }
    missing_images = []
    for sample in samples:
        for kind, relative in sample.get('images', {}).items():
            if not (session_dir / relative).is_file():
                missing_images.append({'sample': sample['index'], 'kind': kind, 'path': relative})
    return {'summary': summary, 'missing_images': missing_images}


def _frame_equivalence_report(path, expected_base_frame):
    """Validate and average ROS-end <- controller-flange observations."""
    path = Path(path).resolve()
    document = json.loads(path.read_text())
    samples = document.get('samples', [])
    if len(samples) < 3:
        raise ValueError(f'frame cross-check needs at least 3 samples; got {len(samples)}')
    ros_frames = {sample.get('wrist_frame') for sample in samples}
    base_frames = {sample.get('base_frame') for sample in samples}
    if len(ros_frames) != 1 or None in ros_frames:
        raise ValueError(f'frame cross-check mixes ROS end frames: {sorted(ros_frames)}')
    if base_frames != {expected_base_frame}:
        raise ValueError(
            f'frame cross-check base frame mismatch: {sorted(base_frames)} != '
            f'{expected_base_frame}')
    transforms = [
        np.asarray(sample['wrist_from_flange_snapshot'], dtype=float)
        for sample in samples
    ]
    centre, translation_mm, rotation_deg = transform_spread(transforms)
    joint_difference_deg = max(
        float(sample['joint_max_absolute_difference_deg']) for sample in samples)
    gates = {
        'sample_count': len(samples) >= 3,
        'translation_spread': float(np.max(translation_mm)) <= 0.1,
        'rotation_spread': float(np.max(rotation_deg)) <= 0.01,
        'joint_agreement': joint_difference_deg <= 0.01,
    }
    return {
        'accepted': all(gates.values()),
        'path': str(path),
        'sample_count': len(samples),
        'base_frame': expected_base_frame,
        'ros_frame': next(iter(ros_frames)),
        'controller_frame': 'controller_flange',
        'ros_frame_from_controller_flange': centre.tolist(),
        'translation_spread_max_mm': float(np.max(translation_mm)),
        'rotation_spread_max_deg': float(np.max(rotation_deg)),
        'joint_max_absolute_difference_deg': joint_difference_deg,
        'limits': {
            'translation_spread_max_mm': 0.1,
            'rotation_spread_max_deg': 0.01,
            'joint_max_absolute_difference_deg': 0.01,
        },
        'gates': gates,
    }


def _solve_method(base_training, camera_training, base_validation, camera_validation,
                  validation_sample_indices, name, method):
    rotation, translation = cv2.calibrateHandEye(
        [matrix[:3, :3] for matrix in base_training],
        [matrix[:3, 3].reshape(3, 1) for matrix in base_training],
        [matrix[:3, :3] for matrix in camera_training],
        [matrix[:3, 3].reshape(3, 1) for matrix in camera_training],
        method=method,
    )
    wrist_from_camera = transform_matrix(rotation, translation)
    reference, train_translation, train_rotation = handeye_residuals(
        base_training, wrist_from_camera, camera_training)
    _, validation_translation, validation_rotation = handeye_residuals(
        base_validation, wrist_from_camera, camera_validation, reference)
    return {
        'method': name,
        'wrist_from_camera': wrist_from_camera.tolist(),
        'xyz_m': wrist_from_camera[:3, 3].tolist(),
        'rpy_rad': matrix_to_rpy(wrist_from_camera[:3, :3]),
        'base_from_board_training_reference': reference.tolist(),
        'training_translation_mm': _percentiles(train_translation),
        'training_rotation_deg': _percentiles(train_rotation),
        'validation_translation_mm': _percentiles(validation_translation),
        'validation_rotation_deg': _percentiles(validation_rotation),
        'validation_per_sample': [
            {
                'sample': index,
                'translation_mm': float(translation_value),
                'rotation_deg': float(rotation_value),
            }
            for index, translation_value, rotation_value in zip(
                validation_sample_indices,
                validation_translation,
                validation_rotation,
            )
        ],
    }


def analyze(session_dir, training_samples=25, minimum_validation_samples=10,
            median_limit_mm=3.0, p95_limit_mm=8.0,
            minimum_orientation_span_deg=25.0, robot_poses_path=None,
            frame_crosscheck_path=None, random_validation_samples=None,
            random_seed=0):
    session_dir = Path(session_dir).resolve()
    samples_path = session_dir / 'samples.json'
    document = json.loads(samples_path.read_text())
    samples = document['samples']
    indices = [sample['index'] for sample in samples]
    if indices != list(range(1, len(samples) + 1)):
        raise ValueError(f'sample indices are not continuous: {indices}')
    training_positions, validation_positions, split_strategy = _split_positions(
        indices,
        training_samples,
        minimum_validation_samples,
        random_validation_samples=random_validation_samples,
        random_seed=random_seed,
    )
    training = [samples[position] for position in training_positions]
    validation = [samples[position] for position in validation_positions]
    training_indices = [sample['index'] for sample in training]
    validation_indices = [sample['index'] for sample in validation]
    pose_source = {
        'type': 'captured_ros_tf',
        'frame': document['frames']['wrist'],
        'path': str(samples_path),
    }
    if robot_poses_path:
        robot_poses_path = Path(robot_poses_path).resolve()
        robot_document = json.loads(robot_poses_path.read_text())
        robot_samples = robot_document['samples']
        robot_indices = [sample['index'] for sample in robot_samples]
        if robot_indices != indices:
            raise ValueError(
                f'robot-pose indices do not match captured samples: {robot_indices}')
        base_all = [
            np.asarray(sample['base_from_flange_controller'], dtype=float)
            for sample in robot_samples
        ]
        pose_source = {
            'type': 'controller_get_forward_kin_reconstruction',
            'frame': robot_document.get('pose_frame', 'controller_flange'),
            'path': str(robot_poses_path),
            'provenance': robot_document.get('provenance', {}),
        }
    else:
        base_all = [np.asarray(sample['base_from_wrist'], dtype=float) for sample in samples]
    camera_all = [np.asarray(sample['camera_from_board'], dtype=float) for sample in samples]
    base_training = [base_all[position] for position in training_positions]
    camera_training = [camera_all[position] for position in training_positions]
    base_validation = [base_all[position] for position in validation_positions]
    camera_validation = [camera_all[position] for position in validation_positions]

    methods = []
    failures = {}
    for name, method in METHODS.items():
        try:
            result = _solve_method(
                base_training, camera_training, base_validation, camera_validation,
                validation_indices, name, method)
            if np.isfinite(np.asarray(result['wrist_from_camera'])).all():
                methods.append(result)
        except (cv2.error, ValueError) as error:
            failures[name] = str(error)
    if not methods:
        raise RuntimeError(f'all OpenCV hand-eye methods failed: {failures}')
    methods.sort(key=lambda value: value['validation_translation_mm']['p95'])
    selected = methods[0]

    translations = np.asarray([matrix[:3, 3] for matrix in base_training])
    extent = (np.max(translations, axis=0) - np.min(translations, axis=0)) * 1000.0
    max_pair_rotation = max(
        rotation_distance_deg(first[:3, :3], second[:3, :3])
        for index, first in enumerate(base_training)
        for second in base_training[index + 1:]
    )
    quality = _quality_report(samples, session_dir)
    invariant = _motion_invariant_report(base_all, camera_all)
    gates = {
        'sample_count': len(training) >= training_samples
                        and len(validation) >= minimum_validation_samples,
        'images_complete': not quality['missing_images'],
        'orientation_span': max_pair_rotation >= minimum_orientation_span_deg,
        'validation_translation_median': (
            selected['validation_translation_mm']['median'] <= median_limit_mm),
        'validation_translation_p95': (
            selected['validation_translation_mm']['p95'] <= p95_limit_mm),
    }
    accepted = all(gates.values())
    frame_equivalence = None
    calibration_parent_frame = pose_source['frame']
    calibration_transform = np.asarray(selected['wrist_from_camera'], dtype=float)
    if frame_crosscheck_path:
        if not robot_poses_path:
            raise ValueError('--frame-crosscheck requires --robot-poses')
        frame_equivalence = _frame_equivalence_report(
            frame_crosscheck_path, document['frames']['base'])
        calibration_parent_frame = frame_equivalence['ros_frame']
        calibration_transform = (
            np.asarray(frame_equivalence['ros_frame_from_controller_flange'], dtype=float)
            @ calibration_transform
        )
        deployable = accepted and frame_equivalence['accepted']
    else:
        deployable = accepted and pose_source['frame'] == document['frames']['wrist']
    if accepted and not deployable:
        status = 'accepted_handeye_integration_blocked'
    elif accepted:
        status = 'accepted'
    else:
        status = 'rejected_validation'
    report = {
        'schema_version': 1,
        'session': str(session_dir),
        'accepted': accepted,
        'deployable_in_current_ros_tf': deployable,
        'status': status,
        'robot_pose_source': pose_source,
        'integration_frame_crosscheck': frame_equivalence,
        'training_samples': len(training),
        'validation_samples': len(validation),
        'sample_indices': indices,
        'training_sample_indices': training_indices,
        'validation_sample_indices': validation_indices,
        'split_strategy': split_strategy,
        'selected_method': selected['method'],
        'acceptance_limits': {
            'validation_translation_median_mm': median_limit_mm,
            'validation_translation_p95_mm': p95_limit_mm,
            'minimum_training_orientation_span_deg': minimum_orientation_span_deg,
        },
        'gates': gates,
        'training_wrist_translation_extent_xyz_mm': extent.tolist(),
        'training_max_pairwise_wrist_rotation_deg': max_pair_rotation,
        'capture_quality': quality,
        'motion_invariant': invariant,
        'method_failures': failures,
        'methods': methods,
    }
    report_name = ('solve_report_controller_fk.json' if robot_poses_path
                   else 'solve_report.json')
    (session_dir / report_name).write_text(json.dumps(report, indent=2) + '\n')

    frames = document['frames']
    calibration = {
        'calibrated': deployable,
        'handeye_accepted': accepted,
        'deployable_in_current_ros_tf': deployable,
        'status': status,
        'parent_frame': calibration_parent_frame,
        'child_frame': frames['camera'],
        'xyz': calibration_transform[:3, 3].tolist(),
        'rpy': matrix_to_rpy(calibration_transform[:3, :3]),
        'camera_serial': document['camera_serial'],
        'mount_version': document['mount_version'],
        'image_size': [document['camera_info']['width'], document['camera_info']['height']],
        'board': document['board'],
        'method': selected['method'],
        'session': str(session_dir),
        'robot_pose_source': pose_source,
        'integration_frame_crosscheck': frame_equivalence,
        'validation_translation_median_mm': selected['validation_translation_mm']['median'],
        'validation_translation_p95_mm': selected['validation_translation_mm']['p95'],
    }
    if accepted and deployable:
        output_name = ('flange_to_dabai_dcw2.yaml'
                       if calibration_parent_frame == 'flange_link'
                       else 'handeye_to_dabai_dcw2.yaml')
    elif accepted:
        output_name = 'controller_flange_to_dabai_dcw2.accepted_not_deployable.yaml'
    else:
        output_name = 'wrist3_to_dabai_dcw2.candidate.yaml'
    (session_dir / output_name).write_text(yaml.safe_dump(calibration, sort_keys=False))
    markdown_name = ('P13_handeye_analysis_controller_fk.md' if robot_poses_path
                     else 'P13_handeye_analysis.md')
    _write_markdown(session_dir / markdown_name, report)
    report['report_path'] = str(session_dir / report_name)
    report['calibration_path'] = str(session_dir / output_name)
    report['markdown_path'] = str(session_dir / markdown_name)
    return report


def _write_markdown(path, report):
    selected = next(
        item for item in report['methods'] if item['method'] == report['selected_method'])
    invariant = report['motion_invariant']
    if report['accepted'] and report['deployable_in_current_ros_tf']:
        status = '通过，可加载正式 YAML'
    elif report['accepted']:
        status = '手眼数值验收通过，但 ROS 运动学尚未修正，禁止加载'
    else:
        status = '未通过，禁止加载候选 YAML'
    lines = [
        '# P13 手眼标定离线分析',
        '',
        f'- 结论：{status}',
        f'- 数据：{report["training_samples"]} 组求解 + '
        f'{report["validation_samples"]} 组独立验证',
        f'- 求解样本编号：{report["training_sample_indices"]}',
        f'- 验证样本编号：{report["validation_sample_indices"]}',
        f'- 机器人位姿来源：{report["robot_pose_source"]["type"]} '
        f'（{report["robot_pose_source"]["frame"]}）',
        f'- 选择方法：{report["selected_method"]}',
        f'- 验证平移误差：中位数 '
        f'{selected["validation_translation_mm"]["median"]:.3f} mm，P95 '
        f'{selected["validation_translation_mm"]["p95"]:.3f} mm，最大 '
        f'{selected["validation_translation_mm"]["max"]:.3f} mm',
        f'- 验证旋转残差：中位数 '
        f'{selected["validation_rotation_deg"]["median"]:.3f}°，P95 '
        f'{selected["validation_rotation_deg"]["p95"]:.3f}°',
        f'- 求解姿态跨度：{report["training_max_pairwise_wrist_rotation_deg"]:.3f}°',
        f'- 相对旋转角不变量误差：中位数 '
        f'{invariant["absolute_rotation_angle_mismatch_deg"]["median"]:.3f}°，P95 '
        f'{invariant["absolute_rotation_angle_mismatch_deg"]["p95"]:.3f}°，最大 '
        f'{invariant["absolute_rotation_angle_mismatch_deg"]["max"]:.3f}° '
        f'（样本 {invariant["worst_pair"]["first_sample"]}/'
        f'{invariant["worst_pair"]["second_sample"]}）',
        '',
        '## 方法对比',
        '',
        '| 方法 | 验证平移中位数 / mm | 验证平移 P95 / mm | 验证旋转中位数 / ° | 验证旋转 P95 / ° |',
        '|---|---:|---:|---:|---:|',
    ]
    for method in report['methods']:
        lines.append(
            f'| {method["method"]} | '
            f'{method["validation_translation_mm"]["median"]:.3f} | '
            f'{method["validation_translation_mm"]["p95"]:.3f} | '
            f'{method["validation_rotation_deg"]["median"]:.3f} | '
            f'{method["validation_rotation_deg"]["p95"]:.3f} |')
    lines.extend(['', '## 判断', ''])
    if report['accepted'] and report['deployable_in_current_ros_tf']:
        crosscheck = report.get('integration_frame_crosscheck')
        if crosscheck:
            lines.append(
                f'控制器法兰与当前 ROS `{crosscheck["ros_frame"]}` 的 '
                f'{crosscheck["sample_count"]} 姿态交叉检查通过：平移离散最大 '
                f'{crosscheck["translation_spread_max_mm"]:.6f} mm，旋转离散最大 '
                f'{crosscheck["rotation_spread_max_deg"]:.6f}°。已将该固定变换与控制器法兰手眼结果'
                '组合为当前 ROS 可发布外参。')
        else:
            lines.append('固定板手眼模型通过留出验证，且父坐标系与当前 ROS TF 一致。')
    elif report['accepted'] and not report['deployable_in_current_ros_tf']:
        lines.append(
            '使用控制器 `GetForwardKin` 重建的实际法兰位姿后，固定板手眼模型通过留出验证；'
            '原始 ROS/URDF 位姿失败而控制器位姿通过，定位为 ROS 运动学模型问题。'
            '本结果定义的是 `controller_flange → camera_link`，不能冒充当前错误模型中的 '
            '`wrist3_link → camera_link`。')
    else:
        lines.append(
            '图像检测和单姿态静止性均通过，但跨姿态数据不能由一个固定的手眼变换解释。'
            '优先核对采样时使用的 ROS/URDF `base_link → wrist3_link` 与控制器实际法兰位姿的一致性，'
            '同时检查相机支架、线缆牵拉和标定板平整/固定情况。不得通过人工删点把本次结果改成“通过”。')
    lines.append('')
    if report['deployable_in_current_ros_tf']:
        lines.append(
            '当前结果 `deployable_in_current_ros_tf: true` 且 `calibrated: true`，可作为当前机械臂'
            '模型的正式手眼 TF。')
    else:
        lines.append(
            '只有 `deployable_in_current_ros_tf: true` 的结果才可加载。当前生成文件中的 '
            '`calibrated` 保持为 `false`。')
    path.write_text('\n'.join(lines) + '\n')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Solve and independently validate an existing hand-eye session.')
    parser.add_argument('session', help='session directory containing samples.json')
    parser.add_argument('--training-samples', type=int, default=25)
    parser.add_argument('--minimum-validation-samples', type=int, default=10)
    parser.add_argument(
        '--random-validation-samples', type=int,
        help=('randomly select this many held-out samples from the complete session; '
              'requires exactly training + validation samples'))
    parser.add_argument(
        '--random-seed', type=int, default=0,
        help='deterministic seed used with --random-validation-samples')
    parser.add_argument(
        '--robot-poses',
        help='optional controller FK reconstruction JSON, indexed like samples.json')
    parser.add_argument(
        '--frame-crosscheck',
        help=('three-or-more-pose JSON proving the current ROS end frame has a fixed '
              'relationship to the controller flange'))
    options = parser.parse_args(args)
    report = analyze(
        options.session,
        training_samples=options.training_samples,
        minimum_validation_samples=options.minimum_validation_samples,
        robot_poses_path=options.robot_poses,
        frame_crosscheck_path=options.frame_crosscheck,
        random_validation_samples=options.random_validation_samples,
        random_seed=options.random_seed,
    )
    print(json.dumps({
        'accepted': report['accepted'],
        'deployable_in_current_ros_tf': report['deployable_in_current_ros_tf'],
        'status': report['status'],
        'selected_method': report['selected_method'],
        'training_samples': report['training_samples'],
        'validation_samples': report['validation_samples'],
        'training_sample_indices': report['training_sample_indices'],
        'validation_sample_indices': report['validation_sample_indices'],
        'split_strategy': report['split_strategy'],
        'report': report['report_path'],
        'calibration': report['calibration_path'],
    }, indent=2))
    raise SystemExit(0 if report['accepted'] else 2)


if __name__ == '__main__':
    main()
