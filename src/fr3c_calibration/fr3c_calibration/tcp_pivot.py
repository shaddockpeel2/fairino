"""Read-only capture and robust offline solve for fixed-point TCP calibration."""

import argparse
from datetime import datetime
import fcntl
import itertools
import json
import math
import os
from pathlib import Path
import socket
import tempfile
import time
import xmlrpc.client

import numpy as np
import yaml

from .geometry import (
    average_transform,
    matrix_to_rpy,
    rotation_distance_deg,
    transform_matrix,
)
from .pose_crosscheck import rpy_degrees_to_rotation


DEFAULT_ROBOT_IP = '192.168.58.2'
DEFAULT_NOMINAL_TCP_MM = np.asarray([0.0, 0.0, 163.0])


def _now():
    return datetime.now().astimezone().isoformat()


def _atomic_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(handle, 'w') as stream:
            json.dump(document, stream, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _controller_read(proxy):
    flange = proxy.GetActualToolFlangePose(0)
    joints = proxy.GetActualJointPosDegree(0)
    if not flange or int(flange[0]) != 0 or len(flange) < 7:
        raise RuntimeError(f'GetActualToolFlangePose failed: {flange}')
    if not joints or int(joints[0]) != 0 or len(joints) < 7:
        raise RuntimeError(f'GetActualJointPosDegree failed: {joints}')
    flange_values = np.asarray(flange[1:7], dtype=float)
    joint_values = np.asarray(joints[1:7], dtype=float)
    if not np.all(np.isfinite(flange_values)) or not np.all(np.isfinite(joint_values)):
        raise RuntimeError('controller returned non-finite flange pose or joint position')
    matrix = transform_matrix(
        rpy_degrees_to_rotation(flange_values[3:]), flange_values[:3])
    return matrix, joint_values, flange_values


def _pairwise_orientation_span(transforms):
    if len(transforms) < 2:
        return 0.0
    return max(
        rotation_distance_deg(first[:3, :3], second[:3, :3])
        for first, second in itertools.combinations(transforms, 2)
    )


def _session_status(document):
    transforms = [
        np.asarray(sample['base_from_flange_mm'], dtype=float)
        for sample in document.get('samples', [])
    ]
    return {
        'sample_count': len(transforms),
        'orientation_pair_span_deg': _pairwise_orientation_span(transforms),
        'recommended_samples': 12,
        'minimum_samples_for_analysis': 8,
    }


def capture_sample(session_dir, robot_ip=DEFAULT_ROBOT_IP, observations=15,
                   interval_s=0.08, max_joint_spread_deg=0.03,
                   max_translation_spread_mm=0.15,
                   max_rotation_spread_deg=0.05):
    if observations < 5:
        raise ValueError('at least 5 observations are required for a stopped-pose sample')
    session_dir = Path(session_dir).expanduser().resolve()
    session_dir.mkdir(parents=True, exist_ok=True)
    samples_path = session_dir / 'tcp_samples.json'
    lock_path = session_dir / '.tcp_capture.lock'

    with lock_path.open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        if samples_path.exists():
            document = json.loads(samples_path.read_text())
            if document.get('calibration_type') != 'fixed_point_pivot':
                raise ValueError(f'not a TCP pivot session: {samples_path}')
            if document.get('robot', {}).get('ip') != robot_ip:
                raise ValueError('robot IP differs from the existing session')
        else:
            document = {
                'schema_version': 1,
                'calibration_type': 'fixed_point_pivot',
                'created_at': _now(),
                'robot': {'ip': robot_ip, 'pose_source': 'GetActualToolFlangePose(0)'},
                'frames': {'parent': 'controller_base', 'tool_mount': 'flange_link'},
                'pose_convention': {
                    'controller_pose': '[x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg]',
                    'rotation': 'Rz(rz) * Ry(ry) * Rx(rx)',
                },
                'tcp': {
                    'definition': 'YDI-J22 flexible-finger common closing tip',
                    'nominal_xyz_mm': DEFAULT_NOMINAL_TCP_MM.tolist(),
                    'rpy_deg': [0.0, 0.0, 0.0],
                    'orientation_source': 'mechanical mounting; not observable by pivot solve',
                },
                'samples': [],
            }

        socket.setdefaulttimeout(3.0)
        proxy = xmlrpc.client.ServerProxy(f'http://{robot_ip}:20003/RPC2')
        matrices = []
        joints = []
        flange_values = []
        raw = []
        started = time.monotonic()
        for index in range(observations):
            matrix, joint, flange = _controller_read(proxy)
            matrices.append(matrix)
            joints.append(joint)
            flange_values.append(flange)
            raw.append({
                'offset_s': float(time.monotonic() - started),
                'joint_deg': joint.tolist(),
                'flange_pose_mm_deg': flange.tolist(),
            })
            if index + 1 < observations:
                time.sleep(interval_s)

        # Controller translations and the pivot solver both use millimetres.
        # geometry.transform_spread() expects metres and multiplies translation
        # differences by 1000, so compute this burst spread in native mm here.
        centre = average_transform(matrices)
        translation_spread = np.asarray([
            np.linalg.norm(matrix[:3, 3] - centre[:3, 3]) for matrix in matrices
        ])
        rotation_spread = np.asarray([
            rotation_distance_deg(centre[:3, :3], matrix[:3, :3])
            for matrix in matrices
        ])
        joint_array = np.asarray(joints)
        joint_spread = np.ptp(joint_array, axis=0)
        nominal_tip_positions = np.asarray([
            matrix[:3, 3] + matrix[:3, :3] @ DEFAULT_NOMINAL_TCP_MM
            for matrix in matrices
        ])
        nominal_tip_centre = np.mean(nominal_tip_positions, axis=0)
        nominal_tip_spread = np.linalg.norm(
            nominal_tip_positions - nominal_tip_centre, axis=1)
        quality = {
            'observation_count': observations,
            'duration_s': float((observations - 1) * interval_s),
            'joint_spread_max_deg': float(np.max(joint_spread)),
            'flange_translation_spread_max_mm': float(np.max(translation_spread)),
            'flange_rotation_spread_max_deg': float(np.max(rotation_spread)),
            'nominal_tip_spread_max_mm': float(np.max(nominal_tip_spread)),
            'limits': {
                'joint_spread_max_deg': max_joint_spread_deg,
                'flange_translation_spread_max_mm': max_translation_spread_mm,
                'flange_rotation_spread_max_deg': max_rotation_spread_deg,
            },
        }
        quality['accepted'] = bool(
            quality['joint_spread_max_deg'] <= max_joint_spread_deg
            and quality['flange_translation_spread_max_mm'] <= max_translation_spread_mm
            and quality['flange_rotation_spread_max_deg'] <= max_rotation_spread_deg
        )
        if not quality['accepted']:
            return {
                'captured': False,
                'reason': 'robot_not_stationary',
                'quality': quality,
                'session': str(session_dir),
                **_session_status(document),
            }

        previous = [
            np.asarray(sample['base_from_flange_mm'], dtype=float)
            for sample in document['samples']
        ]
        nearest_orientation = None
        if previous:
            nearest_orientation = min(
                rotation_distance_deg(centre[:3, :3], item[:3, :3])
                for item in previous
            )
        sample = {
            'index': len(document['samples']) + 1,
            'captured_at': _now(),
            'base_from_flange_mm': centre.tolist(),
            'controller_flange_pose_mm_deg_mean': np.mean(
                np.asarray(flange_values), axis=0).tolist(),
            'controller_joint_deg_mean': np.mean(joint_array, axis=0).tolist(),
            'nearest_prior_orientation_deg': (
                None if nearest_orientation is None else float(nearest_orientation)),
            'quality': quality,
            'raw_observations': raw,
        }
        document['samples'].append(sample)
        document['updated_at'] = _now()
        document['status'] = _session_status(document)
        _atomic_json(samples_path, document)
        return {
            'captured': True,
            'sample_index': sample['index'],
            'session': str(session_dir),
            'nearest_prior_orientation_deg': sample['nearest_prior_orientation_deg'],
            'quality': quality,
            **document['status'],
        }


def _linear_pivot_solve(transforms):
    transforms = [np.asarray(item, dtype=float).reshape(4, 4) for item in transforms]
    matrix = np.vstack([
        np.hstack([item[:3, :3], -np.eye(3)]) for item in transforms
    ])
    target = np.hstack([-item[:3, 3] for item in transforms])
    solution, _, rank, singular = np.linalg.lstsq(matrix, target, rcond=None)
    tcp = solution[:3]
    pivot = solution[3:]
    estimates = np.asarray([
        item[:3, 3] + item[:3, :3] @ tcp for item in transforms
    ])
    residuals = np.linalg.norm(estimates - pivot, axis=1)
    condition = float(np.inf if singular[-1] <= 0 else singular[0] / singular[-1])
    return {
        'tcp_xyz_mm': tcp,
        'pivot_xyz_mm': pivot,
        'pivot_estimates_mm': estimates,
        'residuals_mm': residuals,
        'rank': int(rank),
        'condition_number': condition,
        'singular_values': singular,
    }


def solve_pivot(transforms, outlier_limit_mm=2.0):
    transforms = [np.asarray(item, dtype=float).reshape(4, 4) for item in transforms]
    if len(transforms) < 4:
        raise ValueError('pivot solve requires at least 4 poses')
    best = None
    combinations = list(itertools.combinations(range(len(transforms)), 4))
    if len(combinations) > 5000:
        rng = np.random.default_rng(20260910)
        selected = rng.choice(len(combinations), size=5000, replace=False)
        combinations = [combinations[index] for index in sorted(selected)]
    for indices in combinations:
        candidate = _linear_pivot_solve([transforms[index] for index in indices])
        if candidate['rank'] < 6:
            continue
        tcp = candidate['tcp_xyz_mm']
        pivot = candidate['pivot_xyz_mm']
        residuals = np.asarray([
            np.linalg.norm(item[:3, 3] + item[:3, :3] @ tcp - pivot)
            for item in transforms
        ])
        inliers = residuals <= outlier_limit_mm
        inlier_count = int(np.sum(inliers))
        if inlier_count < 4:
            continue
        score = (inlier_count, -float(np.median(residuals[inliers])))
        if best is None or score > best[0]:
            best = (score, inliers)
    if best is None:
        raise ValueError('pose set is rank deficient; add different flange tilts')
    inliers = best[1]
    if int(np.sum(inliers)) < 4:
        raise ValueError('fewer than 4 mutually consistent fixed-point samples')
    solved = _linear_pivot_solve([
        transform for transform, keep in zip(transforms, inliers) if keep
    ])
    final_residuals = np.asarray([
        np.linalg.norm(
            item[:3, 3] + item[:3, :3] @ solved['tcp_xyz_mm']
            - solved['pivot_xyz_mm'])
        for item in transforms
    ])
    inliers = final_residuals <= outlier_limit_mm
    if int(np.sum(inliers)) >= 4 and not np.all(inliers):
        solved = _linear_pivot_solve([
            transform for transform, keep in zip(transforms, inliers) if keep
        ])
        final_residuals = np.asarray([
            np.linalg.norm(
                item[:3, 3] + item[:3, :3] @ solved['tcp_xyz_mm']
                - solved['pivot_xyz_mm'])
            for item in transforms
        ])
        inliers = final_residuals <= outlier_limit_mm
    solved['all_residuals_mm'] = final_residuals
    solved['inlier_mask'] = inliers
    return solved


def _leave_one_out_stability(transforms, inlier_indices, outlier_limit_mm):
    estimates = []
    if len(inlier_indices) < 5:
        return {'solutions': 0, 'max_tcp_delta_mm': float('inf')}
    selected = [transforms[index] for index in inlier_indices]
    full = _linear_pivot_solve(selected)['tcp_xyz_mm']
    for omitted in range(len(selected)):
        subset = selected[:omitted] + selected[omitted + 1:]
        solved = solve_pivot(subset, outlier_limit_mm)
        estimates.append(solved['tcp_xyz_mm'])
    deltas = np.linalg.norm(np.asarray(estimates) - full, axis=1)
    return {
        'solutions': len(estimates),
        'max_tcp_delta_mm': float(np.max(deltas)),
        'median_tcp_delta_mm': float(np.median(deltas)),
    }


def analyze_session(session_dir, minimum_samples=8, outlier_limit_mm=2.0,
                    rms_limit_mm=1.5, loo_limit_mm=1.0,
                    orientation_span_min_deg=25.0,
                    condition_number_limit=100.0,
                    nominal_delta_limit_mm=10.0,
                    maximum_tcp_length_mm=180.0):
    session_dir = Path(session_dir).expanduser().resolve()
    samples_path = session_dir / 'tcp_samples.json'
    document = json.loads(samples_path.read_text())
    samples = document.get('samples', [])
    transforms = [
        np.asarray(sample['base_from_flange_mm'], dtype=float) for sample in samples
    ]
    if len(transforms) < 4:
        raise ValueError(f'need at least 4 samples to solve; got {len(transforms)}')
    solved = solve_pivot(transforms, outlier_limit_mm)
    inlier_mask = solved['inlier_mask']
    inlier_indices = np.flatnonzero(inlier_mask).tolist()
    inlier_residuals = solved['all_residuals_mm'][inlier_mask]
    orientation_span = _pairwise_orientation_span([
        transforms[index] for index in inlier_indices
    ])
    nominal = np.asarray(document['tcp']['nominal_xyz_mm'], dtype=float)
    nominal_delta = solved['tcp_xyz_mm'] - nominal
    loo = _leave_one_out_stability(transforms, inlier_indices, outlier_limit_mm)
    required_inliers = max(6, int(math.ceil(0.75 * len(samples))))
    gates = {
        'minimum_samples': len(samples) >= minimum_samples,
        'minimum_inliers': len(inlier_indices) >= required_inliers,
        'full_rank': solved['rank'] == 6,
        'orientation_span': orientation_span >= orientation_span_min_deg,
        'condition_number': solved['condition_number'] <= condition_number_limit,
        'residual_rms': float(np.sqrt(np.mean(inlier_residuals ** 2))) <= rms_limit_mm,
        'residual_max': float(np.max(inlier_residuals)) <= outlier_limit_mm,
        'leave_one_out_stability': loo['max_tcp_delta_mm'] <= loo_limit_mm,
        'physical_tcp_length': (
            float(np.linalg.norm(solved['tcp_xyz_mm'])) <= maximum_tcp_length_mm),
    }
    diagnostics = {
        'nominal_delta_within_review_limit': (
            float(np.linalg.norm(nominal_delta)) <= nominal_delta_limit_mm),
    }
    accepted = all(gates.values())
    limits = {
        'minimum_samples': minimum_samples,
        'minimum_inliers': required_inliers,
        'outlier_and_residual_max_mm': outlier_limit_mm,
        'residual_rms_mm': rms_limit_mm,
        'leave_one_out_max_tcp_delta_mm': loo_limit_mm,
        'orientation_pair_span_min_deg': orientation_span_min_deg,
        'condition_number_max': condition_number_limit,
        'nominal_tcp_delta_max_mm': nominal_delta_limit_mm,
        'physical_tcp_length_max_mm': maximum_tcp_length_mm,
    }
    report = {
        'schema_version': 1,
        'analyzed_at': _now(),
        'accepted': accepted,
        'session': str(session_dir),
        'sample_count': len(samples),
        'inlier_sample_indices': [index + 1 for index in inlier_indices],
        'outlier_sample_indices': [
            index + 1 for index, keep in enumerate(inlier_mask) if not keep
        ],
        'tcp_xyz_mm': solved['tcp_xyz_mm'].tolist(),
        'tcp_rpy_deg': document['tcp']['rpy_deg'],
        'pivot_xyz_mm': solved['pivot_xyz_mm'].tolist(),
        'nominal_tcp_xyz_mm': nominal.tolist(),
        'delta_from_nominal_mm': nominal_delta.tolist(),
        'delta_from_nominal_norm_mm': float(np.linalg.norm(nominal_delta)),
        'tcp_length_mm': float(np.linalg.norm(solved['tcp_xyz_mm'])),
        'residuals_mm': [
            {'sample': index + 1, 'value': float(value), 'inlier': bool(inlier_mask[index])}
            for index, value in enumerate(solved['all_residuals_mm'])
        ],
        'inlier_residual_rms_mm': float(np.sqrt(np.mean(inlier_residuals ** 2))),
        'inlier_residual_max_mm': float(np.max(inlier_residuals)),
        'orientation_pair_span_deg': orientation_span,
        'matrix_rank': solved['rank'],
        'condition_number': solved['condition_number'],
        'singular_values': solved['singular_values'].tolist(),
        'leave_one_out': loo,
        'limits': limits,
        'gates': gates,
        'diagnostics': diagnostics,
        'interpretation': (
            'Pivot calibration observes TCP translation only. Tool RPY remains the '
            'mechanically defined flange-aligned orientation.'),
    }
    _atomic_json(session_dir / 'tcp_solve_report.json', report)
    output_key = 'ydi_j22_tcp' if accepted else 'ydi_j22_tcp_candidate'
    candidate = {
        output_key: {
            'version': 2,
            'calibrated': accepted,
            'production_config_updated': False,
            'parent_frame': 'flange_link',
            'child_frame': 'grasp_frame',
            'xyz_m': [float(value / 1000.0) for value in solved['tcp_xyz_mm']],
            'rpy_rad': [float(value) for value in matrix_to_rpy(np.eye(3))],
            'controller_pose_mm_deg': [
                *[float(value) for value in solved['tcp_xyz_mm']], 0.0, 0.0, 0.0],
            'translation_source': 'fixed_point_pivot_calibration',
            'orientation_source': 'mechanical_flange_alignment',
            'source_session': str(session_dir),
            'report': str(session_dir / 'tcp_solve_report.json'),
            'controller_tool_id': None,
        }
    }
    output_name = 'ydi_j22_tcp_v2.yaml' if accepted else 'ydi_j22_tcp_v2.candidate.yaml'
    (session_dir / output_name).write_text(yaml.safe_dump(candidate, sort_keys=False))
    return report, session_dir / output_name


def capture_main(args=None):
    parser = argparse.ArgumentParser(
        description='Append one stationary controller-flange pose to a TCP pivot session.')
    parser.add_argument('--session', required=True)
    parser.add_argument('--robot-ip', default=DEFAULT_ROBOT_IP)
    parser.add_argument('--observations', type=int, default=15)
    parser.add_argument('--interval', type=float, default=0.08)
    parser.add_argument('--status', action='store_true')
    options = parser.parse_args(args)
    session_dir = Path(options.session).expanduser().resolve()
    if options.status:
        samples_path = session_dir / 'tcp_samples.json'
        if not samples_path.is_file():
            raise SystemExit(f'TCP session does not exist: {samples_path}')
        document = json.loads(samples_path.read_text())
        print(json.dumps({'session': str(session_dir), **_session_status(document)}, indent=2))
        return
    try:
        result = capture_sample(
            session_dir, options.robot_ip, options.observations, options.interval)
    except BlockingIOError:
        result = {'captured': False, 'reason': 'another_capture_is_running'}
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result.get('captured') else 2)


def analyze_main(args=None):
    parser = argparse.ArgumentParser(
        description='Robustly solve and validate a fixed-point TCP calibration session.')
    parser.add_argument('session')
    parser.add_argument('--minimum-samples', type=int, default=8)
    parser.add_argument('--outlier-limit-mm', type=float, default=2.0)
    parser.add_argument('--rms-limit-mm', type=float, default=1.5)
    parser.add_argument('--maximum-tcp-length-mm', type=float, default=180.0)
    options = parser.parse_args(args)
    report, output = analyze_session(
        options.session, minimum_samples=options.minimum_samples,
        outlier_limit_mm=options.outlier_limit_mm,
        rms_limit_mm=options.rms_limit_mm,
        maximum_tcp_length_mm=options.maximum_tcp_length_mm)
    print(json.dumps({
        'accepted': report['accepted'],
        'session': report['session'],
        'sample_count': report['sample_count'],
        'inlier_sample_indices': report['inlier_sample_indices'],
        'outlier_sample_indices': report['outlier_sample_indices'],
        'tcp_xyz_mm': report['tcp_xyz_mm'],
        'delta_from_nominal_mm': report['delta_from_nominal_mm'],
        'tcp_length_mm': report['tcp_length_mm'],
        'inlier_residual_rms_mm': report['inlier_residual_rms_mm'],
        'orientation_pair_span_deg': report['orientation_pair_span_deg'],
        'leave_one_out': report['leave_one_out'],
        'gates': report['gates'],
        'diagnostics': report['diagnostics'],
        'output': str(output),
    }, indent=2))
    raise SystemExit(0 if report['accepted'] else 2)
