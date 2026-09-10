import math

import numpy as np

from fr3c_calibration.geometry import transform_matrix
from fr3c_calibration.tcp_pivot import capture_sample, solve_pivot


def _rotation(vector):
    angle = np.linalg.norm(vector)
    if angle == 0.0:
        return np.eye(3)
    axis = np.asarray(vector, dtype=float) / angle
    skew = np.asarray([[0.0, -axis[2], axis[1]],
                       [axis[2], 0.0, -axis[0]],
                       [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _synthetic_poses(tcp, pivot):
    vectors = [
        [0.0, 0.0, 0.0], [0.25, 0.0, 0.0], [-0.25, 0.0, 0.0],
        [0.0, 0.3, 0.0], [0.0, -0.3, 0.0], [0.18, 0.22, 0.1],
        [-0.2, 0.16, -0.15], [0.24, -0.18, 0.2], [-0.22, -0.2, -0.1],
    ]
    poses = []
    for vector in vectors:
        rotation = _rotation(np.asarray(vector))
        translation = pivot - rotation @ tcp
        poses.append(transform_matrix(rotation, translation))
    return poses


def test_pivot_solver_recovers_exact_tcp():
    tcp = np.asarray([1.2, -0.8, 162.4])
    pivot = np.asarray([420.0, -115.0, 260.0])
    solved = solve_pivot(_synthetic_poses(tcp, pivot))
    assert np.all(solved['inlier_mask'])
    assert np.allclose(solved['tcp_xyz_mm'], tcp, atol=1e-9)
    assert np.allclose(solved['pivot_xyz_mm'], pivot, atol=1e-9)
    assert float(np.max(solved['all_residuals_mm'])) < 1e-9


def test_pivot_solver_rejects_moved_reference_sample():
    tcp = np.asarray([0.5, -1.0, 163.5])
    pivot = np.asarray([350.0, 20.0, 180.0])
    poses = _synthetic_poses(tcp, pivot)
    poses[4] = poses[4].copy()
    poses[4][:3, 3] += np.asarray([8.0, -5.0, 4.0])
    solved = solve_pivot(poses, outlier_limit_mm=1.0)
    assert not solved['inlier_mask'][4]
    assert int(np.sum(solved['inlier_mask'])) == len(poses) - 1
    assert np.allclose(solved['tcp_xyz_mm'], tcp, atol=1e-8)


def test_capture_stationarity_uses_controller_millimetres(monkeypatch, tmp_path):
    tcp = np.asarray([0.0, 0.0, 163.0])
    pivot = np.asarray([420.0, -115.0, 260.0])
    rotation = _rotation(np.asarray([0.2, -0.1, 0.05]))
    flange = pivot - rotation @ tcp
    observations = iter([
        (transform_matrix(rotation, flange + [offset, 0.0, 0.0]),
         np.zeros(6), np.zeros(6))
        for offset in np.linspace(-0.004, 0.004, 15)
    ])
    monkeypatch.setattr(
        'fr3c_calibration.tcp_pivot._controller_read', lambda _proxy: next(observations))
    monkeypatch.setattr(
        'fr3c_calibration.tcp_pivot.xmlrpc.client.ServerProxy', lambda _url: object())
    monkeypatch.setattr('fr3c_calibration.tcp_pivot.time.sleep', lambda _seconds: None)
    result = capture_sample(tmp_path, observations=15)
    assert result['captured']
    assert result['quality']['flange_translation_spread_max_mm'] < 0.005
