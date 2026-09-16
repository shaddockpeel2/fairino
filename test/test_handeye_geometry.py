import math

import cv2
import numpy as np

from fr3c_calibration.analyze_session import _split_positions
from fr3c_calibration.geometry import (
    average_transform,
    handeye_residuals,
    invert_transform,
    matrix_to_rpy,
    rotation_distance_deg,
    transform_matrix,
)


def test_random_handeye_split_is_reproducible_and_not_tail_only():
    training, validation, strategy = _split_positions(
        range(1, 21), 15, 5, random_validation_samples=5, random_seed=20260914)
    assert [position + 1 for position in validation] == [1, 12, 14, 15, 18]
    assert len(training) == 15
    assert set(training).isdisjoint(validation)
    assert strategy == {
        'type': 'random_holdout',
        'seed': 20260914,
        'requested_validation_samples': 5,
    }


def test_default_handeye_split_keeps_ordered_tail_behavior():
    training, validation, strategy = _split_positions(range(1, 41), 25, 10)
    assert training == list(range(25))
    assert validation == list(range(25, 40))
    assert strategy == {'type': 'ordered_tail'}


def rotation_z(angle):
    return np.asarray([[math.cos(angle), -math.sin(angle), 0.0],
                       [math.sin(angle), math.cos(angle), 0.0],
                       [0.0, 0.0, 1.0]])


def test_inverse_and_average_transform():
    matrix = transform_matrix(rotation_z(0.2), [0.1, -0.2, 0.3])
    assert np.allclose(matrix @ invert_transform(matrix), np.eye(4))
    assert np.allclose(average_transform([matrix, matrix]), matrix)


def test_rpy_and_rotation_distance():
    assert np.allclose(matrix_to_rpy(rotation_z(0.3)), [0.0, 0.0, 0.3])
    assert abs(rotation_distance_deg(np.eye(3), rotation_z(math.pi / 2)) - 90.0) < 1e-9


def test_exact_handeye_residual_is_zero():
    wrist_from_camera = transform_matrix(rotation_z(0.15), [0.03, 0.02, 0.08])
    base_from_board = transform_matrix(rotation_z(-0.1), [0.4, -0.2, 0.01])
    base_from_wrist = [
        transform_matrix(rotation_z(angle), [0.1 + index * 0.02, -0.3, 0.4])
        for index, angle in enumerate([0.0, 0.2, -0.25])
    ]
    camera_from_board = [
        invert_transform(wrist_from_camera) @ invert_transform(base) @ base_from_board
        for base in base_from_wrist
    ]
    _, translation, rotation = handeye_residuals(
        base_from_wrist, wrist_from_camera, camera_from_board)
    assert float(np.max(translation)) < 1e-9
    assert float(np.max(rotation)) < 1e-5


def test_opencv_handeye_direction_matches_wrist_from_camera():
    wrist_from_camera = transform_matrix(
        cv2.Rodrigues(np.asarray([0.12, -0.08, 0.2]))[0], [0.03, 0.02, 0.08])
    base_from_board = transform_matrix(
        cv2.Rodrigues(np.asarray([-0.05, 0.1, -0.15]))[0], [0.4, -0.2, 0.01])
    base_from_wrist = []
    for index, vector in enumerate([
            [0.0, 0.0, 0.0], [0.25, 0.05, -0.1], [-0.2, 0.18, 0.08],
            [0.1, -0.24, 0.2], [-0.15, -0.12, -0.22], [0.3, -0.1, 0.15],
            [-0.28, 0.22, -0.05], [0.08, 0.3, -0.18]]):
        rotation = cv2.Rodrigues(np.asarray(vector, dtype=float))[0]
        base_from_wrist.append(transform_matrix(
            rotation, [0.1 + index * 0.015, -0.3 + index * 0.01, 0.4 - index * 0.008]))
    camera_from_board = [
        invert_transform(wrist_from_camera) @ invert_transform(base) @ base_from_board
        for base in base_from_wrist
    ]
    rotation, translation = cv2.calibrateHandEye(
        [matrix[:3, :3] for matrix in base_from_wrist],
        [matrix[:3, 3] for matrix in base_from_wrist],
        [matrix[:3, :3] for matrix in camera_from_board],
        [matrix[:3, 3] for matrix in camera_from_board],
        method=cv2.CALIB_HAND_EYE_PARK)
    solved = transform_matrix(rotation, translation)
    assert np.linalg.norm(solved[:3, 3] - wrist_from_camera[:3, 3]) < 1e-9
    assert rotation_distance_deg(solved[:3, :3], wrist_from_camera[:3, :3]) < 1e-5
