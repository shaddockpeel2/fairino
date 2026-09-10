"""Small SE(3) helpers kept independent from ROS for testing."""
import math

import cv2
import numpy as np


def transform_matrix(rotation, translation):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def invert_transform(matrix):
    matrix = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def quaternion_to_rotation(x, y, z, w):
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = quaternion / norm
    return np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def average_rotation(rotations):
    matrices = np.asarray(rotations, dtype=np.float64).reshape(-1, 3, 3)
    if not len(matrices):
        raise ValueError('cannot average zero rotations')
    u, _, vt = np.linalg.svd(np.sum(matrices, axis=0))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def average_transform(transforms):
    matrices = np.asarray(transforms, dtype=np.float64).reshape(-1, 4, 4)
    return transform_matrix(
        average_rotation(matrices[:, :3, :3]),
        np.mean(matrices[:, :3, 3], axis=0),
    )


def rotation_distance_deg(first, second):
    relative = np.asarray(first).reshape(3, 3).T @ np.asarray(second).reshape(3, 3)
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


def transform_spread(transforms):
    matrices = np.asarray(transforms, dtype=np.float64).reshape(-1, 4, 4)
    centre = average_transform(matrices)
    translation_mm = np.linalg.norm(matrices[:, :3, 3] - centre[:3, 3], axis=1) * 1000.0
    rotation_deg = np.asarray([
        rotation_distance_deg(centre[:3, :3], matrix[:3, :3]) for matrix in matrices
    ])
    return centre, translation_mm, rotation_deg


def matrix_to_rpy(rotation):
    """Return fixed-axis XYZ roll/pitch/yaw for R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    pitch = math.asin(float(np.clip(-r[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return [roll, pitch, yaw]


def rodrigues_transform(rvec, tvec):
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return transform_matrix(rotation, tvec)


def handeye_residuals(base_from_wrist, wrist_from_camera, camera_from_board,
                      reference=None):
    estimates = [
        np.asarray(base) @ np.asarray(wrist_from_camera) @ np.asarray(camera)
        for base, camera in zip(base_from_wrist, camera_from_board)
    ]
    if reference is None:
        reference = average_transform(estimates)
    translation_mm = np.asarray([
        np.linalg.norm(matrix[:3, 3] - reference[:3, 3]) * 1000.0 for matrix in estimates
    ])
    rotation_deg = np.asarray([
        rotation_distance_deg(reference[:3, :3], matrix[:3, :3]) for matrix in estimates
    ])
    return reference, translation_mm, rotation_deg
