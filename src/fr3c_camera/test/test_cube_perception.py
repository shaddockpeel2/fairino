from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from fr3c_camera.cube_perception import (
    blue_mask,
    extract_top_surface,
    find_blue_candidates,
    quaternion_angular_distance,
    registered_points_from_mask,
    rpy_to_quaternion,
    transform_points,
)


def camera_info(width=640, height=480, distortion=None):
    if distortion is None:
        distortion = np.zeros(5)
    return SimpleNamespace(
        width=width, height=height,
        k=[480.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0],
        d=list(distortion), distortion_model='plumb_bob')


def test_blue_candidate_is_unique_and_uses_eroded_interior():
    image = np.full((240, 320, 3), 220, dtype=np.uint8)
    cv2.rectangle(image, (130, 90), (175, 137), (120, 30, 10), -1)
    mask = blue_mask(image)
    candidates = find_blue_candidates(mask)
    assert len(candidates) == 1
    candidate = candidates[0]
    assert 1500 < candidate.area_px < 2500
    assert np.count_nonzero(candidate.mask) < np.count_nonzero(mask)
    assert candidate.solidity > 0.95


def test_multiple_blue_squares_are_not_silently_merged():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 30), (65, 65), (255, 0, 0), -1)
    cv2.rectangle(image, (180, 120), (220, 160), (255, 0, 0), -1)
    assert len(find_blue_candidates(blue_mask(image))) == 2


def test_registered_depth_back_projection_and_top_fit():
    info = camera_info(width=100, height=80)
    info.k = [200.0, 0.0, 50.0, 0.0, 200.0, 40.0, 0.0, 0.0, 1.0]
    depth = np.full((80, 100), 0.50, dtype=np.float32)
    mask = np.zeros_like(depth, dtype=np.uint8)
    mask[34:47, 44:57] = 255
    points, valid_fraction = registered_points_from_mask(depth, info, info, mask)
    points_base = transform_points(points, [0.1, -0.2, 0.0], [0.0, 0.0, 0.0, 1.0])
    surface = extract_top_surface(
        points_base, valid_fraction, top_quantile=60.0,
        top_tolerance_m=0.002, min_top_points=20)
    assert surface.center == pytest.approx([0.1, -0.2, 0.5], abs=0.002)
    assert surface.size_xy == pytest.approx([0.03, 0.03], abs=0.006)
    assert surface.depth_valid_fraction == 1.0


def test_rpy_quaternion_distance_detects_mismatched_online_tf():
    expected = rpy_to_quaternion(0.1, -0.2, 0.3)
    same_sign_flipped = -expected
    mismatch = rpy_to_quaternion(0.1, -0.2, 0.5)
    assert quaternion_angular_distance(expected, same_sign_flipped) < 1e-9
    assert quaternion_angular_distance(expected, mismatch) == pytest.approx(0.2, abs=0.01)
