from types import SimpleNamespace

import numpy as np
import pytest

from fr3c_camera.cube_perception import (
    blue_mask,
    extract_top_surface,
    registered_points_from_mask,
    transform_points,
)
from fr3c_camera.rknn_detector import Detection
from fr3c_camera.yolo_cube_candidate import candidate_from_detection


def detection(box):
    return Detection(13, 'Used_batteries', 0.9, box)


def camera_info():
    return SimpleNamespace(
        width=100, height=80,
        k=[500.0, 0.0, 50.0, 0.0, 500.0, 40.0, 0.0, 0.0, 1.0],
        d=[0.0] * 5, distortion_model='plumb_bob')


def test_yolo_box_localizes_nonblue_cube_with_existing_depth_geometry():
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    image[25:55, 35:65] = (0, 0, 255)
    assert np.count_nonzero(blue_mask(image)) == 0

    depth = np.full((80, 100), 0.53, dtype=np.float32)
    depth[25:55, 35:65] = 0.50
    candidate = candidate_from_detection(
        detection((30, 20, 70, 60)), image.shape, 120, 12000)
    points, valid_fraction = registered_points_from_mask(
        depth, camera_info(), camera_info(), candidate.mask)
    base_points = transform_points(points, [0.0, 0.0, 0.535], [1.0, 0.0, 0.0, 0.0])
    surface = extract_top_surface(base_points, valid_fraction)

    assert surface.center == pytest.approx([0.0, 0.0, 0.035], abs=0.002)
    assert surface.size_xy == pytest.approx([0.029, 0.029], abs=0.003)
    assert surface.axis_ratio < 1.25


def test_invalid_yolo_boxes_are_rejected_before_depth_localization():
    for box in ((30, 20, 30, 60), (0, 0, 5, 5), (float('nan'), 0, 40, 40)):
        with pytest.raises(ValueError):
            candidate_from_detection(detection(box), (80, 100, 3), 120, 12000)
