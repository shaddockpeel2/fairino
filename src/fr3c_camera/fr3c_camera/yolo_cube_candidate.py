"""Convert one YOLO garbage-class box into a cube-localization image mask."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class YoloCubeCandidate:
    contour: np.ndarray
    mask: np.ndarray
    center_px: tuple
    area_px: float


def candidate_from_detection(
        detection, image_shape, min_area_px, max_area_px,
        inset_fraction=0.06):
    """Use an inset box so RGB-D top fitting can reject non-cube detections."""
    height, width = image_shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError('invalid_yolo_image_shape')
    if not 0.0 <= inset_fraction < 0.5:
        raise ValueError('invalid_yolo_inset_fraction')
    x1, y1, x2, y2 = detection.bbox_xyxy
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        raise ValueError('nonfinite_yolo_box')
    x1 = int(max(0, min(width, math.floor(x1))))
    y1 = int(max(0, min(height, math.floor(y1))))
    x2 = int(max(0, min(width, math.ceil(x2))))
    y2 = int(max(0, min(height, math.ceil(y2))))
    area = float((x2 - x1) * (y2 - y1))
    if x2 <= x1 or y2 <= y1 or not min_area_px <= area <= max_area_px:
        raise ValueError('yolo_box_size_out_of_range')
    inset_x = int(round((x2 - x1) * inset_fraction))
    inset_y = int(round((y2 - y1) * inset_fraction))
    left, top = x1 + inset_x, y1 + inset_y
    right, bottom = x2 - inset_x, y2 - inset_y
    if right - left < 3 or bottom - top < 3:
        raise ValueError('yolo_box_interior_too_small')
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[top:bottom, left:right] = 255
    contour = np.array([
        [[x1, y1]], [[x2 - 1, y1]],
        [[x2 - 1, y2 - 1]], [[x1, y2 - 1]],
    ], dtype=np.int32)
    return YoloCubeCandidate(
        contour=contour,
        mask=mask,
        center_px=((x1 + x2) * 0.5, (y1 + y2) * 0.5),
        area_px=area,
    )
