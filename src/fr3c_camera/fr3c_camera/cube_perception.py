"""
Pure blue-cube perception helpers used by the ROS localization node.

All image coordinates are pixels and all 3-D quantities are metres.  The
functions deliberately contain no ROS imports so the geometry can be tested
offline with synthetic data.
"""

from dataclasses import dataclass, replace

import cv2
import numpy as np


@dataclass(frozen=True)
class BlueCandidate:
    contour: np.ndarray
    mask: np.ndarray
    center_px: tuple
    area_px: float
    aspect_ratio: float
    solidity: float
    rectangularity: float


@dataclass(frozen=True)
class TopSurface:
    center: np.ndarray
    size_xy: np.ndarray
    long_axis_yaw_rad: float
    square_edge_yaw_rad: float
    axis_ratio: float
    point_count: int
    top_point_count: int
    depth_valid_fraction: float
    z_spread: float


def compensate_top_surface_z(surface, z_compensation_m):
    """Apply a signed base-Z correction without altering raw surface quality."""
    compensation = float(z_compensation_m)
    center = np.asarray(surface.center, dtype=np.float64)
    if center.shape != (3,) or not np.isfinite(center).all() or \
            not np.isfinite(compensation):
        raise ValueError('invalid_top_surface_z_compensation')
    corrected = center.copy()
    corrected[2] += compensation
    return replace(surface, center=corrected)


def blue_mask(
        bgr, hue_min=90, hue_max=135, saturation_min=80, value_min=35,
        morphology_size=3):
    """Segment blue pixels in a BGR image using an OpenCV HSV interval."""
    image = np.asarray(bgr)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('bgr_image_must_be_uint8_hwc3')
    values = (hue_min, hue_max, saturation_min, value_min)
    if not all(np.isfinite(values)) or not 0 <= hue_min < hue_max <= 179:
        raise ValueError('invalid_hue_interval')
    if not 0 <= saturation_min <= 255 or not 0 <= value_min <= 255:
        raise ValueError('invalid_sv_threshold')
    if morphology_size < 0:
        raise ValueError('invalid_morphology_size')

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        (int(hue_min), int(saturation_min), int(value_min)),
        (int(hue_max), 255, 255),
    )
    if morphology_size >= 2:
        size = int(morphology_size) | 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def color_mask(bgr, hue_intervals, saturation_min=80, value_min=35,
               morphology_size=3):
    """Segment one named color, including hue intervals crossing red's seam."""
    if not hue_intervals or len(hue_intervals) % 2:
        raise ValueError('invalid_color_hue_intervals')
    mask = np.zeros(np.asarray(bgr).shape[:2], dtype=np.uint8)
    for lower, upper in zip(hue_intervals[::2], hue_intervals[1::2]):
        if not 0 <= lower <= upper <= 179:
            raise ValueError('invalid_color_hue_intervals')
        # Reuse the existing validated HSV/morphology path for each interval.
        if lower == upper:
            raise ValueError('invalid_color_hue_intervals')
        mask = cv2.bitwise_or(mask, blue_mask(
            bgr, lower, upper, saturation_min, value_min, morphology_size))
    return mask


def find_color_candidates(bgr, color_intervals, saturation_min=80,
                          value_min=35, morphology_size=3, **candidate_filter):
    """Return (color, candidate) pairs; callers require exactly one overall."""
    found = []
    for color, intervals in color_intervals.items():
        mask = color_mask(bgr, intervals, saturation_min, value_min,
                          morphology_size)
        found.extend((color, item) for item in
                     find_blue_candidates(mask, **candidate_filter))
    return found


def is_tabletop_cube(surface, cube_height_m, cube_max_axis_ratio,
                     plane_z_m, maximum_plane_gap_m):
    """Require a square top at the expected height above the work plane."""
    if not np.isfinite((cube_height_m, cube_max_axis_ratio, plane_z_m,
                        maximum_plane_gap_m)).all() or cube_height_m <= 0 or \
            cube_max_axis_ratio < 1 or maximum_plane_gap_m < 0:
        raise ValueError('invalid_tabletop_cube_filter')
    center = np.asarray(surface.center, dtype=np.float64)
    if center.shape != (3,) or not np.isfinite(center).all() or \
            not np.isfinite(surface.axis_ratio):
        return False
    gap = float(center[2] - cube_height_m - plane_z_m)
    return surface.axis_ratio <= cube_max_axis_ratio and \
        0.0 <= gap <= maximum_plane_gap_m


def find_blue_candidates(
        mask, min_area_px=120.0, max_area_px=12000.0,
        min_aspect_ratio=0.45, max_aspect_ratio=2.2,
        min_solidity=0.72, min_rectangularity=0.55,
        interior_fraction=0.23):
    """Return geometry-filtered connected blue regions and interior masks."""
    binary = np.asarray(mask)
    if binary.ndim != 2 or binary.dtype != np.uint8:
        raise ValueError('mask_must_be_uint8_hw')
    if not 0 < min_area_px < max_area_px:
        raise ValueError('invalid_area_interval')
    if not 0 < min_aspect_ratio <= 1 <= max_aspect_ratio:
        raise ValueError('invalid_aspect_interval')
    if not 0 <= interior_fraction < 1:
        raise ValueError('invalid_interior_fraction')

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area_px or area > max_area_px:
            continue
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        (_, _), (width, height), _ = cv2.minAreaRect(contour)
        if hull_area <= 0 or width <= 0 or height <= 0:
            continue
        aspect = float(width / height)
        solidity = area / hull_area
        rectangularity = area / (width * height)
        if not (min_aspect_ratio <= aspect <= max_aspect_ratio):
            continue
        if solidity < min_solidity or rectangularity < min_rectangularity:
            continue

        filled = np.zeros_like(binary)
        cv2.drawContours(filled, [contour], -1, 255, thickness=cv2.FILLED)
        distance = cv2.distanceTransform(filled, cv2.DIST_L2, 5)
        maximum = float(distance.max())
        interior = np.zeros_like(binary)
        if maximum > 0:
            interior[distance >= max(1.5, maximum * interior_fraction)] = 255
        moments = cv2.moments(contour)
        if moments['m00'] <= 0 or np.count_nonzero(interior) < 9:
            continue
        center = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'])
        candidates.append(BlueCandidate(
            contour=contour,
            mask=interior,
            center_px=center,
            area_px=area,
            aspect_ratio=aspect,
            solidity=solidity,
            rectangularity=rectangularity,
        ))
    return sorted(candidates, key=lambda item: item.area_px, reverse=True)


def _camera_matrix(info):
    matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=np.float64)
    if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError('invalid_intrinsics')
    if not np.isfinite(distortion).all():
        raise ValueError('invalid_distortion')
    return matrix, distortion


def _undistort(pixels, info):
    matrix, distortion = _camera_matrix(info)
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    model = info.distortion_model
    if model in ('plumb_bob', 'rational_polynomial'):
        return cv2.undistortPoints(points, matrix, distortion).reshape(-1, 2)
    if model == 'equidistant':
        if distortion.size != 4:
            raise ValueError('invalid_fisheye_distortion')
        return cv2.fisheye.undistortPoints(points, matrix, distortion).reshape(-1, 2)
    if distortion.size == 0 or np.all(distortion == 0):
        result = points.reshape(-1, 2).copy()
        result[:, 0] = (result[:, 0] - matrix[0, 2]) / matrix[0, 0]
        result[:, 1] = (result[:, 1] - matrix[1, 2]) / matrix[1, 1]
        return result
    raise ValueError('unsupported_distortion_model')


def _distort(rays, info):
    matrix, distortion = _camera_matrix(info)
    rays = np.asarray(rays, dtype=np.float64).reshape(-1, 2)
    model = info.distortion_model
    if model in ('plumb_bob', 'rational_polynomial'):
        xyz = np.column_stack((rays, np.ones(rays.shape[0], dtype=np.float64)))
        pixels, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), matrix, distortion)
        return pixels.reshape(-1, 2)
    if model == 'equidistant':
        pixels = cv2.fisheye.distortPoints(rays.reshape(-1, 1, 2), matrix, distortion)
        return pixels.reshape(-1, 2)
    if distortion.size == 0 or np.all(distortion == 0):
        result = rays.copy()
        result[:, 0] = result[:, 0] * matrix[0, 0] + matrix[0, 2]
        result[:, 1] = result[:, 1] * matrix[1, 1] + matrix[1, 2]
        return result
    raise ValueError('unsupported_distortion_model')


def registered_points_from_mask(
        depth_m, color_info, depth_info, mask,
        min_depth_m=0.15, max_depth_m=1.5):
    """Back-project an RGB mask through the registered-depth camera model."""
    depth = np.asarray(depth_m, dtype=np.float32)
    binary = np.asarray(mask)
    if depth.ndim != 2 or binary.shape != (color_info.height, color_info.width):
        raise ValueError('image_shape_mismatch')
    if depth.shape != (depth_info.height, depth_info.width):
        raise ValueError('depth_shape_mismatch')
    if not 0 < min_depth_m < max_depth_m:
        raise ValueError('invalid_depth_interval')

    color_v, color_u = np.nonzero(binary)
    if color_u.size == 0:
        raise ValueError('empty_candidate_mask')
    color_pixels = np.column_stack((color_u, color_v)).astype(np.float64)
    color_rays = _undistort(color_pixels, color_info)
    depth_pixels = _distort(color_rays, depth_info)
    depth_uv = np.rint(depth_pixels).astype(np.int64)
    inside = (
        (depth_uv[:, 0] >= 0) & (depth_uv[:, 0] < depth.shape[1]) &
        (depth_uv[:, 1] >= 0) & (depth_uv[:, 1] < depth.shape[0]))
    attempted = int(color_u.size)
    if not np.any(inside):
        raise ValueError('mapped_mask_outside_depth_image')
    depth_uv = depth_uv[inside]
    z = depth[depth_uv[:, 1], depth_uv[:, 0]].astype(np.float64)
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    if not np.any(valid):
        raise ValueError('no_valid_candidate_depth')
    depth_uv = depth_uv[valid]
    z = z[valid]
    depth_rays = _undistort(depth_uv.astype(np.float64), depth_info)
    points = np.column_stack((depth_rays[:, 0] * z, depth_rays[:, 1] * z, z))
    return points, float(points.shape[0] / attempted)


def quaternion_matrix_xyzw(quaternion):
    """Create a 3x3 rotation matrix from an xyzw quaternion."""
    q = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.dot(q, q))
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('invalid_quaternion')
    x, y, z, w = q / np.sqrt(norm)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def transform_points(points, translation, quaternion_xyzw):
    points = np.asarray(points, dtype=np.float64)
    offset = np.asarray(translation, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or offset.shape != (3,):
        raise ValueError('invalid_transform_shapes')
    if not np.isfinite(points).all() or not np.isfinite(offset).all():
        raise ValueError('nonfinite_transform')
    return points @ quaternion_matrix_xyzw(quaternion_xyzw).T + offset


def extract_top_surface(
        points_base, depth_valid_fraction, top_quantile=65.0,
        top_tolerance_m=0.006, min_top_points=30):
    """Estimate a horizontal cube top from the highest robust point cluster."""
    points = np.asarray(points_base, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < min_top_points:
        raise ValueError('insufficient_candidate_points')
    if not np.isfinite(points).all() or not 50 <= top_quantile < 100:
        raise ValueError('invalid_top_surface_input')
    if top_tolerance_m <= 0 or not 0 <= depth_valid_fraction <= 1:
        raise ValueError('invalid_top_surface_parameters')

    threshold = float(np.percentile(points[:, 2], top_quantile))
    seed = points[points[:, 2] >= threshold]
    top_z = float(np.median(seed[:, 2]))
    top = points[np.abs(points[:, 2] - top_z) <= top_tolerance_m]
    if top.shape[0] < min_top_points:
        raise ValueError('insufficient_top_surface_points')

    xy = top[:, :2]
    robust_origin = np.median(xy, axis=0)
    centered = xy - robust_origin
    covariance = centered.T @ centered / max(1, centered.shape[0] - 1)
    eigenvalues, axes = np.linalg.eigh(covariance)
    coordinates = centered @ axes
    low, high = np.percentile(coordinates, [2.5, 97.5], axis=0)
    size_xy = np.sort(high - low)
    center_xy = robust_origin + ((low + high) * 0.5) @ axes.T
    # eigh() returns the minor and major axes in ascending eigenvalue order.
    # A rectangle has an unoriented long axis: yaw and yaw + pi describe the
    # same physical axis.  Keep the published representative in [-pi/2, pi/2)
    # and let downstream code handle that pi symmetry explicitly.
    long_axis = axes[:, 1]
    long_axis_yaw = float(np.arctan2(long_axis[1], long_axis[0]))
    long_axis_yaw = float((long_axis_yaw + np.pi * 0.5) % np.pi - np.pi * 0.5)
    minor_variance = max(float(eigenvalues[0]), 1e-12)
    axis_ratio = float(np.sqrt(max(float(eigenvalues[1]), 0.0) / minor_variance))
    # PCA intentionally has no preferred direction for a square. Recover a
    # grasp-equivalent edge from the minimum-area top-surface rectangle and
    # canonicalize it modulo 90 degrees. Either adjacent edge is therefore
    # equivalent and angle wrapping cannot create a false discontinuity.
    rectangle = cv2.minAreaRect(xy.astype(np.float32))
    corners = cv2.boxPoints(rectangle).astype(np.float64)
    edges = np.roll(corners, -1, axis=0) - corners
    edge = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
    square_edge_yaw = float(np.arctan2(edge[1], edge[0]))
    square_period = np.pi * 0.5
    square_edge_yaw = float(
        (square_edge_yaw + square_period * 0.5) % square_period -
        square_period * 0.5)
    z_low, z_high = np.percentile(top[:, 2], [10.0, 90.0])
    return TopSurface(
        center=np.array([center_xy[0], center_xy[1], np.median(top[:, 2])]),
        size_xy=size_xy,
        long_axis_yaw_rad=long_axis_yaw,
        square_edge_yaw_rad=square_edge_yaw,
        axis_ratio=axis_ratio,
        point_count=int(points.shape[0]),
        top_point_count=int(top.shape[0]),
        depth_valid_fraction=float(depth_valid_fraction),
        z_spread=float(z_high - z_low),
    )


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], dtype=np.float64)


def quaternion_angular_distance(first, second):
    a = np.asarray(first, dtype=np.float64)
    b = np.asarray(second, dtype=np.float64)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    return float(2.0 * np.arccos(np.clip(abs(np.dot(a, b)), -1.0, 1.0)))
