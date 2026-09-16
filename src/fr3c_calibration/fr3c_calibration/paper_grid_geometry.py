"""Geometry for an A4 sheet measured at its four TCP-contact corners."""

import math

import numpy as np


CORNER_NAMES = ('TL', 'TR', 'BR', 'BL')
REGION_NAMES = ('A', 'B', 'C', 'D')  # top left, top right, bottom right, bottom left


def _unit(vector, description):
    length = float(np.linalg.norm(vector))
    if length < 1e-8:
        raise ValueError(f'{description} has zero length')
    return vector / length


def quaternion_z_axis(quaternion):
    """Return the TCP local +Z axis in the base frame (xyzw quaternion)."""
    x, y, z, w = _unit(np.asarray(quaternion, dtype=float), 'TCP quaternion')
    return np.asarray([2 * (x * z + w * y),
                       2 * (y * z - w * x),
                       1 - 2 * (x * x + y * y)])


def axis_angle_deg(first, second):
    return math.degrees(math.acos(float(np.clip(np.dot(
        _unit(first, 'first axis'), _unit(second, 'second axis')), -1, 1))))


def make_grid(corners, plane_z_m=None):
    """Fit a rectangle and split it into four equal-area rectangles.

    `corners` maps TL/TR/BR/BL to records containing xyz_m and
    quaternion_xyzw. Input corners are TCP measurements in base_link. When
    plane_z_m is given, project them along base Z onto that horizontal plane.
    """
    if set(corners) != set(CORNER_NAMES):
        raise ValueError('four corners TL, TR, BR, BL are required')
    measured = np.asarray([corners[name]['xyz_m'] for name in CORNER_NAMES], dtype=float)
    if measured.shape != (4, 3) or not np.all(np.isfinite(measured)):
        raise ValueError('corner positions must be finite 3-D coordinates')
    points = measured.copy()
    if plane_z_m is not None:
        if not math.isfinite(plane_z_m):
            raise ValueError('projection plane Z must be finite')
        points[:, 2] = plane_z_m
    tl, tr, br, bl = points
    u_raw = ((tr - tl) + (br - bl)) / 2
    v_raw = ((bl - tl) + (br - tr)) / 2
    u = _unit(u_raw, 'paper width direction')
    normal = _unit(np.cross(u_raw, v_raw), 'paper normal')
    v = _unit(np.cross(normal, u), 'paper height direction')
    width = float(np.dot(u_raw, u))
    height = float(np.dot(v_raw, v))
    if width <= 0 or height <= 0:
        raise ValueError('corner order is reversed or degenerate')
    if any(abs(actual - expected) > 0.025 for actual, expected in
           zip(sorted((width, height)), (0.210, 0.297))):
        raise ValueError(f'corner dimensions {width*1000:.1f} x '
                         f'{height*1000:.1f} mm differ from A4 (210 x 297 mm)')
    center = np.mean(points, axis=0)

    def at(x, y):
        return center + (x - 0.5) * width * u + (y - 0.5) * height * v

    fitted = np.asarray([at(0, 0), at(1, 0), at(1, 1), at(0, 1)])
    fit_error = float(np.max(np.linalg.norm(points - fitted, axis=1)))
    plane_error = float(np.max(np.abs((points - center) @ normal)))
    if plane_error > 0.004:
        raise ValueError(f'corners are not coplanar: {plane_error*1000:.1f} mm')
    if fit_error > 0.012:
        raise ValueError(f'corners do not form a rectangle: fit error '
                         f'{fit_error*1000:.1f} mm')
    axes = [quaternion_z_axis(corners[name]['quaternion_xyzw'])
            for name in CORNER_NAMES]
    orientation_spread = max(axis_angle_deg(axes[0], axis) for axis in axes[1:])
    if orientation_spread > 8.0:
        raise ValueError(f'TCP Z axes differ by {orientation_spread:.1f} degrees')
    normal_deviation = max(min(axis_angle_deg(axis, normal),
                               axis_angle_deg(axis, -normal)) for axis in axes)
    if normal_deviation > 12.0:
        raise ValueError(f'TCP Z is not parallel to paper normal: '
                         f'{normal_deviation:.1f} degrees')

    bounds = ((0, 0, .5, .5), (.5, 0, 1, .5),
              (.5, .5, 1, 1), (0, .5, .5, 1))
    regions = {}
    for name, (x0, y0, x1, y1) in zip(REGION_NAMES, bounds):
        regions[name] = {
            'corners_m': [at(x0, y0).tolist(), at(x1, y0).tolist(),
                          at(x1, y1).tolist(), at(x0, y1).tolist()],
            'center_m': at((x0 + x1) / 2, (y0 + y1) / 2).tolist(),
            'width_m': width / 2,
            'height_m': height / 2,
            'area_m2': width * height / 4,
        }
    return {
        'projection': {'mode': 'base_z', 'plane_z_m': plane_z_m} if plane_z_m is not None
                      else {'mode': 'measured_3d'},
        'measured_tcp_z_range_m': [float(np.min(measured[:, 2])),
                                   float(np.max(measured[:, 2]))],
        'measured_tcp_z_span_mm': float(np.ptp(measured[:, 2]) * 1000),
        'center_m': center.tolist(),
        'normal': normal.tolist(),
        'width_m': width,
        'height_m': height,
        'max_plane_error_mm': plane_error * 1000,
        'max_rectangle_fit_error_mm': fit_error * 1000,
        'max_tcp_z_spread_deg': orientation_spread,
        'max_tcp_z_to_plane_normal_deg': normal_deviation,
        'regions': regions,
    }
