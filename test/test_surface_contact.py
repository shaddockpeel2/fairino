import numpy as np
import pytest

from fr3c_calibration.surface_contact import (
    _axis_angle_deg,
    _comparison,
    _position_statistics,
    _quaternion_z_axis,
    _rpy_deg,
)


def test_position_statistics_use_robust_median_and_mm_units():
    result = _position_statistics([
        [0.100, -0.200, 0.030],
        [0.101, -0.201, 0.031],
        [0.099, -0.199, 0.029],
    ])
    assert result['median_xyz_m'] == pytest.approx([0.1, -0.2, 0.03])
    assert result['range_xyz_mm'] == pytest.approx([2.0, 2.0, 2.0])
    assert result['max_deviation_mm'] == pytest.approx(np.sqrt(3.0))


def test_comparison_is_tcp_minus_visual_top():
    result = _comparison(
        {'median_xyz_m': [-0.01, -0.47, 0.036]},
        {'median_xyz_m': [-0.009, -0.472, 0.031]},
    )
    assert result['tcp_minus_color_top_xyz_mm'] == pytest.approx([1, -2, -5])
    assert result['xy_distance_mm'] == pytest.approx(np.sqrt(5))
    assert result['z_difference_mm'] == pytest.approx(-5)


def test_quaternion_helpers_for_identity_and_half_turn_x():
    identity_axis = _quaternion_z_axis([0, 0, 0, 1])
    inverted_axis = _quaternion_z_axis([1, 0, 0, 0])
    assert identity_axis == pytest.approx([0, 0, 1])
    assert inverted_axis == pytest.approx([0, 0, -1])
    assert _axis_angle_deg(identity_axis, inverted_axis) == pytest.approx(180)
    assert _rpy_deg([1, 0, 0, 0]) == pytest.approx([180, 0, 0])
