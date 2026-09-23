import math

import pytest

from stage4_pulse_logic import (
    command_for_side, error_improved, pulse_nominal_limit_m,
    pulse_observed_limit_m, pulse_verdict, rotate_xy,
)


def target(error_x, error_y):
    return {'error_px': [error_x, error_y],
            'image_size_px': [640.0, 480.0]}


def test_camera_rotation_and_limited_visual_command():
    half = math.sqrt(0.5)
    assert rotate_xy((1.0, 0.0), (0.0, 0.0, half, half)) == pytest.approx(
        (0.0, 1.0))
    command = command_for_side(
        'right', target(200.0, 0.0), (0.0, 0.0, half, half))
    assert command[0] == pytest.approx(0.0, abs=1e-9)
    assert command[1] == pytest.approx(0.005)
    assert command_for_side(
        'up', target(0.0, -100.0), (0.0, 0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, -0.020 * 100.0 / 480.0))


def test_wrong_side_or_nonfinite_geometry_cannot_pulse():
    with pytest.raises(ValueError, match='not_left'):
        command_for_side('left', target(70.0, 0.0), (0, 0, 0, 1))
    with pytest.raises(ValueError, match='invalid_target_geometry'):
        command_for_side('right', target(70.0, math.nan), (0, 0, 0, 1))
    with pytest.raises(ValueError, match='invalid_target_geometry'):
        command_for_side('right', target(70.0, 0.0), (0, 0, 0, 1),
                         max_speed_mps=0.076)
    assert command_for_side(
        'right', target(200.0, 0.0), (0, 0, 0, 1),
        gain_mps=0.120, max_speed_mps=0.010) == pytest.approx((0.010, 0.0))


def test_pulse_verdict_uses_command_direction_and_distance():
    kwargs = dict(servo_targets=5, controller_outputs=5, nonzero_count=10)
    assert pulse_verdict((0.005, 0), 0.5, (0.001, 0, 0),
                         **kwargs) == 'direction_confirmed'
    assert pulse_verdict((0.005, 0), 0.5, (0.0001, 0, 0),
                         **kwargs) == 'undertravel'
    assert pulse_verdict((0, -0.005), 0.5, (0, 0.001, 0),
                         **kwargs) == 'wrong_direction_or_no_motion'
    assert pulse_verdict((0.005, 0), 0.5, (0.006, 0, 0),
                         **kwargs) == 'displacement_limit_exceeded'
    assert pulse_verdict((0.005, 0), 0.5, (0.001, 0, 0),
                         servo_targets=0, controller_outputs=5,
                         nonzero_count=10) == 'no_servo_trajectory_observed'


def test_high_speed_pulse_requires_two_servo_updates_and_limits_distance():
    kwargs = dict(controller_outputs=5, nonzero_count=10)
    assert pulse_nominal_limit_m(0.075) == pytest.approx(0.015)
    assert pulse_observed_limit_m(0.075, 0.2) == pytest.approx(0.020)
    assert pulse_verdict((0.075, 0), 0.2, (0.007, 0, 0),
                         servo_targets=1, **kwargs) == \
        'insufficient_servo_trajectories'
    assert pulse_verdict((0.075, 0), 0.2, (0.007, 0, 0),
                         servo_targets=2, **kwargs) == 'direction_confirmed'
    assert pulse_verdict((0.075, 0), 0.2, (0.021, 0, 0),
                         servo_targets=2, **kwargs) == \
        'displacement_limit_exceeded'


def test_image_error_change_is_axis_specific():
    assert error_improved('left', (-100, 50), (-96, 100))
    assert not error_improved('left', (-100, 50), (-99.8, 50))
    assert error_improved('down', (100, 100), (100, 95))
