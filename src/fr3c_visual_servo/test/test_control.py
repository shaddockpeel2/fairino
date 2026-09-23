import math

import pytest

from fr3c_visual_servo.control import (
    ControllerConfig,
    Observation,
    Phase,
    Termination,
    TrackingController,
)


def observation(now, sample_id=1, error=(100.0, 0.0), valid=True,
                reason='stable_target', color='red', age=0.0):
    return Observation(
        sample_id=sample_id,
        color=color,
        valid=valid,
        reason=reason,
        target_u_px=320.0 + error[0],
        target_v_px=240.0 + error[1],
        center_u_px=320.0,
        center_v_px=240.0,
        image_width_px=640.0,
        image_height_px=480.0,
        age_at_receive_s=age,
        received_monotonic_s=now,
    )


def controller(config=None, max_duration=0.0):
    return TrackingController(
        'red', config or ControllerConfig(), started_s=0.0,
        max_duration_s=max_duration)


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError, match='exit_region'):
        ControllerConfig(exit_half_width_px=30.0).validate()
    with pytest.raises(ValueError, match='axis_sign'):
        ControllerConfig(axis_sign_x=0.0).validate()
    with pytest.raises(ValueError, match='75_mm_per_second'):
        ControllerConfig(max_speed_mps=0.0751).validate()
    with pytest.raises(ValueError, match='dropout_grace'):
        ControllerConfig(dropout_grace_s=0.21).validate()
    with pytest.raises(ValueError, match='max_acceleration_mps2_exceeds'):
        ControllerConfig(max_acceleration_mps2=0.091).validate()
    with pytest.raises(ValueError, match='initial_center_region'):
        ControllerConfig(initial_center_half_width_px=41.0).validate()


@pytest.mark.parametrize('gain,speed,acceleration', [
    (0.120, 0.010, 0.030),
    (0.240, 0.015, 0.090),
    (1.200, 0.075, 0.090),
])
def test_fast_profile_caps_diagonal_speed_and_stops_on_target_loss(
        gain, speed, acceleration):
    config = ControllerConfig(
        gain_x_mps=gain, gain_y_mps=gain,
        max_speed_mps=speed, max_acceleration_mps2=acceleration,
        dropout_grace_s=0.0)
    tracker = controller(config)
    output = None
    for index in range(50):
        now = index * 0.02
        output = tracker.update(
            observation(now, sample_id=index, error=(100.0, 100.0)), now)
    assert output.phase == Phase.INITIAL_CENTERING
    assert math.hypot(output.command_x_mps, output.command_y_mps) == \
        pytest.approx(speed)
    lost = tracker.update(observation(
        1.00, sample_id=50, valid=False, reason='target_not_found'), 1.00)
    assert lost.phase == Phase.LOST_HOLD
    assert (lost.command_x_mps, lost.command_y_mps) == (0.0, 0.0)


def test_no_target_stays_acquiring_and_commands_zero():
    tracker = controller()
    output = tracker.update(None, 0.1)
    assert output.phase == Phase.ACQUIRING
    assert (output.command_x_mps, output.command_y_mps) == (0.0, 0.0)
    assert not output.done


def test_acquisition_timeout_reports_not_found():
    tracker = controller(ControllerConfig(acquisition_timeout_s=0.5))
    missing = observation(
        0.5, valid=False, reason='target_not_found', error=(0.0, 0.0))
    output = tracker.update(missing, 0.5)
    assert output.phase == Phase.STOPPING
    assert output.termination == Termination.TARGET_NOT_FOUND


def test_off_center_control_has_expected_direction_acceleration_and_speed_limits():
    config = ControllerConfig(max_acceleration_mps2=0.090)
    tracker = controller(config)
    tracker.update(observation(0.0, error=(320.0, 0.0)), 0.0)
    output = tracker.update(observation(
        0.1, sample_id=2, error=(320.0, 0.0)), 0.1)
    assert output.phase == Phase.INITIAL_CENTERING
    assert output.command_x_mps == pytest.approx(config.max_speed_mps)
    assert output.command_y_mps == 0.0

    diagonal = controller(config)
    diagonal.update(observation(0.0, error=(320.0, 240.0)), 0.0)
    output = diagonal.update(observation(
        0.1, sample_id=2, error=(320.0, 240.0)), 0.1)
    assert output.command_x_mps > 0.0
    assert output.command_y_mps > 0.0
    assert math.hypot(output.command_x_mps, output.command_y_mps) == \
        pytest.approx(config.max_speed_mps)


def test_acceleration_limit_applies_between_tracking_commands():
    tracker = controller()
    tracker.update(observation(0.0, error=(200.0, 0.0)), 0.0)
    output = tracker.update(observation(
        0.02, sample_id=2, error=(200.0, 0.0)), 0.02)
    assert output.command_x_mps == pytest.approx(0.0006)


def test_center_entry_counts_unique_camera_frames_only():
    tracker = controller()
    first = observation(0.0, sample_id=10, error=(10.0, 5.0))
    assert tracker.update(first, 0.0).phase == Phase.INITIAL_CENTERING
    assert tracker.update(first, 0.02).phase == Phase.INITIAL_CENTERING
    second = observation(0.1, sample_id=11, error=(9.0, 4.0))
    assert tracker.update(second, 0.1).phase == Phase.INITIAL_CENTERING
    third = observation(0.2, sample_id=12, error=(8.0, 3.0))
    output = tracker.update(third, 0.2)
    assert output.phase == Phase.CENTER_HOLD
    assert output.centered
    assert (output.command_x_mps, output.command_y_mps) == (0.0, 0.0)


def test_center_hold_hysteresis_ignores_motion_between_regions():
    tracker = controller()
    for sample_id, now in enumerate((0.0, 0.1, 0.2), start=1):
        output = tracker.update(
            observation(now, sample_id=sample_id, error=(0.0, 0.0)), now)
    assert output.phase == Phase.CENTER_HOLD
    output = tracker.update(
        observation(0.3, sample_id=4, error=(50.0, 40.0)), 0.3)
    assert output.phase == Phase.CENTER_HOLD
    assert (output.command_x_mps, output.command_y_mps) == (0.0, 0.0)


def test_center_exit_requires_two_unique_frames():
    tracker = controller()
    for sample_id, now in enumerate((0.0, 0.1, 0.2), start=1):
        tracker.update(
            observation(now, sample_id=sample_id, error=(0.0, 0.0)), now)
    outside = observation(0.3, sample_id=4, error=(80.0, 0.0))
    assert tracker.update(outside, 0.3).phase == Phase.CENTER_HOLD
    assert tracker.update(outside, 0.32).phase == Phase.CENTER_HOLD
    output = tracker.update(
        observation(0.4, sample_id=5, error=(80.0, 0.0)), 0.4)
    assert output.phase == Phase.TRACKING
    assert output.command_x_mps > 0.0


def test_visible_off_center_target_is_centered_before_normal_tracking():
    tracker = controller(ControllerConfig(max_acceleration_mps2=0.090))
    first = tracker.update(observation(0.1, error=(100.0, -60.0)), 0.1)
    assert first.phase == Phase.INITIAL_CENTERING
    assert first.detail == 'centering_initial_target'
    assert first.command_x_mps > 0.0
    assert first.command_y_mps < 0.0
    assert not first.centered

    nearly_centered = tracker.update(observation(
        0.2, sample_id=2, error=(20.0, 0.0)), 0.2)
    assert nearly_centered.phase == Phase.INITIAL_CENTERING
    assert nearly_centered.command_x_mps > 0.0

    for sample_id, now in enumerate((0.3, 0.4, 0.5), start=3):
        centered = tracker.update(observation(
            now, sample_id=sample_id, error=(5.0, 4.0)), now)
    assert centered.phase == Phase.CENTER_HOLD
    assert centered.centered
    assert centered.command_x_mps == centered.command_y_mps == 0.0

    tracker.update(observation(0.6, sample_id=6, error=(-90.0, 0.0)), 0.6)
    following = tracker.update(observation(
        0.7, sample_id=7, error=(-90.0, 0.0)), 0.7)
    assert following.phase == Phase.TRACKING
    assert following.command_x_mps < 0.0


def test_invalid_observation_immediately_zeros_then_recovers_tracking():
    tracker = controller(ControllerConfig(
        max_acceleration_mps2=0.090, dropout_grace_s=0.0))
    tracker.update(observation(0.0, error=(200.0, 0.0)), 0.0)
    moving = tracker.update(observation(
        0.1, sample_id=2, error=(200.0, 0.0)), 0.1)
    assert moving.command_x_mps > 0.0
    missing = observation(
        0.11, sample_id=3, valid=False, reason='target_not_found')
    lost = tracker.update(missing, 0.11)
    assert lost.phase == Phase.LOST_HOLD
    assert (lost.command_x_mps, lost.command_y_mps) == (0.0, 0.0)
    recovered = tracker.update(observation(
        0.2, sample_id=4, error=(-100.0, 0.0)), 0.2)
    assert recovered.phase == Phase.INITIAL_CENTERING
    assert recovered.command_x_mps < 0.0


def test_center_hold_recovers_without_chatter_after_short_loss():
    tracker = controller()
    for sample_id, now in enumerate((0.0, 0.1, 0.2), start=1):
        tracker.update(
            observation(now, sample_id=sample_id, error=(0.0, 0.0)), now)
    tracker.update(observation(
        0.3, sample_id=4, valid=False, reason='target_not_found'), 0.3)
    recovered = tracker.update(observation(
        0.4, sample_id=5, error=(45.0, 35.0)), 0.4)
    assert recovered.phase == Phase.CENTER_HOLD
    assert (recovered.command_x_mps, recovered.command_y_mps) == (0.0, 0.0)


def test_sustained_non_unique_target_stops_with_specific_code():
    tracker = controller(ControllerConfig(lost_timeout_s=0.5))
    tracker.update(observation(0.0, error=(100.0, 0.0)), 0.0)
    invalid = observation(
        0.1, sample_id=2, valid=False, reason='target_not_unique')
    assert tracker.update(invalid, 0.1).phase == Phase.LOST_HOLD
    refreshed_invalid = observation(
        0.61, sample_id=3, valid=False, reason='target_not_unique')
    output = tracker.update(refreshed_invalid, 0.61)
    assert output.phase == Phase.STOPPING
    assert output.termination == Termination.TARGET_NOT_UNIQUE


def test_default_target_loss_waits_at_zero_until_recovery_or_cancel():
    tracker = controller()
    tracker.update(observation(0.0, error=(100.0, 0.0)), 0.0)
    for index, now in enumerate((0.1, 1.1, 5.1), start=2):
        lost = tracker.update(observation(
            now, sample_id=index, valid=False, reason='target_not_unique'),
            now)
        assert lost.phase == Phase.LOST_HOLD
        assert not lost.done
        assert (lost.command_x_mps, lost.command_y_mps) == (0.0, 0.0)
    recovered = tracker.update(observation(
        5.2, sample_id=5, error=(100.0, 0.0)), 5.2)
    assert recovered.phase == Phase.INITIAL_CENTERING
    assert recovered.command_x_mps > 0.0
    canceled = tracker.stop(Termination.CANCELED, 'cancel_requested')
    assert canceled.done and canceled.command_x_mps == 0.0


def test_short_dropout_bridges_once_but_stops_after_time_or_distance_limit():
    tracker = controller(ControllerConfig(
        gain_x_mps=1.2, max_speed_mps=0.075,
        max_acceleration_mps2=0.09))
    for index in range(15):
        now = index * 0.02
        moving = tracker.update(observation(
            now, sample_id=index, error=(320.0, 0.0)), now)
    speed = moving.command_x_mps
    invalid = observation(
        0.30, sample_id=15, valid=False, reason='target_not_found')
    bridged = tracker.update(invalid, 0.30)
    assert bridged.phase == Phase.INITIAL_CENTERING
    assert bridged.detail == 'bridging_short_target_gap:target_not_found'
    assert bridged.command_x_mps == speed
    # 5 mm / 25.2 mm/s is longer than the 160 ms grace.
    lost = tracker.update(invalid, 0.45)
    assert lost.phase == Phase.LOST_HOLD
    assert lost.command_x_mps == 0.0

    fast = controller(ControllerConfig(
        gain_x_mps=1.2, max_speed_mps=0.075,
        max_acceleration_mps2=0.09))
    for index in range(46):
        now = index * 0.02
        moving = fast.update(observation(
            now, sample_id=index, error=(320.0, 0.0)), now)
    assert moving.command_x_mps == pytest.approx(0.075)
    assert fast.update(observation(
        0.94, sample_id=46, valid=False, reason='target_not_unique'),
        0.94).phase == Phase.INITIAL_CENTERING
    assert fast.update(observation(
        0.98, sample_id=47, valid=False, reason='target_not_unique'),
        0.98).phase == Phase.LOST_HOLD


def test_stale_image_stream_never_bridges_target_dropout():
    tracker = controller(ControllerConfig(max_acceleration_mps2=0.09))
    tracker.update(observation(0.0, error=(320.0, 0.0)), 0.0)
    moving = tracker.update(observation(
        0.1, sample_id=2, error=(320.0, 0.0)), 0.1)
    assert moving.command_x_mps > 0
    stale = tracker.update(observation(
        0.12, sample_id=3, valid=False, reason='image_stream_stale'),
        0.12)
    assert stale.phase == Phase.LOST_HOLD
    assert stale.command_x_mps == 0.0


def test_stale_observation_can_only_produce_zero():
    tracker = controller()
    stale = observation(
        0.0, error=(200.0, 0.0),
        age=tracker.config.observation_stale_s + 0.05)
    output = tracker.update(stale, 0.0)
    assert output.phase == Phase.ACQUIRING
    assert (output.command_x_mps, output.command_y_mps) == (0.0, 0.0)


def test_nonzero_max_duration_is_a_successful_bounded_run():
    tracker = controller(max_duration=0.5)
    output = tracker.update(None, 0.5)
    assert output.phase == Phase.STOPPING
    assert output.termination == Termination.OK


def test_tracking_continues_past_old_80_mm_relative_displacement():
    tracker = controller(ControllerConfig(max_acceleration_mps2=0.090))
    for i in range(200):
        now = i * 0.1
        output = tracker.update(
            observation(now, sample_id=i, error=(320.0, 0.0)), now)
        assert not output.done
    assert tracker.estimated_displacement_xy_m[0] > 0.08
    assert output.command_x_mps > 0.0
