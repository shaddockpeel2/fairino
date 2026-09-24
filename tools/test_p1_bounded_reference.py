from dataclasses import replace
import math

import pytest

from p1_bounded_reference import BoundedReference, Limits


def limits():
    return Limits(lower=(-1.0,), upper=(1.0,), velocity=(0.1,),
                  acceleration=(1.0,), following_error=(0.01,), travel=(0.1,))


def armed(mode='integrated', bounds=None, guard=lambda actual, target: True):
    controller = BoundedReference(bounds or limits(), guard, mode)
    assert controller.arm(0, [0], 0, lease_active=True).state == 'active'
    return controller


def step(controller, now=0.02, actual=(0,), velocity=(0.05,), **kwargs):
    return controller.step(now, actual, kwargs.pop('feedback_stamp', now), velocity,
                           kwargs.pop('command_stamp', now),
                           lease_active=kwargs.pop('lease_active', True), **kwargs)


def test_stationary_feedback_does_not_erase_reference_but_stall_latches():
    integrated, feedback = armed(), armed('feedback_step')
    for i in range(1, 10):
        a, b = step(integrated, i * 0.02), step(feedback, i * 0.02)
    assert a.position[0] > 5 * b.position[0]
    for i in range(10, 30):
        result = step(integrated, i * 0.02)
        if result.stop_required:
            break
    assert result.reason == 'following_error'
    assert result.position is None
    assert integrated.reference is None
    assert step(integrated, 1.0).state == 'fault'
    with pytest.raises(RuntimeError):
        integrated.arm(1, [0], 1, lease_active=True)


@pytest.mark.parametrize('kwargs,reason', [
    ({'velocity': (0,)}, 'zero_command'),
    ({'cancel': True}, 'cancel'),
    ({'lease_active': False}, 'lease_lost'),
    ({'velocity': (math.nan,)}, 'invalid_vector'),
    ({'actual': (math.inf,)}, 'invalid_vector'),
    ({'velocity': (0.2,)}, 'velocity_limit'),
    ({'actual': (1.1,)}, 'joint_limit'),
    ({'velocity': (0.01, 0.01)}, 'invalid_vector'),
    ({'now': 0.04}, 'invalid_period'),
    ({'now': -0.01}, 'invalid_period'),
    ({'now': math.nan}, 'invalid_time'),
    ({'command_stamp': 0.03}, 'command_stale_or_reordered'),
    ({'feedback_stamp': 0.03}, 'feedback_stale_or_reordered'),
])
def test_invalid_inputs_and_stop_revoke_target(kwargs, reason):
    controller = armed()
    result = step(controller, **kwargs)
    assert result.reason == reason
    assert result.stop_required and result.position is None
    assert controller.reference is None
    assert step(controller, 0.04).position is None


@pytest.mark.parametrize('source,reason', [
    ('command_stamp', 'command_stale_or_reordered'),
    ('feedback_stamp', 'feedback_stale_or_reordered'),
])
def test_no_refresh_and_reordered_timestamps_stop(source, reason):
    controller = armed()
    for i in range(1, 7):
        result = step(controller, i * 0.02, **{source: 0})
        if result.stop_required:
            break
    assert result.reason == reason
    controller = armed()
    step(controller)
    assert step(controller, 0.04, **{source: 0.01}).reason == reason


def test_reference_rate_and_acceleration_bounded_through_reversal():
    controller = armed()
    actual = (0,)
    previous_velocity = 0
    for i in range(1, 31):
        requested = 0.05 if i < 15 else -0.05
        result = step(controller, i * 0.02, actual, (requested,))
        assert result.state == 'active'
        velocity = (result.position[0] - actual[0]) / 0.02
        assert abs(velocity) <= 0.1 + 1e-12
        assert abs(velocity - previous_velocity) <= 0.02 + 1e-12
        previous_velocity = velocity
        actual = result.position


def test_path_rejection_and_validator_exception_fail_closed():
    for guard in (lambda actual, candidate: False,
                  lambda actual, candidate: 1,
                  lambda actual, candidate: 1 / 0):
        controller = armed()
        controller.segment_allowed = guard
        assert step(controller).reason == 'segment_rejected'


@pytest.mark.parametrize('bounds,reason', [
    (replace(limits(), travel=(0.0001,)), 'travel_limit'),
    (replace(limits(), upper=(0.0001,)), 'joint_limit'),
])
def test_reference_limits_stop_before_publishing(bounds, reason):
    assert step(armed(bounds=bounds)).reason == reason


def test_new_session_initializes_from_current_feedback():
    old = armed()
    step(old, cancel=True)
    new = BoundedReference(limits(), lambda a, b: True)
    result = new.arm(10, [0.3], 10, lease_active=True)
    assert result.position == (0.3,)
    assert new.origin == (0.3,)


@pytest.mark.parametrize('changes', [
    {'max_step_s': math.nan}, {'travel': (0,)}, {'lower': ()},
    {'upper': (-2,)}, {'velocity': (math.inf,)},
])
def test_invalid_limits_rejected(changes):
    with pytest.raises(ValueError):
        replace(limits(), **changes)
