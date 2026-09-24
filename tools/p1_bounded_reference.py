"""Offline-only prototype: bounded position reference and fail-closed lifecycle.

This is not a ROS controller or a certified stop implementation. A caller must
honour stop_required by cancelling the downstream target and stopping the plant.
No reference is returned after stop/fault; holding the last target is NOT safe.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple
    following_error: tuple
    travel: tuple
    command_timeout_s: float = 0.06
    feedback_timeout_s: float = 0.04
    max_step_s: float = 0.02

    def __post_init__(self):
        size = len(self.lower)
        if size == 0 or any(len(v) != size for v in (
                self.upper, self.velocity, self.acceleration,
                self.following_error, self.travel)):
            raise ValueError('limit dimensions differ')
        if not all(math.isfinite(x) for vector in (
                self.lower, self.upper, self.velocity, self.acceleration,
                self.following_error, self.travel) for x in vector):
            raise ValueError('nonfinite limits')
        if any(lo >= hi for lo, hi in zip(self.lower, self.upper)):
            raise ValueError('invalid joint bounds')
        if any(x <= 0 for vector in (
                self.velocity, self.acceleration, self.following_error,
                self.travel) for x in vector):
            raise ValueError('limits must be positive')
        if any(not math.isfinite(x) or x <= 0 for x in (
                self.command_timeout_s, self.feedback_timeout_s,
                self.max_step_s)):
            raise ValueError('invalid time limits')


@dataclass(frozen=True)
class Result:
    state: str
    reason: str
    position: tuple | None = None
    stop_required: bool = False


class BoundedReference:
    """One lease/session, explicit arm, latched faults, no automatic restart.

    All times must be from one monotonic clock. segment_allowed must check the
    path from actual to proposed reference (including collision/workspace) and
    return literal True. It is deliberately mandatory, with no permissive default.
    """

    def __init__(self, limits, segment_allowed, mode='integrated'):
        if mode not in ('integrated', 'feedback_step'):
            raise ValueError('unknown mode')
        if not callable(segment_allowed):
            raise ValueError('a segment validator is required')
        self.limits = limits
        self.segment_allowed = segment_allowed
        self.mode = mode
        self.state = 'idle'
        self.reason = ''
        self.reference = None
        self.origin = None
        self.velocity = None
        self.last_time = None
        self.last_command_stamp = None
        self.last_feedback_stamp = None

    def _vector(self, values):
        try:
            result = tuple(float(v) for v in values)
        except (TypeError, ValueError):
            return None
        return result if (len(result) == len(self.limits.lower)
                          and all(math.isfinite(v) for v in result)) else None

    def _stop(self, reason, fault=True):
        self.state = 'fault' if fault else 'stopped'
        self.reason = reason
        self.reference = None
        self.velocity = None
        return Result(self.state, reason, stop_required=True)

    def arm(self, now, actual, feedback_stamp, *, lease_active):
        if self.state != 'idle':
            raise RuntimeError('Create a new session after verified stop and release')
        actual = self._vector(actual)
        if (actual is None or not math.isfinite(now)
                or not math.isfinite(feedback_stamp)
                or not 0 <= now - feedback_stamp <= self.limits.feedback_timeout_s
                or not lease_active):
            return self._stop('invalid_arm')
        if any(not lo <= q <= hi for q, lo, hi in zip(
                actual, self.limits.lower, self.limits.upper)):
            return self._stop('joint_limit')
        if not self._allowed(actual, actual):
            return self._stop('segment_rejected')
        self.reference = self.origin = actual
        self.velocity = (0.0,) * len(actual)
        self.last_time = now
        self.last_feedback_stamp = feedback_stamp
        self.state = 'active'
        return Result('active', 'armed', actual)

    def _allowed(self, actual, candidate):
        try:
            return self.segment_allowed(actual, candidate) is True
        except Exception:
            return False

    def step(self, now, actual, feedback_stamp, requested_velocity,
             command_stamp, *, lease_active, cancel=False):
        if self.state != 'active':
            return Result(self.state, self.reason, stop_required=True)
        if cancel or not lease_active:
            return self._stop('cancel' if cancel else 'lease_lost', fault=False)
        if not all(math.isfinite(t) for t in (now, feedback_stamp, command_stamp)):
            return self._stop('invalid_time')
        dt = now - self.last_time
        if not 0 < dt <= self.limits.max_step_s + 1e-12:
            return self._stop('invalid_period')
        if (feedback_stamp < self.last_feedback_stamp
                or not 0 <= now - feedback_stamp <= self.limits.feedback_timeout_s):
            return self._stop('feedback_stale_or_reordered')
        if ((self.last_command_stamp is not None
             and command_stamp < self.last_command_stamp)
                or not 0 <= now - command_stamp <= self.limits.command_timeout_s):
            return self._stop('command_stale_or_reordered')
        actual = self._vector(actual)
        requested = self._vector(requested_velocity)
        if actual is None or requested is None:
            return self._stop('invalid_vector')
        if any(not lo <= q <= hi for q, lo, hi in zip(
                actual, self.limits.lower, self.limits.upper)):
            return self._stop('joint_limit')
        if all(v == 0 for v in requested):
            return self._stop('zero_command', fault=False)
        if any(abs(v) > limit for v, limit in zip(requested, self.limits.velocity)):
            return self._stop('velocity_limit')
        velocity = tuple(old + max(-a * dt, min(a * dt, new - old))
                         for old, new, a in zip(
                             self.velocity, requested, self.limits.acceleration))
        base = self.reference if self.mode == 'integrated' else actual
        candidate = tuple(q + v * dt for q, v in zip(base, velocity))
        if any(abs(ref - q) > limit for ref, q, limit in zip(
                candidate, actual, self.limits.following_error)):
            return self._stop('following_error')
        if any(abs(q - origin) > limit for q, origin, limit in zip(
                candidate, self.origin, self.limits.travel)):
            return self._stop('travel_limit')
        if any(not lo <= q <= hi for q, lo, hi in zip(
                candidate, self.limits.lower, self.limits.upper)):
            return self._stop('joint_limit')
        if not self._allowed(actual, candidate):
            return self._stop('segment_rejected')
        self.reference = candidate
        self.velocity = velocity
        self.last_time = now
        self.last_command_stamp = command_stamp
        self.last_feedback_stamp = feedback_stamp
        return Result('active', '', candidate)
