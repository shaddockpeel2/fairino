"""ROS-independent state machine and bounded 2-D control law."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple


SUPPORTED_COLORS = ('red', 'yellow', 'blue', 'green')


class Phase(str, Enum):
    ACQUIRING = 'ACQUIRING'
    TRACKING = 'TRACKING'
    CENTER_HOLD = 'CENTER_HOLD'
    LOST_HOLD = 'LOST_HOLD'
    STOPPING = 'STOPPING'


class Termination(str, Enum):
    OK = 'OK'
    BUSY = 'BUSY'
    TARGET_NOT_FOUND = 'TARGET_NOT_FOUND'
    TARGET_NOT_UNIQUE = 'TARGET_NOT_UNIQUE'
    TARGET_STALE = 'TARGET_STALE'
    CONTROL_UNAVAILABLE = 'CONTROL_UNAVAILABLE'
    SERVO_FAULT = 'SERVO_FAULT'
    SAFETY_LIMIT = 'SAFETY_LIMIT'
    CANCELED = 'CANCELED'
    INTERNAL_ERROR = 'INTERNAL_ERROR'


@dataclass(frozen=True)
class ControllerConfig:
    control_rate_hz: float = 50.0
    gain_x_mps: float = 0.020
    gain_y_mps: float = 0.020
    axis_sign_x: float = 1.0
    axis_sign_y: float = 1.0
    max_speed_mps: float = 0.005
    max_acceleration_mps2: float = 0.030
    enter_half_width_px: float = 40.0
    enter_half_height_px: float = 30.0
    exit_half_width_px: float = 55.0
    exit_half_height_px: float = 45.0
    enter_frames: int = 3
    exit_frames: int = 2
    observation_stale_s: float = 0.40
    lost_timeout_s: float = 1.0
    acquisition_timeout_s: float = 5.0
    max_relative_xy_m: float = 0.08

    def validate(self) -> None:
        positive = {
            'control_rate_hz': self.control_rate_hz,
            'gain_x_mps': self.gain_x_mps,
            'gain_y_mps': self.gain_y_mps,
            'max_speed_mps': self.max_speed_mps,
            'max_acceleration_mps2': self.max_acceleration_mps2,
            'enter_half_width_px': self.enter_half_width_px,
            'enter_half_height_px': self.enter_half_height_px,
            'exit_half_width_px': self.exit_half_width_px,
            'exit_half_height_px': self.exit_half_height_px,
            'observation_stale_s': self.observation_stale_s,
            'lost_timeout_s': self.lost_timeout_s,
            'acquisition_timeout_s': self.acquisition_timeout_s,
            'max_relative_xy_m': self.max_relative_xy_m,
        }
        if any(not math.isfinite(value) or value <= 0.0
               for value in positive.values()):
            raise ValueError('controller_limits_must_be_finite_and_positive')
        if self.enter_frames < 1 or self.exit_frames < 1:
            raise ValueError('hysteresis_frame_counts_must_be_positive')
        if (self.exit_half_width_px <= self.enter_half_width_px
                or self.exit_half_height_px <= self.enter_half_height_px):
            raise ValueError('exit_region_must_enclose_enter_region')
        if self.axis_sign_x not in (-1.0, 1.0):
            raise ValueError('axis_sign_x_must_be_plus_or_minus_one')
        if self.axis_sign_y not in (-1.0, 1.0):
            raise ValueError('axis_sign_y_must_be_plus_or_minus_one')


@dataclass(frozen=True)
class Observation:
    sample_id: int
    color: str
    valid: bool
    reason: str
    target_u_px: float = 0.0
    target_v_px: float = 0.0
    center_u_px: float = 0.0
    center_v_px: float = 0.0
    image_width_px: float = 0.0
    image_height_px: float = 0.0
    age_at_receive_s: float = 0.0
    received_monotonic_s: float = 0.0

    def age_s(self, now_s: float) -> float:
        return max(0.0, self.age_at_receive_s +
                   max(0.0, now_s - self.received_monotonic_s))

    def geometry_valid(self) -> bool:
        values = (
            self.target_u_px, self.target_v_px,
            self.center_u_px, self.center_v_px,
            self.image_width_px, self.image_height_px,
        )
        return (all(math.isfinite(value) for value in values)
                and self.image_width_px > 0.0
                and self.image_height_px > 0.0)


@dataclass(frozen=True)
class ControlOutput:
    phase: Phase
    detail: str
    command_x_mps: float
    command_y_mps: float
    error_x_px: float
    error_y_px: float
    target_u_px: float
    target_v_px: float
    observation_age_s: float
    centered: bool
    done: bool = False
    termination: Optional[Termination] = None


class TrackingController:
    """Deterministic tracking state machine driven by a monotonic clock."""

    def __init__(self, color: str, config: ControllerConfig,
                 started_s: float, max_duration_s: float = 0.0):
        color = color.strip().lower()
        if color not in SUPPORTED_COLORS:
            raise ValueError('invalid_target_color')
        config.validate()
        if not math.isfinite(started_s):
            raise ValueError('invalid_start_time')
        if not math.isfinite(max_duration_s) or max_duration_s < 0.0:
            raise ValueError('invalid_max_duration')
        self.color = color
        self.config = config
        self.started_s = float(started_s)
        self.max_duration_s = float(max_duration_s)
        self.phase = Phase.ACQUIRING
        self._phase_before_loss = Phase.TRACKING
        self._loss_started_s = None
        self._last_step_s = float(started_s)
        self._last_sample_id = None
        self._enter_count = 0
        self._exit_count = 0
        self._command = (0.0, 0.0)
        self._estimated_xy = (0.0, 0.0)
        self._last_error = (0.0, 0.0)
        self._last_target = (0.0, 0.0)

    @property
    def estimated_displacement_xy_m(self) -> Tuple[float, float]:
        return self._estimated_xy

    def stop(self, termination: Termination, detail: str,
             observation_age_s: float = -1.0) -> ControlOutput:
        self.phase = Phase.STOPPING
        self._command = (0.0, 0.0)
        return self._output(
            detail, observation_age_s, done=True, termination=termination)

    def update(self, observation: Optional[Observation],
               now_s: float, *, measured_xy_m=None) -> ControlOutput:
        if not math.isfinite(now_s):
            return self.stop(Termination.INTERNAL_ERROR, 'invalid_clock')
        if self.phase == Phase.STOPPING:
            return self._output('already_stopping', -1.0, done=True,
                                termination=Termination.INTERNAL_ERROR)

        elapsed_s = max(0.0, now_s - self.started_s)
        if self.max_duration_s > 0.0 and elapsed_s >= self.max_duration_s:
            return self.stop(Termination.OK, 'maximum_duration_reached')

        dt_s = max(0.0, now_s - self._last_step_s)
        # A delayed executor wakeup must not cause a large one-cycle command
        # jump or a correspondingly large dead-reckoned displacement.
        dt_s = min(dt_s, 0.1)
        self._last_step_s = now_s

        valid, reason, age_s = self._classify_observation(observation, now_s)
        if not valid:
            return self._handle_invalid(reason, age_s, now_s, elapsed_s)

        is_new = observation.sample_id != self._last_sample_id
        if is_new:
            self._last_sample_id = observation.sample_id

        error_x = observation.target_u_px - observation.center_u_px
        error_y = observation.target_v_px - observation.center_v_px
        self._last_error = (error_x, error_y)
        self._last_target = (
            observation.target_u_px, observation.target_v_px)
        inside_enter = (
            abs(error_x) <= self.config.enter_half_width_px
            and abs(error_y) <= self.config.enter_half_height_px)
        outside_exit = (
            abs(error_x) > self.config.exit_half_width_px
            or abs(error_y) > self.config.exit_half_height_px)

        if self.phase == Phase.ACQUIRING:
            self.phase = Phase.TRACKING
            self._phase_before_loss = Phase.TRACKING
            self._loss_started_s = None
        elif self.phase == Phase.LOST_HOLD:
            if (self._phase_before_loss == Phase.CENTER_HOLD
                    and not outside_exit):
                self.phase = Phase.CENTER_HOLD
            else:
                self.phase = Phase.TRACKING
            self._loss_started_s = None

        if self.phase == Phase.CENTER_HOLD:
            if is_new:
                self._exit_count = self._exit_count + 1 if outside_exit else 0
            if self._exit_count >= self.config.exit_frames:
                self.phase = Phase.TRACKING
                self._exit_count = 0
                self._enter_count = 0
            else:
                self._command = (0.0, 0.0)
                return self._output('target_inside_exit_region', age_s)

        if is_new:
            self._enter_count = self._enter_count + 1 if inside_enter else 0
        if self._enter_count >= self.config.enter_frames:
            self.phase = Phase.CENTER_HOLD
            self._enter_count = 0
            self._exit_count = 0
            self._command = (0.0, 0.0)
            return self._output('center_entry_confirmed', age_s)

        if inside_enter:
            self._command = (0.0, 0.0)
            return self._output('confirming_center_entry', age_s)

        desired = self._desired_velocity(observation, error_x, error_y)
        command = self._acceleration_limited(desired, dt_s)
        candidate_xy = (
            self._estimated_xy[0] + command[0] * dt_s,
            self._estimated_xy[1] + command[1] * dt_s,
        )
        displacement = (math.hypot(*candidate_xy) if measured_xy_m is None
                        else measured_xy_m)
        if (not math.isfinite(displacement) or displacement < 0.0
                or displacement >= self.config.max_relative_xy_m):
            return self.stop(
                Termination.SAFETY_LIMIT,
                'estimated_relative_xy_limit_reached', age_s)
        self._estimated_xy = candidate_xy
        self._command = command
        return self._output('tracking_target', age_s)

    def _classify_observation(self, observation, now_s):
        if observation is None:
            return False, 'no_observation', math.inf
        age_s = observation.age_s(now_s)
        if observation.color != self.color:
            return False, 'target_color_mismatch', age_s
        if age_s > self.config.observation_stale_s:
            return False, 'target_stale', age_s
        if not observation.valid:
            return False, observation.reason or 'target_not_found', age_s
        if not observation.geometry_valid():
            return False, 'invalid_observation_geometry', age_s
        return True, 'valid', age_s

    def _handle_invalid(self, reason, age_s, now_s, elapsed_s):
        self._command = (0.0, 0.0)
        self._enter_count = 0
        self._exit_count = 0
        if self.phase == Phase.ACQUIRING:
            if elapsed_s >= self.config.acquisition_timeout_s:
                return self.stop(self._termination_for_reason(reason),
                                 f'acquisition_timeout:{reason}', age_s)
            return self._output(f'waiting_for_target:{reason}', age_s)

        if self.phase != Phase.LOST_HOLD:
            self._phase_before_loss = self.phase
            self.phase = Phase.LOST_HOLD
            self._loss_started_s = now_s
        if (self._loss_started_s is not None
                and now_s - self._loss_started_s >= self.config.lost_timeout_s):
            return self.stop(self._termination_for_reason(reason),
                             f'target_loss_timeout:{reason}', age_s)
        return self._output(f'holding_for_target_recovery:{reason}', age_s)

    @staticmethod
    def _termination_for_reason(reason):
        if 'not_unique' in reason:
            return Termination.TARGET_NOT_UNIQUE
        if 'not_found' in reason:
            return Termination.TARGET_NOT_FOUND
        return Termination.TARGET_STALE

    def _desired_velocity(self, observation, error_x, error_y):
        velocity_x = (self.config.axis_sign_x * self.config.gain_x_mps
                      * error_x / observation.image_width_px)
        velocity_y = (self.config.axis_sign_y * self.config.gain_y_mps
                      * error_y / observation.image_height_px)
        magnitude = math.hypot(velocity_x, velocity_y)
        if magnitude > self.config.max_speed_mps:
            scale = self.config.max_speed_mps / magnitude
            velocity_x *= scale
            velocity_y *= scale
        return velocity_x, velocity_y

    def _acceleration_limited(self, desired, dt_s):
        delta_x = desired[0] - self._command[0]
        delta_y = desired[1] - self._command[1]
        delta = math.hypot(delta_x, delta_y)
        maximum_delta = self.config.max_acceleration_mps2 * dt_s
        if delta > maximum_delta and delta > 0.0:
            scale = maximum_delta / delta
            delta_x *= scale
            delta_y *= scale
        return self._command[0] + delta_x, self._command[1] + delta_y

    def _output(self, detail, age_s, done=False, termination=None):
        return ControlOutput(
            phase=self.phase,
            detail=detail,
            command_x_mps=self._command[0],
            command_y_mps=self._command[1],
            error_x_px=self._last_error[0],
            error_y_px=self._last_error[1],
            target_u_px=self._last_target[0],
            target_v_px=self._last_target[1],
            observation_age_s=age_s if math.isfinite(age_s) else -1.0,
            centered=self.phase == Phase.CENTER_HOLD,
            done=done,
            termination=termination,
        )
