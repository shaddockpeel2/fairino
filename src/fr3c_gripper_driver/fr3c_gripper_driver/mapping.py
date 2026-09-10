"""Pure helpers for guarded aperture-to-PWM conversion."""

from dataclasses import dataclass
from bisect import bisect_right
import math


@dataclass(frozen=True)
class GripperCalibration:
    closed_position_m: float
    open_position_m: float
    operating_close_us: int
    operating_open_us: int
    electrical_min_us: int = 500
    electrical_max_us: int = 2500
    calibration_positions_m: tuple = ()
    calibration_pulses_us: tuple = ()

    def validate(self):
        values = (self.closed_position_m, self.open_position_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('gripper positions must be finite')
        if not 0.0 <= self.closed_position_m < self.open_position_m:
            raise ValueError('positions must satisfy 0 <= closed < open')
        if not (
            self.electrical_min_us <= self.operating_close_us
            < self.operating_open_us <= self.electrical_max_us
        ):
            raise ValueError(
                'pulse values must satisfy electrical_min <= close < open <= electrical_max')
        positions = tuple(float(value) for value in self.calibration_positions_m)
        pulses = tuple(int(value) for value in self.calibration_pulses_us)
        if bool(positions) != bool(pulses):
            raise ValueError('calibration position and pulse arrays must both be set')
        if positions:
            if len(positions) != len(pulses) or len(positions) < 2:
                raise ValueError('calibration arrays must have equal length >= 2')
            if not all(math.isfinite(value) for value in positions):
                raise ValueError('calibration positions must be finite')
            if any(b <= a for a, b in zip(positions, positions[1:])):
                raise ValueError('calibration positions must be strictly increasing')
            if any(b <= a for a, b in zip(pulses, pulses[1:])):
                raise ValueError('calibration pulses must be strictly increasing')
            if not math.isclose(positions[0], self.closed_position_m):
                raise ValueError('first calibration position must equal closed position')
            if not math.isclose(positions[-1], self.open_position_m):
                raise ValueError('last calibration position must equal open position')
            if pulses[0] != self.operating_close_us:
                raise ValueError('first calibration pulse must equal close pulse')
            if pulses[-1] != self.operating_open_us:
                raise ValueError('last calibration pulse must equal open pulse')

    def _points(self):
        if self.calibration_positions_m:
            return (
                tuple(float(value) for value in self.calibration_positions_m),
                tuple(int(value) for value in self.calibration_pulses_us),
            )
        return (
            (self.closed_position_m, self.open_position_m),
            (self.operating_close_us, self.operating_open_us),
        )

    @staticmethod
    def _interpolate(value, inputs, outputs):
        if value <= inputs[0]:
            return outputs[0]
        if value >= inputs[-1]:
            return outputs[-1]
        upper = bisect_right(inputs, value)
        lower = upper - 1
        fraction = ((value - inputs[lower])
                    / (inputs[upper] - inputs[lower]))
        return outputs[lower] + fraction * (outputs[upper] - outputs[lower])

    def position_to_pulse(self, position_m):
        self.validate()
        position_m = float(position_m)
        if not math.isfinite(position_m):
            raise ValueError('position must be finite')
        tolerance = 1e-9
        if not (self.closed_position_m - tolerance
                <= position_m <= self.open_position_m + tolerance):
            raise ValueError(
                f'position must be in {self.closed_position_m}..{self.open_position_m} m')
        positions, pulses = self._points()
        pulse = self._interpolate(position_m, positions, pulses)
        return int(round(pulse))

    def pulse_to_position(self, pulse_us):
        self.validate()
        pulse_us = int(pulse_us)
        if not self.operating_close_us <= pulse_us <= self.operating_open_us:
            raise ValueError('pulse is outside the operating range')
        positions, pulses = self._points()
        return self._interpolate(pulse_us, pulses, positions)


def build_pwm_frames(pulses_us, sample_rate_hz=100_000, frame_us=20_000,
                     signal_mask=0x80):
    """Build one 20 ms frame per requested pulse value."""
    if sample_rate_hz <= 0 or frame_us <= 0:
        raise ValueError('sample rate and frame duration must be positive')
    if signal_mask <= 0 or signal_mask > 0x80 or signal_mask & (signal_mask - 1):
        raise ValueError('signal mask must select one FT232H low GPIO pin')
    samples_per_frame = round(sample_rate_hz * frame_us / 1_000_000)
    payload = bytearray()
    for pulse_us in pulses_us:
        if not 0 < int(pulse_us) < frame_us:
            raise ValueError('pulse must be inside the PWM frame')
        high_samples = round(sample_rate_hz * int(pulse_us) / 1_000_000)
        payload.extend(bytes([signal_mask]) * high_samples)
        payload.extend(bytes(samples_per_frame - high_samples))
    return bytes(payload)


def advance_pulse(current_us, target_us, slew_us_per_s, frame_us=20_000):
    """Advance one PWM frame toward target with a bounded pulse slew rate."""
    if slew_us_per_s <= 0:
        return int(target_us)
    maximum_step = max(1, round(slew_us_per_s * frame_us / 1_000_000))
    difference = int(target_us) - int(current_us)
    if abs(difference) <= maximum_step:
        return int(target_us)
    return int(current_us) + (maximum_step if difference > 0 else -maximum_step)
