"""
Finite FT232H asynchronous-bitbang output for the YDI-J22 servo.

This packages the controller proven by the standalone ``gripper_ft232h.py``
test. A command is one complete, bounded PWM burst; there is no background
refill loop and no application-level chunk pacing.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path
import sys
import time


@dataclass(frozen=True)
class MoveResult:
    requested_pulse_us: float
    effective_pulse_us: int
    high_samples: int
    period_samples: int
    frames: int
    bytes_written: int
    actual_sample_rate_hz: float


def angle_to_pulse_us(
    angle_deg: float,
    minimum_us: int = 500,
    maximum_us: int = 2500,
) -> float:
    """Linearly map 0..180 degrees to the servo pulse range."""
    angle_deg = float(angle_deg)
    if not 0.0 <= angle_deg <= 180.0:
        raise ValueError('angle must be in the range 0..180 degrees')
    if not 0 < int(minimum_us) < int(maximum_us):
        raise ValueError('invalid pulse-width range')
    return minimum_us + (maximum_us - minimum_us) * angle_deg / 180.0


def effective_pulse_us(pulse_us: float, sample_rate_hz: float) -> int:
    """Return the pulse represented by the nearest bit-bang sample count."""
    if sample_rate_hz <= 0.0:
        raise ValueError('sample rate must be positive')
    high_samples = round(float(sample_rate_hz) * float(pulse_us) / 1_000_000)
    return round(high_samples * 1_000_000 / float(sample_rate_hz))


def load_gpio_controller(pyftdi_site_packages: str = ''):
    try:
        module = importlib.import_module('pyftdi.gpio')
    except ImportError:
        path = Path(pyftdi_site_packages).expanduser()
        if not path.is_dir():
            raise RuntimeError(
                'pyftdi is unavailable; configure pyftdi_site_packages')
        sys.path.insert(0, str(path))
        module = importlib.import_module('pyftdi.gpio')
    return module.GpioAsyncController


class GripperController:
    """One-session finite PWM controller matching the successful bench test."""

    def __init__(
        self,
        *,
        url: str = 'ftdi://ftdi:232h/1',
        signal_pin: int = 7,
        sample_rate_hz: int = 10_000,
        pwm_hz: int = 50,
        electrical_min_us: int = 500,
        electrical_max_us: int = 2500,
        post_write_delay_s: float = 0.05,
        pyftdi_site_packages: str = '',
        controller_factory=None,
        sleep=time.sleep,
    ):
        if not 0 <= int(signal_pin) <= 7:
            raise ValueError('signal_pin must be in 0..7')
        if int(sample_rate_hz) <= 0 or int(pwm_hz) <= 0:
            raise ValueError('sample and PWM rates must be positive')
        if int(sample_rate_hz) % int(pwm_hz):
            raise ValueError('sample rate must be an integer multiple of PWM rate')
        if not 0 < int(electrical_min_us) < int(electrical_max_us):
            raise ValueError('invalid pulse-width range')
        if float(post_write_delay_s) < 0.0:
            raise ValueError('post_write_delay_s must be non-negative')
        self.url = str(url)
        self.signal_mask = 1 << int(signal_pin)
        self.requested_sample_rate_hz = int(sample_rate_hz)
        self.pwm_hz = int(pwm_hz)
        self.electrical_min_us = int(electrical_min_us)
        self.electrical_max_us = int(electrical_max_us)
        self.post_write_delay_s = float(post_write_delay_s)
        self.pyftdi_site_packages = str(pyftdi_site_packages)
        self._controller_factory = controller_factory
        self._sleep = sleep
        self.gpio = None
        self.actual_sample_rate_hz = 0.0

    def __enter__(self):
        factory = self._controller_factory or load_gpio_controller(
            self.pyftdi_site_packages)
        self.gpio = factory()
        try:
            self.gpio.configure(
                self.url,
                direction=self.signal_mask,
                frequency=self.requested_sample_rate_hz,
                initial=0,
            )
            self.actual_sample_rate_hz = float(self.gpio.frequency)
            if self.actual_sample_rate_hz <= 0.0:
                raise RuntimeError('FT232H reported a non-positive sample rate')
        except Exception:
            self.gpio.close()
            raise
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        if self.gpio is None:
            return
        try:
            if getattr(self.gpio, 'is_connected', True):
                self.gpio.write(0)
        finally:
            self.gpio.close()

    @property
    def period_samples(self) -> int:
        return round(self.actual_sample_rate_hz / self.pwm_hz)

    def frame_for_pulse(self, pulse_us: float):
        if not self.electrical_min_us <= pulse_us <= self.electrical_max_us:
            raise ValueError(
                f'pulse must be in {self.electrical_min_us}..'
                f'{self.electrical_max_us} us')
        high_samples = round(
            self.actual_sample_rate_hz * float(pulse_us) / 1_000_000)
        if not 0 < high_samples < self.period_samples:
            raise RuntimeError('pulse cannot be represented at this sample rate')
        frame = (
            bytes([self.signal_mask]) * high_samples
            + bytes(self.period_samples - high_samples)
        )
        represented_us = round(
            high_samples * 1_000_000 / self.actual_sample_rate_hz)
        return frame, represented_us, high_samples

    def move_pulse(self, pulse_us: float, seconds: float = 0.8) -> MoveResult:
        if self.gpio is None:
            raise RuntimeError('controller is not configured')
        if float(seconds) <= 0.0:
            raise ValueError('duration must be positive')
        frame, represented_us, high_samples = self.frame_for_pulse(pulse_us)
        frames = max(1, round(float(seconds) * self.pwm_hz))
        payload = frame * frames
        self.gpio.write(payload)
        # Preserve the timing of the successful standalone implementation.
        self._sleep(frames / self.pwm_hz + self.post_write_delay_s)
        return MoveResult(
            requested_pulse_us=float(pulse_us),
            effective_pulse_us=represented_us,
            high_samples=high_samples,
            period_samples=self.period_samples,
            frames=frames,
            bytes_written=len(payload),
            actual_sample_rate_hz=self.actual_sample_rate_hz,
        )

    def move_angle(self, angle_deg: float, seconds: float = 0.8) -> MoveResult:
        return self.move_pulse(
            angle_to_pulse_us(
                angle_deg, self.electrical_min_us, self.electrical_max_us),
            seconds,
        )
