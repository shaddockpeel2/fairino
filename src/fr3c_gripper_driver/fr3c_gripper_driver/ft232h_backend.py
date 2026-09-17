"""ROS-facing finite-command FT232H backend for the YDI-J22 gripper."""

import threading
import time

from .ft232h_servo import GripperController, angle_to_pulse_us


class Ft232hPwmBackend:
    """Wrap the proven standalone controller behind the existing backend API."""

    def __init__(self, *, url, sample_rate_hz, frame_us, signal_pin,
                 command_duration_s=0.8, post_write_delay_s=0.05,
                 electrical_min_us=500, electrical_max_us=2500,
                 pyftdi_site_packages='', controller_factory=None,
                 sleep=time.sleep):
        self.url = str(url)
        self.sample_rate_hz = int(sample_rate_hz)
        self.frame_us = int(frame_us)
        if self.frame_us <= 0 or 1_000_000 % self.frame_us:
            raise ValueError('frame_us must divide one second exactly')
        self.pwm_hz = 1_000_000 // self.frame_us
        self.signal_pin = int(signal_pin)
        self.command_duration_s = float(command_duration_s)
        self.post_write_delay_s = float(post_write_delay_s)
        self.electrical_min_us = int(electrical_min_us)
        self.electrical_max_us = int(electrical_max_us)
        self.pyftdi_site_packages = str(pyftdi_site_packages)
        self._controller_factory = controller_factory
        self._sleep = sleep
        if self.command_duration_s <= 0.0:
            raise ValueError('command_duration_s must be positive')
        self._condition = threading.Condition()
        self._active = False
        self._current_us = None
        self._target_us = None
        self._error = None
        self._actual_rate_hz = None
        self._frames_written = 0
        self._bytes_written = 0
        self._recovery_count = 0

    @property
    def current_us(self):
        with self._condition:
            return self._current_us

    @property
    def target_us(self):
        with self._condition:
            return self._target_us

    @property
    def error(self):
        with self._condition:
            return self._error

    @property
    def actual_rate_hz(self):
        with self._condition:
            return self._actual_rate_hz

    @property
    def frames_written(self):
        with self._condition:
            return self._frames_written

    @property
    def bytes_written(self):
        with self._condition:
            return self._bytes_written

    @property
    def recovery_count(self):
        with self._condition:
            return self._recovery_count

    def command(self, pulse_us):
        return self._run_command(
            round(float(pulse_us)),
            lambda controller: controller.move_pulse(
                pulse_us, self.command_duration_s),
        )

    def command_angle(self, angle_deg):
        """Execute the exact angle mapping used by the standalone test."""
        requested_pulse_us = angle_to_pulse_us(
            angle_deg, self.electrical_min_us, self.electrical_max_us)
        return self._run_command(
            round(requested_pulse_us),
            lambda controller: controller.move_angle(
                angle_deg, self.command_duration_s),
        )

    def _run_command(self, requested_pulse_us, move):
        with self._condition:
            if self._active:
                raise RuntimeError('FT232H command is already active')
            if self._error is not None:
                self._error = None
                self._recovery_count += 1
            self._active = True
            self._target_us = int(requested_pulse_us)
        try:
            with GripperController(
                url=self.url,
                signal_pin=self.signal_pin,
                sample_rate_hz=self.sample_rate_hz,
                pwm_hz=self.pwm_hz,
                electrical_min_us=self.electrical_min_us,
                electrical_max_us=self.electrical_max_us,
                post_write_delay_s=self.post_write_delay_s,
                pyftdi_site_packages=self.pyftdi_site_packages,
                controller_factory=self._controller_factory,
                sleep=self._sleep,
            ) as controller:
                result = move(controller)
            with self._condition:
                self._current_us = result.effective_pulse_us
                self._target_us = result.effective_pulse_us
                self._actual_rate_hz = result.actual_sample_rate_hz
                self._frames_written += result.frames
                self._bytes_written += result.bytes_written
            return result.effective_pulse_us
        except Exception as error:
            with self._condition:
                self._error = f'{type(error).__name__}: {error}'
            raise
        finally:
            with self._condition:
                self._active = False

    def freeze(self):
        # command() returns only after the bounded PWM burst and AD7-low write.
        return None

    def close(self):
        # Each command owns and closes its own FT232H session.
        return None
