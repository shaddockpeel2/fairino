"""Continuous open-loop FT232H PWM backend.

This backend deliberately opens the USB device only after the first command.
No PWM is emitted merely by starting the ROS node.
"""

import importlib
from pathlib import Path
import sys
import threading
import time

from .mapping import advance_pulse, build_pwm_frames


class Ft232hPwmBackend:
    def __init__(self, *, url, sample_rate_hz, frame_us, signal_pin,
                 chunk_frames, slew_us_per_s, pyftdi_site_packages=''):
        self.url = url
        self.sample_rate_hz = int(sample_rate_hz)
        self.frame_us = int(frame_us)
        self.signal_mask = 1 << int(signal_pin)
        self.chunk_frames = int(chunk_frames)
        self.slew_us_per_s = float(slew_us_per_s)
        self.pyftdi_site_packages = str(pyftdi_site_packages)
        if self.chunk_frames < 2:
            raise ValueError('chunk_frames must be >= 2')
        self._condition = threading.Condition()
        self._thread = None
        self._stop = False
        self._current_us = None
        self._target_us = None
        self._error = None
        self._actual_rate_hz = None
        self._frames_written = 0
        self._bytes_written = 0

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

    def command(self, pulse_us):
        # The asynchronous bit-bang sample period is the real pulse
        # resolution (10 us at 100 kHz). Keep state/diagnostics consistent
        # with the waveform that is physically emitted.
        pulse_us = int(round(
            round(int(pulse_us) * self.sample_rate_hz / 1_000_000)
            * 1_000_000 / self.sample_rate_hz))
        with self._condition:
            if self._error:
                raise RuntimeError(self._error)
            if self._thread is None:
                # The first goal is an intentional motion request. Start at its
                # target because the physical servo position is not observable.
                self._current_us = pulse_us
                self._target_us = pulse_us
                self._thread = threading.Thread(
                    target=self._run, name='ft232h_pwm', daemon=True)
                self._thread.start()
            else:
                self._target_us = pulse_us
            self._condition.notify_all()
        return pulse_us

    def freeze(self):
        with self._condition:
            if self._current_us is not None:
                self._target_us = self._current_us

    def close(self):
        with self._condition:
            self._stop = True
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)

    def _load_controller(self):
        try:
            module = importlib.import_module('pyftdi.gpio')
        except ImportError:
            path = Path(self.pyftdi_site_packages).expanduser()
            if not path.is_dir():
                raise RuntimeError(
                    'pyftdi is unavailable; configure pyftdi_site_packages')
            sys.path.insert(0, str(path))
            module = importlib.import_module('pyftdi.gpio')
        return module.GpioAsyncController

    def _run(self):
        gpio = None
        try:
            controller = self._load_controller()
            gpio = controller()
            gpio.configure(
                self.url,
                direction=self.signal_mask,
                initial=0,
                frequency=self.sample_rate_hz,
            )
            with self._condition:
                self._actual_rate_hz = float(gpio.frequency)
            while True:
                with self._condition:
                    if self._stop:
                        break
                    current = self._current_us
                    target = self._target_us
                pulses = []
                for _ in range(self.chunk_frames):
                    current = advance_pulse(
                        current, target, self.slew_us_per_s, self.frame_us)
                    pulses.append(current)
                payload = build_pwm_frames(
                    pulses,
                    sample_rate_hz=self.sample_rate_hz,
                    frame_us=self.frame_us,
                    signal_mask=self.signal_mask,
                )
                started = time.monotonic()
                gpio.write(payload)

                # A USB bulk write may return before the FT232H has clocked all
                # samples onto the pin. Pace the producer by the represented
                # waveform duration so old goals cannot accumulate in host/USB
                # buffers and emerge seconds later as unexpected motion. If
                # write() itself blocks, only wait for the remaining duration.
                waveform_seconds = len(payload) / self._actual_rate_hz
                remaining = waveform_seconds - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
                with self._condition:
                    self._current_us = current
                    self._frames_written += len(pulses)
                    self._bytes_written += len(payload)
        except Exception as error:  # hardware/USB errors must reach diagnostics
            with self._condition:
                self._error = f'{type(error).__name__}: {error}'
        finally:
            if gpio is not None:
                try:
                    gpio.close()
                except Exception:
                    pass
