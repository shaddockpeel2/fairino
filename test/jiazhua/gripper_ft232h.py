#!/usr/bin/env python3
"""Standalone YDI-J22 test using one bounded FT232H AD7 PWM burst."""

from __future__ import annotations

import argparse
import time
from collections.abc import Iterable

from pyftdi.gpio import GpioAsyncController


FTDI_URL = "ftdi://ftdi:232h/1"
AD7 = 0x80


def angle_to_pulse_us(angle: float, minimum_us: int, maximum_us: int) -> float:
    """Linearly map 0..180 degrees to the configured pulse-width range."""
    if not 0 <= angle <= 180:
        raise ValueError(f"angle {angle} is outside 0..180 degrees")
    return minimum_us + (maximum_us - minimum_us) * angle / 180.0


class GripperController:
    """Configure, emit a finite waveform, pull AD7 low, then close FT232H."""

    def __init__(
        self,
        url: str = FTDI_URL,
        pin: int = AD7,
        sample_rate: int = 10_000,
        pwm_hz: int = 50,
        minimum_us: int = 500,
        maximum_us: int = 2_500,
    ) -> None:
        if sample_rate % pwm_hz:
            raise ValueError("sample rate must be an integer multiple of PWM rate")
        if not 1 <= minimum_us < maximum_us:
            raise ValueError("invalid pulse-width range")
        self.url = url
        self.pin = pin
        self.requested_sample_rate = sample_rate
        self.pwm_hz = pwm_hz
        self.minimum_us = minimum_us
        self.maximum_us = maximum_us
        self.gpio = GpioAsyncController()
        self.sample_rate = 0.0

    def __enter__(self) -> "GripperController":
        try:
            self.gpio.configure(
                self.url,
                direction=self.pin,
                frequency=self.requested_sample_rate,
                initial=0,
            )
            self.sample_rate = self.gpio.frequency
        except Exception:
            self.gpio.close()
            raise
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        try:
            if self.gpio.is_connected:
                self.gpio.write(0)
        finally:
            self.gpio.close()

    @property
    def period_samples(self) -> int:
        return round(self.sample_rate / self.pwm_hz)

    def frame(self, angle: float) -> tuple[bytes, float, int]:
        pulse_us = angle_to_pulse_us(angle, self.minimum_us, self.maximum_us)
        high_samples = round(self.sample_rate * pulse_us / 1_000_000)
        if not 0 < high_samples < self.period_samples:
            raise RuntimeError("pulse cannot be represented at this sample rate")
        frame = (
            bytes([self.pin]) * high_samples
            + bytes([0]) * (self.period_samples - high_samples)
        )
        return frame, pulse_us, high_samples

    def move(self, angle: float, seconds: float) -> tuple[float, int, int]:
        if seconds <= 0:
            raise ValueError("duration must be positive")
        frame, pulse_us, high_samples = self.frame(angle)
        frames = max(1, round(seconds * self.pwm_hz))
        self.gpio.write(frame * frames)
        time.sleep(frames / self.pwm_hz + 0.05)
        return pulse_us, high_samples, frames

    def run_sequence(self, angles: Iterable[float], seconds: float) -> None:
        for angle in angles:
            pulse_us, high_samples, frames = self.move(angle, seconds)
            print(
                f"angle={angle:6.1f} deg  pulse={pulse_us:7.1f} us  "
                f"high_samples={high_samples:2d}  frames={frames}"
            )


def parse_angles(text: str) -> list[float]:
    try:
        angles = [float(item.strip()) for item in text.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("angles must be comma-separated numbers") from exc
    if not angles:
        raise argparse.ArgumentTypeError("at least one angle is required")
    if any(angle not in (31.0, 70.0) for angle in angles):
        raise argparse.ArgumentTypeError(
            "this project enables only 31 degrees (grasp) and 70 degrees (release)"
        )
    return angles


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control the YDI-J22 through FT232H AD7 finite-burst PWM."
    )
    parser.add_argument(
        "--angles",
        type=parse_angles,
        required=True,
        help="31 for the 26 mm grasp state, or 70 for release",
    )
    parser.add_argument(
        "--seconds", type=float, default=0.8, help="PWM burst per angle (default: 0.8)"
    )
    parser.add_argument("--url", default=FTDI_URL)
    parser.add_argument("--sample-rate", type=int, default=10_000)
    parser.add_argument("--pwm-hz", type=int, default=50)
    parser.add_argument("--min-us", type=int, default=500)
    parser.add_argument("--max-us", type=int, default=2_500)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    print(
        f"FT232H URL={args.url}, pin=AD7, PWM={args.pwm_hz} Hz, "
        f"pulse={args.min_us}..{args.max_us} us"
    )
    try:
        with GripperController(
            url=args.url,
            sample_rate=args.sample_rate,
            pwm_hz=args.pwm_hz,
            minimum_us=args.min_us,
            maximum_us=args.max_us,
        ) as gripper:
            print(f"actual sample rate={gripper.sample_rate:.1f} samples/s")
            gripper.run_sequence(args.angles, args.seconds)
    except KeyboardInterrupt:
        print("interrupted; AD7 returned low")
        return 130
    print("sequence complete; AD7 returned low")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
