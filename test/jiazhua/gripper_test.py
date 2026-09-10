#!/usr/bin/env python3
"""Safe bench test for a YDI-J22 gripper driven by an FT232H GPIO pin."""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from dataclasses import dataclass


FTDI_VENDOR_ID = "0403"
FT232H_PRODUCT_ID = "6014"
DEFAULT_URL = "ftdi://ftdi:232h/1"
EXECUTE_CONFIRMATION = "I_HAVE_EXTERNAL_5V_POWER"


@dataclass(frozen=True)
class PwmConfig:
    sample_rate_hz: int = 100_000
    frame_us: int = 20_000
    min_pulse_us: int = 500
    max_pulse_us: int = 2_500
    signal_mask: int = 0x80  # Bench wiring: FT232H ADBUS7 / AD7

    def validate(self) -> None:
        if self.sample_rate_hz <= 0:
            raise ValueError("sample rate must be positive")
        if not 0 < self.min_pulse_us < self.max_pulse_us < self.frame_us:
            raise ValueError("pulse limits must satisfy 0 < min < max < frame")
        if self.signal_mask <= 0 or self.signal_mask > 0x80 or (
            self.signal_mask & (self.signal_mask - 1)
        ):
            raise ValueError("signal mask must select exactly one low GPIO pin")


def angle_to_pulse_us(angle: float, config: PwmConfig) -> int:
    if not 0.0 <= angle <= 180.0:
        raise ValueError("angle must be in the range 0..180 degrees")
    span = config.max_pulse_us - config.min_pulse_us
    return round(config.min_pulse_us + span * angle / 180.0)


def build_pwm_frames(pulse_us: int, duration_s: float, config: PwmConfig) -> bytes:
    config.validate()
    if not config.min_pulse_us <= pulse_us <= config.max_pulse_us:
        raise ValueError(
            f"pulse must be in {config.min_pulse_us}..{config.max_pulse_us} us"
        )
    if not 0.02 <= duration_s <= 30.0:
        raise ValueError("duration must be in 0.02..30 seconds")

    samples_per_frame = round(config.sample_rate_hz * config.frame_us / 1_000_000)
    high_samples = round(config.sample_rate_hz * pulse_us / 1_000_000)
    frame_count = max(1, round(duration_s * 1_000_000 / config.frame_us))
    frame = bytes([config.signal_mask]) * high_samples
    frame += bytes(samples_per_frame - high_samples)
    return frame * frame_count


def find_ft232h_sysfs() -> list[str]:
    matches: list[str] = []
    for vendor_path in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        device_dir = os.path.dirname(vendor_path)
        product_path = os.path.join(device_dir, "idProduct")
        try:
            with open(vendor_path, encoding="ascii") as vendor_file:
                vendor = vendor_file.read().strip().lower()
            with open(product_path, encoding="ascii") as product_file:
                product = product_file.read().strip().lower()
        except (FileNotFoundError, PermissionError, OSError):
            continue
        if vendor == FTDI_VENDOR_ID and product == FT232H_PRODUCT_ID:
            matches.append(os.path.basename(device_dir))
    return sorted(matches)


def detect() -> int:
    devices = find_ft232h_sysfs()
    tty_devices = sorted(glob.glob("/dev/serial/by-id/*FTDI*"))
    if not devices:
        print("未检测到 FT232H (USB 0403:6014)。")
        return 1
    print(f"检测到 {len(devices)} 个 FT232H: USB sysfs {', '.join(devices)}")
    if tty_devices:
        for path in tty_devices:
            print(f"串口映射: {path} -> {os.path.realpath(path)}")
    print("检测通过；这只证明转换器已连接，不代表舵机供电符合要求。")
    return 0


def require_confirmation(value: str | None) -> None:
    if value != EXECUTE_CONFIRMATION:
        raise SystemExit(
            "拒绝输出 PWM：请用独立稳压 5V（建议持续 >=2A）给夹爪供电、与 "
            "FT232H 共地并让夹爪悬空。确认后追加：\n"
            f"  --execute {EXECUTE_CONFIRMATION}"
        )


def output_pwm(payload: bytes, config: PwmConfig, url: str) -> float:
    try:
        from pyftdi.gpio import GpioAsyncController
    except ImportError as exc:
        raise SystemExit(
            "缺少 pyftdi。请在本目录创建虚拟环境并安装：\n"
            "  python3 -m venv .venv\n"
            "  .venv/bin/pip install -r requirements.txt"
        ) from exc

    gpio = GpioAsyncController()
    try:
        gpio.configure(
            url,
            direction=config.signal_mask,
            initial=0,
            frequency=config.sample_rate_hz,
        )
        actual_rate = gpio.frequency
        gpio.write(payload)
        # Allow the final USB/FTDI buffer to drain before returning the pin to input.
        time.sleep(config.frame_us / 1_000_000 * 2)
        return actual_rate
    finally:
        gpio.close()


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="YDI-J22 + FT232H 三线舵机 PWM 安全测试"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("detect", help="只检测 FT232H，不输出信号")

    move = subparsers.add_parser("move", help="输出一段指定角度的 PWM")
    move.add_argument("--angle", type=float, required=True, help="0..180 度")
    move.add_argument("--duration", type=float, default=1.0, help="输出秒数，默认 1")
    move.add_argument("--min-us", type=int, default=500, help="0 度脉宽")
    move.add_argument("--max-us", type=int, default=2500, help="180 度脉宽")
    move.add_argument("--sample-rate", type=int, default=100_000)
    move.add_argument("--pin", type=int, default=7, choices=range(8), help="ADBUS 编号")
    move.add_argument("--url", default=DEFAULT_URL, help="PyFtdi 设备 URL")
    move.add_argument("--execute", metavar="CONFIRMATION")

    pulse = subparsers.add_parser("pulse", help="直接输出指定脉宽，适合标定")
    pulse.add_argument("--us", type=int, required=True, help="500..2500 微秒")
    pulse.add_argument("--duration", type=float, default=1.0, help="输出秒数，默认 1")
    pulse.add_argument("--sample-rate", type=int, default=100_000)
    pulse.add_argument("--pin", type=int, default=7, choices=range(8))
    pulse.add_argument("--url", default=DEFAULT_URL)
    pulse.add_argument("--execute", metavar="CONFIRMATION")

    sweep = subparsers.add_parser("sweep", help="依次测试 90→60→120→90 度")
    sweep.add_argument("--duration", type=float, default=0.7, help="每个位置的秒数")
    sweep.add_argument("--min-us", type=int, default=500)
    sweep.add_argument("--max-us", type=int, default=2500)
    sweep.add_argument("--sample-rate", type=int, default=100_000)
    sweep.add_argument("--pin", type=int, default=7, choices=range(8))
    sweep.add_argument("--url", default=DEFAULT_URL)
    sweep.add_argument("--execute", metavar="CONFIRMATION")
    return parser


def config_from_args(args: argparse.Namespace) -> PwmConfig:
    return PwmConfig(
        sample_rate_hz=args.sample_rate,
        min_pulse_us=getattr(args, "min_us", 500),
        max_pulse_us=getattr(args, "max_us", 2500),
        signal_mask=1 << args.pin,
    )


def main(argv: list[str] | None = None) -> int:
    args = make_parser().parse_args(argv)
    if args.command == "detect":
        return detect()

    require_confirmation(args.execute)
    config = config_from_args(args)
    if args.command == "pulse":
        payload = build_pwm_frames(args.us, args.duration, config)
        print(f"输出 {args.us} us，持续 {args.duration:g} s ...")
        actual_rate = output_pwm(payload, config, args.url)
        print(f"完成，FT232H 实际采样率 {actual_rate:.0f} samples/s")
        return 0

    angles = [args.angle] if args.command == "move" else [90.0, 60.0, 120.0, 90.0]
    for angle in angles:
        pulse_us = angle_to_pulse_us(angle, config)
        payload = build_pwm_frames(pulse_us, args.duration, config)
        print(f"输出 {angle:g}° ({pulse_us} us)，持续 {args.duration:g} s ...")
        actual_rate = output_pwm(payload, config, args.url)
        print(f"完成，FT232H 实际采样率 {actual_rate:.0f} samples/s")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已中止，信号脚已恢复为输入。", file=sys.stderr)
        raise SystemExit(130)
