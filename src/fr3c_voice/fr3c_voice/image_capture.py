"""Convert one ROS RGB image into a bounded JPEG snapshot for Qwen3-VL."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time
import uuid

import cv2
import numpy as np


@dataclass(frozen=True)
class RgbFrame:
    width: int
    height: int
    step: int
    encoding: str
    is_bigendian: bool
    data: bytes
    ros_stamp_ns: int
    observed_monotonic: float
    received_wall_ns: int


@dataclass(frozen=True)
class SavedSnapshot:
    path: Path
    captured_at: str
    ros_stamp_ns: int
    width: int
    height: int
    source_width: int
    source_height: int


def frame_from_message(message, observed_monotonic: float | None = None,
                       received_wall_ns: int | None = None) -> RgbFrame:
    encoding = str(message.encoding).lower()
    if encoding not in {'rgb8', 'bgr8'}:
        raise ValueError(f'unsupported RGB encoding: {message.encoding}')
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    if width <= 0 or height <= 0 or step < width * 3:
        raise ValueError('invalid RGB image dimensions or stride')
    data = bytes(message.data)
    if len(data) < height * step:
        raise ValueError('RGB image buffer is shorter than height * step')
    stamp = message.header.stamp
    return RgbFrame(
        width=width, height=height, step=step, encoding=encoding,
        is_bigendian=bool(message.is_bigendian), data=data,
        ros_stamp_ns=int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
        observed_monotonic=(time.monotonic() if observed_monotonic is None
                            else float(observed_monotonic)),
        received_wall_ns=(time.time_ns() if received_wall_ns is None
                          else int(received_wall_ns)),
    )


def frame_to_bgr(frame: RgbFrame) -> np.ndarray:
    row_bytes = frame.width * 3
    if frame.step < row_bytes or len(frame.data) < frame.height * frame.step:
        raise ValueError('invalid cached RGB frame')
    array = np.ndarray(
        (frame.height, frame.width, 3), dtype=np.uint8,
        buffer=frame.data, strides=(frame.step, 3, 1))
    if frame.encoding == 'rgb8':
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(array)


def save_snapshot(frame: RgbFrame, directory: Path | str,
                  max_edge: int = 1280, jpeg_quality: int = 88,
                  max_saved: int = 20) -> SavedSnapshot:
    if max_edge < 1 or not 1 <= jpeg_quality <= 100 or max_saved < 1:
        raise ValueError('invalid snapshot settings')
    image = frame_to_bgr(frame)
    source_height, source_width = image.shape[:2]
    longest = max(source_width, source_height)
    if longest > max_edge:
        scale = max_edge / float(longest)
        image = cv2.resize(
            image,
            (max(1, round(source_width * scale)),
             max(1, round(source_height * scale))),
            interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(
        '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
    if not ok or encoded.size == 0:
        raise RuntimeError('OpenCV failed to encode the camera snapshot')
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True, mode=0o700)
    captured = datetime.fromtimestamp(frame.received_wall_ns / 1_000_000_000)
    name = f'camera-{captured:%Y%m%d_%H%M%S_%f}-{uuid.uuid4().hex[:8]}.jpg'
    destination = root / name
    destination.write_bytes(encoded.tobytes())
    try:
        destination.chmod(0o600)
    except OSError:
        pass
    captures = sorted(
        (path for path in root.glob('camera-*.jpg') if path.is_file()),
        key=lambda path: path.stat().st_mtime_ns, reverse=True)
    for stale in captures[max_saved:]:
        try:
            stale.unlink()
        except OSError:
            pass
    height, width = image.shape[:2]
    return SavedSnapshot(
        path=destination,
        captured_at=captured.astimezone().strftime('%Y-%m-%d %H:%M:%S %z'),
        ros_stamp_ns=frame.ros_stamp_ns,
        width=width, height=height,
        source_width=source_width, source_height=source_height)
