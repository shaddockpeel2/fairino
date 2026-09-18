from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from fr3c_voice.image_capture import frame_from_message, save_snapshot


def message(encoding='rgb8'):
    # Two RGB pixels plus two bytes of row padding; conversion must honor step.
    return SimpleNamespace(
        width=2, height=1, step=8, encoding=encoding, is_bigendian=False,
        data=bytes([255, 0, 0, 0, 255, 0, 99, 99]),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=34)))


def test_ros_rgb_frame_is_saved_as_decodable_bounded_jpeg(tmp_path):
    frame = frame_from_message(
        message(), observed_monotonic=10.0, received_wall_ns=1_700_000_000_000_000_000)
    snapshot = save_snapshot(frame, tmp_path, max_edge=1, jpeg_quality=90, max_saved=2)
    decoded = cv2.imread(str(snapshot.path))
    assert decoded is not None
    assert decoded.shape[:2] == (1, 1)
    assert snapshot.ros_stamp_ns == 12_000_000_034
    assert snapshot.source_width == 2
    assert snapshot.source_height == 1


def test_invalid_encoding_and_buffer_fail_closed():
    with pytest.raises(ValueError, match='unsupported'):
        frame_from_message(message('mono8'))
    broken = message()
    broken.data = b'123'
    with pytest.raises(ValueError, match='shorter'):
        frame_from_message(broken)
