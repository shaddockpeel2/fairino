#!/usr/bin/env python3
"""Save one ROS Image without altering its numeric samples."""
import argparse
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Snapshot(Node):
    def __init__(self, topic):
        super().__init__('fr3c_image_snapshot')
        self.message = None
        self.create_subscription(Image, topic, self.receive, qos_profile_sensor_data)

    def receive(self, message):
        self.message = message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=5.0)
    options = parser.parse_args()
    rclpy.init()
    node = Snapshot(options.topic)
    deadline = time.monotonic() + options.timeout_s
    while rclpy.ok() and node.message is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.message is None:
        raise SystemExit('image timeout')
    message = node.message
    if message.encoding not in ('mono8', 'mono16', '16UC1'):
        raise SystemExit(f'unsupported encoding: {message.encoding}')
    dtype = np.dtype(('>' if message.is_bigendian else '<') +
                     ('u1' if message.encoding == 'mono8' else 'u2'))
    row_bytes = message.width * dtype.itemsize
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = rows[:, :row_bytes].copy().view(dtype).reshape(message.height, message.width)
    options.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(options.output), image)
    valid = image[np.isfinite(image)]
    low, high = np.percentile(valid, [1, 99])
    preview = np.clip((image.astype(np.float32) - low) * 255.0 /
                      max(float(high - low), 1.0), 0, 255).astype(np.uint8)
    preview_path = options.output.with_name(options.output.stem + '_adaptive.png')
    cv2.imwrite(str(preview_path), preview)
    print({
        'topic': options.topic,
        'encoding': message.encoding,
        'shape': list(image.shape),
        'min': int(valid.min()),
        'p50': float(np.percentile(valid, 50)),
        'p99': float(np.percentile(valid, 99)),
        'max': int(valid.max()),
        'output': str(options.output),
        'adaptive_output': str(preview_path),
    })
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
