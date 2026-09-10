"""Audit a raw or registered depth stream without changing its pixels."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import warnings

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .preprocess import colorize_adaptive_depth
from .rgbd import depth_metres, stamp_seconds


class RawDepthAudit(Node):
    def __init__(self, options):
        super().__init__('fr3c_raw_depth_audit')
        self.options = options
        self.seen = 0
        self.arrivals = []
        self.stamps = []
        self.ages = []
        self.full_valid = []
        self.rois = []
        self.last_depth = None
        self.roi_pixel = None
        self.info = None
        self.message_layout = None
        self.byte_decode_matches = []
        self.failures = Counter()
        ns = options.namespace.rstrip('/')
        self.create_subscription(Image, ns + '/depth/image_raw', self.depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, ns + '/depth/camera_info', self.camera_info,
                                 qos_profile_sensor_data)

    def camera_info(self, message):
        self.info = message

    def depth(self, message):
        self.seen += 1
        if self.seen <= self.options.warmup_frames or len(self.rois) >= self.options.frames:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            if message.encoding not in ('16UC1', '32FC1'):
                raise ValueError('unsupported_depth_encoding')
            if stamp_seconds(message) <= 0 or now - stamp_seconds(message) > 0.5:
                raise ValueError('stale_depth')
            depth = depth_metres(message)
            if message.encoding == '16UC1':
                byte_order = '>' if message.is_bigendian else '<'
                row_bytes = message.width * np.dtype(byte_order + 'u2').itemsize
                if message.step < row_bytes or len(message.data) < message.step * message.height:
                    raise ValueError('invalid_depth_buffer_layout')
                byte_rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
                    message.height, message.step)
                direct_mm = byte_rows[:, :row_bytes].copy().view(byte_order + 'u2').reshape(
                    message.height, message.width)
                decoded_mm = np.rint(depth * 1000.0).astype(np.uint16)
                self.byte_decode_matches.append(bool(np.array_equal(decoded_mm, direct_mm)))
            else:
                direct_mm = None
            if self.message_layout is None:
                self.message_layout = {
                    'encoding': message.encoding,
                    'is_bigendian': bool(message.is_bigendian),
                    'step_bytes': int(message.step),
                    'data_bytes': len(message.data),
                    'expected_min_step_bytes': int(message.width * (2 if message.encoding == '16UC1' else 4)),
                    'direct_uint16_min_mm': (int(direct_mm.min()) if direct_mm is not None else None),
                    'direct_uint16_max_mm': (int(direct_mm.max()) if direct_mm is not None else None),
                }
            if self.info is None or (self.info.width, self.info.height) != (message.width, message.height):
                raise ValueError('missing_or_mismatched_camera_info')
            u = self.options.pixel_u if self.options.pixel_u is not None else message.width // 2
            v = self.options.pixel_v if self.options.pixel_v is not None else message.height // 2
            self.roi_pixel = [u, v]
            radius = self.options.roi_radius
            x0, x1 = max(0, u - radius), min(message.width, u + radius)
            y0, y1 = max(0, v - radius), min(message.height, v + radius)
            roi = depth[y0:y1, x0:x1].copy()
            if roi.size == 0:
                raise ValueError('empty_roi')
        except (ValueError, cv2.error) as exc:
            self.failures[str(exc)] += 1
            return
        self.arrivals.append(time.monotonic())
        self.stamps.append(stamp_seconds(message))
        self.ages.append(now - stamp_seconds(message))
        valid = np.isfinite(depth) & (depth > 0)
        self.full_valid.append(float(valid.mean()))
        self.rois.append(roi)
        self.last_depth = depth

    def report(self):
        stack = np.asarray(self.rois, dtype=np.float32)
        valid = np.isfinite(stack) & (stack > 0)
        masked = np.where(valid, stack, np.nan)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', category=RuntimeWarning)
            frame_medians = np.nanmedian(masked, axis=(1, 2)) if stack.size else np.array([])
            pixel_std = np.nanstd(masked, axis=0) if stack.size else np.array([])
        finite_medians = frame_medians[np.isfinite(frame_medians)]
        stable_pixels = ((valid.sum(axis=0) >= len(stack) * 0.8)
                         if stack.size else np.zeros((0, 0), dtype=bool))
        stable_std = pixel_std[stable_pixels] if pixel_std.size else np.array([])
        gaps = np.diff(self.arrivals)
        stamp_steps = np.diff(self.stamps)
        temporal_span = (float(np.percentile(finite_medians, 95) -
                               np.percentile(finite_medians, 5)) * 1000.0
                         if finite_medians.size else None)
        median_valid = float(valid.mean()) if valid.size else 0.0
        valid_values_mm = stack[valid] * 1000.0 if valid.size else np.array([])
        checks = {
            'collected_requested_frames': len(self.rois) == self.options.frames,
            'expected_dimensions': bool(self.info) and
                                   (self.info.width, self.info.height) ==
                                   (self.options.expected_width, self.options.expected_height),
            'expected_frame_id': bool(self.info) and
                                 self.info.header.frame_id == self.options.expected_frame_id,
            'median_roi_valid_at_least_95pct': median_valid >= 0.95,
            'temporal_p95_span_at_most_5mm': temporal_span is not None and temporal_span <= 5.0,
            'monotonic_timestamps': not bool(np.any(stamp_steps <= 0)),
        }
        return {
            'pass': all(checks.values()),
            'checks': checks,
            'frames': len(self.rois),
            'failures': dict(self.failures),
            'stream': {
                'width': self.info.width if self.info else None,
                'height': self.info.height if self.info else None,
                'frame_id': self.info.header.frame_id if self.info else None,
                'k': list(self.info.k) if self.info else None,
                'fps': ((len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])
                        if len(self.arrivals) > 1 else 0.0),
                'max_gap_s': float(np.max(gaps)) if gaps.size else None,
                'max_age_s': float(np.max(self.ages)) if self.ages else None,
                'message_layout': self.message_layout,
                'direct_byte_decode_equal_all_frames': (all(self.byte_decode_matches)
                                                        if self.byte_decode_matches else None),
                'full_frame_valid_fraction_median': (float(np.median(self.full_valid))
                                                     if self.full_valid else None),
            },
            'roi': {
                'pixel': self.roi_pixel,
                'size': [self.options.roi_radius * 2, self.options.roi_radius * 2],
                'valid_fraction': median_valid,
                'depth_median_mm': (float(np.median(finite_medians)) * 1000.0
                                    if finite_medians.size else None),
                'temporal_p95_span_mm': temporal_span,
                'frame_median_std_mm': (float(np.std(finite_medians)) * 1000.0
                                        if finite_medians.size else None),
                'stable_pixel_fraction': (float(stable_pixels.mean()) if stable_pixels.size else 0.0),
                'per_pixel_temporal_std_median_mm': (float(np.median(stable_std)) * 1000.0
                                                     if stable_std.size else None),
                'per_pixel_temporal_std_p95_mm': (float(np.percentile(stable_std, 95)) * 1000.0
                                                  if stable_std.size else None),
                'valid_depth_percentiles_mm': ({
                    str(percentile): float(np.percentile(valid_values_mm, percentile))
                    for percentile in (1, 5, 50, 95, 99)
                } if valid_values_mm.size else {}),
            },
            'limitations': ['Pixel values are audited; geometric RGB-D alignment is not independently tested.',
                            'The ROI must be placed on one static matte surface.',
                            'No measured ground truth means absolute bias is not tested.'],
        }


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='/camera')
    parser.add_argument('--output', required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--warmup-frames', type=int, default=30)
    parser.add_argument('--roi-radius', type=int, default=50)
    parser.add_argument('--pixel-u', type=int)
    parser.add_argument('--pixel-v', type=int)
    parser.add_argument('--timeout-s', type=float, default=15.0)
    parser.add_argument('--expected-width', type=int, default=640)
    parser.add_argument('--expected-height', type=int, default=400)
    parser.add_argument('--expected-frame-id', default='camera_depth_optical_frame')
    options = parser.parse_args(args)
    if (options.frames < 1 or options.warmup_frames < 0 or options.roi_radius < 1 or
            options.expected_width < 1 or options.expected_height < 1 or
            not options.expected_frame_id or
            (options.pixel_u is None) != (options.pixel_v is None)):
        parser.error('invalid frame or ROI parameters')
    rclpy.init()
    node = RawDepthAudit(options)
    deadline = time.monotonic() + options.timeout_s
    try:
        while rclpy.ok() and len(node.rois) < options.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    output = Path(options.output)
    output.mkdir(parents=True, exist_ok=True)
    report = node.report()
    (output / 'raw_depth_report.json').write_text(json.dumps(report, indent=2) + '\n')
    if node.last_depth is not None:
        depth_mm = np.clip(np.nan_to_num(node.last_depth) * 1000.0, 0, 65535).astype(np.uint16)
        preview, _, scale = colorize_adaptive_depth(node.last_depth)
        cv2.imwrite(str(output / 'raw_depth_mm.png'), depth_mm)
        cv2.imwrite(str(output / 'raw_depth_adaptive.png'), preview)
        report['visualization_scale_mm'] = scale
        (output / 'raw_depth_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if report['pass'] else 1)


if __name__ == '__main__':
    main()
