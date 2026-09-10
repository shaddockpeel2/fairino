"""Collect a fixed RGB ROI for 300-frame grasp-distance depth quality."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .rgbd import map_color_pixel_to_depth, stamp_seconds, validate_pair


class ROIQuality(Node):
    def __init__(self, options):
        super().__init__('fr3c_rgbd_roi_quality')
        self.options = options
        self.samples = []
        self.failures = {}
        self.started = time.monotonic()
        ns = options.namespace.rstrip('/')
        self.subs = [Subscriber(self, typ, ns + topic, qos_profile=qos_profile_sensor_data)
                     for typ, topic in [(Image, '/color/image_raw'), (Image, '/depth/image_raw'),
                                        (CameraInfo, '/color/camera_info'),
                                        (CameraInfo, '/depth/camera_info')]]
        self.sync = ApproximateTimeSynchronizer(self.subs, 20, options.max_skew_s)
        self.sync.registerCallback(self.pair)

    def pair(self, color, depth, ci, di):
        if len(self.samples) >= self.options.frames:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            metres = validate_pair(color, depth, ci, di, now,
                                   self.options.max_age_s, self.options.max_skew_s)
            ud, vd = map_color_pixel_to_depth(ci, di, self.options.pixel_u, self.options.pixel_v)
            x, y, r = int(round(ud)), int(round(vd)), self.options.roi_radius
            roi = metres[max(0, y-r):min(depth.height, y+r+1),
                         max(0, x-r):min(depth.width, x+r+1)]
            valid = np.isfinite(roi) & (roi >= self.options.min_depth_m) & (roi <= self.options.max_depth_m)
            values = roi[valid] * 1000.0
            if values.size:
                median = float(np.median(values))
                spatial_span = float(np.percentile(values, 95) - np.percentile(values, 5))
            else:
                median, spatial_span = None, None
            self.samples.append({'median_mm': median, 'valid_fraction': float(valid.mean()),
                                 'spatial_p95_span_mm': spatial_span,
                                 'skew_ms': abs(stamp_seconds(color)-stamp_seconds(depth))*1000.0,
                                 'age_ms': max(now-stamp_seconds(color), now-stamp_seconds(depth))*1000.0,
                                 'depth_pixel': [ud, vd]})
        except (ValueError, cv2.error) as exc:
            key = str(exc)
            self.failures[key] = self.failures.get(key, 0) + 1

    def report(self):
        valid_samples = [sample for sample in self.samples if sample['median_mm'] is not None]
        medians = np.asarray([sample['median_mm'] for sample in valid_samples])
        fractions = np.asarray([sample['valid_fraction'] for sample in self.samples])
        spatial = np.asarray([sample['spatial_p95_span_mm'] for sample in valid_samples])
        skews = np.asarray([sample['skew_ms'] for sample in self.samples])
        ages = np.asarray([sample['age_ms'] for sample in self.samples])
        expected = self.options.expected_distance_mm
        measured = float(np.median(medians)) if medians.size else None
        temporal_span = (float(np.percentile(medians, 95) - np.percentile(medians, 5))
                         if medians.size else None)
        checks = {
            'collected_requested_frames': len(self.samples) == self.options.frames,
            'valid_depth_every_frame': len(valid_samples) == len(self.samples) and bool(self.samples),
            'median_roi_valid_at_least_95pct': bool(fractions.size) and float(np.median(fractions)) >= 0.95,
            'temporal_p95_span_at_most_5mm': temporal_span is not None and temporal_span <= 5.0,
            'timing_contract': bool(skews.size) and float(np.max(skews)) <= self.options.max_skew_s*1000.0
                               and float(np.max(ages)) <= self.options.max_age_s*1000.0,
        }
        return {
            'pass': all(checks.values()), 'checks': checks, 'requested_frames': self.options.frames,
            'collected_frames': len(self.samples), 'pair_failures': self.failures,
            'color_pixel': [self.options.pixel_u, self.options.pixel_v],
            'depth_pixel': valid_samples[-1]['depth_pixel'] if valid_samples else None,
            'roi_radius_px': self.options.roi_radius,
            'median_depth_mm': measured,
            'expected_distance_mm': expected,
            'bias_mm': measured-expected if measured is not None and expected is not None else None,
            'roi_valid_fraction': {
                'median': float(np.median(fractions)) if fractions.size else None,
                'p05': float(np.percentile(fractions, 5)) if fractions.size else None},
            'temporal_depth_mm': {
                'p05': float(np.percentile(medians, 5)) if medians.size else None,
                'p50': measured,
                'p95': float(np.percentile(medians, 95)) if medians.size else None,
                'p95_span': temporal_span,
                'stddev': float(np.std(medians)) if medians.size else None},
            'spatial_p95_span_mm': {
                'median': float(np.median(spatial)) if spatial.size else None,
                'p95': float(np.percentile(spatial, 95)) if spatial.size else None},
            'timing_ms': {
                'skew_p50': float(np.median(skews)) if skews.size else None,
                'skew_p95': float(np.percentile(skews, 95)) if skews.size else None,
                'max_age': float(np.max(ages)) if ages.size else None},
            'limitations': ['The operator must place the ROI fully inside one static planar target.',
                            'Bias is unavailable without measured lens-to-plane ground truth.',
                            'Camera-frame test only; no hand-eye or robot command.']}


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='/camera')
    parser.add_argument('--output', required=True)
    parser.add_argument('--pixel-u', type=float, required=True)
    parser.add_argument('--pixel-v', type=float, required=True)
    parser.add_argument('--roi-radius', type=int, default=20)
    parser.add_argument('--frames', type=int, default=300)
    parser.add_argument('--timeout-s', type=float, default=45.0)
    parser.add_argument('--min-depth-m', type=float, default=0.1)
    parser.add_argument('--max-depth-m', type=float, default=1.2)
    parser.add_argument('--max-age-s', type=float, default=0.5)
    parser.add_argument('--max-skew-s', type=float, default=0.06)
    parser.add_argument('--expected-distance-mm', type=float)
    options = parser.parse_args(args)
    if (options.frames < 1 or options.roi_radius < 1 or
            not 0 < options.min_depth_m < options.max_depth_m):
        parser.error('invalid frame, ROI, or depth parameters')
    rclpy.init()
    node = ROIQuality(options)
    deadline = time.monotonic() + options.timeout_s
    try:
        while rclpy.ok() and len(node.samples) < options.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    result = node.report()
    folder = Path(options.output)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'roi_quality.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if result['pass'] else 1)


if __name__ == '__main__':
    main()
