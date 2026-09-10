"""Interactive live RGB-D review panel; camera-only and read-only."""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .rgbd import (depth_metres, image_array, map_color_pixel_to_depth,
                   stamp_seconds, validate_pair)
from .preprocess import (colorize_adaptive_depth, depth_discontinuity_mask,
                         roi_depth_statistics, temporal_consensus_depth)


def _text(canvas, line, value, color=(225, 225, 225)):
    cv2.putText(canvas, value, (18, 32 + line * 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.58, color, 1, cv2.LINE_AA)


class LiveReview(Node):
    def __init__(self, options):
        super().__init__('fr3c_rgbd_live_review')
        self.options = options
        self.latest = None
        self.error = 'waiting for synchronized RGB-D + CameraInfo'
        self.target = ((options.pixel_u, options.pixel_v)
                       if options.pixel_u is not None else None)
        self.arrivals = {'color': deque(maxlen=120), 'depth': deque(maxlen=120)}
        self.depth_history = deque(maxlen=300)
        self.depth_frames = deque(maxlen=options.temporal_frames)
        self.last_history_stamp = None
        self.last_report = {}
        ns = options.namespace.rstrip('/')
        self.subs = [Subscriber(self, typ, ns + topic, qos_profile=qos_profile_sensor_data)
                     for typ, topic in [(Image, '/color/image_raw'), (Image, '/depth/image_raw'),
                                        (CameraInfo, '/color/camera_info'),
                                        (CameraInfo, '/depth/camera_info')]]
        self.subs[0].registerCallback(lambda _msg: self.observe('color'))
        self.subs[1].registerCallback(lambda _msg: self.observe('depth'))
        self.sync = ApproximateTimeSynchronizer(self.subs, 20, options.max_skew_s)
        self.sync.registerCallback(self.pair)

    @staticmethod
    def _fps(values):
        return (len(values) - 1) / (values[-1] - values[0]) if len(values) > 1 else 0.0

    def observe(self, stream):
        self.arrivals[stream].append(time.monotonic())

    def pair(self, color, depth, ci, di):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            metres = validate_pair(color, depth, ci, di, now,
                                   self.options.max_age_s, self.options.max_skew_s)
            rgb = image_array(color)
            if color.encoding == 'rgb8':
                rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                rgb = rgb.copy()
            metres = metres.copy()
        except (ValueError, cv2.error) as exc:
            self.error = str(exc)
            return
        self.latest = (rgb, metres, color, depth, ci, di, now)
        self.depth_frames.append(metres)
        if self.target is None:
            self.target = (ci.width // 2, ci.height // 2)
        try:
            ud, vd = map_color_pixel_to_depth(ci, di, *self.target)
            sample = roi_depth_statistics(metres, ud, vd, self.options.roi_radius,
                                          self.options.min_depth_m, self.options.max_depth_m)
            history_key = (depth.header.stamp.sec, depth.header.stamp.nanosec)
            if sample['median_mm'] is not None and history_key != self.last_history_stamp:
                self.depth_history.append(sample['median_mm'])
                self.last_history_stamp = history_key
        except (ValueError, cv2.error):
            pass
        self.error = ''

    def mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and 0 <= x < 640 and 0 <= y < 480:
            if self.options.display_rotation == 180:
                x, y = 639 - x, 479 - y
            self.target = (x, y)
            self.depth_history.clear()
            self.last_history_stamp = None

    def _display_image(self, image):
        return cv2.rotate(image, cv2.ROTATE_180) if self.options.display_rotation == 180 else image

    def _display_point(self, point, width, height):
        x, y = point
        return ((width - 1 - x, height - 1 - y)
                if self.options.display_rotation == 180 else (x, y))

    def render(self):
        if self.latest is None:
            canvas = np.zeros((480, 800, 3), np.uint8)
            _text(canvas, 0, self.error, (0, 200, 255))
            return canvas
        rgb, depth_m, color, depth, ci, di, now = self.latest
        h, w = depth_m.shape
        u, v = self.target
        ud, vd = map_color_pixel_to_depth(ci, di, u, v)
        radius = self.options.roi_radius
        xd, yd = int(round(ud)), int(round(vd))
        raw_roi = roi_depth_statistics(depth_m, xd, yd, radius,
                                       self.options.min_depth_m, self.options.max_depth_m)
        median_mm = raw_roi['median_mm']
        p05_mm, p95_mm = raw_roi['p05_mm'], raw_roi['p95_mm']
        mad_mm = raw_roi['mad_sigma_mm']
        temporal_mm = (float(np.std(self.depth_history))
                       if len(self.depth_history) >= 5 else None)
        temporal_span_mm = (float(np.percentile(self.depth_history, 95) -
                                  np.percentile(self.depth_history, 5))
                            if len(self.depth_history) >= 5 else None)

        filtered_depth, stable, consensus = temporal_consensus_depth(
            list(self.depth_frames), self.options.min_depth_m, self.options.max_depth_m,
            self.options.temporal_min_valid, self.options.temporal_max_span_mm / 1000.0)
        stable_roi = roi_depth_statistics(filtered_depth, xd, yd, radius,
                                          self.options.min_depth_m, self.options.max_depth_m)
        # Keep the primary view equivalent to the Windows retest: current
        # unfiltered D2C depth with a per-frame 2%-98% adaptive color scale.
        depth_color, raw_visual_valid, adaptive_scale = colorize_adaptive_depth(depth_m)

        kc = np.asarray(ci.k, dtype=np.float64).reshape(3, 3)
        kd = np.asarray(di.k, dtype=np.float64).reshape(3, 3)
        dc = np.asarray(ci.d, dtype=np.float64)
        if ci.distortion_model in ('plumb_bob', 'rational_polynomial'):
            map_x, map_y = cv2.initUndistortRectifyMap(kc, dc, np.eye(3), kd, (w, h), cv2.CV_32FC1)
            rectified = cv2.remap(rgb, map_x, map_y, cv2.INTER_LINEAR)
        else:
            rectified = rgb.copy()
        alignment = rectified.copy()
        blended = cv2.addWeighted(rectified, 0.58, depth_color, 0.42, 0)
        alignment[raw_visual_valid] = blended[raw_visual_valid]
        edges = depth_discontinuity_mask(depth_m, self.options.min_depth_m,
                                         self.options.max_depth_m)
        if median_mm is not None:
            local_layer = np.abs(depth_m - median_mm / 1000.0) <= self.options.edge_band_m
            alignment[edges & local_layer] = (0, 0, 255)

        rgb = self._display_image(rgb)
        depth_color = self._display_image(depth_color)
        alignment = self._display_image(alignment)
        color_point = self._display_point((u, v), rgb.shape[1], rgb.shape[0])
        depth_point = self._display_point((xd, yd), w, h)
        for pane, point in ((rgb, color_point), (depth_color, depth_point),
                            (alignment, depth_point)):
            px, py = point
            cv2.rectangle(pane, (px-radius, py-radius), (px+radius, py+radius), (0, 255, 0), 2)
            cv2.drawMarker(pane, point, (255, 255, 255), cv2.MARKER_CROSS, 18, 1)
        cv2.putText(rgb, 'COLOR (click ROI)', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, (255,255,255), 2)
        scale_label = ('NO DEPTH' if adaptive_scale['low_mm'] is None else
                       f'{adaptive_scale["low_mm"]:.0f}-{adaptive_scale["high_mm"]:.0f}mm')
        cv2.putText(depth_color, f'RAW D2C DEPTH ADAPTIVE {scale_label}',
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .62, (255,255,255), 2)
        cv2.putText(alignment, 'ALIGNED RGB + RAW D2C / LOCAL EDGES', (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .62, (255,255,255), 2)

        metrics = np.zeros_like(rgb)
        skew_ms = abs(stamp_seconds(color) - stamp_seconds(depth)) * 1000.0
        age_ms = max(now - stamp_seconds(color), now - stamp_seconds(depth)) * 1000.0
        k_delta = float(np.max(np.abs(np.asarray(ci.k) - np.asarray(di.k))))
        expected = self.options.expected_distance_mm
        bias = median_mm - expected if median_mm is not None and expected is not None else None
        valid_fraction = raw_roi['valid_fraction']
        stable_fraction = stable_roi['valid_fraction']
        quality_ready = len(self.depth_history) == self.depth_history.maxlen
        status_ok = (quality_ready and median_mm is not None and valid_fraction >= 0.95 and
                     temporal_span_mm is not None and temporal_span_mm <= 5.0 and
                     skew_ms <= self.options.max_skew_s * 1000.0 and age_ms <= self.options.max_age_s * 1000.0)
        rows = [
            ('LIVE CONTRACT: ', 'PASS' if status_ok else 'CHECK'),
            ('RGB: ', f'{color.width}x{color.height} {color.encoding}  {self._fps(self.arrivals["color"]):.1f} FPS'),
            ('DEPTH: ', f'{depth.width}x{depth.height} {depth.encoding} (mm)  {self._fps(self.arrivals["depth"]):.1f} FPS'),
            ('FRAME: ', color.header.frame_id),
            ('K max delta: ', f'{k_delta:.6g}'),
            ('RGB D: ', f'{ci.distortion_model} {np.asarray(ci.d)[:5]}'),
            ('DEPTH D: ', f'{di.distortion_model} {np.asarray(di.d)[:5]}'),
            ('timestamp skew: ', f'{skew_ms:.1f} ms'),
            ('message age: ', f'{age_ms:.1f} ms'),
            ('ROI raw->D2C px: ', f'({u},{v}) -> ({ud:.1f},{vd:.1f})'),
            ('ROI median Z: ', 'NO VALID DEPTH' if median_mm is None else f'{median_mm:.1f} mm'),
            ('ROI valid raw/stable: ', f'{valid_fraction*100:.1f}% / {stable_fraction*100:.1f}%'),
            ('ROI P05..P95: ', '-' if median_mm is None else f'{p05_mm:.1f} .. {p95_mm:.1f} mm'),
            ('ROI robust noise: ', '-' if mad_mm is None else f'{mad_mm:.2f} mm MAD-sigma'),
            ('temporal P95 span: ', '-' if temporal_span_mm is None else
             f'{temporal_span_mm:.2f} mm (sigma {temporal_mm:.2f}, n={len(self.depth_history)}/300)'),
            ('expected / bias: ', 'not set (quality only)' if expected is None else f'{expected:.1f} / {bias:+.1f} mm'),
            ('preprocess: ', f'{consensus["frames"]} frames, <= {self.options.temporal_max_span_mm:.0f} mm span'),
            ('display: ', f'rotation {self.options.display_rotation} deg (coordinates stay optical)'),
            ('controls: ', 'click target | S snapshot | Q quit'),
        ]
        for line, (label, value) in enumerate(rows):
            color_value = (80, 230, 80) if line == 0 and status_ok else ((0, 180, 255) if line == 0 else (225,225,225))
            _text(metrics, line, label + value, color_value)

        self.last_report = {
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'contract_pass': status_ok,
            'color': {'width': color.width, 'height': color.height, 'encoding': color.encoding,
                      'frame_id': color.header.frame_id},
            'depth': {'width': depth.width, 'height': depth.height, 'encoding': depth.encoding,
                      'unit': 'millimetres for 16UC1'},
            'camera_info': {'color_k': list(ci.k), 'color_d': list(ci.d),
                            'depth_k': list(di.k), 'depth_d': list(di.d),
                            'distortion_models': [ci.distortion_model, di.distortion_model],
                            'max_k_delta': k_delta},
            'timing': {'skew_ms': skew_ms, 'age_ms': age_ms},
            'roi': {'color_pixel': [u, v], 'depth_pixel': [ud, vd],
                    'radius_px': radius, 'median_mm': median_mm,
                    'valid_fraction': valid_fraction,
                    'stable_valid_fraction': stable_fraction, 'p05_mm': p05_mm,
                    'p95_mm': p95_mm, 'robust_noise_mm': mad_mm,
                    'temporal_sigma_mm': temporal_mm, 'temporal_p95_span_mm': temporal_span_mm,
                    'quality_ready_300_frames': quality_ready,
                    'unique_depth_samples': len(self.depth_history),
                    'expected_distance_mm': expected, 'bias_mm': bias},
            'preprocess': {**consensus,
                           'policy': 'no hole filling; latest sample required; stationary target only'},
            'visualization': {'source': 'current unfiltered D2C depth',
                              'adaptive_percentiles': [2, 98], **adaptive_scale},
            'display_rotation_deg': self.options.display_rotation,
            'limitations': ['Visual edge overlay requires a scene with shared color/depth edges.',
                            'Absolute accuracy requires a measured lens-to-target ground truth.',
                            'This viewer emits no robot command and no base_link transform.']}
        return np.vstack((np.hstack((rgb, depth_color)), np.hstack((alignment, metrics))))

    def save(self, canvas):
        folder = Path(self.options.output)
        folder.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(folder / 'live_review.png'), canvas)
        if self.last_report:
            (folder / 'live_report.json').write_text(json.dumps(self.last_report, indent=2) + '\n')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='/camera')
    parser.add_argument('--output', default='/tmp/fr3c_camera_live')
    parser.add_argument('--roi-radius', type=int, default=20)
    parser.add_argument('--pixel-u', type=int)
    parser.add_argument('--pixel-v', type=int)
    parser.add_argument('--min-depth-m', type=float, default=0.3)
    parser.add_argument('--max-depth-m', type=float, default=1.2)
    parser.add_argument('--max-age-s', type=float, default=0.5)
    parser.add_argument('--max-skew-s', type=float, default=0.06)
    parser.add_argument('--expected-distance-mm', type=float)
    parser.add_argument('--autosave-seconds', type=float, default=2.0)
    parser.add_argument('--temporal-frames', type=int, default=5)
    parser.add_argument('--temporal-min-valid', type=float, default=0.6)
    parser.add_argument('--temporal-max-span-mm', type=float, default=15.0)
    parser.add_argument('--edge-band-m', type=float, default=0.15)
    parser.add_argument('--display-rotation', type=int, choices=(0, 180), default=0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration-s', type=float, default=0.0)
    options = parser.parse_args(args)
    if ((options.pixel_u is None) != (options.pixel_v is None) or
            (options.pixel_u is not None and
             not (0 <= options.pixel_u < 640 and 0 <= options.pixel_v < 480)) or
            options.roi_radius < 1 or not 0 < options.min_depth_m < options.max_depth_m or
            options.temporal_frames < 1 or not 0 < options.temporal_min_valid <= 1 or
            options.temporal_max_span_mm < 0 or options.edge_band_m <= 0 or
            (options.headless and options.duration_s <= 0)):
        parser.error('invalid ROI or depth range')
    rclpy.init()
    node = LiveReview(options)
    window = 'FR3-C Gemini Pro RGB-D live review'
    if not options.headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 1280, 960)
        cv2.setMouseCallback(window, node.mouse)
    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    last_save = 0.0
    try:
        deadline = time.monotonic() + options.duration_s if options.duration_s > 0 else None
        while rclpy.ok():
            canvas = node.render()
            if not options.headless:
                cv2.imshow(window, canvas)
            if (node.latest is not None and options.autosave_seconds > 0 and
                    time.monotonic() - last_save >= options.autosave_seconds):
                node.save(canvas)
                last_save = time.monotonic()
            if deadline is not None and time.monotonic() >= deadline:
                break
            if options.headless:
                time.sleep(0.02)
                continue
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                node.save(canvas)
    except KeyboardInterrupt:
        pass
    finally:
        if node.latest is not None:
            node.save(node.render())
        if not options.headless:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
