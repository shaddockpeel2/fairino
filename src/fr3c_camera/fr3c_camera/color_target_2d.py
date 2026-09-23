"""Low-latency, color-only 2-D cube observations for visual servoing.

This node intentionally has no robot, controller, or motion-command interface.
The image-processing and continuity helpers are ROS-independent so the fail-
closed observation policy can be tested with synthetic images.
"""

from array import array
from collections import deque
from dataclasses import dataclass
import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

from fr3c_camera.cube_perception import find_color_candidates
from fr3c_camera.rgbd import image_array, stamp_seconds


SUPPORTED_COLORS = ('red', 'yellow', 'blue', 'green')


@dataclass(frozen=True)
class TargetDetection:
    """Result of one frame's geometry-filtered target-color search."""

    candidate: object
    candidate_count: int
    reason: str


def detect_color_target(bgr, target_color, color_intervals, **candidate_options):
    """Return a target only when exactly one qualified region is present."""
    if target_color not in SUPPORTED_COLORS or target_color not in color_intervals:
        raise ValueError('invalid_target_color')
    matches = find_color_candidates(
        bgr, {target_color: color_intervals[target_color]},
        saturation_min=candidate_options.pop('saturation_min', 80),
        value_min=candidate_options.pop('value_min', 35),
        morphology_size=candidate_options.pop('morphology_size', 3),
        **candidate_options)
    count = len(matches)
    if count != 1:
        return TargetDetection(
            candidate=None,
            candidate_count=count,
            reason='target_not_found' if count == 0 else 'target_not_unique')
    return TargetDetection(matches[0][1], 1, 'candidate_unique')


class TargetContinuity:
    """Require consecutive, increasing, spatially continuous observations."""

    def __init__(self, required_samples=2, maximum_jump_px=45.0):
        if required_samples < 1:
            raise ValueError('required_samples_must_be_positive')
        if not math.isfinite(maximum_jump_px) or maximum_jump_px <= 0:
            raise ValueError('maximum_jump_px_must_be_positive')
        self.required_samples = int(required_samples)
        self.maximum_jump_px = float(maximum_jump_px)
        self._samples = deque(maxlen=self.required_samples)
        self._last_seen_stamp = None
        self._transient_reject_used = False

    @property
    def sample_count(self):
        return len(self._samples)

    def reset(self):
        self._samples.clear()
        self._transient_reject_used = False

    def reject(self, *, transient=False):
        """Keep one detection glitch after lock; repeated loss clears it."""
        if (transient and not self._transient_reject_used
                and len(self._samples) == self.required_samples):
            self._transient_reject_used = True
            return
        self.reset()

    def update(self, stamp_s, center_px):
        center = np.asarray(center_px, dtype=np.float64)
        if center.shape != (2,) or not np.isfinite(center).all():
            self.reset()
            return False, 'invalid_center'
        if not math.isfinite(stamp_s):
            self.reset()
            return False, 'invalid_timestamp'
        if self._last_seen_stamp is not None and stamp_s <= self._last_seen_stamp:
            self.reset()
            return False, 'timestamp_not_increasing'
        if (self._transient_reject_used and self._last_seen_stamp is not None
                and stamp_s - self._last_seen_stamp > 0.25):
            self.reset()
        self._last_seen_stamp = float(stamp_s)
        self._transient_reject_used = False
        if self._samples:
            jump = float(np.linalg.norm(center - self._samples[-1][1]))
            if jump > self.maximum_jump_px:
                self.reset()
                self._samples.append((float(stamp_s), center))
                return False, 'center_jump'
        self._samples.append((float(stamp_s), center))
        if len(self._samples) < self.required_samples:
            return False, 'collecting_stable_samples'
        return True, 'stable_target'


def candidate_bbox(candidate):
    """Return an axis-aligned integer (x, y, width, height) bounding box."""
    x, y, width, height = cv2.boundingRect(candidate.contour)
    return int(x), int(y), int(width), int(height)


def candidate_quality(candidate):
    """Conservative 0..1 score retaining individual metrics in diagnostics."""
    aspect = float(candidate.aspect_ratio)
    aspect_score = min(aspect, 1.0 / aspect) if aspect > 0 else 0.0
    return float(np.clip(min(
        aspect_score, candidate.solidity, candidate.rectangularity), 0.0, 1.0))


class ColorTarget2D(Node):
    """Publish fail-closed observations of one configured colored cube."""

    def __init__(self):
        super().__init__('color_target_2d')
        self._declare_parameters()
        self._read_parameters()
        self._lock = threading.Lock()
        self._camera_info = None
        self._last_frame_monotonic = None
        self._last_frame_stamp = None
        self._stream_stale = True
        self._last_log_signature = None
        # The servo observation must prefer the newest image over queued old
        # frames. A depth-five sensor queue was observed to build latency under
        # load even though the camera itself continued at 10 FPS.
        self._sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self._center_pub = self.create_publisher(PointStamped, '~/center', 10)
        self._bbox_pub = self.create_publisher(RegionOfInterest, '~/bbox', 10)
        self._detection_pub = self.create_publisher(Detection2D, '~/detection', 10)
        self._valid_pub = self.create_publisher(Bool, '~/valid', 10)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self._debug_pub = self.create_publisher(
            Image, '~/debug_image', self._sensor_qos)
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info,
            self._sensor_qos)
        self.create_subscription(
            Image, self._image_topic, self._on_image, self._sensor_qos)
        self.create_timer(min(0.05, self._stale_after_s / 2.0), self._heartbeat)
        self.get_logger().info(
            f'Waiting for one {self._target_color} cube on {self._image_topic}; '
            f'{self._required_samples} continuous frames required')

    def _declare_parameters(self):
        defaults = {
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'target_color': 'red',
            'red_hue_intervals': [0, 10, 170, 179],
            'yellow_hue_intervals': [18, 34],
            'blue_hue_intervals': [90, 135],
            'green_hue_intervals': [40, 85],
            'saturation_min': 80,
            'value_min': 35,
            'morphology_size': 3,
            'min_area_px': 120.0,
            'max_area_px': 12000.0,
            'min_aspect_ratio': 0.45,
            'max_aspect_ratio': 2.2,
            'min_solidity': 0.72,
            'min_rectangularity': 0.55,
            'interior_fraction': 0.23,
            'required_stable_frames': 2,
            'maximum_center_jump_px': 45.0,
            'stale_after_s': 0.25,
            'future_stamp_tolerance_s': 0.05,
            'center_half_width_px': 40,
            'center_half_height_px': 40,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        def value(name):
            return self.get_parameter(name).value

        self._image_topic = str(value('image_topic'))
        self._camera_info_topic = str(value('camera_info_topic'))
        self._target_color = str(value('target_color')).strip().lower()
        if self._target_color not in SUPPORTED_COLORS:
            raise ValueError(
                f'target_color must be one of {", ".join(SUPPORTED_COLORS)}')
        self._color_intervals = {
            name: list(value(f'{name}_hue_intervals'))
            for name in SUPPORTED_COLORS}
        self._segmentation = {
            'saturation_min': int(value('saturation_min')),
            'value_min': int(value('value_min')),
            'morphology_size': int(value('morphology_size')),
        }
        self._candidate_options = {
            'min_area_px': float(value('min_area_px')),
            'max_area_px': float(value('max_area_px')),
            'min_aspect_ratio': float(value('min_aspect_ratio')),
            'max_aspect_ratio': float(value('max_aspect_ratio')),
            'min_solidity': float(value('min_solidity')),
            'min_rectangularity': float(value('min_rectangularity')),
            'interior_fraction': float(value('interior_fraction')),
        }
        self._required_samples = int(value('required_stable_frames'))
        self._continuity = TargetContinuity(
            self._required_samples, float(value('maximum_center_jump_px')))
        self._stale_after_s = float(value('stale_after_s'))
        self._future_tolerance_s = float(value('future_stamp_tolerance_s'))
        self._center_half_width = int(value('center_half_width_px'))
        self._center_half_height = int(value('center_half_height_px'))
        if self._stale_after_s <= 0 or self._future_tolerance_s < 0:
            raise ValueError('invalid_timestamp_limits')
        if self._center_half_width <= 0 or self._center_half_height <= 0:
            raise ValueError('invalid_center_region')

    def _on_camera_info(self, message):
        with self._lock:
            self._camera_info = message

    def _reference_center(self, width, height):
        with self._lock:
            info = self._camera_info
        if info is not None and info.width == width and info.height == height:
            values = np.asarray(info.k, dtype=np.float64)
            if values.shape == (9,) and np.isfinite(values).all():
                cx, cy = float(values[2]), float(values[5])
                if 0.0 <= cx < width and 0.0 <= cy < height:
                    return (cx, cy), 'camera_info_principal_point'
        return ((width - 1) * 0.5, (height - 1) * 0.5), 'image_geometry'

    def _on_image(self, message):
        received_at = time.monotonic()
        stamp_s = stamp_seconds(message)
        with self._lock:
            self._last_frame_monotonic = received_at
            self._last_frame_stamp = stamp_s
            self._stream_stale = False
        annotated = None
        status = self._base_status(message, stamp_s)
        detection = TargetDetection(None, 0, 'processing_error')
        try:
            image = image_array(message)
            if message.encoding == 'rgb8':
                bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif message.encoding == 'bgr8':
                bgr = image.copy()
            else:
                raise ValueError('unsupported_color_encoding')
            annotated = bgr.copy()
            center_reference, center_source = self._reference_center(
                message.width, message.height)
            status.update({
                'image_width': int(message.width),
                'image_height': int(message.height),
                'center_reference_px': list(center_reference),
                'center_source': center_source,
            })
            self._draw_center_region(annotated, center_reference)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            source_age = now_s - stamp_s
            status['observation_age_s'] = source_age
            if stamp_s <= 0.0 or source_age > self._stale_after_s or \
                    source_age < -self._future_tolerance_s:
                self._continuity.reject()
                status.update(valid=False, reason='image_timestamp_stale')
            else:
                detection = detect_color_target(
                    bgr, self._target_color, self._color_intervals,
                    **self._segmentation, **self._candidate_options)
                status['candidate_count'] = detection.candidate_count
                if detection.candidate is None:
                    self._continuity.reject(transient=True)
                    status.update(valid=False, reason=detection.reason)
                else:
                    candidate = detection.candidate
                    cv2.drawContours(
                        annotated, [candidate.contour], -1, (0, 165, 255), 2)
                    stable, reason = self._continuity.update(
                        stamp_s, candidate.center_px)
                    bbox = candidate_bbox(candidate)
                    quality = candidate_quality(candidate)
                    status.update({
                        'valid': stable,
                        'reason': reason,
                        'target_center_px': [float(v) for v in candidate.center_px],
                        'bbox_xywh': list(bbox),
                        'quality': {
                            'score': quality,
                            'area_px': float(candidate.area_px),
                            'aspect_ratio': float(candidate.aspect_ratio),
                            'solidity': float(candidate.solidity),
                            'rectangularity': float(candidate.rectangularity),
                        },
                    })
                    self._draw_candidate(annotated, candidate, bbox, stable)
                    if stable:
                        self._publish_observation(message, candidate, bbox, quality)
        except (ValueError, cv2.error) as error:
            self._continuity.reject()
            status.update(valid=False, reason=str(error))
        status['stable_samples'] = self._continuity.sample_count
        status['required_stable_samples'] = self._required_samples
        self._annotate_status(annotated, status)
        self._publish_state(status)
        if annotated is not None:
            self._publish_debug(annotated, message.header)

    def _base_status(self, message, stamp_s):
        return {
            'schema_version': 1,
            'valid': False,
            'reason': 'processing',
            'target_color': self._target_color,
            'source_stamp_s': float(stamp_s),
            'source_frame': message.header.frame_id,
            'candidate_count': 0,
            'target_center_px': None,
            'bbox_xywh': None,
            'quality': None,
        }

    def _draw_center_region(self, image, center):
        cx, cy = (int(round(value)) for value in center)
        cv2.rectangle(
            image,
            (max(0, cx - self._center_half_width),
             max(0, cy - self._center_half_height)),
            (min(image.shape[1] - 1, cx + self._center_half_width),
             min(image.shape[0] - 1, cy + self._center_half_height)),
            (255, 255, 0), 2)
        cv2.drawMarker(image, (cx, cy), (255, 255, 0), cv2.MARKER_CROSS, 16, 2)

    @staticmethod
    def _draw_candidate(image, candidate, bbox, stable):
        color = (0, 255, 0) if stable else (0, 165, 255)
        x, y, width, height = bbox
        cv2.rectangle(image, (x, y), (x + width - 1, y + height - 1), color, 2)
        center = tuple(int(round(value)) for value in candidate.center_px)
        cv2.drawMarker(image, center, color, cv2.MARKER_CROSS, 18, 2)

    def _annotate_status(self, image, status):
        if image is None:
            return
        valid = bool(status.get('valid', False))
        text = f'{self._target_color}: {status.get("reason", "unknown")}'
        color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.rectangle(image, (0, 0), (min(image.shape[1] - 1, 520), 30),
                      (0, 0, 0), cv2.FILLED)
        cv2.putText(image, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 1, cv2.LINE_AA)

    def _publish_observation(self, image_message, candidate, bbox, quality):
        center = PointStamped()
        center.header = image_message.header
        center.point.x = float(candidate.center_px[0])
        center.point.y = float(candidate.center_px[1])
        center.point.z = 0.0
        self._center_pub.publish(center)

        x, y, width, height = bbox
        roi = RegionOfInterest()
        roi.x_offset = x
        roi.y_offset = y
        roi.width = width
        roi.height = height
        roi.do_rectify = False
        self._bbox_pub.publish(roi)

        detection = Detection2D()
        detection.header = image_message.header
        detection.id = f'{self._target_color}-cube'
        detection.bbox.center.position.x = float(candidate.center_px[0])
        detection.bbox.center.position.y = float(candidate.center_px[1])
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self._target_color
        hypothesis.hypothesis.score = quality
        detection.results.append(hypothesis)
        self._detection_pub.publish(detection)

    def _publish_state(self, status):
        valid = bool(status.get('valid', False))
        self._valid_pub.publish(Bool(data=valid))
        text = json.dumps(status, sort_keys=True, separators=(',', ':'))
        self._status_pub.publish(String(data=text))
        signature = (valid, status.get('reason'), status.get('stable_samples'))
        if signature != self._last_log_signature:
            log = self.get_logger().info if valid else self.get_logger().debug
            log(f'2-D target status: {text}')
            self._last_log_signature = signature

    def _publish_debug(self, bgr, header):
        message = Image()
        message.header = header
        message.height, message.width = bgr.shape[:2]
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = message.width * 3
        # Passing bytes makes generated ROS setters validate ~0.9 million
        # pixels one by one in Python. array('B') takes the zero-validation
        # fast path and keeps the 10 FPS input queue current.
        message.data = array('B', np.ascontiguousarray(bgr).tobytes())
        self._debug_pub.publish(message)

    def _heartbeat(self):
        now = time.monotonic()
        with self._lock:
            last_frame = self._last_frame_monotonic
            already_stale = self._stream_stale
            last_stamp = self._last_frame_stamp
            stale = last_frame is None or now - last_frame > self._stale_after_s
            if stale:
                self._stream_stale = True
        if stale:
            if not already_stale:
                self._continuity.reject()
            self._publish_state({
                'schema_version': 1,
                'valid': False,
                'reason': 'image_stream_stale',
                'target_color': self._target_color,
                'source_stamp_s': last_stamp,
                'observation_age_s': None if last_frame is None else now - last_frame,
                'candidate_count': 0,
                'target_center_px': None,
                'bbox_xywh': None,
                'quality': None,
                'stable_samples': 0,
                'required_stable_samples': self._required_samples,
            })


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ColorTarget2D()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
