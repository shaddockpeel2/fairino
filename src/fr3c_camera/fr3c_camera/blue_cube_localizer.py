"""Locate one stationary blue 30 mm cube from registered RGB-D images."""

from collections import deque
import json
import math
from pathlib import Path
import threading

import cv2
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import yaml

from fr3c_camera.cube_perception import (
    blue_mask,
    extract_top_surface,
    find_blue_candidates,
    quaternion_angular_distance,
    registered_points_from_mask,
    rpy_to_quaternion,
    transform_points,
)
from fr3c_camera.rgbd import image_array, stamp_seconds, validate_pair


def _finite_vector(values, length, name):
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f'{name}_must_have_{length}_finite_values')
    return result


def _update_sample_window(
        samples, stamp, center, payload, maximum_gap, maximum_jump,
        required_samples, xy_std_limit, z_std_limit):
    """Append one valid observation and evaluate a short rolling window."""
    if samples:
        gap = stamp - samples[-1][0]
        jump = float(np.linalg.norm(center - samples[-1][1]))
        if gap <= 0.0 or gap > maximum_gap or jump > maximum_jump:
            samples.clear()
    samples.append((stamp, center, payload))
    centers = np.asarray([sample[1] for sample in samples])
    standard_deviation = np.std(centers, axis=0)
    stable = (
        len(samples) == required_samples and
        float(max(standard_deviation[0], standard_deviation[1])) <= xy_std_limit and
        float(standard_deviation[2]) <= z_std_limit)
    return stable, standard_deviation


class BlueCubeLocalizer(Node):
    def __init__(self):
        super().__init__('blue_cube_localizer')
        self._lock = threading.Lock()
        self._declare_parameters()
        self._read_parameters()
        self._hand_eye = self._load_hand_eye(self._hand_eye_file)
        self._tcp = self._load_tcp(self._tcp_file)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # TF must continue updating while an RGB-D callback waits for an exact
        # image-time transform. A dedicated listener node avoids starving TF
        # behind CPU-heavy image processing in this node's executor.
        self._tf_listener = TransformListener(
            self._tf_buffer, None, spin_thread=True)
        self._pose_pub = self.create_publisher(PoseStamped, '~/top_pose', 1)
        self._valid_pub = self.create_publisher(Bool, '~/valid', 1)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self._marker_pub = self.create_publisher(Marker, '~/marker', 1)
        self.create_service(Trigger, '~/reset', self._reset_service)
        self.create_subscription(
            JointState, self._joint_topic, self._joint_callback,
            qos_profile_sensor_data)

        subscribers = [
            message_filters.Subscriber(
                self, Image, self._color_topic, qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(
                self, Image, self._depth_topic, qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(
                self, CameraInfo, self._color_info_topic,
                qos_profile=qos_profile_sensor_data),
            message_filters.Subscriber(
                self, CameraInfo, self._depth_info_topic,
                qos_profile=qos_profile_sensor_data),
        ]
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            subscribers, queue_size=30, slop=self._max_skew_s)
        self._synchronizer.registerCallback(self._rgbd_callback)
        self.create_timer(0.25, self._heartbeat)

        self._joint_history = deque()
        self._samples = deque(maxlen=self._stable_samples)
        self._stable_pose = None
        self._stable_surface = None
        self._last_frame_time = None
        self._last_processed_stamp = None
        self._last_status = {
            'valid': False, 'stable': False, 'tracking': True,
            'frozen': False, 'has_stable_pose': False,
            'reason': 'waiting_for_rgbd'}
        self._last_status_text = ''
        self.get_logger().info(
            'Blue-cube localizer started in continuous tracking mode; valid pose '
            f'publication requires {self._stable_samples} stable samples.')

    def _declare_parameters(self):
        defaults = {
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'color_info_topic': '/camera/color/camera_info',
            'depth_info_topic': '/camera/depth/camera_info',
            'joint_state_topic': '/joint_states',
            'base_frame': 'base_link',
            'expected_optical_frame': 'camera_color_optical_frame',
            'expected_camera_serial': 'CH82C5201C9',
            'hand_eye_file': '',
            'tcp_file': '',
            'cube_side_m': 0.030,
            'fixed_yaw_rad': 0.0,
            'max_age_s': 0.50,
            'max_skew_s': 0.065,
            'tf_timeout_s': 0.12,
            'sample_period_s': 0.18,
            'hand_eye_translation_tolerance_m': 0.0005,
            'hand_eye_rotation_tolerance_rad': 0.002,
            'tcp_translation_tolerance_m': 0.00001,
            'tcp_rotation_tolerance_rad': 0.00001,
            'hue_min': 90,
            'hue_max': 135,
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
            'min_depth_m': 0.15,
            'max_depth_m': 1.50,
            'min_depth_valid_fraction': 0.55,
            'top_quantile': 65.0,
            'top_tolerance_m': 0.006,
            'min_top_points': 30,
            'min_observed_side_m': 0.014,
            'max_observed_side_m': 0.050,
            'max_top_z_spread_m': 0.010,
            'workspace_x': [-1.0, 1.0],
            'workspace_y': [-1.0, 1.0],
            'workspace_z': [-0.10, 1.20],
            'robot_joint_names': [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'joint_state_max_age_s': 0.25,
            'stationary_window_s': 0.40,
            'stationary_joint_span_rad': 0.001,
            'stable_samples': 8,
            'stable_xy_std_m': 0.003,
            'stable_z_std_m': 0.003,
            'stable_sample_max_gap_s': 0.75,
            'max_sample_jump_m': 0.020,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        def value(name):
            return self.get_parameter(name).value

        self._color_topic = value('color_topic')
        self._depth_topic = value('depth_topic')
        self._color_info_topic = value('color_info_topic')
        self._depth_info_topic = value('depth_info_topic')
        self._joint_topic = value('joint_state_topic')
        self._base_frame = value('base_frame')
        self._optical_frame = value('expected_optical_frame')
        self._expected_camera_serial = value('expected_camera_serial')
        self._hand_eye_file = value('hand_eye_file')
        self._tcp_file = value('tcp_file')
        self._cube_side = float(value('cube_side_m'))
        self._fixed_yaw = float(value('fixed_yaw_rad'))
        self._max_age_s = float(value('max_age_s'))
        self._max_skew_s = float(value('max_skew_s'))
        self._tf_timeout_s = float(value('tf_timeout_s'))
        self._sample_period_s = float(value('sample_period_s'))
        self._hand_eye_translation_tolerance = float(
            value('hand_eye_translation_tolerance_m'))
        self._hand_eye_rotation_tolerance = float(
            value('hand_eye_rotation_tolerance_rad'))
        self._tcp_translation_tolerance = float(
            value('tcp_translation_tolerance_m'))
        self._tcp_rotation_tolerance = float(
            value('tcp_rotation_tolerance_rad'))
        self._segmentation = {
            key: value(key) for key in (
                'hue_min', 'hue_max', 'saturation_min', 'value_min',
                'morphology_size')}
        self._candidate_filter = {
            key: value(key) for key in (
                'min_area_px', 'max_area_px', 'min_aspect_ratio',
                'max_aspect_ratio', 'min_solidity', 'min_rectangularity',
                'interior_fraction')}
        self._min_depth = float(value('min_depth_m'))
        self._max_depth = float(value('max_depth_m'))
        self._min_depth_valid = float(value('min_depth_valid_fraction'))
        self._top_quantile = float(value('top_quantile'))
        self._top_tolerance = float(value('top_tolerance_m'))
        self._min_top_points = int(value('min_top_points'))
        self._min_side = float(value('min_observed_side_m'))
        self._max_side = float(value('max_observed_side_m'))
        self._max_top_z_spread = float(value('max_top_z_spread_m'))
        self._workspace = np.array([
            _finite_vector(value('workspace_x'), 2, 'workspace_x'),
            _finite_vector(value('workspace_y'), 2, 'workspace_y'),
            _finite_vector(value('workspace_z'), 2, 'workspace_z'),
        ])
        if np.any(self._workspace[:, 0] >= self._workspace[:, 1]):
            raise ValueError('workspace_intervals_must_increase')
        self._joint_names = list(value('robot_joint_names'))
        self._joint_max_age = float(value('joint_state_max_age_s'))
        self._stationary_window = float(value('stationary_window_s'))
        self._stationary_span = float(value('stationary_joint_span_rad'))
        self._stable_samples = int(value('stable_samples'))
        self._stable_xy_std = float(value('stable_xy_std_m'))
        self._stable_z_std = float(value('stable_z_std_m'))
        self._stable_sample_max_gap = float(value('stable_sample_max_gap_s'))
        self._max_sample_jump = float(value('max_sample_jump_m'))
        scalars = [
            self._cube_side, self._max_age_s, self._max_skew_s,
            self._tf_timeout_s, self._sample_period_s,
            self._hand_eye_translation_tolerance,
            self._hand_eye_rotation_tolerance,
            self._tcp_translation_tolerance, self._tcp_rotation_tolerance,
            self._min_depth,
            self._max_depth, self._min_depth_valid, self._top_tolerance,
            self._min_side, self._max_side, self._max_top_z_spread,
            self._joint_max_age, self._stationary_window, self._stationary_span,
            self._stable_xy_std, self._stable_z_std,
            self._stable_sample_max_gap, self._max_sample_jump,
        ]
        if not all(math.isfinite(number) and number > 0 for number in scalars):
            raise ValueError('numeric_parameters_must_be_finite_and_positive')
        if self._stable_samples < 2 or len(self._joint_names) != 6:
            raise ValueError('stable_samples_or_robot_joint_names_invalid')

    def _load_hand_eye(self, path_text):
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        if not path.is_file():
            raise RuntimeError(f'hand-eye file does not exist: {path}')
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(data, dict) or data.get('calibrated') is not True:
            raise RuntimeError(f'hand-eye file is not marked calibrated: {path}')
        if data.get('handeye_accepted') is not True or data.get('status') != 'accepted':
            raise RuntimeError(f'hand-eye file is not accepted: {path}')
        calibrated_serial = str(data.get('camera_serial', '')).strip()
        if not self._expected_camera_serial or calibrated_serial != self._expected_camera_serial:
            raise RuntimeError(
                'hand-eye camera serial does not match expected_camera_serial: '
                f'{calibrated_serial!r} != {self._expected_camera_serial!r}')
        parent = str(data.get('parent_frame', '')).strip()
        child = str(data.get('child_frame', '')).strip()
        xyz = _finite_vector(data.get('xyz'), 3, 'hand_eye_xyz')
        rpy = _finite_vector(data.get('rpy'), 3, 'hand_eye_rpy')
        if not parent or not child or parent == child:
            raise RuntimeError('hand-eye frames are invalid')
        return {
            'path': str(path), 'parent': parent, 'child': child,
            'translation': xyz, 'quaternion': rpy_to_quaternion(*rpy),
            'camera_serial': calibrated_serial,
        }

    def _load_tcp(self, path_text):
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        if not path.is_file():
            raise RuntimeError(f'TCP file does not exist: {path}')
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
        data = document.get('ydi_j22_tcp') if isinstance(document, dict) else None
        if not isinstance(data, dict) or data.get('defined') is not True or \
                data.get('calibrated') is not True:
            raise RuntimeError(
                f'TCP file is not marked defined and calibrated: {path}')
        parent = str(data.get('parent_frame', '')).strip()
        child = str(data.get('child_frame', '')).strip()
        translation = _finite_vector(data.get('xyz_m'), 3, 'tcp_xyz_m')
        rpy = _finite_vector(data.get('rpy_rad'), 3, 'tcp_rpy_rad')
        if parent != 'flange_link' or child != 'grasp_frame':
            raise RuntimeError(
                'TCP must define flange_link -> grasp_frame for this task')
        return {
            'path': str(path), 'parent': parent, 'child': child,
            'translation': translation,
            'quaternion': rpy_to_quaternion(*rpy),
            'version': data.get('version'),
        }

    def _joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and math.isfinite(positions[name])
                   for name in self._joint_names):
            return
        stamp = stamp_seconds(message)
        sample = np.array([positions[name] for name in self._joint_names])
        with self._lock:
            self._joint_history.append((stamp, sample))
            cutoff = stamp - max(2.0, self._stationary_window * 3.0)
            while self._joint_history and self._joint_history[0][0] < cutoff:
                self._joint_history.popleft()

    def _robot_stationary(self, image_stamp):
        with self._lock:
            history = list(self._joint_history)
        nearest_age = min(
            (abs(image_stamp - stamp) for stamp, _ in history), default=math.inf)
        if nearest_age > self._joint_max_age:
            return False, 'joint_state_stale'
        start = image_stamp - self._stationary_window
        samples = [position for stamp, position in history if start <= stamp <= image_stamp + 0.02]
        if len(samples) < 2 or history[0][0] > start + 0.05:
            return False, 'stationary_window_incomplete'
        span = np.ptp(np.asarray(samples), axis=0)
        if float(np.max(span)) > self._stationary_span:
            return False, 'robot_not_stationary'
        return True, ''

    @staticmethod
    def _transform_arrays(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            np.array([translation.x, translation.y, translation.z]),
            np.array([rotation.x, rotation.y, rotation.z, rotation.w]),
        )

    def _lookup(self, target, source, stamp):
        return self._tf_buffer.lookup_transform(
            target, source, rclpy.time.Time.from_msg(stamp),
            timeout=Duration(seconds=self._tf_timeout_s))

    def _verify_hand_eye(self, stamp):
        if self._hand_eye is None:
            raise ValueError('hand_eye_file_required')
        transform = self._lookup(
            self._hand_eye['parent'], self._hand_eye['child'], stamp)
        translation, quaternion = self._transform_arrays(transform)
        translation_error = float(np.linalg.norm(
            translation - self._hand_eye['translation']))
        rotation_error = quaternion_angular_distance(
            quaternion, self._hand_eye['quaternion'])
        if translation_error > self._hand_eye_translation_tolerance or \
                rotation_error > self._hand_eye_rotation_tolerance:
            raise ValueError(
                'online_hand_eye_mismatch:'
                f'{translation_error:.6f}m:{rotation_error:.6f}rad')

    def _verify_tcp(self, stamp):
        if self._tcp is None:
            raise ValueError('tcp_file_required')
        transform = self._lookup(
            self._tcp['parent'], self._tcp['child'], stamp)
        translation, quaternion = self._transform_arrays(transform)
        translation_error = float(np.linalg.norm(
            translation - self._tcp['translation']))
        rotation_error = quaternion_angular_distance(
            quaternion, self._tcp['quaternion'])
        if translation_error > self._tcp_translation_tolerance or \
                rotation_error > self._tcp_rotation_tolerance:
            raise ValueError(
                'online_tcp_mismatch:'
                f'{translation_error:.6f}m:{rotation_error:.6f}rad')

    def _rgbd_callback(self, color, depth, color_info, depth_info):
        frame_stamp = stamp_seconds(color)
        self._last_frame_time = self.get_clock().now().nanoseconds * 1e-9
        if self._last_processed_stamp is not None and \
                frame_stamp - self._last_processed_stamp < self._sample_period_s:
            return
        self._last_processed_stamp = frame_stamp
        annotated = None
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            depth_m = validate_pair(
                color, depth, color_info, depth_info, now,
                max_age=self._max_age_s, max_skew=self._max_skew_s)
            if color.header.frame_id != self._optical_frame:
                raise ValueError('unexpected_optical_frame')
            rgb = image_array(color)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if color.encoding == 'rgb8' else rgb.copy()
            annotated = bgr.copy()
            mask = blue_mask(bgr, **self._segmentation)
            candidates = find_blue_candidates(mask, **self._candidate_filter)
            for candidate in candidates:
                cv2.drawContours(annotated, [candidate.contour], -1, (0, 255, 255), 2)
            if len(candidates) != 1:
                raise ValueError(f'target_count_{len(candidates)}')
            candidate = candidates[0]
            cv2.drawContours(annotated, [candidate.contour], -1, (0, 255, 0), 2)

            stationary, reason = self._robot_stationary(frame_stamp)
            if not stationary:
                raise ValueError(reason)
            self._verify_hand_eye(color.header.stamp)
            self._verify_tcp(color.header.stamp)
            base_transform = self._lookup(
                self._base_frame, self._optical_frame, color.header.stamp)
            translation, quaternion = self._transform_arrays(base_transform)
            points_camera, valid_fraction = registered_points_from_mask(
                depth_m, color_info, depth_info, candidate.mask,
                self._min_depth, self._max_depth)
            if valid_fraction < self._min_depth_valid:
                raise ValueError(f'insufficient_valid_depth_fraction:{valid_fraction:.3f}')
            points_base = transform_points(points_camera, translation, quaternion)
            surface = extract_top_surface(
                points_base, valid_fraction, self._top_quantile,
                self._top_tolerance, self._min_top_points)
            if surface.z_spread > self._max_top_z_spread:
                raise ValueError(f'top_surface_noisy:{surface.z_spread:.6f}')
            if np.any(surface.size_xy < self._min_side) or np.any(
                    surface.size_xy > self._max_side):
                raise ValueError(
                    'observed_size_out_of_range:' +
                    ','.join(f'{size:.4f}' for size in surface.size_xy))
            if np.any(surface.center < self._workspace[:, 0]) or \
                    np.any(surface.center > self._workspace[:, 1]):
                raise ValueError('target_outside_workspace')

            pose = self._make_pose(surface.center, color.header.stamp)
            status = self._accept_sample(pose, surface, candidate)
            self._set_status(status)
            cv2.circle(
                annotated,
                (int(round(candidate.center_px[0])), int(round(candidate.center_px[1]))),
                4, (0, 0, 255), -1)
        except (ValueError, TransformException) as error:
            with self._lock:
                has_stable_pose = self._stable_pose is not None
            self._set_status({
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': has_stable_pose,
                'reason': str(error),
                'sample_stamp': frame_stamp,
            })
            if annotated is not None:
                cv2.putText(
                    annotated, str(error)[:72], (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 1, cv2.LINE_AA)
        except Exception as error:  # Keep malformed sensor data from killing the node.
            self.get_logger().error(f'Unexpected localization failure: {error}')
            with self._lock:
                has_stable_pose = self._stable_pose is not None
            self._set_status({
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': has_stable_pose,
                'reason': f'internal_error:{type(error).__name__}',
                'sample_stamp': frame_stamp,
            })
        if annotated is not None:
            self._publish_debug(annotated, color.header)

    def _make_pose(self, center, stamp):
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(center[0])
        pose.pose.position.y = float(center[1])
        pose.pose.position.z = float(center[2])
        pose.pose.orientation.z = math.sin(self._fixed_yaw * 0.5)
        pose.pose.orientation.w = math.cos(self._fixed_yaw * 0.5)
        return pose

    def _accept_sample(self, pose, surface, candidate):
        center = np.array([
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        stamp = stamp_seconds(pose)
        with self._lock:
            stable, standard_deviation = _update_sample_window(
                self._samples, stamp, center, (pose, surface),
                self._stable_sample_max_gap, self._max_sample_jump,
                self._stable_samples, self._stable_xy_std, self._stable_z_std)
            if stable:
                centers = np.asarray([sample[1] for sample in self._samples])
                median = np.median(centers, axis=0)
                pose.pose.position.x = float(median[0])
                pose.pose.position.y = float(median[1])
                pose.pose.position.z = float(median[2])
                self._stable_pose = pose
                self._stable_surface = surface
            has_stable_pose = self._stable_pose is not None
            sample_count = len(self._samples)
        status = {
            'valid': stable, 'stable': stable, 'tracking': True,
            'frozen': False, 'has_stable_pose': has_stable_pose,
            'reason': 'stable_target_updated' if stable else 'collecting_stable_samples',
            'samples': sample_count,
            'required_samples': self._stable_samples,
            'sample_stamp': stamp_seconds(pose),
            'center_base_m': center.tolist(),
            'observed_side_m': surface.size_xy.tolist(),
            'top_z_spread_m': surface.z_spread,
            'depth_valid_fraction': surface.depth_valid_fraction,
            'candidate_area_px': candidate.area_px,
            'position_std_m': standard_deviation.tolist(),
            'orientation_source': 'fixed_yaw_parameter',
        }
        if stable:
            self._pose_pub.publish(pose)
            self._marker_pub.publish(self._make_marker(pose))
            self.get_logger().info(
                f'Updated stable blue-cube top center in {self._base_frame}: '
                f'[{pose.pose.position.x:.4f}, {pose.pose.position.y:.4f}, '
                f'{pose.pose.position.z:.4f}] m')
        return status

    def _make_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'blue_cube_localization'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z - self._cube_side * 0.5
        marker.pose.orientation = pose.pose.orientation
        marker.scale.x = self._cube_side
        marker.scale.y = self._cube_side
        marker.scale.z = self._cube_side
        marker.color.r = 0.05
        marker.color.g = 0.25
        marker.color.b = 1.0
        marker.color.a = 0.80
        return marker

    def _publish_debug(self, bgr, header):
        message = Image()
        message.header = header
        message.height, message.width = bgr.shape[:2]
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = np.ascontiguousarray(bgr).tobytes()
        self._debug_pub.publish(message)

    def _set_status(self, update):
        with self._lock:
            self._last_status = update

    def _heartbeat(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            status = dict(self._last_status)
            pose = self._stable_pose
        stream_stale = (
            self._last_frame_time is None or
            now - self._last_frame_time > self._max_age_s)
        if stream_stale:
            status = {
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': pose is not None,
                'reason': 'rgbd_stream_stale'}
        text = json.dumps(status, sort_keys=True, separators=(',', ':'))
        self._valid_pub.publish(Bool(data=bool(status.get('valid', False))))
        self._status_pub.publish(String(data=text))
        if pose is not None:
            self._pose_pub.publish(pose)
            self._marker_pub.publish(self._make_marker(pose))
        log_signature = json.dumps({
            key: status.get(key) for key in
            ('valid', 'stable', 'tracking', 'reason', 'samples', 'required_samples')
        }, sort_keys=True, separators=(',', ':'))
        if log_signature != self._last_status_text:
            self.get_logger().info(f'Localization status: {text}')
            self._last_status_text = log_signature

    def _reset_service(self, _request, response):
        with self._lock:
            self._samples.clear()
            self._stable_pose = None
            self._stable_surface = None
            self._last_processed_stamp = None
            self._last_status = {
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': False,
                'reason': 'reset_waiting_for_rgbd'}
        delete = Marker()
        delete.header.frame_id = self._base_frame
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.ns = 'blue_cube_localization'
        delete.id = 0
        delete.action = Marker.DELETE
        self._marker_pub.publish(delete)
        response.success = True
        response.message = 'Saved stable target cleared; continuous tracking restarted.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BlueCubeLocalizer()
        rclpy.spin(node)
    except (RuntimeError, ValueError) as error:
        if rclpy.ok():
            if node is not None:
                node.get_logger().fatal(f'Configuration rejected: {error}')
            else:
                print(f'Configuration rejected: {error}')
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
