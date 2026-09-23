"""Locate one stationary blue box from registered RGB-D images."""

from collections import deque
import json
import math
from pathlib import Path
import threading

import cv2
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from fairino_msgs.srv import SetTargetColor
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import yaml

from fr3c_camera.cube_perception import (
    blue_mask,
    compensate_top_surface_z,
    find_color_candidates,
    extract_top_surface,
    find_blue_candidates,
    quaternion_angular_distance,
    registered_points_from_mask,
    rpy_to_quaternion,
    tabletop_cube_tier,
    transform_points,
)
from fr3c_camera.rgbd import image_array, stamp_seconds, validate_pair
from fr3c_camera.inference_gate import InferenceGate
from fr3c_camera.yolo_cube_candidate import candidate_from_detection


SORT_COLOR_PRIORITY = {'red': 0, 'yellow': 1, 'blue': 2, 'green': 3}


def select_sort_target(
        physical_matches, skipped_targets, skip_radius_m, target_color=None):
    """Return one eligible cube, optionally requiring one unique requested color."""
    available = []
    skipped_count = 0
    for match in physical_matches:
        color_name, _, surface = match[0], match[1], match[2]
        if target_color is not None and color_name != target_color:
            continue
        center = np.asarray(surface.center, dtype=np.float64)
        skipped = any(
            color_name == skipped_color and
            float(np.linalg.norm(center[:2] - skipped_center[:2])) <= skip_radius_m
            for skipped_color, skipped_center in skipped_targets)
        if skipped:
            skipped_count += 1
        else:
            available.append(match)
    available.sort(key=lambda item: (
        SORT_COLOR_PRIORITY.get(item[0], len(SORT_COLOR_PRIORITY)),
        float(item[2].center[0]), float(item[2].center[1])))
    if target_color is not None and len(available) != 1:
        return None, skipped_count, len(available)
    return (available[0] if available else None), skipped_count, len(available)


def _finite_vector(values, length, name):
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f'{name}_must_have_{length}_finite_values')
    return result


def _update_sample_window(
        samples, stamp, center, payload, maximum_gap, maximum_jump,
        required_samples, xy_std_limit, z_std_limit, yaw=None,
        maximum_yaw_jump=math.inf, yaw_period=math.pi):
    """Append one valid observation and evaluate a short rolling window."""
    if samples:
        gap = stamp - samples[-1][0]
        jump = float(np.linalg.norm(center - samples[-1][1]))
        yaw_jump = 0.0
        if yaw is not None and len(samples[-1]) >= 4 and samples[-1][3] is not None:
            yaw_jump = abs(_periodic_angle_delta(yaw, samples[-1][3], yaw_period))
        if gap <= 0.0 or gap > maximum_gap or jump > maximum_jump or \
                yaw_jump > maximum_yaw_jump:
            samples.clear()
    samples.append((stamp, center, payload, yaw))
    centers = np.asarray([sample[1] for sample in samples])
    standard_deviation = np.std(centers, axis=0)
    stable = (
        len(samples) == required_samples and
        float(max(standard_deviation[0], standard_deviation[1])) <= xy_std_limit and
        float(standard_deviation[2]) <= z_std_limit)
    return stable, standard_deviation


def _periodic_angle_delta(first, second, period):
    """Shortest signed difference for an angle whose physical period is period."""
    return (float(first) - float(second) + period * 0.5) % period - period * 0.5


def _periodic_angle_mean_std(angles, period=math.pi):
    """Circular mean/std for an unoriented rectangle axis (default: pi-periodic)."""
    values = np.asarray(angles, dtype=np.float64)
    if values.ndim != 1 or values.size == 0 or not np.isfinite(values).all() or period <= 0:
        raise ValueError('invalid_periodic_angles')
    scale = 2.0 * math.pi / period
    phases = values * scale
    mean_phase = math.atan2(float(np.mean(np.sin(phases))),
                            float(np.mean(np.cos(phases))))
    mean = mean_phase / scale
    differences = np.array([
        _periodic_angle_delta(value, mean, period) for value in values])
    return float(mean), float(np.sqrt(np.mean(differences * differences)))


class BlueCubeLocalizer(Node):
    def __init__(self):
        super().__init__('blue_cube_localizer')
        self._lock = threading.Lock()
        self._declare_parameters()
        self._read_parameters()
        self._target_color = None
        self._filter_generation = 0
        self._inference_gate = InferenceGate()
        self._frames_processed = 0
        self._processing_error_count = 0
        self._yolo_detector = None
        if self._detection_mode == 'yolo':
            from fr3c_camera.rknn_detector import RKNNDetector
            self._yolo_detector = RKNNDetector(
                model_path=self._yolo_model_path,
                confidence_threshold=self._yolo_confidence,
                iou_threshold=self._yolo_iou,
                core_mode=self._yolo_core_mode,
            )
        self._hand_eye = self._load_hand_eye(self._hand_eye_file)
        self._tcp = self._load_tcp(self._tcp_file)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # TF must continue updating while an RGB-D callback waits for an exact
        # image-time transform. A dedicated listener node avoids starving TF
        # behind CPU-heavy image processing in this node's executor.
        self._tf_listener = TransformListener(
            self._tf_buffer, None, spin_thread=True)
        self._pose_pub = self.create_publisher(PoseStamped, '~/top_pose', 1)
        self._size_pub = self.create_publisher(Vector3Stamped, '~/object_size', 1)
        self._valid_pub = self.create_publisher(Bool, '~/valid', 1)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self._marker_pub = self.create_publisher(Marker, '~/marker', 1)
        self.create_service(Trigger, '~/reset', self._reset_service)
        self.create_service(
            SetBool, '~/set_inference_enabled',
            self._set_inference_enabled_service)
        self.create_service(
            SetTargetColor, '~/set_target_color', self._set_target_color_service)
        self.create_service(
            Trigger, '~/skip_current_sort_target',
            self._skip_current_sort_target_service)
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
        self._stable_object_size = None
        self._stable_color = None
        self._sort_skipped_targets = []
        self._last_frame_time = None
        self._last_processed_stamp = None
        self._last_status = {
            'valid': False, 'stable': False, 'tracking': True,
            'frozen': False, 'has_stable_pose': False,
            'detection_mode': self._detection_mode,
            'target_filter_color': '', 'filter_generation': 0,
            'reason': 'waiting_for_rgbd'}
        self._last_status_text = ''
        self.get_logger().info(
            f'Box localizer started with {self._detection_mode} detection; valid pose '
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
            'top_pose_z_compensation_m': -0.004,
            'cube_size_m': [0.030, 0.030, 0.030],
            'rectangular_box_size_m': [0.060, 0.030, 0.030],
            'visual_yaw_enabled': True,
            'fixed_yaw_rad': 0.0,
            'cube_max_axis_ratio': 1.40,
            'rectangle_min_axis_ratio': 1.55,
            'stable_yaw_std_rad': 0.087266463,
            'max_sample_yaw_jump_rad': 0.34906585,
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
            'max_aspect_ratio': 2.8,
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
            'max_observed_side_m': 0.075,
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
            'detection_mode': 'blue',
            'red_hue_intervals': [0, 10, 170, 179],
            'green_hue_intervals': [40, 85],
            'blue_hue_intervals': [90, 135],
            'yellow_hue_intervals': [18, 34],
            'sorting_plane_z_m': 0.0,
            'sorting_max_cube_plane_gap_m': 0.010,
            'sorting_max_cube_plane_penetration_m': 0.002,
            # P2 stacking: a cube whose bottom rests on top of another cube
            # publishes its top one cube height higher, so one entry per stacked
            # tier is needed to recognise it. Three entries cover the four-cube
            # tower that pick_stack allows; the lower cube of the tower is
            # occluded and is not reported separately. An empty list disables
            # the tier detection.
            'stacking_cube_height_offsets_m': [0.030, 0.060, 0.090],
            'stacking_tier_tolerance_m': 0.012,
            'sorting_skip_radius_m': 0.025,
            'yolo_model_path': '',
            'yolo_confidence_threshold': 0.25,
            'yolo_iou_threshold': 0.45,
            'yolo_core_mode': 'core0',
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
        self._top_pose_z_compensation = float(
            value('top_pose_z_compensation_m'))
        self._cube_size = _finite_vector(value('cube_size_m'), 3, 'cube_size_m')
        self._rectangle_size = _finite_vector(
            value('rectangular_box_size_m'), 3, 'rectangular_box_size_m')
        self._visual_yaw_enabled = bool(value('visual_yaw_enabled'))
        self._fixed_yaw = float(value('fixed_yaw_rad'))
        self._cube_max_axis_ratio = float(value('cube_max_axis_ratio'))
        self._rectangle_min_axis_ratio = float(value('rectangle_min_axis_ratio'))
        self._stable_yaw_std = float(value('stable_yaw_std_rad'))
        self._max_sample_yaw_jump = float(value('max_sample_yaw_jump_rad'))
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
        self._detection_mode = str(value('detection_mode')).lower()
        self._color_intervals = {
            color: list(value(f'{color}_hue_intervals'))
            for color in ('red', 'green', 'blue', 'yellow')}
        self._sorting_plane_z = float(value('sorting_plane_z_m'))
        self._sorting_max_cube_plane_gap = float(
            value('sorting_max_cube_plane_gap_m'))
        self._sorting_max_cube_plane_penetration = float(
            value('sorting_max_cube_plane_penetration_m'))
        self._stacking_cube_height_offsets = tuple(
            float(offset) for offset in value('stacking_cube_height_offsets_m'))
        self._stacking_tier_tolerance = float(
            value('stacking_tier_tolerance_m'))
        self._sorting_skip_radius = float(value('sorting_skip_radius_m'))
        self._yolo_model_path = str(value('yolo_model_path'))
        self._yolo_confidence = float(value('yolo_confidence_threshold'))
        self._yolo_iou = float(value('yolo_iou_threshold'))
        self._yolo_core_mode = str(value('yolo_core_mode'))
        if self._detection_mode not in ('blue', 'colors', 'yolo'):
            raise ValueError('detection_mode_must_be_blue_colors_or_yolo')
        if self._detection_mode == 'yolo' and not self._yolo_model_path:
            raise ValueError('yolo_model_path_required_in_yolo_mode')
        if not math.isfinite(self._sorting_plane_z) or not math.isfinite(
                self._sorting_max_cube_plane_gap) or not math.isfinite(
                self._sorting_max_cube_plane_penetration) or \
                self._sorting_max_cube_plane_gap < 0 or \
                self._sorting_max_cube_plane_penetration < 0:
            raise ValueError('invalid_sorting_plane_parameters')
        if not math.isfinite(self._stacking_tier_tolerance) or \
                self._stacking_tier_tolerance <= 0 or \
                any(not math.isfinite(offset) or offset <= 0
                    for offset in self._stacking_cube_height_offsets):
            raise ValueError('invalid_stacking_tier_parameters')
        if not all(math.isfinite(v) and 0.0 < v <= 1.0 for v in (
                self._yolo_confidence, self._yolo_iou)):
            raise ValueError('yolo_thresholds_must_be_in_(0,1]')
        if not math.isfinite(self._top_pose_z_compensation) or \
                abs(self._top_pose_z_compensation) > 0.020:
            raise ValueError('top_pose_z_compensation_must_be_within_20mm')
        scalars = [
            *self._cube_size, *self._rectangle_size,
            self._max_age_s, self._max_skew_s,
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
            self._cube_max_axis_ratio, self._rectangle_min_axis_ratio,
            self._stable_yaw_std,
            self._max_sample_yaw_jump,
            self._sorting_skip_radius,
        ]
        if not all(math.isfinite(number) and number > 0 for number in scalars):
            raise ValueError('numeric_parameters_must_be_finite_and_positive')
        if self._stable_samples < 2 or len(self._joint_names) != 6:
            raise ValueError('stable_samples_or_robot_joint_names_invalid')
        if self._cube_max_axis_ratio <= 1.0 or \
                self._rectangle_min_axis_ratio <= self._cube_max_axis_ratio or \
                self._stable_yaw_std >= math.pi * 0.5:
            raise ValueError('visual_yaw_thresholds_invalid')

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

    def _candidate_surface(self, candidate, depth_m, color_info, depth_info,
                           translation, quaternion):
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
        surface = compensate_top_surface_z(
            surface, self._top_pose_z_compensation)
        if np.any(surface.center < self._workspace[:, 0]) or \
                np.any(surface.center > self._workspace[:, 1]):
            raise ValueError('target_outside_workspace')
        return surface

    def _rgbd_callback(self, color, depth, color_info, depth_info):
        self._last_frame_time = self.get_clock().now().nanoseconds * 1e-9
        if not self._inference_gate.enter():
            gate = self._inference_gate.snapshot()
            with self._lock:
                self._last_status = {
                    'valid': False, 'stable': False, 'tracking': False,
                    'frozen': False, 'has_stable_pose': False,
                    'detection_mode': self._detection_mode,
                    'target_filter_color': self._target_color or '',
                    'filter_generation': self._filter_generation,
                    'reason': 'perception_paused',
                    'inference_enabled': gate.enabled,
                    'active_inferences': gate.active_inferences,
                    'pause_generation': gate.pause_generation,
                }
            return
        success = False
        try:
            self._process_rgbd_callback(color, depth, color_info, depth_info)
            with self._lock:
                reason = str(self._last_status.get('reason', ''))
            success = not reason.startswith('internal_error:')
            if success:
                self._frames_processed += 1
            else:
                self._processing_error_count += 1
        except BaseException:
            self._processing_error_count += 1
            raise
        finally:
            self._inference_gate.leave(success=success)

    def _process_rgbd_callback(self, color, depth, color_info, depth_info):
        frame_stamp = stamp_seconds(color)
        self._last_frame_time = self.get_clock().now().nanoseconds * 1e-9
        if self._last_processed_stamp is not None and \
                frame_stamp - self._last_processed_stamp < self._sample_period_s:
            return
        self._last_processed_stamp = frame_stamp
        annotated = None
        with self._lock:
            target_color = self._target_color
            filter_generation = self._filter_generation
        diagnostics = {
            'target_filter_color': target_color or '',
            'filter_generation': filter_generation,
        }
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
            detection = None
            color_name = None
            matches = None
            if self._detection_mode == 'blue':
                mask = blue_mask(bgr, **self._segmentation)
                candidates = find_blue_candidates(mask, **self._candidate_filter)
                for item in candidates:
                    cv2.drawContours(annotated, [item.contour], -1, (0, 255, 255), 2)
                if len(candidates) != 1:
                    raise ValueError(f'target_count_{len(candidates)}')
                candidate = candidates[0]
                color_name = 'blue'
                cv2.drawContours(annotated, [candidate.contour], -1, (0, 255, 0), 2)
            elif self._detection_mode == 'colors':
                matches = find_color_candidates(
                    bgr, self._color_intervals,
                    saturation_min=self._segmentation['saturation_min'],
                    value_min=self._segmentation['value_min'],
                    morphology_size=self._segmentation['morphology_size'],
                    **self._candidate_filter)
                diagnostics['rgb_color_regions'] = len(matches)
                for _, item in matches:
                    cv2.drawContours(annotated, [item.contour], -1,
                                     (0, 165, 255), 1)
                if not matches:
                    diagnostics.update({
                        'physical_cube_count': 0,
                        'skipped_cube_count': 0,
                        'actionable_cube_count': 0,
                        'rejected_color_regions': {},
                        'sort_color_order': ['red', 'yellow', 'blue', 'green'],
                        'detected_targets': [],
                        'physical_color_counts': {},
                        'requested_color_physical_count': 0,
                    })
                    raise ValueError('color_target_count_0')
            else:
                detections = self._yolo_detector.infer(bgr)
                if len(detections) != 1:
                    raise ValueError(f'yolo_target_count_{len(detections)}')
                detection = detections[0]
                candidate = candidate_from_detection(
                    detection, bgr.shape,
                    self._candidate_filter['min_area_px'],
                    self._candidate_filter['max_area_px'])
                cv2.drawContours(annotated, [candidate.contour], -1, (255, 0, 255), 2)
                x1, y1, _, _ = map(round, detection.bbox_xyxy)
                cv2.putText(
                    annotated, f'{detection.class_name} {detection.confidence:.2f}',
                    (x1, max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 255), 1, cv2.LINE_AA)

            stationary, reason = self._robot_stationary(frame_stamp)
            if not stationary:
                raise ValueError(reason)
            self._verify_hand_eye(color.header.stamp)
            self._verify_tcp(color.header.stamp)
            base_transform = self._lookup(
                self._base_frame, self._optical_frame, color.header.stamp)
            translation, quaternion = self._transform_arrays(base_transform)
            if matches is not None:
                physical_matches = []
                rejected_reasons = {}
                rejected_target_details = []
                for name, item in matches:
                    item_surface = None
                    try:
                        item_surface = self._candidate_surface(
                            item, depth_m, color_info, depth_info,
                            translation, quaternion)
                        tier = tabletop_cube_tier(
                            item_surface, self._cube_size[2],
                            self._cube_max_axis_ratio, self._sorting_plane_z,
                            self._sorting_max_cube_plane_gap,
                            stacked_offsets_m=self._stacking_cube_height_offsets,
                            stack_tolerance_m=self._stacking_tier_tolerance,
                            maximum_plane_penetration_m=
                            self._sorting_max_cube_plane_penetration)
                        if tier is None:
                            bottom_gap = (
                                float(item_surface.center[2]) -
                                float(self._cube_size[2]) -
                                self._sorting_plane_z)
                            raise ValueError(
                                'not_a_cube_on_sorting_plane:'
                                f'color={name},'
                                f'top_z_m={float(item_surface.center[2]):.9f},'
                                f'bottom_gap_m={bottom_gap:.9f},'
                                f'axis_ratio={float(item_surface.axis_ratio):.6f},'
                                f'max_axis_ratio={self._cube_max_axis_ratio:.6f},'
                                f'max_plane_gap_m='
                                f'{self._sorting_max_cube_plane_gap:.6f},'
                                f'max_plane_penetration_m='
                                f'{self._sorting_max_cube_plane_penetration:.6f}')
                        physical_matches.append((name, item, item_surface, tier))
                    except ValueError as error:
                        key = str(error).split(':', 1)[0]
                        rejected_reasons[key] = rejected_reasons.get(key, 0) + 1
                        detail = {'color': name, 'reason': str(error)}
                        if item_surface is not None:
                            detail.update({
                                'top_z_base_m': float(item_surface.center[2]),
                                'bottom_gap_m': (
                                    float(item_surface.center[2]) -
                                    float(self._cube_size[2]) -
                                    self._sorting_plane_z),
                                'axis_ratio': float(item_surface.axis_ratio),
                                'observed_short_long_m': [
                                    float(value) for value in item_surface.size_xy],
                            })
                        rejected_target_details.append(detail)
                diagnostics['physical_cube_count'] = len(physical_matches)
                diagnostics['rejected_color_regions'] = rejected_reasons
                diagnostics['rejected_target_details'] = rejected_target_details
                color_counts = {
                    name: sum(1 for color_name, _, _, _ in physical_matches
                              if color_name == name)
                    for name in SORT_COLOR_PRIORITY
                }
                diagnostics['physical_color_counts'] = color_counts
                diagnostics['requested_color_physical_count'] = (
                    color_counts.get(target_color, 0)
                    if target_color is not None else 0)
                with self._lock:
                    skipped_targets = list(self._sort_skipped_targets)
                selected, skipped_count, actionable_count = select_sort_target(
                    physical_matches, skipped_targets, self._sorting_skip_radius,
                    target_color=target_color)
                diagnostics['skipped_cube_count'] = skipped_count
                diagnostics['actionable_cube_count'] = actionable_count
                diagnostics['sort_color_order'] = ['red', 'yellow', 'blue', 'green']
                diagnostics['detected_targets'] = [
                    self._detected_target_entry(name, item_surface, tier)
                    for name, _, item_surface, tier in physical_matches
                ]
                diagnostics['stacked_cube_count'] = sum(
                    1 for _, _, _, tier in physical_matches if tier > 0)
                if target_color is not None and color_counts.get(target_color, 0) == 0:
                    raise ValueError(f'requested_color_not_found:{target_color}')
                if target_color is not None and actionable_count != 1:
                    raise ValueError(
                        f'requested_color_not_unique:{target_color}:count={actionable_count}')
                if selected is None:
                    raise ValueError(
                        f'actionable_cube_count_0_of_{len(physical_matches)}_physical_cubes')
                color_name, candidate, surface, _ = selected
                cv2.drawContours(annotated, [candidate.contour], -1,
                                 (0, 255, 0), 2)
                cv2.putText(annotated, color_name,
                            (int(candidate.center_px[0]),
                             int(candidate.center_px[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            else:
                surface = self._candidate_surface(
                    candidate, depth_m, color_info, depth_info,
                    translation, quaternion)

            object_class, object_size, visual_yaw, yaw_period = \
                self._surface_grasp_geometry(surface)
            yaw = visual_yaw if self._visual_yaw_enabled else self._fixed_yaw
            pose = self._make_pose(surface.center, yaw, color.header.stamp)
            status = self._accept_sample(
                pose, surface, candidate, object_class, object_size,
                yaw, yaw_period,
                detector_class=detection.class_name if detection is not None else None,
                color_name=color_name)
            if detection is not None:
                status['yolo_class'] = detection.class_name
                status['yolo_confidence'] = detection.confidence
            status.update(diagnostics)
            self._set_status(status, filter_generation)
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
                **diagnostics,
            }, filter_generation)
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
                **diagnostics,
            }, filter_generation)
        if annotated is not None:
            self._publish_debug(annotated, color.header)

    def _make_pose(self, center, yaw, stamp):
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(center[0])
        pose.pose.position.y = float(center[1])
        pose.pose.position.z = float(center[2])
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _surface_grasp_geometry(self, surface):
        if surface.axis_ratio <= self._cube_max_axis_ratio:
            return (
                'cube_30mm', self._cube_size.copy(),
                surface.square_edge_yaw_rad, math.pi * 0.5)
        if surface.axis_ratio >= self._rectangle_min_axis_ratio:
            return (
                'rectangular_box_60x30mm', self._rectangle_size.copy(),
                surface.long_axis_yaw_rad, math.pi)
        raise ValueError(
            f'object_shape_ambiguous:axis_ratio={surface.axis_ratio:.3f}')

    def _detected_target_entry(self, color_name, surface, tier):
        """Describe one physically valid cube for downstream stack planning.

        The P2 stack task needs the orientation of both the upper and the lower
        cube in the same frame, not only the one selected by the colour filter.
        Every entry here comes from the single RGB-D frame that also produced
        the selected target, so a consumer may treat them as one snapshot.
        """
        object_class, object_size, visual_yaw, yaw_period = \
            self._surface_grasp_geometry(surface)
        return {
            'color': color_name,
            'center_base_m': [float(value) for value in surface.center],
            'top_z_base_m': float(surface.center[2]),
            'grasp_axis_yaw_base_rad': float(visual_yaw),
            'grasp_axis_yaw_period_rad': float(yaw_period),
            'observed_short_long_m': [float(value) for value in surface.size_xy],
            'object_class': object_class,
            'nominal_object_size_m': [float(value) for value in object_size],
            'stack_tier': int(tier),
        }

    def _make_size_message(self, pose, object_size):
        message = Vector3Stamped()
        message.header = pose.header
        message.vector.x = float(object_size[0])
        message.vector.y = float(object_size[1])
        message.vector.z = float(object_size[2])
        return message

    def _accept_sample(
            self, pose, surface, candidate, object_class, object_size,
            yaw, yaw_period, detector_class=None, color_name=None):
        center = np.array([
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        stamp = stamp_seconds(pose)
        with self._lock:
            if self._samples and self._samples[-1][2][2:] != \
                    (object_class, detector_class, color_name):
                self._samples.clear()
            position_stable, standard_deviation = _update_sample_window(
                self._samples, stamp, center,
                (pose, surface, object_class, detector_class, color_name),
                self._stable_sample_max_gap, self._max_sample_jump,
                self._stable_samples, self._stable_xy_std, self._stable_z_std,
                yaw=yaw, maximum_yaw_jump=self._max_sample_yaw_jump,
                yaw_period=yaw_period)
            yaw_mean, yaw_std = _periodic_angle_mean_std(
                [sample[3] for sample in self._samples], period=yaw_period)
            stable = position_stable and yaw_std <= self._stable_yaw_std
            if stable:
                centers = np.asarray([sample[1] for sample in self._samples])
                median = np.median(centers, axis=0)
                pose.pose.position.x = float(median[0])
                pose.pose.position.y = float(median[1])
                pose.pose.position.z = float(median[2])
                pose.pose.orientation.z = math.sin(yaw_mean * 0.5)
                pose.pose.orientation.w = math.cos(yaw_mean * 0.5)
                self._stable_pose = pose
                self._stable_surface = surface
                self._stable_object_size = object_size.copy()
                self._stable_color = color_name
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
            'raw_center_base_m': [
                float(center[0]), float(center[1]),
                float(center[2] - self._top_pose_z_compensation)],
            'top_pose_z_compensation_m': self._top_pose_z_compensation,
            'observed_short_long_m': surface.size_xy.tolist(),
            'object_class': object_class,
            'color': color_name,
            'nominal_object_size_m': object_size.tolist(),
            'grasp_axis_yaw_base_rad': yaw_mean,
            'grasp_axis_yaw_period_rad': yaw_period,
            'grasp_axis_yaw_std_rad': yaw_std,
            'top_axis_ratio': surface.axis_ratio,
            'top_z_spread_m': surface.z_spread,
            'depth_valid_fraction': surface.depth_valid_fraction,
            'candidate_area_px': candidate.area_px,
            'position_std_m': standard_deviation.tolist(),
            'orientation_source': (
                ('top_surface_square_min_area_edge' if object_class == 'cube_30mm'
                 else 'top_surface_long_axis_pca') if self._visual_yaw_enabled
                else 'fixed_yaw_parameter'),
        }
        if stable:
            self._pose_pub.publish(pose)
            self._size_pub.publish(self._make_size_message(pose, object_size))
            self._marker_pub.publish(self._make_marker(pose, object_size, color_name))
            self.get_logger().info(
                f'Updated stable box top pose in {self._base_frame}: '
                f'[{pose.pose.position.x:.4f}, {pose.pose.position.y:.4f}, '
                f'{pose.pose.position.z:.4f}] m, class={object_class}, '
                f'grasp-axis yaw={yaw_mean:.3f} rad')
        return status

    def _make_marker(self, pose, object_size, color_name=None):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'blue_cube_localization'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z - object_size[2] * 0.5
        marker.pose.orientation = pose.pose.orientation
        marker.scale.x = object_size[0]
        marker.scale.y = object_size[1]
        marker.scale.z = object_size[2]
        marker.color.r, marker.color.g, marker.color.b = {
            'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.1),
            'blue': (0.05, 0.25, 1.0), 'yellow': (1.0, 0.9, 0.05),
        }.get(color_name, (0.05, 0.25, 1.0))
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

    def _set_status(self, update, expected_filter_generation=None):
        with self._lock:
            if expected_filter_generation is not None and \
                    expected_filter_generation != self._filter_generation:
                return
            self._last_status = {'detection_mode': self._detection_mode, **update}

    def _heartbeat(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        gate = self._inference_gate.snapshot()
        with self._lock:
            status = dict(self._last_status)
            pose = self._stable_pose
            object_size = self._stable_object_size
            color_name = self._stable_color
        status.update({
            'inference_enabled': gate.enabled,
            'active_inferences': gate.active_inferences,
            'pause_generation': gate.pause_generation,
            'frames_inferred': self._frames_processed,
            'frames_skipped_paused': gate.frames_skipped_paused,
            'last_inference_start_ns': gate.last_inference_start_ns,
            'last_inference_end_ns': gate.last_inference_end_ns,
            'last_success_ns': gate.last_success_ns,
            'error_count': self._processing_error_count,
            'npu_inference': self._detection_mode == 'yolo',
        })
        if not gate.enabled:
            status.update({
                'valid': False, 'stable': False, 'tracking': False,
                'has_stable_pose': False, 'reason': 'perception_paused',
            })
            pose = None
            object_size = None
        stream_stale = (
            self._last_frame_time is None or
            now - self._last_frame_time > self._max_age_s)
        if stream_stale and gate.enabled:
            status = {
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': pose is not None,
                'detection_mode': self._detection_mode,
                'target_filter_color': status.get('target_filter_color', ''),
                'filter_generation': status.get('filter_generation', 0),
                'reason': 'rgbd_stream_stale',
                'inference_enabled': gate.enabled,
                'active_inferences': gate.active_inferences,
                'pause_generation': gate.pause_generation,
                'frames_inferred': self._frames_processed,
                'frames_skipped_paused': gate.frames_skipped_paused,
                'last_inference_start_ns': gate.last_inference_start_ns,
                'last_inference_end_ns': gate.last_inference_end_ns,
                'last_success_ns': gate.last_success_ns,
                'error_count': self._processing_error_count,
                'npu_inference': self._detection_mode == 'yolo'}
        text = json.dumps(status, sort_keys=True, separators=(',', ':'))
        self._valid_pub.publish(Bool(data=bool(status.get('valid', False))))
        self._status_pub.publish(String(data=text))
        if pose is not None and object_size is not None:
            self._pose_pub.publish(pose)
            self._size_pub.publish(self._make_size_message(pose, object_size))
            self._marker_pub.publish(self._make_marker(pose, object_size, color_name))
        log_signature = json.dumps({
            key: status.get(key) for key in
            ('valid', 'stable', 'tracking', 'reason', 'samples', 'required_samples')
        }, sort_keys=True, separators=(',', ':'))
        if log_signature != self._last_status_text:
            self.get_logger().info(f'Localization status: {text}')
            self._last_status_text = log_signature

    def _reset_service(self, _request, response):
        with self._lock:
            self._target_color = None
            self._filter_generation += 1
            self._samples.clear()
            self._stable_pose = None
            self._stable_surface = None
            self._stable_object_size = None
            self._stable_color = None
            self._sort_skipped_targets.clear()
            self._last_processed_stamp = None
            self._last_status = {
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': False,
                'detection_mode': self._detection_mode,
                'target_filter_color': '',
                'filter_generation': self._filter_generation,
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

    def _set_inference_enabled_service(self, request, response):
        try:
            snapshot = self._inference_gate.set_enabled(
                bool(request.data), timeout_s=3.0)
        except (RuntimeError, TimeoutError, ValueError) as error:
            response.success = False
            response.message = f'Perception gate transition failed: {error}'
            return response
        if not request.data:
            with self._lock:
                self._samples.clear()
                self._stable_pose = None
                self._stable_surface = None
                self._stable_object_size = None
                self._stable_color = None
                self._last_processed_stamp = None
                self._last_status = {
                    'valid': False, 'stable': False, 'tracking': False,
                    'frozen': False, 'has_stable_pose': False,
                    'detection_mode': self._detection_mode,
                    'target_filter_color': self._target_color or '',
                    'filter_generation': self._filter_generation,
                    'reason': 'perception_paused',
                }
        else:
            with self._lock:
                self._samples.clear()
                self._last_processed_stamp = None
                self._last_status = {
                    'valid': False, 'stable': False, 'tracking': True,
                    'frozen': False, 'has_stable_pose': False,
                    'detection_mode': self._detection_mode,
                    'target_filter_color': self._target_color or '',
                    'filter_generation': self._filter_generation,
                    'reason': 'perception_resumed_waiting_for_fresh_rgbd',
                }
        response.success = True
        response.message = (
            f'inference_enabled={str(snapshot.enabled).lower()};'
            f'active={snapshot.active_inferences};'
            f'generation={snapshot.pause_generation};'
            f'npu_inference={str(self._detection_mode == "yolo").lower()}')
        return response

    def _set_target_color_service(self, request, response):
        color = str(request.color).strip().lower()
        allowed = set(SORT_COLOR_PRIORITY)
        if self._detection_mode != 'colors':
            response.success = False
            response.message = 'Color targeting requires detection_mode=colors.'
            with self._lock:
                response.generation = self._filter_generation
            return response
        if color and color not in allowed:
            response.success = False
            response.message = f'Unsupported target color: {color!r}.'
            with self._lock:
                response.generation = self._filter_generation
            return response
        with self._lock:
            self._target_color = color or None
            self._filter_generation += 1
            generation = self._filter_generation
            self._samples.clear()
            self._stable_pose = None
            self._stable_surface = None
            self._stable_object_size = None
            self._stable_color = None
            self._sort_skipped_targets.clear()
            self._last_processed_stamp = None
            self._last_status = {
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': False,
                'detection_mode': self._detection_mode,
                'target_filter_color': color,
                'filter_generation': generation,
                'reason': 'target_filter_changed_waiting_for_fresh_rgbd',
            }
        response.success = True
        response.generation = generation
        response.message = (
            f'Target filter generation {generation} set to {color}.'
            if color else f'Target filter generation {generation} cleared.')
        return response

    def _skip_current_sort_target_service(self, _request, response):
        with self._lock:
            if self._detection_mode != 'colors':
                response.success = False
                response.message = 'Skipping targets is only available in colors mode.'
                return response
            if self._stable_pose is None or self._stable_color is None:
                response.success = False
                response.message = 'No stable color target is available to skip.'
                return response
            center = np.array([
                self._stable_pose.pose.position.x,
                self._stable_pose.pose.position.y,
                self._stable_pose.pose.position.z,
            ], dtype=np.float64)
            self._sort_skipped_targets.append((self._stable_color, center))
            skipped_color = self._stable_color
            skipped_center = center.copy()
            self._samples.clear()
            self._stable_pose = None
            self._stable_surface = None
            self._stable_object_size = None
            self._stable_color = None
            self._last_status = {
                'valid': False, 'stable': False, 'tracking': True,
                'frozen': False, 'has_stable_pose': False,
                'detection_mode': self._detection_mode,
                'target_filter_color': self._target_color or '',
                'filter_generation': self._filter_generation,
                'reason': 'current_sort_target_skipped',
            }
        response.success = True
        response.message = (
            f'Skipped {skipped_color} target at '
            f'[{skipped_center[0]:.4f}, {skipped_center[1]:.4f}, '
            f'{skipped_center[2]:.4f}] m for this batch.')
        return response

    def destroy_node(self):
        try:
            self._inference_gate.close()
        except TimeoutError as error:
            self.get_logger().error(f'Perception gate close timed out: {error}')
        if self._yolo_detector is not None:
            self._yolo_detector.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BlueCubeLocalizer()
        rclpy.spin(node)
    except (RuntimeError, ValueError, FileNotFoundError) as error:
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
