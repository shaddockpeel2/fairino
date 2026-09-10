"""Interactive, read-only ChArUco hand-eye collection and solving.

The node has no command publishers or motion services. The operator moves the
robot manually, waits for it to stop, then types ``capture``.
"""
import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import select
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import yaml

from .geometry import (
    average_transform,
    handeye_residuals,
    matrix_to_rpy,
    quaternion_to_rotation,
    rodrigues_transform,
    rotation_distance_deg,
    transform_matrix,
    transform_spread,
)


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def tf_to_matrix(transform):
    value = transform.transform
    return transform_matrix(
        quaternion_to_rotation(value.rotation.x, value.rotation.y,
                               value.rotation.z, value.rotation.w),
        [value.translation.x, value.translation.y, value.translation.z],
    )


def json_matrix(matrix):
    return [[float(value) for value in row] for row in np.asarray(matrix)]


class HandEyeCollector(Node):
    def __init__(self, options):
        super().__init__('fr3c_handeye_collector')
        self.options = options
        self.camera_info = None
        self.observations = deque(maxlen=max(60, options.capture_frames * 4))
        self.frame_count = 0
        self.rejects = {}

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard(
            (options.squares_x, options.squares_y),
            options.square_length_m, options.marker_length_m, dictionary)
        self.board.setLegacyPattern(True)
        detector_parameters = cv2.aruco.DetectorParameters()
        detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        charuco_parameters = cv2.aruco.CharucoParameters()
        self.detector = cv2.aruco.CharucoDetector(
            self.board, charuco_parameters, detector_parameters)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, options.camera_info_topic,
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, options.image_topic,
                                 self.on_image, qos_profile_sensor_data)

    def reject(self, reason):
        self.rejects[reason] = self.rejects.get(reason, 0) + 1

    def on_camera_info(self, message):
        self.camera_info = message

    def on_image(self, message):
        self.frame_count += 1
        info = self.camera_info
        if info is None:
            self.reject('no_camera_info')
            return
        if (message.width, message.height) != (info.width, info.height):
            self.reject('camera_info_size_mismatch')
            return
        if message.header.frame_id != self.options.optical_frame:
            self.reject('unexpected_image_frame')
            return
        if abs(stamp_ns(message) - stamp_ns(info)) > self.options.max_info_skew_ms * 1e6:
            self.reject('camera_info_timestamp_skew')
            return
        try:
            image = self.decode_image(message)
            charuco_corners, charuco_ids, marker_corners, marker_ids = \
                self.detector.detectBoard(image)
        except (ValueError, cv2.error) as exc:
            self.reject(f'detection:{exc}')
            return
        corner_count = 0 if charuco_ids is None else len(charuco_ids)
        marker_count = 0 if marker_ids is None else len(marker_ids)
        if corner_count < self.options.min_corners:
            self.reject('too_few_charuco_corners')
            return
        try:
            base_from_wrist = tf_to_matrix(self.tf_buffer.lookup_transform(
                self.options.base_frame, self.options.wrist_frame,
                Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.1)))
            camera_from_optical = tf_to_matrix(self.tf_buffer.lookup_transform(
                self.options.camera_frame, self.options.optical_frame,
                Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.1)))
        except Exception as exc:  # tf2 exceptions differ across ROS 2 patch releases.
            self.reject(f'tf:{type(exc).__name__}')
            return

        object_points, image_points = self.board.matchImagePoints(
            charuco_corners, charuco_ids)
        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64)
        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, camera_matrix, distortion,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not success:
                self.reject('solvepnp_failed')
                return
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points, image_points, camera_matrix, distortion, rvec, tvec)
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec, camera_matrix, distortion)
        except cv2.error as exc:
            self.reject(f'pnp:{exc}')
            return
        residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
        reprojection_rms_px = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if reprojection_rms_px > self.options.max_reprojection_rms_px:
            self.reject('reprojection_rms_too_high')
            return
        optical_from_board = rodrigues_transform(rvec, tvec)
        camera_from_board = camera_from_optical @ optical_from_board
        self.observations.append({
            'sequence': self.frame_count,
            'stamp_ns': stamp_ns(message),
            'corners': int(corner_count),
            'markers': int(marker_count),
            'reprojection_rms_px': reprojection_rms_px,
            'base_from_wrist': base_from_wrist,
            'camera_from_board': camera_from_board,
            'optical_from_board': optical_from_board,
            # Retained only in the bounded in-memory window. Session.capture()
            # writes one raw and one annotated audit image, never this array to JSON.
            'image': image,
            'charuco_corners': charuco_corners,
            'charuco_ids': charuco_ids,
            'marker_corners': marker_corners,
            'marker_ids': marker_ids,
        })

    @staticmethod
    def decode_image(message):
        encoding = message.encoding.lower()
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        if encoding in ('rgb8', 'bgr8'):
            image = array[:, :message.width * 3].reshape(message.height, message.width, 3)
            return (cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    if encoding == 'rgb8' else image.copy())
        if encoding in ('mono8', '8uc1'):
            return array[:, :message.width].copy()
        raise ValueError(f'unsupported image encoding: {message.encoding}')


class Session:
    def __init__(self, node, options, output_dir, resume=False):
        self.node = node
        self.options = options
        self.output_dir = output_dir
        self.samples_path = output_dir / 'samples.json'
        if resume:
            document = json.loads(self.samples_path.read_text())
            self.samples = document.get('samples', [])
            self.created_at = document['created_at']
            self.validate_resume(document)
        else:
            self.samples = []
            self.created_at = datetime.now().astimezone().isoformat()
            output_dir.mkdir(parents=True, exist_ok=False)
            self.save_samples()

    def validate_resume(self, document):
        expected = self.metadata()
        for key in ('camera_serial', 'mount_version', 'frames', 'topics', 'board'):
            if document.get(key) != expected.get(key):
                raise ValueError(f'resume metadata mismatch: {key}')
        previous = document.get('camera_info')
        current = expected.get('camera_info')
        if (not previous or not current or
                previous.get('width') != current.get('width') or
                previous.get('height') != current.get('height') or
                previous.get('distortion_model') != current.get('distortion_model') or
                not np.allclose(previous.get('k'), current.get('k'), atol=1e-12, rtol=0.0) or
                not np.allclose(previous.get('d'), current.get('d'), atol=1e-12, rtol=0.0)):
            raise ValueError('resume metadata mismatch: camera_info')
        expected_indices = list(range(1, len(self.samples) + 1))
        if [sample.get('index') for sample in self.samples] != expected_indices:
            raise ValueError('resume sample indices are not contiguous')

    def metadata(self):
        info = self.node.camera_info
        return {
            'schema_version': 1,
            'created_at': self.created_at,
            'camera_serial': self.options.camera_serial,
            'mount_version': self.options.mount_version,
            'frames': {
                'base': self.options.base_frame,
                'wrist': self.options.wrist_frame,
                'camera': self.options.camera_frame,
                'optical': self.options.optical_frame,
            },
            'topics': {
                'image': self.options.image_topic,
                'camera_info': self.options.camera_info_topic,
            },
            'board': {
                'type': 'charuco',
                'squares_x': self.options.squares_x,
                'squares_y': self.options.squares_y,
                'square_length_m': self.options.square_length_m,
                'marker_length_m': self.options.marker_length_m,
                'dictionary': 'DICT_4X4_50',
                'legacy_pattern': True,
            },
            'camera_info': None if info is None else {
                'width': info.width,
                'height': info.height,
                'distortion_model': info.distortion_model,
                'k': [float(value) for value in info.k],
                'd': [float(value) for value in info.d],
            },
            'capture_policy': {
                'frames_per_pose': self.options.capture_frames,
                'min_charuco_corners': self.options.min_corners,
                'max_reprojection_rms_px': self.options.max_reprojection_rms_px,
                'max_wrist_translation_spread_mm': self.options.max_wrist_translation_spread_mm,
                'max_wrist_rotation_spread_deg': self.options.max_wrist_rotation_spread_deg,
                'max_board_translation_spread_mm': self.options.max_board_translation_spread_mm,
                'max_board_rotation_spread_deg': self.options.max_board_rotation_spread_deg,
            },
        }

    def save_samples(self):
        document = self.metadata()
        document['samples'] = self.samples
        self.samples_path.write_text(json.dumps(document, indent=2) + '\n')

    def capture(self):
        start_sequence = self.node.frame_count
        observations = []
        deadline = time.monotonic() + self.options.capture_timeout_s
        while rclpy.ok() and len(observations) < self.options.capture_frames:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            observations = [value for value in self.node.observations
                            if value['sequence'] > start_sequence]
            if time.monotonic() >= deadline:
                break
        if len(observations) < self.options.capture_frames:
            return False, {
                'reason': 'capture_timeout',
                'valid_frames': len(observations),
                'required_frames': self.options.capture_frames,
                'rejects': self.node.rejects,
            }
        observations = observations[:self.options.capture_frames]
        base_values = [value['base_from_wrist'] for value in observations]
        camera_values = [value['camera_from_board'] for value in observations]
        base_mean, base_translation, base_rotation = transform_spread(base_values)
        camera_mean, camera_translation, camera_rotation = transform_spread(camera_values)
        report = {
            'frames': len(observations),
            'corners_min': min(value['corners'] for value in observations),
            'markers_min': min(value['markers'] for value in observations),
            'reprojection_rms_px_median': float(np.median([
                value['reprojection_rms_px'] for value in observations])),
            'reprojection_rms_px_max': float(np.max([
                value['reprojection_rms_px'] for value in observations])),
            'wrist_translation_spread_max_mm': float(np.max(base_translation)),
            'wrist_rotation_spread_max_deg': float(np.max(base_rotation)),
            'board_translation_spread_max_mm': float(np.max(camera_translation)),
            'board_rotation_spread_max_deg': float(np.max(camera_rotation)),
        }
        checks = {
            'wrist_translation_stable': report['wrist_translation_spread_max_mm']
                <= self.options.max_wrist_translation_spread_mm,
            'wrist_rotation_stable': report['wrist_rotation_spread_max_deg']
                <= self.options.max_wrist_rotation_spread_deg,
            'board_translation_stable': report['board_translation_spread_max_mm']
                <= self.options.max_board_translation_spread_mm,
            'board_rotation_stable': report['board_rotation_spread_max_deg']
                <= self.options.max_board_rotation_spread_deg,
        }
        report['checks'] = checks
        if not all(checks.values()):
            report['reason'] = 'stationarity_gate_failed'
            return False, report
        sample_number = len(self.samples) + 1
        image_dir = self.output_dir / 'images'
        image_dir.mkdir(exist_ok=True)
        representative = observations[len(observations) // 2]
        raw_name = f'sample_{sample_number:03d}_rgb.png'
        detected_name = f'sample_{sample_number:03d}_detected.png'
        annotated = representative['image'].copy()
        if representative['marker_ids'] is not None:
            cv2.aruco.drawDetectedMarkers(
                annotated, representative['marker_corners'], representative['marker_ids'])
        cv2.aruco.drawDetectedCornersCharuco(
            annotated, representative['charuco_corners'], representative['charuco_ids'])
        if (not cv2.imwrite(str(image_dir / raw_name), representative['image']) or
                not cv2.imwrite(str(image_dir / detected_name), annotated)):
            report['reason'] = 'audit_image_write_failed'
            return False, report
        sample = {
            'index': sample_number,
            'role': 'training' if sample_number <= self.options.training_samples else 'validation',
            'stamp_ns_first': observations[0]['stamp_ns'],
            'stamp_ns_last': observations[-1]['stamp_ns'],
            'quality': report,
            'images': {
                'rgb': f'images/{raw_name}',
                'detected': f'images/{detected_name}',
            },
            'base_from_wrist': json_matrix(base_mean),
            'camera_from_board': json_matrix(camera_mean),
            'raw_frames': [{
                'stamp_ns': value['stamp_ns'],
                'corners': value['corners'],
                'markers': value['markers'],
                'reprojection_rms_px': value['reprojection_rms_px'],
                'base_from_wrist': json_matrix(value['base_from_wrist']),
                'camera_from_board': json_matrix(value['camera_from_board']),
            } for value in observations],
        }
        self.samples.append(sample)
        self.save_samples()
        return True, sample

    def solve(self):
        training = self.samples[:self.options.training_samples]
        validation = self.samples[self.options.training_samples:]
        if len(training) < self.options.min_solve_samples:
            return False, {'reason': 'too_few_training_samples',
                           'training': len(training),
                           'required': self.options.min_solve_samples}
        base = [np.asarray(sample['base_from_wrist']) for sample in training]
        camera = [np.asarray(sample['camera_from_board']) for sample in training]
        methods = {
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        results = []
        for name, method in methods.items():
            try:
                rotation, translation = cv2.calibrateHandEye(
                    [matrix[:3, :3] for matrix in base],
                    [matrix[:3, 3].reshape(3, 1) for matrix in base],
                    [matrix[:3, :3] for matrix in camera],
                    [matrix[:3, 3].reshape(3, 1) for matrix in camera],
                    method=method)
                wrist_from_camera = transform_matrix(rotation, translation)
                if not np.isfinite(wrist_from_camera).all():
                    continue
                reference, train_translation, train_rotation = handeye_residuals(
                    base, wrist_from_camera, camera)
                if validation:
                    validation_base = [np.asarray(sample['base_from_wrist']) for sample in validation]
                    validation_camera = [np.asarray(sample['camera_from_board']) for sample in validation]
                    _, val_translation, val_rotation = handeye_residuals(
                        validation_base, wrist_from_camera, validation_camera, reference)
                else:
                    val_translation = np.asarray([])
                    val_rotation = np.asarray([])
                results.append({
                    'method': name,
                    'wrist_from_camera': wrist_from_camera,
                    'base_from_board_training_reference': reference,
                    'training_translation_median_mm': float(np.median(train_translation)),
                    'training_translation_p95_mm': float(np.percentile(train_translation, 95)),
                    'training_rotation_median_deg': float(np.median(train_rotation)),
                    'training_rotation_p95_deg': float(np.percentile(train_rotation, 95)),
                    'validation_translation_median_mm': (None if not len(val_translation)
                                                        else float(np.median(val_translation))),
                    'validation_translation_p95_mm': (None if not len(val_translation)
                                                     else float(np.percentile(val_translation, 95))),
                    'validation_rotation_median_deg': (None if not len(val_rotation)
                                                       else float(np.median(val_rotation))),
                    'validation_rotation_p95_deg': (None if not len(val_rotation)
                                                    else float(np.percentile(val_rotation, 95))),
                })
            except cv2.error:
                continue
        if not results:
            return False, {'reason': 'all_handeye_methods_failed'}
        results.sort(key=lambda value: (
            value['validation_translation_p95_mm']
            if value['validation_translation_p95_mm'] is not None
            else value['training_translation_p95_mm']))
        best = results[0]

        max_pair_rotation = max(
            rotation_distance_deg(first[:3, :3], second[:3, :3])
            for index, first in enumerate(base) for second in base[index + 1:])
        translations = np.asarray([matrix[:3, 3] for matrix in base])
        translation_extent_mm = (np.max(translations, axis=0) - np.min(translations, axis=0)) * 1000.0
        accepted = (
            len(training) >= self.options.training_samples
            and len(validation) >= self.options.validation_samples
            and best['validation_translation_median_mm'] <= 3.0
            and best['validation_translation_p95_mm'] <= 8.0
            and max_pair_rotation >= self.options.min_orientation_span_deg
        )
        report = {
            'accepted': bool(accepted),
            'training_samples': len(training),
            'validation_samples': len(validation),
            'required_training_samples': self.options.training_samples,
            'required_validation_samples': self.options.validation_samples,
            'max_pairwise_wrist_rotation_deg': float(max_pair_rotation),
            'wrist_translation_extent_xyz_mm': [float(value) for value in translation_extent_mm],
            'selected_method': best['method'],
            'methods': [],
        }
        for result in results:
            serializable = dict(result)
            serializable['wrist_from_camera'] = json_matrix(result['wrist_from_camera'])
            serializable['base_from_board_training_reference'] = json_matrix(
                result['base_from_board_training_reference'])
            report['methods'].append(serializable)
        (self.output_dir / 'solve_report.json').write_text(json.dumps(report, indent=2) + '\n')
        calibration = {
            'calibrated': bool(accepted),
            'status': 'accepted' if accepted else 'candidate_only',
            'parent_frame': self.options.wrist_frame,
            'child_frame': self.options.camera_frame,
            'xyz': [float(value) for value in best['wrist_from_camera'][:3, 3]],
            'rpy': [float(value) for value in matrix_to_rpy(best['wrist_from_camera'][:3, :3])],
            'camera_serial': self.options.camera_serial,
            'mount_version': self.options.mount_version,
            'image_size': [self.node.camera_info.width, self.node.camera_info.height],
            'board': self.metadata()['board'],
            'method': best['method'],
            'session': str(self.output_dir),
        }
        output_prefix = ('flange_to_dabai_dcw2'
                         if self.options.wrist_frame == 'flange_link'
                         else 'wrist3_to_dabai_dcw2')
        output_name = (f'{output_prefix}.yaml' if accepted
                       else f'{output_prefix}.candidate.yaml')
        (self.output_dir / output_name).write_text(
            yaml.safe_dump(calibration, sort_keys=False))
        return True, report


def print_status(node, session):
    latest = node.observations[-1] if node.observations else None
    payload = {
        'samples': len(session.samples),
        'training': min(len(session.samples), session.options.training_samples),
        'validation': max(0, len(session.samples) - session.options.training_samples),
        'valid_observations_buffered': len(node.observations),
        'latest': None if latest is None else {
            'corners': latest['corners'],
            'markers': latest['markers'],
            'reprojection_rms_px': latest['reprojection_rms_px'],
            'board_distance_m': float(np.linalg.norm(latest['optical_from_board'][:3, 3])),
        },
        'rejects': node.rejects,
    }
    print(json.dumps(payload, indent=2), flush=True)


def add_common_arguments(parser):
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/color/camera_info')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument(
        '--wrist-frame', default='flange_link',
        help='robot end frame used by hand-eye calibration (default: flange_link)')
    parser.add_argument('--camera-frame', default='camera_link')
    parser.add_argument('--optical-frame', default='camera_color_optical_frame')
    parser.add_argument('--camera-serial', default='CH82C5201C9')
    parser.add_argument('--mount-version', default='final_mount_v1')
    parser.add_argument('--squares-x', type=int, default=11)
    parser.add_argument('--squares-y', type=int, default=8)
    parser.add_argument('--square-length-m', type=float, default=0.015)
    parser.add_argument('--marker-length-m', type=float, default=0.011)
    parser.add_argument('--min-corners', type=int, default=20)
    parser.add_argument('--max-reprojection-rms-px', type=float, default=1.0)
    parser.add_argument('--max-info-skew-ms', type=float, default=20.0)
    parser.add_argument('--capture-frames', type=int, default=12)
    parser.add_argument('--capture-timeout-s', type=float, default=8.0)
    parser.add_argument('--max-wrist-translation-spread-mm', type=float, default=0.5)
    parser.add_argument('--max-wrist-rotation-spread-deg', type=float, default=0.1)
    parser.add_argument('--max-board-translation-spread-mm', type=float, default=1.0)
    parser.add_argument('--max-board-rotation-spread-deg', type=float, default=0.5)
    parser.add_argument('--training-samples', type=int, default=25)
    parser.add_argument('--validation-samples', type=int, default=10)
    parser.add_argument('--min-solve-samples', type=int, default=8)
    parser.add_argument('--min-orientation-span-deg', type=float, default=25.0)


def validate_options(parser, options):
    if not 0 < options.marker_length_m < options.square_length_m:
        parser.error('marker length must be positive and smaller than square length')
    if options.capture_frames < 3 or options.min_corners < 4:
        parser.error('capture-frames must be >= 3 and min-corners must be >= 4')


def wait_until_ready(node, timeout_s=12.0):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and (node.camera_info is None or not node.observations):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() >= deadline:
            return False
    return True


def main(args=None):
    parser = argparse.ArgumentParser()
    add_common_arguments(parser)
    parser.add_argument('--output-root', default='evidence/P13')
    parser.add_argument('--resume', help='existing P13 session directory to resume')
    options = parser.parse_args(args)
    validate_options(parser, options)

    rclpy.init()
    node = HandEyeCollector(options)
    if not wait_until_ready(node):
        print_status(node, type('Empty', (), {'samples': [], 'options': options})())
        raise SystemExit('camera/TF/ChArUco readiness timeout')
    if options.resume:
        session = Session(node, options, Path(options.resume).resolve(), resume=True)
    else:
        session_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        session = Session(node, options, Path(options.output_root) / session_name)
    print(f'READY session={session.output_dir}', flush=True)
    print('Commands: capture | status | solve | quit', flush=True)
    print_status(node, session)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                continue
            command = sys.stdin.readline().strip().lower()
            if command == 'capture':
                success, result = session.capture()
                print(('CAPTURED' if success else 'REJECTED'),
                      json.dumps(result, indent=2), flush=True)
            elif command == 'status':
                print_status(node, session)
            elif command == 'solve':
                success, result = session.solve()
                print(('SOLVED' if success else 'SOLVE_FAILED'),
                      json.dumps(result, indent=2), flush=True)
            elif command in ('quit', 'exit'):
                break
            elif command:
                print('Unknown command. Use: capture | status | solve | quit', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
