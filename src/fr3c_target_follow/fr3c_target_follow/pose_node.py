"""Publish fresh top-label position AND orientation; no robot command interface."""
import json
import math
import time
from array import array

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from fr3c_camera.rgbd import image_array, stamp_seconds
from .marker_pose import (MarkerDetector, camera_parameters, rpy_degrees,
                          solve_marker_pose, square_points, stamp_is_fresh)


class TagPoseNode(Node):
    def __init__(self):
        super().__init__('tag_pose')
        defaults = dict(dictionary='DICT_APRILTAG_36h11', target_id=1,
                        marker_size_m=0.020, cube_edge_m=0.030,
                        image_topic='/camera/color/image_raw',
                        camera_info_topic='/camera/color/camera_info',
                        max_image_age_s=0.25, min_edge_px=12.0,
                        max_reprojection_error_px=2.0, ambiguity_margin_px=0.15,
                        continuity_margin_px=0.20, publish_tag_tf=True,
                        adaptive_thresh_constant=13.0, adaptive_thresh_step=4,
                        fallback_thresh_constants=[21.0,25.0],
                        publish_annotated_image=True)
        self.options = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        square_points(self.options['marker_size_m'])
        for key in ('max_image_age_s', 'min_edge_px', 'max_reprojection_error_px'):
            if not math.isfinite(self.options[key]) or self.options[key] <= 0:
                raise ValueError(key + ' must be positive')
        self.detector = MarkerDetector(self.options['dictionary'],
                                       self.options['adaptive_thresh_constant'], self.options['adaptive_thresh_step'],
                                       self.options['fallback_thresh_constants'], self.options['target_id'])
        self.latest = None
        self.camera_info = None
        self.previous_rotation = None
        self.last_valid_received = 0.0
        self.last_received = time.monotonic()
        self.last_status = 0.0
        self.last_stamp = 0.0
        self.total = self.valid = 0
        self.received_frames = self.overwritten_frames = 0
        # Standard sensor DDS history permits large fragmented RGB samples to finish.
        # The application still stores only the newest frame, never a processing queue.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.options['image_topic'], self.receive_image, qos)
        self.create_subscription(CameraInfo, self.options['camera_info_topic'], self.receive_info, qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_follow/tag_pose', 1)
        self.quality_pub = self.create_publisher(String, '/target_follow/quality', 10)
        self.image_pub = self.create_publisher(Image, '/target_follow/image_annotated', qos)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.01, self.process_latest)
        self.get_logger().info(
            f"Measuring {self.options['dictionary']} id={self.options['target_id']}, "
            f"edge={self.options['marker_size_m'] * 1000:.1f} mm; OpenCV={cv2.__version__}. "
            'Position + orientation are data only; no robot motion.')

    def receive_image(self, message):
        self.received_frames += 1
        if self.latest is not None:
            self.overwritten_frames += 1
        self.last_received = time.monotonic()
        self.latest = (message, self.last_received)

    def receive_info(self, message):
        self.camera_info = message

    def publish_quality(self, status, message=None, **values):
        now = time.monotonic()
        self.last_status = now
        data = dict(status=status, dictionary=self.options['dictionary'],
                    target_id=self.options['target_id'], marker_size_m=self.options['marker_size_m'],
                    cube_edge_m=self.options['cube_edge_m'], processed_frames=self.total,
                    valid_frames=self.valid, motion_commands_enabled=False,
                    received_frames=self.received_frames, overwritten_frames=self.overwritten_frames,
                    pose_origin='top_label_center', **values)
        if message is not None:
            data.update(frame_id=message.header.frame_id, stamp_s=stamp_seconds(message))
        self.quality_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def process_latest(self):
        monotonic_now = time.monotonic()
        if self.latest is None:
            if monotonic_now - self.last_received > self.options['max_image_age_s'] and monotonic_now - self.last_status > 0.5:
                self.previous_rotation = None
                self.publish_quality('no_fresh_image')
            return
        message, received = self.latest
        self.latest = None
        self.total += 1
        stamp = stamp_seconds(message)
        now = self.get_clock().now().nanoseconds / 1e9
        if stamp <= self.last_stamp or not stamp_is_fresh(stamp, now, self.options['max_image_age_s']):
            self.publish_quality('stale_or_out_of_order_image', message, image_age_ms=(now - stamp) * 1000)
            return
        self.last_stamp = stamp
        if monotonic_now - self.last_valid_received > self.options['max_image_age_s']:
            self.previous_rotation = None
        start = time.perf_counter()
        bgr = None
        pose = None
        status = 'valid'
        values = {}
        try:
            if self.camera_info is None:
                raise ValueError('waiting_for_camera_info')
            bgr = image_array(message)
            bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR) if message.encoding == 'rgb8' else bgr.copy()
            k, d = camera_parameters(self.camera_info, bgr.shape, message.header.frame_id)
            corners, ids, rejected = self.detector.detect(bgr)
            values['detector_threshold'] = self.detector.last_threshold
            values['detector_passes'] = self.detector.last_pass_count
            values['detected_ids'] = [] if ids is None else ids.flatten().tolist()
            matches = [] if ids is None else np.flatnonzero(ids.flatten() == self.options['target_id']).tolist()
            if len(matches) != 1:
                raise ValueError('target_not_found' if not matches else 'duplicate_target_id')
            selected = corners[matches[0]]
            values['corners_px'] = selected.reshape(4, 2).tolist()
            pose = solve_marker_pose(
                selected, self.options['marker_size_m'], k, d,
                min_edge_px=self.options['min_edge_px'], max_error_px=self.options['max_reprojection_error_px'],
                previous_rotation=self.previous_rotation,
                continuity_margin_px=self.options['continuity_margin_px'],
                ambiguity_margin_px=self.options['ambiguity_margin_px'])
            finished_age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if finished_age > self.options['max_image_age_s']:
                raise ValueError('image_expired_during_processing')
            self.previous_rotation = pose.rotation
            self.last_valid_received = received
            self.valid += 1
            output = PoseStamped()
            output.header = message.header
            output.pose.position.x, output.pose.position.y, output.pose.position.z = pose.position_m.tolist()
            output.pose.orientation.x, output.pose.orientation.y, output.pose.orientation.z, output.pose.orientation.w = pose.quaternion.tolist()
            self.pose_pub.publish(output)
            if self.options['publish_tag_tf']:
                transform = TransformStamped()
                transform.header = message.header
                transform.child_frame_id = 'follow_tag_' + str(self.options['target_id'])
                transform.transform.translation.x = output.pose.position.x
                transform.transform.translation.y = output.pose.position.y
                transform.transform.translation.z = output.pose.position.z
                transform.transform.rotation = output.pose.orientation
                self.tf.sendTransform(transform)
            values.update(position_m=pose.position_m.tolist(), quaternion_xyzw=pose.quaternion.tolist(),
                          rpy_deg=rpy_degrees(pose.rotation), distance_m=float(np.linalg.norm(pose.position_m)),
                          reprojection_error_px=pose.reprojection_error_px, min_edge_px=pose.min_edge_px,
                          candidate_count=pose.candidate_count, candidate_margin_px=pose.candidate_margin_px,
                          orientation_ambiguous=pose.orientation_ambiguous,
                          solver=pose.solver,
                          image_age_ms=finished_age * 1000)
            if self.options['publish_annotated_image']:
                cv2.aruco.drawDetectedMarkers(bgr, [selected], np.array([[self.options['target_id']]], np.int32))
                cv2.drawFrameAxes(bgr, k, d, pose.rvec, pose.position_m, self.options['marker_size_m'] / 2)
                xyz = pose.position_m * 1000
                cv2.putText(bgr, f'id={self.options["target_id"]} XYZ mm: {xyz[0]:.1f} {xyz[1]:.1f} {xyz[2]:.1f}',
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 230, 30), 1, cv2.LINE_AA)
                cv2.putText(bgr, f'error={pose.reprojection_error_px:.2f}px orientation ambiguous={pose.orientation_ambiguous}',
                            (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 230, 30), 1, cv2.LINE_AA)
                rpy = values['rpy_deg']
                cv2.putText(bgr, f'RPY deg: {rpy[0]:.1f} {rpy[1]:.1f} {rpy[2]:.1f} | DATA ONLY',
                            (10, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 230, 30), 1, cv2.LINE_AA)
        except (ValueError, cv2.error) as exc:
            status = str(exc)
            pose = None
        values['processing_ms'] = (time.perf_counter() - start) * 1000
        if bgr is not None and self.options['publish_annotated_image']:
            if pose is None:
                cv2.putText(bgr, status[:70], (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            image = Image()
            image.header = message.header
            image.height, image.width = bgr.shape[:2]
            image.encoding = 'bgr8'
            image.step = image.width * 3
            # Passing bytes triggers two Python loops over ~921k pixels in Humble.
            # The array('B') setter is constant-time and preserves the same samples.
            image.data = array('B', np.ascontiguousarray(bgr).tobytes())
            self.image_pub.publish(image)
        values['callback_ms'] = (time.perf_counter() - start) * 1000
        self.publish_quality(status, message, **values)


def main(args=None):
    cv2.setNumThreads(2)
    rclpy.init(args=args)
    node = None
    try:
        node = TagPoseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
