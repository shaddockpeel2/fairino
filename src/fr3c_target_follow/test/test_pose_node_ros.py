"""Run explicitly in an isolated ROS domain with FR3C_RUN_ROS_TESTS=1."""
import json
import os
import time
from array import array

import cv2
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1',
                                reason='ROS integration is opt-in in an isolated domain')


def test_fresh_pose_loss_stale_input_and_recovery_over_ros():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from fr3c_target_follow.pose_node import TagPoseNode

    rclpy.init(args=['--ros-args', '-p', 'image_topic:=/test_camera/image',
                    '-p', 'camera_info_topic:=/test_camera/info'])
    measurement = TagPoseNode()
    probe = Node('target_pose_contract_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(measurement)
    executor.add_node(probe)
    poses, quality = [], []
    image_pub = probe.create_publisher(Image, '/test_camera/image', qos_profile_sensor_data)
    info_pub = probe.create_publisher(CameraInfo, '/test_camera/info', qos_profile_sensor_data)
    probe.create_subscription(PoseStamped, '/target_follow/tag_pose', poses.append, 10)
    probe.create_subscription(String, '/target_follow/quality', lambda m: quality.append(json.loads(m.data)), 10)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = (cv2.aruco.generateImageMarker(dictionary, 1, 100)
           if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dictionary, 1, 100))
    canvas = np.full((240, 320), 255, np.uint8)
    canvas[70:170, 110:210] = tag
    canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    def publish(image, age_s=0):
        stamp = (probe.get_clock().now() - rclpy.duration.Duration(seconds=age_s)).to_msg()
        info = CameraInfo()
        info.header.frame_id = 'test_color_optical'
        info.header.stamp = stamp
        info.width, info.height = 320, 240
        info.k = [480., 0., 160., 0., 480., 120., 0., 0., 1.]
        info.d = [0.] * 5
        info.distortion_model = 'plumb_bob'
        message = Image()
        message.header = info.header
        message.width, message.height = 320, 240
        message.encoding, message.step = 'bgr8', 320 * 3
        message.data = array('B', image.tobytes())
        info_pub.publish(info)
        spin_for(.05)
        image_pub.publish(message)
        deadline = time.monotonic() + 1.5
        stamp_s = stamp.sec + stamp.nanosec / 1e9
        while time.monotonic() < deadline and not any(abs(row.get('stamp_s', 0)-stamp_s) < 1e-7 for row in quality):
            executor.spin_once(timeout_sec=.02)
        spin_for(.05)
        return stamp

    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and (image_pub.get_subscription_count() == 0 or info_pub.get_subscription_count() == 0):
            executor.spin_once(timeout_sec=.02)
        assert image_pub.get_subscription_count() > 0 and info_pub.get_subscription_count() > 0
        stamp = publish(canvas)
        assert poses and quality[-1]['status'] == 'valid', quality
        assert poses[-1].header.stamp == stamp
        assert poses[-1].header.frame_id == 'test_color_optical'
        assert .09 < poses[-1].pose.position.z < .11
        q = poses[-1].pose.orientation
        assert np.linalg.norm([q.x, q.y, q.z, q.w]) == pytest.approx(1)
        count = len(poses)
        publish(np.full_like(canvas, 255))
        assert len(poses) == count and quality[-1]['status'] == 'target_not_found'
        publish(canvas, age_s=1)
        assert len(poses) == count and quality[-1]['status'] == 'stale_or_out_of_order_image'
        spin_for(.6)
        assert quality[-1]['status'] == 'no_fresh_image'
        publish(canvas)
        assert len(poses) == count + 1 and quality[-1]['status'] == 'valid', quality
        assert all(row['motion_commands_enabled'] is False for row in quality)
    finally:
        executor.shutdown()
        measurement.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
