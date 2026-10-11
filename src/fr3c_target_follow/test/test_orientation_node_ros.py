"""Isolated synthetic inputs; no camera, SDK connection, or motion owner."""
import json
import os
from pathlib import Path
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1', reason='isolated ROS domain required')
ROOT = Path(__file__).resolve().parents[3]


class OrientationHarness:
    def __init__(self, tmp_path):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster
        from fr3c_target_follow.follow_geometry import PoseHistory, quaternion_from_rotation, transform_rpy
        from fr3c_target_follow.orientation_node import OrientationCheck
        from fr3c_target_follow.preview_node import FollowPreview

        self.rclpy, self.Pose, self.String = rclpy, PoseStamped, String
        self.transform_rpy, self.quaternion = transform_rpy, quaternion_from_rotation
        tcp = tmp_path/'tcp.yaml'
        tcp.write_text('ydi_j22_tcp:\n  calibrated: true\n  parent_frame: flange_link\n  child_frame: grasp_frame\n  xyz_m: [0, 0, .1]\n  rpy_rad: [0, 0, 0]\n')
        rclpy.init(args=['--ros-args', '-p', 'tcp_file:='+str(tcp), '-p', 'transform_only:=true'])
        self.converter, self.monitor = FollowPreview(), OrientationCheck()
        self.probe = Node('orientation_contract_probe')
        self.executor = SingleThreadedExecutor()
        for node in (self.converter, self.monitor, self.probe):
            self.executor.add_node(node)
        self.rows, self.states, self.base_poses = [], [], []
        self.probe.create_subscription(String, '/target_follow/pose_observation',
                                       lambda m: self.rows.append(json.loads(m.data)), 50)
        self.probe.create_subscription(String, '/target_follow/orientation_state',
                                       lambda m: self.states.append(json.loads(m.data)), 50)
        self.probe.create_subscription(PoseStamped, '/target_follow/tag_pose_base', self.base_poses.append, 20)
        self.flange_pub = self.probe.create_publisher(PoseStamped, '/target_follow/flange_pose', 50)
        self.tag_pub = self.probe.create_publisher(PoseStamped, '/target_follow/tag_pose', 10)
        self.quality_pub = self.probe.create_publisher(String, '/target_follow/quality', 10)
        self.history = PoseHistory()
        self.robot_yaw = 0.
        self.handeye = transform_rpy([.01, -.02, .03], [.1, -.2, .4])
        self.target = transform_rpy([.12, -.35, .04], [.2, .1, .3])
        static = TransformStamped()
        static.header.frame_id, static.child_frame_id = 'flange_link', 'camera_color_optical_frame'
        static.header.stamp = self.probe.get_clock().now().to_msg()
        p, q = self.handeye[:3, 3], self.quaternion(self.handeye[:3, :3])
        static.transform.translation.x, static.transform.translation.y, static.transform.translation.z = map(float, p)
        (static.transform.rotation.x, static.transform.rotation.y,
         static.transform.rotation.z, static.transform.rotation.w) = map(float, q)
        self.static = StaticTransformBroadcaster(self.probe)
        self.static.sendTransform(static)
        self.timer = self.probe.create_timer(.008, self.robot)
        self.spin(.35)

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.005)

    def robot(self):
        matrix = self.transform_rpy([.1, -.2, .35], [np.pi, .1, self.robot_yaw])
        message = self.message(matrix, self.probe.get_clock().now().to_msg(), 'base_link')
        self.history.add(message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                         matrix[:3, 3], self.quaternion(matrix[:3, :3]))
        self.flange_pub.publish(message)

    def message(self, matrix, stamp, frame):
        message = self.Pose()
        message.header.stamp, message.header.frame_id = stamp, frame
        p, q = matrix[:3, 3], self.quaternion(matrix[:3, :3])
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, p)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, q)
        return message

    def observation(self, *, ambiguous=False, quality_first=False, offset_ns=0, publish_quality=True):
        from rclpy.duration import Duration
        stamp = (self.probe.get_clock().now()-Duration(seconds=.06)).to_msg()
        at_image, _ = self.history.at(stamp.sec+stamp.nanosec/1e9)
        optical = np.linalg.inv(at_image @ self.handeye) @ self.target
        pose = self.message(optical, stamp, 'camera_color_optical_frame')
        quality = dict(status='valid', target_id=1, frame_id='camera_color_optical_frame',
                       stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec+offset_ns), valid_position=True,
                       orientation_measurement_valid=not ambiguous, orientation_ambiguous=ambiguous,
                       solver='IPPE_SQUARE', candidate_count=2, candidate_errors_px=[.01, .6],
                       candidate_rotation_distance_deg=30., selected_candidate_index=0)
        if quality_first and publish_quality:
            self.quality_pub.publish(self.String(data=json.dumps(quality)))
            self.spin(.015)
        self.tag_pub.publish(pose)
        self.spin(.035)
        if not quality_first and publish_quality:
            self.quality_pub.publish(self.String(data=json.dumps(quality)))
        self.spin(.05)
        return stamp, quality

    def close(self, name):
        output = ROOT/'evidence/target_follow/stage6/ros_integration'/name
        output.mkdir(parents=True, exist_ok=True)
        samples = [dict(r, kind='observation') for r in self.rows] + [dict(r, kind='state') for r in self.states]
        (output/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in samples))
        from fr3c_target_follow.orientation_audit import orientation_report
        (output/'report.json').write_text(json.dumps(orientation_report(samples), indent=2)+'\n')
        self.executor.shutdown()
        self.converter.destroy_node()
        self.monitor.destroy_node()
        self.probe.destroy_node()
        self.rclpy.shutdown()


def test_eye_in_hand_base_rotation_quality_pairing_and_jump_recovery(tmp_path):
    h = OrientationHarness(tmp_path)
    try:
        for i in range(5):
            h.observation(quality_first=i % 2 == 0)
        assert h.rows[-1]['valid_orientation'] is True, h.rows
        original_q = h.rows[-1]['quaternion_xyzw']
        # Camera/flange rotates rapidly. The fixed base target must not look like a jump.
        for i in range(1, 7):
            h.robot_yaw = i*.2
            h.spin(.09)
            h.observation()
            assert h.rows[-1]['valid_orientation'], h.rows[-1]
            assert abs(np.dot(h.rows[-1]['quaternion_xyzw'], original_q)) == pytest.approx(1, abs=1e-10)
        stamp, _ = h.observation(ambiguous=True)
        assert h.rows[-1]['stamp'] == dict(sec=stamp.sec, nanosec=stamp.nanosec)
        assert h.rows[-1]['valid_position'] and not h.rows[-1]['valid_orientation']
        assert h.rows[-1]['status'] == 'orientation_ambiguous'
        for i in range(5):
            h.observation()
            assert h.rows[-1]['valid_orientation'] is (i == 4)
        saved = h.target.copy()
        h.target = h.transform_rpy(h.target[:3, 3], [.2, .1, 2.8])
        h.observation()
        assert h.rows[-1]['status'] == 'orientation_jump'
        h.target = saved
        for _ in range(5):
            h.observation()
        assert h.rows[-1]['valid_orientation']
        assert all(not r['motion_commands_enabled'] and not r['orientation_control_enabled'] for r in h.rows)
    finally:
        h.close('base_rotation_recovery')


def test_wrong_nanosecond_legacy_quality_missing_data_and_feedback_timeout(tmp_path):
    h = OrientationHarness(tmp_path)
    try:
        h.observation(offset_ns=1)
        assert not any(r.get('position_m') for r in h.rows)
        assert h.monitor.pairer.paired == 0
        for _ in range(5):
            _, quality = h.observation()
        assert h.rows[-1]['valid_orientation']
        future = dict(quality, stamp=dict(sec=h.probe.get_clock().now().to_msg().sec+10, nanosec=0),
                      orientation_ambiguous=True)
        watermark = h.monitor.watermark
        h.quality_pub.publish(h.String(data=json.dumps(future)))
        h.spin(.06)
        assert h.states[-1]['status'] == 'stale_quality' and h.monitor.watermark == watermark
        for _ in range(5):
            _, quality = h.observation()
        assert h.rows[-1]['valid_orientation']
        paired = h.monitor.pairer.paired
        # An old duplicate rejection cannot revoke a newer paired observation.
        h.quality_pub.publish(h.String(data=json.dumps(dict(quality, orientation_ambiguous=True))))
        h.spin(.03)
        assert h.states[-1]['valid_orientation']
        del quality['stamp']
        quality['stamp_s'] = h.probe.get_clock().now().nanoseconds/1e9
        h.quality_pub.publish(h.String(data=json.dumps(quality)))
        h.spin(.08)
        assert h.states[-1]['status'] == 'missing_exact_image_stamp'
        for _ in range(5):
            h.observation()
        assert h.rows[-1]['valid_orientation']
        h.observation(publish_quality=False)
        assert h.monitor.pairer.paired == paired+5
        h.spin(.4)
        assert h.states[-1]['status'] == 'stale_target' and not h.states[-1]['valid_orientation']
        for _ in range(5):
            h.observation()
        assert h.rows[-1]['valid_orientation']
        h.timer.cancel()
        deadline = time.monotonic()+.4
        while h.states[-1]['status'] != 'stale_robot_feedback' and time.monotonic() < deadline:
            h.spin(.02)
        assert h.states[-1]['status'] == 'stale_robot_feedback'
        assert not h.states[-1]['valid_position'] and not h.states[-1]['valid_orientation']
    finally:
        h.close('pairing_timeouts')
