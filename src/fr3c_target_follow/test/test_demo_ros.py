"""Unified Demo lifecycle in an isolated ROS domain, using only MockClient."""
from array import array
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import cv2
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1',
                                reason='explicit isolated mock ROS run only')
ROOT = Path(__file__).resolve().parents[3]


class Demo:
    def __init__(self, tmp_path, name, detector=True, occupied=False, use_default_params=False,
                 collision_enabled=None):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster
        from fr3c_target_follow.follow_geometry import PoseHistory, transform, quaternion_from_rotation

        self.rclpy, self.Pose, self.Image, self.Info = rclpy, PoseStamped, Image, CameraInfo
        self.transform, self.quaternion = transform, quaternion_from_rotation
        rclpy.init(args=[])
        self.node = Node('demo_vision_contract')
        self.history = PoseHistory()
        self.states = []
        self.enabled, self.target = True, np.array([0., -.3, .03])
        self.image_pub = self.node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.info_pub = self.node.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
        self.tag_pub = self.node.create_publisher(PoseStamped, '/target_follow/tag_pose', 5) if detector else None
        if occupied:
            self.owner = self.node.create_publisher(PoseStamped, '/target_follow/flange_pose', 10)
        self.node.create_subscription(PoseStamped, '/target_follow/flange_pose', self.flange, 50)
        self.node.create_subscription(String, '/target_follow/follow_state',
                                      lambda m: self.states.append(json.loads(m.data)), 10)
        self.static = StaticTransformBroadcaster(self.node)
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'flange_link', 'camera_color_optical_frame'
        tf.transform.rotation.w = 1.
        self.static.sendTransform(tf)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        tag = (cv2.aruco.generateImageMarker(dictionary, 1, 100) if hasattr(cv2.aruco, 'generateImageMarker')
               else cv2.aruco.drawMarker(dictionary, 1, 100))
        canvas = np.full((240, 320), 255, np.uint8)
        canvas[70:170, 110:210] = tag
        self.canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
        self.timer = self.node.create_timer(.05, self.publish)
        self.output = ROOT / 'evidence/target_follow/stage5/mock_integration' / name
        self.output.mkdir(parents=True, exist_ok=True)
        params = tmp_path / 'demo_driver.yaml'
        params.write_text('position_follow:\n  ros__parameters:\n    collision_enabled: false\n')
        self.env = dict(os.environ, FR3C_FOLLOW_DOMAIN_ID=os.environ['ROS_DOMAIN_ID'])
        self.log_file = (self.output / 'launch.log').open('w+')
        command = [
            str(ROOT / 'scripts/target_follow/start_demo.sh'), '--mock', '--no-view',
            'output_dir:=' + str(self.output),
        ]
        if not use_default_params:
            # Exercise Demo ownership with the original ideal mock actuator;
            # the driver's collision model has separate integration tests.
            command.append('driver_params_file:=' + str(params))
        if collision_enabled is not None:
            command.append('collision_enabled:=' + str(collision_enabled).lower())
        self.process = subprocess.Popen(command, env=self.env, stdout=self.log_file, stderr=subprocess.STDOUT)

    def flange(self, message):
        p, q = message.pose.position, message.pose.orientation
        self.history.add(message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                         [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def publish(self):
        from rclpy.duration import Duration
        stamp = self.node.get_clock().now() - Duration(seconds=.06)
        if self.tag_pub is not None:
            if not self.enabled:
                return
            try:
                flange, _ = self.history.at(stamp.nanoseconds / 1e9)
            except ValueError:
                return
            optical = np.linalg.inv(flange) @ self.transform(self.target, [0, 0, 0, 1])
            message = self.Pose()
            message.header.frame_id = 'camera_color_optical_frame'
            message.header.stamp = stamp.to_msg()
            p, q = optical[:3, 3], self.quaternion(optical[:3, :3])
            message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, p)
            (message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w) = map(float, q)
            self.tag_pub.publish(message)
        else:
            info = self.Info()
            info.header.frame_id = 'camera_color_optical_frame'
            info.header.stamp = stamp.to_msg()
            info.width, info.height = 320, 240
            info.k = [480., 0., 160., 0., 480., 120., 0., 0., 1.]
            info.d, info.distortion_model = [0.] * 5, 'plumb_bob'
            message = self.Image()
            message.header = info.header
            message.width, message.height = 320, 240
            message.encoding, message.step = 'bgr8', 320 * 3
            message.data = array('B', self.canvas.tobytes())
            self.info_pub.publish(info)
            self.image_pub.publish(message)

    def wait(self, predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=.01)
            if predicate():
                return
        self.log_file.flush()
        raise AssertionError((self.states[-3:], (self.output / 'launch.log').read_text()))

    def state(self, predicate):
        self.wait(lambda: self.states and predicate(self.states[-1]))
        return self.states[-1]

    def control(self, action):
        command = subprocess.Popen([str(ROOT / 'scripts/target_follow/follow_control.sh'), action, '--mock'],
                                   env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.wait(lambda: command.poll() is not None)
            stdout, stderr = command.communicate()
            assert command.returncode == 0, stdout + stderr
            return stdout
        finally:
            if command.poll() is None:
                command.terminate()
                command.communicate(timeout=6)

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=12)
        finally:
            self.log_file.close()
            self.node.destroy_node()
            self.rclpy.shutdown()


def test_reuse_vision_controls_recovery_and_shutdown(tmp_path):
    demo = Demo(tmp_path, 'reuse_and_controls')
    try:
        demo.state(lambda s: s['status'] == 'idle' and s.get('target_age_ms', 999) < 200)
        session = json.loads((demo.output / 'session.json').read_text())
        assert not any(session['owned_sources'].values())
        assert demo.states[-1]['servo_commands_sent'] == 0
        assert '"status": "idle"' in demo.control('status')
        assert 'recorded' in demo.control('teach')
        assert 'started' in demo.control('start')
        demo.target += [.005, 0, 0]
        demo.state(lambda s: s['servo_commands_sent'] > 5)
        demo.enabled = False
        demo.state(lambda s: s['status'] == 'paused_target')
        demo.enabled = True
        demo.state(lambda s: s['status'] == 'following' and s['target_resumes'] > 0)
        assert 'Driver stopped' in demo.control('stop')
        stopped = demo.state(lambda s: s['status'] == 'stopped')
        assert stopped['stop_motion_code'] == stopped['servo_end_code'] == 0
        sent = stopped['servo_commands_sent']
        demo.target += [0, .005, 0]
        assert '"status": "stopped"' in demo.control('status')
        assert demo.states[-1]['servo_commands_sent'] == sent
        demo.control('start')
        demo.state(lambda s: s['status'] == 'following' and s['starts'] == 2)
        demo.process.send_signal(signal.SIGINT)
        demo.wait(lambda: demo.process.poll() is not None)
        demo.wait(lambda: demo.node.count_publishers('/target_follow/flange_pose') == 0)
        assert demo.node.count_publishers('/camera/color/image_raw') == 1
        assert demo.node.count_publishers('/target_follow/tag_pose') == 1
    finally:
        demo.close()
    report = json.loads((demo.output / 'report.json').read_text())
    assert report['mock'] and report['hardware_servo_commands_sent'] == 0
    assert report['stop_motion_code'] == report['servo_end_code'] == 0


def test_missing_detector_is_started_and_component_exit_ends_demo(tmp_path):
    demo = Demo(tmp_path, 'start_detector_and_exit', detector=False)
    try:
        demo.state(lambda s: s['status'] == 'idle' and s.get('target_age_ms', 999) < 200)
        session = json.loads((demo.output / 'session.json').read_text())
        assert session['owned_sources']['start_detector'] and not session['owned_sources']['start_camera']
        log = (demo.output / 'launch.log').read_text()
        match = re.search(r'\[tag_pose-\d+\]: process started with pid \[(\d+)\]', log)
        assert match, log
        demo.control('start')
        demo.state(lambda s: s['servo_commands_sent'] > 0)
        os.kill(int(match[1]), signal.SIGTERM)
        demo.wait(lambda: demo.process.poll() is not None)
        demo.wait(lambda: demo.node.count_publishers('/target_follow/flange_pose') == 0)
        assert demo.node.count_publishers('/camera/color/image_raw') == 1
    finally:
        demo.close()
    report = json.loads((demo.output / 'report.json').read_text())
    assert report['servo_commands_sent'] > 0 and report['hardware_servo_commands_sent'] == 0
    assert report['stop_motion_code'] == report['servo_end_code'] == 0
    assert 'Demo component exited' in (demo.output / 'launch.log').read_text()


def test_existing_feedback_owner_blocks_start_before_sdk(tmp_path):
    demo = Demo(tmp_path, 'existing_owner', occupied=True)
    try:
        demo.wait(lambda: demo.process.poll() is not None)
        assert demo.process.returncode != 0
        assert 'Close the existing feedback/preview/follow/servo launch first' in (demo.output / 'launch.log').read_text()
        assert not (demo.output / 'session.json').exists()
        assert not demo.states
    finally:
        demo.close()


def test_current_default_driver_configuration_starts_idle(tmp_path):
    demo = Demo(tmp_path, 'default_config_idle', use_default_params=True)
    try:
        state = demo.state(lambda s: s['status'] == 'idle' and 'actual_tcp_m' in s)
        assert state['servo_commands_sent'] == 0
        session = json.loads((demo.output / 'session.json').read_text())
        assert session['driver_params_file'].endswith('/config/position_follow.yaml')
    finally:
        demo.close()
    report = json.loads((demo.output / 'report.json').read_text())
    assert report['mock'] and report['servo_commands_sent'] == report['hardware_servo_commands_sent'] == 0


@pytest.mark.parametrize('enabled', [False, True])
def test_select_modes_follow_z_and_reject_hot_switch(tmp_path, enabled):
    demo = Demo(tmp_path, 'mode_' + ('guarded' if enabled else 'legacy'),
                use_default_params=True, collision_enabled=enabled)
    try:
        state = demo.state(lambda s: s['status']=='idle' and s.get('target_age_ms',999)<200 and
                           (not enabled or s.get('collision',{}).get('ready')))
        assert state['collision']['enabled'] is enabled
        assert state['tracking_mode'] == ('collision_guarded' if enabled else 'legacy')
        assert state['servo_commands_sent']==0
        # The selected mode is fixed throughout this SDK session. A rejected
        # parameter change must leave the reported mode unchanged.
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        client = demo.node.create_client(SetParameters, '/position_follow/set_parameters')
        demo.wait(client.service_is_ready)
        request = SetParameters.Request()
        request.parameters = [Parameter('collision_enabled', value=not enabled).to_parameter_msg()]
        future = client.call_async(request)
        demo.wait(future.done)
        assert not future.result().results[0].successful
        assert 'started' in demo.control('start')
        demo.target[2]+=.006
        demo.state(lambda s: s['servo_commands_sent']>10 and s.get('error_norm_mm',999)<.4)
        assert 'Driver stopped' in demo.control('stop')
        stopped = demo.state(lambda s: s['status']=='stopped')
        count = stopped['servo_commands_sent']; demo.target[2]+=.006
        demo.control('status')
        assert demo.states[-1]['servo_commands_sent']==count
        assert demo.states[-1]['collision']['enabled'] is enabled
    finally:
        demo.close()
    report = json.loads((demo.output/'report.json').read_text())
    assert report['collision_enabled'] is enabled
    assert report['hardware_servo_commands_sent']==0
    if not enabled:
        assert report['collision_check_ms'] is None
        assert report['collision_pauses']==report['collision_resumes']==0
