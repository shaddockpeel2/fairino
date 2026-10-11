"""Stage 9 contracts over isolated ROS with explicit MockClient only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1', reason='isolated mock ROS required')
ROOT = Path(__file__).resolve().parents[3]
BIN = ROOT/'install/fr3c_follow_driver/lib/fr3c_follow_driver/pose_follow'
TCP = [.002, -.003, .185]
RPY = [.1, -.2, .3]


class Rig:
    def __init__(self, name, tmp_path, stable=False, collision=False, bridge=False, overrides=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from fr3c_target_follow.follow_geometry import transform_rpy, transform, quaternion_from_rotation, PoseHistory
        self.rclpy, self.PoseStamped, self.String, self.Trigger = rclpy, PoseStamped, String, Trigger
        self.transform, self.rpy, self.quaternion = transform, transform_rpy, quaternion_from_rotation
        self.enabled, self.status, self.bridge, self.malformed = True, 'valid', bridge, None
        self.approximate = False
        self.states, self.history, self.flange_messages = [], PoseHistory(), []
        self.target = transform_rpy([.05, -.3, .03], [0., 0., 0.])
        self.output = Path(os.environ.get('FR3C_POSE_TEST_OUTPUT_ROOT',
                                         str(ROOT/'evidence/target_follow/stage9/mock_integration')))/name
        params = yaml.safe_load((ROOT/'src/fr3c_follow_driver/config/pose_follow.yaml').read_text())['pose_follow']['ros__parameters']
        params.update(mock=True, output_dir=str(self.output), tcp_xyz_m=TCP, tcp_rpy_rad=RPY)
        if not stable:
            params.update(position_tau_s=.02, rotation_tau_s=.03, deadband_enter_deg=0., deadband_exit_deg=0.,
                          rotation_mapping_gain=1., control_position_deadband_m=.0002, control_angle_deadband_deg=.05,
                          goal_position_threshold_m=0., goal_angle_threshold_deg=0., goal_confirmation_s=0.)
        params.update(overrides or {})
        if collision:
            import xacro
            robot = ROOT/'src/fairino3mt_v6_moveit2_config/config'
            params.update(collision_enabled=True, robot_description=xacro.process_file(str(robot/'fairino3mt_v6_robot.urdf.xacro'),
                mappings={'initial_positions_file': str(robot/'initial_positions.yaml'), 'hardware_plugin': 'mock_components/GenericSystem'}).toxml(),
                robot_description_semantic=(robot/'fairino3mt_v6_robot.srdf').read_text())
        path = tmp_path/(name+'.yaml')
        path.write_text(yaml.safe_dump({'pose_follow': {'ros__parameters': params}}))
        self.process = subprocess.Popen([str(BIN), '--ros-args', '--params-file', str(path)],
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        rclpy.init(args=[])
        self.node = rclpy.create_node('pose_follow_contract_'+name)
        self.executor = SingleThreadedExecutor(); self.executor.add_node(self.node)
        self.node.create_subscription(String, '/target_follow/follow_state', lambda m: self.states.append(json.loads(m.data)), 50)
        self.node.create_subscription(PoseStamped, '/target_follow/flange_pose', self.feedback, 100)
        self.services = {key: self.node.create_client(Trigger, '/target_follow/'+path) for key, path in
                         [('start', 'follow_start'), ('teach', 'teach_offset'), ('stop', 'servo_stop')]}
        self.observations = self.node.create_publisher(String, '/target_follow/pose_observation', 20) if not bridge else None
        self.converter = self.monitor = None
        if bridge:
            from fr3c_target_follow.preview_node import FollowPreview
            from fr3c_target_follow.orientation_node import OrientationCheck
            from tf2_ros import StaticTransformBroadcaster
            tcp_file = tmp_path/'bridge_tcp.yaml'
            tcp_file.write_text(yaml.safe_dump({'ydi_j22_tcp': dict(calibrated=True, parent_frame='flange_link', child_frame='grasp_frame', xyz_m=TCP, rpy_rad=RPY)}))
            self.converter = self.create_python_node(FollowPreview, [('tcp_file', str(tcp_file)), ('transform_only', True)])
            monitor_params = yaml.safe_load((ROOT/'src/fr3c_target_follow/config/orientation_check_pose_demo.yaml').read_text())['orientation_check']['ros__parameters']
            self.monitor = self.create_python_node(OrientationCheck, list(monitor_params.items()))
            self.executor.add_node(self.converter); self.executor.add_node(self.monitor)
            self.tags = self.node.create_publisher(PoseStamped, '/target_follow/tag_pose', 20)
            self.quality = self.node.create_publisher(String, '/target_follow/quality', 20)
            self.static = StaticTransformBroadcaster(self.node)
            tf = TransformStamped(); tf.header.frame_id='flange_link'; tf.child_frame_id='camera_color_optical_frame'; tf.transform.rotation.w=1.
            self.static.sendTransform(tf)
        self.timer = self.node.create_timer(.05, self.publish)

    @staticmethod
    def create_python_node(cls, values):
        # These existing nodes consume global CLI arguments at construction.
        # Patch declare_parameter only while this one instance is constructed.
        from unittest.mock import patch
        from rclpy.node import Node
        original = Node.declare_parameter
        overrides = dict(values)
        def declare(node, name, value=None, *args, **kwargs):
            return original(node, name, overrides.get(name, value), *args, **kwargs)
        with patch.object(Node, 'declare_parameter', declare):
            return cls()

    def feedback(self, message):
        p, q = message.pose.position, message.pose.orientation
        self.history.add(message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                         [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        self.flange_messages.append(message)

    def quality_row(self, stamp):
        row = dict(status='valid', target_id=1, frame_id='camera_color_optical_frame', stamp=stamp.copy(),
                    valid_position=True, orientation_measurement_valid=True, orientation_ambiguous=False,
                    solver='IPPE_SQUARE', candidate_count=2, selected_candidate_index=0,
                    candidate_errors_px=[.01, .6], candidate_rotation_distance_deg=30.)
        if self.approximate:
            row.update(selected_candidate_index=1, candidate_errors_px=[.08, .14],
                       orientation_candidate_tolerance_deg=35., orientation_approximate=True)
        return row

    def publish(self):
        if not self.enabled:
            return
        from rclpy.duration import Duration
        stamp = (self.node.get_clock().now()-Duration(seconds=.06)).to_msg()
        image_stamp = dict(sec=stamp.sec, nanosec=stamp.nanosec)
        quality = self.quality_row(image_stamp)
        if self.bridge:
            try:
                flange, _ = self.history.at(stamp.sec+stamp.nanosec/1e9)
            except ValueError:
                return
            optical = np.linalg.inv(flange) @ self.target
            message = self.PoseStamped(); message.header.frame_id='camera_color_optical_frame'; message.header.stamp=stamp
            message.pose.position.x, message.pose.position.y, message.pose.position.z = optical[:3, 3].tolist()
            q = self.quaternion(optical[:3, :3]).tolist()
            message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = q
            self.tags.publish(message); self.quality.publish(self.String(data=json.dumps(quality)))
            return
        row = dict(schema_version=1, status=self.status, frame_id='base_link', source_frame_id='camera_color_optical_frame',
                   stamp=image_stamp, target_id=1, valid_position=True, valid_orientation=self.status == 'valid',
                   orientation_measurement_valid=self.status == 'valid', continuity_frame='base_link', confirmation_count=2,
                   position_m=self.target[:3, 3].tolist(), quaternion_xyzw=self.quaternion(self.target[:3, :3]).tolist(),
                   quality=quality, motion_commands_enabled=False, orientation_control_enabled=False)
        if self.malformed == 'quality_stamp':
            row['quality']['stamp']['nanosec'] = (stamp.nanosec+1) % 10**9
        elif self.malformed == 'future':
            row['stamp']['sec'] += 5
            row['quality']['stamp']['sec'] += 5
        elif self.malformed == 'wrong_frame':
            row['frame_id'] = 'camera_color_optical_frame'
        self.observations.publish(self.String(data=json.dumps(row)))

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.process.communicate()
            self.executor.spin_once(timeout_sec=.01)

    def wait(self, predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.pump(.02)
            if self.states and predicate(self.states[-1]):
                return self.states[-1]
        raise AssertionError(self.states[-4:])

    def call(self, name):
        client = self.services[name]; deadline = time.monotonic()+3
        while not client.service_is_ready() and time.monotonic() < deadline:
            self.pump(.02)
        assert client.service_is_ready()
        future = client.call_async(self.Trigger.Request()); deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            self.pump(.01)
        assert future.done(); return future.result()

    def ready(self):
        return self.wait(lambda s: s['status'] == 'idle' and s.get('quality_reason') == '' and 'actual_tcp_m' in s)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        stdout, stderr = self.process.communicate(timeout=10)
        self.timer.cancel(); self.executor.shutdown()
        for node in (self.converter, self.monitor, self.node):
            if node is not None:
                node.destroy_node()
        self.rclpy.shutdown()
        self.report = json.loads((self.output/'report.json').read_text())
        assert self.report['mock'] and self.report['hardware_servo_commands_sent'] == 0
        assert not self.report['live_pose_follow_validated']
        return self.process.returncode, stdout+stderr


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_full_relation_each_axis_nonzero_tool_and_no_jump(tmp_path, axis):
    rig = Rig('axis_'+str(axis), tmp_path)
    try:
        initial = rig.ready(); assert initial['servo_commands_sent'] == 0
        assert rig.call('teach').success
        recorded = rig.wait(lambda s: s['offset_recorded'])
        assert recorded['desired_tcp_m'] == pytest.approx(recorded['actual_tcp_m'], abs=1e-10)
        assert rig.call('start').success
        angles = np.zeros(3); angles[axis] = np.radians(8.)
        rig.target = rig.rpy([.055, -.298, .03], angles)
        state = rig.wait(lambda s: s.get('rotation_error_deg', 100) < .4 and s.get('error_norm_mm', 100) < 1.
                         and s['servo_commands_sent'] > 100, timeout=12)
        expected = rig.target @ np.asarray(state['offset_matrix'])
        actual = rig.transform(state['actual_tcp_m'], state['actual_tcp_quaternion_xyzw'])
        from fr3c_target_follow.marker_pose import rotation_distance_deg
        assert rotation_distance_deg(expected[:3, :3], actual[:3, :3]) < .4
        assert np.linalg.norm(expected[:3, 3]-actual[:3, 3]) < .001
        assert not rig.call('teach').success
        assert rig.call('stop').success
        rig.wait(lambda s: s['status'] == 'stopped' and not s['following_requested'])
        assert rig.call('start').success
        restart = rig.wait(lambda s: s['starts'] == 2)
        assert restart['error_norm_mm'] < .001 and restart['rotation_error_deg'] < .001
    finally:
        code, output = rig.close()
    assert code == 0, output
    assert rig.report['stop_motion_code'] == rig.report['servo_end_code'] == 0


def test_demo_jitter_held_quality_timeout_recovery_and_hard_loss(tmp_path):
    rig = Rig('quality_recovery', tmp_path, stable=True)
    try:
        start = rig.ready(); assert rig.call('start').success
        for i in range(20):
            rig.target = rig.rpy([.05, -.3, .03], [0., 0., np.radians(4.5*(-1 if i % 2 else 1))])
            rig.pump(.05)
        from fr3c_target_follow.marker_pose import rotation_distance_deg
        assert rotation_distance_deg(rig.transform(start['actual_tcp_m'], start['actual_tcp_quaternion_xyzw'])[:3, :3],
                                     rig.transform(rig.states[-1]['actual_tcp_m'], rig.states[-1]['actual_tcp_quaternion_xyzw'])[:3, :3]) < .01
        rig.target = rig.rpy([.06, -.3, .03], [0., 0., np.radians(20.)]); rig.pump(2.)
        rig.status = 'orientation_ambiguous'
        held = rig.wait(lambda s: s['status'] == 'holding_quality')
        stamp, goal = held['trusted_image_stamp'], held['desired_tcp_m']
        rig.pump(.10)
        assert rig.states[-1]['trusted_image_stamp'] == stamp
        assert rig.states[-1]['desired_tcp_m'] == pytest.approx(goal)
        paused = rig.wait(lambda s: s['status'] == 'paused_target')
        assert paused['angular_velocity_deg_s'] == [0., 0., 0.]
        relation, sent = paused['offset_matrix'], paused['servo_commands_sent']
        rig.status = 'valid'
        resumed = rig.wait(lambda s: s['status'] == 'following' and s['servo_commands_sent'] > sent)
        assert resumed['offset_matrix'] == relation and resumed['target_resumes'] >= 1
        rig.status = 'orientation_jump'
        hard_paused = rig.wait(lambda s: s['status'] == 'paused_target' and s['quality_reason'] == 'orientation_jump')
        rig.status = 'acquiring_orientation'; rig.pump(.10)
        assert rig.states[-1]['status'] == 'paused_target'
        assert rig.states[-1]['servo_commands_sent'] == hard_paused['servo_commands_sent']
        rig.status = 'valid'; rig.wait(lambda s: s['status'] == 'following')
        rig.status = 'target_not_found'
        rig.wait(lambda s: s['status'] == 'paused_target' and s['quality_reason'] == 'target_not_found')
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_large_relocation_and_raw_range_pause_require_explicit_rerecord(tmp_path):
    rig = Rig('relocation', tmp_path, overrides={'follow_radius_m': .20, 'recovery_position_gap_m': .04})
    try:
        rig.ready(); assert rig.call('start').success
        rig.status = 'target_not_found'; paused = rig.wait(lambda s: s['status'] == 'paused_target')
        sent = paused['servo_commands_sent']
        rig.target[:3, 3] += [.07, 0., 0.]; rig.status = 'valid'
        waiting = rig.wait(lambda s: s['status'] == 'paused_reteach')
        assert waiting['servo_commands_sent'] == sent and waiting['requires_reteach']
        assert rig.call('start').success
        restarted = rig.wait(lambda s: s['starts'] == 2 and s['status'] == 'following')
        assert restarted['error_norm_mm'] < .001
        rig.target[:3, 3] += [.22, 0., 0.]
        waiting = rig.wait(lambda s: s['status'] == 'paused_reteach')
        assert 'range' in waiting['reason']
        assert rig.call('start').success
        rig.wait(lambda s: s['starts'] == 3)
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_malformed_stamp_future_and_wrong_frame_never_refresh_trusted_time(tmp_path):
    rig = Rig('invalid_observation', tmp_path)
    try:
        rig.ready(); assert rig.call('start').success
        for malformed in ('quality_stamp', 'future', 'wrong_frame'):
            rig.malformed = malformed
            paused = rig.wait(lambda s: s['status'] == 'paused_target')
            stamp, sent = paused['trusted_image_stamp'], paused['servo_commands_sent']
            rig.pump(.15)
            assert rig.states[-1]['trusted_image_stamp'] == stamp and rig.states[-1]['servo_commands_sent'] == sent
            rig.malformed = None
            rig.wait(lambda s: s['status'] == 'following' and s['servo_commands_sent'] > sent)
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_image_time_chain_with_camera_rotation_uses_paired_quality(tmp_path):
    rig = Rig('image_time_bridge', tmp_path, bridge=True)
    try:
        rig.ready(); assert rig.call('start').success
        # Gradual actual target rotation preserves the stage 6 continuity contract.
        for angle in np.linspace(0, 8, 16):
            rig.target = rig.rpy([.055, -.298, .03], [np.radians(angle), 0., 0.]); rig.pump(.1)
        state = rig.wait(lambda s: s.get('rotation_error_deg', 100) < .5 and s['servo_commands_sent'] > 150, timeout=10)
        assert state['follow_mode'] == 'pose' and state['quality_reason'] == ''
        assert np.linalg.norm(np.array(state['target_base_m'])-rig.target[:3, 3]) < .0001
        assert rig.node.count_publishers('/target_follow/desired_tcp_pose') == 1
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_start_can_use_recent_trusted_anchor_during_soft_quality_only(tmp_path):
    rig = Rig('start_soft_quality', tmp_path, stable=True)
    try:
        rig.ready(); rig.status = 'orientation_ambiguous'; rig.pump(.06)
        stamp = rig.states[-1]['trusted_image_stamp']
        assert rig.call('start').success
        held = rig.wait(lambda s: s['status'] == 'holding_quality')
        assert held['trusted_image_stamp'] == stamp and held['error_norm_mm'] < .001
        paused = rig.wait(lambda s: s['status'] == 'paused_target')
        assert not rig.call('start').success
        assert rig.states[-1]['servo_commands_sent'] == paused['servo_commands_sent']
        rig.status = 'valid'; rig.wait(lambda s: s['status'] == 'following')
        assert rig.call('stop').success; rig.wait(lambda s: s['status'] == 'stopped')
        rig.status = 'orientation_jump'; rig.pump(.06)
        assert not rig.call('start').success
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_approximate_alternative_candidate_reaches_driver_through_real_quality_chain(tmp_path):
    rig = Rig('approximate_bridge', tmp_path, bridge=True, stable=True)
    try:
        rig.approximate = True
        ready = rig.ready()
        assert ready['orientation_approximate'] and ready['orientation_candidate_tolerance_deg'] == 35.
        assert rig.call('start').success
        for angle in np.linspace(0, 18, 18):
            rig.target = rig.rpy([.055, -.3, .03], [0., 0., np.radians(angle)]); rig.pump(.1)
        followed = rig.wait(lambda s: s['servo_commands_sent'] > 150 and s.get('rotation_error_deg',100) < 5.)
        assert followed['status'] == 'following' and followed['orientation_approximate']
        assert followed['target_pauses'] == 0
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_tilt_noise_and_raw_thirty_degree_recovery_keep_original_relation(tmp_path):
    rig = Rig('tilt_recovery', tmp_path, bridge=True, stable=True)
    try:
        rig.approximate = True
        initial = rig.ready(); assert rig.call('start').success
        for angle in [14., -8., 16., -7.]*4 + [10., 20., 26.]:
            rig.target = rig.rpy([.05, -.3, .03], [0., np.radians(angle), 0.]); rig.pump(.10)
        rig.pump(2.)
        following = rig.states[-1]
        assert following['status'] == 'following' and following['target_pauses'] == 0
        relation = following['offset_matrix']
        from fr3c_target_follow.marker_pose import rotation_distance_deg
        assert rotation_distance_deg(rig.transform(initial['actual_tcp_m'], initial['actual_tcp_quaternion_xyzw'])[:3, :3],
                                     rig.transform(following['actual_tcp_m'], following['actual_tcp_quaternion_xyzw'])[:3, :3]) > 1.
        assert max(np.linalg.norm(s['angular_velocity_deg_s']) for s in rig.states) <= 3.+1e-8
        rig.enabled = False; paused = rig.wait(lambda s: s['status'] == 'paused_target')
        sent = paused['servo_commands_sent']
        # 30 deg raw change remains recoverable with the same relation.
        rig.target = rig.rpy([.05, -.3, .03], [0., np.radians(-4.), 0.]); rig.enabled = True
        resumed = rig.wait(lambda s: s['status'] == 'following' and s['servo_commands_sent'] > sent)
        assert not resumed['requires_reteach'] and resumed['offset_matrix'] == relation
        rig.enabled = False; rig.wait(lambda s: s['status'] == 'paused_target')
        # 43 deg is also recoverable now; an extreme 134 deg relocation still
        # requires recapture after loss, even inside the enlarged motion range.
        rig.target = rig.rpy([.05, -.3, .03], [0., np.radians(39.), 0.]); rig.enabled = True
        rig.wait(lambda s: s['status'] == 'following' and not s['requires_reteach'])
        rig.target = rig.rpy([.05, -.3, .03], [0., np.radians(-4.), 0.]); rig.pump(1.)
        rig.enabled = False; rig.wait(lambda s: s['status'] == 'paused_target')
        rig.target = rig.rpy([.05, -.3, .03], [0., np.radians(130.), 0.]); rig.enabled = True
        relocated = rig.wait(lambda s: s['status'] == 'paused_reteach')
        assert relocated['requires_reteach'] and 'relocated' in relocated['reason']
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_large_tilt_settles_and_full_pose_noise_does_not_restart_corrections(tmp_path):
    rig = Rig('large_tilt_hold', tmp_path, bridge=True, stable=True)
    try:
        initial = rig.ready(); assert rig.call('start').success
        for angle in np.linspace(0, 60, 30):
            rig.target = rig.rpy([.07, -.3, .03], [0., np.radians(angle), 0.]); rig.pump(.08)
        def settled(s):
            return (s['status'] == 'following' and s['settled_hold'] and
                    np.linalg.norm(s['angular_velocity_deg_s']) < .0001 and
                    np.linalg.norm(s['velocity_m_s']) < 1e-8)
        rig.wait(settled, timeout=20); rig.pump(3.); stopped = rig.wait(settled)
        from fr3c_target_follow.marker_pose import rotation_distance_deg
        assert rotation_distance_deg(rig.transform(initial['actual_tcp_m'], initial['actual_tcp_quaternion_xyzw'])[:3,:3],
                                     rig.transform(stopped['actual_tcp_m'], stopped['actual_tcp_quaternion_xyzw'])[:3,:3]) > 20.
        assert stopped['target_pauses'] == 0 and not stopped['requires_reteach']
        marker = len(rig.states)
        for i in range(30):
            sign = -1 if i % 2 else 1
            rig.target = rig.rpy([.07+sign*.002, -.3-sign*.002, .03+sign*.001],
                                 [0., np.radians(60.+sign*3.), 0.]); rig.pump(.05)
        for state in rig.states[marker:]:
            assert state['settled_hold'] and state['status'] == 'following'
            assert state['desired_tcp_m'] == pytest.approx(stopped['desired_tcp_m'], abs=1e-12)
            assert state['reference_tcp_m'] == pytest.approx(stopped['reference_tcp_m'], abs=1e-12)
            assert np.linalg.norm(state['angular_velocity_deg_s']) < .0001
            assert np.linalg.norm(state['velocity_m_s']) < 1e-8
        rig.enabled = False; rig.wait(lambda s: s['status'] == 'paused_target')
        rig.target = rig.rpy([.10, -.3, .03], [0., np.radians(60.), 0.]); rig.enabled = True
        released = rig.wait(lambda s: s['status'] == 'following' and not s['settled_hold'] and
                            np.linalg.norm(s['velocity_m_s']) > .001)
        assert released['offset_matrix'] == stopped['offset_matrix']
    finally:
        code, output = rig.close()
    assert code == 0, output


def test_enabled_collision_guard_predicts_rotating_tool_in_actual_driver(tmp_path):
    rig = Rig('collision_on', tmp_path, collision=True)
    try:
        rig.ready(); rig.wait(lambda s: s['collision']['ready'] and s['collision']['speed_scale'] > 0)
        assert rig.call('start').success
        rig.target = rig.rpy([.054, -.3, .03], [0., 0., np.radians(4.)])
        state = rig.wait(lambda s: s['servo_commands_sent'] > 50 and s.get('rotation_error_deg', 100) < 3.8, timeout=10)
        assert state['collision']['enabled'] and state['collision']['pose_prediction']
        assert state['status'] != 'fault'
    finally:
        code, output = rig.close()
    assert code == 0, output
