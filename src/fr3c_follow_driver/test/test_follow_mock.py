"""Stage 4 ROS contracts. All actuator calls use MockClient, never hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1',
                                reason='explicit isolated mock ROS run only')
ROOT = Path(__file__).resolve().parents[3]
BIN = ROOT/'install/fr3c_follow_driver/lib/fr3c_follow_driver/position_follow'
TCP = [-.000847397, .003027928, .184614459]


class Rig:
    def __init__(self, name, tmp_path, bridge=False, collision_parameters=None):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import StaticTransformBroadcaster
        from fr3c_target_follow.follow_geometry import PoseHistory, quaternion_from_rotation, transform

        self.rclpy, self.PoseStamped, self.Trigger = rclpy, PoseStamped, Trigger
        self.transform, self.quaternion = transform, quaternion_from_rotation
        self.history = PoseHistory()
        self.bridge, self.enabled, self.states = bridge, True, []
        self.target = np.array([0., -.3, .03])
        self.output = Path(os.environ.get('FR3C_POSITION_TEST_OUTPUT_ROOT',
                                         str(ROOT/'evidence/target_follow/stage4/mock_integration')))/name
        cmd = [str(BIN), '--ros-args', '-p', 'mock:=true', '-p', 'output_dir:='+str(self.output),
               '-p', 'tcp_xyz_m:='+str(TCP), '-p', 'tcp_rpy_rad:=[0.0, 0.0, 0.0]',
               '-p', 'collision_enabled:=false']  # Original ideal actuator contracts; geometry tested separately.
        if collision_parameters is not None:
            cmd = cmd[:-2] + ['--params-file', str(collision_parameters)]
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        tcp_file = tmp_path/'tcp.yaml'
        tcp_file.write_text('ydi_j22_tcp:\n  calibrated: true\n  parent_frame: flange_link\n'
                            '  child_frame: grasp_frame\n  xyz_m: '+str(TCP)+'\n  rpy_rad: [0.0, 0.0, 0.0]\n')
        rclpy.init(args=['--ros-args', '-p', 'tcp_file:='+str(tcp_file), '-p', 'transform_only:=true'])
        self.node = rclpy.create_node('position_follow_contract')
        self.executor = SingleThreadedExecutor(); self.executor.add_node(self.node)
        self.node.create_subscription(String, '/target_follow/follow_state',
                                      lambda m: self.states.append(json.loads(m.data)), 10)
        self.node.create_subscription(PoseStamped, '/target_follow/flange_pose', self.flange, 50)
        self.services = {name: self.node.create_client(Trigger, '/target_follow/'+path)
                         for name, path in [('start', 'follow_start'), ('stop', 'servo_stop'), ('teach', 'teach_offset')]}
        self.publisher = self.node.create_publisher(PoseStamped,
                         '/target_follow/tag_pose' if bridge else '/target_follow/tag_pose_base', 5)
        self.converter = None
        if bridge:
            from fr3c_target_follow.preview_node import FollowPreview
            self.converter = FollowPreview(); self.executor.add_node(self.converter)
            self.static = StaticTransformBroadcaster(self.node)
            tf = TransformStamped(); tf.header.stamp = self.node.get_clock().now().to_msg()
            tf.header.frame_id = 'flange_link'; tf.child_frame_id = 'camera_color_optical_frame'
            tf.transform.rotation.w = 1.; self.static.sendTransform(tf)
        self.timer = self.node.create_timer(.05, self.publish)

    def flange(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.history.add(m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                         [p.x,p.y,p.z], [q.x,q.y,q.z,q.w])

    def publish(self):
        if not self.enabled:
            return
        from rclpy.duration import Duration
        message = self.PoseStamped()
        stamp = self.node.get_clock().now()-Duration(seconds=.06)
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = 'camera_color_optical_frame' if self.bridge else 'base_link'
        p = self.target
        q = [0.,0.,0.,1.]
        if self.bridge:
            try:
                flange, _ = self.history.at(stamp.nanoseconds/1e9)
            except ValueError:
                return
            optical = np.linalg.inv(flange)@self.transform(p,q)
            p, q = optical[:3,3], self.quaternion(optical[:3,:3])
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float,p)
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = map(float,q)
        self.publisher.publish(message)

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic()<deadline:
            assert self.process.poll() is None, self.process.communicate()
            self.executor.spin_once(timeout_sec=.01)

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.pump(.02)
            if self.states and predicate(self.states[-1]):
                return self.states[-1]
        raise AssertionError(self.states[-5:])

    def call(self, name):
        client = self.services[name]
        deadline = time.monotonic()+2
        while not client.service_is_ready() and time.monotonic()<deadline:
            self.pump(.02)  # Keep simulated vision running during DDS service discovery.
        assert client.service_is_ready()
        future = client.call_async(self.Trigger.Request())
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic()<deadline:
            self.pump(.01)
        assert future.done()
        return future.result()

    def ready(self):
        self.wait(lambda s: s['status']=='idle' and 'actual_tcp_m' in s and
                  s.get('target_age_ms', float('inf'))<150)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        stdout, stderr = self.process.communicate(timeout=10)
        self.executor.shutdown()
        if self.converter:
            self.converter.destroy_node()
        self.node.destroy_node(); self.rclpy.shutdown()
        self.report = json.loads((self.output/'report.json').read_text())
        assert self.report['mock'] and self.report['hardware_servo_commands_sent']==0
        assert not self.report['live_follow_validated']
        return self.process.returncode, stdout+stderr


@pytest.mark.parametrize('axis', [0,1,2])
def test_axis_follow_and_stop_convergence(axis, tmp_path):
    rig = Rig('axis_'+'xyz'[axis], tmp_path)
    try:
        rig.ready()
        assert rig.states[-1]['servo_commands_sent']==0  # Launch alone never starts motion.
        assert rig.call('start').success
        initial = rig.wait(lambda s: s['status']=='following')['actual_tcp_m']
        for _ in range(12):
            rig.target[axis]+=.0005; rig.pump(.1)  # Target moves at 5 mm/s.
        final = rig.wait(lambda s: s.get('error_norm_mm',100)<.15, timeout=6)
        delta = np.array(final['actual_tcp_m'])-initial
        assert delta[axis]==pytest.approx(.006,abs=.0002)
        assert np.linalg.norm(np.delete(delta,axis))<1e-9
        assert np.linalg.norm(final['velocity_m_s'])<.0003
        assert all(not s['orientation_control_enabled'] and not s['motion_commands_enabled'] for s in rig.states)
        assert rig.call('stop').success
        stopped = rig.wait(lambda s: s['status']=='stopped')
        sent = stopped['servo_commands_sent']; rig.target[axis]+=.02; rig.pump(.2)
        assert rig.states[-1]['servo_commands_sent']==sent
    finally:
        code, output = rig.close()
    assert code==0, output


def test_image_time_bridge_combined_motion_occlusion_and_restart(tmp_path):
    rig = Rig('bridge_recovery_restart', tmp_path, bridge=True)
    try:
        rig.ready(); assert rig.call('teach').success; assert rig.call('start').success
        start = rig.wait(lambda s: s['status']=='following')
        initial, offset = np.array(start['actual_tcp_m']), np.array(start['offset_m'])
        for _ in range(10):
            rig.target += np.array([.0004,-.0003,.0002]); rig.pump(.1)
        final = rig.wait(lambda s: s.get('error_norm_mm',100)<.2, timeout=6)
        assert np.array(final['actual_tcp_m'])-initial==pytest.approx([.004,-.003,.002],abs=.00025)
        rig.enabled=False
        paused = rig.wait(lambda s: s['status']=='paused_target')
        assert paused['velocity_m_s']==[0.,0.,0.]
        sent = paused['servo_commands_sent']; actual = paused['actual_tcp_m']
        rig.target += [.006,0,0]; rig.pump(.3)
        assert rig.states[-1]['servo_commands_sent']==sent
        assert rig.states[-1]['actual_tcp_m']==actual
        rig.enabled=True
        resumed = rig.wait(lambda s: s['status']=='following' and s['target_resumes']>0)
        assert resumed['offset_m']==pytest.approx(offset)
        assert np.linalg.norm(np.array(resumed['actual_tcp_m'])-actual)<.002
        rig.wait(lambda s: s.get('error_norm_mm',100)<.2, timeout=6)
        assert rig.call('stop').success; rig.wait(lambda s: s['status']=='stopped')
        rig.target += [0,.008,0]; rig.pump(.2)
        assert rig.call('start').success
        restarted = rig.wait(lambda s: s['status']=='following' and s['starts']==2)
        assert restarted['error_norm_mm']<.15
        assert np.linalg.norm(np.array(restarted['offset_m'])-offset)>.007
        # Signal arrives during active following; final cleanup must stop and end servo.
    finally:
        code, output = rig.close()
    assert code==0, output
    assert rig.report['stop_motion_code']==rig.report['servo_end_code']==0
    assert rig.report['target_pauses']>=1 and rig.report['target_resumes']>=1


def test_reject_stale_wrong_frame_and_out_of_range_recovery(tmp_path):
    rig = Rig('invalid_input_range', tmp_path)
    try:
        rig.ready(); rig.enabled=False; rig.pump(.45)
        assert not rig.call('start').success
        rig.enabled=True; rig.ready(); assert rig.call('start').success
        rig.enabled=False; rig.wait(lambda s: s['status']=='paused_target')
        sent = rig.states[-1]['servo_commands_sent']
        from rclpy.duration import Duration
        for frame, age in [('camera_color_optical_frame',0),('base_link',2)]:
            m = rig.PoseStamped(); m.header.frame_id=frame
            m.header.stamp=(rig.node.get_clock().now()-Duration(seconds=age)).to_msg()
            m.pose.position.x=.02; m.pose.orientation.w=1.
            rig.publisher.publish(m); rig.pump(.1)
        assert rig.states[-1]['servo_commands_sent']==sent
        rig.target += [.101,0,0]; rig.enabled=True
        fault = rig.wait(lambda s: s['status']=='fault')
        assert 'range' in fault['reason'] and not fault['following_requested']
        assert fault['servo_commands_sent']==sent
        assert not rig.call('start').success
    finally:
        code, output = rig.close()
    assert code==1, output
    assert rig.report['stop_motion_code']==rig.report['servo_end_code']==0
