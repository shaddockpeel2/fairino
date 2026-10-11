"""Pose preview contracts over isolated ROS, using synthetic feedback only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1', reason='isolated ROS domain required')
ROOT = Path(__file__).resolve().parents[3]


def spin(executor, seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.005)


def pose_message(matrix, stamp):
    from geometry_msgs.msg import PoseStamped
    from fr3c_target_follow.follow_geometry import quaternion_from_rotation
    message = PoseStamped()
    message.header.frame_id, message.header.stamp = 'base_link', stamp
    message.pose.position.x, message.pose.position.y, message.pose.position.z = matrix[:3, 3].tolist()
    q = quaternion_from_rotation(matrix[:3, :3]).tolist()
    message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = q
    return message


def test_teach_deadband_hold_expiry_hard_rejection_and_feedback_stop(tmp_path):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from fr3c_target_follow.follow_geometry import transform_rpy, quaternion_from_rotation
    from fr3c_target_follow.pose_preview_node import PosePreviewNode, PREFIX

    tcp = tmp_path/'tcp.yaml'
    tcp.write_text('ydi_j22_tcp:\n  calibrated: true\n  parent_frame: flange_link\n  child_frame: grasp_frame\n  xyz_m: [.002, -.003, .185]\n  rpy_rad: [.1, -.2, .3]\n')
    rclpy.init(args=['--ros-args', '-p', 'tcp_file:='+str(tcp)])
    preview, probe = PosePreviewNode(), Node('pose_preview_contract_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(preview); executor.add_node(probe)
    states = []
    probe.create_subscription(String, PREFIX+'state', lambda m: states.append(json.loads(m.data)), 50)
    feedback = probe.create_publisher(PoseStamped, '/target_follow/flange_pose', 50)
    observations = probe.create_publisher(String, '/target_follow/pose_observation', 50)
    client = probe.create_client(Trigger, PREFIX+'teach')
    robot = transform_rpy([.2, -.3, .35], [np.pi, .1, .2])
    timer = probe.create_timer(.01, lambda: feedback.publish(pose_message(robot, probe.get_clock().now().to_msg())))

    def observation(angle=0., status='valid', malformed=False, future=False):
        stamp = (probe.get_clock().now()-Duration(seconds=.06)).to_msg()
        if future:
            stamp.sec += 5
        image_stamp = dict(sec=stamp.sec, nanosec=stamp.nanosec)
        target = transform_rpy([.1, -.4, .02], [0, 0, np.radians(angle)])
        quality = dict(status='valid', valid_position=True, orientation_ambiguous=False,
                       orientation_measurement_valid=True, solver='IPPE_SQUARE', candidate_count=2,
                       candidate_errors_px=[.01, .6], candidate_rotation_distance_deg=30.,
                       selected_candidate_index=0, stamp=image_stamp)
        row = dict(schema_version=1, frame_id='base_link', source_frame_id='camera_color_optical_frame',
                   stamp=image_stamp, status=status, target_id=1, position_m=target[:3, 3].tolist(),
                   quaternion_xyzw=quaternion_from_rotation(target[:3, :3]).tolist(), valid_position=True,
                   valid_orientation=status == 'valid', orientation_measurement_valid=status == 'valid',
                   motion_commands_enabled=False, orientation_control_enabled=False,
                   quality=[] if malformed else quality)
        observations.publish(String(data=json.dumps(row)))
        spin(executor, .055)

    def teach():
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+2.
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert future.done()
        return future.result()

    def await_state(predicate):
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            if states and predicate(states[-1]):
                return states[-1]
            executor.spin_once(timeout_sec=.005)
        raise AssertionError(states[-3:])

    try:
        spin(executor, .4)
        assert not teach().success
        observation()
        assert teach().success
        recorded = await_state(lambda row: row['teach_count'] == 1 and row['status'] == 'preview')
        assert recorded['status'] == 'preview'
        assert recorded['desired_tcp_m'] == pytest.approx(recorded['actual_tcp_m'], abs=1e-10)
        assert recorded['desired_tcp_quaternion_xyzw'] == pytest.approx(recorded['actual_tcp_quaternion_xyzw'], abs=1e-10)
        observation(angle=2.)
        assert states[-1]['desired_tcp_quaternion_xyzw'] == pytest.approx(recorded['desired_tcp_quaternion_xyzw'])
        for _ in range(12):
            observation(angle=20.)
        assert states[-1]['rotation_error_deg'] > 5.
        goal = states[-1]['desired_tcp_m']
        good_ns = preview.good_ns
        observation(angle=80., status='orientation_ambiguous')
        assert states[-1]['status'] == 'holding_quality'
        assert states[-1]['desired_tcp_m'] == pytest.approx(goal)
        assert preview.good_ns == good_ns and not teach().success
        for _ in range(7):
            observation(angle=80., status='orientation_ambiguous')
        assert states[-1]['status'] == 'stale_trusted_pose'
        assert states[-1]['angular_velocity_deg_s'] == [0., 0., 0.]
        observation(angle=20.)
        observation(angle=100., status='orientation_jump')
        assert states[-1]['status'] == 'orientation_jump'
        assert states[-1]['velocity_m_s'] == [0., 0., 0.]
        assert states[-1]['angular_velocity_deg_s'] == [0., 0., 0.]
        observation(angle=20., malformed=True)
        assert states[-1]['status'] == 'invalid_trusted_pose'
        watermark = preview.event_ns
        observation(angle=20., future=True)
        assert preview.event_ns == watermark
        observation(angle=20.)
        assert teach().success
        rerecorded = await_state(lambda row: row['teach_count'] == 2 and row['status'] == 'preview')
        assert rerecorded['rotation_error_deg'] < 1e-5
        observation(angle=20., status='target_not_found')
        assert states[-1]['status'] == 'target_not_found'
        assert states[-1]['velocity_m_s'] == [0., 0., 0.]
        observation(angle=20.)
        timer.cancel()
        spin(executor, .24)
        assert states[-1]['status'] == 'stale_robot_feedback'
        assert all(not row['motion_commands_enabled'] and not row['orientation_control_enabled'] for row in states)
        names = [name for name, types in probe.get_service_names_and_types()]
        assert PREFIX+'teach' in names
        assert '/target_follow/follow_start' not in names and '/target_follow/servo_stop' not in names
    finally:
        output = ROOT/'evidence/target_follow/stage7/ros_contract'
        output.mkdir(parents=True, exist_ok=True)
        (output/'states.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in states))
        executor.shutdown(); timer.cancel()
        preview.destroy_node(); probe.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('mapping,with_viewer', [
    (1., False), (.5, False), pytest.param(1., True, id='gui', marks=pytest.mark.skipif(
        os.environ.get('FR3C_RUN_GUI_TESTS') != '1', reason='working display required'))])
def test_read_only_launch_auto_records_and_closes_owned_nodes(mapping, with_viewer):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from std_msgs.msg import String
    from fr3c_target_follow.follow_geometry import transform_rpy

    rclpy.init(args=[])
    probe = Node('pose_preview_launch_synthetic_source')
    feedback = probe.create_publisher(PoseStamped, '/target_follow/flange_pose', 50)
    metadata = probe.create_publisher(String, '/target_follow/robot_feedback', 20)
    base = probe.create_publisher(PoseStamped, '/target_follow/tag_pose_base', 20)
    quality = probe.create_publisher(String, '/target_follow/quality', 20)
    viewer_clients = []
    image_pub = image_message = None
    if with_viewer:
        from sensor_msgs.msg import Image
        from rclpy.qos import qos_profile_sensor_data
        import cv2
        image_pub = probe.create_publisher(Image, '/target_follow/image_annotated', qos_profile_sensor_data)
        canvas = np.full((480, 640, 3), 235, np.uint8)
        cv2.putText(canvas, 'SYNTHETIC INPUT - NO CAMERA OR ROBOT', (10, 35), 0, .55, (0, 0, 0), 1)
        image_message = Image(height=480, width=640, encoding='bgr8', step=640*3, data=canvas.tobytes())
    began = time.monotonic()

    def publish():
        feedback.publish(pose_message(transform_rpy([.2, -.3, .35], [np.pi, 0., 0.]), probe.get_clock().now().to_msg()))
        metadata.publish(String(data='{"status":"valid","motion_commands_enabled":false}'))
        angle = np.radians(15.)*np.sin((time.monotonic()-began)*2.)
        stamp = (probe.get_clock().now()-Duration(seconds=.06)).to_msg()
        base.publish(pose_message(transform_rpy([.1, -.4, .02], [0., angle, 0.]), stamp))
        quality.publish(String(data=json.dumps(dict(
            status='valid', target_id=1, frame_id='camera_color_optical_frame',
            stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec), valid_position=True,
            orientation_measurement_valid=True, orientation_ambiguous=False,
            solver='IPPE_SQUARE', candidate_count=2, selected_candidate_index=0,
            candidate_errors_px=[.01, .6], candidate_rotation_distance_deg=30.))))
        if image_pub is not None:
            image_message.header.stamp = probe.get_clock().now().to_msg()
            image_pub.publish(image_message)

    timer = probe.create_timer(.066, publish)
    output = ROOT/'evidence/target_follow/stage7'/('launch_gui' if with_viewer else 'launch_full' if mapping == 1. else 'launch_half')
    output.mkdir(parents=True, exist_ok=True)
    process = None
    with (output/'launch.log').open('w+') as log:
        try:
            env = dict(os.environ, FR3C_FOLLOW_DOMAIN_ID=os.environ['ROS_DOMAIN_ID'])
            process = subprocess.Popen([str(ROOT/'scripts/target_follow/start_pose_preview.sh'), '--no-view',
                'viewer:='+str(with_viewer).lower(),
                'start_sources:=false', 'auto_teach:=true', 'duration_s:=4',
                'rotation_mapping_gain:='+str(mapping), 'output_dir:='+str(output)],
                env=env, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+25.
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=.01)
                if with_viewer and 'pose_preview_viewer' in [name for name, namespace in probe.get_node_names_and_namespaces()]:
                    viewer_clients = probe.get_client_names_and_types_by_node('pose_preview_viewer', '/')
            assert process.poll() == 0, (output/'launch.log').read_text()
            logtext = (output/'launch.log').read_text()
            assert 'process has died' not in logtext and 'Traceback' not in logtext
            report = json.loads((output/'report.json').read_text())
            session = json.loads((output/'session.json').read_text())
            assert report['teach_count'] == 1 and report['trusted_observations'] >= 10
            assert report['rotation_mapping_gain'] == mapping
            assert max(report['desired_position_axis_span_mm']) > 1.
            assert report['max_preview_speed_mm_s'] <= 20.001
            assert report['max_preview_angular_speed_deg_s'] <= 3.001
            assert not report['motion_commands_enabled'] and report['hardware_servo_commands_sent'] == 0
            assert session['stage'] == 7 and not any(session['owned_sources'].values())
            names = [name for name, namespace in probe.get_node_names_and_namespaces()]
            assert 'pose_preview_launch_synthetic_source' in names
            assert 'pose_preview' not in names and 'orientation_check' not in names
            if with_viewer:
                assert {name for name, types in viewer_clients} == {'/target_follow/pose_preview/teach'}
                assert 'pose_preview_viewer' not in names
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT); process.wait(timeout=5)
            timer.cancel(); probe.destroy_node(); rclpy.shutdown()
