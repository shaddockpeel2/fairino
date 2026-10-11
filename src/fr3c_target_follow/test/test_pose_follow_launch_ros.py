"""Stage 9 entry with synthetic eye-in-hand poses and explicit mock driver."""
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


def test_launch_reuses_synthetic_detector_loads_stable_profile_and_cleans_owned_nodes():
    import rclpy
    from rclpy.duration import Duration
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from fr3c_target_follow.follow_geometry import PoseHistory, transform_rpy, quaternion_from_rotation
    from fr3c_target_follow.marker_pose import rotation_distance_deg

    rclpy.init()
    node = rclpy.create_node('pose_follow_launch_synthetic_source')
    history, states = PoseHistory(), []
    tags = node.create_publisher(PoseStamped, '/target_follow/tag_pose', 10)
    quality = node.create_publisher(String, '/target_follow/quality', 10)
    node.create_subscription(String, '/target_follow/follow_state', lambda m: states.append(json.loads(m.data)), 50)

    def feedback(message):
        p, q = message.pose.position, message.pose.orientation
        history.add(message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                    [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    node.create_subscription(PoseStamped, '/target_follow/flange_pose', feedback, 100)
    handeye = yaml.safe_load((ROOT/'src/fairino3mt_v6_moveit2_config/config/dabai_dcw2_handeye.yaml').read_text())
    optical_tf = transform_rpy([.001, -.002, .003], [-np.pi/2, 0., -np.pi/2])
    flange_optical = transform_rpy(handeye['xyz'], handeye['rpy']) @ optical_tf
    static = StaticTransformBroadcaster(node)
    message = TransformStamped(); message.header.frame_id='camera_link'; message.child_frame_id='camera_color_optical_frame'
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = optical_tf[:3, 3].tolist()
    q = quaternion_from_rotation(optical_tf[:3, :3]).tolist()
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = q
    static.sendTransform(message)
    target = transform_rpy([.05, -.3, .03], [0., 0., 0.])

    def publish():
        stamp = (node.get_clock().now()-Duration(seconds=.06)).to_msg()
        try:
            flange, _ = history.at(stamp.sec+stamp.nanosec/1e9)
        except ValueError:
            return
        optical = np.linalg.inv(flange @ flange_optical) @ target
        pose = PoseStamped(); pose.header.frame_id='camera_color_optical_frame'; pose.header.stamp=stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = optical[:3, 3].tolist()
        q = quaternion_from_rotation(optical[:3, :3]).tolist()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        tags.publish(pose)
        quality.publish(String(data=json.dumps(dict(status='valid', target_id=1, frame_id='camera_color_optical_frame',
            stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec), valid_position=True, orientation_measurement_valid=True,
            orientation_ambiguous=False, solver='IPPE_SQUARE', candidate_count=2, selected_candidate_index=0,
            candidate_errors_px=[.01, .6], candidate_rotation_distance_deg=30.))))
    timer = node.create_timer(.05, publish)
    output = Path(os.environ.get('FR3C_POSE_LAUNCH_TEST_OUTPUT',
                                str(ROOT/'evidence/target_follow/stage9/launch_mock')))
    output.mkdir(parents=True, exist_ok=True)
    process = None
    env = dict(os.environ, FR3C_FOLLOW_DOMAIN_ID=os.environ['ROS_DOMAIN_ID'])

    def pump(duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            assert process.poll() is None, (output/'launch.log').read_text()
            rclpy.spin_once(node, timeout_sec=.01)

    def wait(predicate, timeout=18):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            pump(.02)
            if states and predicate(states[-1]):
                return states[-1]
        raise AssertionError(states[-3:])

    with (output/'launch.log').open('w') as log:
        try:
            process = subprocess.Popen([str(ROOT/'scripts/target_follow/start_pose_follow.sh'), '--mock', '--no-view',
                'output_dir:='+str(output)], env=env, stdout=log, stderr=subprocess.STDOUT)
            ready = wait(lambda s: s['status'] == 'idle' and s['quality_reason'] == '')
            assert ready['servo_commands_sent'] == 0 and ready['rotation_mapping_gain'] == .5
            assert ready['deadband_enter_deg'] == pytest.approx(6.)
            client = node.create_client(Trigger, '/target_follow/follow_start')
            deadline = time.monotonic()+3
            while not client.service_is_ready() and time.monotonic() < deadline:
                pump(.02)
            assert client.service_is_ready()
            future = client.call_async(Trigger.Request())
            while not future.done() and time.monotonic() < deadline:
                pump(.02)
            assert future.done() and future.result().success
            initial = wait(lambda s: s['starts'] == 1)
            for angle in np.linspace(0, 20, 20):
                target[:3, :3] = transform_rpy([0., 0., 0.], [0., np.radians(angle), 0.])[:3, :3]
                pump(.1)
            pump(3.)
            state = states[-1]
            assert state['status'] in ('following', 'holding_quality')
            assert np.linalg.norm(np.array(state['target_base_m'])-target[:3, 3]) < .0001
            def rotation(row):
                from fr3c_target_follow.follow_geometry import rotation_from_quaternion
                return rotation_from_quaternion(row['actual_tcp_quaternion_xyzw'])
            assert rotation_distance_deg(rotation(state), rotation(initial)) > 2.
            assert node.count_publishers('/target_follow/desired_tcp_pose') == 1
            process.send_signal(signal.SIGINT); process.wait(timeout=12)
            assert process.returncode == 0
            report = json.loads((output/'report.json').read_text())
            session = json.loads((output/'session.json').read_text())
            assert report['mock'] and report['hardware_servo_commands_sent'] == 0 and report['starts'] == 1
            assert report['stop_motion_code'] == report['servo_end_code'] == 0
            assert session['stage'] == 9 and session['driver_parameters']['rotation_mapping_gain'] == .5
            assert session['driver_parameters']['follow_radius_m'] == 1.00
            assert session['driver_parameters']['follow_angle_deg'] == 90.
            assert session['driver_parameters']['recovery_angle_gap_deg'] == 120.
            assert session['driver_parameters']['goal_position_threshold_m'] == .008
            assert session['driver_parameters']['goal_angle_threshold_deg'] == 4.
            assert session['driver_parameters']['goal_confirmation_s'] == .20
            assert 'settled_hold' in state and 'goal_pending' in state
            assert session['orientation_parameters']['orientation_check']['ros__parameters']['jitter_allowance_deg'] == 25.
            assert session['tag_parameters']['tag_pose']['ros__parameters']['orientation_candidate_tolerance_deg'] == 35.
            assert not session['owned_sources']['start_detector'] and not session['owned_sources']['start_camera']
            assert 'Traceback' not in (output/'launch.log').read_text() and 'process has died' not in (output/'launch.log').read_text()
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            names = [name for name, namespace in node.get_node_names_and_namespaces()]
            assert 'pose_follow_launch_synthetic_source' in names and 'pose_follow' not in names and 'orientation_check' not in names
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT); process.wait(timeout=12)
            timer.cancel(); node.destroy_node(); rclpy.shutdown()
