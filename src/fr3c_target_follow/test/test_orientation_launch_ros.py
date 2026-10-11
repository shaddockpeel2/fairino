"""Real launch and audit lifecycle, with externally published synthetic data only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1', reason='isolated ROS domain required')
ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('with_viewer', [False, pytest.param(True, marks=pytest.mark.skipif(
    os.environ.get('FR3C_RUN_GUI_TESTS') != '1', reason='working display required'))])
@pytest.mark.parametrize('quality_profile', ['conservative', 'demo'])
def test_read_only_entry_records_and_closes_owned_nodes_preserving_external_sources(with_viewer, quality_profile):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init(args=[])
    probe = Node('orientation_launch_synthetic_source')
    flange = probe.create_publisher(PoseStamped, '/target_follow/flange_pose', 20)
    feedback = probe.create_publisher(String, '/target_follow/robot_feedback', 20)
    base = probe.create_publisher(PoseStamped, '/target_follow/tag_pose_base', 10)
    quality = probe.create_publisher(String, '/target_follow/quality', 10)

    def publish():
        robot = PoseStamped()
        robot.header.frame_id, robot.header.stamp = 'base_link', probe.get_clock().now().to_msg()
        robot.pose.position.z, robot.pose.orientation.w = .3, 1.
        flange.publish(robot)
        feedback.publish(String(data=json.dumps(dict(
            status='valid', motion_commands_enabled=False, tool_id=2, work_object_id=3,
            tool_coord_mm_deg=[0., 0., 184., 0., 0., 0.],
            work_object_coord_mm_deg=[10., 20., 0., 0., 0., 0.]))))
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = (probe.get_clock().now()-Duration(seconds=.06)).to_msg()
        pose.pose.position.z, pose.pose.orientation.w = .03, 1.
        base.publish(pose)
        quality.publish(String(data=json.dumps(dict(
            status='valid', target_id=1, frame_id='camera_color_optical_frame',
            stamp=dict(sec=pose.header.stamp.sec, nanosec=pose.header.stamp.nanosec),
            valid_position=True, orientation_measurement_valid=True, orientation_ambiguous=False,
            solver='IPPE_SQUARE', candidate_count=2, selected_candidate_index=0,
            candidate_errors_px=[.01, .6], candidate_rotation_distance_deg=30.))))

    timer = probe.create_timer(.05, publish)
    case = 'launch_smoke_gui' if with_viewer else 'launch_smoke'
    if quality_profile == 'demo':
        case += '_demo'
    output = ROOT/'evidence/target_follow/stage6'/case
    output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, FR3C_FOLLOW_DOMAIN_ID=os.environ['ROS_DOMAIN_ID'])
    process = None
    with (output/'launch.log').open('w+') as logfile:
        try:
            process = subprocess.Popen([
                str(ROOT/'scripts/target_follow/start_orientation_check.sh'),
                'viewer:='+str(with_viewer).lower(),
                'quality_profile:='+quality_profile,
                'start_sources:=false', 'duration_s:=1.5', 'scenario:=static', 'output_dir:='+str(output),
            ], env=env, stdout=logfile, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+20
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=.02)
            assert process.poll() == 0, (output/'launch.log').read_text()
            assert 'process has died' not in (output/'launch.log').read_text()
            report = json.loads((output/'report.json').read_text())
            session = json.loads((output/'session.json').read_text())
            assert report['paired_observations'] >= 5
            assert report['duration_s'] >= 1.5
            assert report['trusted_orientation_observations'] > 0
            assert report['hardware_servo_commands_sent'] == 0 and not report['motion_commands_enabled']
            assert session['quality_profile'] == quality_profile
            assert session['detector_params_applied'] is False
            assert report['latest_paired_observation']['confirmation_frames'] == (2 if quality_profile == 'demo' else 5)
            if quality_profile == 'demo':
                assert 'Reusing external detector: its ambiguity threshold is unchanged' in (output/'launch.log').read_text()
            records = [json.loads(line) for line in (output/'samples.jsonl').read_text().splitlines()]
            controller = [row for row in records if row['kind'] == 'feedback']
            assert controller and all(row['tool_id'] == 2 and row['work_object_id'] == 3
                                      and row['tool_coord_mm_deg'][2] == 184.
                                      and row['work_object_coord_mm_deg'][:2] == [10., 20.]
                                      for row in controller)
            assert not any(session['owned_sources'].values())
            names = [name for name, namespace in probe.get_node_names_and_namespaces()]
            assert 'orientation_check' not in names and 'orientation_audit' not in names
            assert 'orientation_launch_synthetic_source' in names
            assert base.get_subscription_count() == 0
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            timer.cancel()
            probe.destroy_node()
            rclpy.shutdown()
