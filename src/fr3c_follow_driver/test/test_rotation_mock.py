"""Built stage 8 executable and operator entry, using a mock SDK only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1',
                                reason='explicit isolated mock ROS run required')
ROOT = Path(__file__).resolve().parents[3]
ENTRY = ROOT/'scripts/target_follow/rotation_demo.sh'
BIN = ROOT/'install/fr3c_follow_driver/lib/fr3c_follow_driver/rotation_probe'


def command(name, *args):
    output = Path(os.environ.get('FR3C_ROTATION_TEST_OUTPUT_ROOT',
                                str(ROOT/'evidence/target_follow/stage8/mock_integration')))/name
    env = dict(os.environ, FR3C_FOLLOW_DOMAIN_ID=os.environ['ROS_DOMAIN_ID'])
    return [str(ENTRY), '--mock', '--output-dir', str(output), *args], output, env


def report(output):
    row = json.loads((output/'report.json').read_text())
    assert row['mock'] and row['hardware_servo_commands_sent'] == 0
    assert not row['live_rotation_validated']
    return row


@pytest.mark.parametrize('axis,angle', [('x', 2.5), ('y', -2.5), ('z', 2.5), ('xyz', -2.5)])
def test_round_trip_preserves_tcp_center_and_rotation_axis(axis, angle):
    from fr3c_target_follow.follow_geometry import transform
    from fr3c_target_follow.pose_preview import rotation_vector
    cmd, output, env = command('axis_'+axis, '--move', '--axis', axis, '--angle-deg', str(angle))
    completed = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=25)
    assert completed.returncode == 0, completed.stderr+completed.stdout
    row = report(output)
    assert row['result'] == 'completed' and row['round_trip']
    assert row['ik_preflight_passed'] and not row['mock_ik_is_physical']
    assert row['max_observed_rotation_deg'] == pytest.approx(abs(angle), abs=1e-7)
    assert row['max_observed_tcp_drift_mm'] < 1e-7
    assert row['final_tcp_angle_to_start_deg'] < 1e-7
    assert row['stop_motion_code'] == row['servo_end_code'] == row['close_code'] == 0
    records = [json.loads(line) for line in (output/'samples.jsonl').read_text().splitlines()]
    samples = [s for s in records if s['kind'] == 'servo']
    peak = max(samples, key=lambda s: abs(s['planned_angle_deg']))
    start = transform(row['initial_tcp']['position_m'], row['initial_tcp']['quaternion_xyzw'])
    target = transform(peak['reference_tcp']['position_m'], peak['reference_tcp']['quaternion_xyzw'])
    vector = rotation_vector(target[:3, :3] @ start[:3, :3].T)
    axis_vector = np.ones(3)/np.sqrt(3) if axis == 'xyz' else np.eye(3)['xyz'.index(axis)]
    assert vector == pytest.approx(axis_vector*np.radians(angle), abs=1e-8)
    assert target[:3, 3] == pytest.approx(start[:3, 3], abs=1e-12)
    assert {'rotating_out', 'holding_out', 'rotating_back', 'holding_back'} <= {s['phase'] for s in samples}
    assert len([s for s in records if s['kind'] == 'ik_preflight']) == 11


def test_one_way_with_rotated_tool_and_scheduled_stop(tmp_path):
    tcp = tmp_path/'tcp.yaml'
    tcp.write_text('ydi_j22_tcp:\n  calibrated: true\n  parent_frame: flange_link\n  child_frame: grasp_frame\n  xyz_m: [.002, -.003, .185]\n  rpy_rad: [.1, -.2, .3]\n')
    for name, extra in [('one_way', []), ('scheduled_stop', ['--stop-after-s', '.3'])]:
        cmd, output, env = command(name, '--move', '--axis', 'y', '--angle-deg', '-2.5',
                                  '--one-way', '--tcp-file', str(tcp), *extra)
        completed = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=20)
        assert completed.returncode == 0, completed.stderr+completed.stdout
        row = report(output)
        assert row['final_tcp_drift_mm'] < 1e-7
        assert row['flange_tcp_m_rad'][3:] == [.1, -.2, .3]
        assert row['stop_motion_code'] == row['servo_end_code'] == 0
        if extra:
            assert row['result'] == 'stopped' and row['stop_reason'] == 'scheduled'
            assert 0 < row['final_tcp_angle_to_start_deg'] < 2.5
        else:
            assert row['result'] == 'completed'
            assert row['final_tcp_angle_to_start_deg'] == pytest.approx(2.5)


def test_inspect_and_invalid_limits_never_send():
    cmd, output, env = command('inspect')
    completed = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=10)
    assert completed.returncode == 0, completed.stderr
    row = report(output)
    assert row['result'] == 'inspected' and row['servo_commands_sent'] == 0
    assert 'servo_start_code' not in row and (output/'tcp_used.yaml').exists()
    for args in [[], ['--angle-deg', '6'], ['--period-ms', '30'], ['--speed-deg-s', 'nan']]:
        extra = [] if not args else ['--tcp-m-rad', '0', '0', '.185', '0', '0', '0']
        invalid = subprocess.run([str(BIN), '--mock', '--move', *extra, *args],
                                 capture_output=True, text=True, timeout=10)
        assert invalid.returncode == 1


@pytest.mark.parametrize('method', ['service', 'signal'])
def test_mid_rotation_stop_does_not_automatically_return(method):
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    cmd, output, env = command('stop_'+method, '--move', '--angle-deg', '5')
    rclpy.init()
    node = rclpy.create_node('rotation_stop_test_'+method)
    states = []
    node.create_subscription(String, '/target_follow/servo_state', lambda m: states.append(json.loads(m.data)), 50)
    client = node.create_client(Trigger, '/target_follow/servo_stop')
    process = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic()+12
        while time.monotonic() < deadline and not any(s['status'] == 'rotating_out' for s in states):
            assert process.poll() is None, process.communicate()
            rclpy.spin_once(node, timeout_sec=.02)
        assert any(s['status'] == 'rotating_out' for s in states)
        if method == 'signal':
            process.send_signal(signal.SIGINT)
        else:
            assert client.wait_for_service(timeout_sec=1)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            assert future.result() and future.result().success
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 0, stderr+stdout
        row = report(output)
        assert row['result'] == 'stopped' and row['stop_reason'] == method
        assert 0 < row['final_tcp_angle_to_start_deg'] < 5
        assert row['stop_motion_code'] == row['servo_end_code'] == 0
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT); process.communicate(timeout=10)
        node.destroy_node(); rclpy.shutdown()


def test_duplicate_feedback_owner_prevents_servo_start():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node = rclpy.create_node('rotation_duplicate_feedback_test')
    node.create_publisher(PoseStamped, '/target_follow/flange_pose', 10)
    cmd, output, env = command('duplicate_owner', '--move')
    try:
        completed = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=15)
        assert completed.returncode == 1, completed.stdout
        row = report(output)
        assert row['result'] == 'fault' and 'another follow feedback owner' in row['error']
        assert row['servo_commands_sent'] == 0 and 'servo_start_code' not in row
    finally:
        node.destroy_node(); rclpy.shutdown()
