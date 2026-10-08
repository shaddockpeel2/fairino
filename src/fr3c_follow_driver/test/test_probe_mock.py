"""Exercise the built executable with a mock client; never connect a robot."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1',
                                reason='explicit mock ROS integration run only')
ROOT = Path(__file__).resolve().parents[3]
BIN = ROOT / 'install/fr3c_follow_driver/lib/fr3c_follow_driver/servo_probe'


def command(name, *arguments):
    output = ROOT / 'evidence/target_follow/stage3/mock_integration' / name
    return [str(BIN), '--mock', '--move', '--output-dir', str(output), *arguments], output


@pytest.mark.parametrize('axis,distance', [('x', 5), ('y', -5), ('z', 5)])
def test_complete_translation(axis, distance):
    cmd, output = command('axis_' + axis, '--axis', axis, '--distance-mm', str(distance))
    completed = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    assert completed.returncode == 0, completed.stderr
    report = json.loads((output / 'report.json').read_text())
    assert report['mock'] and report['hardware_servo_commands_sent'] == 0
    assert report['result'] == 'completed'
    delta = report['measured_delta_mm']
    assert delta['xyz'.index(axis)] == pytest.approx(distance)
    assert sum(abs(v) for v in delta) == pytest.approx(abs(distance))
    assert report['measured_rotation_delta_deg'] == pytest.approx(0)
    assert report['stop_motion_code'] == report['servo_end_code'] == 0


def test_scheduled_stop_does_not_finish_the_move():
    cmd, output = command('scheduled_stop', '--stop-after-s', '.3')
    completed = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    assert completed.returncode == 0, completed.stderr
    report = json.loads((output / 'report.json').read_text())
    assert report['result'] == 'stopped' and report['stop_reason'] == 'scheduled'
    assert 0 < report['measured_delta_mm'][0] < 5
    assert report['stop_motion_code'] == report['servo_end_code'] == 0


@pytest.mark.parametrize('method', ['service', 'signal'])
def test_stop_during_active_motion(method):
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    cmd, output = command('stop_' + method)
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    rclpy.init()
    node = rclpy.create_node('mock_stop_test_' + method)
    states = []
    node.create_subscription(String, '/target_follow/servo_state',
                             lambda msg: states.append(json.loads(msg.data)), 10)
    client = node.create_client(Trigger, '/target_follow/servo_stop')
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not any(s['status'] == 'moving' for s in states):
            assert process.poll() is None, process.communicate()
            rclpy.spin_once(node, timeout_sec=.02)
        assert any(s['status'] == 'moving' for s in states)
        if method == 'signal':
            process.send_signal(signal.SIGINT)
        else:
            assert client.wait_for_service(timeout_sec=1)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            assert future.result() is not None and future.result().success
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 0, stderr + stdout
        report = json.loads((output / 'report.json').read_text())
        assert report['result'] == 'stopped' and report['stop_reason'] == method
        assert report['measured_delta_mm'][0] < 5
        assert report['stop_motion_code'] == report['servo_end_code'] == 0
        assert report['hardware_servo_commands_sent'] == 0
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.communicate(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
