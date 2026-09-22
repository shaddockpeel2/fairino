import asyncio
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import pytest
from fairino_msgs.srv import ControlLease
from rclpy.action import GoalResponse


BRIDGE_PATH = (
    Path(__file__).resolve().parents[2]
    / 'fairino3mt_v6_moveit2_config'
    / 'scripts'
    / 'trajectory_control_bridge.py'
)
SPEC = importlib.util.spec_from_file_location('trajectory_bridge_test', BRIDGE_PATH)
BRIDGE_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE_MODULE)


class Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def bridge_fixture():
    bridge = object.__new__(BRIDGE_MODULE.TrajectoryControlBridge)
    bridge._lock = threading.RLock()
    bridge._goal_reserved = False
    bridge._execution_goal_handle = None
    bridge._control_acquired = False
    bridge._owner = None
    bridge._lease_id = ''
    bridge._last_tracking_heartbeat_s = 0.0
    bridge._lease_timeout_s = 0.5
    bridge._tracking_heartbeat_timeout_s = 0.5
    bridge._cleanup_in_progress = False
    bridge._fault_latched = False
    bridge._servo_hard_fault = None
    bridge._last_released_lease_id = ''
    bridge._servo_start_client = object()
    bridge._servo_stop_client = object()
    bridge.get_logger = lambda: Logger()
    bridge._publish_state = lambda _state: None

    async def switch(_activate):
        return True

    async def trigger(_client, _operation):
        return True

    bridge._switch_execution_controller = switch
    bridge._call_servo_trigger = trigger
    return bridge


def trajectory_request():
    return SimpleNamespace(trajectory=SimpleNamespace(
        joint_names=['joint1'], points=[object()]))


def lease_request(operation, lease_id=''):
    return SimpleNamespace(
        operation=operation,
        owner='SERVO_TRACKING',
        lease_id=lease_id,
        heartbeat_timeout_s=0.5,
    )


def test_tracking_owner_blocks_trajectory_and_releases_cleanly():
    bridge = bridge_fixture()
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert acquired.success
    assert bridge._owner == 'SERVO_TRACKING'
    assert bridge._goal_callback(trajectory_request()) == GoalResponse.REJECT

    released = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.RELEASE, acquired.lease_id),
        ControlLease.Response()))
    assert released.success
    assert bridge._owner is None


def test_trajectory_reservation_blocks_tracking_acquisition():
    bridge = bridge_fixture()
    assert bridge._goal_callback(trajectory_request()) == GoalResponse.ACCEPT
    response = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert not response.success
    assert response.message == 'busy:TRAJECTORY'
    assert bridge._owner == 'TRAJECTORY'


def test_tracking_heartbeat_requires_matching_lease():
    bridge = bridge_fixture()
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    accepted = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.HEARTBEAT, acquired.lease_id),
        ControlLease.Response()))
    rejected = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.HEARTBEAT, 'wrong-token'),
        ControlLease.Response()))
    assert accepted.success
    assert not rejected.success


def test_expired_tracking_heartbeat_forces_release():
    bridge = bridge_fixture()
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert acquired.success
    bridge._last_tracking_heartbeat_s = time.monotonic() - 1.0
    asyncio.run(bridge._watchdog())
    assert bridge._owner is None
    assert not bridge._control_acquired


def test_failed_controller_release_latches_fault():
    bridge = bridge_fixture()
    calls = []

    async def switch(activate):
        calls.append(activate)
        return activate

    bridge._switch_execution_controller = switch
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    released = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.RELEASE, acquired.lease_id),
        ControlLease.Response()))
    assert calls == [True, False]
    assert not released.success
    assert bridge._fault_latched
    assert bridge._owner == 'FAULT'
    assert bridge._goal_callback(trajectory_request()) == GoalResponse.REJECT


def test_camera_vector_quaternion_rotation():
    from fr3c_visual_servo.track_2d_action_server import Track2DActionServer

    # +90 degrees around base Z maps camera +X to base +Y.
    root_half = 2.0 ** -0.5
    vector = Track2DActionServer._rotate_vector_by_quaternion(
        (1.0, 0.0, 0.0), (0.0, 0.0, root_half, root_half))
    assert abs(vector[0]) < 1e-9
    assert abs(vector[1] - 1.0) < 1e-9
    assert abs(vector[2]) < 1e-9


def test_stop_sequence_runs_without_an_asyncio_event_loop():
    from fr3c_visual_servo.track_2d_action_server import Track2DActionServer

    server = object.__new__(Track2DActionServer)
    server._stop_publish_cycles = 2
    server._config = SimpleNamespace(control_rate_hz=1000.0)
    published = []
    server._publish_expected_twist = lambda *_args, **_kwargs: published.append('expected')
    server._publish_servo_twist = lambda *_args, **_kwargs: published.append('servo')
    server._publish_feedback = lambda *_args, **_kwargs: published.append('feedback')
    coroutine = Track2DActionServer._publish_stop_sequence(
        server, object(), object(), publish_servo=True)
    with pytest.raises(StopIteration):
        coroutine.send(None)
    assert published == ['expected', 'servo', 'feedback'] * 2


@pytest.mark.parametrize('xyz,quaternion,reason', [
    ((0.081, 0, 0), (0, 0, 0, 1), 'measured_relative_xy'),
    ((0, 0, 0.006), (0, 0, 0, 1), 'height_drift'),
    ((0, 0, 0), (0, 0, 0.1, 0.995), 'orientation_drift'),
])
def test_real_tcp_bounds_include_height_and_orientation(xyz, quaternion, reason):
    from fr3c_visual_servo.track_2d_action_server import Track2DActionServer
    server = object.__new__(Track2DActionServer)
    server._base_frame = 'base_link'
    server._tf_timeout_s = 0.01
    server._config = SimpleNamespace(max_relative_xy_m=0.08)
    server._tcp_origin = (0, 0, 0, 0, 0, 0, 1)
    server._check_transform_age = lambda _: None
    values = {'tcp_frame': 'grasp_frame', 'max_z_drift_m': 0.005,
              'max_orientation_drift_rad': 0.05}
    server.get_parameter = lambda name: SimpleNamespace(value=values[name])
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(**dict(zip(('x', 'y', 'z'), xyz))),
        rotation=SimpleNamespace(**dict(zip(('x', 'y', 'z', 'w'), quaternion)))))
    server._tf_buffer = SimpleNamespace(lookup_transform=lambda *a, **kw: transform)
    with pytest.raises(ValueError, match=reason):
        server._check_tcp_bounds()
