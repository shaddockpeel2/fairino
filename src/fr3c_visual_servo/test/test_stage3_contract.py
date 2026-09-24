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


class FakeListControllersClient:
    def __init__(self, states):
        self.states = states

    def wait_for_service(self, timeout_sec):
        return timeout_sec > 0.0

    async def call_async(self, _request):
        return SimpleNamespace(controller=[
            SimpleNamespace(name=name, state=state)
            for name, state in self.states.items()
        ])


class FakeSwitchControllerClient:
    def __init__(self, states):
        self.states = states
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return timeout_sec > 0.0

    async def call_async(self, request):
        self.requests.append(request)
        for name in request.deactivate_controllers:
            self.states[name] = 'inactive'
        # Model controller_manager's stop-before-start resource switch.
        assert sum(state == 'active' for state in self.states.values()) <= 1
        for name in request.activate_controllers:
            self.states[name] = 'active'
        assert sum(state == 'active' for state in self.states.values()) <= 1
        return SimpleNamespace(ok=True)


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
    bridge._service_wait_timeout_s = 1.0
    bridge._switch_timeout_s = 1.0
    bridge._cleanup_in_progress = False
    bridge._fault_latched = False
    bridge._service_timeout_latched = False
    bridge._servo_hard_fault = None
    bridge._servo_command_fault = None
    bridge._last_servo_command_s = time.monotonic()
    bridge._servo_command_timeout_s = 0.25
    bridge._last_released_lease_id = ''
    bridge._execution_controller = 'fairino3mt_execution_controller'
    bridge._servo_controller = 'fairino3mt_servo_position_controller'
    bridge._servo_start_client = object()
    bridge._servo_stop_client = object()
    bridge._path_ready_client = object()
    bridge._stop_witness = None
    bridge._stop_report_publisher = SimpleNamespace(publish=lambda _msg: None)
    bridge.get_logger = lambda: Logger()
    bridge._publish_state = lambda _state: None

    async def switch(_controller, _activate):
        return True

    async def trigger(_client, _operation):
        return True

    bridge._switch_controller = switch
    bridge._call_servo_trigger = trigger
    async def stationary():
        return True

    async def bounded(client, request, _timeout):
        return await client.call_async(request)

    bridge._wait_for_stationary = stationary
    bridge._bounded_call = bounded
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


def test_mock_controller_switch_is_atomic_and_mutually_exclusive():
    bridge = bridge_fixture()
    states = {
        bridge._execution_controller: 'active',
        bridge._servo_controller: 'inactive',
    }
    bridge._list_controllers_client = FakeListControllersClient(states)
    switch_client = FakeSwitchControllerClient(states)
    bridge._switch_client = switch_client
    switched = asyncio.run(
        BRIDGE_MODULE.TrajectoryControlBridge._switch_controller(
            bridge, bridge._servo_controller, True))
    assert switched
    assert states == {
        bridge._execution_controller: 'inactive',
        bridge._servo_controller: 'active',
    }
    assert len(switch_client.requests) == 1
    assert switch_client.requests[0].activate_controllers == [
        bridge._servo_controller]
    assert switch_client.requests[0].deactivate_controllers == [
        bridge._execution_controller]


def test_mock_detects_and_stops_double_active_controller_state():
    bridge = bridge_fixture()
    states = {
        bridge._execution_controller: 'active',
        bridge._servo_controller: 'active',
    }
    bridge._list_controllers_client = FakeListControllersClient(states)
    bridge._switch_client = FakeSwitchControllerClient(states)
    switched = asyncio.run(
        BRIDGE_MODULE.TrajectoryControlBridge._switch_controller(
            bridge, bridge._servo_controller, True))
    assert not switched
    assert set(states.values()) == {'inactive'}


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


def test_servo_start_failure_rolls_back_forward_controller():
    bridge = bridge_fixture()
    calls = []

    async def switch(controller, activate):
        calls.append((controller, activate))
        return True

    async def trigger(_client, operation):
        return operation != 'start'

    bridge._switch_controller = switch
    bridge._call_servo_trigger = trigger
    response = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert not response.success
    assert calls == [
        ('fairino3mt_servo_position_controller', True),
        ('fairino3mt_servo_position_controller', False),
    ]
    assert bridge._owner is None
    assert not bridge._fault_latched


def test_failed_startup_rollback_latches_fault():
    bridge = bridge_fixture()

    async def switch(_controller, _activate):
        return False

    bridge._switch_controller = switch
    response = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert not response.success
    assert bridge._owner == 'FAULT'
    assert bridge._fault_latched


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


def test_forward_command_timeout_forces_controller_release():
    bridge = bridge_fixture()
    calls = []

    async def switch(controller, activate):
        calls.append((controller, activate))
        return True

    bridge._switch_controller = switch
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert acquired.success
    bridge._last_servo_command_s = time.monotonic() - 1.0
    asyncio.run(bridge._watchdog())
    assert calls == [
        ('fairino3mt_servo_position_controller', True),
        ('fairino3mt_servo_position_controller', False),
    ]
    assert bridge._owner is None
    assert not bridge._control_acquired


def test_valid_forward_command_refreshes_watchdog():
    bridge = bridge_fixture()
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert acquired.success
    bridge._last_servo_command_s = time.monotonic() - 1.0
    bridge._on_servo_command(SimpleNamespace(data=[0.0] * 6))
    asyncio.run(bridge._watchdog())
    assert bridge._owner == 'SERVO_TRACKING'


@pytest.mark.parametrize('data', [[0.0] * 5, [0.0] * 5 + [float('nan')]])
def test_invalid_forward_command_latches_stop(data):
    bridge = bridge_fixture()
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert acquired.success
    bridge._on_servo_command(SimpleNamespace(data=data))
    asyncio.run(bridge._watchdog())
    assert bridge._owner is None


@pytest.mark.parametrize('status', [2, 5])
def test_reachability_halt_keeps_tracking_lease(status):
    bridge = bridge_fixture()
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert acquired.success
    bridge._on_servo_status(SimpleNamespace(data=status))
    asyncio.run(bridge._watchdog())
    assert bridge._owner == 'SERVO_TRACKING'
    assert bridge._control_acquired


def test_collision_halt_forces_tracking_release():
    bridge = bridge_fixture()
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert acquired.success
    bridge._on_servo_status(SimpleNamespace(data=4))
    asyncio.run(bridge._watchdog())
    assert bridge._owner is None
    assert not bridge._control_acquired


def test_action_server_only_latches_collision_halt():
    from fr3c_visual_servo.track_2d_action_server import Track2DActionServer
    server = object.__new__(Track2DActionServer)
    server._lock = threading.Lock()
    server._servo_fault = None
    for status in (2, 5):
        server._on_servo_status(SimpleNamespace(data=status))
        assert server._servo_fault is None
    server._on_servo_status(SimpleNamespace(data=4))
    assert server._servo_fault == 4


def test_failed_controller_release_latches_fault():
    bridge = bridge_fixture()
    calls = []

    async def switch(_controller, activate):
        calls.append(activate)
        return activate

    bridge._switch_controller = switch
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


def test_stop_requires_feedback_even_after_successful_sdk_switch():
    bridge = bridge_fixture()
    order = []
    reports = []

    async def switch(_controller, activate):
        order.append('activate' if activate else 'inhibit_and_stop_hardware')
        return True

    async def trigger(_client, operation):
        order.append(operation)
        return True

    async def stationary():
        order.append('verify_stationary')
        return False

    bridge._switch_controller = switch
    bridge._call_servo_trigger = trigger
    bridge._wait_for_stationary = stationary
    bridge._stop_report_publisher = SimpleNamespace(publish=reports.append)
    acquired = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    response = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.RELEASE, acquired.lease_id), ControlLease.Response()))
    assert not response.success
    assert bridge._fault_latched
    assert order[-3:] == ['inhibit_and_stop_hardware', 'stop', 'verify_stationary']
    assert '"stationary_verified": false' in reports[0].data


def test_missing_path_guard_blocks_activation():
    bridge = bridge_fixture()
    activations = []

    async def trigger(_client, operation):
        return operation != 'path_guard_ready'

    async def switch(_controller, activate):
        activations.append(activate)
        return True

    bridge._switch_controller = switch
    bridge._call_servo_trigger = trigger
    response = asyncio.run(bridge._control_lease_callback(
        lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
    assert not response.success
    assert True not in activations


def test_path_fault_and_operator_stop_use_same_verified_transaction():
    for path_fault in (False, True):
        bridge = bridge_fixture()
        asyncio.run(bridge._control_lease_callback(
            lease_request(ControlLease.Request.ACQUIRE), ControlLease.Response()))
        if path_fault:
            asyncio.run(bridge._on_path_fault(SimpleNamespace(data='path_collision')))
        else:
            result = asyncio.run(bridge._operator_stop(None, SimpleNamespace()))
            assert result.success
        assert bridge._owner is None
        assert bridge._stop_witness is not None


def test_stop_witness_requires_fresh_distinct_stationary_window():
    witness = BRIDGE_MODULE.StopWitness(10)
    for i in range(12):
        witness.observe(10 + i * 0.02, 100 + i * 0.02, 0.01, [0] * 6, [0] * 6)
    assert witness.ready(10.22)
    assert not witness.ready(10.5)
    # Cached repeated source timestamps cannot manufacture stopped evidence.
    witness.observe(10.24, 100.22, 0.01, [0] * 6, [0] * 6)
    assert not witness.ready(10.24)


@pytest.mark.parametrize('positions,velocities,age', [
    ([0] * 6, [0.003] * 6, 0.01),
    ([0] * 6, [0] * 6, 0.2),
    ([0] * 6, [], 0.01),
    ([float('nan')] * 6, [0] * 6, 0.01),
])
def test_stop_witness_rejects_moving_stale_missing_or_invalid_feedback(positions, velocities, age):
    witness = BRIDGE_MODULE.StopWitness(0)
    for i in range(30):
        witness.observe(i * 0.02, 100 + i * 0.02, age, positions, velocities)
    assert not witness.ready(0.58)


def test_stop_witness_rejects_slow_position_drift_even_with_zero_reported_speed():
    witness = BRIDGE_MODULE.StopWitness(0)
    for i in range(30):
        witness.observe(i * 0.02, 100 + i * 0.02, 0.01, [i * 0.0001] * 6, [0] * 6)
    assert not witness.ready(0.58)


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
    ((0, 0, 0.006), (0, 0, 0, 1), 'height_drift'),
    ((0, 0, 0), (0, 0, 0.1, 0.995), 'orientation_drift'),
])
def test_real_tcp_bounds_include_height_and_orientation(xyz, quaternion, reason):
    from fr3c_visual_servo.track_2d_action_server import Track2DActionServer
    server = object.__new__(Track2DActionServer)
    server._base_frame = 'base_link'
    server._tf_timeout_s = 0.01
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


def test_real_tcp_can_move_beyond_old_80_mm_boundary():
    from fr3c_visual_servo.track_2d_action_server import Track2DActionServer
    server = object.__new__(Track2DActionServer)
    server._base_frame = 'base_link'
    server._tf_timeout_s = 0.01
    server._tcp_origin = (0, 0, 0, 0, 0, 0, 1)
    server._check_transform_age = lambda _: None
    values = {'tcp_frame': 'grasp_frame', 'max_z_drift_m': 0.005,
              'max_orientation_drift_rad': 0.05}
    server.get_parameter = lambda name: SimpleNamespace(value=values[name])
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.081, y=0.0, z=0.0),
        rotation=SimpleNamespace(x=0, y=0, z=0, w=1)))
    server._tf_buffer = SimpleNamespace(lookup_transform=lambda *a, **kw: transform)
    server._check_tcp_bounds()
    assert server._measured_xy_m == pytest.approx(0.081)
