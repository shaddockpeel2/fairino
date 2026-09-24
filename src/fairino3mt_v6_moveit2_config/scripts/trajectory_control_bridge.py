#!/usr/bin/env python3
"""Arbitrate FR3-C trajectory and MoveIt Servo ownership of ServoJ control."""

from __future__ import annotations

import math
import json
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from fairino_msgs.srv import ControlLease
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, Int8, String
from std_srvs.srv import Trigger


class StopWitness:
    """Require distinct, fresh source samples and a continuous stationary window."""

    def __init__(self, since):
        self.since = since
        self.start = None
        self.origin = None
        self.last_receive = None
        self.last_stamp = None
        self.count = 0
        self.confirmed = False

    def observe(self, now, stamp, source_age, positions, velocities):
        valid = (now >= self.since and len(positions) == len(velocities) == 6
                 and all(math.isfinite(v) for v in
                         [now, stamp, source_age, *positions, *velocities])
                 and 0 <= source_age <= 0.1
                 and (self.last_stamp is None or stamp > self.last_stamp)
                 and max(abs(v) for v in velocities) <= 0.002)
        continuous = (self.last_receive is None or now - self.last_receive <= 0.1)
        if not valid or not continuous:
            self.start = self.origin = None
            self.count = 0
            self.confirmed = False
        if not valid:
            return
        self.last_stamp = stamp
        self.last_receive = now
        if (self.origin is None
                or max(abs(a - b) for a, b in zip(positions, self.origin)) > 0.0002):
            self.origin = tuple(positions)
            self.start = now
            self.count = 0
            self.confirmed = False
        self.count += 1
        self.confirmed = self.count >= 5 and now - self.start >= 0.2

    def ready(self, now):
        return (self.confirmed and self.last_receive is not None
                and 0 <= now - self.last_receive <= 0.1)


class TrajectoryControlBridge(Node):
    """Proxy trajectories while owning controller lifecycle transitions."""

    def __init__(self):
        """Create the public action proxy and private execution client."""
        super().__init__('fr3c_trajectory_control_bridge')
        self.declare_parameter(
            'proxy_action', '/fairino3mt_controller/follow_joint_trajectory')
        self.declare_parameter(
            'execution_action',
            '/fairino3mt_execution_controller/follow_joint_trajectory')
        self.declare_parameter(
            'execution_controller', 'fairino3mt_execution_controller')
        self.declare_parameter(
            'servo_controller', 'fairino3mt_servo_position_controller')
        self.declare_parameter(
            'switch_service', '/controller_manager/switch_controller')
        self.declare_parameter(
            'list_controllers_service',
            '/controller_manager/list_controllers')
        self.declare_parameter('service_wait_timeout_s', 10.0)
        self.declare_parameter('switch_timeout_s', 5.0)
        self.declare_parameter('control_lease_service', '/fairino/control_lease')
        self.declare_parameter('servo_start_service', '/servo_node/start_servo')
        self.declare_parameter('servo_stop_service', '/servo_node/stop_servo')
        self.declare_parameter('servo_status_topic', '/servo_node/status')
        self.declare_parameter(
            'servo_command_topic',
            '/fairino3mt_servo_position_controller/commands')
        self.declare_parameter('servo_command_timeout_s', 0.25)
        self.declare_parameter('tracking_heartbeat_timeout_s', 0.5)
        self.declare_parameter('watchdog_period_s', 0.1)
        self.declare_parameter('stop_confirmation_timeout_s', 2.0)

        self._execution_controller = str(
            self.get_parameter('execution_controller').value)
        self._servo_controller = str(
            self.get_parameter('servo_controller').value)
        if self._servo_controller == self._execution_controller:
            raise ValueError(
                'servo_controller and execution_controller must be distinct')
        self._service_wait_timeout_s = float(
            self.get_parameter('service_wait_timeout_s').value)
        self._switch_timeout_s = float(
            self.get_parameter('switch_timeout_s').value)
        self._tracking_heartbeat_timeout_s = float(
            self.get_parameter('tracking_heartbeat_timeout_s').value)
        self._watchdog_period_s = float(
            self.get_parameter('watchdog_period_s').value)
        self._servo_command_timeout_s = float(
            self.get_parameter('servo_command_timeout_s').value)
        self._stop_confirmation_timeout_s = float(
            self.get_parameter('stop_confirmation_timeout_s').value)
        if (not math.isfinite(self._stop_confirmation_timeout_s)
                or not 0.3 <= self._stop_confirmation_timeout_s <= 5.0):
            raise ValueError('stop_confirmation_timeout_s must be in [0.3, 5]')
        if (self._service_wait_timeout_s <= 0.0
                or self._switch_timeout_s <= 0.0
                or self._tracking_heartbeat_timeout_s <= 0.0
                or self._watchdog_period_s <= 0.0
                or self._servo_command_timeout_s <= 0.0):
            raise ValueError(
                'bridge timeout parameters must be positive')

        callback_group = ReentrantCallbackGroup()
        self._callback_group = callback_group
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String, '/fairino/control_state', state_qos)
        self._switch_client = self.create_client(
            SwitchController,
            str(self.get_parameter('switch_service').value),
            callback_group=callback_group,
        )
        self._list_controllers_client = self.create_client(
            ListControllers,
            str(self.get_parameter('list_controllers_service').value),
            callback_group=callback_group,
        )
        self._execution_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter('execution_action').value),
            callback_group=callback_group,
        )
        self._servo_start_client = self.create_client(
            Trigger, str(self.get_parameter('servo_start_service').value),
            callback_group=callback_group)
        self._servo_stop_client = self.create_client(
            Trigger, str(self.get_parameter('servo_stop_service').value),
            callback_group=callback_group)
        self._path_ready_client = self.create_client(
            Trigger, '/p1/path_guard/ready', callback_group=callback_group)
        self._stop_report_publisher = self.create_publisher(
            String, '/fairino/stop_report', state_qos)

        self._lock = threading.RLock()
        self._goal_reserved = False
        self._execution_goal_handle = None
        self._control_acquired = False
        self._owner = None
        self._lease_id = ''
        self._last_tracking_heartbeat_s = 0.0
        self._lease_timeout_s = self._tracking_heartbeat_timeout_s
        self._cleanup_in_progress = False
        self._fault_latched = False
        self._service_timeout_latched = False
        self._servo_hard_fault = None
        self._servo_command_fault = None
        self._last_servo_command_s = 0.0
        self._last_released_lease_id = ''
        self._stop_witness = None
        self._lease_service = self.create_service(
            ControlLease,
            str(self.get_parameter('control_lease_service').value),
            self._control_lease_callback,
            callback_group=callback_group,
        )
        self.create_subscription(
            Int8, str(self.get_parameter('servo_status_topic').value),
            self._on_servo_status, 10, callback_group=callback_group)
        self.create_subscription(
            JointState, '/joint_states', self._on_stop_feedback, 10,
            callback_group=callback_group)
        self.create_subscription(
            String, '/p1/path_guard/fault', self._on_path_fault, 10,
            callback_group=callback_group)
        self.create_service(
            Trigger, '/fairino/stop_servo', self._operator_stop,
            callback_group=callback_group)
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter('servo_command_topic').value),
            self._on_servo_command, 10, callback_group=callback_group)
        self._watchdog_timer = self.create_timer(
            self._watchdog_period_s, self._watchdog,
            callback_group=callback_group)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter('proxy_action').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self._publish_state('monitoring_manual_ready')
        self.get_logger().info(
            'Trajectory bridge ready: flange drag remains available until '
            'MoveIt '
            'submits an execution goal')

    def _publish_state(self, state: str) -> None:
        message = String()
        message.data = state
        self._state_publisher.publish(message)

    def _on_stop_feedback(self, message):
        with self._lock:
            if self._stop_witness is None:
                return
            names = [f'joint{i}' for i in range(1, 7)]
            lookup = {name: i for i, name in enumerate(message.name)}
            if (not all(name in lookup for name in names)
                    or len(message.position) != len(message.name)
                    or len(message.velocity) != len(message.name)):
                self._stop_witness.observe(time.monotonic(), 0, 1, [], [])
                return
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            source_age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            self._stop_witness.observe(
                time.monotonic(), stamp, source_age,
                [message.position[lookup[n]] for n in names],
                [message.velocity[lookup[n]] for n in names])

    async def _wait_for_stationary(self):
        future = Future()
        deadline = time.monotonic() + self._stop_confirmation_timeout_s

        def poll():
            with self._lock:
                ready = self._stop_witness.ready(time.monotonic())
            if not future.done() and (ready or time.monotonic() >= deadline):
                future.set_result(ready)

        timer = self.create_timer(0.02, poll, callback_group=self._callback_group)
        try:
            return await future
        finally:
            self.destroy_timer(timer)

    async def _operator_stop(self, _request, response):
        with self._lock:
            if self._owner is None and not self._fault_latched:
                response.success = True
                response.message = 'already_idle'
                return response
            if self._owner != 'SERVO_TRACKING' or self._cleanup_in_progress:
                response.success = False
                response.message = 'not_servo_owner_or_stop_in_progress_or_fault'
                return response
            lease_id = self._lease_id
            self._owner = 'SERVO_TRACKING_STOPPING'
            self._cleanup_in_progress = True
        response.success = await self._stop_tracking_control(lease_id, 'operator_stop')
        response.message = 'stop_verified' if response.success else 'stop_unconfirmed'
        return response

    async def _on_path_fault(self, message):
        with self._lock:
            if self._owner != 'SERVO_TRACKING' or self._cleanup_in_progress:
                return
            lease_id = self._lease_id
            self._owner = 'SERVO_TRACKING_STOPPING'
            self._cleanup_in_progress = True
        await self._stop_tracking_control(lease_id, 'path_guard:' + message.data)

    def _goal_callback(self, goal_request):
        if not goal_request.trajectory.joint_names:
            self.get_logger().error('Rejecting trajectory without joint names')
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().error('Rejecting trajectory without points')
            return GoalResponse.REJECT
        with self._lock:
            if self._fault_latched:
                self.get_logger().error(
                    'Rejecting trajectory: control release fault is latched')
                return GoalResponse.REJECT
            if self._goal_reserved or self._owner is not None:
                self.get_logger().warning(
                    f'Rejecting arm trajectory: control owned by {self._owner}')
                return GoalResponse.REJECT
            self._goal_reserved = True
            self._owner = 'TRAJECTORY'
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        with self._lock:
            execution_goal_handle = self._execution_goal_handle
        if execution_goal_handle is not None:
            execution_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    @staticmethod
    def _duration(seconds: float):
        duration = SwitchController.Request().timeout
        whole_seconds = int(seconds)
        duration.sec = whole_seconds
        duration.nanosec = int((seconds - whole_seconds) * 1_000_000_000)
        return duration

    async def _bounded_call(self, client, request, timeout_s):
        pending = client.call_async(request)
        completed = Future()

        def done(future):
            if completed.done():
                return
            try:
                completed.set_result(future.result())
            except Exception as error:
                completed.set_exception(error)

        def expired():
            if not completed.done():
                with self._lock:
                    self._service_timeout_latched = True
                    self._fault_latched = True
                completed.set_exception(TimeoutError('service_response_timeout'))

        pending.add_done_callback(done)
        timer = self.create_timer(timeout_s, expired, callback_group=self._callback_group)
        try:
            return await completed
        finally:
            self.destroy_timer(timer)
            if not pending.done():
                pending.cancel()
        # A ROS service timeout cannot cancel server-side execution. Callers
        # latch FAULT rather than assuming an unacknowledged switch succeeded.

    async def _controller_states(self):
        if not self._list_controllers_client.wait_for_service(
                timeout_sec=self._service_wait_timeout_s):
            self.get_logger().error(
                'controller_manager list service is unavailable')
            return None
        try:
            response = await self._bounded_call(
                self._list_controllers_client, ListControllers.Request(),
                self._switch_timeout_s)
        except Exception as error:  # pragma: no cover
            self.get_logger().error(
                'Controller list call failed: '
                f'{type(error).__name__}: {error}')
            return None
        return {controller.name: controller.state
                for controller in response.controller}

    async def _switch_controller(self, controller: str, activate: bool) -> bool:
        """Switch one arm controller and verify strict mutual exclusion."""
        if not self._switch_client.wait_for_service(
                timeout_sec=self._service_wait_timeout_s):
            self.get_logger().error(
                'controller_manager switch service is unavailable')
            return False

        managed = (self._execution_controller, self._servo_controller)
        if controller not in managed:
            self.get_logger().error(f'Unmanaged controller requested: {controller}')
            return False
        states = await self._controller_states()
        if states is None:
            return False
        missing = [name for name in managed if name not in states]
        if missing:
            self.get_logger().error(
                f'Arm controllers are not loaded: {missing}')
            return False
        active = {name for name in managed if states[name] == 'active'}
        if len(active) > 1:
            self.get_logger().error(
                'Unsafe controller state: JTC and Servo forward controller '
                'are active concurrently')
            request = SwitchController.Request()
            request.deactivate_controllers = sorted(active)
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.activate_asap = True
            request.timeout = self._duration(self._switch_timeout_s)
            try:
                await self._bounded_call(self._switch_client, request, self._switch_timeout_s)
            except Exception:  # pragma: no cover
                pass
            return False

        request = SwitchController.Request()
        if activate:
            if states[controller] != 'active':
                request.activate_controllers = [controller]
            request.deactivate_controllers = [
                name for name in managed
                if name != controller and states[name] == 'active']
        else:
            if states[controller] == 'active':
                request.deactivate_controllers = [controller]
            # A release never leaves the other arm controller running if it
            # was activated outside this bridge.
            request.deactivate_controllers.extend(
                name for name in managed
                if name != controller and states[name] == 'active')
        if not (request.activate_controllers
                or request.deactivate_controllers):
            return not activate or states[controller] == 'active'
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = self._duration(self._switch_timeout_s)
        try:
            response = await self._bounded_call(
                self._switch_client, request, self._switch_timeout_s)
        except Exception as error:  # pragma: no cover
            self.get_logger().error(
                'Controller switch call failed: '
                f'{type(error).__name__}: {error}')
            return False
        if not response.ok:
            operation = 'activate' if activate else 'deactivate'
            self.get_logger().error(
                f'Failed to {operation} {controller}')
            return False
        verified = await self._controller_states()
        if verified is None:
            return False
        expected_state = 'active' if activate else 'inactive'
        other_active = any(
            verified[name] == 'active' for name in managed
            if name != controller)
        if verified[controller] != expected_state or other_active:
            self.get_logger().error(
                f'Controller switch verification failed: {verified}')
            return False
        return True

    async def _call_servo_trigger(self, client, operation: str) -> bool:
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout_s):
            self.get_logger().error(
                f'MoveIt Servo {operation} service is unavailable')
            return False
        try:
            response = await self._bounded_call(client, Trigger.Request(), self._switch_timeout_s)
        except Exception as error:  # pragma: no cover
            self.get_logger().error(
                f'MoveIt Servo {operation} call failed: '
                f'{type(error).__name__}: {error}')
            return False
        if not response.success:
            self.get_logger().error(
                f'MoveIt Servo {operation} rejected: {response.message}')
            return False
        return True

    def _lease_response(self, response, success, message, lease_id=''):
        response.success = bool(success)
        response.lease_id = lease_id
        with self._lock:
            response.owner_state = (
                'FAULT' if self._fault_latched else (self._owner or 'IDLE'))
        response.message = message
        return response

    async def _control_lease_callback(self, request, response):
        if request.owner.strip().upper() != 'SERVO_TRACKING':
            return self._lease_response(
                response, False, 'unsupported_owner')

        if request.operation == ControlLease.Request.ACQUIRE:
            timeout_s = float(request.heartbeat_timeout_s)
            if timeout_s <= 0.0:
                timeout_s = self._tracking_heartbeat_timeout_s
            timeout_s = min(max(timeout_s, 0.2), 2.0)
            with self._lock:
                if self._fault_latched:
                    return self._lease_response(
                        response, False, 'control_fault_latched')
                if self._owner is not None:
                    return self._lease_response(
                        response, False, f'busy:{self._owner}')
                lease_id = str(uuid.uuid4())
                self._owner = 'SERVO_TRACKING_STARTING'
                self._lease_id = lease_id
                self._lease_timeout_s = timeout_s
                self._cleanup_in_progress = True
            self._publish_state('acquiring_servo_tracking_control')
            path_ready = await self._call_servo_trigger(
                self._path_ready_client, 'path_guard_ready')
            activated = path_ready and await self._switch_controller(
                self._servo_controller, True)
            started = False
            if activated:
                started = await self._call_servo_trigger(
                    self._servo_start_client, 'start')
            if not activated or not started:
                # Even a failed activation can leave a controller in an
                # unexpected state.  Verify that both arm controllers are
                # inactive before claiming monitoring mode is restored.
                if activated:
                    released = await self._stop_tracking_control(lease_id, 'servo_start_failed')
                else:
                    released = await self._switch_controller(self._servo_controller, False)
                released = released and not self._service_timeout_latched
                with self._lock:
                    self._control_acquired = False
                    self._cleanup_in_progress = False
                    self._owner = None if released else 'FAULT'
                    self._lease_id = ''
                    self._fault_latched = not released
                self._publish_state(
                    'monitoring_manual_ready' if released
                    else 'fault_control_not_released')
                return self._lease_response(
                    response, False,
                    'servo_unavailable' if activated else
                    'controller_activation_failed')
            with self._lock:
                self._owner = 'SERVO_TRACKING'
                self._control_acquired = True
                self._last_tracking_heartbeat_s = time.monotonic()
                self._cleanup_in_progress = False
                self._servo_hard_fault = None
                self._servo_command_fault = None
                # Give MoveIt Servo one full watchdog window to publish the
                # first six-axis command after start.
                self._last_servo_command_s = time.monotonic()
            self._publish_state('executing_servo_tracking')
            return self._lease_response(
                response, True, 'tracking_control_acquired', lease_id)

        if request.operation == ControlLease.Request.HEARTBEAT:
            with self._lock:
                valid = (self._owner == 'SERVO_TRACKING'
                         and request.lease_id == self._lease_id
                         and not self._fault_latched)
                if valid:
                    self._last_tracking_heartbeat_s = time.monotonic()
                    lease_id = self._lease_id
                else:
                    lease_id = ''
            return self._lease_response(
                response, valid,
                'heartbeat_accepted' if valid else 'lease_not_active',
                lease_id)

        if request.operation == ControlLease.Request.RELEASE:
            with self._lock:
                if (self._owner is None
                        and request.lease_id == self._last_released_lease_id):
                    return self._lease_response(
                        response, True, 'already_released')
                valid = (self._owner == 'SERVO_TRACKING'
                         and request.lease_id == self._lease_id
                         and not self._cleanup_in_progress)
                if valid:
                    self._cleanup_in_progress = True
                    self._owner = 'SERVO_TRACKING_STOPPING'
            if not valid:
                return self._lease_response(
                    response, False, 'lease_not_active')
            released = await self._stop_tracking_control(
                request.lease_id, 'tracking_release_requested')
            return self._lease_response(
                response, released,
                'tracking_control_released' if released
                else 'tracking_control_release_failed')

        return self._lease_response(response, False, 'invalid_operation')

    async def _stop_tracking_control(self, lease_id, reason):
        self._publish_state('releasing_servo_tracking_control')
        started = time.monotonic()
        with self._lock:
            self._stop_witness = StopWitness(started)
        # Revoke the hardware command interface FIRST. Its mode switch inhibits
        # ServoJ, calls ServoMoveEnd and StopMotion, and latches SDK stop errors.
        controller_stopped = await self._switch_controller(
            self._servo_controller, False)
        servo_stopped = await self._call_servo_trigger(
            self._servo_stop_client, 'stop')
        stationary = await self._wait_for_stationary()
        released = (servo_stopped and controller_stopped and stationary
                    and not self._service_timeout_latched)
        report = {
            'reason': reason, 'controller_inactive': controller_stopped,
            'servo_generator_stopped': servo_stopped, 'stationary_verified': stationary,
            'released': released, 'elapsed_s': time.monotonic() - started,
            'joint_speed_limit_radps': 0.002, 'joint_drift_limit_rad': 0.0002,
            'stationary_window_s': 0.2,
        }
        self._stop_report_publisher.publish(String(data=json.dumps(report)))
        with self._lock:
            self._control_acquired = False if controller_stopped else True
            self._cleanup_in_progress = False
            self._last_released_lease_id = lease_id
            self._lease_id = ''
            if released:
                self._owner = None
            else:
                self._owner = 'FAULT'
                self._fault_latched = True
        self._publish_state(
            'monitoring_manual_ready' if released
            else 'fault_control_not_released')
        if not released:
            self.get_logger().error(
                f'Tracking cleanup failed ({reason}); Servo/control fault '
                'latched')
        return released

    def _on_servo_status(self, message):
        # Servo already stops motion for singularity and joint-bound statuses.
        # Keep the tracking lease so a changed target can command motion away
        # from that boundary; collision halt still forces a full stop.
        if int(message.data) == 4:
            with self._lock:
                self._servo_hard_fault = int(message.data)

    def _on_servo_command(self, message):
        values = list(message.data)
        valid = (len(values) == 6
                 and all(math.isfinite(float(value)) for value in values))
        with self._lock:
            if valid:
                self._last_servo_command_s = time.monotonic()
            elif self._owner in ('SERVO_TRACKING',
                                 'SERVO_TRACKING_STARTING'):
                self._servo_command_fault = (
                    f'invalid_forward_command_size:{len(values)}')

    async def _watchdog(self):
        with self._lock:
            if (self._owner != 'SERVO_TRACKING'
                    or self._cleanup_in_progress):
                return
            now_s = time.monotonic()
            heartbeat_expired = (
                now_s - self._last_tracking_heartbeat_s
                > self._lease_timeout_s)
            hard_fault = self._servo_hard_fault
            command_fault = self._servo_command_fault
            command_expired = (
                now_s - self._last_servo_command_s
                > self._servo_command_timeout_s)
            if (not heartbeat_expired and hard_fault is None
                    and command_fault is None and not command_expired):
                return
            lease_id = self._lease_id
            self._cleanup_in_progress = True
            self._owner = 'SERVO_TRACKING_STOPPING'
        reason = ('tracking_heartbeat_timeout' if heartbeat_expired
                  else f'servo_hard_fault:{hard_fault}'
                  if hard_fault is not None
                  else command_fault
                  if command_fault is not None
                  else 'servo_command_timeout')
        self.get_logger().error(f'Forcing tracking stop: {reason}')
        await self._stop_tracking_control(lease_id, reason)

    def _forward_feedback(self, proxy_goal_handle, feedback_message) -> None:
        proxy_goal_handle.publish_feedback(feedback_message.feedback)

    @staticmethod
    def _error_result(message: str):
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = message
        return result

    async def _execute(self, proxy_goal_handle):
        acquired = False
        released = False
        try:
            self._publish_state('acquiring_automatic_control')
            if not self._execution_client.wait_for_server(
                    timeout_sec=self._service_wait_timeout_s):
                proxy_goal_handle.abort()
                return self._error_result(
                    'execution trajectory action is unavailable')

            if proxy_goal_handle.is_cancel_requested:
                proxy_goal_handle.canceled()
                return self._error_result(
                    'trajectory canceled before control acquisition')

            if not await self._switch_controller(
                    self._execution_controller, True):
                proxy_goal_handle.abort()
                return self._error_result(
                    'automatic control acquisition failed; release the flange '
                    'drag button '
                    'and check robot state')
            acquired = True
            with self._lock:
                self._control_acquired = True
            self._publish_state('executing_automatic')

            if proxy_goal_handle.is_cancel_requested:
                proxy_goal_handle.canceled()
                return self._error_result(
                    'trajectory canceled during control acquisition')

            send_future = self._execution_client.send_goal_async(
                proxy_goal_handle.request,
                feedback_callback=lambda message: self._forward_feedback(
                    proxy_goal_handle, message),
            )
            execution_goal_handle = await send_future
            if not execution_goal_handle.accepted:
                proxy_goal_handle.abort()
                return self._error_result(
                    'execution controller rejected the trajectory')
            with self._lock:
                self._execution_goal_handle = execution_goal_handle
            if proxy_goal_handle.is_cancel_requested:
                execution_goal_handle.cancel_goal_async()

            wrapped_result = await execution_goal_handle.get_result_async()
            self._publish_state('releasing_automatic_control')
            released = await self._switch_controller(
                self._execution_controller, False)
            if released:
                acquired = False
                with self._lock:
                    self._control_acquired = False
                self._publish_state('monitoring_manual_ready')
            else:
                with self._lock:
                    self._fault_latched = True
                    self._owner = 'FAULT'
                self._publish_state('fault_control_not_released')
                proxy_goal_handle.abort()
                return self._error_result(
                    'trajectory ended but automatic control could not be '
                    'released')

            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                proxy_goal_handle.succeed()
            elif (wrapped_result.status == GoalStatus.STATUS_CANCELED
                  or proxy_goal_handle.is_cancel_requested):
                proxy_goal_handle.canceled()
            else:
                proxy_goal_handle.abort()
            return wrapped_result.result
        except Exception as error:  # pragma: no cover
            self.get_logger().error(
                f'Trajectory bridge failed: {type(error).__name__}: {error}')
            proxy_goal_handle.abort()
            return self._error_result(
                'trajectory bridge exception: '
                f'{type(error).__name__}: {error}')
        finally:
            if acquired and not released:
                self._publish_state('releasing_automatic_control')
                if await self._switch_controller(
                        self._execution_controller, False):
                    with self._lock:
                        self._control_acquired = False
                        self._owner = None
                    self._publish_state('monitoring_manual_ready')
                else:
                    with self._lock:
                        self._fault_latched = True
                        self._owner = 'FAULT'
                    self._publish_state('fault_control_not_released')
            with self._lock:
                self._execution_goal_handle = None
                self._goal_reserved = False
                if self._owner == 'TRAJECTORY':
                    self._owner = None

    def destroy_node(self):
        """Destroy action entities before destroying the ROS node."""
        self._action_server.destroy()
        self._execution_client.destroy()
        return super().destroy_node()


def main(args=None):
    """Run the trajectory bridge with reentrant callbacks."""
    rclpy.init(args=args)
    node = TrajectoryControlBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
