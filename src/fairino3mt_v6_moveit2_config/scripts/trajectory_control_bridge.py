#!/usr/bin/env python3
"""Arbitrate FR3-C trajectory and MoveIt Servo ownership of ServoJ control."""

from __future__ import annotations

import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_srvs.srv import Trigger


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
            'switch_service', '/controller_manager/switch_controller')
        self.declare_parameter('service_wait_timeout_s', 10.0)
        self.declare_parameter('switch_timeout_s', 5.0)
        self.declare_parameter('control_lease_service', '/fairino/control_lease')
        self.declare_parameter('servo_start_service', '/servo_node/start_servo')
        self.declare_parameter('servo_stop_service', '/servo_node/stop_servo')
        self.declare_parameter('servo_status_topic', '/servo_node/status')
        self.declare_parameter('tracking_heartbeat_timeout_s', 0.5)
        self.declare_parameter('watchdog_period_s', 0.1)

        self._execution_controller = str(
            self.get_parameter('execution_controller').value)
        self._service_wait_timeout_s = float(
            self.get_parameter('service_wait_timeout_s').value)
        self._switch_timeout_s = float(
            self.get_parameter('switch_timeout_s').value)
        self._tracking_heartbeat_timeout_s = float(
            self.get_parameter('tracking_heartbeat_timeout_s').value)
        self._watchdog_period_s = float(
            self.get_parameter('watchdog_period_s').value)
        if (self._service_wait_timeout_s <= 0.0
                or self._switch_timeout_s <= 0.0
                or self._tracking_heartbeat_timeout_s <= 0.0
                or self._watchdog_period_s <= 0.0):
            raise ValueError(
                'bridge timeout parameters must be positive')

        callback_group = ReentrantCallbackGroup()
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
        self._servo_hard_fault = None
        self._last_released_lease_id = ''
        self._lease_service = self.create_service(
            ControlLease,
            str(self.get_parameter('control_lease_service').value),
            self._control_lease_callback,
            callback_group=callback_group,
        )
        self.create_subscription(
            Int8, str(self.get_parameter('servo_status_topic').value),
            self._on_servo_status, 10, callback_group=callback_group)
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

    async def _switch_execution_controller(self, activate: bool) -> bool:
        if not self._switch_client.wait_for_service(
                timeout_sec=self._service_wait_timeout_s):
            self.get_logger().error(
                'controller_manager switch service is unavailable')
            return False

        request = SwitchController.Request()
        if activate:
            request.activate_controllers = [self._execution_controller]
        else:
            request.deactivate_controllers = [self._execution_controller]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = self._duration(self._switch_timeout_s)
        try:
            response = await self._switch_client.call_async(request)
        except Exception as error:  # pragma: no cover
            self.get_logger().error(
                'Controller switch call failed: '
                f'{type(error).__name__}: {error}')
            return False
        if not response.ok:
            operation = 'activate' if activate else 'deactivate'
            self.get_logger().error(
                f'Failed to {operation} {self._execution_controller}')
            return False
        return True

    async def _call_servo_trigger(self, client, operation: str) -> bool:
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout_s):
            self.get_logger().error(
                f'MoveIt Servo {operation} service is unavailable')
            return False
        try:
            response = await client.call_async(Trigger.Request())
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
            activated = await self._switch_execution_controller(True)
            started = False
            if activated:
                started = await self._call_servo_trigger(
                    self._servo_start_client, 'start')
            if not activated or not started:
                released = (not activated or
                            await self._switch_execution_controller(False))
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
        servo_stopped = await self._call_servo_trigger(
            self._servo_stop_client, 'stop')
        controller_stopped = await self._switch_execution_controller(False)
        released = servo_stopped and controller_stopped
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
        if int(message.data) in (2, 4, 5):
            with self._lock:
                self._servo_hard_fault = int(message.data)

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
            if not heartbeat_expired and hard_fault is None:
                return
            lease_id = self._lease_id
            self._cleanup_in_progress = True
            self._owner = 'SERVO_TRACKING_STOPPING'
        reason = ('tracking_heartbeat_timeout' if heartbeat_expired
                  else f'servo_hard_fault:{hard_fault}')
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

            if not await self._switch_execution_controller(True):
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
            released = await self._switch_execution_controller(False)
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
                if await self._switch_execution_controller(False):
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
