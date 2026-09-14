#!/usr/bin/env python3
"""Acquire ServoJ control only while a MoveIt trajectory is executing."""

from __future__ import annotations

import threading

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
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

        self._execution_controller = str(
            self.get_parameter('execution_controller').value)
        self._service_wait_timeout_s = float(
            self.get_parameter('service_wait_timeout_s').value)
        self._switch_timeout_s = float(
            self.get_parameter('switch_timeout_s').value)
        if (self._service_wait_timeout_s <= 0.0
                or self._switch_timeout_s <= 0.0):
            raise ValueError(
                'service_wait_timeout_s and switch_timeout_s must be positive')

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

        self._lock = threading.Lock()
        self._goal_reserved = False
        self._execution_goal_handle = None
        self._control_acquired = False
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
            if self._goal_reserved:
                self.get_logger().warning(
                    'Rejecting concurrent arm trajectory: busy')
                return GoalResponse.REJECT
            self._goal_reserved = True
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
                    self._publish_state('monitoring_manual_ready')
                else:
                    self._publish_state('fault_control_not_released')
            with self._lock:
                self._execution_goal_handle = None
                self._goal_reserved = False

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
