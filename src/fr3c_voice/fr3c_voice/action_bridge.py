"""ROS 2 Action bridge for one colour-targeted grasp/place task."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import queue
import re
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fairino_msgs.action import PickColor
from fairino_msgs.action import MoveToPose

from .image_capture import RgbFrame, frame_from_message
from .task_guard import TaskDispatchGuard


@dataclass(frozen=True)
class ActionUiEvent:
    kind: str
    request_id: str = ''
    phase: str = ''
    detail: str = ''
    success: bool | None = None
    error_code: int | None = None
    color: str = ''
    zone: str = ''
    support_color: str = ''
    executed: bool | None = None


class ColorActionBridge(Node):
    HOLD_ACTION = '/fr3c_first_grasp/pick_hold'
    SORT_ACTION = '/fr3c_first_grasp/pick_sort'
    PLACE_ACTION = '/fr3c_first_grasp/pick_place'
    STACK_ACTION = '/fr3c_first_grasp/pick_stack'
    AUTO_STACK_ACTION = '/fr3c_first_grasp/pick_stack_auto'
    MOVE_TO_POSE_ACTION = '/fr3c_first_grasp/move_to_pose'
    CANCEL_SERVICE = '/fr3c_first_grasp/cancel'
    RELEASE_ACK_SERVICE = '/fr3c_first_grasp/acknowledge_object_released'
    RESET_TABLETOP_SERVICE = '/fr3c_first_grasp/reset_tabletop_objects'
    STATE_TOPIC = '/fr3c_first_grasp/state'
    PERCEPTION_STATUS_TOPIC = '/perception/blue_cube_localizer/status'
    DEFAULT_COLOR_TOPIC = '/camera/color/image_raw'
    GRIPPER_ACTION = '/ydi_j22_gripper_controller/gripper_cmd'
    GRIPPER_CLOSED_POSITION_M = 0.026
    GRIPPER_OPEN_POSITION_M = 0.060

    def __init__(self, color_topic: str = DEFAULT_COLOR_TOPIC) -> None:
        super().__init__('fr3c_dialogue_action_bridge')
        self.events: queue.Queue[ActionUiEvent] = queue.Queue()
        self.guard = TaskDispatchGuard()
        self._lock = threading.Lock()
        self._active_goal = None
        self._cancel_pending = False
        self._active_gripper_goal = None
        self._gripper_request_id = ''
        self._latest_state: dict = {}
        self._state_observed = 0.0
        self._latest_perception: dict = {}
        self._perception_observed = 0.0
        self._latest_color_frame: RgbFrame | None = None
        self._color_frame_error = ''
        self._action_clients = {
            'pick_hold': ActionClient(self, PickColor, self.HOLD_ACTION),
            'pick_sort': ActionClient(self, PickColor, self.SORT_ACTION),
            'pick_place': ActionClient(self, PickColor, self.PLACE_ACTION),
            'pick_stack': ActionClient(self, PickColor, self.STACK_ACTION),
            'pick_auto_stack': ActionClient(
                self, PickColor, self.AUTO_STACK_ACTION),
        }
        self._cancel_client = self.create_client(Trigger, self.CANCEL_SERVICE)
        self._release_ack_client = self.create_client(
            Trigger, self.RELEASE_ACK_SERVICE)
        self._reset_tabletop_client = self.create_client(
            Trigger, self.RESET_TABLETOP_SERVICE)
        self._gripper_client = ActionClient(
            self, GripperCommand, self.GRIPPER_ACTION)
        self._move_to_pose_client = ActionClient(
            self, MoveToPose, self.MOVE_TO_POSE_ACTION)
        self.create_subscription(String, self.STATE_TOPIC, self._on_state, 10)
        self.create_subscription(
            String, self.PERCEPTION_STATUS_TOPIC, self._on_perception, 10)
        self.create_subscription(
            Image, color_topic, self._on_color_image, qos_profile_sensor_data)

    def _emit(self, event: ActionUiEvent) -> None:
        self.events.put(event)

    def _on_state(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(value, dict):
            return
        with self._lock:
            self._latest_state = value
            self._state_observed = time.monotonic()
        # This topic republishes periodically.  The GUI reads the latest-value
        # snapshot directly, so enqueueing every copy only floods its event
        # loop and log without changing anything on screen.

    def state_snapshot(self) -> tuple[dict, float]:
        with self._lock:
            return self._latest_state.copy(), self._state_observed

    def _on_perception(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(value, dict):
            return
        with self._lock:
            self._latest_perception = value
            self._perception_observed = time.monotonic()
        # Perception status arrives several times per second.  Its snapshot is
        # sufficient for the status bar and must not become no-op UI events.

    def perception_snapshot(self) -> tuple[dict, float]:
        with self._lock:
            return self._latest_perception.copy(), self._perception_observed

    def _on_color_image(self, message: Image) -> None:
        """Cache one immutable raw frame; JPEG encoding happens only on demand."""
        try:
            frame = frame_from_message(message)
        except (TypeError, ValueError) as error:
            with self._lock:
                self._color_frame_error = str(error)
            return
        with self._lock:
            self._latest_color_frame = frame
            self._color_frame_error = ''

    def latest_color_frame(self, max_age_s: float = 1.0) -> RgbFrame:
        if max_age_s <= 0:
            raise ValueError('max_age_s must be positive')
        with self._lock:
            frame = self._latest_color_frame
            error = self._color_frame_error
        if frame is None:
            detail = f'：{error}' if error else ''
            raise RuntimeError(f'还没有收到相机RGB画面{detail}')
        age = time.monotonic() - frame.observed_monotonic
        if age < 0 or age > max_age_s:
            raise RuntimeError(f'相机RGB画面已过期（{age:.2f}秒）')
        return frame

    def action_servers_ready(self) -> bool:
        return (all(client.server_is_ready()
                    for client in self._action_clients.values())
                and self._move_to_pose_client.server_is_ready())

    def action_server_ready(self, task: str) -> bool:
        if task == 'move_to_pose':
            return self._move_to_pose_client.server_is_ready()
        client = self._action_clients.get(task)
        return bool(client is not None and client.server_is_ready())

    def gripper_server_ready(self) -> bool:
        return self._gripper_client.server_is_ready()

    def gripper_command_active(self) -> bool:
        with self._lock:
            return bool(self._gripper_request_id)

    def send(self, request_id: str, task: str, color: str, zone: str = '',
             support_color: str = '', server_timeout_s: float = 2.0) -> None:
        if task not in self._action_clients:
            raise ValueError(f'unsupported task: {task}')
        normalized_zone = re.sub(r'\s+', '', zone or '').lower()
        normalized_support = re.sub(r'\s+', '', support_color or '').lower()
        if task == 'pick_auto_stack':
            # The robot side reads the colours from perception and decides the
            # whole stack order, so this goal must stay completely empty.
            if color or normalized_zone or normalized_support:
                raise ValueError(
                    'pick_auto_stack does not accept a color, zone, or '
                    'support color')
        else:
            if color not in {'red', 'yellow', 'blue', 'green'}:
                raise ValueError(f'unsupported color: {color}')
            if task == 'pick_place':
                if not re.fullmatch(r'[a-z]+(?:[1-9][0-9]?)?', normalized_zone):
                    raise ValueError(f'invalid placement zone: {zone!r}')
            elif normalized_zone:
                raise ValueError(f'{task} does not accept a placement zone')
            if task == 'pick_stack':
                if normalized_support not in {'red', 'yellow', 'blue', 'green'}:
                    raise ValueError(f'invalid stack support color: {support_color!r}')
                if normalized_support == color:
                    raise ValueError('stack source and support colors must differ')
            elif normalized_support:
                raise ValueError(f'{task} does not accept a support color')
        if self.gripper_command_active():
            raise RuntimeError('manual gripper command is still active')
        self.guard.begin(
            request_id, task, color, normalized_zone, normalized_support)
        client = self._action_clients[task]
        if not client.wait_for_server(timeout_sec=server_timeout_s):
            self.guard.finish(
                request_id, False, False, 'Action server is unavailable')
            self._emit(ActionUiEvent(
                kind='result', request_id=request_id, phase='failed',
                detail='抓取 Action 服务不可用。', success=False))
            return
        goal = PickColor.Goal()
        goal.color = color
        goal.zone = normalized_zone
        goal.support_color = normalized_support
        try:
            future = client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: self._on_feedback(
                    request_id, feedback))
        except Exception as error:
            detail = f'Action goal提交失败：{error}'
            with self._lock:
                self._cancel_pending = False
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='result', request_id=request_id,
                phase='failed', detail=detail, success=False))
            return
        future.add_done_callback(
            lambda completed: self._on_goal_response(request_id, completed))

    def send_pose(self, request_id: str, x: float, y: float, z: float,
                  server_timeout_s: float = 2.0) -> None:
        coordinates = (float(x), float(y), float(z))
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError('XYZ coordinates must be finite numbers')
        if self.gripper_command_active():
            raise RuntimeError('manual gripper command is still active')
        self.guard.begin(request_id, 'move_to_pose', '', '', '')
        if not self._move_to_pose_client.wait_for_server(
                timeout_sec=server_timeout_s):
            detail = 'XYZ 移动 Action 服务不可用。'
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='pose_result', request_id=request_id, phase='failed',
                detail=detail, success=False))
            return
        goal = MoveToPose.Goal()
        goal.x, goal.y, goal.z = coordinates
        try:
            future = self._move_to_pose_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: self._on_pose_feedback(
                    request_id, feedback))
        except Exception as error:
            detail = f'XYZ Action goal提交失败：{error}'
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='pose_result', request_id=request_id,
                phase='failed', detail=detail, success=False))
            return
        future.add_done_callback(
            lambda completed: self._on_pose_goal_response(request_id, completed))

    def command_gripper(self, request_id: str, command: str) -> bool:
        """Send one of the two calibrated manual gripper commands.

        The YDI-J22 driver is intentionally two-state.  A close command may
        either hold an object or reach the fully closed pose; the open-loop
        hardware cannot distinguish those outcomes.
        """
        positions = {
            'open': self.GRIPPER_OPEN_POSITION_M,
            'close': self.GRIPPER_CLOSED_POSITION_M,
        }
        if command not in positions:
            raise ValueError(f'unsupported gripper command: {command}')
        if not request_id:
            raise ValueError('request_id must not be empty')
        robot_state, _observed = self.state_snapshot()
        if self.guard.snapshot().active or robot_state.get('action_active'):
            self._emit(ActionUiEvent(
                kind='gripper_result', request_id=request_id,
                phase=command, success=False,
                detail='抓取任务正在运行，已拒绝手动夹爪命令。'))
            return False
        with self._lock:
            if self._gripper_request_id:
                self._emit(ActionUiEvent(
                    kind='gripper_result', request_id=request_id,
                    phase=command, success=False,
                    detail='上一条夹爪命令尚未结束。'))
                return False
            self._gripper_request_id = request_id
        if not self._gripper_client.server_is_ready():
            with self._lock:
                self._gripper_request_id = ''
            self._emit(ActionUiEvent(
                kind='gripper_result', request_id=request_id,
                phase=command, success=False,
                detail='夹爪 Action 服务不可用。'))
            return False
        goal = GripperCommand.Goal()
        goal.command.position = positions[command]
        # This open-loop gripper has no force/current feedback; its driver
        # rejects non-zero effort goals.
        goal.command.max_effort = 0.0
        try:
            future = self._gripper_client.send_goal_async(goal)
        except Exception as error:
            with self._lock:
                self._gripper_request_id = ''
            self._emit(ActionUiEvent(
                kind='gripper_result', request_id=request_id,
                phase=command, success=False,
                detail=f'夹爪命令提交失败：{error}'))
            return False
        future.add_done_callback(
            lambda completed: self._on_gripper_goal_response(
                request_id, command, completed))
        self._emit(ActionUiEvent(
            kind='gripper_submitted', request_id=request_id,
            phase=command, detail='夹爪命令已提交。'))
        return True

    def _on_gripper_goal_response(self, request_id: str, command: str,
                                  future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._finish_gripper_request(request_id)
            self._emit(ActionUiEvent(
                kind='gripper_result', request_id=request_id,
                phase=command, success=False,
                detail=f'夹爪命令提交失败：{error}'))
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_gripper_request(request_id)
            self._emit(ActionUiEvent(
                kind='gripper_result', request_id=request_id,
                phase=command, success=False,
                detail='夹爪驱动拒绝了命令；请检查硬件输出门禁和驱动状态。'))
            return
        with self._lock:
            if self._gripper_request_id == request_id:
                self._active_gripper_goal = goal_handle
        self._emit(ActionUiEvent(
            kind='gripper_accepted', request_id=request_id,
            phase=command, detail='夹爪正在动作。'))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_gripper_result(
                request_id, command, completed))

    def _on_gripper_result(self, request_id: str, command: str, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED
            reached = bool(result and result.reached_goal)
            success = succeeded and reached
            if success:
                detail = (
                    '夹爪已打开。' if command == 'open'
                    else '夹爪已执行闭合；有物体时为夹持，无物体时为完全闭合。')
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                detail = '夹爪命令已取消。'
            else:
                detail = '夹爪动作未成功完成。'
        except Exception as error:
            success = False
            detail = f'读取夹爪结果失败：{error}'
        finally:
            self._finish_gripper_request(request_id)
        self._emit(ActionUiEvent(
            kind='gripper_result', request_id=request_id,
            phase=command, success=success, detail=detail))

    def _finish_gripper_request(self, request_id: str) -> None:
        with self._lock:
            if self._gripper_request_id == request_id:
                self._gripper_request_id = ''
                self._active_gripper_goal = None

    def _on_goal_response(self, request_id: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            detail = f'Action goal提交失败：{error}'
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='result', request_id=request_id,
                phase='failed', detail=detail, success=False))
            return
        if goal_handle is None or not goal_handle.accepted:
            detail = '抓取 Action 拒绝了请求。'
            with self._lock:
                self._cancel_pending = False
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='result', request_id=request_id,
                phase='failed', detail=detail, success=False))
            return
        with self._lock:
            self._active_goal = goal_handle
            cancel_now = self._cancel_pending
        self.guard.update(request_id, 'accepted', 'Action goal accepted')
        self._emit(ActionUiEvent(
            kind='accepted', request_id=request_id, phase='accepted',
            detail='抓取请求已接受，等待实际执行结果。'))
        if cancel_now:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_result(request_id, completed))

    def _on_pose_goal_response(self, request_id: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            detail = f'XYZ Action goal提交失败：{error}'
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='pose_result', request_id=request_id,
                phase='failed', detail=detail, success=False))
            return
        if goal_handle is None or not goal_handle.accepted:
            detail = '机械臂拒绝了 XYZ 移动请求。'
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='pose_result', request_id=request_id,
                phase='failed', detail=detail, success=False))
            return
        with self._lock:
            self._active_goal = goal_handle
            cancel_now = self._cancel_pending
        self.guard.update(request_id, 'accepted', 'XYZ Action goal accepted')
        self._emit(ActionUiEvent(
            kind='pose_accepted', request_id=request_id, phase='accepted',
            detail='XYZ 移动请求已接受，正在审核碰撞。'))
        if cancel_now:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_pose_result(request_id, completed))

    def _on_pose_feedback(self, request_id: str, feedback_message) -> None:
        feedback = feedback_message.feedback
        phase = str(feedback.phase)
        detail = str(feedback.detail)
        self.guard.update(request_id, phase, detail)
        self._emit(ActionUiEvent(
            kind='pose_feedback', request_id=request_id,
            phase=phase, detail=detail))

    def _on_pose_result(self, request_id: str, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(result.success)
            error_code = int(result.error_code)
            detail = str(result.message)
            canceled = error_code == int(MoveToPose.Result.CANCELED)
            self.guard.finish(request_id, success, canceled, detail)
            self._emit(ActionUiEvent(
                kind='pose_result', request_id=request_id,
                phase='canceled' if canceled else 'done' if success else 'failed',
                detail=detail, success=success, error_code=error_code,
                executed=bool(result.executed)))
        except Exception as error:
            detail = f'读取 XYZ Action 结果失败：{error}'
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='pose_result', request_id=request_id,
                phase='failed', detail=detail, success=False))
        finally:
            with self._lock:
                self._active_goal = None
                self._cancel_pending = False

    def _on_feedback(self, request_id: str, feedback_message) -> None:
        feedback = feedback_message.feedback
        phase = str(feedback.phase)
        detail = str(feedback.detail)
        self.guard.update(request_id, phase, detail)
        self._emit(ActionUiEvent(
            kind='feedback', request_id=request_id,
            phase=phase, detail=detail))

    def _on_result(self, request_id: str, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(result.success)
            error_code = int(result.error_code)
            detail = str(result.message)
            canceled = error_code == int(PickColor.Result.CANCELED)
            self.guard.finish(request_id, success, canceled, detail)
            self._emit(ActionUiEvent(
                kind='result', request_id=request_id,
                phase='canceled' if canceled else 'done' if success else 'failed',
                detail=detail, success=success, error_code=error_code,
                color=str(result.color), zone=str(result.zone),
                support_color=str(result.support_color),
                executed=bool(result.executed)))
        except Exception as error:
            detail = f'读取 Action 结果失败：{error}'
            self.guard.finish(request_id, False, False, detail)
            self._emit(ActionUiEvent(
                kind='result', request_id=request_id,
                phase='failed', detail=detail, success=False))
        finally:
            with self._lock:
                self._active_goal = None
                self._cancel_pending = False

    def cancel_direct(self, service_timeout_s: float = 1.0) -> bool:
        """Cancel without entering the model queue."""
        snapshot = self.guard.snapshot()
        with self._lock:
            goal_handle = self._active_goal
            # Keep this flag only while a local goal is still being submitted.
            # A service-only cancel must not cancel the next unrelated goal.
            self._cancel_pending = snapshot.active and goal_handle is None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
            self._emit(ActionUiEvent(
                kind='cancel_requested', detail='已直接请求取消当前 Action。'))
            return True
        if not self._cancel_client.wait_for_service(timeout_sec=service_timeout_s):
            with self._lock:
                self._cancel_pending = False
            self._emit(ActionUiEvent(
                kind='cancel_failed', detail='取消服务不可用。'))
            return False
        future = self._cancel_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_cancel_service)
        self._emit(ActionUiEvent(
            kind='cancel_requested', detail='已直接请求停止抓取。'))
        return True

    def _on_cancel_service(self, future) -> None:
        try:
            response = future.result()
            success = bool(response and response.success)
            detail = str(response.message if response else '取消服务无响应')
        except Exception as error:
            success = False
            detail = f'取消服务失败：{error}'
        with self._lock:
            if self.guard.snapshot().phase != 'submitting':
                self._cancel_pending = False
        self._emit(ActionUiEvent(
            kind='cancel_result', detail=detail, success=success))

    def reset_tabletop_direct(self, service_timeout_s: float = 0.1) -> bool:
        """Clear retained desktop objects without using the model or moving the arm."""
        if self.guard.snapshot().active or self.gripper_command_active():
            self._emit(ActionUiEvent(
                kind='tabletop_reset_result', success=False,
                detail='任务或夹爪命令仍在运行，请等待结束后重置。'))
            return False
        try:
            if not self._reset_tabletop_client.wait_for_service(
                    timeout_sec=service_timeout_s):
                raise RuntimeError('重置服务不可用，请确认抓取节点已更新并启动。')
            future = self._reset_tabletop_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_reset_tabletop_service)
            return True
        except Exception as error:
            self._emit(ActionUiEvent(
                kind='tabletop_reset_result', success=False, detail=str(error)))
            return False

    def _on_reset_tabletop_service(self, future) -> None:
        try:
            response = future.result()
            success = bool(response and response.success)
            detail = str(response.message if response else '重置服务无响应')
        except Exception as error:
            success = False
            detail = f'重置失败：{error}'
        self._emit(ActionUiEvent(
            kind='tabletop_reset_result', success=success, detail=detail))

    def acknowledge_release_direct(self, service_timeout_s: float = 1.0) -> bool:
        """Synchronize a physically empty gripper; this sends no motion command."""
        if self.guard.snapshot().active:
            self._emit(ActionUiEvent(
                kind='release_ack_failed', success=False,
                detail='当前抓取任务仍在运行，不能确认上一物体已取下。'))
            return False
        if not self._release_ack_client.wait_for_service(
                timeout_sec=service_timeout_s):
            self._emit(ActionUiEvent(
                kind='release_ack_failed', success=False,
                detail='物体释放确认服务不可用。'))
            return False
        self._emit(ActionUiEvent(
            kind='release_ack_requested',
            detail='正在同步已取下物体的状态；不会发送运动命令。'))
        try:
            future = self._release_ack_client.call_async(Trigger.Request())
        except Exception as error:
            self._emit(ActionUiEvent(
                kind='release_ack_failed', success=False,
                detail=f'物体释放确认提交失败：{error}'))
            return False
        future.add_done_callback(self._on_release_ack_service)
        return True

    def _on_release_ack_service(self, future) -> None:
        try:
            response = future.result()
            success = bool(response and response.success)
            detail = str(response.message if response else '释放确认服务无响应')
        except Exception as error:
            success = False
            detail = f'释放确认服务失败：{error}'
        self._emit(ActionUiEvent(
            kind='release_ack_result', detail=detail, success=success))


class RosActionRuntime:
    def __init__(self, color_topic: str = ColorActionBridge.DEFAULT_COLOR_TOPIC) -> None:
        rclpy.init(args=None)
        self.bridge = ColorActionBridge(color_topic=color_topic)
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.bridge,), daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=2.0)
