"""ROS bridge for perception pause/resume and robot-action guards."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .coordinator import PerceptionSnapshot, RobotSnapshot


class RosResourceBridge(Node):
    def __init__(self, perception_status_topic: str,
                 perception_control_service: str,
                 grasp_status_topic: str) -> None:
        super().__init__('fr3c_qwen_resource_bridge')
        self._condition = threading.Condition()
        self._perception: dict[str, Any] = {}
        self._perception_observed = 0.0
        self._robot: dict[str, Any] = {}
        self._robot_observed = 0.0
        self.create_subscription(
            String, perception_status_topic, self._on_perception, 10)
        self.create_subscription(String, grasp_status_topic, self._on_robot, 10)
        self.control = self.create_client(SetBool, perception_control_service)

    @staticmethod
    def _decode(message: String) -> dict[str, Any] | None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return None
        return value if isinstance(value, dict) else None

    def _on_perception(self, message: String) -> None:
        payload = self._decode(message)
        if payload is None:
            return
        with self._condition:
            self._perception = payload
            self._perception_observed = time.monotonic()
            self._condition.notify_all()

    def _on_robot(self, message: String) -> None:
        payload = self._decode(message)
        if payload is None:
            return
        with self._condition:
            self._robot = payload
            self._robot_observed = time.monotonic()
            self._condition.notify_all()

    def perception_snapshot(self) -> PerceptionSnapshot:
        with self._condition:
            value = self._perception.copy()
            observed = self._perception_observed
        mode = str(value.get('detection_mode', ''))
        return PerceptionSnapshot(
            available=bool(value),
            enabled=bool(value.get('inference_enabled', True)),
            active_inferences=int(value.get('active_inferences', 0)),
            pause_generation=int(value.get('pause_generation', 0)),
            frames_inferred=int(value.get('frames_inferred', 0)),
            error_count=int(value.get('error_count', 0)),
            last_success_ns=value.get('last_success_ns'),
            observed_monotonic=observed,
            detection_mode=mode,
            npu_inference=bool(value.get('npu_inference', mode == 'yolo')),
        )

    def robot_snapshot(self) -> RobotSnapshot:
        with self._condition:
            value = self._robot.copy()
            observed = self._robot_observed
        return RobotSnapshot(
            available=bool(value),
            active=bool(value.get('action_active', False)),
            state=str(value.get('state', '')),
            observed_monotonic=observed,
        )

    def _set_enabled(self, enabled: bool, timeout_s: float) -> None:
        started = time.monotonic()
        if not self.control.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError('perception control service is unavailable')
        request = SetBool.Request()
        request.data = enabled
        future = self.control.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        remaining = timeout_s - (time.monotonic() - started)
        if remaining <= 0 or not completed.wait(remaining):
            raise TimeoutError('perception control request timed out')
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response is not None else 'empty response'
            raise RuntimeError(f'perception control rejected: {detail}')

    def _wait_perception(
        self, predicate: Callable[[dict[str, Any]], bool], timeout_s: float,
    ) -> PerceptionSnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._perception and predicate(self._perception):
                    return self.perception_snapshot()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('perception status confirmation timed out')
                self._condition.wait(remaining)

    def pause_and_drain(self, timeout_s: float) -> PerceptionSnapshot:
        started = time.monotonic()
        self._set_enabled(False, timeout_s)
        remaining = max(0.001, timeout_s - (time.monotonic() - started))
        return self._wait_perception(
            lambda value: value.get('inference_enabled') is False
            and int(value.get('active_inferences', -1)) == 0,
            remaining)

    def resume_and_verify(self, frames_before: int,
                          timeout_s: float) -> PerceptionSnapshot:
        started = time.monotonic()
        self._set_enabled(True, timeout_s)
        remaining = max(0.001, timeout_s - (time.monotonic() - started))
        return self._wait_perception(
            lambda value: value.get('inference_enabled') is True
            and int(value.get('active_inferences', -1)) == 0
            and int(value.get('frames_inferred', 0)) > frames_before
            and value.get('last_success_ns') is not None,
            remaining)


class RosBridgeRuntime:
    def __init__(self, status_topic: str, control_service: str,
                 grasp_topic: str) -> None:
        rclpy.init(args=None)
        self.bridge = RosResourceBridge(
            status_topic, control_service, grasp_topic)
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.bridge,), daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=2.0)
