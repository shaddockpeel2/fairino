"""Fail-fast ownership for Qwen and FR3C perception resources."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import threading
import time
from typing import Callable, Protocol, TypeVar

from .trace import TraceWriter


class CoordinationError(RuntimeError):
    pass


class CoordinationBusy(CoordinationError):
    pass


class CoordinationFault(CoordinationError):
    pass


class NpuOwner(str, Enum):
    QWEN_LOAD = 'qwen_load'
    QWEN_TEXT = 'qwen_text'
    QWEN_VISION = 'qwen_vision'


@dataclass(frozen=True)
class PerceptionSnapshot:
    available: bool = False
    enabled: bool = True
    active_inferences: int = 0
    pause_generation: int = 0
    frames_inferred: int = 0
    error_count: int = 0
    last_success_ns: int | None = None
    observed_monotonic: float = 0.0
    detection_mode: str = ''
    npu_inference: bool = False


@dataclass(frozen=True)
class RobotSnapshot:
    available: bool = False
    active: bool = False
    state: str = ''
    observed_monotonic: float = 0.0


class ResourceBridge(Protocol):
    def perception_snapshot(self) -> PerceptionSnapshot: ...
    def robot_snapshot(self) -> RobotSnapshot: ...
    def pause_and_drain(self, timeout_s: float) -> PerceptionSnapshot: ...
    def resume_and_verify(
        self, frames_before: int, timeout_s: float,
    ) -> PerceptionSnapshot: ...


T = TypeVar('T')


class NpuCoordinator:
    """Serialize heavy requests and pause only perception that actually uses NPU."""

    def __init__(
        self,
        bridge: ResourceBridge | None,
        pause_timeout_s: float,
        resume_timeout_s: float,
        status_stale_timeout_s: float,
        minimum_available_memory_mib: int,
        trace: TraceWriter | None = None,
    ) -> None:
        self.bridge = bridge
        self.pause_timeout_s = pause_timeout_s
        self.resume_timeout_s = resume_timeout_s
        self.status_stale_timeout_s = status_stale_timeout_s
        self.minimum_available_memory_mib = minimum_available_memory_mib
        self.trace = trace or TraceWriter(None)
        self._operation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._owner: NpuOwner | None = None
        self._fault = ''

    @property
    def owner(self) -> str:
        with self._state_lock:
            return self._owner.value if self._owner else ''

    @property
    def fault(self) -> str:
        with self._state_lock:
            return self._fault

    @staticmethod
    def available_memory_mib() -> float:
        for line in open('/proc/meminfo', encoding='ascii'):
            if line.startswith('MemAvailable:'):
                return int(line.split()[1]) / 1024.0
        raise CoordinationError('MemAvailable is unavailable')

    def _check_guards(self, request_id: str) -> PerceptionSnapshot:
        with self._state_lock:
            if self._fault:
                raise CoordinationFault(self._fault)
        memory = self.available_memory_mib()
        if memory < self.minimum_available_memory_mib:
            raise CoordinationBusy(
                f'insufficient memory: {memory:.1f} MiB available')
        if self.bridge is None:
            return PerceptionSnapshot()
        now = time.monotonic()
        robot = self.bridge.robot_snapshot()
        if robot.available and now - robot.observed_monotonic > self.status_stale_timeout_s:
            raise CoordinationFault('robot status is stale')
        if robot.active:
            raise CoordinationBusy(f'robot action is active: {robot.state}')
        perception = self.bridge.perception_snapshot()
        if perception.npu_inference:
            if not perception.available:
                raise CoordinationFault('NPU perception status is unavailable')
            if now - perception.observed_monotonic > self.status_stale_timeout_s:
                raise CoordinationFault('NPU perception status is stale')
        self.trace.write(
            'resource_guard_passed', request_id,
            mem_available_mib=memory,
            robot=asdict(robot), perception=asdict(perception))
        return perception

    def run(self, owner: NpuOwner | str, request_id: str,
            operation: Callable[[], T]) -> T:
        try:
            resource_owner = owner if isinstance(owner, NpuOwner) else NpuOwner(owner)
        except ValueError as error:
            raise CoordinationError(f'unsupported NPU owner: {owner}') from error
        if not request_id:
            raise CoordinationError('request_id is required')
        if not self._operation_lock.acquire(blocking=False):
            raise CoordinationBusy(f'NPU is busy: owner={self.owner or "unknown"}')
        restore = False
        frames_before = 0
        operation_error: BaseException | None = None
        try:
            perception = self._check_guards(request_id)
            with self._state_lock:
                self._owner = resource_owner
            restore = bool(perception.available and perception.npu_inference)
            self.trace.write(
                'npu_owner_requested', request_id, owner=resource_owner.value,
                pause_perception=restore,
                perception_mode=perception.detection_mode)
            if restore:
                try:
                    paused = self.bridge.pause_and_drain(self.pause_timeout_s)
                except Exception as error:
                    self._enter_fault(
                        f'pause_failed:{type(error).__name__}:{error}', request_id)
                    raise CoordinationFault(self.fault) from error
                if paused.enabled or paused.active_inferences != 0:
                    self._enter_fault('pause_contract_not_confirmed', request_id)
                    raise CoordinationFault(self.fault)
                frames_before = paused.frames_inferred
            self.trace.write(
                'npu_owner_granted', request_id, owner=resource_owner.value,
                frames_before=frames_before)
            try:
                result = operation()
            except BaseException as error:
                operation_error = error
            if restore:
                try:
                    resumed = self.bridge.resume_and_verify(
                        frames_before, self.resume_timeout_s)
                except Exception as error:
                    message = f'resume_failed:{type(error).__name__}:{error}'
                    if operation_error is not None:
                        message += f';operation_failed:{type(operation_error).__name__}'
                    self._enter_fault(message, request_id)
                    raise CoordinationFault(self.fault) from error
                if not resumed.enabled or resumed.frames_inferred <= frames_before:
                    self._enter_fault('resume_contract_not_confirmed', request_id)
                    raise CoordinationFault(self.fault)
                self.trace.write(
                    'perception_fresh_after_resume', request_id,
                    frames_before=frames_before,
                    frames_after=resumed.frames_inferred,
                    last_success_ns=resumed.last_success_ns)
            self.trace.write(
                'npu_owner_released', request_id, owner=resource_owner.value,
                success=operation_error is None)
            if operation_error is not None:
                raise operation_error
            return result
        finally:
            with self._state_lock:
                self._owner = None
            self._operation_lock.release()

    def reset_fault(self) -> None:
        if not self._operation_lock.acquire(blocking=False):
            raise CoordinationBusy('cannot reset while an operation is active')
        try:
            if self.bridge is not None:
                snapshot = self.bridge.perception_snapshot()
                if snapshot.npu_inference:
                    self.bridge.resume_and_verify(
                        snapshot.frames_inferred, self.resume_timeout_s)
            with self._state_lock:
                self._fault = ''
            self.trace.write('coordination_fault_reset')
        finally:
            self._operation_lock.release()

    def _enter_fault(self, message: str, request_id: str) -> None:
        with self._state_lock:
            self._fault = message
        self.trace.write('coordination_fault', request_id, error=message)
