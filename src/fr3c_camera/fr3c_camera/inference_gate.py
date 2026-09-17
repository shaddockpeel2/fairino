"""Thread-safe pause-and-drain gate for perception processing."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class InferenceGateSnapshot:
    enabled: bool
    active_inferences: int
    pause_generation: int
    frames_skipped_paused: int
    last_inference_start_ns: int | None
    last_inference_end_ns: int | None
    last_success_ns: int | None
    closed: bool


class InferenceGate:
    """Prevent new processing from entering and drain work already in flight."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._enabled = True
        self._active = 0
        self._pause_generation = 0
        self._frames_skipped_paused = 0
        self._last_inference_start_ns: int | None = None
        self._last_inference_end_ns: int | None = None
        self._last_success_ns: int | None = None
        self._closed = False

    def enter(self) -> bool:
        with self._condition:
            if not self._enabled or self._closed:
                self._frames_skipped_paused += 1
                return False
            self._active += 1
            self._last_inference_start_ns = time.monotonic_ns()
            return True

    def leave(self, success: bool) -> None:
        with self._condition:
            if self._active <= 0:
                raise RuntimeError('inference gate leave without enter')
            now_ns = time.monotonic_ns()
            self._active -= 1
            self._last_inference_end_ns = now_ns
            if success:
                self._last_success_ns = now_ns
            self._condition.notify_all()

    def set_enabled(self, enabled: bool, timeout_s: float) -> InferenceGateSnapshot:
        if timeout_s <= 0:
            raise ValueError('timeout_s must be positive')
        deadline = time.monotonic() + timeout_s
        with self._condition:
            if self._closed:
                raise RuntimeError('inference gate is closed')
            requested = bool(enabled)
            if requested != self._enabled:
                self._enabled = requested
                if not requested:
                    self._pause_generation += 1
                self._condition.notify_all()
            if not requested:
                while self._active:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('active inference did not drain')
                    self._condition.wait(remaining)
            return self._snapshot_locked()

    def snapshot(self) -> InferenceGateSnapshot:
        with self._condition:
            return self._snapshot_locked()

    def close(self, timeout_s: float = 5.0) -> InferenceGateSnapshot:
        if timeout_s <= 0:
            raise ValueError('timeout_s must be positive')
        deadline = time.monotonic() + timeout_s
        with self._condition:
            self._enabled = False
            self._closed = True
            self._condition.notify_all()
            while self._active:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('active inference did not drain during close')
                self._condition.wait(remaining)
            return self._snapshot_locked()

    def _snapshot_locked(self) -> InferenceGateSnapshot:
        return InferenceGateSnapshot(
            enabled=self._enabled,
            active_inferences=self._active,
            pause_generation=self._pause_generation,
            frames_skipped_paused=self._frames_skipped_paused,
            last_inference_start_ns=self._last_inference_start_ns,
            last_inference_end_ns=self._last_inference_end_ns,
            last_success_ns=self._last_success_ns,
            closed=self._closed,
        )
