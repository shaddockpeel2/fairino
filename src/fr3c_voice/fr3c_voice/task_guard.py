"""Pure state guard preventing duplicate GUI action dispatch."""

from __future__ import annotations

from dataclasses import dataclass
import threading


@dataclass(frozen=True)
class TaskSnapshot:
    request_id: str = ''
    task: str = ''
    color: str = ''
    zone: str = ''
    support_color: str = ''
    phase: str = 'idle'
    detail: str = ''

    @property
    def active(self) -> bool:
        return self.phase not in {'idle', 'done', 'failed', 'canceled'}


class TaskDispatchGuard:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot = TaskSnapshot()

    def snapshot(self) -> TaskSnapshot:
        with self._lock:
            return self._snapshot

    def begin(self, request_id: str, task: str, color: str,
              zone: str = '', support_color: str = '') -> TaskSnapshot:
        with self._lock:
            if self._snapshot.active:
                raise RuntimeError('another grasp request is active')
            self._snapshot = TaskSnapshot(
                request_id=request_id, task=task, color=color, zone=zone,
                support_color=support_color,
                phase='submitting', detail='waiting for Action goal response')
            return self._snapshot

    def update(self, request_id: str, phase: str, detail: str = '') -> TaskSnapshot:
        with self._lock:
            if request_id != self._snapshot.request_id:
                return self._snapshot
            self._snapshot = TaskSnapshot(
                request_id=request_id, task=self._snapshot.task,
                color=self._snapshot.color, zone=self._snapshot.zone,
                support_color=self._snapshot.support_color,
                phase=phase, detail=detail)
            return self._snapshot

    def finish(self, request_id: str, success: bool, canceled: bool,
               detail: str) -> TaskSnapshot:
        phase = 'canceled' if canceled else 'done' if success else 'failed'
        return self.update(request_id, phase, detail)
