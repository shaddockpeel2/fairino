"""P1-compatible monotonic JSONL trace writer."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
import uuid
from typing import Any


class TraceWriter:
    def __init__(self, path: Path | str | None, run_id: str | None = None) -> None:
        self.path = Path(path) if path is not None else None
        self.run_id = run_id or uuid.uuid4().hex
        self._lock = threading.Lock()
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, request_id: str = '', **details: Any) -> None:
        if self.path is None:
            return
        record = {
            'schema_version': 1,
            'run_id': self.run_id,
            'request_id': request_id,
            'event': event,
            'clock': 'CLOCK_MONOTONIC',
            'monotonic_ns': time.monotonic_ns(),
            'wall_time_ns': time.time_ns(),
            'pid': os.getpid(),
            **details,
        }
        line = json.dumps(record, ensure_ascii=False, separators=(',', ':'))
        with self._lock:
            with self.path.open('a', encoding='utf-8') as handle:
                handle.write(line + '\n')
