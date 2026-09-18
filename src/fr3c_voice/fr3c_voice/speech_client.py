"""Client for the phase-6 streaming speech service.

The GUI uses this to turn an answer into sound while the answer is still being
generated. The contract is small on purpose:

* :meth:`SpeechClient.begin` opens a session and cuts off whatever was still
  playing, which is what a user expects when they ask something new
* :meth:`SpeechClient.feed` forwards a token as it arrives from the model
* :meth:`SpeechClient.finish` blocks until the audio has actually been played
* :meth:`SpeechClient.stop` silences the device immediately

Every call opens its own short-lived connection, matching the JSONL protocol of
the service; the service keeps the session state.
"""
from __future__ import annotations

import json
from pathlib import Path
import socket
import threading
import time
import uuid


class SpeechServiceError(RuntimeError):
    pass


class SpeechClient:
    def __init__(self, socket_path: Path | str, timeout_s: float = 30.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = float(timeout_s)

    # -- plumbing --------------------------------------------------------- #

    def _call(self, payload: dict, timeout_s: float | None = None) -> list[dict]:
        timeout = self.timeout_s if timeout_s is None else timeout_s
        records: list[dict] = []
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(timeout)
                connection.connect(str(self.socket_path))
                connection.sendall(
                    json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
                with connection.makefile('r', encoding='utf-8') as reader:
                    for line in reader:
                        value = json.loads(line)
                        records.append(value)
                        if value.get('event') in ('result', 'error', 'stopped'):
                            break
        except (OSError, json.JSONDecodeError) as error:
            raise SpeechServiceError(f'speech service request failed: {error}') from error
        errors = [item for item in records if item.get('event') == 'error']
        if errors:
            raise SpeechServiceError(str(errors[-1].get('error', 'unknown error')))
        return records

    def available(self) -> bool:
        """True when the speech service answers; used to grey out the controls."""
        if not self.socket_path.exists():
            return False
        try:
            records = self._call({'op': 'health'}, timeout_s=5.0)
        except SpeechServiceError:
            return False
        return bool(records and records[0].get('healthy'))

    def health(self) -> dict:
        records = self._call({'op': 'health'}, timeout_s=5.0)
        return records[0] if records else {}

    def preload(self, texts: list[str]) -> list[str]:
        records = self._call({'op': 'preload', 'texts': texts})
        return list(records[0].get('added') or []) if records else []

    # -- speaking --------------------------------------------------------- #

    def begin(self, replace: bool = True, request_id: str = '') -> str:
        identifier = request_id or uuid.uuid4().hex
        self._call({'op': 'speak_start', 'request_id': identifier, 'replace': replace})
        return identifier

    def feed(self, request_id: str, text: str) -> None:
        if not text:
            return
        self._call({'op': 'speak_text', 'request_id': request_id, 'text': text})

    def finish(self, request_id: str, timeout_s: float = 600.0) -> dict:
        records = self._call(
            {'op': 'speak_end', 'request_id': request_id, 'timeout_s': timeout_s},
            timeout_s=timeout_s + 60.0)
        return next((record for record in reversed(records)
                     if record.get('event') == 'result'), {})

    def stop(self, reason: str = 'user') -> dict:
        records = self._call({'op': 'stop', 'reason': reason}, timeout_s=10.0)
        return records[0] if records else {}

    def status(self) -> dict:
        records = self._call({'op': 'status'}, timeout_s=10.0)
        return records[0] if records else {}

    def speak_text(self, text: str) -> dict:
        """Speak a complete string in one go; used for fixed replies."""
        request_id = self.begin()
        for index in range(0, len(text), 24):
            self.feed(request_id, text[index:index + 24])
        return self.finish(request_id)


class StreamingSpeaker:
    """Feed tokens in from a model thread without blocking the UI.

    The model callback runs on whichever thread the model client uses, so the
    speech calls happen there. Errors are kept rather than raised, because a
    speech failure must never take the dialogue down with it.
    """

    def __init__(self, client: SpeechClient, enabled: bool = False) -> None:
        self.client = client
        self.enabled = bool(enabled)
        self._lock = threading.Lock()
        self._request_id = ''
        self._error = ''
        self._started_at = 0.0
        self._last_result: dict = {}

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._request_id)

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._error

    @property
    def last_result(self) -> dict:
        with self._lock:
            return dict(self._last_result)

    def start(self, request_id: str = '') -> bool:
        """Open a session if speech is enabled and the service is up."""
        if not self.enabled:
            return False
        with self._lock:
            active_request = self._request_id
            if active_request and (not request_id or request_id == active_request):
                return True
        try:
            identifier = self.client.begin(replace=True, request_id=request_id)
        except SpeechServiceError as error:
            with self._lock:
                self._error = str(error)
            return False
        with self._lock:
            self._request_id = identifier
            self._error = ''
            self._started_at = time.monotonic()
        return True

    def feed(self, token: str) -> None:
        with self._lock:
            request_id = self._request_id
        if not request_id or not token:
            return
        try:
            self.client.feed(request_id, token)
        except SpeechServiceError as error:
            with self._lock:
                self._error = str(error)
                self._request_id = ''

    def finish(self, timeout_s: float = 600.0) -> dict:
        with self._lock:
            request_id = self._request_id
            self._request_id = ''
        if not request_id:
            return {}
        try:
            result = self.client.finish(request_id, timeout_s=timeout_s)
        except SpeechServiceError as error:
            with self._lock:
                self._error = str(error)
            return {}
        with self._lock:
            self._last_result = result
        return result

    def stop(self, reason: str = 'user') -> dict:
        with self._lock:
            self._request_id = ''
        try:
            return self.client.stop(reason)
        except SpeechServiceError as error:
            with self._lock:
                self._error = str(error)
            return {}
