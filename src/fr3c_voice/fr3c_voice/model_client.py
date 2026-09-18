"""Small non-printing client for the stage-3 local model service."""

from __future__ import annotations

import json
from pathlib import Path
import socket
import threading
import uuid
from typing import Callable


class ModelServiceError(RuntimeError):
    pass


class ModelServiceClient:
    def __init__(self, socket_path: Path | str, timeout_s: float = 660.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = timeout_s

    def _request(self, payload: dict,
                 on_event: Callable[[dict], None] | None = None) -> list[dict]:
        records = []
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(self.timeout_s)
            try:
                connection.connect(str(self.socket_path))
                connection.sendall(
                    json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
                with connection.makefile('r', encoding='utf-8') as reader:
                    for line in reader:
                        value = json.loads(line)
                        records.append(value)
                        if on_event is not None:
                            on_event(value)
            except (OSError, json.JSONDecodeError) as error:
                raise ModelServiceError(f'model service request failed: {error}') from error
        errors = [item for item in records if item.get('event') == 'error']
        if errors:
            raise ModelServiceError(str(errors[-1].get('error', 'unknown error')))
        return records

    def health(self) -> dict:
        records = self._request({'op': 'health'})
        if not records or records[-1].get('event') != 'health':
            raise ModelServiceError('model service returned no health record')
        return records[-1]

    def generate(self, prompt: str, request_id: str = '',
                 image_path: Path | str | None = None,
                 on_token: Callable[[str], None] | None = None,
                 knowledge_question: str = '',
                 on_knowledge: Callable[[dict], None] | None = None) -> str:
        """Run one generation, optionally forwarding tokens as they arrive.

        `on_token` is what lets speech start before the answer is finished; it
        is called on the socket-reading thread.
        """
        identifier = request_id or uuid.uuid4().hex
        payload = {
            'op': ('chat_image' if image_path is not None else
                   'chat_auto' if knowledge_question else 'chat_text'),
            'request_id': identifier,
            'prompt': prompt, 'reset_before': True,
        }
        if knowledge_question:
            payload['question'] = knowledge_question
        if image_path is not None:
            payload['image_path'] = str(Path(image_path).resolve())

        def forward(record: dict) -> None:
            if on_token is not None and record.get('event') == 'token':
                text = record.get('text')
                if isinstance(text, str) and text:
                    on_token(text)

        records = self._request(payload, on_event=forward if on_token else None)
        results = [item for item in records if item.get('event') == 'result']
        if len(results) != 1 or not isinstance(results[0].get('text'), str):
            raise ModelServiceError('model service returned no unique result')
        metadata = results[0].get('knowledge')
        if on_knowledge is not None and isinstance(metadata, dict):
            on_knowledge(metadata)
        return results[0]['text']

    def cancel_model(self, request_id: str = '') -> bool:
        records = self._request({'op': 'cancel', 'request_id': request_id})
        return bool(records and records[-1].get('cancelled'))


class CancellableGeneration:
    """Track the one classifier request owned by the GUI."""

    def __init__(self, client: ModelServiceClient) -> None:
        self.client = client
        self._lock = threading.Lock()
        self._request_id = ''

    @property
    def request_id(self) -> str:
        with self._lock:
            return self._request_id

    def generate(self, prompt: str, request_id: str = '',
                 image_path: Path | str | None = None,
                 on_token: Callable[[str], None] | None = None,
                 knowledge_question: str = '',
                 on_knowledge: Callable[[dict], None] | None = None) -> str:
        identifier = request_id or uuid.uuid4().hex
        with self._lock:
            if self._request_id:
                raise ModelServiceError('another dialogue request is active')
            self._request_id = identifier
        try:
            return self.client.generate(
                prompt, identifier, image_path=image_path, on_token=on_token,
                knowledge_question=knowledge_question,
                on_knowledge=on_knowledge)
        finally:
            with self._lock:
                self._request_id = ''

    def cancel(self) -> bool:
        identifier = self.request_id
        return bool(identifier) and self.client.cancel_model(identifier)
