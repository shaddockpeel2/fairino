"""Bounded length-framed protocol for the native Qwen worker."""

from __future__ import annotations

import socket
import struct
from pathlib import Path
import threading
import time
from typing import Iterator, Optional


OP_CHAT = 1
OP_RESET = 2
OP_PING = 3
EVENT_TOKEN = 1
EVENT_DONE = 2
EVENT_ERROR = 3
EVENT_RESET = 4
EVENT_PONG = 5
MAX_FRAME_BYTES = 16 * 1024 * 1024
MAX_PATH_BYTES = 4096


class WorkerUnavailable(RuntimeError):
    pass


class WorkerProtocolError(RuntimeError):
    pass


class RequestCancelled(WorkerUnavailable):
    pass


class RequestTimedOut(WorkerUnavailable):
    pass


class WorkerClient:
    def __init__(self, socket_path: Path | str, timeout_s: float,
                 max_prompt_bytes: int) -> None:
        self.socket_path = str(socket_path)
        self.timeout_s = float(timeout_s)
        self.max_prompt_bytes = max_prompt_bytes

    def _connect(self, deadline: float) -> socket.socket:
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            connection.settimeout(max(0.001, deadline - time.monotonic()))
            connection.connect(self.socket_path)
            return connection
        except OSError as error:
            connection.close()
            raise WorkerUnavailable('Qwen worker is not available') from error

    @staticmethod
    def _check(deadline: float, cancel_event: threading.Event | None) -> None:
        if cancel_event is not None and cancel_event.is_set():
            raise RequestCancelled('Qwen request was cancelled')
        if time.monotonic() >= deadline:
            raise RequestTimedOut('Qwen request exceeded its total timeout')

    @classmethod
    def _read_exact(cls, connection: socket.socket, size: int, deadline: float,
                    cancel_event: threading.Event | None) -> Optional[bytes]:
        chunks = []
        while size:
            cls._check(deadline, cancel_event)
            connection.settimeout(min(0.1, max(0.001, deadline - time.monotonic())))
            try:
                chunk = connection.recv(size)
            except socket.timeout:
                continue
            except OSError as error:
                raise WorkerUnavailable('Qwen worker connection failed') from error
            if not chunk:
                return None
            chunks.append(chunk)
            size -= len(chunk)
        return b''.join(chunks)

    @staticmethod
    def _send_frame(connection: socket.socket, payload: bytes) -> None:
        if not payload or len(payload) > MAX_FRAME_BYTES:
            raise WorkerProtocolError('invalid worker request frame size')
        connection.sendall(struct.pack('!I', len(payload)) + payload)

    @classmethod
    def _read_frame(cls, connection: socket.socket, deadline: float,
                    cancel_event: threading.Event | None = None
                    ) -> Optional[tuple[int, bytes]]:
        header = cls._read_exact(connection, 4, deadline, cancel_event)
        if header is None:
            return None
        (size,) = struct.unpack('!I', header)
        if size < 1 or size > MAX_FRAME_BYTES:
            raise WorkerProtocolError('worker returned an invalid frame size')
        payload = cls._read_exact(connection, size, deadline, cancel_event)
        if payload is None:
            raise WorkerProtocolError('worker disconnected during a response')
        return payload[0], payload[1:]

    def _simple_request(self, operation: int, expected: int) -> bytes:
        deadline = time.monotonic() + self.timeout_s
        with self._connect(deadline) as connection:
            self._send_frame(connection, bytes((operation,)))
            frame = self._read_frame(connection, deadline)
        if frame is None:
            raise WorkerUnavailable('worker disconnected without a response')
        event, body = frame
        if event == EVENT_ERROR:
            raise WorkerProtocolError(body.decode('utf-8', errors='replace'))
        if event != expected:
            raise WorkerProtocolError(f'unexpected worker event: {event}')
        return body

    def ping(self) -> bool:
        try:
            self._simple_request(OP_PING, EVENT_PONG)
            return True
        except (OSError, WorkerUnavailable, WorkerProtocolError):
            return False

    def reset(self) -> None:
        self._simple_request(OP_RESET, EVENT_RESET)

    def chat(self, prompt: str, image_path: Path | str | None = None,
             cancel_event: threading.Event | None = None
             ) -> Iterator[tuple[int, bytes]]:
        prompt_bytes = prompt.encode('utf-8')
        path_bytes = str(image_path or '').encode('utf-8')
        if not prompt_bytes:
            raise WorkerProtocolError('prompt must not be empty')
        if len(prompt_bytes) > self.max_prompt_bytes:
            raise WorkerProtocolError('prompt exceeds the configured byte limit')
        if len(path_bytes) > MAX_PATH_BYTES:
            raise WorkerProtocolError('image path is too long')
        payload = (
            bytes((OP_CHAT,))
            + struct.pack('!II', len(path_bytes), len(prompt_bytes))
            + path_bytes + prompt_bytes
        )
        deadline = time.monotonic() + self.timeout_s
        with self._connect(deadline) as connection:
            self._send_frame(connection, payload)
            while True:
                frame = self._read_frame(connection, deadline, cancel_event)
                if frame is None:
                    raise WorkerUnavailable('worker disconnected before completion')
                event, body = frame
                if event == EVENT_ERROR:
                    raise WorkerProtocolError(body.decode('utf-8', errors='replace'))
                if event not in (EVENT_TOKEN, EVENT_DONE):
                    raise WorkerProtocolError(f'unexpected worker event: {event}')
                yield event, body
                if event == EVENT_DONE:
                    return
