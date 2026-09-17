import socket
import struct
import threading
import time

import pytest

from fr3c_voice.protocol import (
    EVENT_DONE, EVENT_PONG, EVENT_RESET, EVENT_TOKEN, MAX_FRAME_BYTES,
    OP_CHAT, OP_PING, OP_RESET, RequestCancelled, WorkerClient,
    RequestTimedOut, WorkerProtocolError,
)


def frame(event, body=b''):
    payload = bytes((event,)) + body
    return struct.pack('!I', len(payload)) + payload


class FakeWorker:
    def __init__(self, malformed=False):
        self.malformed = malformed
        self.requests = []

    def connect(self, _deadline):
        worker = self

        class Connection:
            def __init__(self):
                self.response = b''
            def __enter__(self): return self
            def __exit__(self, *_args): return None
            def settimeout(self, _timeout): pass
            def sendall(self, request):
                size = struct.unpack('!I', request[:4])[0]
                payload = request[4:4 + size]
                worker.requests.append(payload)
                if worker.malformed:
                    self.response = struct.pack('!I', MAX_FRAME_BYTES + 1)
                elif payload[0] == OP_PING:
                    self.response = frame(EVENT_PONG, b'ready')
                elif payload[0] == OP_RESET:
                    self.response = frame(EVENT_RESET, b'ok')
                elif payload[0] == OP_CHAT:
                    self.response = frame(EVENT_TOKEN, '你好'.encode()) + frame(EVENT_DONE)
            def recv(self, size):
                chunk, self.response = self.response[:size], self.response[size:]
                return chunk

        return Connection()


def test_bounded_worker_protocol(tmp_path, monkeypatch):
    worker = FakeWorker()
    client = WorkerClient(tmp_path / 'worker.sock', 1, 128)
    monkeypatch.setattr(client, '_connect', worker.connect)
    assert client.ping()
    client.reset()
    assert list(client.chat('测试')) == [
        (EVENT_TOKEN, '你好'.encode()), (EVENT_DONE, b'')]
    assert [item[0] for item in worker.requests] == [OP_PING, OP_RESET, OP_CHAT]


def test_invalid_response_and_prompt_limit(tmp_path, monkeypatch):
    client = WorkerClient(tmp_path / 'worker.sock', 1, 5)
    with pytest.raises(WorkerProtocolError, match='byte limit'):
        list(client.chat('三个字'))
    worker = FakeWorker(malformed=True)
    monkeypatch.setattr(client, '_connect', worker.connect)
    with pytest.raises(WorkerProtocolError, match='frame size'):
        client.reset()


def test_cancel_is_checked_while_waiting(tmp_path, monkeypatch):
    left, right = socket.socketpair()
    client = WorkerClient(tmp_path / 'unused', 2, 128)
    monkeypatch.setattr(client, '_connect', lambda _deadline: left)
    cancel = threading.Event()
    timer = threading.Timer(0.05, cancel.set)
    timer.start()
    try:
        with pytest.raises(RequestCancelled):
            list(client.chat('取消', cancel_event=cancel))
    finally:
        timer.cancel()
        right.close()


def test_total_request_deadline_is_enforced(tmp_path, monkeypatch):
    left, right = socket.socketpair()
    client = WorkerClient(tmp_path / 'unused', 0.05, 128)
    monkeypatch.setattr(client, '_connect', lambda _deadline: left)
    try:
        with pytest.raises(RequestTimedOut):
            list(client.chat('超时'))
    finally:
        right.close()
