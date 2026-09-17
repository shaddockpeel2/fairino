from pathlib import Path

import pytest

import fr3c_voice.controller as controller_module
from fr3c_voice.config import QwenConfig
from fr3c_voice.controller import ControllerError, ControllerState, QwenController
from fr3c_voice.protocol import EVENT_DONE, EVENT_TOKEN, WorkerProtocolError


class FakeProcess:
    pid = 4242
    returncode = None
    def poll(self): return self.returncode


class FakeClient:
    fail = False
    def __init__(self, *_args, **_kwargs): pass
    def ping(self): return True
    def reset(self): return None
    def chat(self, _prompt, _image=None, cancel_event=None):
        if self.fail:
            raise WorkerProtocolError('simulated disconnect')
        yield EVENT_TOKEN, '本地'.encode()
        yield EVENT_DONE, b''


@pytest.fixture
def config(tmp_path):
    assets = []
    for name in ('worker', 'vision', 'language', 'rkllm.so', 'rknn.so'):
        path = tmp_path / name
        path.write_bytes(b'asset')
        assets.append(path)
    return QwenConfig(
        asset_root=tmp_path, model_id='qwen', vision_path=assets[1],
        language_path=assets[2], worker_path=assets[0],
        rkllm_runtime_path=assets[3], rknn_runtime_path=assets[4],
        socket_name='worker.sock', max_new_tokens=16, max_context_len=128,
        npu_core_num=1, startup_timeout_s=1, request_timeout_s=1,
        cancel_grace_s=0.1, terminate_grace_s=0.1,
        max_prompt_bytes=32, max_image_bytes=64)


def ready_controller(monkeypatch, config, tmp_path):
    monkeypatch.setattr(controller_module, 'WorkerClient', FakeClient)
    value = QwenController(config, tmp_path / 'run')
    value.runtime_dir.mkdir(parents=True)
    value.image_dir.mkdir(parents=True)
    value._process = FakeProcess()
    value._state = ControllerState.READY
    return value


def test_text_and_managed_image(monkeypatch, config, tmp_path):
    value = ready_controller(monkeypatch, config, tmp_path)
    assert value.chat_text('你好', 'req-1').text == '本地'
    image = tmp_path / 'input.jpg'
    image.write_bytes(b'\xff\xd8\xff' + b'x' * 10)
    assert value.chat_image('描述', image).text == '本地'
    assert list(value.image_dir.iterdir()) == []


def test_invalid_inputs_and_busy(monkeypatch, config, tmp_path):
    value = ready_controller(monkeypatch, config, tmp_path)
    corrupt = tmp_path / 'bad.jpg'
    corrupt.write_bytes(b'not an image')
    with pytest.raises(ControllerError, match='invalid or unsupported'):
        value.chat_image('描述', corrupt)
    with pytest.raises(ControllerError, match='byte limit'):
        value.chat_text('中' * 20)
    assert value._request_lock.acquire(blocking=False)
    try:
        with pytest.raises(ControllerError, match='already running'):
            value.chat_text('并发')
    finally:
        value._request_lock.release()


def test_worker_failure_enters_fault_without_silent_restart(
        monkeypatch, config, tmp_path):
    value = ready_controller(monkeypatch, config, tmp_path)
    FakeClient.fail = True
    monkeypatch.setattr(
        value, '_terminate_locked', lambda: setattr(value, '_process', None))
    try:
        with pytest.raises(ControllerError, match='simulated disconnect'):
            value.chat_text('失败')
        assert value.state == ControllerState.FAULT
        assert value.worker_pid is None
    finally:
        FakeClient.fail = False
