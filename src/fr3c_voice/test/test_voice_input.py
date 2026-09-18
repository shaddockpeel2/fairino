"""Exercise subprocess cleanup, final-once semantics and GUI routing without ROS."""
import json
from pathlib import Path
import queue
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from fr3c_voice.voice_input import VoiceInput
from fr3c_voice.dialogue_gui import DialogueGui

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setattr('fr3c_voice.voice_input.CAPTURE_LOCK_PATH', str(tmp_path / 'capture.lock'))
    (tmp_path / 'config/voice').mkdir(parents=True)
    cfg = json.loads((ROOT / 'config/voice/asr.json').read_text())
    cfg.update(initial_silence_s=0.12, maximum_record_s=0.4, minimum_record_s=0.01,
               minimum_speech_s=0.01, trailing_silence_s=0.06)
    (tmp_path / 'config/voice/asr.json').write_text(json.dumps(cfg))
    worker = tmp_path / 'worker.py'
    worker.write_text('''import sys,json
print(json.dumps({'event':'ready'}),flush=True)
print(json.dumps({'event':'partial','text':'抓红'}),flush=True)
data=sys.stdin.buffer.read()
print(json.dumps({'event':'final','text':'抓取红色方块'}),flush=True)
''')
    capture = tmp_path / 'capture.py'
    capture.write_text('''import sys,time
while True:
 sys.stdout.buffer.write(b'\\x00\\x10'*800);sys.stdout.buffer.flush();time.sleep(.01)
''')
    original = subprocess.Popen
    processes = []
    def popen(command, **kwargs):
        actual = [sys.executable, str(capture if command[0] == 'arecord' else worker)]
        proc = original(actual, **kwargs)
        processes.append(proc)
        return proc
    monkeypatch.setattr('fr3c_voice.voice_input.subprocess.Popen', popen)
    events = queue.Queue()
    voice = VoiceInput(tmp_path, events.put)
    monkeypatch.setattr(voice, 'validate', lambda: None)
    yield voice, events, processes, worker, capture
    voice.close()


def terminal(events):
    deadline = time.monotonic() + 5
    received = []
    while time.monotonic() < deadline:
        event = events.get(timeout=3)
        received.append(event)
        if event[0] in ('asr_final', 'asr_empty', 'asr_error'):
            return event, received
    raise AssertionError('no final')


def test_thirty_rounds_release_children_and_unique_final(rig):
    voice, events, processes, _, _ = rig
    ids = set()
    for _ in range(30):
        rid = voice.start()
        assert rid and rid not in ids
        ids.add(rid)
        assert voice.start() == ''
        result, received = terminal(events)
        voice._thread.join(1)
        assert result[0] == 'asr_final', result
        assert all(e[1] == rid for e in received)
        assert all(p.poll() is not None for p in processes)
        assert events.empty()


def test_empty_capture_never_submits_hallucinated_text(rig):
    voice, events, _, _, capture = rig
    capture.write_text("import sys,time\nwhile True:\n sys.stdout.buffer.write(bytes(1600));sys.stdout.buffer.flush();time.sleep(.01)\n")
    voice.start()
    result, _ = terminal(events)
    assert result[0] == 'asr_empty'


def test_manual_finish_and_cancel_release_capture(rig):
    voice, events, processes, _, _ = rig
    voice.start()
    while events.get(timeout=3)[0] != 'asr_recording':
        pass
    time.sleep(.06)
    voice.finish()
    result, _ = terminal(events)
    assert result[0] == 'asr_final'
    assert result[2]['reason'] == 'button'
    voice._thread.join(1)
    voice.start()
    voice.close()
    assert all(p.poll() is not None for p in processes)
    assert events.empty()


def test_stop_failure_prevents_microphone_open(rig):
    voice, events, processes, _, _ = rig
    def fail():
        raise RuntimeError('speaker stop failed')
    voice.before_capture = fail
    voice.start()
    result, _ = terminal(events)
    assert result[0] == 'asr_error'
    assert not processes


def test_worker_crash_releases_device_and_next_round_recovers(rig):
    voice, events, processes, worker, _ = rig
    source = worker.read_text()
    worker.write_text("import sys\nsys.exit(1)\n")
    voice.start()
    result, _ = terminal(events)
    assert result[0] == 'asr_error'
    voice._thread.join(1)
    assert all(p.poll() is not None for p in processes)
    worker.write_text(source)
    voice.start()
    result, _ = terminal(events)
    assert result[0] == 'asr_final'


def test_gui_replaces_partial_and_submits_final_only_once():
    values, submissions = [], []
    gui = SimpleNamespace(
        _voice_request='round1', input_var=SimpleNamespace(set=values.append),
        voice_button=SimpleNamespace(configure=lambda **kw: None),
        _set_busy=lambda busy: None, send_text=lambda text, **kw: submissions.append((text, kw)),
        _append=lambda *args: None)
    for text in ('抓红', '抓取红色', '抓取红色方块'):
        DialogueGui._handle_voice_event(gui, 'asr_partial', 'round1', {'text': text})
    assert values == ['抓红', '抓取红色', '抓取红色方块']
    DialogueGui._handle_voice_event(gui, 'asr_final', 'round1', {'text': '抓取红色方块'})
    DialogueGui._handle_voice_event(gui, 'asr_final', 'round1', {'text': '抓取红色方块'})
    DialogueGui._handle_voice_event(gui, 'asr_partial', 'old', {'text': '旧内容'})
    assert submissions == [('抓取红色方块', {'request_id': 'round1'})]
    assert values[-1] == '抓取红色方块'


def test_busy_robot_cannot_start_second_voice_task():
    messages = []
    gui = SimpleNamespace(
        _voice_request='', voice=SimpleNamespace(start=lambda: pytest.fail('started recording')),
        _dialogue_busy=False,
        bridge=SimpleNamespace(state_snapshot=lambda: ({'action_active': True}, 0),
                               guard=SimpleNamespace(snapshot=lambda: SimpleNamespace(active=False))),
        _append=lambda *args: messages.append(args))
    DialogueGui.toggle_voice(gui)
    assert messages


def test_unacknowledged_stop_is_not_silently_ignored():
    import threading
    from fr3c_voice.speech_client import SpeechServiceError
    def fail(reason):
        raise SpeechServiceError('timeout')
    gui = SimpleNamespace(_speech_start_lock=threading.Lock(),
                          speech=SimpleNamespace(client=SimpleNamespace(
                              socket_path=SimpleNamespace(exists=lambda: True), stop=fail)))
    with pytest.raises(SpeechServiceError):
        DialogueGui._stop_before_capture(gui)


def test_native_endpoint_closes_input_before_final_arrives(rig):
    voice, events, processes, worker, _ = rig
    worker.write_text('''import os,time,json
print(json.dumps({'event':'ready'}),flush=True)
os.close(0)
time.sleep(.15)
print(json.dumps({'event':'final','text':'抓起红色方块'}),flush=True)
''')
    voice.start()
    result, _ = terminal(events)
    assert result[0] == 'asr_final', result
    assert result[2]['reason'] == 'sherpa_endpoint'
    assert all(p.poll() is not None for p in processes)
