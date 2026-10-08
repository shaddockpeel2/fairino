"""Audio-free tests for routing, ownership, delayed ACKs and tab lifecycle."""
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from fr3c_voice.dialogue_gui import DialogueGui
from fr3c_voice.edge_speech_page import EdgeSpeechPage, _active_speech_request
from fr3c_voice.speech_client import SpeechServiceError
from fr3c_voice.speech_resource_coordinator import SpeechResourceCoordinator


class Widget:
    def __init__(self, *args, **kwargs):
        self.value = ''
        self.options = {}
        self.clipboard = ''

    def configure(self, **options):
        self.options.update(options)

    def get(self, *args):
        return self.value

    def set(self, value):
        self.value = value

    def insert(self, _where, value):
        self.value = value

    def delete(self, *args):
        self.value = ''

    def after(self, *args):
        return 'timer'

    def after_cancel(self, *args):
        pass

    def clipboard_clear(self):
        self.clipboard = ''

    def clipboard_append(self, value):
        self.clipboard += value


class Notebook:
    def __init__(self):
        self.selected = 'original'

    def add(self, page, **kwargs):
        self.page = page

    def select(self, page=None):
        if page is not None:
            self.selected = str(page)
        return self.selected

    def bind(self, *args, **kwargs):
        return 'binding'

    def unbind(self, *args):
        pass


class Voice:
    def __init__(self, _workspace, emit, before_capture):
        self.emit = emit
        self.before_capture = before_capture
        self._thread = None
        self.canceled = False
        self.release = threading.Event()
        self.recording = threading.Event()

    def validate(self):
        pass

    def start(self):
        self.release.clear()
        self.recording.clear()
        self.canceled = False
        rid = 'capture-' + str(time.monotonic_ns())

        def worker():
            try:
                self.before_capture()
                self.emit(('asr_recording', rid, {}))
                self.recording.set()
                self.release.wait(3)
                if not self.canceled:
                    self.emit(('asr_final', rid, {'text': '抓取红色方块',
                                                'raw_text': '转起红色方块', 'audio_s': 1.2}))
            except Exception as error:
                self.recording.set()
                self.emit(('asr_error', rid, {'message': str(error)}))

        self._thread = threading.Thread(target=worker, daemon=True)
        self._thread.start()
        return rid

    def finish(self):
        self.release.set()

    def close(self):
        # Deliberately do not release: prove a bounded join isn't treated as
        # completed cleanup while the microphone thread is still alive.
        self.canceled = True


class Client:
    def __init__(self, socket_path):
        self.socket_path = socket_path
        self.active = ''
        self.begins = []
        self.stops = []
        self.text = ''
        self.finish_entered = threading.Event()
        self.release = threading.Event()
        self.status_failed = False
        self.begin_gate = None
        self.begin_entered = threading.Event()
        self.lose_ack = False
        self.text_end_s = None

    def health(self):
        return {'healthy': True, 'candidate': 'existing-backend'}

    def status(self):
        if self.status_failed:
            raise SpeechServiceError('status timeout')
        return {'session_running': bool(self.active),
                'last_metrics': {'request_id': self.active, 'text_end_s': self.text_end_s}}

    def begin(self, replace, request_id):
        self.begins.append((replace, request_id))
        self.begin_entered.set()
        if self.begin_gate:
            assert self.begin_gate.wait(3)
        if self.active:
            raise SpeechServiceError('another session is running')
        self.release.clear()
        self.text_end_s = None
        self.active = request_id
        if self.lose_ack:
            raise SpeechServiceError('lost start ACK')
        return request_id

    def feed(self, rid, text):
        assert rid == self.active
        self.text += text

    def finish(self, rid, timeout_s):
        self.text_end_s = 0.1
        self.finish_entered.set()
        assert self.release.wait(3)
        if self.active == rid:
            self.active = ''
        return {}

    def stop(self, reason):
        self.stops.append(self.active)
        self.active = ''
        self.release.set()
        return {'stopped': True}


@pytest.fixture
def rig(tmp_path, monkeypatch):
    from tkinter import ttk

    monkeypatch.setattr(ttk, 'Frame', Widget)

    def build(page):
        for name in ('device_status', 'resource_status', 'asr_text', 'tts_text',
                     'asr_status', 'tts_status', 'record_button', 'finish_button',
                     'cancel_button', 'copy_button', 'fill_button', 'speak_button', 'stop_button'):
            setattr(page, name, Widget())

    monkeypatch.setattr(EdgeSpeechPage, '_build', build)
    state = SimpleNamespace(busy=False, wake_active=False, pauses=0, resumes=0)

    def pause():
        state.pauses += 1

    def resume():
        state.resumes += 1

    resources = SpeechResourceCoordinator(lambda: state.busy, pause,
                                          lambda: state.wake_active, resume)
    notebook = Notebook()
    client = Client(tmp_path / 'tts.sock')
    page = EdgeSpeechPage(notebook, tmp_path, resources, client.socket_path,
                          voice_factory=Voice, client=client)
    page._next_health = float('inf')
    page._tts_ready = True
    yield page, resources, state, client
    state.busy = False
    client.status_failed = False
    if client.begin_gate:
        client.begin_gate.set()
    client.release.set()
    page.voice.release.set()
    page.close()


def enter(page):
    page.notebook.select(page.page)
    page._tab_changed()


def settle(page, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        page._tick()
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('operation did not settle')


def test_entering_busy_page_preserves_original_task_and_wake(rig):
    page, resources, state, client = rig
    state.busy = True
    enter(page)
    page.start_recording()
    page.tts_text.value = '测试'
    page.start_speaking()
    assert not resources.reserved and not page.audio_active
    assert state.pauses == 0 and not client.begins
    state.busy = False
    page._tick()
    assert resources.ready and state.pauses == 1


def test_waits_for_wake_exit_even_when_immediately_leaving(rig):
    page, resources, state, _ = rig
    state.wake_active = True
    enter(page)
    assert resources.reserved and not resources.ready
    page.notebook.select('original')
    page._tab_changed()
    assert resources.reserved and state.resumes == 0
    state.wake_active = False
    page._tick()
    assert not resources.reserved and state.resumes == 1


def test_asr_raw_final_is_consumed_once_and_only_manual_fill_or_copy(rig):
    page, _, _, client = rig
    enter(page)
    page.start_recording()
    assert page.voice.recording.wait(1)
    rid = page.asr_request
    page.events.put(('asr_partial', rid, {'text': '转起红色'}))
    page._drain_events()
    assert page.asr_text.value == '转起红色'
    page.finish_recording()
    settle(page, lambda: not page.audio_active)
    assert page.final_text == '转起红色方块'
    assert not page.tts_text.value and not client.begins
    page.events.put(('asr_final', rid, {'text': '旧的机械臂指令'}))
    page._drain_events()
    assert page.final_text == '转起红色方块'
    page.copy_text()
    assert page.page.clipboard == page.final_text
    page.fill_tts()
    assert page.tts_text.value == page.final_text


def test_tab_exit_holds_lease_until_capture_really_exits_and_drops_late_final(rig):
    page, resources, state, _ = rig
    enter(page)
    page.start_recording()
    assert page.voice.recording.wait(1)
    rid = page.asr_request
    page.notebook.select('original')
    page._tab_changed()
    page._tick()
    assert resources.reserved and state.resumes == 0
    page.events.put(('asr_final', rid, {'text': '抓取红色方块'}))
    page._drain_events()
    assert not page.final_text
    page.voice.release.set()
    settle(page, lambda: not resources.reserved)
    assert state.resumes == 1


def test_rapid_reentry_cannot_start_a_second_capture_before_old_exit(rig):
    page, _, _, _ = rig
    enter(page)
    page.start_recording()
    assert page.voice.recording.wait(1)
    old_thread = page.voice._thread
    page.notebook.select('original')
    page._tab_changed()
    enter(page)
    page.start_recording()
    assert page.voice._thread is old_thread
    page.voice.release.set()
    settle(page, lambda: not page.audio_active)
    page.start_recording()
    assert page.voice._thread is not old_thread


def test_empty_and_error_clear_partial_text_without_tts(rig):
    page, _, _, client = rig
    enter(page)
    for kind, data in (('asr_empty', {}), ('asr_error', {'message': '麦克风断开'})):
        page.asr_request = 'synthetic'
        page.asr_text.value = '未定稿'
        page.events.put((kind, 'synthetic', data))
        page._drain_events()
        assert not page.asr_request and not page.asr_text.value
    assert page.asr_status.value == '麦克风断开' and not client.begins


def test_shared_tts_busy_is_never_replaced_or_stopped(rig):
    page, _, _, client = rig
    enter(page)
    client.active = 'original-dialogue'
    page.tts_text.value = '实验文本'
    page.start_speaking()
    settle(page, lambda: not page.audio_active)
    assert client.begins[0][0] is False
    assert client.active == 'original-dialogue' and not client.stops
    assert '朗读失败' in page.tts_status.value


def test_asr_refuses_capture_through_external_playback_without_global_stop(rig):
    page, _, _, client = rig
    enter(page)
    client.socket_path.touch()
    client.active = 'another-client'
    page.start_recording()
    settle(page, lambda: not page.audio_active)
    assert '正在播报' in page.asr_status.value
    assert client.active == 'another-client' and not client.stops


def test_recording_and_speaking_are_mutually_exclusive(rig):
    page, _, _, client = rig
    enter(page)
    page.start_recording()
    assert page.voice.recording.wait(1)
    page.tts_text.value = '不能同时朗读'
    page.start_speaking()
    assert not client.begins
    page.finish_recording()
    settle(page, lambda: not page.audio_active)
    page.start_speaking()
    assert client.finish_entered.wait(1)
    page.start_recording()
    assert not page.asr_request
    page.stop_speaking()
    settle(page, lambda: not page.audio_active)


def test_normal_tts_reuses_streaming_calls_without_global_stop(rig):
    page, _, _, client = rig
    enter(page)
    text = '这是一段超过二十四个字符的文本，用于检查当前分段接口的复用。'
    page.tts_text.value = text
    page.start_speaking()
    assert client.finish_entered.wait(1)
    client.release.set()
    settle(page, lambda: not page.audio_active)
    assert client.text == text and not client.stops
    assert page.tts_status.value == '朗读完成'


def test_replaced_session_is_not_stopped_by_tab_cleanup(rig):
    page, _, _, client = rig
    enter(page)
    page.tts_text.value = '实验朗读'
    page.start_speaking()
    assert client.finish_entered.wait(1)
    client.active = 'external-replacement'
    client.release.set()
    page.stop_speaking()
    settle(page, lambda: not page.audio_active)
    assert client.active == 'external-replacement' and not client.stops


def test_stops_only_own_session_on_exit_and_waits_before_resuming_wake(rig):
    page, resources, state, client = rig
    enter(page)
    page.tts_text.value = '本地朗读'
    page.start_speaking()
    assert client.finish_entered.wait(1)
    rid = page.tts_request
    page.notebook.select('original')
    page._tab_changed()
    settle(page, lambda: not resources.reserved)
    assert client.stops == [rid] and state.resumes == 1


def test_lost_start_ack_is_reconciled_and_own_session_is_cleaned_up(rig):
    page, _, _, client = rig
    enter(page)
    client.lose_ack = True
    page.tts_text.value = '测试丢失响应'
    page.start_speaking()
    settle(page, lambda: not page.audio_active)
    assert len(client.stops) == 1 and not client.active
    assert 'lost start ACK' in page.tts_status.value


def test_delayed_start_ack_cannot_restart_playback_after_exit(rig):
    page, resources, _, client = rig
    enter(page)
    client.begin_gate = threading.Event()
    page.tts_text.value = '延迟开始'
    page.start_speaking()
    assert client.begin_entered.wait(1)
    page.notebook.select('original')
    page._tab_changed()
    assert resources.reserved
    client.begin_gate.set()
    settle(page, lambda: not resources.reserved)
    assert not client.active and not client.text and len(client.stops) == 1


def test_unknown_tts_state_keeps_lease_until_recovery(rig):
    page, resources, state, client = rig
    enter(page)
    page.tts_text.value = '状态不可用'
    page.start_speaking()
    assert client.finish_entered.wait(1)
    client.status_failed = True
    page.notebook.select('original')
    page._tab_changed()
    settle(page, lambda: '重试释放' in page.tts_status.value)
    assert resources.reserved and state.resumes == 0
    client.status_failed = False
    settle(page, lambda: not resources.reserved)
    assert state.resumes == 1


def test_original_robot_activity_cancels_experiment_but_not_original_task(rig):
    page, _, state, _ = rig
    enter(page)
    page.start_recording()
    assert page.voice.recording.wait(1)
    state.busy = True
    page._tick()
    assert page.voice.canceled and state.busy


def test_incomplete_speech_status_does_not_authorize_release():
    for status in ({}, {'session_running': True}, {'session_running': None}):
        with pytest.raises(SpeechServiceError):
            _active_speech_request(status)


@pytest.mark.parametrize('method,args', [
    ('send_text', ('抓取红色方块',)), ('toggle_voice', ()), ('ask_image', ()),
    ('execute_selected_action', ()), ('execute_xyz_move', ()), ('command_gripper', ('open',)),
])
def test_original_dispatch_waits_for_experiment_release(method, args):
    messages = []
    gui = SimpleNamespace(speech_resources=SimpleNamespace(reserved=True),
                          _append=lambda *value: messages.append(value))
    # No model or bridge exists: any accidental dispatch would fail this test.
    getattr(DialogueGui, method)(gui, *args)
    assert messages


def test_stop_robot_remains_available_and_original_global_speech_stop_is_guarded():
    calls = []
    gui = SimpleNamespace(speech_resources=SimpleNamespace(reserved=True),
                          bridge=SimpleNamespace(cancel_direct=lambda: calls.append('robot-stop') or True))
    DialogueGui.stop_action(gui)
    DialogueGui.stop_playback(gui)
    assert calls == ['robot-stop']


def test_text_stop_shortcut_still_uses_original_route_during_release():
    routes = []
    gui = SimpleNamespace(
        speech_resources=SimpleNamespace(reserved=True), _voice_request='',
        input_var=Widget(), _pause_wake=lambda: None, _append=lambda *args: None,
        _request_sources={}, _handle_intent=lambda rid, text, intent: routes.append(intent.route))
    DialogueGui.send_text(gui, '停止抓取')
    assert routes == ['stop_action']


def test_delayed_original_speech_start_cannot_replace_experiment():
    gui = SimpleNamespace(
        speech_resources=SimpleNamespace(reserved=True),
        _speech_start_lock=threading.Lock(), _closing=False,
        _speech_epoch=1, _voice_request='', speech=SimpleNamespace(
            start=lambda **kwargs: pytest.fail('original speech replaced experiment')))
    assert DialogueGui._begin_generation_speech(gui, 'original', True, 1) is False


def test_disabled_wake_with_old_followup_mode_does_not_block_experiment():
    gui = SimpleNamespace(
        _closing=False, _dialogue_busy=False, _gripper_busy=False,
        _active_action_request='', _voice_request='', _playback_busy=False,
        _wake_pending_followup='', _wake_resume_mode='followup', _wake_request='',
        _wake_is_enabled=lambda: False,
        bridge=SimpleNamespace(state_snapshot=lambda: ({}, 0),
                               guard=SimpleNamespace(snapshot=lambda: SimpleNamespace(active=False))))
    assert DialogueGui._edge_speech_foreground_busy(gui) is False


def test_original_wake_setting_is_preserved_when_lease_ends(rig):
    page, resources, _, _ = rig
    checkbox = Widget()
    checkbox.set(False)
    gui = SimpleNamespace(speech_resources=resources, wake_enabled=checkbox,
                          wake=SimpleNamespace(active=False), _closing=False, _voice_request='',
                          _dialogue_busy=False, _playback_busy=False)
    gui._wake_is_enabled = lambda: DialogueGui._wake_is_enabled(gui)
    enter(page)
    assert not DialogueGui._wake_can_listen(gui)
    page.notebook.select('original')
    page._tab_changed()
    assert checkbox.get() is False and not DialogueGui._wake_can_listen(gui)


def test_real_tk_layout_and_text_transfer(tmp_path):
    import tkinter as tk
    from tkinter import ttk
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('Tk display is not available')
    root.withdraw()
    resources = SpeechResourceCoordinator(lambda: False, lambda: None, lambda: False, lambda: None)
    notebook = ttk.Notebook(root)
    notebook.pack(fill='both', expand=True)
    notebook.add(ttk.Frame(notebook), text='original')
    page = EdgeSpeechPage(notebook, tmp_path, resources, tmp_path / 'absent.sock',
                          voice_factory=Voice, client=Client(tmp_path / 'absent.sock'))
    try:
        notebook.select(page.page)
        root.update()
        assert '端侧语音感知' in notebook.tab(page.page, 'text')
        page.final_text = '离线语音测试'
        page.fill_tts()
        assert page.tts_text.get('1.0', 'end-1c') == page.final_text
        page.copy_text()
        assert root.clipboard_get() == page.final_text
        notebook.select(0)
        root.update()
        assert not resources.reserved
    finally:
        page.close()
        root.destroy()


def test_real_gui_mounts_page_and_keeps_controls_visible_at_minimum_size(tmp_path, monkeypatch):
    import tkinter as tk
    import fr3c_voice.dialogue_gui as gui_module
    import fr3c_voice.edge_speech_page as page_module
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('Tk display is not available')
    root.withdraw()
    monkeypatch.setattr(gui_module, 'VoiceInput', Voice)
    monkeypatch.setattr(page_module, 'VoiceInput', Voice)
    # Default argument binds the original factory: supply the fake explicitly.
    original_page_init = EdgeSpeechPage.__init__

    def page_init(self, *args, **kwargs):
        kwargs.update(voice_factory=Voice, client=Client(tmp_path / 'absent.sock'))
        original_page_init(self, *args, **kwargs)

    monkeypatch.setattr(EdgeSpeechPage, '__init__', page_init)
    wake = SimpleNamespace(enabled_by_default=True, active=False,
                           cancel=lambda **kwargs: None, close=lambda: None)
    monkeypatch.setattr(gui_module, 'WakeInput', lambda *args: wake)
    monkeypatch.setattr(DialogueGui, '_poll', lambda self: None)
    monkeypatch.setattr(DialogueGui, '_ensure_wake_listener', lambda self: None)
    snapshot = SimpleNamespace(active=False, task='', phase='')
    bridge = SimpleNamespace(
        guard=SimpleNamespace(snapshot=lambda: snapshot), state_snapshot=lambda: ({}, 0),
        action_server_ready=lambda *args: False, gripper_command_active=lambda: False,
        gripper_server_ready=lambda: False)
    runtime = SimpleNamespace(bridge=bridge, close=lambda: None)
    gui = DialogueGui(root, SimpleNamespace(), runtime, tmp_path / 'gui.log', workspace=tmp_path)
    try:
        page = gui.edge_speech_page
        assert page is not None
        gui.pages.select(page.page)
        root.geometry('760x560')
        root.deiconify()
        root.update()
        assert gui.speech_resources.reserved and gui.wake_enabled.get()
        for button in (page.record_button, page.copy_button, page.fill_button,
                       page.speak_button, page.stop_button):
            assert button.winfo_y() + button.winfo_height() <= button.master.winfo_height()
            assert button.master.winfo_y() + button.master.winfo_height() <= button.master.master.winfo_height()
        gui.pages.select(0)
        root.update()
        assert not gui.speech_resources.reserved and gui.wake_enabled.get()
    finally:
        gui.close()
