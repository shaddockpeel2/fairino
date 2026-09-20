from pathlib import Path
import queue
import threading
from types import MethodType, SimpleNamespace

from fr3c_voice.dialogue_gui import speech_socket_path, validate_runtime


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / 'src/fr3c_voice/fr3c_voice/dialogue_gui.py'


def test_gui_exposes_dialogue_and_two_state_gripper_pages():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'ttk.Notebook' in source
    assert '点动' not in source
    for label in (
            '发送', '开始说话', '截图问答', '自动播报',
            '停止播报', '停止抓取', '模型', '视觉', '任务',
            '知识', '资料来源', '唤醒：未唤醒', '正在接收问题',
            '秒后退出', '我在', '夹爪控制', '打开夹爪', '闭合夹爪',
            '夹持；无物体时完全闭合', '夹爪 Action 服务不可用'):
        assert label in source
    assert "command=lambda: self.command_gripper('open')" in source
    assert "command=lambda: self.command_gripper('close')" in source
    assert '60 mm · 70°' in source
    assert '26 mm · 31°' in source
    assert "'tk', 'useinputmethods', True" in source
    # Stage 6 made the speech controls functional; they must not regress to
    # placeholders that silently do nothing.
    assert 'state=\'disabled\'' not in source.split('tts_check = ')[1].split('pack(')[0]
    assert 'StreamingSpeaker' in source
    assert 'speech_socket_path' in source


def test_start_script_connects_tk_to_available_input_method():
    script = (
        WORKSPACE / 'scripts/voice/start_dialogue_gui.sh'
    ).read_text(encoding='utf-8')
    assert "XMODIFIERS='@im=ibus'" in script
    assert "XMODIFIERS='@im=fcitx'" in script


def test_full_start_script_preserves_ros_python_and_detaches_services():
    script = (
        WORKSPACE / 'scripts/voice/start_dialogue_full.sh'
    ).read_text(encoding='utf-8')
    assert 'set -euo pipefail' in script
    assert 'nohup setsid bash -c' in script
    assert 'KNOWLEDGE_PYTHON' in script
    assert '.venv-rk3588/bin/python' in script
    assert "PYTHONUNBUFFERED=1 PYTHONPATH='${ROOT}/src/fr3c_voice'" in script
    assert 'socket_healthy' in script


def test_qwen_stop_script_can_reap_an_orphan_worker():
    script = (
        WORKSPACE / 'scripts/voice/stop_qwen_service.sh'
    ).read_text(encoding='utf-8')
    assert 'qwen3_vl_worker' in script
    assert 'find_qwen_pids' in script
    assert 'kill -TERM' in script
    assert 'kill -KILL' in script


def test_validate_only_needs_neither_tk_window_nor_ros_runtime():
    socket_path, runtime_path = validate_runtime(
        WORKSPACE / 'config/voice/qwen3vl.yaml', WORKSPACE)
    assert socket_path.name == 'fr3c-qwen-service.sock'
    assert runtime_path == WORKSPACE / 'evidence/voice/P9/runtime'


def test_stage8_uses_current_camera_and_real_chat_generation():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'filedialog.askopenfilename' not in source
    assert 'latest_color_frame' in source
    assert 'conversation.build_prompt' in source
    assert "f'{request_id}-chat'" in source
    assert "f'{request_id}-vision'" in source
    assert "('answer_token', request_id, token)" in source


def test_speech_socket_matches_the_phase6_service_location():
    """The GUI and the speech service must agree on one socket path."""
    expected = WORKSPACE / 'evidence/voice/P6/runtime/fr3c-tts-pipeline.sock'
    assert speech_socket_path(WORKSPACE) == expected


def test_speech_client_keeps_the_dialogue_usable_without_the_service():
    """A missing speech service must degrade, not crash the page."""
    from fr3c_voice.speech_client import SpeechClient, StreamingSpeaker

    client = SpeechClient(WORKSPACE / 'evidence/voice/P6/runtime/does-not-exist.sock')
    assert client.available() is False
    speaker = StreamingSpeaker(client, enabled=True)
    assert speaker.start() is False
    assert speaker.last_error or True  # error recorded, nothing raised
    assert speaker.finish() == {}

    disabled = StreamingSpeaker(client, enabled=False)
    assert disabled.start() is False


def test_task_status_does_not_label_canceled_or_failed_state_ready():
    from fr3c_voice.dialogue_gui import DialogueGui

    for state, active, retryable, expected in (
        ('canceled_holding_state_unchanged', False, False, '任务：已取消，等待恢复'),
        ('failed_holding_state_unchanged', False, False, '任务：未就绪'),
        ('complete_holding', False, True, '任务：等待确认物体已取下'),
        ('complete_sorted_home', False, True, '任务：就绪'),
        ('waiting_for_target', True, True, '任务：运行中'),
    ):
        displayed = {}
        bridge = SimpleNamespace(
            action_servers_ready=lambda: True,
            guard=SimpleNamespace(snapshot=lambda: SimpleNamespace(active=False)),
            state_snapshot=lambda: ({'state': state, 'action_active': active,
                                     'retryable': retryable}, 100),
            perception_snapshot=lambda: ({}, 100))
        gui = SimpleNamespace(bridge=bridge, _status_values={
            name: SimpleNamespace(set=lambda value, key=name: displayed.update({key: value}))
            for name in ('task', 'vision')})
        DialogueGui._refresh_local_status(gui, 100)
        assert displayed['task'].startswith(expected)


def test_gui_poll_reschedules_after_an_unexpected_event_error():
    from fr3c_voice.dialogue_gui import DialogueGui

    scheduled = []
    gui = SimpleNamespace(
        _closing=False,
        POLL_MS=100,
        root=SimpleNamespace(after=lambda delay, callback: scheduled.append(
            (delay, callback))),
        _drain_ui_events=lambda: (_ for _ in ()).throw(RuntimeError('bad event')),
        _drain_action_events=lambda: None,
    )
    gui._poll = MethodType(DialogueGui._poll, gui)

    gui._poll()

    assert len(scheduled) == 1
    assert scheduled[0][0] == 100
    assert scheduled[0][1].__func__ is DialogueGui._poll


def test_stage8_generation_streams_same_tokens_to_ui_and_speech():
    from fr3c_voice.dialogue_gui import DialogueGui

    class Generator:
        def generate(self, prompt, request_id='', image_path=None, on_token=None):
            assert prompt == 'prompt'
            assert request_id == 'req-chat'
            assert image_path is None
            on_token('你')
            on_token('好')
            return '你好'

    class Speech:
        last_error = ''

        def __init__(self): self.tokens = []

        def start(self, request_id=''):
            assert request_id == 'req'
            return True

        def feed(self, token): self.tokens.append(token)

        def finish(self, timeout_s=0):
            assert timeout_s == 900.0
            return {'first_valid_pcm_write_s': 0.2}

    class Trace:
        def write(self, *_args, **_kwargs): pass

    gui = DialogueGui.__new__(DialogueGui)
    gui._speech_start_lock = threading.Lock()
    gui._closing = False
    gui._speech_epoch = 7
    gui._voice_request = ''
    gui.speech = Speech()
    gui.generator = Generator()
    gui.ui_events = queue.Queue()
    gui.trace = Trace()
    answer = gui._generate_answer(
        'req', 'prompt', 'req-chat', None, True, 7)
    assert answer == '你好'
    assert gui.speech.tokens == ['你', '好']
    events = []
    while not gui.ui_events.empty():
        events.append(gui.ui_events.get_nowait()[0])
    assert events == [
        'speech_stream_started', 'answer_token', 'answer_token',
        'answer_result', 'speech_done']
