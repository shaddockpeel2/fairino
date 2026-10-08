from pathlib import Path
from collections import deque
from types import SimpleNamespace
import time
import wave

import pytest

from fr3c_voice.dialogue_gui import DialogueGui
from fr3c_voice.wake_input import (
    WakeInput,
    _append_pre_roll,
    extract_wake_query,
    wake_query_from_asr,
)


WORKSPACE = Path(__file__).resolve().parents[3]


def test_pre_roll_keeps_time_by_bytes_with_short_capture_reads():
    ring = deque()
    size = 0
    for value in range(8):
        size = _append_pre_roll(ring, size, bytes([value]) * 2, 10)
    assert size == 10
    assert b''.join(ring) == b''.join(bytes([value]) * 2 for value in range(3, 8))


def test_wake_query_parser_keeps_same_utterance_question():
    phrases = ('法奥同学',)
    assert extract_wake_query('法奥同学，介绍一下机械臂', phrases) == (
        True, '介绍一下机械臂')
    assert extract_wake_query('你好，法奥同学！你看到了什么', phrases) == (
        True, '你看到了什么')
    assert extract_wake_query('发澳童雪，抓起红色方块', phrases) == (
        True, '抓起红色方块')
    assert extract_wake_query('法奥同学法奥同学，抓起蓝色方块', phrases) == (
        True, '抓起蓝色方块')
    assert extract_wake_query('我刚才提到法奥同学', phrases) == (False, '')


def test_cpu_kws_is_authority_but_wake_fragments_are_not_questions():
    phrases = ('法奥同学',)
    assert wake_query_from_asr('澳童雪', phrases) == ''
    assert wake_query_from_asr('发澳童雪', phrases) == ''
    assert wake_query_from_asr('法奥同学', phrases) == ''
    assert wake_query_from_asr('打奥同学', phrases) == ''
    assert wake_query_from_asr('浩同学', phrases) == ''
    assert wake_query_from_asr('老同学', phrases) == ''
    assert wake_query_from_asr('啥好同学', phrases) == ''
    assert wake_query_from_asr('打奥同学，介绍一下机械臂', phrases) == '介绍一下机械臂'
    assert wake_query_from_asr('浩同学，抓起红色方块', phrases) == '抓起红色方块'
    assert wake_query_from_asr('法奥同介绍一下机械臂', phrases) == '介绍一下机械臂'
    assert wake_query_from_asr('介绍一下机械臂', phrases) == '介绍一下机械臂'


def test_stage9_assets_and_timing_configuration_are_valid():
    wake = WakeInput(WORKSPACE, lambda _event: None)
    wake.validate()
    assert wake.enabled_by_default is True
    assert wake.phrases == ('玉米',)
    command, _env = wake._asr_command()
    assert '--hotwords-file' not in command
    assert wake.config['pre_roll_s'] == 3.0
    assert wake.config['followup_initial_silence_s'] == 7.0
    assert wake.config['save_debug_audio'] is True
    assert wake.asr['trailing_silence_s'] == 0.65
    assert wake.asr['maximum_record_s'] == 30.0


def test_debug_recording_preserves_asr_pcm(tmp_path):
    path = tmp_path / 'asr.wav'
    wake = object.__new__(WakeInput)
    wake._debug_audio = wave.open(str(path), 'wb')
    wake._debug_audio.setnchannels(1)
    wake._debug_audio.setsampwidth(2)
    wake._debug_audio.setframerate(16000)
    wake.trace = SimpleNamespace(write=lambda *_args, **_kwargs: None)
    raw = b'\x01\x00\xff\x7f' * 100
    wake._append_debug_audio('test', raw)
    wake._debug_audio.close()
    with wave.open(str(path), 'rb') as recording:
        assert recording.getframerate() == 16000
        assert recording.readframes(recording.getnframes()) == raw


def _event_gui():
    displayed = []
    values = []
    submissions = []
    scheduled = []
    gui = SimpleNamespace(
        wake=SimpleNamespace(
            phrases=('法奥同学',),
            config={'retry_delay_s': 2.0, 'resume_delay_s': 0.5}),
        _wake_request='wake-1',
        _wake_mode='wake',
        _wake_last_error='',
        _wake_followup_deadline=0.0,
        _wake_followup_heard=False,
        _wake_pending_followup='',
        _reply_request='',
        _playback_busy=False,
        _status_values={'voice': SimpleNamespace(set=displayed.append)},
        input_var=SimpleNamespace(set=values.append),
        _append=lambda *args: displayed.append(args),
        speak=lambda text: displayed.append(('speak', text)),
        send_text=lambda text, **kwargs: submissions.append((text, kwargs)),
        _schedule_wake=lambda mode, delay_s=0.0: scheduled.append((mode, delay_s)),
    )
    gui._play_wake_acknowledgement = (
        lambda rid: DialogueGui._play_wake_acknowledgement(gui, rid))
    gui._refresh_wake_countdown = (
        lambda now: DialogueGui._refresh_wake_countdown(gui, now))
    return gui, displayed, values, submissions, scheduled


def test_wake_final_submits_same_utterance_once():
    gui, _displayed, values, submissions, scheduled = _event_gui()
    DialogueGui._handle_wake_event(
        gui, 'wake_final', 'wake-1',
        {'mode': 'wake', 'text': '发澳童雪，请介绍机械臂'})
    DialogueGui._handle_wake_event(
        gui, 'wake_final', 'wake-1',
        {'mode': 'wake', 'text': '发澳童雪，请介绍机械臂'})
    assert values == ['请介绍机械臂']
    assert submissions == [(
        '请介绍机械臂', {'request_id': 'wake-1', 'source': 'wake'})]
    assert scheduled == []


def test_wake_only_says_ack_then_opens_one_followup_window():
    gui, displayed, values, submissions, scheduled = _event_gui()
    DialogueGui._handle_wake_event(
        gui, 'wake_final', 'wake-1',
        {'mode': 'wake', 'text': '发澳童雪'})
    assert values == ['']
    assert submissions == []
    assert ('assistant', '我在') in displayed
    assert ('speak', '我在') in displayed
    assert scheduled == [('followup', 0.0)]


@pytest.mark.parametrize('asr_text', ('打奥同学', '浩同学', '老同学', '啥好同学'))
def test_misrecognized_wake_only_never_reaches_model(asr_text):
    gui, displayed, values, submissions, scheduled = _event_gui()
    DialogueGui._handle_wake_event(
        gui, 'wake_final', 'wake-1', {'mode': 'wake', 'text': asr_text})
    assert values == ['']
    assert submissions == []
    assert ('speak', '我在') in displayed
    assert scheduled == [('followup', 0.0)]


def test_wake_ack_playback_finishes_before_followup_window():
    gui, displayed, _values, _submissions, scheduled = _event_gui()
    def start_playback(text):
        displayed.append(('speak', text))
        gui._playback_busy = True
    gui.speak = start_playback
    DialogueGui._play_wake_acknowledgement(gui, 'wake-1')
    assert gui._wake_pending_followup == 'wake-1'
    assert scheduled == []
    assert ('speak', '我在') in displayed


def test_followup_status_shows_awake_countdown_then_recognizing():
    gui, displayed, values, _submissions, _scheduled = _event_gui()
    gui._wake_mode = 'followup'
    gui.wake.config['followup_initial_silence_s'] = 7.0
    DialogueGui._handle_wake_event(
        gui, 'wake_followup_listening', 'wake-1', {})
    assert gui._wake_followup_deadline > time.monotonic()
    assert any('已唤醒' in value and '秒后退出' in value
               for value in displayed if isinstance(value, str))
    DialogueGui._handle_wake_event(
        gui, 'wake_partial', 'wake-1', {'text': '介绍一下'})
    assert values[-1] == '介绍一下'
    assert gui._wake_followup_deadline == 0.0
    assert displayed[-1] == '唤醒：已唤醒 · 正在识别问题'


def test_spoken_wake_reply_waits_for_playback_before_followup():
    gui = SimpleNamespace(
        _request_sources={'request-1': 'wake'},
        _playback_busy=True,
        _wake_pending_followup='',
        _status_values={'voice': SimpleNamespace(set=lambda _value: None)},
        wake=SimpleNamespace(config={'resume_delay_s': 0.5}),
        _schedule_wake=lambda *_args, **_kwargs: None)
    DialogueGui._resume_after_request(gui, 'request-1')
    assert gui._wake_pending_followup == 'request-1'
    assert gui._request_sources == {}
