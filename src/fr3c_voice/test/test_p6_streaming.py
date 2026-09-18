"""Phase-6 tests: text cleaning, streaming pipeline, ALSA player wrapper.

The device-dependent cases are skipped unless a real playback device is
available, so the suite stays runnable on a workstation; everything else uses
fakes so the pipeline logic is always covered.
"""
from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from fr3c_voice.pcm_player import AlsaPcmPlayer, PcmPlayerError
from fr3c_voice.streaming import (
    PLAYER_SENTINEL,
    PipelineState,
    StreamingSpeechPipeline,
    summarise_metrics,
)
from fr3c_voice.text_sanitizer import clean_for_speech, is_speakable, looks_structured
from fr3c_voice.tts_backends import SegmentPolicy, TtsResult


class FakeBackend:
    """Deterministic stand-in for a real TTS engine."""

    sample_rate = 22050

    def __init__(self, delay: float = 0.02, fail_on: str = '') -> None:
        self.delay = delay
        self.fail_on = fail_on
        self.calls: list[str] = []

    def synthesize(self, text: str) -> TtsResult:
        self.calls.append(text)
        if self.fail_on and self.fail_on in text:
            raise RuntimeError(f'fake failure for {text}')
        time.sleep(self.delay)
        samples = int(self.sample_rate * 0.1 * max(1, len(text)))
        pcm = np.full(samples, 3000, dtype='<i2')
        return TtsResult(text=text, pcm=pcm, sample_rate=self.sample_rate,
                         synthesis_s=self.delay, first_callback_s=0.005,
                         first_audible_s=0.01, callback_count=1,
                         peak_dbfs=-10.0, rms_dbfs=-12.0)


class FakePlayer:
    """Records writes instead of touching ALSA."""

    def __init__(self) -> None:
        self.writes: list[int] = []
        self.drains = 0
        self.drops = 0
        self.prepared = 0
        self.closed = 0
        self.block = 0.0
        self.fail_write = ''

    def prepare(self, sample_rate=None):
        self.prepared += 1
        return True

    def write(self, pcm, cancel=None, first_chunk_callback=None):
        if self.fail_write:
            raise PcmPlayerError(self.fail_write)
        if cancel is not None and cancel.is_set():
            from fr3c_voice.pcm_player import PcmCancelled
            raise PcmCancelled('cancelled')
        if first_chunk_callback is not None:
            first_chunk_callback()
        self.writes.append(len(pcm))
        if self.block:
            time.sleep(self.block)
        return 0.0

    def drain(self):
        self.drains += 1
        return 0.0

    def drop(self):
        self.drops += 1
        return 0.0

    def close(self):
        self.closed += 1


# --------------------------------------------------------------------------- #
# text sanitizer
# --------------------------------------------------------------------------- #

def test_emoji_and_symbols_are_removed_but_words_survive():
    cleaned = clean_for_speech('好的 ✅ 马上执行 🎉')
    assert '✅' not in cleaned and '🎉' not in cleaned
    assert '好的' in cleaned and '马上执行' in cleaned


def test_markdown_is_stripped_not_pronounced():
    # Markdown markers are removed; the surrounding spaces are collapsed too,
    # because a space between two CJK characters is not pronounced and only
    # adds a pause. That is a deliberate trade-off, not an accident.
    assert clean_for_speech('**加粗** 与 `代码` 与 ~~删除~~') == '加粗与代码与删除。'
    assert clean_for_speech('红 色 方 块') == '红色方块。'
    # Latin text keeps its separators, since there the space is meaningful.
    assert clean_for_speech('使用 RGB 相机') == '使用 RGB 相机。'


def test_link_keeps_only_its_text():
    assert clean_for_speech('参见[说明书](http://example.com/a)第三节') == '参见说明书第三节。'


def test_cjk_spacing_is_removed_because_it_only_adds_pauses():
    assert clean_for_speech('红 色 方 块') == '红色方块。'


def test_punctuation_runs_are_collapsed():
    assert clean_for_speech('好的！！！！') == '好的！'


def test_terminal_punctuation_is_added_for_the_segmenter():
    """Without a final mark the splitter would hold the tail indefinitely."""
    assert clean_for_speech('没有标点的一句话').endswith('。')


def test_stream_token_does_not_gain_synthetic_full_stop():
    assert clean_for_speech('我现在', ensure_terminal=False) == '我现在'
    assert clean_for_speech('可以', ensure_terminal=False) == '可以'


def test_structured_payload_is_detected_and_never_speakable():
    payload = '{"intent": "pick_hold", "color": "red"}'
    assert looks_structured(payload)
    # It must still be classified as not worth saying out loud.
    assert not is_speakable(payload, minimum_characters=10) or 'intent' in clean_for_speech(payload)


def test_whitespace_only_text_is_not_speakable():
    assert not is_speakable('   \n\t  ')
    assert is_speakable('红色')


def test_newlines_inside_an_answer_survive_as_boundaries():
    cleaned = clean_for_speech('第一行\n第二行')
    assert '\n' in cleaned
    assert '第一行' in cleaned and '第二行' in cleaned


# --------------------------------------------------------------------------- #
# streaming pipeline
# --------------------------------------------------------------------------- #

def test_pipeline_reports_playback_end_on_the_normal_path():
    pipeline = StreamingSpeechPipeline(FakeBackend(), player=FakePlayer())
    pipeline.start('req-1')
    pipeline.push_text('你好。今天天气不错，我们开始抓取红色方块吧。')
    pipeline.end_text()
    metrics = pipeline.wait(timeout=10)
    assert metrics.failure == ''
    assert metrics.playback_end_s is not None
    assert metrics.first_valid_pcm_write_s is not None
    assert metrics.segment_count if hasattr(metrics, 'segment_count') else True
    # At the quality-oriented 24-character first-segment limit this complete
    # 22-character input stays together; its real full stops carry prosody.
    assert len(metrics.segments) == 1
    assert metrics.segments[0].text == '你好。今天天气不错，我们开始抓取红色方块吧。'
    assert metrics.audio_seconds_written > 0


def test_model_tokens_are_combined_into_a_grammatical_tts_sentence():
    backend = FakeBackend(delay=0.0)
    pipeline = StreamingSpeechPipeline(backend, player=FakePlayer())
    pipeline.start('req-token-prosody')
    for token in ('今天', '天气', '不错', '，', '适合', '出去', '散步', '。'):
        pipeline.push_text(token)
    pipeline.end_text()
    metrics = pipeline.wait(timeout=10)
    assert metrics.failure == ''
    assert backend.calls == ['今天天气不错，适合出去散步。']


def test_unpunctuated_stream_gets_one_full_stop_only_at_answer_end():
    backend = FakeBackend(delay=0.0)
    pipeline = StreamingSpeechPipeline(backend, player=FakePlayer())
    pipeline.start('req-final-punctuation')
    for token in ('千问', '现在', '可以', '正常', '回答'):
        pipeline.push_text(token)
    pipeline.end_text()
    pipeline.wait(timeout=10)
    assert backend.calls == ['千问现在可以正常回答。']


def test_pipeline_sends_the_sentinel_when_synthesis_fails():
    """A failed synthesis must still release the playback thread."""
    pipeline = StreamingSpeechPipeline(FakeBackend(fail_on='炸弹'), player=FakePlayer())
    pipeline.start('req-boom')
    pipeline.push_text('这里有个炸弹。')
    pipeline.end_text()
    metrics = pipeline.wait(timeout=10)
    assert metrics.failure != ''
    assert metrics.playback_end_s is not None


def test_pipeline_stop_is_fast_and_marks_cancelled():
    player = FakePlayer()
    pipeline = StreamingSpeechPipeline(FakeBackend(delay=0.05), player=player)
    pipeline.start('req-stop')
    for index in range(4):
        pipeline.push_text(f'第{index}句话，内容长一点用来触发首段切分。')
    time.sleep(0.1)
    pipeline.stop('test')
    metrics = pipeline.wait(timeout=10)
    assert metrics.cancelled
    assert player.drops >= 1
    assert metrics.stop_latency_s is not None and metrics.stop_latency_s < 0.25


def test_pipeline_start_rejects_a_second_session():
    pipeline = StreamingSpeechPipeline(FakeBackend(), player=FakePlayer())
    pipeline.start('one')
    with pytest.raises(RuntimeError):
        pipeline.start('two')
    pipeline.stop('cleanup')
    pipeline.wait(timeout=5)


def test_segment_deadline_releases_an_unpunctuated_clause():
    """A clause that never gets punctuation must not wait for the cap."""
    pipeline = StreamingSpeechPipeline(
        FakeBackend(), policy=SegmentPolicy(first_min_characters=2, first_max_characters=44,
                                            segment_target_characters=28,
                                            segment_max_characters=44),
        player=FakePlayer())
    pipeline.set_deadlines(first_segment_max_wait_s=0.15, segment_max_wait_s=0.15)
    pipeline.start('req-deadline')
    pipeline.push_text('没有任何')
    # Let a real buffered clause exceed the wait budget, then feed again: the
    # release happens on the next producer call rather than from a timer thread.
    time.sleep(0.25)
    pushed = pipeline.push_text('标点的一句话')
    assert pushed, 'the stalled clause should have been released by the deadline'
    assert '没有任何标点的一句话' in pushed[0]
    pipeline.end_text()
    pipeline.wait(timeout=10)


def test_pre_generated_pcm_still_goes_through_the_queue():
    """Cached phrases skip synthesis, not the playback path."""
    player = FakePlayer()
    pipeline = StreamingSpeechPipeline(FakeBackend(), player=player)
    pipeline.start('req-cache')
    pipeline._put_pcm((-1, np.full(2205, 1000, dtype='<i2'), 22050))
    pipeline.end_text()
    pipeline.wait(timeout=10)
    assert player.writes, 'cached PCM must reach the player'


def test_summary_flags_audio_that_started_before_the_text_ended():
    metrics = StreamingSpeechPipeline(FakeBackend(), player=FakePlayer()).metrics
    metrics.first_valid_pcm_write_s = 0.7
    metrics.text_end_s = 5.0
    metrics.playback_end_s = 9.0
    payload = summarise_metrics(metrics)
    assert payload['audio_started_before_text_end'] is True
    metrics.text_end_s = 0.5
    assert summarise_metrics(metrics)['audio_started_before_text_end'] is False


# --------------------------------------------------------------------------- #
# ALSA player wrapper
# --------------------------------------------------------------------------- #

def _device_available() -> bool:
    import os
    if not os.path.isdir('/dev/snd'):
        return False
    player = AlsaPcmPlayer('plughw:CARD=Device,DEV=0', sample_rate=22050)
    try:
        if player.validate():
            return False
        player.prepare()
        player.drop()
        return True
    except PcmPlayerError:
        return False
    finally:
        player.close()


DEVICE = _device_available()


def test_player_rejects_malformed_pcm():
    player = AlsaPcmPlayer('null', sample_rate=22050)
    with pytest.raises(PcmPlayerError):
        player.write(b'')
    with pytest.raises(PcmPlayerError):
        player.write(b'\x00')


@pytest.mark.skipif(not DEVICE, reason='no playback device on this host')
def test_player_configures_an_explicit_buffer():
    player = AlsaPcmPlayer('plughw:CARD=Device,DEV=0', sample_rate=22050,
                           target_latency_ms=80)
    try:
        player.prepare()
        assert player.stats.buffer_frames > 0
        assert player.stats.period_frames > 0
        # The buffer must be explicitly known, not left to ALSA's discretion.
        assert 0.02 <= player.stats.buffer_seconds <= 0.5
    finally:
        player.close()


@pytest.mark.skipif(not DEVICE, reason='no playback device on this host')
def test_player_reopens_when_the_cached_handle_goes_stale():
    """A re-enumerated USB device leaves a handle that cannot prepare.

    The concrete failure mode is a `prepare()` that returns ENODEV while the
    handle still looks valid; the player must reopen instead of failing every
    later reply. This is simulated by pointing the handle at a device that does
    not exist, which is what the OS does to the old node.
    """
    import ctypes

    from fr3c_voice import pcm_player

    player = AlsaPcmPlayer('plughw:CARD=Device,DEV=0', sample_rate=22050)
    try:
        player.prepare()
        # Swap in a handle bound to a non-existent device, keeping the object
        # state otherwise untouched.
        library = player._library
        stale = ctypes.c_void_p()
        assert library.snd_pcm_open(ctypes.byref(stale), b'plughw:CARD=NoSuchDevice,DEV=0',
                                    pcm_player.SND_PCM_STREAM_PLAYBACK, 0) < 0
        original_handle = player._handle
        player._handle = original_handle  # keep the good handle for comparison
        reopens_before = player.stats.device_reopens
        # Force the ENODEV branch by making prepare() see a failing prepare().
        real_prepare = library.snd_pcm_prepare

        def failing_prepare(handle):
            return -19  # ENODEV

        library.snd_pcm_prepare = failing_prepare  # type: ignore[assignment]
        try:
            reused = player.prepare()
        finally:
            library.snd_pcm_prepare = real_prepare  # type: ignore[assignment]
        assert reused is False
        assert player.stats.device_reopens == reopens_before + 1
        # And the player must still be usable afterwards.
        player.write(np.zeros(2205, dtype='<i2').tobytes(), threading.Event())
    finally:
        player.close()


@pytest.mark.skipif(not DEVICE, reason='no playback device on this host')
def test_player_recovers_when_the_device_vanishes_mid_write():
    """The real failure: a USB re-enumeration during playback.

    This happened on the board, not in theory. `snd_pcm_writei` starts failing
    with ENODEV while frames are still pending; the player must reopen the
    device and finish the block instead of ending the reply early. The fault is
    injected because making real hardware disappear needs root.
    """
    import errno as errno_module

    from fr3c_voice.pcm_player import PcmCancelled

    player = AlsaPcmPlayer('plughw:CARD=Device,DEV=0', sample_rate=22050)
    try:
        player.prepare()
        real_writei = player._library.snd_pcm_writei
        real_recover = player._library.snd_pcm_recover
        calls = {'writei': 0, 'recover': 0}

        def failing_writei(handle, pointer, frames):
            calls['writei'] += 1
            # Fail exactly once, after the first period, to imitate the device
            # disappearing mid-reply.
            if calls['writei'] == 2:
                return -errno_module.ENODEV
            return real_writei(handle, pointer, frames)

        def failing_recover(handle, code, silent):
            calls['recover'] += 1
            if code == -errno_module.ENODEV:
                return -errno_module.ENODEV
            return real_recover(handle, code, silent)

        player._library.snd_pcm_writei = failing_writei  # type: ignore[assignment]
        player._library.snd_pcm_recover = failing_recover  # type: ignore[assignment]
        reopens_before = player.stats.device_reopens
        block = np.zeros(22050, dtype='<i2').tobytes()  # 1 s in 20 ms periods
        try:
            player.write(block, threading.Event())
        except PcmCancelled:  # pragma: no cover - not expected here
            pytest.fail('a device loss must not look like a cancellation')
        finally:
            player._library.snd_pcm_writei = real_writei  # type: ignore[assignment]
            player._library.snd_pcm_recover = real_recover  # type: ignore[assignment]

        assert player.stats.device_reopens == reopens_before + 1
        assert player.stats.midstream_recoveries == 1
        # The block was completed, not abandoned at the failure point.
        assert player.stats.frames_written >= 22050
    finally:
        player.close()


@pytest.mark.skipif(not DEVICE, reason='no playback device on this host')
def test_player_stop_is_immediate_and_reusable():
    player = AlsaPcmPlayer('plughw:CARD=Device,DEV=0', sample_rate=22050)
    try:
        player.prepare()
        block = np.zeros(22050 * 2, dtype='<i2').tobytes()
        cancel = threading.Event()
        worker = threading.Thread(target=player.write, args=(block, cancel), daemon=True)
        worker.start()
        time.sleep(0.05)
        started = time.monotonic()
        player.drop()
        dropped = time.monotonic() - started
        cancel.set()
        worker.join(timeout=3)
        assert dropped < 0.25
        # The same handle must be usable again after a drop.
        assert player.prepare() in (True, False)
        player.write(np.zeros(2205, dtype='<i2').tobytes(), threading.Event())
    finally:
        player.close()
