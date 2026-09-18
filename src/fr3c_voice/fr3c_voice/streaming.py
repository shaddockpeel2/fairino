"""Streaming speech pipeline: tokens in, sound out, with bounded everything.

Shape of the pipeline::

    token -> sanitize -> FirstSegmentSplitter -> bounded text queue
          -> synthesis thread -> bounded PCM queue -> playback thread -> ALSA

Three properties are enforced rather than hoped for:

* **Audio starts before the text is finished.**  The first segment is released
  on the first sentence boundary or the first-segment cap, not at the end of the
  answer, so playback overlaps generation.  Phase 5 measured the cost of that
  choice: first-valid-PCM roughly 0.26 s for a segment of up to 24 characters.
* **Queues are bounded.**  A consumer that stalls applies backpressure to the
  producer instead of growing memory; latency cannot silently accumulate.
* **"Queued" is not "heard".**  ``playback_end`` is emitted after the queue is
  empty *and* the device has drained, and the trace records both moments.

The synthesiser is driven through :mod:`fr3c_voice.tts_backends`, which is the
same code path phase 5 measured, so the latency numbers stay comparable.
"""
from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable

from .pcm_player import AlsaPcmPlayer, PcmCancelled, PcmPlayerError
from .text_sanitizer import clean_for_speech, is_speakable, looks_structured
from .tts_backends import FirstSegmentSplitter, SegmentPolicy, TtsError

PLAYER_SENTINEL = object()
TEXT_SENTINEL = object()

DEFAULT_PCM_QUEUE_BLOCKS = 12
DEFAULT_TEXT_QUEUE_SEGMENTS = 8


class PipelineState:
    IDLE = 'idle'
    GENERATING = 'generating'
    SPEAKING = 'speaking'
    CANCELLING = 'cancelling'


@dataclass
class PipelineEvent:
    kind: str
    state: str
    request_id: str = ''
    text: str = ''
    message: str = ''
    elapsed_s: float | None = None
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class SegmentRecord:
    index: int
    text: str
    characters: int
    submitted_s: float
    synthesized_s: float
    synthesis_seconds: float
    audio_seconds: float
    first_valid_pcm_s: float
    rtf: float
    queue_wait_s: float


@dataclass
class PipelineMetrics:
    """Everything the phase-6 acceptance needs, in one place."""
    request_id: str = ''
    text_characters: int = 0
    segments: list[SegmentRecord] = field(default_factory=list)
    first_token_s: float | None = None
    first_segment_s: float | None = None
    first_pcm_s: float | None = None
    first_valid_pcm_write_s: float | None = None
    synthesis_end_s: float | None = None
    text_end_s: float | None = None
    playback_end_s: float | None = None
    stop_requested_s: float | None = None
    stop_effective_s: float | None = None
    audio_seconds_written: float = 0.0
    text_queue_max: int = 0
    pcm_queue_max: int = 0
    text_queue_full_waits: int = 0
    pcm_queue_full_waits: int = 0
    xruns: int = 0
    dropped_at_cancel_s: float | None = None
    cancelled: bool = False
    failure: str = ''

    @property
    def stop_latency_s(self) -> float | None:
        if self.stop_requested_s is None or self.stop_effective_s is None:
            return None
        return self.stop_effective_s - self.stop_requested_s

    def as_dict(self) -> dict[str, Any]:
        # request_id is deliberately absent: the trace record already carries
        # it, and duplicating the key here made every trace call ambiguous.
        return {
            'text_characters': self.text_characters,
            'segments': [segment.__dict__ for segment in self.segments],
            'segment_count': len(self.segments),
            'first_token_s': self.first_token_s,
            'first_segment_s': self.first_segment_s,
            'first_pcm_s': self.first_pcm_s,
            'first_valid_pcm_write_s': self.first_valid_pcm_write_s,
            'synthesis_end_s': self.synthesis_end_s,
            'text_end_s': self.text_end_s,
            'playback_end_s': self.playback_end_s,
            'stop_latency_s': self.stop_latency_s,
            'audio_seconds_written': round(self.audio_seconds_written, 4),
            'text_queue_max': self.text_queue_max,
            'pcm_queue_max': self.pcm_queue_max,
            'text_queue_full_waits': self.text_queue_full_waits,
            'pcm_queue_full_waits': self.pcm_queue_full_waits,
            'xruns': self.xruns,
            'cancelled': self.cancelled,
            'failure': self.failure,
        }


class StreamingSpeechPipeline:
    """Own one speaker and stream TTS into it while text is still arriving."""

    def __init__(self, backend: Any,
                 policy: SegmentPolicy | None = None,
                 player: AlsaPcmPlayer | None = None,
                 trace: Any | None = None,
                 pcm_queue_blocks: int = DEFAULT_PCM_QUEUE_BLOCKS,
                 text_queue_segments: int = DEFAULT_TEXT_QUEUE_SEGMENTS,
                 allow_backend_cache: bool = False) -> None:
        # Dynamic answers must not be served from a cache: a hit would report a
        # first-segment latency of zero, which is exactly the kind of number
        # this phase exists to avoid. Fixed status phrases are played through
        # `speak_cached` instead, which is a different, explicit path.
        self.allow_backend_cache = bool(allow_backend_cache)
        if hasattr(backend, 'allow_cache'):
            backend.allow_cache = self.allow_backend_cache
        self.backend = backend
        self.policy = policy or SegmentPolicy()
        self.player = player
        self.trace = trace
        self._text_queue: queue.Queue[Any] = queue.Queue(maxsize=text_queue_segments)
        self._pcm_queue: queue.Queue[Any] = queue.Queue(maxsize=pcm_queue_blocks)
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._callbacks: list[Callable[[PipelineEvent], None]] = []
        self._threads: list[threading.Thread] = []
        self._session_id = ''
        self._splitter = FirstSegmentSplitter(self.policy)
        self._metrics = PipelineMetrics()
        self._started_at = 0.0
        self._segment_counter = 0
        self._running = False
        self._text_closed = False
        self._deadline_seconds: float | None = None
        self._first_segment_deadline_seconds: float | None = None
        self._deadline_fired = False
        self._stop_event_emitted = threading.Event()

    # -- configuration ---------------------------------------------------- #

    def subscribe(self, callback: Callable[[PipelineEvent], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def set_deadlines(self, first_segment_max_wait_s: float | None,
                      segment_max_wait_s: float | None) -> None:
        """Bound how long a partial segment may wait for more text.

        Without this the splitter would hold an unpunctuated clause until the
        hard cap, which is exactly the "wait for a full stop that never comes"
        failure the phase plan warns about.
        """
        self._first_segment_deadline_seconds = first_segment_max_wait_s
        self._deadline_seconds = segment_max_wait_s

    @property
    def metrics(self) -> PipelineMetrics:
        with self._lock:
            return self._metrics

    # -- session lifecycle ------------------------------------------------ #

    def start(self, request_id: str, sample_rate: int | None = None) -> None:
        with self._lock:
            if self._running:
                raise RuntimeError('a speech session is already running')
            self._cancel = threading.Event()
            self._stop_event_emitted = threading.Event()
            self._splitter.reset()
            self._metrics = PipelineMetrics(request_id=request_id)
            self._text_queue = queue.Queue(maxsize=self._text_queue.maxsize)
            self._pcm_queue = queue.Queue(maxsize=self._pcm_queue.maxsize)
            self._segment_counter = 0
            self._text_closed = False
            self._started_at = time.monotonic()
            self._segment_buffer_since = self._started_at
            self._running = True
        if self.player is not None:
            sample_rate = sample_rate or getattr(self.backend, 'sample_rate', 22050)
            # A session boundary must be silent: drop whatever the previous
            # reply left in the device buffer.
            if hasattr(self.player, 'begin_session'):
                self.player.begin_session(sample_rate)
            else:
                self.player.prepare(sample_rate)
        self._emit('speech_start', PipelineState.GENERATING, request_id=request_id)
        self._threads = [
            threading.Thread(target=self._synthesis_loop, name='tts-synthesis', daemon=True),
            threading.Thread(target=self._playback_loop, name='tts-playback', daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def push_text(self, chunk: str) -> tuple[str, ...]:
        """Feed a token or sentence; returns segments released by this call."""
        if not chunk:
            return ()
        with self._lock:
            if not self._running:
                raise RuntimeError('no active speech session')
            if self._metrics.first_token_s is None:
                self._metrics.first_token_s = time.monotonic() - self._started_at
        # This is a model token, not a complete sentence.  Appending terminal
        # punctuation here would make every token sound like a separate phrase.
        cleaned = clean_for_speech(chunk, ensure_terminal=False)
        if not cleaned:
            return ()
        self._metrics.text_characters += len(cleaned)
        if self._splitter.buffered_characters == 0:
            self._segment_buffer_since = time.monotonic()
        segments = list(self._splitter.feed(cleaned))
        if self._splitter.buffered_characters == 0:
            self._segment_buffer_since = time.monotonic()
        # A partial clause must not wait forever for punctuation that may never
        # come; this runs on the producer side so the splitter is only ever
        # touched by one thread.
        stalled = self._flush_if_stalled()
        if stalled:
            segments.extend(stalled)
        self._submit_segments(segments)
        return tuple(segments)

    def _flush_if_stalled(self) -> tuple[str, ...]:
        """Release a buffered partial segment once its wait budget is spent."""
        with self._lock:
            emitted = self._splitter.emitted_segments
            buffered = self._splitter.buffered_characters
            limit = (self._first_segment_deadline_seconds if emitted == 0
                     else self._deadline_seconds)
            if limit is None or buffered <= 0:
                return ()
            waited = time.monotonic() - self._segment_buffer_since
            if waited < limit:
                return ()
            segments = self._splitter.flush()
        if segments:
            self._emit('segment_deadline_flush', PipelineState.GENERATING,
                       buffered_characters=buffered, deadline_s=limit,
                       waited_s=round(waited, 4))
        return segments

    def end_text(self) -> tuple[str, ...]:
        """Signal that generation finished; flushes the tail as a final segment."""
        with self._lock:
            if not self._running or self._text_closed:
                return ()
            self._text_closed = True
            self._metrics.text_end_s = time.monotonic() - self._started_at
        segments = self._splitter.finish()
        self._submit_segments(segments)
        self._put_text(TEXT_SENTINEL)
        return segments

    def wait(self, timeout: float | None = None) -> PipelineMetrics:
        for thread in list(self._threads):
            thread.join(timeout=timeout)
        return self.metrics

    def stop(self, reason: str = 'user') -> float:
        """Cancel and silence the device immediately. Returns drop time."""
        requested = time.monotonic()
        with self._lock:
            if not self._running:
                return 0.0
            self._metrics.cancelled = True
            self._metrics.stop_requested_s = requested - self._started_at
            self._cancel.set()
            player = self.player
        dropped = 0.0
        if player is not None:
            dropped = player.drop()
        with self._lock:
            self._metrics.stop_effective_s = time.monotonic() - self._started_at
        # Unblock both workers so they can observe the cancel flag.
        self._drain_queue(self._text_queue)
        self._drain_queue(self._pcm_queue)
        self._put_text(TEXT_SENTINEL)
        self._put_pcm(PLAYER_SENTINEL)
        self._emit('speech_stopped', PipelineState.CANCELLING, message=reason)
        self._stop_event_emitted.set()
        return dropped

    def close(self) -> None:
        if self._running:
            self.stop('close')
        if self.player is not None:
            self.player.close()

    # -- internals -------------------------------------------------------- #

    def _drain_queue(self, target: queue.Queue) -> None:
        while True:
            try:
                target.get_nowait()
                target.task_done()
            except queue.Empty:
                return

    def _put_text(self, item: Any) -> None:
        while True:
            try:
                self._text_queue.put(item, timeout=0.05)
                return
            except queue.Full:
                if self._cancel.is_set():
                    return

    def _put_pcm(self, item: Any) -> None:
        while True:
            try:
                self._pcm_queue.put(item, timeout=0.05)
                return
            except queue.Full:
                if self._cancel.is_set():
                    return
                with self._lock:
                    self._metrics.pcm_queue_full_waits += 1

    def _submit_segments(self, segments: Iterable[str]) -> None:
        for segment in segments:
            if not segment:
                continue
            with self._lock:
                index = self._segment_counter
                self._segment_counter += 1
                if self._metrics.first_segment_s is None:
                    self._metrics.first_segment_s = time.monotonic() - self._started_at
            self._emit('first_text_segment' if index == 0 else 'text_segment',
                       PipelineState.GENERATING, text=segment, index=index,
                       characters=len(segment))
            self._put_text((index, segment))

    def _synthesis_loop(self) -> None:
        try:
            while True:
                item = self._text_queue.get()
                try:
                    if item is TEXT_SENTINEL:
                        return
                    index, segment = item
                    if self._cancel.is_set():
                        return
                    queued_at = time.monotonic()
                    result = self.backend.synthesize(segment)
                    now = time.monotonic() - self._started_at
                    with self._lock:
                        self._metrics.segments.append(SegmentRecord(
                            index=index, text=segment, characters=len(segment),
                            submitted_s=queued_at - self._started_at,
                            synthesized_s=now,
                            synthesis_seconds=result.synthesis_s,
                            audio_seconds=result.audio_duration_s,
                            first_valid_pcm_s=result.first_audible_s or 0.0,
                            rtf=result.rtf or 0.0,
                            queue_wait_s=queued_at - self._started_at,
                        ))
                        if self._metrics.first_pcm_s is None:
                            self._metrics.first_pcm_s = now
                    self._emit('tts_first_pcm', PipelineState.SPEAKING, index=index,
                               synthesis_s=result.synthesis_s,
                               audio_s=result.audio_duration_s,
                               first_valid_pcm_s=result.first_audible_s)
                    if self._cancel.is_set():
                        return
                    self._put_pcm((index, result.pcm, result.sample_rate))
                finally:
                    self._text_queue.task_done()
        except (TtsError, RuntimeError) as error:
            with self._lock:
                self._metrics.failure = repr(error)
            self._emit('speech_error', PipelineState.IDLE, message=repr(error))
        finally:
            # Whichever way synthesis ended, the playback thread must be told
            # there is nothing more coming, or it waits forever and the session
            # never reports playback_end.
            self._put_pcm(PLAYER_SENTINEL)

    def _playback_loop(self) -> None:
        import numpy as np

        try:
            while True:
                item = self._pcm_queue.get()
                try:
                    if item is PLAYER_SENTINEL:
                        break
                    index, pcm, sample_rate = item
                    if self._cancel.is_set():
                        break
                    with self._lock:
                        self._metrics.pcm_queue_max = max(
                            self._metrics.pcm_queue_max, self._pcm_queue.qsize())
                    def mark_first_chunk() -> None:
                        with self._lock:
                            if self._metrics.first_valid_pcm_write_s is None:
                                self._metrics.first_valid_pcm_write_s = (
                                    time.monotonic() - self._started_at)
                                self._emit('first_valid_pcm_write',
                                           PipelineState.SPEAKING, index=index)

                    if self.player is not None:
                        self.player.write(pcm.tobytes(), self._cancel, mark_first_chunk)
                    else:
                        mark_first_chunk()
                    with self._lock:
                        self._metrics.audio_seconds_written += float(pcm.size) / sample_rate
                finally:
                    self._pcm_queue.task_done()
        except PcmCancelled:
            pass
        except PcmPlayerError as error:
            with self._lock:
                self._metrics.failure = repr(error)
            self._emit('speech_error', PipelineState.IDLE, message=repr(error))

        if self._cancel.is_set() or self._metrics.failure:
            self._finish(cancelled=True)
            return
        try:
            if self.player is not None:
                drain_seconds = self.player.drain()
                self._emit('player_drained', PipelineState.SPEAKING, drain_s=drain_seconds)
        except PcmPlayerError as error:
            with self._lock:
                self._metrics.failure = repr(error)
            self._emit('speech_error', PipelineState.IDLE, message=repr(error))
        self._finish(cancelled=False)

    def _finish(self, cancelled: bool) -> None:
        with self._lock:
            self._metrics.playback_end_s = time.monotonic() - self._started_at
            self._metrics.synthesis_end_s = self._metrics.playback_end_s
            self._metrics.dropped_at_cancel_s = (
                self._metrics.playback_end_s if cancelled else None)
            stats = getattr(self.player, 'stats', None) if self.player else None
            self._metrics.xruns = getattr(stats, 'xruns', 0) or 0
            self._running = False
            metrics = self._metrics
        # The metrics payload already carries a `cancelled` field, so the
        # session-level flag is emitted under a distinct name.
        self._emit('playback_end', PipelineState.IDLE,
                   session_cancelled=cancelled, **metrics.as_dict())

    def _emit(self, kind: str, state: str, **details: Any) -> None:
        request_id = self._metrics.request_id
        elapsed = None
        if self._started_at:
            elapsed = time.monotonic() - self._started_at
        text = details.pop('text', '')
        event = PipelineEvent(kind=kind, state=state, request_id=request_id,
                              text=text, elapsed_s=elapsed, details=details)
        with self._lock:
            callbacks = list(self._callbacks)
            trace = self.trace
        if trace is not None:
            # request_id is a parameter of TraceWriter.write, so it must not
            # also appear inside **details.
            details.pop('request_id', None)
            trace.write(kind, request_id, state=state,
                        elapsed_s=elapsed, text=text, **details)
        for callback in callbacks:
            try:
                callback(event)
            except Exception:  # a bad subscriber must not break audio
                continue


def summarise_metrics(metrics: PipelineMetrics) -> dict[str, Any]:
    """Metrics plus the two derived answers the acceptance asks for."""
    payload = metrics.as_dict()
    payload['audio_started_before_text_end'] = (
        metrics.first_valid_pcm_write_s is not None
        and metrics.text_end_s is not None
        and metrics.first_valid_pcm_write_s < metrics.text_end_s
    )
    payload['inter_segment_gaps_s'] = [
        round(metrics.segments[i + 1].submitted_s - metrics.segments[i].synthesized_s, 4)
        for i in range(len(metrics.segments) - 1)
    ]
    return payload


def write_metrics(path: Path | str, metrics: PipelineMetrics) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(summarise_metrics(metrics), ensure_ascii=False, indent=2) + '\n')
