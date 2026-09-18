"""Speaker-owning streaming speech service.

One small service owns the ALSA device so two writers can never fight over it,
exactly as the phase-5 plan requires.  It does not talk to Qwen: the caller
drives generation and forwards tokens, which keeps generation and playback
independently testable and lets the acceptance harness substitute a controlled
slow token source.

Protocol (JSONL over a local Unix socket, same style as the Qwen service)::

    {"op": "health"}                                   -> {"event": "health", ...}
    {"op": "status"}                                   -> {"event": "status", ...}
    {"op": "speak_start", "request_id": "..."}         -> {"event": "speech_start", ...}
    {"op": "speak_text", "request_id": "...", "text": "..."} -> {"event": "ack", ...}
    {"op": "speak_end", "request_id": "..."}           -> blocks until playback_end
    {"op": "stop", "reason": "user"}                   -> {"event": "stopped", ...}
    {"op": "shutdown"}                                 -> {"event": "shutdown", ...}

A request handler thread executes ``speak_end`` synchronously and forwards every
pipeline event to that client, so the caller sees the whole timeline.  ``stop``
is handled on a different connection and takes effect immediately.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import fcntl
import json
import os
from pathlib import Path
import signal
import socketserver
import threading
import time
from typing import Any

from .pcm_player import AlsaPcmPlayer
from .streaming import PipelineEvent, StreamingSpeechPipeline, summarise_metrics
from .text_sanitizer import clean_for_speech, is_speakable
from .trace import TraceWriter
from .tts_backends import FirstSegmentSplitter, SegmentPolicy, TtsError, resolve

WORKSPACE = Path(__file__).resolve().parents[3]  # src/fr3c_voice/fr3c_voice/<file>
MAX_REQUEST_BYTES = 1024 * 1024

DEFAULT_DEVICE = 'plughw:CARD=Device,DEV=0'
DEFAULT_SOCKET = 'fr3c-tts-pipeline.sock'


class PipelineConfig:
    """Runtime configuration, kept explicit so the report can quote it."""

    def __init__(self, *, device: str = DEFAULT_DEVICE, candidate: str = 'matcha-icefall-zh-baker',
                 threads: int = 3, speed: float = 1.0, speaker_id: int = 0,
                 target_latency_ms: int = 80,
                 first_min_characters: int = 4, first_max_characters: int = 24,
                 segment_target_characters: int = 28, segment_max_characters: int = 44,
                 first_segment_max_wait_s: float = 0.65,
                 segment_max_wait_s: float = 0.8,
                 pcm_queue_blocks: int = 12, text_queue_segments: int = 8,
                 affinity: str = 'big', warmup_text: str = '好的',
                 runtime_dir: Path | None = None,
                 candidate_config: Path | None = None,
                 asset_root: Path | None = None) -> None:
        self.device = device
        self.candidate = candidate
        self.threads = threads
        self.speed = speed
        self.speaker_id = speaker_id
        self.target_latency_ms = target_latency_ms
        self.first_min_characters = first_min_characters
        self.first_max_characters = first_max_characters
        self.segment_target_characters = segment_target_characters
        self.segment_max_characters = segment_max_characters
        self.first_segment_max_wait_s = first_segment_max_wait_s
        self.segment_max_wait_s = segment_max_wait_s
        self.pcm_queue_blocks = pcm_queue_blocks
        self.text_queue_segments = text_queue_segments
        self.affinity = affinity
        self.warmup_text = warmup_text
        self.runtime_dir = Path(runtime_dir or (WORKSPACE / 'evidence/voice/P6/runtime'))
        self.candidate_config = Path(candidate_config or (WORKSPACE / 'config/voice/tts_candidates.json'))
        self.asset_root = Path(asset_root or (WORKSPACE / 'assets/voice/tts'))
        self.socket_name = DEFAULT_SOCKET

    def apply_affinity(self) -> str:
        if not hasattr(os, 'sched_setaffinity'):
            return 'unsupported'
        if self.affinity == 'big':
            os.sched_setaffinity(0, {4, 5, 6, 7})
            return 'big:4-7'
        if self.affinity == 'little':
            os.sched_setaffinity(0, {0, 1, 2, 3})
            return 'little:0-3'
        return 'default'


class SpeechServiceCore:
    """Owns the backend and the device; creates one pipeline per session."""

    def __init__(self, config: PipelineConfig, trace: TraceWriter | None = None) -> None:
        self.config = config
        self.trace = trace
        self._lock = threading.RLock()
        self._pipeline: StreamingSpeechPipeline | None = None
        self._subscribers: list[Any] = []
        self._cache: dict[str, Any] = {}
        self._closed = False
        self.affinity = config.apply_affinity()

        self.backend = resolve(config.candidate_config, config.asset_root, config.candidate)
        # Pre-generated phrases live in this table, not in a synthesis cache, so
        # a dynamic answer can never be answered from a previous run.
        self.backend.allow_cache = False
        self.backend.threads = config.threads
        self.backend.speed = config.speed
        self.backend.speaker_id = config.speaker_id
        load_started = time.monotonic()
        self.backend.load()
        self.load_s = time.monotonic() - load_started
        self.player = AlsaPcmPlayer(config.device, sample_rate=self.backend.sample_rate,
                                    target_latency_ms=config.target_latency_ms)
        errors = self.player.validate()
        if errors:
            raise RuntimeError(f'ALSA player unavailable: {errors}')
        self.player.prepare(self.backend.sample_rate)
        self.warmup()

    # -- lifecycle -------------------------------------------------------- #

    def warmup(self) -> dict[str, Any]:
        """Synthesise the fixed phrases once so status speech never waits."""
        phrases = [self.config.warmup_text, '我在', '收到', '好的']
        for phrase in dict.fromkeys(clean_for_speech(text) for text in phrases):
            if phrase and phrase not in self._cache:
                self._cache[phrase] = self.backend.precompute(phrase)
        return {'cached_phrases': sorted(self._cache), 'warmup_phrases': len(self._cache)}

    def preload(self, texts: list[str]) -> list[str]:
        """Pre-generate fixed status phrases; they still need a real trigger."""
        added = []
        for text in texts:
            cleaned = clean_for_speech(text)
            if not cleaned or cleaned in self._cache:
                continue
            self._cache[cleaned] = self.backend.precompute(cleaned)
            added.append(cleaned)
        return added

    @property
    def cached_texts(self) -> list[str]:
        return sorted(self._cache)

    def close(self) -> None:
        with self._lock:
            self._closed = True
            if self._pipeline is not None:
                self._pipeline.close()
                self._pipeline = None
        self.player.close()

    # -- sessions --------------------------------------------------------- #

    def start_session(self, request_id: str, replace: bool = False) -> dict[str, Any]:
        """Start a session, optionally replacing one that is still playing.

        `replace=True` implements the interruption a real caller needs: saying
        something new while the previous answer is still being spoken must cut
        that answer off rather than fail. The default refuses instead, so an
        accidental double start cannot silently truncate a reply.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError('speech service is closed')
            active = self._pipeline
            if active is not None and active._running:  # noqa: SLF001
                if not replace:
                    raise RuntimeError(
                        'a speech session is already running '
                        f'(request_id={active.metrics.request_id}); '
                        'send stop first or start with replace=true')
                active.stop('replaced_by_new_session')
                self._pipeline = None
            policy = SegmentPolicy(
                first_min_characters=self.config.first_min_characters,
                first_max_characters=self.config.first_max_characters,
                segment_target_characters=self.config.segment_target_characters,
                segment_max_characters=self.config.segment_max_characters)
            pipeline = StreamingSpeechPipeline(
                self.backend, policy=policy, player=self.player, trace=self.trace,
                pcm_queue_blocks=self.config.pcm_queue_blocks,
                text_queue_segments=self.config.text_queue_segments)
            pipeline.set_deadlines(self.config.first_segment_max_wait_s,
                                   self.config.segment_max_wait_s)
            for callback in self._subscribers:
                pipeline.subscribe(callback)
            pipeline.start(request_id)
            self._pipeline = pipeline
            return {'request_id': request_id, 'sample_rate': self.backend.sample_rate,
                    'cached_phrases': len(self._cache)}

    def subscribe(self, callback: Any) -> None:
        with self._lock:
            self._subscribers.append(callback)
            if self._pipeline is not None:
                self._pipeline.subscribe(callback)

    def unsubscribe(self, callback: Any) -> None:
        with self._lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)
            if self._pipeline is not None:
                with self._pipeline._lock:  # noqa: SLF001 - deliberate detach
                    if callback in self._pipeline._callbacks:
                        self._pipeline._callbacks.remove(callback)

    def push_text(self, request_id: str, text: str) -> dict[str, Any]:
        pipeline = self._require(request_id)
        segments = pipeline.push_text(text)
        return {'segments': list(segments)}

    def speak_cached(self, request_id: str, text: str) -> bool:
        """Play a pre-generated phrase on a real trigger, without synthesis."""
        cleaned = clean_for_speech(text)
        if cleaned not in self._cache:
            return False
        result = self._cache[cleaned]
        pipeline = self._pipeline
        if pipeline is None:
            raise RuntimeError('no active speech session')
        # Route the cached PCM through the same queue the synthesiser uses, so
        # "pre-generated" changes the cost, not the code path.
        pipeline._put_pcm((-1, result.pcm, result.sample_rate))  # noqa: SLF001
        return True

    def end_text(self, request_id: str) -> StreamingSpeechPipeline:
        pipeline = self._require(request_id)
        pipeline.end_text()
        return pipeline

    def _require(self, request_id: str) -> StreamingSpeechPipeline:
        with self._lock:
            pipeline = self._pipeline
            if pipeline is None or not pipeline._running:  # noqa: SLF001
                raise RuntimeError('no active speech session')
            if request_id and pipeline.metrics.request_id != request_id:
                raise RuntimeError(
                    f'request_id mismatch: active={pipeline.metrics.request_id} got={request_id}')
            return pipeline

    def stop(self, reason: str = 'user') -> dict[str, Any]:
        with self._lock:
            pipeline = self._pipeline
        if pipeline is None:
            return {'stopped': False, 'reason': 'no session'}
        dropped = pipeline.stop(reason)
        return {'stopped': True, 'drop_s': dropped, **pipeline.metrics.as_dict()}

    def status(self) -> dict[str, Any]:
        with self._lock:
            pipeline = self._pipeline
            running = bool(pipeline and pipeline._running)  # noqa: SLF001
            metrics = pipeline.metrics.as_dict() if pipeline else None
        return {
            'candidate': self.config.candidate,
            'sample_rate': self.backend.sample_rate,
            'load_s': self.load_s,
            'affinity': self.affinity,
            'device': self.config.device,
            'device_buffer_seconds': self.player.stats.buffer_seconds,
            'device_period_frames': self.player.stats.period_frames,
            'cached_phrases': len(self._cache),
            'session_running': running,
            'player_stats': self.player.stats.as_dict(),
            'last_metrics': metrics,
        }

    def health(self) -> dict[str, Any]:
        return {'healthy': not self._closed, 'candidate': self.config.candidate,
                'sample_rate': self.backend.sample_rate, 'affinity': self.affinity}


class SpeechRequestHandler(socketserver.StreamRequestHandler):
    def _write(self, value: dict[str, Any]) -> None:
        self.wfile.write(json.dumps(value, ensure_ascii=False,
                                    separators=(',', ':')).encode('utf-8') + b'\n')
        self.wfile.flush()

    def handle(self) -> None:
        raw = self.rfile.readline(MAX_REQUEST_BYTES + 1)
        if not raw or len(raw) > MAX_REQUEST_BYTES:
            self._write({'event': 'error', 'error': 'request_too_large_or_empty'})
            return
        core: SpeechServiceCore = self.server.core  # type: ignore[attr-defined]
        try:
            request = json.loads(raw.decode('utf-8'))
            operation = str(request.get('op') or '')
            request_id = str(request.get('request_id') or '')

            if operation == 'health':
                self._write({'event': 'health', **core.health()})
                return
            if operation == 'status':
                self._write({'event': 'status', **core.status()})
                return
            if operation == 'stop':
                self._write({'event': 'stopped',
                             **core.stop(str(request.get('reason') or 'user'))})
                return
            if operation == 'preload':
                texts = request.get('texts') or []
                if not isinstance(texts, list):
                    raise ValueError('texts must be a list')
                added = core.preload([str(item) for item in texts])
                self._write({'event': 'preloaded', 'added': added,
                             'cached': core.cached_texts})
                return
            if operation == 'shutdown':
                self._write({'event': 'shutdown', 'ok': True})
                threading.Thread(target=self.server.shutdown, daemon=True).start()  # type: ignore[attr-defined]
                return
            if operation == 'speak_start':
                self._write({'event': 'speech_start', **core.start_session(
                    request_id, replace=bool(request.get('replace')))})
                return
            if operation == 'speak_text':
                result = core.push_text(request_id, str(request.get('text') or ''))
                self._write({'event': 'ack', 'request_id': request_id, **result})
                return
            if operation == 'speak_cached':
                used = core.speak_cached(request_id, str(request.get('text') or ''))
                self._write({'event': 'ack', 'request_id': request_id, 'cached': used})
                return
            if operation == 'speak_end':
                pipeline = core.end_text(request_id)
                metrics_holder: dict[str, Any] = {}

                def forward(event: PipelineEvent) -> None:
                    payload = {'event': 'pipeline', 'kind': event.kind, 'state': event.state,
                               'request_id': event.request_id,
                               'elapsed_s': event.elapsed_s, **event.details}
                    if event.text:
                        payload['text'] = event.text
                    if event.kind == 'playback_end':
                        metrics_holder.update(event.details)
                    self._write(payload)

                core.subscribe(forward)
                try:
                    metrics = pipeline.wait(timeout=float(request.get('timeout_s') or 600))
                    summary = summarise_metrics(metrics)
                    metrics_holder.update(summary)
                finally:
                    core.unsubscribe(forward)
                self._write({'event': 'result', 'request_id': request_id,
                             **metrics_holder})
                return
            raise ValueError(f'unsupported operation: {operation}')
        except (BrokenPipeError, ConnectionResetError):
            core.stop('client_disconnected')
        except Exception as error:  # noqa: BLE001 - reported to the client
            try:
                self._write({'event': 'error',
                             'error': f'{type(error).__name__}: {error}'})
            except (BrokenPipeError, ConnectionResetError):
                pass


class SpeechUnixServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = False

    def __init__(self, path: str, core: SpeechServiceCore) -> None:
        self.core = core
        super().__init__(path, SpeechRequestHandler)


def build_core(config: PipelineConfig, trace_path: Path | None = None) -> SpeechServiceCore:
    trace = TraceWriter(trace_path) if trace_path else None
    return SpeechServiceCore(config, trace)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=WORKSPACE)
    parser.add_argument('--device', default=DEFAULT_DEVICE)
    parser.add_argument('--candidate', default='matcha-icefall-zh-baker')
    parser.add_argument('--threads', type=int, default=3)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--speaker-id', type=int, default=0)
    parser.add_argument('--latency-ms', type=int, default=80)
    parser.add_argument('--first-max-characters', type=int, default=24)
    parser.add_argument('--segment-target-characters', type=int, default=28)
    parser.add_argument('--first-max-wait', type=float, default=0.65)
    parser.add_argument('--segment-max-wait', type=float, default=0.8)
    parser.add_argument('--affinity', default='big', choices=('default', 'big', 'little'))
    parser.add_argument('--trace', type=Path)
    parser.add_argument('--runtime-dir', type=Path)
    parser.add_argument('--foreground', action='store_true',
                        help='keep running in the foreground instead of returning after ready')
    args = parser.parse_args()

    runtime_dir = Path(args.runtime_dir or (args.workspace / 'evidence/voice/P6/runtime'))
    runtime_dir.mkdir(parents=True, exist_ok=True)
    lock_path = runtime_dir / 'tts-pipeline.lock'
    lock = lock_path.open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print(json.dumps({'event': 'error', 'error': 'another pipeline instance holds the lock'}),
              flush=True)
        return 2
    lock.write(str(os.getpid()))
    lock.flush()

    config = PipelineConfig(
        device=args.device, candidate=args.candidate, threads=args.threads, speed=args.speed,
        speaker_id=args.speaker_id, target_latency_ms=args.latency_ms,
        first_max_characters=args.first_max_characters,
        segment_target_characters=args.segment_target_characters,
        first_segment_max_wait_s=args.first_max_wait,
        segment_max_wait_s=args.segment_max_wait,
        affinity=args.affinity, runtime_dir=runtime_dir)
    trace_path = args.trace or (runtime_dir / f'pipeline-{int(time.time())}.jsonl')
    try:
        core = build_core(config, trace_path)
    except (RuntimeError, TtsError, FileNotFoundError) as error:
        print(json.dumps({'event': 'error', 'error': f'{type(error).__name__}: {error}'},
                         ensure_ascii=False), flush=True)
        return 1

    socket_path = runtime_dir / config.socket_name
    if socket_path.exists():
        socket_path.unlink()
    server = SpeechUnixServer(str(socket_path), core)
    os.chmod(socket_path, 0o600)

    stopping = threading.Event()

    def handle_signal(signum: int, _frame: Any) -> None:
        stopping.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    print(json.dumps({'event': 'ready', 'socket': str(socket_path), 'pid': os.getpid(),
                      'candidate': config.candidate, 'sample_rate': core.backend.sample_rate,
                      'affinity': core.affinity, 'trace': str(trace_path),
                      'load_s': core.load_s}, ensure_ascii=False), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        server.server_close()
        core.close()
        if socket_path.exists():
            socket_path.unlink()
        lock.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
