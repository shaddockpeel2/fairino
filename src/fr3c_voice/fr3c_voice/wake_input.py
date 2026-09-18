"""CPU wake-word capture followed by the existing isolated RKNN ASR."""
from __future__ import annotations

import audioop
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import re
import selectors
import subprocess
import sys
import threading
import time
import unicodedata
import uuid

from .trace import TraceWriter
from .voice_input import CAPTURE_LOCK_PATH


KWS_MODEL_FILES = (
    'tokens.txt',
    'encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx',
    'decoder-epoch-12-avg-2-chunk-16-left-64.onnx',
    'joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx',
)
_SEPARATORS = r'[\s，。！？、,.!?：:；;·\-]*'
_WAKE_9527_GROUPS = (
    frozenset('9九玖久酒韭灸就旧救舅究疚咎揪纠鸠'),
    frozenset('5五伍武舞午侮吴无吾梧乌屋污物务误悟雾勿'),
    frozenset('2二贰儿而耳尔饵'),
    frozenset('7七柒妻期漆戚欺凄栖齐其奇棋旗骑祈琪起启乞企岂杞器气汽弃契泣'),
)


def _wake_pattern(phrase: str) -> str:
    compact = re.sub(
        r'[\s，。！？、,.!?：:；;·\-]+', '',
        unicodedata.normalize('NFKC', phrase).strip().casefold())
    if compact in ('9527', '九五二七'):
        return _SEPARATORS.join(
            '[' + ''.join(sorted(group)) + ']'
            for group in _WAKE_9527_GROUPS)
    return _SEPARATORS.join(re.escape(char) for char in compact)


def _is_9527_fragment(compact: str) -> bool:
    """Recognize a truncated contiguous part of the 9527 homophone pattern."""
    if not 1 <= len(compact) <= len(_WAKE_9527_GROUPS):
        return False
    for offset in range(len(_WAKE_9527_GROUPS) - len(compact) + 1):
        if all(char in _WAKE_9527_GROUPS[offset + index]
               for index, char in enumerate(compact)):
            return True
    return False


def extract_wake_query(text: str, phrases: tuple[str, ...]) -> tuple[bool, str]:
    """Return a same-utterance query after an exact, start-anchored wake word."""
    candidate = unicodedata.normalize('NFKC', str(text)).strip()
    candidate = re.sub(r'^[\s，。！？、,.!?：:；;]+', '', candidate)
    for greeting in ('你好', '嗨', '嘿', '喂'):
        if candidate.casefold().startswith(greeting.casefold()):
            candidate = re.sub(
                r'^[\s，。！？、,.!?：:；;]+', '',
                candidate[len(greeting):])
            break
    for phrase in sorted(phrases, key=len, reverse=True):
        compact = re.sub(
            r'[\s，。！？、,.!?：:；;·\-]+', '',
            unicodedata.normalize('NFKC', phrase).strip().casefold())
        if not compact:
            continue
        pattern = _wake_pattern(phrase)
        match = re.match(pattern, candidate, flags=re.IGNORECASE)
        if not match:
            continue
        query = re.sub(
            r'^[\s，。！？、,.!?：:；;]+', '',
            candidate[match.end():]).strip()
        repeated = re.match(pattern, query, flags=re.IGNORECASE)
        if repeated:
            query = re.sub(
                r'^[\s，。！？、,.!?：:；;]+', '',
                query[repeated.end():]).strip()
        return True, query
    return False, ''


def wake_query_from_asr(text: str, phrases: tuple[str, ...]) -> str:
    """Strip the wake word; reject a wake-only fragment, keep a real query."""
    matched, query = extract_wake_query(text, phrases)
    if matched:
        return query
    compact = re.sub(
        r'[\s，。！？、,.!?：:；;·\-]+', '',
        unicodedata.normalize('NFKC', str(text)).strip().casefold())
    if len(compact) < 2:
        return ''
    canonical_9527 = any(
        re.sub(r'\D+', '', phrase) == '9527' or phrase == '九五二七'
        for phrase in phrases)
    if canonical_9527 and _is_9527_fragment(compact):
        return ''
    for phrase in phrases:
        normalized = re.sub(
            r'[\s，。！？、,.!?：:；;·\-]+', '',
            unicodedata.normalize('NFKC', phrase).strip().casefold())
        if compact in normalized:
            return ''
    # CPU KWS is the authority. Some ASR results contain only the words after
    # the wake phrase because the pre-roll began mid-phoneme; retain those.
    return str(text).strip()


class WakeInput:
    """Own the microphone for either indefinite KWS or one follow-up window."""

    def __init__(self, workspace: Path, emit, before_capture=lambda: None):
        self.workspace = workspace
        self.config = json.loads(
            (workspace / 'config/voice/wake.json').read_text(encoding='utf-8'))
        self.asr = json.loads(
            (workspace / 'config/voice/asr.json').read_text(encoding='utf-8'))
        self.emit = emit
        self.before_capture = before_capture
        self.trace = TraceWriter(workspace / 'evidence/voice/P9/runtime/trace.jsonl')
        self._lock = threading.Lock()
        self._thread = None
        self._cancel = threading.Event()
        self.request_id = ''
        self.mode = ''

    @property
    def enabled_by_default(self) -> bool:
        return bool(self.config.get('enabled', True))

    @property
    def phrases(self) -> tuple[str, ...]:
        return tuple(str(value).strip() for value in self.config['phrases'])

    @property
    def active(self) -> bool:
        with self._lock:
            return bool(self._thread and self._thread.is_alive())

    def _config_path(self, value: str) -> Path:
        path = Path(value)
        return path if path.is_absolute() else self.workspace / path

    def validate(self) -> None:
        cfg = self.config
        if not self.phrases or any(not phrase for phrase in self.phrases):
            raise RuntimeError('唤醒词配置为空')
        python = self._config_path(cfg['kws_python'])
        model = self._config_path(cfg['kws_model_directory'])
        keywords = self._config_path(cfg['kws_keywords_file'])
        root = Path(self.asr['reference_root'])
        library = root / self.asr['c_api_library']
        paths = [python, keywords, library,
                 root / self.asr['runtime_directory'] / 'librknnrt.so']
        paths += [model / name for name in KWS_MODEL_FILES]
        paths += [root / self.asr['model_directory'] / name
                  for name in ('encoder.rknn', 'decoder.rknn', 'joiner.rknn', 'tokens.txt')]
        missing = [str(path) for path in paths if not path.is_file()]
        if missing:
            raise RuntimeError('唤醒/ASR 资产缺失：' + ', '.join(missing))
        if not os.access(python, os.X_OK):
            raise RuntimeError(f'KWS Python 不可执行：{python}')
        if not 20 <= int(cfg['kws_chunk_ms']) <= 500:
            raise RuntimeError('kws_chunk_ms 必须在 20..500 之间')
        if not 0.5 <= float(cfg['pre_roll_s']) <= 3.0:
            raise RuntimeError('pre_roll_s 必须在 0.5..3.0 之间')

    def start(self, mode: str = 'wake') -> str:
        if mode not in ('wake', 'followup'):
            raise ValueError(f'unsupported wake input mode: {mode}')
        with self._lock:
            if self._thread and self._thread.is_alive():
                return ''
            self._cancel.clear()
            self.request_id = uuid.uuid4().hex
            self.mode = mode
            self._thread = threading.Thread(
                target=self._run, args=(self.request_id, mode), daemon=True,
                name=f'fr3c-{mode}-input')
            self._thread.start()
            return self.request_id

    def cancel(self, wait: bool = True) -> None:
        self._cancel.set()
        thread = self._thread
        if (wait and thread and thread is not threading.current_thread()
                and thread.is_alive()):
            thread.join(timeout=2.0)

    def close(self) -> None:
        self.cancel(wait=True)

    def _event(self, kind: str, rid: str, **data) -> None:
        self.trace.write(kind, rid, **data)
        self.emit((kind, rid, data))

    @staticmethod
    def _terminate(process) -> None:
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        for stream in (process.stdin, process.stdout, process.stderr):
            if stream:
                try:
                    stream.close()
                except OSError:
                    pass

    @staticmethod
    def _json_reader(process, events: queue.Queue) -> None:
        for line in process.stdout:
            try:
                events.put(json.loads(line))
            except (ValueError, UnicodeError):
                continue
        events.put({'event': 'eof'})

    def _spawn(self, command: list[str], errors, env=None):
        process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=errors, env=env, bufsize=0)
        events = queue.Queue()
        threading.Thread(
            target=self._json_reader, args=(process, events), daemon=True).start()
        return process, events

    def _kws_command(self) -> list[str]:
        cfg = self.config
        chunk_samples = self.asr['sample_rate'] * int(cfg['kws_chunk_ms']) // 1000
        return [
            str(self._config_path(cfg['kws_python'])),
            str(Path(__file__).with_name('kws_worker.py')),
            '--model-dir', str(self._config_path(cfg['kws_model_directory'])),
            '--keywords-file', str(self._config_path(cfg['kws_keywords_file'])),
            '--sample-rate', str(self.asr['sample_rate']),
            '--num-threads', str(cfg['kws_num_threads']),
            '--score', str(cfg['kws_score']),
            '--threshold', str(cfg['kws_threshold']),
            '--num-trailing-blanks', str(cfg['kws_num_trailing_blanks']),
            '--chunk-samples', str(chunk_samples),
        ]

    def _asr_command(self) -> tuple[list[str], dict]:
        cfg = self.asr
        root = Path(cfg['reference_root'])
        library = root / cfg['c_api_library']
        env = os.environ.copy()
        env['LD_LIBRARY_PATH'] = ':'.join((
            str(root / cfg['runtime_directory']), str(library.parent),
            env.get('LD_LIBRARY_PATH', '')))
        command = [
            sys.executable, str(Path(__file__).with_name('asr_worker.py')),
            '--c-api-library', str(library),
            '--model-dir', str(root / cfg['model_directory']),
            '--hotwords-file', str(self.workspace / 'config/voice/asr_hotwords.txt'),
            '--sample-rate', str(cfg['sample_rate']),
            '--num-threads', str(cfg['num_threads']), '--chunk-samples', '800',
            '--speech-endpoint-seconds', str(cfg['trailing_silence_s']),
            '--empty-endpoint-seconds', str(self.config['followup_initial_silence_s']),
            '--maximum-utterance-seconds', str(cfg['maximum_record_s']),
        ]
        return command, env

    def _wait_ready(self, events: queue.Queue, label: str) -> dict:
        deadline = time.monotonic() + 15.0
        while not self._cancel.is_set():
            if time.monotonic() >= deadline:
                raise RuntimeError(f'{label} 加载超时')
            try:
                event = events.get(timeout=0.05)
            except queue.Empty:
                continue
            if event.get('event') == 'ready':
                return event
            if event.get('event') in ('error', 'eof'):
                raise RuntimeError(event.get('message', f'{label} 启动失败'))
        return {}

    def _open_capture(self, errors):
        cfg = self.asr
        return subprocess.Popen(
            ['arecord', '-q', '-D', cfg['capture_device'], '-t', 'raw',
             '-f', 'S16_LE', '-r', str(cfg['sample_rate']), '-c', '1'],
            stdout=subprocess.PIPE, stderr=errors, bufsize=0)

    def _wait_for_wake(self, rid: str, capture, kws, events) -> bytes:
        cfg = self.config
        sample_rate = int(self.asr['sample_rate'])
        chunk_bytes = sample_rate * int(cfg['kws_chunk_ms']) // 1000 * 2
        ring = deque(maxlen=max(1, math.ceil(
            float(cfg['pre_roll_s']) * 1000 / int(cfg['kws_chunk_ms']))))
        pending = bytearray()
        os.set_blocking(capture.stdout.fileno(), False)
        os.set_blocking(kws.stdin.fileno(), False)
        self._event('wake_waiting', rid, phrases=list(self.phrases))
        with selectors.DefaultSelector() as selector:
            selector.register(capture.stdout, selectors.EVENT_READ)
            while not self._cancel.is_set():
                while True:
                    try:
                        event = events.get_nowait()
                    except queue.Empty:
                        break
                    kind = event.get('event')
                    if kind == 'wake':
                        self._event(
                            'wake_detected', rid, keyword=event.get('keyword', ''))
                        return b''.join(ring)
                    if kind in ('error', 'eof'):
                        raise RuntimeError(event.get('message', 'KWS 意外退出'))
                for key, _mask in selector.select(timeout=0.05):
                    raw = os.read(key.fd, chunk_bytes)
                    if not raw:
                        raise RuntimeError('麦克风断开或不可用')
                    if len(raw) % 2:
                        raw = raw[:-1]
                    if raw:
                        ring.append(raw)
                        pending.extend(raw)
                if pending:
                    try:
                        count = os.write(kws.stdin.fileno(), pending)
                        del pending[:count]
                    except BlockingIOError:
                        pass
                    except BrokenPipeError:
                        pass
                if len(pending) > sample_rate * 2:
                    raise RuntimeError('KWS 跟不上实时输入')
        return b''

    def _stream_asr(self, rid: str, mode: str, capture, worker,
                    events: queue.Queue, initial_audio: bytes = b''):
        cfg = self.asr
        sample_rate = int(cfg['sample_rate'])
        chunk_bytes = 1600
        pending = bytearray(initial_audio)
        total = len(initial_audio) / (sample_rate * 2)
        voiced = 0.0
        last_voice = None
        recognized = False
        started = time.monotonic()
        closed = False
        reason = ''
        final_deadline = None

        def inspect(raw: bytes) -> None:
            nonlocal voiced, last_voice
            if not raw:
                return
            # Inspect short frames. A single RMS over the two-second pre-roll
            # would dilute the wake phrase with preceding silence and could
            # incorrectly leave ``last_voice`` unset.
            frame_bytes = sample_rate // 20 * 2  # 50 ms
            for offset in range(0, len(raw), frame_bytes):
                frame = raw[offset:offset + frame_bytes]
                frame = frame[:len(frame) // 2 * 2]
                if not frame:
                    continue
                rms = audioop.rms(frame, 2)
                db = 20 * math.log10(max(rms, 1) / 32768)
                if db >= float(cfg['speech_threshold_dbfs']):
                    voiced += len(frame) / (sample_rate * 2)
                    last_voice = time.monotonic()

        inspect(initial_audio)
        os.set_blocking(worker.stdin.fileno(), False)
        os.set_blocking(capture.stdout.fileno(), False)
        event_name = 'wake_asr_recording' if mode == 'wake' else 'wake_followup_listening'
        self._event(event_name, rid)
        with selectors.DefaultSelector() as selector:
            selector.register(capture.stdout, selectors.EVENT_READ)
            while not self._cancel.is_set():
                now = time.monotonic()
                while True:
                    try:
                        event = events.get_nowait()
                    except queue.Empty:
                        break
                    kind = event.get('event')
                    if kind == 'partial':
                        partial = str(event.get('text', '')).strip()
                        if partial:
                            recognized = True
                        self._event('wake_partial', rid, text=partial)
                    elif kind == 'final':
                        text = str(event.get('text', '')).strip()
                        data = dict(
                            text=text, mode=mode,
                            reason=reason or event.get('reason', ''),
                            audio_s=total, voiced_s=voiced,
                            speech_end_monotonic_ns=(
                                int(last_voice * 1e9) if last_voice else None))
                        return ('wake_final' if text else 'wake_empty'), data
                    elif kind in ('error', 'eof'):
                        raise RuntimeError(event.get('message', 'ASR 未返回定稿'))
                if not closed:
                    if now - started >= float(cfg['maximum_record_s']):
                        reason = 'maximum_duration'
                    elif (mode == 'followup'
                          and not recognized
                          and now - started >= float(
                              self.config['followup_initial_silence_s'])):
                        reason = 'no_speech'
                    elif (last_voice is not None
                          and (mode == 'wake' or recognized)
                          and now - last_voice >= float(cfg['trailing_silence_s'])):
                        reason = 'silence'
                    if reason:
                        self.trace.write('wake_input_end', rid, mode=mode, reason=reason)
                        self._terminate(capture)
                        capture = None
                        closed = True
                        final_deadline = now + 5.0
                    else:
                        for key, _mask in selector.select(timeout=0.01):
                            raw = os.read(key.fd, chunk_bytes)
                            if not raw:
                                raise RuntimeError('麦克风断开或不可用')
                            if len(raw) % 2:
                                raw = raw[:-1]
                            pending.extend(raw)
                            total += len(raw) / (sample_rate * 2)
                            inspect(raw)
                if pending:
                    try:
                        count = os.write(worker.stdin.fileno(), pending)
                        del pending[:count]
                    except BlockingIOError:
                        pass
                    except BrokenPipeError:
                        pending.clear()
                        if not closed:
                            reason = 'sherpa_endpoint'
                            self._terminate(capture)
                            capture = None
                            closed = True
                            final_deadline = now + 5.0
                max_pending = int((float(self.config['pre_roll_s']) + 1.0)
                                  * sample_rate * 2)
                if len(pending) > max_pending:
                    raise RuntimeError('ASR 跟不上实时输入')
                if closed:
                    if not pending and not worker.stdin.closed:
                        worker.stdin.close()
                    if now > final_deadline:
                        raise RuntimeError('ASR 定稿超时')
                    time.sleep(0.01)
        return 'wake_empty', {'text': '', 'mode': mode, 'reason': 'canceled'}

    def _run(self, rid: str, mode: str) -> None:
        capture = kws = worker = None
        terminal = ('wake_empty', {'text': '', 'mode': mode, 'reason': 'canceled'})
        runtime = self.workspace / 'evidence/voice/P9/runtime'
        runtime.mkdir(parents=True, exist_ok=True)
        lock = open(CAPTURE_LOCK_PATH, 'a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.validate()
            self.before_capture()
            if self._cancel.is_set():
                return
            with (runtime / 'worker.log').open('ab') as errors:
                initial_audio = b''
                if mode == 'wake':
                    kws, kws_events = self._spawn(self._kws_command(), errors)
                    self._wait_ready(kws_events, 'KWS')
                    capture = self._open_capture(errors)
                    initial_audio = self._wait_for_wake(
                        rid, capture, kws, kws_events)
                    if self._cancel.is_set():
                        return
                    self._terminate(kws)
                    kws = None
                command, env = self._asr_command()
                worker, asr_events = self._spawn(command, errors, env=env)
                ready = self._wait_ready(asr_events, 'ASR')
                self.trace.write(
                    'wake_asr_ready', rid, mode=mode,
                    load_seconds=ready.get('load_seconds'))
                if mode == 'followup':
                    capture = self._open_capture(errors)
                terminal = self._stream_asr(
                    rid, mode, capture, worker, asr_events, initial_audio)
                capture = None
        except Exception as error:
            terminal = ('wake_error', {'message': str(error), 'mode': mode})
        finally:
            self._terminate(capture)
            self._terminate(kws)
            self._terminate(worker)
            lock.close()
            with self._lock:
                if self.request_id == rid:
                    self.mode = ''
            if not self._cancel.is_set():
                self._event(terminal[0], rid, **terminal[1])
