"""Button-scoped, isolated RKNN ASR; no ROS or speaker ownership here."""
from __future__ import annotations

import audioop
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import selectors
import subprocess
import sys
import threading
import time
import uuid
import wave

from .trace import TraceWriter
from .asr_text import normalize_robot_asr_text


CAPTURE_LOCK_PATH = '/tmp/fr3c-button-asr.lock'


class VoiceInput:
    def __init__(self, workspace: Path, emit, before_capture=lambda: None):
        self.workspace = workspace
        self.config = json.loads((workspace / 'config/voice/asr.json').read_text())
        self.emit = emit
        self.before_capture = before_capture
        self.trace = TraceWriter(workspace / 'evidence/voice/P7/runtime/trace.jsonl')
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._cancel = threading.Event()
        self.request_id = ''

    def validate(self):
        root = Path(self.config['reference_root'])
        paths = [root / self.config['c_api_library'],
                 root / self.config['runtime_directory'] / 'librknnrt.so']
        paths += [root / self.config['model_directory'] / name
                  for name in ('encoder.rknn', 'decoder.rknn', 'joiner.rknn', 'tokens.txt')]
        missing = [str(p) for p in paths if not p.is_file()]
        if missing:
            raise RuntimeError('ASR 资产缺失：' + ', '.join(missing))

    def start(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return ''
            self._stop.clear()
            self._cancel.clear()
            self.request_id = uuid.uuid4().hex
            self._thread = threading.Thread(target=self._run, args=(self.request_id,), daemon=True)
            self._thread.start()
            return self.request_id

    def finish(self):
        self._stop.set()

    def close(self):
        self._cancel.set()
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _event(self, kind, rid, **data):
        self.trace.write(kind, rid, **data)
        self.emit((kind, rid, data))

    @staticmethod
    def _terminate(process):
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1)
            for stream in (process.stdin, process.stdout, process.stderr):
                if stream:
                    stream.close()

    def _run(self, rid):
        worker = capture = None
        debug_audio = debug_path = None
        terminal = ('asr_empty', {})
        cfg = self.config
        runtime = self.workspace / 'evidence/voice/P7/runtime'
        runtime.mkdir(parents=True, exist_ok=True)
        lock = open(CAPTURE_LOCK_PATH, 'a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.validate()
            self.before_capture()  # strict stop ACK, never record through playback
            if self._cancel.is_set() or self._stop.is_set():
                return
            root = Path(cfg['reference_root'])
            library = root / cfg['c_api_library']
            env = os.environ.copy()
            env['LD_LIBRARY_PATH'] = ':'.join((str(root / cfg['runtime_directory']),
                                              str(library.parent), env.get('LD_LIBRARY_PATH', '')))
            command = [sys.executable, str(Path(__file__).with_name('asr_worker.py')),
                       '--c-api-library', str(library), '--model-dir', str(root / cfg['model_directory']),
                       '--sample-rate', str(cfg['sample_rate']),
                       '--num-threads', str(cfg['num_threads']), '--chunk-samples', '800',
                       '--speech-endpoint-seconds', str(cfg['trailing_silence_s']),
                       '--empty-endpoint-seconds', str(cfg['initial_silence_s']),
                       '--maximum-utterance-seconds', str(cfg['maximum_record_s'])]
            events = queue.Queue()
            with (runtime / 'worker.log').open('ab') as errors:
                worker = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                          stderr=errors, env=env, bufsize=0)
                def reader():
                    for line in worker.stdout:
                        try:
                            value = json.loads(line)
                        except (ValueError, UnicodeError):
                            continue
                        events.put(value)
                    events.put({'event': 'eof'})
                threading.Thread(target=reader, daemon=True).start()
                deadline = time.monotonic() + 15
                while True:
                    if self._cancel.is_set() or self._stop.is_set():
                        return
                    if time.monotonic() > deadline:
                        raise RuntimeError('ASR 加载超时')
                    try:
                        event = events.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    if event['event'] == 'ready':
                        self.trace.write('asr_ready', rid, load_seconds=event.get('load_seconds'))
                        break
                    if event['event'] in ('error', 'eof'):
                        raise RuntimeError(event.get('message', 'ASR 启动失败，见 worker.log'))
                capture = subprocess.Popen(
                    ['arecord', '-q', '-D', cfg['capture_device'], '-t', 'raw',
                     '-f', 'S16_LE', '-r', str(cfg['sample_rate']), '-c', '1'],
                    stdout=subprocess.PIPE, stderr=errors, bufsize=0)
                if cfg.get('save_debug_audio', False):
                    try:
                        directory = self.workspace / 'evidence/voice/P9/runtime/asr-recordings'
                        directory.mkdir(parents=True, exist_ok=True)
                        debug_path = directory / f'{rid}-button.wav'
                        debug_audio = wave.open(str(debug_path), 'wb')
                        debug_audio.setnchannels(1)
                        debug_audio.setsampwidth(2)
                        debug_audio.setframerate(cfg['sample_rate'])
                    except (OSError, wave.Error) as error:
                        self.trace.write('button_audio_save_error', rid, message=str(error))
                        debug_audio = debug_path = None
                os.set_blocking(worker.stdin.fileno(), False)
                os.set_blocking(capture.stdout.fileno(), False)
                self._event('asr_recording', rid)
                started = time.monotonic()
                total = voiced = 0.0
                last_voice = None
                pending = bytearray()
                closed = False
                final_deadline = None
                reason = ''
                with selectors.DefaultSelector() as selector:
                    selector.register(capture.stdout, selectors.EVENT_READ)
                    while not self._cancel.is_set():
                        now = time.monotonic()
                        while True:
                            try:
                                event = events.get_nowait()
                            except queue.Empty:
                                break
                            kind = event['event']
                            if kind == 'partial':
                                self._event('asr_partial', rid, text=event.get('text', ''))
                            elif kind == 'final':
                                raw_text = event.get('text', '').strip()
                                text = normalize_robot_asr_text(raw_text)
                                if not closed:
                                    self.trace.write('input_end', rid, reason='sherpa_endpoint')
                                data = dict(text=text, reason=reason or event.get('reason'),
                                            audio_s=total, voiced_s=voiced,
                                            speech_end_monotonic_ns=int(last_voice * 1e9) if last_voice else None)
                                if text != raw_text:
                                    data['raw_text'] = raw_text
                                terminal = ('asr_final' if text and total >= cfg['minimum_record_s']
                                            and voiced >= cfg['minimum_speech_s'] else 'asr_empty', data)
                                return
                            elif kind in ('error', 'eof'):
                                raise RuntimeError(event.get('message', 'ASR 未返回定稿'))
                        if not closed:
                            if self._stop.is_set():
                                reason = 'button'
                            elif now - started >= cfg['maximum_record_s']:
                                reason = 'maximum_duration'
                            elif voiced < cfg['minimum_speech_s'] and now - started >= cfg['initial_silence_s']:
                                reason = 'no_speech'
                            elif last_voice and voiced >= cfg['minimum_speech_s'] and now - last_voice >= cfg['trailing_silence_s']:
                                reason = 'silence'
                            if reason:
                                self.trace.write('input_end', rid, reason=reason)
                                self._event('asr_finalizing', rid)
                                self._terminate(capture)
                                capture = None
                                closed = True
                                final_deadline = now + 5
                            else:
                                for key, _ in selector.select(timeout=0.01):
                                    raw = os.read(key.fd, 1600)
                                    if not raw:
                                        raise RuntimeError('麦克风断开或不可用，见 worker.log')
                                    if debug_audio is not None:
                                        try:
                                            debug_audio.writeframes(raw)
                                        except (OSError, wave.Error) as error:
                                            self.trace.write('button_audio_save_error', rid, message=str(error))
                                            debug_audio.close()
                                            debug_audio = debug_path = None
                                    pending.extend(raw)
                                    duration = len(raw) / (cfg['sample_rate'] * 2)
                                    total += duration
                                    rms = audioop.rms(raw[:len(raw)//2*2], 2)
                                    db = 20 * math.log10(max(rms, 1) / 32768)
                                    if db >= cfg['speech_threshold_dbfs']:
                                        voiced += duration
                                        last_voice = time.monotonic()
                        if pending:
                            try:
                                count = os.write(worker.stdin.fileno(), pending)
                                del pending[:count]
                            except BlockingIOError:
                                pass
                            except BrokenPipeError:
                                # Native endpoint may close stdin just before its
                                # final JSON reaches the reader thread. Drain that
                                # event instead of discarding a valid utterance.
                                pending.clear()
                                if not closed:
                                    reason = 'sherpa_endpoint'
                                    self.trace.write('input_end', rid, reason=reason)
                                    self._terminate(capture)
                                    capture = None
                                    closed = True
                                    final_deadline = now + 5
                        if len(pending) > cfg['sample_rate'] * 2:
                            raise RuntimeError('ASR 跟不上实时输入，请重试')
                        if closed:
                            if not pending and not worker.stdin.closed:
                                worker.stdin.close()
                            if now > final_deadline:
                                raise RuntimeError('ASR 定稿超时，请重说')
                            time.sleep(0.01)
        except Exception as error:
            terminal = ('asr_error', {'message': str(error)})
        finally:
            self._terminate(capture)
            self._terminate(worker)
            if debug_audio is not None:
                try:
                    debug_audio.close()
                    self.trace.write('button_audio_saved', rid, path=str(debug_path))
                except OSError as error:
                    self.trace.write('button_audio_save_error', rid, message=str(error))
            lock.close()
            if not self._cancel.is_set():
                # Deliver final only after the microphone and NPU worker exit.
                self._event(terminal[0], rid, **terminal[1])
