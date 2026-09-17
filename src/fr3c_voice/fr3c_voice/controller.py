"""Own exactly one Qwen worker and serialize all model operations."""

from __future__ import annotations

import fcntl
import os
import queue
import shutil
import signal
import subprocess
import threading
import time
import uuid
from enum import Enum
from pathlib import Path
from typing import Callable, Optional

from .config import QwenConfig
from .events import AgentEvent, ChatResult
from .protocol import (
    EVENT_DONE,
    EVENT_TOKEN,
    RequestCancelled,
    RequestTimedOut,
    WorkerClient,
    WorkerProtocolError,
    WorkerUnavailable,
)
from .trace import TraceWriter


class ControllerError(RuntimeError):
    pass


class ControllerState(str, Enum):
    STOPPED = 'stopped'
    LOADING = 'loading'
    READY = 'ready'
    BUSY = 'busy'
    FAULT = 'fault'
    CLOSED = 'closed'


class QwenController:
    def __init__(
        self,
        config: QwenConfig,
        runtime_dir: Path | str,
        trace: TraceWriter | None = None,
        process_factory: Callable[..., subprocess.Popen] = subprocess.Popen,
    ) -> None:
        self.config = config
        self.runtime_dir = Path(runtime_dir)
        self.socket_path = self.runtime_dir / config.socket_name
        self.pid_path = self.socket_path.with_suffix(self.socket_path.suffix + '.pid')
        self.image_dir = self.runtime_dir / 'images'
        self.events: queue.Queue[AgentEvent] = queue.Queue()
        self.trace = trace or TraceWriter(None)
        self._process_factory = process_factory
        self._process: Optional[subprocess.Popen] = None
        self._log_handle = None
        self._instance_lock_handle = None
        self._state = ControllerState.STOPPED
        self._lock = threading.RLock()
        self._request_lock = threading.Lock()
        self._closed = False
        self._cancel_event: threading.Event | None = None
        self._active_request_id = ''
        self.last_error = ''

    @property
    def state(self) -> ControllerState:
        with self._lock:
            self._refresh_process_locked()
            return self._state

    @property
    def worker_pid(self) -> Optional[int]:
        with self._lock:
            return (
                self._process.pid
                if self._process is not None and self._process.poll() is None
                else None
            )

    @property
    def active_request_id(self) -> str:
        with self._lock:
            return self._active_request_id

    def _publish(self, kind: str, message: str = '', request_id: str = '',
                 elapsed_s=None) -> None:
        self.events.put(AgentEvent(
            kind, self._state.value, message, request_id, elapsed_s))

    def _set_state(self, state: ControllerState, message: str = '') -> None:
        self._state = state
        self._publish('state', message)

    def _validate_assets(self) -> None:
        for label, path in (
            ('worker', self.config.worker_path),
            ('vision model', self.config.vision_path),
            ('language model', self.config.language_path),
            ('RKLLM runtime', self.config.rkllm_runtime_path),
            ('RKNN runtime', self.config.rknn_runtime_path),
        ):
            if not path.is_file():
                raise ControllerError(f'missing {label}: {path}')

    def _command(self) -> list[str]:
        worker = str(self.config.worker_path)
        if os.access(worker, os.X_OK):
            prefix = [worker]
        else:
            loader = Path('/usr/lib/ld-linux-aarch64.so.1')
            if not loader.is_file():
                raise ControllerError(
                    'worker is not executable and the aarch64 loader is unavailable')
            prefix = [str(loader), worker]
        return prefix + [
            str(self.config.vision_path), str(self.config.language_path),
            str(self.socket_path), str(self.config.max_new_tokens),
            str(self.config.max_context_len), str(self.config.npu_core_num),
            self.config.platform, '0',
        ]

    def _acquire_instance_lock(self) -> None:
        if self._instance_lock_handle is not None:
            return
        lock_path = self.runtime_dir / 'qwen-instance.lock'
        handle = lock_path.open('a+', encoding='ascii')
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            handle.close()
            raise ControllerError(
                f'another FR3C Qwen service owns {lock_path}') from error
        handle.seek(0)
        handle.truncate()
        handle.write(f'{os.getpid()}\n')
        handle.flush()
        self._instance_lock_handle = handle

    def _foreign_worker_pids(self) -> list[int]:
        expected = str(self.config.worker_path.resolve())
        own_pid = self._process.pid if self._process is not None else None
        matches = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            if pid in (os.getpid(), own_pid):
                continue
            try:
                arguments = (entry / 'cmdline').read_bytes().split(b'\0')
                decoded = [item.decode('utf-8', errors='replace') for item in arguments]
            except (OSError, PermissionError):
                continue
            if expected in decoded:
                matches.append(pid)
        return matches

    def _release_instance_lock(self) -> None:
        handle = self._instance_lock_handle
        self._instance_lock_handle = None
        if handle is not None:
            try:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            finally:
                handle.close()

    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise ControllerError('controller is closed')
            self._refresh_process_locked()
            if self._state == ControllerState.READY:
                return
            if self._process is not None:
                raise ControllerError('worker is already active')
            self._validate_assets()
            self.runtime_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
            os.chmod(self.runtime_dir, 0o700)
            self.image_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
            self._acquire_instance_lock()
            foreign_workers = self._foreign_worker_pids()
            if foreign_workers:
                self._release_instance_lock()
                raise ControllerError(
                    'another Qwen worker is already loaded: pids='
                    + ','.join(str(pid) for pid in foreign_workers))
            self.socket_path.unlink(missing_ok=True)
            self.pid_path.unlink(missing_ok=True)
            self._set_state(ControllerState.LOADING)
            self.trace.write('model_load_begin', component='qwen')
            env = os.environ.copy()
            runtime_dirs = {
                str(self.config.rkllm_runtime_path.parent),
                str(self.config.rknn_runtime_path.parent),
            }
            old_library_path = env.get('LD_LIBRARY_PATH', '')
            env['LD_LIBRARY_PATH'] = ':'.join(
                sorted(runtime_dirs) + ([old_library_path] if old_library_path else []))
            try:
                self._log_handle = (self.runtime_dir / 'worker.log').open(
                    'ab', buffering=0)
                self._process = self._process_factory(
                    self._command(), env=env, start_new_session=True,
                    stdout=self._log_handle, stderr=subprocess.STDOUT,
                )
                self.pid_path.write_text(str(self._process.pid), encoding='ascii')
            except (OSError, subprocess.SubprocessError, ControllerError) as error:
                self._process = None
                self._close_log_locked()
                self._release_instance_lock()
                self.last_error = f'unable to start Qwen worker: {error}'
                self._set_state(ControllerState.FAULT, self.last_error)
                raise ControllerError(self.last_error) from error

        deadline = time.monotonic() + self.config.startup_timeout_s
        client = WorkerClient(
            self.socket_path, min(1.0, self.config.request_timeout_s),
            self.config.max_prompt_bytes)
        while time.monotonic() < deadline:
            with self._lock:
                if self._process is None or self._process.poll() is not None:
                    code = self._process.returncode if self._process is not None else 'unknown'
                    detail = self._worker_log_tail()
                    self.last_error = f'Qwen worker exited during startup (code {code})'
                    if detail:
                        self.last_error += f': {detail}'
                    self._terminate_locked()
                    self._set_state(ControllerState.FAULT, self.last_error)
                    raise ControllerError(self.last_error)
            if self.socket_path.exists() and client.ping():
                with self._lock:
                    self.last_error = ''
                    self._set_state(ControllerState.READY)
                    self.trace.write(
                        'model_ready', component='qwen', worker_pid=self.worker_pid)
                return
            time.sleep(0.1)
        self._fault_and_stop('Qwen worker startup timed out')
        raise ControllerError(self.last_error)

    def _refresh_process_locked(self) -> None:
        if self._process is not None and self._process.poll() is not None:
            code = self._process.returncode
            self._process = None
            self._close_log_locked()
            self._release_instance_lock()
            if self._state not in (ControllerState.STOPPED, ControllerState.CLOSED):
                self.last_error = f'Qwen worker exited unexpectedly (code {code})'
                self._set_state(ControllerState.FAULT, self.last_error)
                self.trace.write('worker_exit', error=self.last_error)

    def health(self) -> bool:
        with self._lock:
            self._refresh_process_locked()
            if self._state != ControllerState.READY:
                return False
        return WorkerClient(
            self.socket_path, 1.0, self.config.max_prompt_bytes).ping()

    def _managed_image(self, source: Path | str) -> Path:
        path = Path(source)
        try:
            size = path.stat().st_size
        except OSError as error:
            raise ControllerError(f'image is not readable: {path}') from error
        if size <= 0 or size > self.config.max_image_bytes:
            raise ControllerError('image exceeds the configured size limit or is empty')
        try:
            with path.open('rb') as handle:
                header = handle.read(16)
        except OSError as error:
            raise ControllerError(f'image is not readable: {path}') from error
        suffix = '.jpg'
        if header.startswith(b'\x89PNG\r\n\x1a\n'):
            suffix = '.png'
        elif header.startswith(b'RIFF') and header[8:12] == b'WEBP':
            suffix = '.webp'
        elif not header.startswith(b'\xff\xd8\xff'):
            raise ControllerError('image format is invalid or unsupported')
        destination = self.image_dir / f'request-{uuid.uuid4().hex}{suffix}'
        shutil.copyfile(path, destination)
        return destination

    def chat_text(self, prompt: str, request_id: str = '',
                  on_token: Callable[[str], None] | None = None) -> ChatResult:
        return self._chat(prompt, None, request_id, on_token)

    def chat_image(self, prompt: str, image_path: Path | str,
                   request_id: str = '',
                   on_token: Callable[[str], None] | None = None) -> ChatResult:
        managed = self._managed_image(image_path)
        try:
            return self._chat(prompt, managed, request_id, on_token)
        finally:
            managed.unlink(missing_ok=True)

    def _chat(self, prompt: str, image_path: Optional[Path], request_id: str,
              on_token: Callable[[str], None] | None) -> ChatResult:
        request_id = request_id or uuid.uuid4().hex
        encoded = prompt.encode('utf-8')
        if len(encoded) > self.config.max_prompt_bytes:
            raise ControllerError('prompt exceeds the configured byte limit')
        if not prompt.strip():
            raise ControllerError('prompt must not be empty')
        self.trace.write('queue_enter', request_id, component='qwen')
        if not self._request_lock.acquire(blocking=False):
            self.trace.write('request_rejected_busy', request_id, component='qwen')
            raise ControllerError('another Qwen request is already running')
        self.trace.write('queue_leave', request_id, component='qwen')
        started = time.monotonic()
        first_token_s = None
        parts: list[str] = []
        cancel_event = threading.Event()
        try:
            with self._lock:
                self._refresh_process_locked()
                if self._state != ControllerState.READY:
                    raise ControllerError(
                        f'Qwen controller is not ready ({self._state.value})')
                self._cancel_event = cancel_event
                self._active_request_id = request_id
                self._set_state(ControllerState.BUSY)
                self._publish('request_started', request_id=request_id)
            client = WorkerClient(
                self.socket_path, self.config.request_timeout_s,
                self.config.max_prompt_bytes)
            for event, body in client.chat(prompt, image_path, cancel_event):
                if event == EVENT_TOKEN:
                    elapsed = time.monotonic() - started
                    if first_token_s is None:
                        first_token_s = elapsed
                        self.trace.write(
                            'qwen_first_token', request_id,
                            component='qwen', elapsed_s=elapsed)
                    token = body.decode('utf-8', errors='replace')
                    parts.append(token)
                    if on_token is not None:
                        on_token(token)
                    self._publish('token', token, request_id, elapsed)
                elif event == EVENT_DONE:
                    break
            total_s = time.monotonic() - started
            result = ChatResult(
                request_id, ''.join(parts).strip(), first_token_s, total_s)
            with self._lock:
                self._set_state(ControllerState.READY)
                self._publish('request_done', result.text, request_id, total_s)
                self.trace.write(
                    'generation_end', request_id, component='qwen',
                    elapsed_s=total_s, output_chars=len(result.text))
            return result
        except RequestCancelled as error:
            self._fault_and_stop(str(error), request_id, event='request_cancelled')
            raise ControllerError(str(error)) from error
        except RequestTimedOut as error:
            self._fault_and_stop(str(error), request_id, event='request_timeout')
            raise ControllerError(str(error)) from error
        except (WorkerUnavailable, WorkerProtocolError, OSError) as error:
            self._fault_and_stop(f'Qwen request failed: {error}', request_id)
            raise ControllerError(self.last_error) from error
        finally:
            with self._lock:
                self._cancel_event = None
                self._active_request_id = ''
            self._request_lock.release()

    def cancel_current(self, request_id: str = '') -> bool:
        with self._lock:
            event = self._cancel_event
            active_id = self._active_request_id
            if event is None or (request_id and request_id != active_id):
                return False
            event.set()
            self._publish('cancel_requested', request_id=active_id)
            self.trace.write('cancel_requested', active_id, component='qwen')
            return True

    def reset_context(self) -> None:
        if not self._request_lock.acquire(blocking=False):
            raise ControllerError('cannot reset while a request is running')
        try:
            with self._lock:
                self._refresh_process_locked()
                if self._state != ControllerState.READY:
                    raise ControllerError('Qwen controller is not ready')
            try:
                WorkerClient(
                    self.socket_path, self.config.request_timeout_s,
                    self.config.max_prompt_bytes).reset()
            except (WorkerUnavailable, WorkerProtocolError, OSError) as error:
                self._fault_and_stop(f'Qwen reset failed: {error}')
                raise ControllerError(self.last_error) from error
            self._publish('context_reset')
        finally:
            self._request_lock.release()

    def recover(self) -> None:
        if self._request_lock.locked():
            raise ControllerError('cannot recover while a request is running')
        self.stop()
        self.start()

    def _terminate_locked(self) -> None:
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=self.config.terminate_grace_s)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=2.0)
        self.socket_path.unlink(missing_ok=True)
        self.pid_path.unlink(missing_ok=True)
        self._close_log_locked()
        self._release_instance_lock()

    def _close_log_locked(self) -> None:
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None

    def _worker_log_tail(self) -> str:
        path = self.runtime_dir / 'worker.log'
        try:
            lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
        except OSError:
            return ''
        return ' | '.join(lines[-3:])[-800:]

    def _fault_and_stop(self, message: str, request_id: str = '',
                        event: str = 'request_error') -> None:
        with self._lock:
            self.last_error = message
            self._terminate_locked()
            self._set_state(ControllerState.FAULT, message)
            self._publish('error', message, request_id)
            self.trace.write(event, request_id, component='qwen', error=message)

    def stop(self) -> None:
        with self._lock:
            self._terminate_locked()
            if not self._closed:
                self.last_error = ''
                self._set_state(ControllerState.STOPPED)

    def close(self) -> None:
        with self._lock:
            self._terminate_locked()
            self._closed = True
            self._set_state(ControllerState.CLOSED)

    def __enter__(self) -> 'QwenController':
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.close()
