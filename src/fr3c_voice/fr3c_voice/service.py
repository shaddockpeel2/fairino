"""Local Unix-socket Qwen service used by the future dialogue UI."""

from __future__ import annotations

import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import socketserver
import threading
import time
import uuid
from typing import Any, Callable

from .config import load_config
from .controller import ControllerState, QwenController
from .coordinator import NpuCoordinator, NpuOwner
from .knowledge import (
    KnowledgeRuntime,
    build_knowledge_prompt,
    load_knowledge_settings,
    result_payload,
    unavailable_metadata,
)
from .trace import TraceWriter


MAX_SERVICE_REQUEST_BYTES = 1024 * 1024


class QwenServiceCore:
    def __init__(self, controller: QwenController,
                 coordinator: NpuCoordinator, auto_recover: bool,
                 knowledge: KnowledgeRuntime | None = None,
                 knowledge_error: str = '') -> None:
        self.controller = controller
        self.coordinator = coordinator
        self.auto_recover = auto_recover
        self.knowledge = knowledge
        self.knowledge_error = knowledge_error

    def ensure_ready(self, request_id: str) -> None:
        state = self.controller.state
        if state == ControllerState.READY and self.controller.health():
            return
        if state == ControllerState.CLOSED:
            raise RuntimeError('Qwen service is closed')
        if state == ControllerState.READY:
            self.controller.stop()
        if not self.auto_recover and state == ControllerState.FAULT:
            raise RuntimeError(
                'Qwen worker is in fault; send an explicit recover request')
        self.coordinator.run(
            NpuOwner.QWEN_LOAD, request_id, self.controller.recover)

    def chat(self, request: dict[str, Any],
             on_token: Callable[[str], None]) -> tuple[Any, dict[str, Any] | None]:
        request_id = str(request.get('request_id') or uuid.uuid4().hex)
        self.ensure_ready(f'{request_id}-load')
        operation = str(request.get('op'))
        prompt = str(request.get('prompt') or '')
        question = str(request.get('question') or '').strip()
        use_knowledge = bool(
            operation == 'chat_auto'
            and question
            and self.knowledge is not None
            and self.knowledge.available
            and self.knowledge.wants_retrieval(question))
        knowledge_metadata: dict[str, Any] | None = None

        def invoke():
            nonlocal prompt, knowledge_metadata
            if operation == 'chat_auto' and use_knowledge:
                try:
                    retrieval = self.knowledge.retrieve(question)
                    prompt = build_knowledge_prompt(question, retrieval)
                    knowledge_metadata = retrieval.metadata()
                    self.controller.trace.write(
                        'knowledge_retrieval_end', request_id,
                        mode=knowledge_metadata['mode'],
                        version=knowledge_metadata['version'],
                        elapsed_ms=knowledge_metadata['elapsed_ms'],
                        source_count=len(knowledge_metadata['sources']))
                except Exception as error:  # knowledge failure must not break chat
                    self.knowledge.disable(error)
                    self.knowledge_error = f'{type(error).__name__}: {error}'
                    knowledge_metadata = unavailable_metadata(self.knowledge_error)
                    self.controller.trace.write(
                        'knowledge_disabled', request_id, error=self.knowledge_error)
            elif operation == 'chat_auto':
                if self.knowledge is None:
                    knowledge_metadata = unavailable_metadata(
                        self.knowledge_error or 'knowledge runtime unavailable',
                        routed=False)
                elif not self.knowledge.available:
                    knowledge_metadata = unavailable_metadata(
                        self.knowledge.last_error or self.knowledge_error,
                        routed=False)
                else:
                    knowledge_metadata = {
                        'routed': False, 'mode': 'skipped',
                        'version': self.knowledge.store.version,
                        'elapsed_ms': 0.0, 'reason': 'ordinary_chat',
                        'sources': [], 'error': '',
                    }
            if bool(request.get('reset_before', True)):
                self.controller.reset_context()
            if operation in ('chat_text', 'chat_auto'):
                return self.controller.chat_text(
                    prompt, request_id=request_id, on_token=on_token)
            if operation == 'chat_image':
                image_path = request.get('image_path')
                if not isinstance(image_path, str) or not image_path:
                    raise ValueError('image_path is required')
                return self.controller.chat_image(
                    prompt, image_path, request_id=request_id,
                    on_token=on_token)
            raise ValueError(f'unsupported chat operation: {operation}')

        owner = (NpuOwner.QWEN_VISION if operation == 'chat_image'
                 else NpuOwner.RAG_QWEN_TEXT if use_knowledge
                 else NpuOwner.QWEN_TEXT)
        chat = self.coordinator.run(owner, request_id, invoke)
        return chat, knowledge_metadata

    def recover(self, request_id: str) -> None:
        if self.coordinator.fault:
            self.coordinator.reset_fault()
        self.coordinator.run(
            NpuOwner.QWEN_LOAD, request_id, self.controller.recover)

    def health(self) -> dict[str, Any]:
        if self.knowledge is not None:
            knowledge = self.knowledge.health()
        else:
            knowledge = {
                'available': False, 'version': '', 'chunks': 0,
                'embedder_loaded': False,
                'last_error': self.knowledge_error or 'not configured',
                'index_dir': '',
            }
        return {
            'state': self.controller.state.value,
            'healthy': self.controller.health(),
            'worker_pid': self.controller.worker_pid,
            'active_request_id': self.controller.active_request_id,
            'npu_owner': self.coordinator.owner,
            'coordination_fault': self.coordinator.fault,
            'last_error': self.controller.last_error,
            'knowledge': knowledge,
        }


class QwenRequestHandler(socketserver.StreamRequestHandler):
    def _write(self, value: dict[str, Any]) -> None:
        self.wfile.write(
            json.dumps(value, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
            + b'\n')
        self.wfile.flush()

    def handle(self) -> None:
        raw = self.rfile.readline(MAX_SERVICE_REQUEST_BYTES + 1)
        if len(raw) > MAX_SERVICE_REQUEST_BYTES:
            self._write({'event': 'error', 'error': 'request_too_large'})
            return
        try:
            request = json.loads(raw.decode('utf-8'))
            if not isinstance(request, dict):
                raise ValueError('request must be a JSON object')
            operation = str(request.get('op') or '')
            core: QwenServiceCore = self.server.core  # type: ignore[attr-defined]
            if operation == 'health':
                self._write({'event': 'health', **core.health()})
                return
            if operation == 'cancel':
                cancelled = core.controller.cancel_current(
                    str(request.get('request_id') or ''))
                self._write({'event': 'cancel', 'cancelled': cancelled})
                return
            if operation == 'reset':
                core.controller.reset_context()
                self._write({'event': 'reset', 'ok': True})
                return
            if operation == 'recover':
                core.recover(str(request.get('request_id') or 'manual-recover'))
                self._write({'event': 'recovered', **core.health()})
                return
            if operation == 'shutdown':
                self._write({'event': 'shutdown', 'ok': True})
                threading.Thread(
                    target=self.server.shutdown, daemon=True).start()  # type: ignore[attr-defined]
                return
            if operation not in ('chat_text', 'chat_image', 'chat_auto'):
                raise ValueError(f'unsupported operation: {operation}')
            request_id = str(request.get('request_id') or uuid.uuid4().hex)
            request['request_id'] = request_id
            self._write({'event': 'accepted', 'request_id': request_id})
            result, knowledge = core.chat(
                request,
                lambda token: self._write({
                    'event': 'token', 'request_id': request_id,
                    'text': token,
                }),
            )
            self._write({'event': 'result', **result_payload(result, knowledge)})
        except (BrokenPipeError, ConnectionResetError):
            core = getattr(self.server, 'core', None)  # type: ignore[attr-defined]
            if core is not None:
                core.controller.cancel_current()
        except Exception as error:
            try:
                self._write({
                    'event': 'error',
                    'error': f'{type(error).__name__}: {error}',
                })
            except (BrokenPipeError, ConnectionResetError):
                pass


class ThreadedUnixServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = False

    def __init__(self, path: str, core: QwenServiceCore) -> None:
        self.core = core
        super().__init__(path, QwenRequestHandler)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    workspace = Path(__file__).resolve().parents[3]
    parser.add_argument(
        '--config', type=Path,
        default=workspace / 'config/voice/qwen3vl.yaml')
    parser.add_argument('--workspace', type=Path, default=workspace)
    parser.add_argument('--no-ros', action='store_true',
                        help='run without perception and robot status guards')
    args = parser.parse_args()
    qwen_config, service_config = load_config(args.config, args.workspace)
    knowledge_runtime = None
    knowledge_error = ''
    try:
        knowledge_settings = load_knowledge_settings(args.config)
        knowledge_runtime = KnowledgeRuntime.load(knowledge_settings)
    except Exception as error:
        knowledge_error = f'{type(error).__name__}: {error}'
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    trace = TraceWriter(service_config.trace_directory / stamp / 'trace.jsonl')
    ros_runtime = None
    bridge = None
    if not args.no_ros:
        from .ros_bridge import RosBridgeRuntime
        ros_runtime = RosBridgeRuntime(
            service_config.perception_status_topic,
            service_config.perception_control_service,
            service_config.grasp_status_topic)
        bridge = ros_runtime.bridge
        time.sleep(0.6)
    coordinator = NpuCoordinator(
        bridge,
        service_config.pause_confirm_timeout_s,
        service_config.resume_confirm_timeout_s,
        service_config.status_stale_timeout_s,
        service_config.minimum_available_memory_mib,
        trace)
    controller = QwenController(
        qwen_config, service_config.runtime_dir, trace=trace)
    core = QwenServiceCore(
        controller, coordinator, service_config.auto_recover,
        knowledge_runtime, knowledge_error)
    socket_path = service_config.runtime_dir / service_config.service_socket_name
    service_config.runtime_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    service_lock = (service_config.runtime_dir / 'qwen-service.lock').open(
        'a+', encoding='ascii')
    try:
        fcntl.flock(
            service_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        service_lock.close()
        raise RuntimeError('another FR3C Qwen service is already running') from error
    service_lock.seek(0)
    service_lock.truncate()
    service_lock.write(f'{os.getpid()}\n')
    service_lock.flush()
    socket_path.unlink(missing_ok=True)
    server = ThreadedUnixServer(str(socket_path), core)
    os.chmod(socket_path, 0o600)

    def request_stop(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        core.ensure_ready('service-startup')
        server.serve_forever(poll_interval=0.2)
        return 0
    finally:
        server.server_close()
        socket_path.unlink(missing_ok=True)
        controller.close()
        if knowledge_runtime is not None:
            knowledge_runtime.close()
        if ros_runtime is not None:
            ros_runtime.close()
        fcntl.flock(service_lock.fileno(), fcntl.LOCK_UN)
        service_lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
