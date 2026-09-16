#!/usr/bin/env python3
"""P1 isolated production-backend benchmark; never issues robot commands or plays audio."""
import argparse
from dataclasses import replace
import fcntl
import json
import os
from pathlib import Path
import sys
import threading
import time

from p1_baseline import REFERENCE, ROOT, sample, write_json
sys.dont_write_bytecode = True
sys.path.insert(0, str(REFERENCE / 'dofbot/src/dofbot_pro_agent'))
sys.path.insert(0, str(REFERENCE / "dofbot/src/dofbot_pro_yolov11"))

EVENTS = ('input_end', 'asr_final', 'queue_enter', 'queue_leave', 'qwen_first_token',
          'first_text_segment', 'tts_first_pcm', 'first_valid_pcm_write',
          'generation_end', 'playback_end', 'action_state')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('tts', 'qwen', 'qwen_tts'), required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'trace.jsonl').exists():
        parser.error('output already has a trace; choose a new run directory')
    lock = (ROOT / 'evidence/voice/P1/benchmark.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = path.read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(Path(os.fsdecode(arg)).name == 'qwen3_vl_worker' for arg in argv if arg):
            raise RuntimeError('Existing Qwen worker detected; do not start a second model instance')
        if args.mode != 'tts' and any(b'blue_cube_localizer' in arg for arg in argv) and any(b'detection_mode:=yolo' in arg for arg in argv):
            raise RuntimeError('Stop YOLO perception before Qwen: current FR3C node has no coordinated pause acknowledgement')
    from dofbot_pro_agent.audio import KokoroTts, load_audio_config
    from dofbot_pro_agent import QwenController, load_config
    tracing = (args.output / 'trace.jsonl').open('w')
    trace_lock = threading.Lock()
    def emit(request_id, event, **details):
        with trace_lock:
            tracing.write(json.dumps({'schema_version': 1, 'run_id': args.output.name,
            'request_id': request_id, 'event': event, 'clock': 'CLOCK_MONOTONIC',
            'monotonic_ns': time.monotonic_ns(), 'wall_time_ns': time.time_ns(),
            'pid': os.getpid(), 'mode': args.mode, **details}, ensure_ascii=False) + '\n')
            tracing.flush()
    controller = None
    # Continuous sampling ends only after the benchmark; no time-box truncation.
    stop = threading.Event()
    def monitor():
        from p1_baseline import snapshot
        with (args.output / 'resources.jsonl').open('w') as handle:
            while not stop.is_set():
                handle.write(json.dumps(snapshot()) + '\n')
                handle.flush()
                stop.wait(.25)
    sampler = threading.Thread(target=monitor)
    sampler.start()
    try:
        if args.mode != 'tts':
            config = load_config(REFERENCE / 'dofbot/config/agent/qwen3vl.yaml', REFERENCE)
            config = replace(config, max_new_tokens=64)
            class TracedController(QwenController):
                def _publish(self, kind, message='', request_id='', elapsed_s=None):
                    if kind == 'request_started':
                        self._p1_first_token = True
                    if kind == 'token' and self._p1_first_token:
                        emit(request_id, 'qwen_first_token', latency_s=elapsed_s)
                        self._p1_first_token = False
                    super()._publish(kind, message, request_id, elapsed_s)
            controller = TracedController(config, args.output / 'worker')
            emit('startup', 'model_load_begin')
            controller.start()
            emit('startup', 'model_ready', worker_pid=controller.worker_pid)
        config = load_audio_config(REFERENCE / 'dofbot/config/agent/audio.yaml', REFERENCE)
        backend = KokoroTts(config) if args.mode != 'qwen' else None
        texts = json.loads((ROOT / 'config/voice/p1_texts.json').read_text())
        for phase in ('cold', 'warm'):
            for item in texts:
                request_id = f'{phase}-{item["characters"]}'
                # cold = first use of a fresh backend for every text length.
                if phase == 'cold' and backend:
                    backend = KokoroTts(config)
                errors = []
                result = {}
                def qwen():
                    try:
                        controller.reset_context()
                        value = controller.chat_text('请简短回答：' + item['text'], request_id)
                        result.update(qwen_first_token_s=value.first_token_s, qwen_total_s=value.total_s)
                    except Exception as error:
                        errors.append(str(error))
                emit(request_id, 'input_end', text_characters=item['characters'], phase=phase,
                     cold_scope='fresh_tts_backend_os_cache_retained' if phase == 'cold' else 'retained_backend',
                     unavailable_events=['asr_final', 'first_valid_pcm_write', 'playback_end', 'action_state'])
                emit(request_id, 'queue_enter')
                emit(request_id, 'queue_leave')
                worker = None
                if controller:
                    worker = threading.Thread(target=qwen)
                    worker.start()
                if backend:
                    first = [True]
                    emit(request_id, 'first_text_segment', source='fixed_test_text')
                    started = time.monotonic()
                    def pcm(data, rate):
                        if first[0]:
                            emit(request_id, 'tts_first_pcm', bytes=len(data), sample_rate=rate,
                                 latency_s=time.monotonic() - started)
                            first[0] = False
                        return True
                    duration, synthesis = backend.stream(item['text'], pcm)
                    result.update(audio_duration_s=duration, synthesis_s=synthesis,
                                  wall_s=time.monotonic() - started, rtf=synthesis / duration)
                if worker:
                    worker.join()
                if errors:
                    raise RuntimeError('; '.join(errors))
                emit(request_id, 'generation_end', **result)
                print(request_id, result, flush=True)
        emit('run', 'complete', playback_measured=False, whole_robot_concurrency=False)
    except Exception as error:
        emit('run', 'error', error=repr(error))
        raise
    finally:
        if controller:
            controller.close()
        stop.set()
        sampler.join()
        tracing.close()
        lock.close()


if __name__ == '__main__':
    main()
