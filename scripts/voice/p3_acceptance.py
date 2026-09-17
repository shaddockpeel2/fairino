#!/usr/bin/env python3
"""Stage-3 board acceptance through the production Unix service."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import threading
import time
import uuid


def call(socket_path: Path, payload: dict) -> list[dict]:
    records = []
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(180)
        connection.connect(str(socket_path))
        connection.sendall(
            json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
        with connection.makefile('r', encoding='utf-8') as reader:
            for line in reader:
                records.append(json.loads(line))
    return records


def mem_available_mib() -> float:
    for line in Path('/proc/meminfo').read_text(encoding='ascii').splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) / 1024.0
    raise RuntimeError('MemAvailable missing')


def swap_used_mib() -> float:
    values = {}
    for line in Path('/proc/meminfo').read_text(encoding='ascii').splitlines():
        key = line.split(':', 1)[0]
        if key in ('SwapTotal', 'SwapFree'):
            values[key] = int(line.split()[1])
    return (values['SwapTotal'] - values['SwapFree']) / 1024.0


def result_record(records: list[dict]) -> dict:
    errors = [item for item in records if item.get('event') == 'error']
    if errors:
        raise RuntimeError(errors[-1].get('error', 'service error'))
    results = [item for item in records if item.get('event') == 'result']
    if len(results) != 1 or not results[0].get('text'):
        raise RuntimeError('request did not produce one non-empty result')
    return results[0]


def write(handle, value: dict) -> None:
    handle.write(json.dumps(
        value, ensure_ascii=False, separators=(',', ':')) + '\n')
    handle.flush()


def main() -> int:
    workspace = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', type=Path,
                        default=workspace / 'evidence/voice/P3/runtime/fr3c-qwen-service.sock')
    parser.add_argument('--text-count', type=int, default=30)
    parser.add_argument('--image-count', type=int, default=3)
    parser.add_argument(
        '--image', type=Path,
        default=Path('/home/industio/DofbotPro_JetsonNano_0106/'
                     'rknn-llm-main/examples/multimodal_model_demo/data/demo.jpg'))
    parser.add_argument('--output', type=Path,
                        default=workspace / 'evidence/voice/P3/acceptance.jsonl')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    minimum_memory = mem_available_mib()
    passed = {'text': 0, 'image': 0, 'busy': 0, 'cancel_recovery': 0,
              'worker_exit_recovery': 0}
    initial_health = call(args.socket, {'op': 'health'})[-1]
    initial_pid = int(initial_health['worker_pid'])
    with args.output.open('w', encoding='utf-8') as output:
        write(output, {'event': 'start', 'health': initial_health,
                       'mem_available_mib': minimum_memory})
        for index in range(1, args.text_count + 1):
            started = time.monotonic()
            result = result_record(call(args.socket, {
                'op': 'chat_text', 'request_id': f'text-{index:02d}',
                'prompt': '请只回答：FR3C第三阶段正常。', 'reset_before': True,
            }))
            memory = mem_available_mib()
            minimum_memory = min(minimum_memory, memory)
            write(output, {
                'event': 'request', 'type': 'text', 'index': index,
                'first_token_s': result['first_token_s'],
                'model_total_s': result['total_s'],
                'client_total_s': time.monotonic() - started,
                'mem_available_mib': memory,
            })
            passed['text'] += 1
        for index in range(1, args.image_count + 1):
            started = time.monotonic()
            result = result_record(call(args.socket, {
                'op': 'chat_image', 'request_id': f'image-{index:02d}',
                'prompt': '请用一句简短中文描述图片。',
                'image_path': str(args.image), 'reset_before': True,
            }))
            memory = mem_available_mib()
            minimum_memory = min(minimum_memory, memory)
            write(output, {
                'event': 'request', 'type': 'image', 'index': index,
                'first_token_s': result['first_token_s'],
                'model_total_s': result['total_s'],
                'client_total_s': time.monotonic() - started,
                'mem_available_mib': memory,
            })
            passed['image'] += 1

        background_result: list[list[dict]] = []
        thread = threading.Thread(target=lambda: background_result.append(call(
            args.socket, {
                'op': 'chat_image', 'request_id': 'busy-holder',
                'prompt': '详细描述图片。', 'image_path': str(args.image),
                'reset_before': True,
            })))
        thread.start()
        time.sleep(0.5)
        busy = call(args.socket, {
            'op': 'chat_text', 'request_id': 'busy-rejected',
            'prompt': '这条请求应被拒绝。', 'reset_before': True,
        })
        thread.join(30)
        if not any('busy' in str(item.get('error', '')).lower() for item in busy):
            raise RuntimeError(f'concurrent request was not rejected: {busy}')
        result_record(background_result[0])
        passed['busy'] = 1
        write(output, {'event': 'busy_rejection', 'ok': True})

        cancelled_result: list[list[dict]] = []
        cancel_id = 'cancel-image'
        thread = threading.Thread(target=lambda: cancelled_result.append(call(
            args.socket, {
                'op': 'chat_image', 'request_id': cancel_id,
                'prompt': '请非常详细地描述图片中的每个细节。',
                'image_path': str(args.image), 'reset_before': True,
            })))
        thread.start()
        time.sleep(0.7)
        cancel_reply = call(args.socket, {
            'op': 'cancel', 'request_id': cancel_id})
        thread.join(15)
        if not cancel_reply[-1].get('cancelled') or not cancelled_result:
            raise RuntimeError('cancel request did not stop the active request')
        recovered = result_record(call(args.socket, {
            'op': 'chat_text', 'request_id': 'after-cancel',
            'prompt': '请只回答：取消后恢复正常。', 'reset_before': True,
        }))
        health_after_cancel = call(args.socket, {'op': 'health'})[-1]
        if not health_after_cancel.get('healthy') or not recovered.get('text'):
            raise RuntimeError('service did not recover after cancellation')
        passed['cancel_recovery'] = 1
        write(output, {
            'event': 'cancel_recovery', 'ok': True,
            'cancel_reply': cancel_reply[-1],
            'worker_pid_after': health_after_cancel.get('worker_pid'),
        })

        exit_pid = int(health_after_cancel['worker_pid'])
        os.kill(exit_pid, signal.SIGKILL)
        time.sleep(0.3)
        recovered = result_record(call(args.socket, {
            'op': 'chat_text', 'request_id': 'after-worker-exit',
            'prompt': '请只回答：进程退出后恢复正常。', 'reset_before': True,
        }))
        final_health = call(args.socket, {'op': 'health'})[-1]
        if not final_health.get('healthy') or int(final_health['worker_pid']) == exit_pid:
            raise RuntimeError('service did not replace the exited worker')
        passed['worker_exit_recovery'] = 1
        minimum_memory = min(minimum_memory, mem_available_mib())
        write(output, {
            'event': 'worker_exit_recovery', 'ok': True,
            'worker_pid_before': exit_pid,
            'worker_pid_after': final_health['worker_pid'],
            'response': recovered['text'],
        })
        summary = {
            'event': 'summary', 'ok': True, 'passed': passed,
            'initial_worker_pid': initial_pid,
            'final_worker_pid': final_health['worker_pid'],
            'minimum_mem_available_mib': minimum_memory,
            'swap_used_mib': swap_used_mib(),
            'final_health': final_health,
        }
        write(output, summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
