#!/usr/bin/env python3
"""Inject a real short Qwen deadline, then verify explicit recovery."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[2]
PACKAGE = WORKSPACE / 'src/fr3c_voice'
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))

from fr3c_voice.config import load_config  # noqa: E402
from fr3c_voice.controller import ControllerError, QwenController  # noqa: E402
from fr3c_voice.trace import TraceWriter  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=WORKSPACE / 'config/voice/qwen3vl.yaml')
    parser.add_argument('--timeout', type=float, default=0.05)
    parser.add_argument('--output', type=Path,
                        default=WORKSPACE / 'evidence/voice/P3/timeout_recovery.json')
    args = parser.parse_args()
    normal, service = load_config(args.config, WORKSPACE)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    trace = TraceWriter(args.output.with_suffix('.trace.jsonl'))
    controller = QwenController(
        replace(normal, request_timeout_s=args.timeout),
        service.runtime_dir, trace=trace)
    timeout_error = ''
    first_pid = None
    try:
        controller.start()
        first_pid = controller.worker_pid
        try:
            controller.chat_text(
                '请只回答：这条请求用于真实超时注入。', request_id='timeout-injected')
        except ControllerError as error:
            timeout_error = str(error)
        if 'timeout' not in timeout_error.lower():
            raise RuntimeError(f'request did not time out as expected: {timeout_error}')
        controller.config = normal
        controller.recover()
        result = controller.chat_text(
            '请只回答：超时后恢复正常。', request_id='after-timeout')
        summary = {
            'ok': True,
            'injected_timeout_s': args.timeout,
            'timeout_error': timeout_error,
            'worker_pid_before': first_pid,
            'worker_pid_after': controller.worker_pid,
            'recovery_result': asdict(result),
            'healthy_after': controller.health(),
        }
        args.output.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8')
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    finally:
        controller.close()


if __name__ == '__main__':
    raise SystemExit(main())
