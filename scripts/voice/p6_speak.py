#!/usr/bin/env python3
"""Drive the streaming speech service, optionally through the Qwen service.

Two modes:

* ``--text`` feeds fixed text straight into the pipeline. This is the mode the
  acceptance harness uses, because it isolates the TTS half of the pipeline
  from model behaviour.
* ``--prompt`` runs the prompt through the phase-3 Qwen service over its own
  socket and forwards each token as it arrives, which is the real product path:
  text generation and speech synthesis overlap.

The ``--token-delay`` option injects a deliberate pause between forwarded
tokens. It exists to answer the question the phase plan asks directly: does
audio start before the last token, or does the client merely wait for the whole
answer? Without a controlled slow source a short answer cannot distinguish the
two.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_PIPELINE_SOCKET = WORKSPACE / 'evidence/voice/P6/runtime/fr3c-tts-pipeline.sock'
DEFAULT_QWEN_SOCKET = WORKSPACE / 'evidence/voice/P3/runtime/fr3c-qwen-service.sock'


def call(socket_path: Path, payload: dict, timeout: float = 600) -> list[dict]:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(socket_path))
        connection.sendall(json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
        records = []
        with connection.makefile('r', encoding='utf-8') as reader:
            for line in reader:
                records.append(json.loads(line))
                if records[-1].get('event') in ('result', 'error', 'stopped'):
                    break
        return records


def stream_qwen(socket_path: Path, prompt: str, request_id: str,
                on_token, timeout: float = 300, token_delay: float = 0.0) -> dict:
    """Send a chat request and forward tokens, keeping the connection open."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(socket_path))
        connection.sendall(json.dumps(
            {'op': 'chat_text', 'prompt': prompt, 'request_id': request_id},
            ensure_ascii=False).encode('utf-8') + b'\n')
        result: dict = {}
        first_token_s = None
        started = time.monotonic()
        with connection.makefile('r', encoding='utf-8') as reader:
            for line in reader:
                record = json.loads(line)
                event = record.get('event')
                if event == 'token':
                    if first_token_s is None:
                        first_token_s = time.monotonic() - started
                    on_token(str(record.get('text') or ''))
                    if token_delay:
                        time.sleep(token_delay)
                elif event == 'result':
                    result = record
                    break
                elif event == 'error':
                    raise RuntimeError(record.get('error', 'qwen service error'))
        result['first_token_s'] = first_token_s
        result['client_total_s'] = time.monotonic() - started
        return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--text', help='fixed text fed directly to the pipeline')
    source.add_argument('--prompt', help='prompt sent to the Qwen service, tokens streamed')
    parser.add_argument('--repeat', type=int, default=1,
                        help='repeat the text; used to build long inputs')
    parser.add_argument('--pipeline-socket', type=Path, default=DEFAULT_PIPELINE_SOCKET)
    parser.add_argument('--qwen-socket', type=Path, default=DEFAULT_QWEN_SOCKET)
    parser.add_argument('--request-id', default='')
    parser.add_argument('--token-delay', type=float, default=0.0,
                        help='seconds to wait after each Qwen token, to prove overlap')
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--json', action='store_true', help='print only the final result record')
    parser.add_argument('--verbose', action='store_true', help='print every pipeline event')
    parser.add_argument('--stop-after', type=float, default=0.0,
                        help='seconds after the first audio write to send stop, to test cancellation')
    args = parser.parse_args()

    request_id = args.request_id or uuid.uuid4().hex
    text = ''
    if args.text:
        text = (args.text + ' ') * args.repeat if args.repeat > 1 else args.text

    started = time.monotonic()
    events: list[dict] = []
    stop_sent = False

    # speak_start first, so the pipeline is running before any text exists.
    start_records = call(args.pipeline_socket, {'op': 'speak_start', 'request_id': request_id},
                         timeout=60)
    if not args.json:
        for record in start_records:
            print(json.dumps(record, ensure_ascii=False), flush=True)

    stop_thread = None
    if args.stop_after > 0:
        import threading

        def delayed_stop() -> None:
            deadline = time.monotonic() + args.stop_after
            while time.monotonic() < deadline:
                time.sleep(0.01)
            call(args.pipeline_socket, {'op': 'stop', 'request_id': request_id,
                                        'reason': 'timed_test'}, timeout=30)
        stop_thread = threading.Thread(target=delayed_stop, daemon=True)
        stop_thread.start()
        stop_sent = True

    qwen_result: dict = {}
    try:
        if args.prompt:
            def forward(token: str) -> None:
                call(args.pipeline_socket,
                     {'op': 'speak_text', 'request_id': request_id, 'text': token},
                     timeout=60)
            qwen_result = stream_qwen(args.qwen_socket, args.prompt, f'{request_id}-qwen',
                                      forward, timeout=args.timeout,
                                      token_delay=args.token_delay)
        else:
            chunk = 24 if args.repeat > 1 else len(text)
            for index in range(0, len(text), chunk):
                call(args.pipeline_socket,
                     {'op': 'speak_text', 'request_id': request_id,
                      'text': text[index:index + chunk]}, timeout=60)
    except Exception as error:  # noqa: BLE001 - reported below
        print(json.dumps({'event': 'error', 'stage': 'text_feed',
                          'error': f'{type(error).__name__}: {error}'}, ensure_ascii=False),
              flush=True)

    text_fed_s = time.monotonic() - started
    records = call(args.pipeline_socket, {'op': 'speak_end', 'request_id': request_id,
                                          'timeout_s': args.timeout},
                   timeout=args.timeout + 60)
    for record in records:
        events.append(record)
        if args.verbose and not args.json:
            print(json.dumps(record, ensure_ascii=False), flush=True)

    final = next((record for record in reversed(records) if record.get('event') == 'result'), {})
    summary = {
        'request_id': request_id,
        'text_characters': len(text) if args.text else None,
        'repeat': args.repeat,
        'text_fed_s': round(text_fed_s, 4),
        'qwen_first_token_s': qwen_result.get('first_token_s'),
        'qwen_total_s': qwen_result.get('client_total_s'),
        'qwen_text': qwen_result.get('text'),
        'stop_sent': stop_sent,
        **{key: final.get(key) for key in (
            'segment_count', 'first_token_s', 'first_segment_s', 'first_pcm_s',
            'first_valid_pcm_write_s', 'text_end_s', 'playback_end_s', 'stop_latency_s',
            'audio_seconds_written', 'audio_started_before_text_end', 'inter_segment_gaps_s',
            'pcm_queue_max', 'text_queue_max', 'xruns', 'cancelled', 'failure')},
        'events': len(events),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2 if not args.json else None), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
