"""Command-line client for the local FR3C Qwen service."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import uuid

from .config import load_config


def request(socket_path: Path, payload: dict) -> list[dict]:
    records = []
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.connect(str(socket_path))
        connection.sendall(
            json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
        reader = connection.makefile('r', encoding='utf-8')
        for line in reader:
            value = json.loads(line)
            records.append(value)
            print(json.dumps(value, ensure_ascii=False), flush=True)
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    workspace = Path(__file__).resolve().parents[3]
    parser.add_argument(
        'operation', choices=(
            'health', 'chat_text', 'chat_image', 'cancel', 'reset',
            'recover', 'shutdown'))
    parser.add_argument('prompt', nargs='?', default='')
    parser.add_argument('--image', type=Path)
    parser.add_argument('--request-id', default='')
    parser.add_argument('--keep-context', action='store_true')
    parser.add_argument(
        '--config', type=Path,
        default=workspace / 'config/voice/qwen3vl.yaml')
    parser.add_argument('--workspace', type=Path, default=workspace)
    args = parser.parse_args()
    _, service = load_config(args.config, args.workspace)
    payload = {
        'op': args.operation,
        'request_id': args.request_id or uuid.uuid4().hex,
    }
    if args.operation.startswith('chat_'):
        payload['prompt'] = args.prompt
        payload['reset_before'] = not args.keep_context
    if args.operation == 'chat_image':
        if args.image is None:
            parser.error('--image is required for chat_image')
        payload['image_path'] = str(args.image.resolve())
    try:
        records = request(
            service.runtime_dir / service.service_socket_name, payload)
    except OSError as error:
        print(f'Qwen service unavailable: {error}', file=sys.stderr)
        return 2
    return 1 if any(item.get('event') == 'error' for item in records) else 0


if __name__ == '__main__':
    raise SystemExit(main())
