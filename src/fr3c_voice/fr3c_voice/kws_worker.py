#!/usr/bin/env python3
"""Isolated CPU Sherpa-Onnx keyword spotter for raw PCM16 input."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np


MODEL_FILES = {
    'tokens': 'tokens.txt',
    'encoder': 'encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx',
    'decoder': 'decoder-epoch-12-avg-2-chunk-16-left-64.onnx',
    'joiner': 'joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx',
}


def emit(event: str, **payload) -> None:
    print(json.dumps(
        {'event': event, **payload}, ensure_ascii=False,
        separators=(',', ':')), flush=True)


def run(args: argparse.Namespace) -> int:
    import sherpa_onnx

    model = Path(args.model_dir)
    paths = {name: model / filename for name, filename in MODEL_FILES.items()}
    missing = [str(path) for path in (*paths.values(), Path(args.keywords_file))
               if not path.is_file()]
    if missing:
        raise RuntimeError('missing KWS assets: ' + ', '.join(missing))
    spotter = sherpa_onnx.KeywordSpotter(
        tokens=os.fspath(paths['tokens']),
        encoder=os.fspath(paths['encoder']),
        decoder=os.fspath(paths['decoder']),
        joiner=os.fspath(paths['joiner']),
        keywords_file=args.keywords_file,
        num_threads=args.num_threads,
        sample_rate=args.sample_rate,
        keywords_score=args.score,
        keywords_threshold=args.threshold,
        num_trailing_blanks=args.num_trailing_blanks,
        provider='cpu')
    stream = spotter.create_stream()
    emit('ready')
    while True:
        raw = sys.stdin.buffer.read(args.chunk_samples * 2)
        if not raw:
            return 0
        if len(raw) % 2:
            raw = raw[:-1]
        if not raw:
            continue
        samples = np.frombuffer(raw, dtype='<i2').astype(np.float32)
        samples /= 32768.0
        stream.accept_waveform(args.sample_rate, samples)
        while spotter.is_ready(stream):
            spotter.decode_stream(stream)
            keyword = str(spotter.get_result(stream)).strip()
            if keyword:
                emit('wake', keyword=keyword)
                return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-dir', required=True)
    parser.add_argument('--keywords-file', required=True)
    parser.add_argument('--sample-rate', type=int, default=16000)
    parser.add_argument('--num-threads', type=int, default=2)
    parser.add_argument('--score', type=float, default=1.8)
    parser.add_argument('--threshold', type=float, default=0.2)
    parser.add_argument('--num-trailing-blanks', type=int, default=1)
    parser.add_argument('--chunk-samples', type=int, default=1600)
    return parser.parse_args()


def main() -> int:
    try:
        return run(parse_args())
    except Exception as error:
        emit('error', message=f'{type(error).__name__}: {error}')
        print(
            f'KWS worker failed: {type(error).__name__}: {error}',
            file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
