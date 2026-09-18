#!/usr/bin/env python3
"""P5 load and concurrency measurement for the selected TTS backends.

Three levels, each measuring the same corpus so the numbers stay comparable:

1. ``tts``      - the TTS backend alone.
2. ``qwen``     - the same corpus while Qwen3-VL is answering text prompts
                  through the real P3 Unix service, so NPU load and CPU TTS run
                  together exactly as the product will.
3. ``full``     - the same again with the perception stack already running.
                  This level does not start or stop the robot stack; it records
                  what was running, so a human-operated stack cannot be
                  corrupted by an automated test and no motion command is ever
                  issued.

Reads ``/proc/meminfo``, per-process RSS/CPU and the rknpu load node through
the P1 sampler so the resource fields match the P1/P3 evidence.  Never plays
audio.  Never issues robot commands.
"""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import socket
import statistics
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config/voice/tts_candidates.json'
TEXTS = ROOT / 'config/voice/p5_texts.json'
ASSETS = ROOT / 'assets/voice/tts'
BENCH = Path(__file__).resolve().parent / 'p5_bench.py'
BASELINE = Path(__file__).resolve().parent / 'p1_baseline.py'
DEFAULT_SOCKET = ROOT / 'evidence/voice/P3/runtime/fr3c-qwen-service.sock'

sys.dont_write_bytecode = True


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class QwenClient:
    """Minimal JSONL client for the P3 voice service socket."""

    def __init__(self, socket_path):
        self.socket_path = Path(socket_path)

    def health(self):
        return self._call({'op': 'health'}, timeout=30)

    def chat_text(self, prompt, request_id, timeout=180):
        # Field names follow the P3 service protocol: `op` selects the
        # operation and `prompt` carries the text. Sending the wrong keys is
        # rejected by the service, so the concurrency numbers would otherwise
        # be measured with zero real Qwen load.
        return self._call({'op': 'chat_text', 'prompt': prompt, 'request_id': request_id},
                          timeout=timeout)

    def _call(self, payload, timeout):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout)
            connection.connect(str(self.socket_path))
            connection.sendall(json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
            records = []
            with connection.makefile('r', encoding='utf-8') as reader:
                for line in reader:
                    records.append(json.loads(line))
        return records


class Sampler(threading.Thread):
    """Resource sampler reusing the P1 snapshot fields.

    The default period is deliberately coarse. ``p1_baseline.snapshot`` walks
    every process and costs about 116 ms on this bench (351 processes), so a
    250 ms period would burn roughly half a core and become the very load this
    measurement is supposed to observe.  A one-second period costs about 12% of
    a core, and the cost is recorded in the output so the overhead stays
    visible instead of silently inflating the TTS numbers.
    """

    def __init__(self, baseline, path, interval=1.0):
        super().__init__(daemon=True)
        self.baseline = baseline
        self.path = path
        self.interval = interval
        self.stop = threading.Event()

    def run(self):
        with self.path.open('w') as handle:
            while not self.stop.is_set():
                handle.write(json.dumps(self.baseline.snapshot()) + '\n')
                handle.flush()
                self.stop.wait(self.interval)


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1 - weight) + ordered[upper] * weight


def running_processes(baseline):
    """Human-readable list of the stack components currently alive."""
    labels = ('qwen3_vl_worker', 'blue_cube_localizer', 'move_group', 'rviz2',
              'component_container', 'ros2_cmd_server', 'first_grasp_node')
    found = {}
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = path.read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except (OSError, PermissionError):
            continue
        for label in labels:
            if label in argv:
                found.setdefault(label, 0)
                found[label] += 1
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', action='append', required=True)
    parser.add_argument('--level', choices=('tts', 'qwen', 'full'), required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--affinity', default='big', choices=('default', 'big', 'little'))
    parser.add_argument('--socket', type=Path, default=DEFAULT_SOCKET)
    parser.add_argument('--qwen-requests', type=int, default=6)
    parser.add_argument('--texts', default=None, help='comma separated text ids')
    parser.add_argument('--sampler-interval', type=float, default=1.0,
                        help='resource sampling period; see Sampler docstring for the self-load tradeoff')
    parser.add_argument('--note', default='')
    args = parser.parse_args()

    if args.affinity == 'big' and hasattr(os, 'sched_setaffinity'):
        os.sched_setaffinity(0, {4, 5, 6, 7})
    elif args.affinity == 'little' and hasattr(os, 'sched_setaffinity'):
        os.sched_setaffinity(0, {0, 1, 2, 3})

    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'load.json').exists():
        parser.error(f'{args.output} already has load.json; choose a new run directory')

    bench = load_module('p5_bench_module', BENCH)
    baseline = load_module('p1_baseline_module', BASELINE)
    config = json.loads(CONFIG.read_text())
    texts = json.loads(TEXTS.read_text())
    wanted = set(args.texts.split(',')) if args.texts else None
    corpus = []
    for category in texts['measurement_texts']['categories']:
        for item in texts['categories'][category]['texts']:
            if wanted is None or item['id'] in wanted:
                corpus.append(item)

    trace = bench.Trace(args.output / 'trace.jsonl')
    stack = running_processes(baseline)
    qwen = None
    if args.level in ('qwen', 'full'):
        qwen = QwenClient(args.socket)
        try:
            health = qwen.health()
        except OSError as error:
            parser.error(f'{args.level} level needs the P3 service socket at {args.socket}: {error}')
        trace.emit('environment', 'qwen_health', records=health)

    sampler = Sampler(baseline, args.output / 'resources.jsonl', interval=args.sampler_interval)
    sampler.start()
    trace.emit('environment', 'run_begin', level=args.level, candidates=args.candidate,
               threads=args.threads, affinity=args.affinity, speed=args.speed,
               repeats=args.repeats, sampler_interval_s=args.sampler_interval,
               socket=str(args.socket) if qwen is not None else None, note=args.note,
               stack_state=stack)
    results = {}
    try:
        for name in args.candidate:
            candidate = config['candidates'][name]
            spec = bench.build_engine(name, candidate, config, args.threads, ASSETS / name)
            speaker_id = config['speaker_ids'].get(name, 0) or 0
            rows = []
            stop_qwen = threading.Event()
            qwen_state = {'done': 0, 'errors': [], 'first_token_s': [], 'total_s': []}

            def qwen_worker():
                # Latency is measured here rather than read from the response:
                # the service stream carries no timing fields, so a client-side
                # clock is the only honest source for these numbers.
                index = 0
                while not stop_qwen.is_set() and index < args.qwen_requests:
                    request_id = f'p5-load-{name}-{index}'
                    started = time.monotonic()
                    first_token = None
                    try:
                        records = qwen.chat_text('请用一句话说明当前状态。', request_id)
                        for record in records:
                            event = record.get('event')
                            if event == 'token' and first_token is None:
                                first_token = time.monotonic() - started
                            elif event == 'error':
                                raise RuntimeError(record.get('error', 'service error'))
                        if first_token is not None:
                            qwen_state['first_token_s'].append(first_token)
                        qwen_state['total_s'].append(time.monotonic() - started)
                        qwen_state['done'] += 1
                    except Exception as error:  # noqa: BLE001 - recorded, not raised
                        qwen_state['errors'].append(repr(error))
                    index += 1

            worker = None
            if qwen is not None:
                worker = threading.Thread(target=qwen_worker, daemon=True)
                worker.start()
                time.sleep(0.5)

            for item in corpus:
                for repeat in range(args.repeats):
                    request_id = f'{item["id"]}#{repeat}'
                    result, pcm16 = bench.synthesize_measured(
                        spec, item['text'], speaker_id, args.speed, trace, request_id,
                        phase=args.level, level=args.level, text_id=item['id'])
                    result['request_id'] = request_id
                    result['text_id'] = item['id']
                    rows.append(result)
                print(f'  {name} {item["id"]:<9} rtf={rows[-1]["rtf"]:.3f} '
                      f'first_valid={rows[-1]["first_valid_pcm_s"]}')

            stop_qwen.set()
            if worker:
                worker.join(timeout=200)

            rtf = [row['rtf'] for row in rows if row.get('rtf')]
            first = [row['first_valid_pcm_s'] for row in rows
                     if row.get('first_valid_pcm_s') is not None]
            results[name] = {
                'samples': len(rows),
                'rtf_p50': percentile(rtf, 0.50),
                'rtf_p95': percentile(rtf, 0.95),
                'rtf_max': max(rtf) if rtf else None,
                'first_valid_pcm_p50': percentile(first, 0.50),
                'first_valid_pcm_p95': percentile(first, 0.95),
                'health': 'failed' if any(row.get('whole_audio_onset_s') is None for row in rows) else 'ok',
                'qwen': {
                    'requests_done': qwen_state['done'],
                    'errors': qwen_state['errors'],
                    'first_token_s_p50': percentile(qwen_state['first_token_s'], 0.50),
                    'total_s_p50': percentile(qwen_state['total_s'], 0.50),
                } if qwen is not None else None,
                'rows': rows,
            }
    finally:
        sampler.stop.set()
        sampler.join(timeout=10)
        trace.close()

    samples = [json.loads(line) for line in (args.output / 'resources.jsonl').read_text().splitlines() if line]
    memory = {
        'minimum_available_mib': min(s['memory']['MemAvailable_kib'] for s in samples) / 1024,
        'maximum_swap_used_mib': max(s['memory']['SwapTotal_kib'] - s['memory']['SwapFree_kib']
                                     for s in samples) / 1024,
        'samples': len(samples),
        'npu_available': any(s['npu']['raw'] is not None for s in samples),
    }
    payload = {
        'schema_version': 1,
        'level': args.level,
        'candidates': args.candidate,
        'threads': args.threads,
        'affinity': args.affinity,
        'speed': args.speed,
        'note': args.note,
        'stack_at_start': stack,
        'sampler_interval_s': args.sampler_interval,
        'memory': memory,
        'per_candidate': {name: {key: value for key, value in entry.items() if key != 'rows'}
                          for name, entry in results.items()},
        'rows': {name: entry['rows'] for name, entry in results.items()},
    }
    (args.output / 'load.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({key: payload[key] for key in ('level', 'candidates', 'memory', 'per_candidate')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
