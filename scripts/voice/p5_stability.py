#!/usr/bin/env python3
"""P5 short-text stability test: repeat the same short item and look for failures.

The main benchmark synthesises each text a handful of times and was clean for
every candidate, yet repeating single characters exposes a real defect: one
candidate intermittently returns a 60-70 ms near-silent buffer (about -80 dBFS)
for common digits.  A robot has to say numbers out loud, so "usually works" is
not good enough and the defect must be measured, not discovered on the bench.

The check is deliberately narrow, because it is a gate rather than a
benchmark:

* fixed short items (single digits, a colour, a two-character acknowledgement)
* repeated many times
* two statistics per item: the worst peak level, and the rate of runs whose
  peak is below the audibility threshold

Never plays audio.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import statistics
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config/voice/tts_candidates.json'
ASSETS = ROOT / 'assets/voice/tts'
BENCH = Path(__file__).resolve().parent / 'p5_bench.py'

# A run quieter than this would be inaudible or barely audible on the bench
# speaker; it is a defect regardless of the text length.
QUIET_PEAK_DBFS = -30.0
# A run shorter than this for a text that should be at least a syllable long is
# a truncated utterance rather than a short one.
TRUNCATED_SECONDS = 0.10

sys.dont_write_bytecode = True

# Short items only: one or two characters, which is where the defect appeared.
ITEMS = [
    ('一', 'digit'), ('二', 'digit'), ('三', 'digit'), ('四', 'digit'),
    ('五', 'digit'), ('六', 'digit'), ('七', 'digit'), ('八', 'digit'),
    ('九', 'digit'), ('十', 'digit'),
    ('红色', 'colour'), ('绿色', 'colour'), ('蓝色', 'colour'),
    ('好的', 'ack'),
]


def load_bench():
    spec = importlib.util.spec_from_file_location('p5_bench_module', BENCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def percentile(values, fraction):
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1 - weight) + ordered[upper] * weight


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', action='append', default=None)
    parser.add_argument('--repeats', type=int, default=10)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--affinity', default='big', choices=('default', 'big', 'little'))
    args = parser.parse_args()

    if args.affinity == 'big' and hasattr(os, 'sched_setaffinity'):
        os.sched_setaffinity(0, {4, 5, 6, 7})
    elif args.affinity == 'little' and hasattr(os, 'sched_setaffinity'):
        os.sched_setaffinity(0, {0, 1, 2, 3})

    bench = load_bench()
    config = json.loads(CONFIG.read_text())
    candidates = args.candidate or list(config['candidates'])

    args.output.mkdir(parents=True, exist_ok=True)
    trace = bench.Trace(args.output / 'trace.jsonl')
    report = {'schema_version': 1, 'repeats': args.repeats, 'threads': args.threads,
              'quiet_peak_dbfs': QUIET_PEAK_DBFS, 'truncated_seconds': TRUNCATED_SECONDS,
              'candidates': {}}
    try:
        for name in candidates:
            spec = bench.build_engine(name, config['candidates'][name], config, args.threads,
                                      ASSETS / name)
            speaker_id = config['speaker_ids'].get(name, 0) or 0
            print(f'== {name}: sr={spec.sample_rate} speakers={spec.num_speakers} '
                  f'sid={speaker_id} repeats={args.repeats}')
            items = []
            for text, category in ITEMS:
                peaks, durations, failures = [], [], []
                for repeat in range(args.repeats):
                    request_id = f'{name}-{category}-{text}-{repeat}'
                    try:
                        result, _ = bench.synthesize_measured(
                            spec, text, speaker_id, args.speed, trace, request_id,
                            phase='stability', item_text=text, repeat=repeat)
                    except Exception as error:  # noqa: BLE001 - recorded as a failure
                        failures.append(repr(error))
                        trace.emit(request_id, 'stability_failure', error=repr(error))
                        continue
                    peaks.append(result['peak_dbfs'])
                    durations.append(result['audio_duration_s'])
                    if result['peak_dbfs'] < QUIET_PEAK_DBFS:
                        trace.emit(request_id, 'stability_quiet', peak_dbfs=result['peak_dbfs'],
                                   audio_duration_s=result['audio_duration_s'])
                    if result['audio_duration_s'] < TRUNCATED_SECONDS:
                        trace.emit(request_id, 'stability_truncated',
                                   audio_duration_s=result['audio_duration_s'])
                quiet = sum(1 for peak in peaks if peak < QUIET_PEAK_DBFS)
                truncated = sum(1 for value in durations if value < TRUNCATED_SECONDS)
                total = len(peaks) + len(failures)
                items.append({
                    'text': text,
                    'category': category,
                    'runs': total,
                    'quiet_runs': quiet,
                    'truncated_runs': truncated,
                    'errors': failures,
                    'quiet_rate': quiet / total if total else None,
                    'peak_dbfs_min': min(peaks) if peaks else None,
                    'peak_dbfs_median': statistics.median(peaks) if peaks else None,
                    'peak_dbfs_max': max(peaks) if peaks else None,
                    'peak_spread_db': (max(peaks) - min(peaks)) if peaks else None,
                    'duration_s_min': min(durations) if durations else None,
                })
                flag = ' <== 不稳定' if (quiet or truncated or failures) else ''
                print(f'   {text:<4} {category:<7} runs={total:>2} quiet={quiet:>2} '
                      f'truncated={truncated:>2} peak={items[-1]["peak_dbfs_min"]}..'
                      f'{items[-1]["peak_dbfs_max"]}dBFS{flag}')
            failing = [item for item in items
                       if item['quiet_runs'] or item['truncated_runs'] or item['errors']]
            report['candidates'][name] = {
                'sample_rate': spec.sample_rate,
                'num_speakers': spec.num_speakers,
                'speaker_id': speaker_id,
                'items': items,
                'items_unstable': [item['text'] for item in failing],
                'overall_quiet_rate': (sum(item['quiet_runs'] for item in items)
                                       / max(1, sum(item['runs'] for item in items))),
                'verdict': 'unstable' if failing else 'stable',
            }
    finally:
        trace.close()

    (args.output / 'stability.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print()
    for name, entry in report['candidates'].items():
        print(f'{name:<26} verdict={entry["verdict"]:<9} 不稳定条目={len(entry["items_unstable"])} '
              f'整体静音率={entry["overall_quiet_rate"] * 100:.1f}%'
              + (f'  {entry["items_unstable"]}' if entry['items_unstable'] else ''))


if __name__ == '__main__':
    main()
