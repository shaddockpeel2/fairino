#!/usr/bin/env python3
"""Summarize observed resource peaks and measured events without imputing missing data."""
import argparse
import json
import os
from pathlib import Path
from p1_baseline import write_json


def summarize(directory):
    rows = [json.loads(line) for line in (directory / 'resources.jsonl').read_text().splitlines()]
    if not rows:
        raise ValueError('empty resource sample')
    components = {}
    previous = {}
    hz = os.sysconf('SC_CLK_TCK')
    for row in rows:
        current = {}
        for proc in row['processes']:
            key = (proc['pid'], proc['start_ticks'])
            item = components.setdefault(proc['component'], {'peak_process_rss_mib': 0, 'peak_process_cpu_percent': 0, 'process_names': set()})
            if proc['component'] == 'other':
                continue
            item['process_names'].add(proc['comm'])
            item['peak_process_rss_mib'] = max(item['peak_process_rss_mib'], proc['rss_kib'] / 1024)
            if key in previous:
                timestamp, ticks = previous[key]
                seconds = (row['monotonic_ns'] - timestamp) / 1e9
                item['peak_process_cpu_percent'] = max(item['peak_process_cpu_percent'], 100 * (proc['cpu_ticks'] - ticks) / hz / seconds)
            current[key] = (row['monotonic_ns'], proc['cpu_ticks'])
        previous = current
    components.pop('other', None)
    for item in components.values():
        item['process_names'] = sorted(item['process_names'])
    traces = directory / 'trace.jsonl'
    events = [json.loads(line) for line in traces.read_text().splitlines()] if traces.exists() else []
    startup = {e['event']: e['monotonic_ns'] for e in events if e['request_id'] == 'startup'}
    result = {'samples': len(rows), 'sampled_duration_s': (rows[-1]['monotonic_ns'] - rows[0]['monotonic_ns']) / 1e9,
        'minimum_available_mib': min(r['memory']['MemAvailable_kib'] for r in rows)/1024,
        'maximum_swap_used_mib': max(r['memory']['SwapTotal_kib']-r['memory']['SwapFree_kib'] for r in rows)/1024,
        'npu_available': any(r['npu']['raw'] is not None for r in rows), 'components': components,
        'model_startup_s': (startup['model_ready'] - startup['model_load_begin'])/1e9 if 'model_ready' in startup else None,
        'completed': any(e['event']=='complete' for e in events) if events else None,
        'generation_results': [e for e in events if e['event']=='generation_end'],
        'first_pcm_results': [e for e in events if e['event']=='tts_first_pcm']}
    write_json(directory / 'summary.json', result)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args=parser.parse_args()
    print(json.dumps(summarize(args.directory), ensure_ascii=False, indent=2))
