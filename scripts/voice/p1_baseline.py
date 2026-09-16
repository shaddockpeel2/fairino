#!/usr/bin/env python3
"""Read-only source inventory and bounded board resource sampler for voice P1."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = Path('/home/industio/DofbotPro_JetsonNano_0106')


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def inventory(output):
    import yaml
    paths = set()
    configurations = {}
    for name in ('qwen3vl', 'audio', 'rag', 'npu_coordination'):
        source = REFERENCE / 'dofbot/config/agent' / (name + '.yaml')
        data = yaml.safe_load(source.read_text())
        configurations[name] = data
        paths.add(source)
        def walk(value):
            if isinstance(value, dict):
                for child in value.values():
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)
            elif isinstance(value, str) and value.startswith(('dofbot/', 'rknn-llm-main/')):
                paths.add(REFERENCE / value)
        walk(data)
    for pattern in ('dofbot/src/dofbot_pro_agent/**/*.py', 'dofbot/tools/p10_operator_gui.py',
                    'dofbot/tools/p17_streaming_asr_worker.py',
                    'dofbot/src/dofbot_pro_yolov11/dofbot_pro_yolov11/target_selection.py',
                    'dofbot/.venv/lib/python*/site-packages/*dist-info/METADATA',
                    'dofbot/kokoro_tts/kokoro-multi-lang-v1_1/LICENSE',
                    'rknn-llm-main/LICENSE',
                    'dofbot/.venv/lib/python*/site-packages/sherpa_onnx/**/*.so*'):
        paths.update(REFERENCE.glob(pattern))
    for pattern in ('src/fr3c_camera/models/*', 'src/fr3c_camera/config/*.yaml',
                    'src/fairino_mtc_demo/config/*.yaml', 'src/fairino3mt_v6_moveit2_config/config/*handeye*',
                    'scripts/run_moveit_camera_rviz.sh', 'evidence/a4_paper_grid.json'):
        paths.update(ROOT.glob(pattern))
    records = []
    expanded = set()
    for path in paths:
        if path.is_dir():
            # Runtime directories are mutable outputs, never immutable assets.
            if '/Log/' not in str(path):
                expanded.update(p for p in path.rglob('*') if p.is_file())
        else:
            expanded.add(path)
    for path in sorted(expanded):
        if '__pycache__' in str(path):
            continue
        row = {'path': str(path), 'resolved_path': str(path.resolve()), 'exists': path.is_file()}
        if path.is_file():
            digest = hashlib.sha256()
            with path.open('rb') as handle:
                for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b''):
                    digest.update(chunk)
            row.update(bytes=path.stat().st_size, sha256=digest.hexdigest())
        records.append(row)
    write_json(output / 'assets.json', {'schema_version': 1, 'reference_root': str(REFERENCE), 'files': records})
    write_json(output / 'source_configurations.json', configurations)


def snapshot():
    memory = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree'):
            memory[key + '_kib'] = int(value.split()[0])
    processes = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            command = (directory / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            stat = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            status = (directory / 'status').read_text().splitlines()
            rss = next((int(x.split()[1]) for x in status if x.startswith('VmRSS:')), 0)
            comm = (directory / 'comm').read_text().strip()
            component = 'other'
            if comm not in ('bash', 'sh', 'rg'):
                if 'scripts/voice/p1_benchmark.py' in command:
                    component = 'p1_benchmark'
                else:
                    component = next((name for name, marker in (('qwen', 'qwen3_vl_worker'), ('blue_cube', 'blue_cube_localizer'), ('move_group', '/move_group'), ('rviz', '/rviz2'), ('camera', 'component_container'), ('operator_gui', 'p10_operator_gui.py'), ('asr', 'sherpa-onnx')) if marker in command), 'other')
            processes.append({'pid': int(directory.name), 'comm': comm,
                              'rss_kib': rss, 'cpu_ticks': int(stat[11]) + int(stat[12]),
                              'start_ticks': int(stat[19]),
                              'component': component})
        except (OSError, ValueError, StopIteration):
            continue
    try:
        npu = {'raw': Path('/sys/kernel/debug/rknpu/load').read_text()}
    except OSError as error:
        npu = {'raw': None, 'unavailable_reason': str(error)}
    return {'monotonic_ns': time.monotonic_ns(), 'wall_time_ns': time.time_ns(), 'memory': memory,
            'npu': npu, 'processes': processes}


def sample(output, duration, interval):
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    deadline = time.monotonic() + duration
    with (output / 'resources.jsonl').open('w') as handle:
        while True:
            row = snapshot()
            rows.append(row)
            handle.write(json.dumps(row) + '\n')
            handle.flush()
            if time.monotonic() >= deadline:
                break
            time.sleep(min(interval, max(0, deadline - time.monotonic())))
    write_json(output / 'resource_summary.json', {
        'samples': len(rows), 'duration_s': (rows[-1]['monotonic_ns'] - rows[0]['monotonic_ns']) / 1e9,
        'minimum_available_mib': min(r['memory']['MemAvailable_kib'] for r in rows) / 1024,
        'maximum_swap_used_mib': max(r['memory']['SwapTotal_kib'] - r['memory']['SwapFree_kib'] for r in rows) / 1024,
        'cpu_ticks_per_second': os.sysconf('SC_CLK_TCK'),
        'npu_available': any(r['npu']['raw'] is not None for r in rows),
        'note': 'RSS is per process, not additive physical memory; CPU = delta ticks / HZ / elapsed seconds; sampled peaks may miss short transients.'})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('inventory', 'sample'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--interval', type=float, default=0.5)
    args = parser.parse_args()
    if args.duration <= 0 or args.interval <= 0:
        parser.error('duration and interval must be positive')
    if args.mode == 'inventory':
        inventory(args.output)
    else:
        sample(args.output, args.duration, args.interval)
