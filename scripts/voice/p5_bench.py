#!/usr/bin/env python3
"""P5 PCM-level benchmark for Chinese TTS backends, with no audio playback.

The reference project's "streaming" Kokoro turned out to deliver one callback
per text segment *after* the whole segment was synthesised, so "first PCM" and
"segment synthesis time" were the same number.  This script therefore measures
the callback structure explicitly instead of trusting the callback API name:

* ``tts_first_callback`` - when the engine first handed us any samples
* ``tts_first_valid_pcm`` - when it first handed us samples whose leading
  frames are actually audible (>= -45 dBFS sustained for >= 20 ms), which is
  what the product objective is really about
* ``generation_end`` - totals, RTF, and the numerical health gate

Cold start is measured in a child process so that model load time is separate
from first-segment latency: the product objective is "first playable text to
audible PCM", and a cold load must not be smuggled into it.

Never plays audio and never issues robot commands.
"""
import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import resource
import statistics
import subprocess
import sys
import time
import wave

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = Path(os.environ.get('FR3C_QWEN_ASSET_ROOT', '/home/industio/DofbotPro_JetsonNano_0106'))
CONFIG = ROOT / 'config/voice/tts_candidates.json'
TEXTS = ROOT / 'config/voice/p5_texts.json'
ASSETS = ROOT / 'assets/voice/tts'
CPU_TICKS_PER_SECOND = os.sysconf('SC_CLK_TCK')

# Audibility gate, in dBFS relative to full scale.
VALID_FRAME_MS = 10
VALID_ONSET_DBFS = -45.0
VALID_ONSET_FRAMES = 2

sys.dont_write_bytecode = True


def load_json(path):
    return json.loads(Path(path).read_text())


# --------------------------------------------------------------------------- #
# backends
# --------------------------------------------------------------------------- #

@dataclass
class EngineSpec:
    name: str
    engine: object
    sample_rate: int
    num_speakers: int
    family: str
    assets: dict


def build_engine(candidate_name, candidate, config, threads, asset_root):
    """Instantiate the sherpa-onnx engine described by the registry entry."""
    import sherpa_onnx

    family = candidate['family']
    root = Path(asset_root)
    missing = [asset['name'] for asset in candidate['assets'] if not (root / asset['name']).is_file()]
    if missing:
        raise RuntimeError(f'{candidate_name}: missing assets {missing}; run p5_fetch_models.py first')

    def path(name):
        return os.fspath(root / name)

    model_kwargs = {}
    dict_dir = os.fspath(root / candidate['dict_dir']) if candidate.get('dict_dir') else ''
    if family == 'kokoro':
        # Kokoro needs espeak-ng data plus lexicons; it has no jieba dict.
        model_kwargs['kokoro'] = sherpa_onnx.OfflineTtsKokoroModelConfig(
            model=path('model.onnx'),
            voices=path('voices.bin'),
            tokens=path('tokens.txt'),
            data_dir=path(candidate.get('espeak_data_dir', 'espeak-ng-data')),
            lexicon=','.join(path(name) for name in ('lexicon-us-en.txt', 'lexicon-zh.txt')
                             if (root / name).is_file()),
        )
    elif family == 'matcha':
        vocoder = candidate['vocoder']['name']
        model_kwargs['matcha'] = sherpa_onnx.OfflineTtsMatchaModelConfig(
            acoustic_model=path('model-steps-3.onnx'),
            vocoder=path(vocoder),
            tokens=path('tokens.txt'),
            lexicon=path('lexicon.txt'),
            dict_dir=dict_dir,
        )
    elif family == 'vits':
        model_name = 'model.onnx'
        for asset in candidate['assets']:
            if asset['name'].endswith('.onnx') and 'int8' not in asset['name']:
                model_name = asset['name']
                break
        model_kwargs['vits'] = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=path(model_name),
            tokens=path('tokens.txt'),
            lexicon=path('lexicon.txt') if (root / 'lexicon.txt').is_file() else '',
            dict_dir=dict_dir,
        )
    else:
        raise RuntimeError(f'{candidate_name}: unsupported family {family}')

    model_config = sherpa_onnx.OfflineTtsModelConfig(
        **model_kwargs, num_threads=threads, provider='cpu', debug=False,
    )
    rules = [path(name) for name in candidate.get('rule_fsts', []) if (root / name).is_file()]
    tts_config = sherpa_onnx.OfflineTtsConfig(
        model=model_config,
        rule_fsts=','.join(rules),
        max_num_sentences=1,
    )
    if not tts_config.validate():
        raise RuntimeError(f'{candidate_name}: OfflineTtsConfig failed validation')
    engine = sherpa_onnx.OfflineTts(tts_config)
    return EngineSpec(
        name=candidate_name, engine=engine, sample_rate=int(engine.sample_rate),
        num_speakers=int(engine.num_speakers), family=family,
        assets={'rule_fsts': [Path(name).name for name in rules],
                'model': Path(model_name if family == 'vits' else
                              'model-steps-3.onnx' if family == 'matcha' else 'model.onnx').name},
    )


# --------------------------------------------------------------------------- #
# measurement
# --------------------------------------------------------------------------- #

def speech_onset_seconds(pcm16, sample_rate):
    """Seconds until the first frame that is audibly non-silent.

    Returns ``None`` when the whole buffer is below the onset threshold, which
    means the backend produced something that would be heard as silence.
    """
    if pcm16.size == 0:
        return None
    frame = max(1, int(sample_rate * VALID_FRAME_MS / 1000))
    usable = pcm16[: (pcm16.size // frame) * frame].reshape(-1, frame).astype(np.float64)
    if usable.size == 0:
        return None
    rms = np.sqrt(np.mean(usable ** 2, axis=1))
    with np.errstate(divide='ignore'):
        dbfs = 20.0 * np.log10(np.maximum(rms, 1e-12) / 32768.0)
    loud = dbfs >= VALID_ONSET_DBFS
    for index in range(len(loud) - VALID_ONSET_FRAMES + 1):
        if loud[index:index + VALID_ONSET_FRAMES].all():
            return index * frame / sample_rate
    return None


class Trace:
    def __init__(self, path):
        self.handle = path.open('w')
        self.path = path

    def emit(self, request_id, event, **details):
        record = {
            'schema_version': 1,
            'run_id': self.path.parent.name,
            'request_id': request_id,
            'event': event,
            'clock': 'CLOCK_MONOTONIC',
            'monotonic_ns': time.monotonic_ns(),
            'wall_time_ns': time.time_ns(),
            'pid': os.getpid(),
            **details,
        }
        self.handle.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.handle.flush()

    def close(self):
        self.handle.close()


def synthesize_measured(spec, text, speaker_id, speed, trace, request_id, **context):
    """Run one synthesis, recording the callback structure and health gate."""
    started = time.monotonic()
    cpu_started = time.process_time()
    callbacks = []
    raw_blocks = []
    state = {'error': None}
    trace.emit(request_id, 'tts_generate_begin', characters=len(text), **context)
    def on_samples(samples, progress):
        if state['error'] is not None:
            return 0
        values = np.asarray(samples, dtype=np.float64)
        if values.ndim != 1 or values.size == 0:
            state['error'] = 'empty PCM callback'
            return 0
        non_finite = int(np.count_nonzero(~np.isfinite(values)))
        if non_finite:
            state['error'] = f'{non_finite}/{values.size} NaN/Inf samples'
            return 0
        elapsed = time.monotonic() - started
        pcm16 = np.rint(np.clip(values, -1.0, 1.0) * 32767.0).astype('<i2')
        callbacks.append({
            'index': len(callbacks),
            't_s': elapsed,
            'samples': int(pcm16.size),
            'progress': float(progress),
            'first_valid_pcm_s': speech_onset_seconds(pcm16, spec.sample_rate),
        })
        raw_blocks.append(pcm16)
        return 1

    spec.engine.generate(text=text, sid=speaker_id, speed=speed, callback=on_samples)
    synthesis_s = time.monotonic() - started
    cpu_s = time.process_time() - cpu_started

    if state['error'] is not None:
        raise RuntimeError(f'{request_id}: {state["error"]}')

    if not raw_blocks:
        raise RuntimeError(f'{request_id}: engine produced no PCM at all')
    pcm16 = np.concatenate(raw_blocks)
    audio_s = pcm16.size / spec.sample_rate
    peak = float(np.max(np.abs(pcm16)))
    rms = float(np.sqrt(np.mean(pcm16.astype(np.float64) ** 2)))

    # A whole-buffer onset is still reported when it is late, so a backend that
    # renders seconds of leading silence is visible rather than "fast".
    onset = speech_onset_seconds(pcm16, spec.sample_rate)
    first_callback = callbacks[0]
    # Onset measured from the start of generation, not from the first callback:
    # late callbacks are exactly the failure mode this phase exists to expose.
    absolute_first_valid = (first_callback['t_s'] + first_callback['first_valid_pcm_s']
                            if first_callback['first_valid_pcm_s'] is not None else None)

    result = {
        'characters': len(text),
        'synthesis_s': synthesis_s,
        'audio_duration_s': audio_s,
        'rtf': synthesis_s / audio_s if audio_s else None,
        'cpu_s': cpu_s,
        'cpu_ratio': cpu_s / synthesis_s if synthesis_s else None,
        'first_callback_s': first_callback['t_s'],
        'first_callback_samples': first_callback['samples'],
        'first_valid_pcm_s': absolute_first_valid,
        'whole_audio_onset_s': onset,
        'callback_count': len(callbacks),
        'callback_latencies_s': [round(item['t_s'], 6) for item in callbacks],
        'incremental_delivery': len(callbacks) > 1,
        'peak_dbfs': 20.0 * float(np.log10(max(peak, 1e-9) / 32768.0)),
        'rms_dbfs': 20.0 * float(np.log10(max(rms, 1e-12) / 32768.0)),
        'clipped_samples': int(np.count_nonzero(np.abs(pcm16) >= 32767)),
        'nan_or_inf_samples': 0,
        'sample_rate': spec.sample_rate,
        'peak_rss_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    }
    trace.emit(request_id, 'tts_first_callback', latency_s=first_callback['t_s'],
               samples=first_callback['samples'], **context)
    if absolute_first_valid is not None:
        trace.emit(request_id, 'tts_first_valid_pcm', latency_s=absolute_first_valid,
                   onset_threshold_dbfs=VALID_ONSET_DBFS, **context)
    else:
        result['health'] = 'silent_audio'
    trace.emit(request_id, 'generation_end', **result, **context)
    return result, pcm16


def write_wav(path, pcm16, sample_rate):
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(os.fspath(path), 'wb') as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(sample_rate)
        handle.writeframes(pcm16.tobytes())


# --------------------------------------------------------------------------- #
# cold start, measured in a child process
# --------------------------------------------------------------------------- #

def cold_start_child(args):
    """Child mode: load once, synthesise one text, report the breakdown.

    The child writes its own trace file; the parent's ``trace.jsonl`` is never
    truncated by a child.
    """
    config = load_json(CONFIG)
    candidate = config['candidates'][args.candidate]
    trace = Trace(args.output / 'cold_child_trace.jsonl')
    load_started = time.monotonic()
    spec = build_engine(args.candidate, candidate, config, args.threads, ASSETS / args.candidate)
    load_s = time.monotonic() - load_started
    trace.emit('cold', 'model_loaded', candidate=args.candidate, threads=args.threads,
               load_s=load_s, sample_rate=spec.sample_rate, num_speakers=spec.num_speakers)
    text = args.cold_text
    result, _ = synthesize_measured(spec, text, args.speaker_id, args.speed, trace, 'cold-generate',
                                    phase='cold', source='child_process')
    print(json.dumps({'load_s': load_s, 'result': result}, ensure_ascii=False))
    trace.close()


def measure_cold(args, trace, repeats, speaker_id):
    """Spawn one child per repeat and read back its JSON line."""
    rows = []
    for index in range(repeats):
        child_trace = trace.path.parent / f'cold_child_{index}.jsonl'
        command = [
            sys.executable, os.fspath(Path(__file__).resolve()),
            '--candidate', args.candidate, '--output', os.fspath(trace.path.parent),
            '--child-cold', '--threads', str(args.threads), '--speed', str(args.speed),
            '--speaker-id', str(speaker_id), '--cold-text', args.cold_text,
            '--affinity', args.affinity,
        ]
        started = time.monotonic()
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
        if completed.returncode != 0:
            raise RuntimeError(f'cold child failed: {completed.stderr.strip()[-800:]}')
        payload = json.loads(completed.stdout.strip().splitlines()[-1])
        payload['wall_s'] = time.monotonic() - started
        payload['index'] = index
        rows.append(payload)
        trace.emit(f'cold-{index}', 'cold_start', **payload)
    return rows


# --------------------------------------------------------------------------- #
# sweep
# --------------------------------------------------------------------------- #

def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1 - weight) + ordered[upper] * weight


def summarize(rows):
    """P50/P95 plus a hard health verdict; a silent candidate fails outright."""
    verdict = {'health': 'ok', 'problems': []}
    for row in rows:
        if row.get('nan_or_inf_samples'):
            verdict['problems'].append(f'{row["request_id"]}: NaN/Inf samples')
        if row.get('whole_audio_onset_s') is None:
            verdict['problems'].append(f'{row["request_id"]}: whole utterance below onset threshold')
    if verdict['problems']:
        verdict['health'] = 'failed'
    rtf = [row['rtf'] for row in rows if row.get('rtf')]
    first = [row['first_valid_pcm_s'] for row in rows if row.get('first_valid_pcm_s') is not None]
    summary = {
        'samples': len(rows),
        'health': verdict['health'],
        'problems': verdict['problems'],
        'rtf_p50': percentile(rtf, 0.50),
        'rtf_p95': percentile(rtf, 0.95),
        'rtf_mean': statistics.fmean(rtf) if rtf else None,
        'first_valid_pcm_p50': percentile(first, 0.50),
        'first_valid_pcm_p95': percentile(first, 0.95),
        'first_valid_pcm_observed': first,
        'incremental_delivery_any': any(row.get('incremental_delivery') for row in rows),
        'max_callback_count': max((row.get('callback_count', 0) for row in rows), default=0),
        'peak_rss_kib': max((row.get('peak_rss_kib', 0) for row in rows), default=0),
    }
    return summary


def by_characters(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row['characters'], []).append(row)
    table = {}
    for characters, items in sorted(grouped.items()):
        first = [item['first_valid_pcm_s'] for item in items if item.get('first_valid_pcm_s') is not None]
        rtf = [item['rtf'] for item in items if item.get('rtf')]
        table[str(characters)] = {
            'runs': len(items),
            'first_valid_pcm_p50': percentile(first, 0.50),
            'first_valid_pcm_p95': percentile(first, 0.95),
            'rtf_p50': percentile(rtf, 0.50),
            'rtf_p95': percentile(rtf, 0.95),
            'audio_duration_p50': percentile([item['audio_duration_s'] for item in items], 0.50),
        }
    return table


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--speaker-id', type=int, default=None)
    parser.add_argument('--affinity', default='default', choices=('default', 'big', 'little'),
                        help='big = cores 4-7 (A76), little = cores 0-3 (A55)')
    parser.add_argument('--cold-repeats', type=int, default=2)
    parser.add_argument('--cold-text', default='好的我知道了')
    parser.add_argument('--child-cold', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--skip-cold', action='store_true')
    parser.add_argument('--texts', default=None, help='override text id list, comma separated')
    args = parser.parse_args()

    if args.affinity == 'big' and hasattr(os, 'sched_setaffinity'):
        os.sched_setaffinity(0, {4, 5, 6, 7})
    elif args.affinity == 'little' and hasattr(os, 'sched_setaffinity'):
        os.sched_setaffinity(0, {0, 1, 2, 3})
    os.environ.setdefault('OMP_NUM_THREADS', str(args.threads))
    os.environ.setdefault('ORT_NUM_THREADS', str(args.threads))

    args.output.mkdir(parents=True, exist_ok=True)
    trace_path = args.output / 'trace.jsonl'
    if args.child_cold:
        cold_start_child(args)
        return

    if trace_path.exists():
        parser.error(f'{args.output} already has a trace; choose a new run directory')

    config = load_json(CONFIG)
    if args.candidate not in config['candidates']:
        parser.error(f'unknown candidate {args.candidate}')
    candidate = config['candidates'][args.candidate]
    speaker_id = args.speaker_id if args.speaker_id is not None else config['speaker_ids'].get(args.candidate, 0)
    if speaker_id is None:
        speaker_id = 0

    texts = load_json(TEXTS)
    wanted = set(args.texts.split(',')) if args.texts else None
    corpus = []
    for category in texts['measurement_texts']['categories']:
        for item in texts['categories'][category]['texts']:
            if wanted is None or item['id'] in wanted:
                corpus.append((category, item))

    trace = Trace(trace_path)
    trace.emit('environment', 'run_begin', candidate=args.candidate, threads=args.threads,
               affinity=args.affinity, repeats=args.repeats, speed=args.speed,
               speaker_id=speaker_id, python=sys.version.split()[0],
               sherpa_onnx=__import__('sherpa_onnx').__version__,
               numpy=np.__version__, corpus=[item['id'] for _, item in corpus])
    try:
        load_started = time.monotonic()
        spec = build_engine(args.candidate, candidate, config, args.threads, ASSETS / args.candidate)
        load_s = time.monotonic() - load_started
        trace.emit('environment', 'model_loaded', load_s=load_s, sample_rate=spec.sample_rate,
                   num_speakers=spec.num_speakers, model=spec.assets['model'],
                   rule_fsts=spec.assets['rule_fsts'])
        print(f'{args.candidate}: load {load_s:.2f}s, sample_rate {spec.sample_rate}, '
              f'speakers {spec.num_speakers}')
        if speaker_id >= spec.num_speakers:
            parser.error(f'speaker_id {speaker_id} outside model range 0..{spec.num_speakers - 1}')

        rows = []
        warmup, _ = synthesize_measured(spec, '好的', speaker_id, args.speed, trace, 'warmup',
                                        phase='warmup', category='warmup')
        for category, item in corpus:
            for repeat in range(args.repeats):
                request_id = f'{item["id"]}#{repeat}'
                result, pcm16 = synthesize_measured(
                    spec, item['text'], speaker_id, args.speed, trace, request_id,
                    phase='warm', category=category, text_id=item['id'],
                )
                result['request_id'] = request_id
                result['category'] = category
                result['text_id'] = item['id']
                rows.append(result)
                if repeat == 0:
                    write_wav(args.output / 'audio' / f'{item["id"]}.wav', pcm16, spec.sample_rate)
            print(f'  {item["id"]:<10} chars={len(item["text"]):>3} '
                  f'first_valid={rows[-1]["first_valid_pcm_s"]} rtf={rows[-1]["rtf"]:.3f}')

        cold_rows = [] if args.skip_cold else measure_cold(args, trace, args.cold_repeats, speaker_id)
        summary = {
            'schema_version': 1,
            'candidate': args.candidate,
            'role': candidate.get('role'),
            'family': candidate['family'],
            'license': candidate.get('license'),
            'threads': args.threads,
            'affinity': args.affinity,
            'speed': args.speed,
            'speaker_id': speaker_id,
            'sample_rate': spec.sample_rate,
            'num_speakers': spec.num_speakers,
            'load_s': load_s,
            'warm': summarize(rows),
            'by_characters': by_characters(rows),
            'cold': {
                'repeats': len(cold_rows),
                'load_s': [round(row['load_s'], 4) for row in cold_rows],
                'load_s_p50': percentile([row['load_s'] for row in cold_rows], 0.50),
                'wall_s': [round(row['wall_s'], 4) for row in cold_rows],
                'first_valid_pcm_s': [row['result'].get('first_valid_pcm_s') for row in cold_rows],
            },
        }
        (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({key: summary[key] for key in ('candidate', 'threads', 'affinity', 'load_s')},
                         ensure_ascii=False))
        print(json.dumps(summary['warm'], ensure_ascii=False, indent=2))
    finally:
        trace.close()


if __name__ == '__main__':
    main()
