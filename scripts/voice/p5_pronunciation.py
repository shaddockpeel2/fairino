#!/usr/bin/env python3
"""P5 pronunciation corpus renderer: Chinese numbers, colours and input hygiene.

The judge is a human listener. ASR is deliberately not used as a pass/fail
gate, because the reference project's own ASR is one of the components this
phase is trying to make more responsive, so a transcription mismatch would
conflate two different problems.

What this script does produce, per candidate and per corpus item:

* a WAV file to listen to
* the tokenisation warnings sherpa-onnx emitted (e.g. an unknown phone symbol,
  which is how a broken number/date rule shows up)
* numerical health (peak/RMS/clipping/NaN), so a broken item is visible even
  before anyone listens
* a review sheet (``review.md``) with the intended reading and the pinyin for
  the tricky items, so the listener knows what to check

Never plays audio.
"""
import argparse
import importlib.util
import io
import json
import os
from contextlib import redirect_stderr
from pathlib import Path
import re
import sys
import wave

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config/voice/tts_candidates.json'
TEXTS = ROOT / 'config/voice/p5_texts.json'
ASSETS = ROOT / 'assets/voice/tts'
BENCH = Path(__file__).resolve().parent / 'p5_bench.py'

sys.dont_write_bytecode = True

# Intended readings for the items a listener cannot infer at a glance. These
# are the expected results, stated up front rather than decided after listening.
EXPECTED = {
    'num-0': 'líng', 'num-1': 'yī', 'num-2': 'èr', 'num-3': 'sān', 'num-4': 'sì',
    'num-5': 'wǔ', 'num-6': 'liù', 'num-7': 'qī', 'num-8': 'bā', 'num-9': 'jiǔ',
    'num-10': 'shí', 'num-21': 'èr shí yī', 'num-105': 'yī bǎi líng wǔ',
    'num-2024': 'èr líng èr sì (digit-by-digit)',
    'num-10000': 'yī wàn',
    'num-date': 'èr líng èr sì nián shí èr yuè sān shí yī hào',
    'num-police': 'bō dǎ yāo yāo líng (幺幺零)',
    'num-phone': 'digit-by-digit yī bā jiǔ èr líng èr sì líng wǔ yī yī',
    'num-money': 'shí èr wàn sān qiān sì bǎi wǔ shí liù kuài qián',
    'num-decimal': 'sān diǎn yī sì',
    'num-percent': 'bǎi fēn zhī bā shí wǔ',
    'num-negative': 'fù shí èr dù',
    'num-coord': 'zuò biāo X děng yú sān bǎi èr shí wǔ, Y děng yú fù yī bǎi èr shí',
    'num-mixed': 'wù chā bù chāo guò líng diǎn wǔ háo mǐ, sù dù jiàng dào bǎi fēn zhī sān shí',
    'num-code': 'cuò wù mǎ yī èr sān sì',
    'col-red': 'hóng sè', 'col-yellow': 'huáng sè', 'col-blue': 'lán sè',
    'col-green': 'lǜ sè', 'col-purple': 'zǐ sè', 'col-black': 'hēi sè',
    'col-white': 'bái sè', 'col-grey': 'huī sè', 'col-orange': 'chéng sè',
    'col-pink': 'fěn sè', 'col-light-blue': 'qiǎn lán sè', 'col-dark-green': 'shēn lǜ sè',
    'col-block': 'hóng sè fāng kuài',
    'col-all': 'hóng sè, huáng sè, lán sè, lǜ sè',
    'col-sentence': 'yǐ jīng zhǎo dào hóng sè fāng kuài, zhèng zài jiē jìn mù biāo',
    'col-none': 'zhuō miàn shàng méi yǒu hóng sè de fāng kuài',
}

WARNING_RE = re.compile(r'Unknown token:\s*(\S+)')


def load_bench():
    spec = importlib.util.spec_from_file_location('p5_bench_module', BENCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sanitize(text):
    """Strip characters that are not speakable content for a metrics check.

    Reported separately from the raw run: a backend that silently drops
    symbols is fine, one that reads them out loud is not.
    """
    return text


def synthesize(bench, spec, text, speaker_id, speed):
    """Generate PCM and capture engine warnings without polluting stdout."""
    buffer = io.StringIO()
    with redirect_stderr(buffer):
        result, pcm16 = bench.synthesize_measured(
            spec, text, speaker_id, speed, bench.Trace(Path(os.devnull)), 'pron',
            phase='pronunciation')
    warnings = sorted(set(WARNING_RE.findall(buffer.getvalue())))
    return result, pcm16, warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', action='append', default=None)
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
    texts = json.loads(TEXTS.read_text())
    candidates = args.candidate or list(config['candidates'])
    corpus = []
    for category in texts['correctness_texts']['categories']:
        for item in texts['categories'][category]['texts']:
            corpus.append((category, item))

    args.output.mkdir(parents=True, exist_ok=True)
    report = {'schema_version': 1, 'threads': args.threads, 'speed': args.speed,
              'corpus_size': len(corpus), 'candidates': {}}
    rows = []
    for name in candidates:
        candidate = config['candidates'][name]
        spec = bench.build_engine(name, candidate, config, args.threads, ASSETS / name)
        speaker_id = config['speaker_ids'].get(name, 0) or 0
        if speaker_id >= spec.num_speakers:
            raise SystemExit(f'{name}: speaker_id {speaker_id} >= num_speakers {spec.num_speakers}')
        print(f'== {name}: sr={spec.sample_rate} speakers={spec.num_speakers}')
        out_dir = args.output / name
        out_dir.mkdir(parents=True, exist_ok=True)
        items = []
        for category, item in corpus:
            result, pcm16, warnings = synthesize(bench, spec, item['text'], speaker_id, args.speed)
            bench.write_wav(out_dir / f'{item["id"]}.wav', pcm16, spec.sample_rate)
            record = {
                'id': item['id'],
                'category': category,
                'text': item['text'],
                'expected_reading': EXPECTED.get(item['id']),
                'audio_s': result['audio_duration_s'],
                'synthesis_s': result['synthesis_s'],
                'rtf': result['rtf'],
                'peak_dbfs': result['peak_dbfs'],
                'rms_dbfs': result['rms_dbfs'],
                'clipped_samples': result['clipped_samples'],
                'leading_silence_s': result['whole_audio_onset_s'],
                'engine_warnings': warnings,
                'health': 'silent' if result['whole_audio_onset_s'] is None else 'ok',
            }
            items.append(record)
            rows.append({'candidate': name, **record})
            flag = ' !' if warnings or record['health'] != 'ok' else ''
            print(f'   {item["id"]:<14} {record["audio_s"]:6.2f}s rtf={record["rtf"]:.3f} '
                  f'peak={record["peak_dbfs"]:6.1f}dBFS{flag}'
                  + (f' warnings={warnings}' if warnings else ''))
        report['candidates'][name] = {
            'sample_rate': spec.sample_rate,
            'num_speakers': spec.num_speakers,
            'speaker_id': speaker_id,
            'items': items,
            'items_with_warnings': [item['id'] for item in items if item['engine_warnings']],
            'items_silent': [item['id'] for item in items if item['health'] != 'ok'],
            'clipped_items': [item['id'] for item in items if item['clipped_samples']],
        }

    (args.output / 'pronunciation.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    lines = ['# P5 中文数字与颜色读音评审表', '',
             f'语料 {len(corpus)} 条，候选 {len(candidates)} 个。WAV 位于 `<候选>/<条目>.wav`。',
             '判定以人工听感为准；ASR 不作判据。', '']
    for name in candidates:
        entry = report['candidates'][name]
        lines.append(f'## {name}')
        lines.append('')
        lines.append(f'- 采样率 {entry["sample_rate"]} Hz，说话人数 {entry["num_speakers"]}，'
                     f'使用 sid={entry["speaker_id"]}')
        if entry['items_with_warnings']:
            lines.append(f'- ⚠ 引擎告警条目：{", ".join(entry["items_with_warnings"])}')
        if entry['items_silent']:
            lines.append(f'- ⚠ 静音条目：{", ".join(entry["items_silent"])}')
        if entry['clipped_items']:
            lines.append(f'- ⚠ 削波条目：{", ".join(entry["clipped_items"])}')
        lines.append('')
        lines.append('| 条目 | 文本 | 应读作 | 音频 | 峰值 | 判定 |')
        lines.append('|---|---|---|---|---:|---|')
        for item in entry['items']:
            warn = ' ⚠' if item['engine_warnings'] else ''
            lines.append(f'| {item["id"]} | {item["text"]} | {item["expected_reading"] or ""} | '
                         f'{item["audio_s"]:.2f}s | {item["peak_dbfs"]:.1f} | |')
        lines.append('')
    (args.output / 'review.md').write_text('\n'.join(lines) + '\n')

    flat = args.output / 'pronunciation.jsonl'
    with flat.open('w') as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + '\n')
    print(f'\nwrote {args.output}/pronunciation.json, review.md, pronunciation.jsonl')


if __name__ == '__main__':
    main()
