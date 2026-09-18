#!/usr/bin/env python3
"""P5 summary: turn the raw evidence into the candidate comparison table.

Reads whatever exists and says so when something is missing, rather than
silently reporting a partial comparison as complete.
"""
import argparse
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config/voice/tts_candidates.json'

# Acceptance objectives from the phase plan; recorded as objectives, not results.
RTF_TARGET_P95 = 0.8
FIRST_SEGMENT_TARGET_P95 = 0.8
# First segments in the product are short by construction; this is the longest
# one the P5 measurements support and the bound the operating point uses.
OPERATING_MAX_CHARACTERS = 24


def load(path):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='evidence run directory, e.g. evidence/voice/P5/20260917')
    args = parser.parse_args()
    run = args.run
    config = json.loads(CONFIG.read_text())

    bench_dir = run / 'bench'
    summaries = {}
    for candidate_dir in sorted(bench_dir.glob('*')) if bench_dir.is_dir() else []:
        summary = load(candidate_dir / 'summary.json')
        if summary:
            summaries[summary['candidate']] = summary

    calibration = load(run / 'acoustic_calibration.json') or load(run / 'acoustic' / 'calibration.json')
    loads = {}
    for level in ('tts', 'qwen', 'full'):
        payload = load(run / f'load_{level}' / 'load.json')
        if payload:
            loads[level] = payload

    pronunciation = load(run / 'pronunciation' / 'pronunciation.json')
    stability = load(run / 'stability' / 'stability.json')
    acoustic = load(run / 'acoustic_report.json')

    operating_max = OPERATING_MAX_CHARACTERS
    rows = []
    for name, summary in summaries.items():
        entry = config['candidates'].get(name, {})
        warm = summary['warm']
        operating = {characters: stats for characters, stats in summary['by_characters'].items()
                     if int(characters) <= operating_max}
        operating_first = [stats['first_valid_pcm_p95'] for stats in operating.values()
                           if stats['first_valid_pcm_p95'] is not None]
        operating_rtf = [stats['rtf_p95'] for stats in operating.values()
                         if stats['rtf_p95'] is not None]
        operating_point = {
            'max_first_segment_characters': operating_max,
            'runs': sum(stats['runs'] for stats in operating.values()),
            'first_valid_pcm_p95_worst_s': max(operating_first) if operating_first else None,
            'rtf_p95_worst': max(operating_rtf) if operating_rtf else None,
            'first_segment_target_met': (bool(operating_first)
                                         and max(operating_first) <= FIRST_SEGMENT_TARGET_P95),
            'rtf_target_met': (bool(operating_rtf)
                               and max(operating_rtf) <= RTF_TARGET_P95),
        }
        rows.append({
            'candidate': name,
            'order': entry.get('order'),
            'role': entry.get('role'),
            'family': summary['family'],
            'license': summary.get('license'),
            'sample_rate': summary['sample_rate'],
            'num_speakers': summary['num_speakers'],
            'load_s': summary['load_s'],
            'cold_load_s_p50': summary['cold'].get('load_s_p50'),
            'rtf_p50': warm['rtf_p50'],
            'rtf_p95': warm['rtf_p95'],
            'first_valid_pcm_p50': warm['first_valid_pcm_p50'],
            'first_valid_pcm_p95': warm['first_valid_pcm_p95'],
            'incremental_delivery': warm['incremental_delivery_any'],
            'max_callback_count': warm['max_callback_count'],
            'peak_rss_mib': warm['peak_rss_kib'] / 1024,
            'health': warm['health'],
            'problems': warm['problems'],
            'rtf_target_met_whole_corpus': (warm['rtf_p95'] is not None
                                            and warm['rtf_p95'] <= RTF_TARGET_P95),
            # The whole-corpus first-segment P95 includes the 120-character item,
            # which is never a first segment in the product. The operating point
            # below is the honest verdict; both are reported.
            'first_segment_target_met_whole_corpus': (
                warm['first_valid_pcm_p95'] is not None
                and warm['first_valid_pcm_p95'] <= FIRST_SEGMENT_TARGET_P95),
            'operating_point': operating_point,
            'by_characters': summary['by_characters'],
        })
    rows.sort(key=lambda item: (item['rtf_p95'] is None, item['rtf_p95'] or 99))

    # The first-segment crossing point: the largest character count whose P95
    # still meets the objective. This is the number the P6 segmenter needs.
    for row in rows:
        crossing = None
        for characters, stats in sorted(row['by_characters'].items(), key=lambda kv: int(kv[0])):
            if stats['first_valid_pcm_p95'] is not None and stats['first_valid_pcm_p95'] <= FIRST_SEGMENT_TARGET_P95:
                crossing = int(characters)
        row['max_characters_meeting_first_segment_target'] = crossing

    payload = {
        'schema_version': 1,
        'run': run.name,
        'objectives': {'rtf_p95_max': RTF_TARGET_P95, 'first_segment_p95_max_s': FIRST_SEGMENT_TARGET_P95},
        'candidates_measured': len(rows),
        'comparison': rows,
        'acoustic_calibration': calibration,
        'acoustic_report': acoustic,
        'stability': {
            name: {'verdict': entry['verdict'],
                   'quiet_rate': entry['overall_quiet_rate'],
                   'items_unstable': entry['items_unstable']}
            for name, entry in (stability or {}).get('candidates', {}).items()
        } or None,
        'load': {level: {'memory': item['memory'], 'per_candidate': item['per_candidate']}
                 for level, item in loads.items()},
        'pronunciation': {
            name: {
                'sample_rate': entry['sample_rate'],
                'num_speakers': entry['num_speakers'],
                'items_with_warnings': entry['items_with_warnings'],
                'items_silent': entry['items_silent'],
                'clipped_items': entry['clipped_items'],
                'items': len(entry['items']),
            } for name, entry in (pronunciation or {}).get('candidates', {}).items()
        } or None,
    }
    (run / 'summary.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')

    # Console table, P95 first because that is the acceptance statistic.
    stability_map = {name: entry['verdict']
                     for name, entry in (stability or {}).get('candidates', {}).items()}
    header = (f'{"候选":<26}{"sr":>6}{"load":>7}{"RTFp50":>8}{"RTFp95":>8}'
              f'{"首段p50":>9}{"首段p95":>9}{"RSS/MiB":>9}{"RTF达标":>8}{"首段达标":>9}'
              f'{"首段字上限":>11}{"稳定性":>10}')
    print(f'运行点 = 首段 <= {OPERATING_MAX_CHARACTERS} 字；达标列按运行点判定\n')
    print(header)
    print('-' * len(header))
    for row in rows:
        verdict = stability_map.get(row['candidate'], '未测')
        print(f'{row["candidate"]:<26}{row["sample_rate"]:>6}{row["load_s"]:>7.2f}'
              f'{(row["rtf_p50"] or 0):>8.3f}{(row["rtf_p95"] or 0):>8.3f}'
              f'{(row["first_valid_pcm_p50"] or 0):>9.3f}{(row["first_valid_pcm_p95"] or 0):>9.3f}'
              f'{row["peak_rss_mib"]:>9.0f}'
              f'{"是" if row["operating_point"]["rtf_target_met"] else "否":>8}'
              f'{"是" if row["operating_point"]["first_segment_target_met"] else "否":>9}'
              f'{str(row["max_characters_meeting_first_segment_target"]):>11}'
              f'{verdict:>10}')
    if calibration:
        print(f'\n声学固定偏置: {calibration["offset_s_p50"] * 1000:.1f} ms '
              f'(范围 {calibration["offset_spread_ms"]:.1f} ms, SNR {calibration["snr_db_p50"]:.1f} dB)')
    print(f'\nwrote {run}/summary.json')


if __name__ == '__main__':
    main()
