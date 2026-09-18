#!/usr/bin/env python3
"""Evaluate P4 text routing against Qwen without ever importing ROS Actions."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    workspace_default = Path(__file__).resolve().parents[2]
    parser.add_argument('--workspace', type=Path, default=workspace_default)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--corpus', type=Path)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument(
        '--case-id', action='append', default=[],
        help='evaluate only this case id; may be repeated')
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    sys.path.insert(0, str(workspace / 'src/fr3c_voice'))

    from fr3c_voice.config import load_config
    from fr3c_voice.intent import IntentProtocolError, classify_text
    from fr3c_voice.model_client import ModelServiceClient

    config_path = args.config or workspace / 'config/voice/qwen3vl.yaml'
    corpus_path = args.corpus or workspace / 'config/voice/p4_intent_corpus.json'
    corpus = json.loads(corpus_path.read_text(encoding='utf-8'))
    cases = corpus.get('cases', [])
    if args.case_id:
        wanted = set(args.case_id)
        cases = [case for case in cases if case.get('id') in wanted]
    if args.limit:
        cases = cases[:args.limit]
    if not cases:
        raise RuntimeError('intent corpus is empty')
    _qwen, service = load_config(config_path, workspace)
    client = ModelServiceClient(
        service.runtime_dir / service.service_socket_name,
        timeout_s=args.timeout)
    health = client.health()
    if not health.get('healthy'):
        raise RuntimeError(f'model service is not healthy: {health}')

    started = time.monotonic()
    results = []
    for index, case in enumerate(cases, start=1):
        row = {
            'id': case['id'], 'category': case['category'],
            'text': case['text'], 'expected': {
                key: case.get(key)
                for key in ('route', 'task', 'color', 'zone', 'support_color')},
        }
        try:
            intent, raw = classify_text(case['text'], client.generate)
            actual = {
                'route': intent.route, 'task': intent.task, 'color': intent.color,
                'zone': intent.zone, 'support_color': intent.support_color}
            row['actual'] = actual
            row['protocol_valid'] = True
            row['exact_match'] = actual == row['expected']
            row['unsafe_action'] = (
                case['route'] != 'action' and intent.route == 'action')
            row['raw'] = raw
        except IntentProtocolError as error:
            row.update({
                'protocol_valid': False, 'exact_match': False,
                'unsafe_action': False, 'error_code': error.code,
                'error': error.detail,
            })
        except Exception as error:
            row.update({
                'protocol_valid': False, 'exact_match': False,
                'unsafe_action': False,
                'error': f'{type(error).__name__}: {error}',
            })
        results.append(row)
        print(
            f'[{index:02d}/{len(cases):02d}] {case["id"]}: '
            f'{"PASS" if row["exact_match"] else "FAIL"}', flush=True)

    category_totals = Counter(item['category'] for item in results)
    category_passed = Counter(
        item['category'] for item in results if item['exact_match'])
    protocol_valid = sum(bool(item['protocol_valid']) for item in results)
    exact = sum(bool(item['exact_match']) for item in results)
    unsafe = sum(bool(item['unsafe_action']) for item in results)
    negative_cases = sum(
        item['expected']['route'] != 'action' for item in results)
    action_cases = [
        item for item in results if item['expected']['route'] == 'action']
    action_exact = sum(bool(item['exact_match']) for item in action_cases)
    summary = {
        'case_count': len(results),
        'protocol_valid': protocol_valid,
        'exact_matches': exact,
        'unsafe_negative_actions': unsafe,
        'negative_case_count': negative_cases,
        'action_case_count': len(action_cases),
        'action_exact_matches': action_exact,
        'elapsed_s': round(time.monotonic() - started, 3),
        'categories': {
            name: {'passed': category_passed[name], 'total': total}
            for name, total in sorted(category_totals.items())
        },
        'acceptance': {
            'at_least_60_cases': len(results) >= 60,
            'all_protocol_valid': protocol_valid == len(results),
            'all_action_parameters_exact': action_exact == len(action_cases),
            'no_negative_false_trigger': unsafe == 0,
        },
    }
    report = {
        'schema_version': 1,
        'generated_at': datetime.now().astimezone().isoformat(),
        'model_health_before': health,
        'summary': summary,
        'results': results,
    }
    output_dir = workspace / 'evidence/voice/P4/evaluation'
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f'intent-eval-{datetime.now():%Y%m%d_%H%M%S}.json'
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f'report={output_path}')
    return 0 if all(summary['acceptance'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
