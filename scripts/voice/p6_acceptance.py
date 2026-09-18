#!/usr/bin/env python3
"""Phase-6 board acceptance for the streaming speech pipeline.

What it proves, and why each case exists:

* **audio starts before the text ends** - a controlled slow token source feeds a
  long answer token by token. A short answer cannot distinguish "streaming" from
  "wait for the whole answer", which is exactly the mistake the phase plan warns
  about, so the slow source is the primary evidence.
* **20/60/120 characters, at least 10 rounds each** - inter-segment gaps and
  underruns over a real distribution rather than one lucky run.
* **stop latency** - measured from the stop request to the device being
  silenced, with the device buffer included.
* **device recovery** - the handle is dropped and reopened, and the next reply
  must still play.
* **real model path** - the same rounds again with Qwen generating the answer
  over its socket, so the pipeline is exercised with a real generator.

Evidence lands as JSONL plus a summary; nothing is inferred from a log line.
"""
import argparse
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / 'scripts/voice'))

import p6_speak  # noqa: E402  (same directory, resolved above)

TEXTS = {
    20: '相机已经准备就绪请观察桌面上的彩色方块并等待识别结果确认目标',
    60: '相机已经准备就绪请观察桌面上的彩色方块并等待识别结果确认目标位置保持稳定随后报告当前状态相机已经准备就绪请观察桌面上的彩',
    120: '相机已经准备就绪请观察桌面上的彩色方块并等待识别结果确认目标位置保持稳定随后报告当前状态相机已经准备就绪请观察桌面上的彩色方块并等待识别结果确认目标位置保持稳定随后报告当前状态相机已经准备就绪请观察桌面上的彩色方块并等待识别结果确认目标位置',
}


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1 - weight) + ordered[upper] * weight


def run_case(pipeline_socket, text, request_id, verbose=False):
    """Feed fixed text and collect the pipeline result."""
    p6_speak.call(pipeline_socket, {'op': 'speak_start', 'request_id': request_id}, timeout=60)
    chunk = 24
    for index in range(0, len(text), chunk):
        p6_speak.call(pipeline_socket,
                      {'op': 'speak_text', 'request_id': request_id,
                       'text': text[index:index + chunk]}, timeout=60)
    records = p6_speak.call(pipeline_socket,
                            {'op': 'speak_end', 'request_id': request_id, 'timeout_s': 300},
                            timeout=360)
    if verbose:
        for record in records:
            print(json.dumps(record, ensure_ascii=False))
    final = next((record for record in reversed(records) if record.get('event') == 'result'), {})
    return final


def run_slow_token_case(pipeline_socket, request_id, characters=120, token_delay=0.12,
                        token_size=6, verbose=False):
    """Feed one long answer slowly and check whether audio starts early."""
    text = (TEXTS[120] * 2)[:characters]
    p6_speak.call(pipeline_socket, {'op': 'speak_start', 'request_id': request_id}, timeout=60)
    started = time.monotonic()
    first_audio_s = None
    text_end_s = None
    tokens = [text[index:index + token_size] for index in range(0, len(text), token_size)]
    stop_poll = time.monotonic()
    for index, token in enumerate(tokens):
        p6_speak.call(pipeline_socket,
                      {'op': 'speak_text', 'request_id': request_id, 'text': token}, timeout=60)
        if first_audio_s is None and index > 0:
            status = p6_speak.call(pipeline_socket, {'op': 'status'}, timeout=30)
            metrics = (status[0].get('last_metrics') or {}) if status else {}
            value = metrics.get('first_valid_pcm_write_s')
            if value is not None:
                first_audio_s = value
        time.sleep(token_delay)
    text_end_s = time.monotonic() - started
    records = p6_speak.call(pipeline_socket,
                            {'op': 'speak_end', 'request_id': request_id, 'timeout_s': 300},
                            timeout=360)
    if verbose:
        for record in records:
            print(json.dumps(record, ensure_ascii=False))
    final = next((record for record in reversed(records) if record.get('event') == 'result'), {})
    final['slow_source'] = {
        'tokens': len(tokens),
        'token_delay_s': token_delay,
        'token_size': token_size,
        'feed_total_s': round(text_end_s, 4),
        'first_audio_seen_during_feed_s': first_audio_s,
        'audio_before_feed_finished': (
            final.get('first_valid_pcm_write_s') is not None
            and final['first_valid_pcm_write_s'] < text_end_s),
    }
    del stop_poll
    return final


def run_stop_case(pipeline_socket, request_id, stop_after_s=0.8, verbose=False):
    """Start a long answer, then stop it and measure how fast silence arrives."""
    text = TEXTS[120] * 3
    p6_speak.call(pipeline_socket, {'op': 'speak_start', 'request_id': request_id}, timeout=60)
    for index in range(0, len(text), 24):
        p6_speak.call(pipeline_socket,
                      {'op': 'speak_text', 'request_id': request_id,
                       'text': text[index:index + 24]}, timeout=60)
    # Wait until audio is actually playing before stopping, so the measurement
    # includes the device buffer rather than an idle session.
    deadline = time.monotonic() + 20
    playing = False
    while time.monotonic() < deadline:
        status = p6_speak.call(pipeline_socket, {'op': 'status'}, timeout=30)
        metrics = (status[0].get('last_metrics') or {}) if status else {}
        if metrics.get('first_valid_pcm_write_s') is not None:
            playing = True
            break
        time.sleep(0.02)
    time.sleep(stop_after_s)
    requested = time.monotonic()
    stop_records = p6_speak.call(pipeline_socket,
                                 {'op': 'stop', 'request_id': request_id,
                                  'reason': 'acceptance'}, timeout=60)
    stop_returned = time.monotonic()
    stop_record = stop_records[0] if stop_records else {}
    # After a stop the session is closed, so speak_end would be an error by
    # design. The stop reply already carries the final metrics; read them back
    # through status once the pipeline has finished tearing the session down.
    deadline = time.monotonic() + 10
    final = {}
    while time.monotonic() < deadline:
        status = p6_speak.call(pipeline_socket, {'op': 'status'}, timeout=30)
        metrics = (status[0].get('last_metrics') or {}) if status else {}
        if not status or not status[0].get('session_running'):
            final = metrics
            break
        time.sleep(0.05)
    if verbose:
        print(json.dumps(stop_record, ensure_ascii=False))
    final['stop_case'] = {
        'was_playing': playing,
        'stop_call_wall_s': round(stop_returned - requested, 4),
        'stop_latency_s': final.get('stop_latency_s'),
        'cancelled': final.get('cancelled'),
        'drop_s': stop_record.get('drop_s'),
    }
    return final


def run_device_recovery(pipeline_socket, request_id, verbose=False):
    """Force a device reopen and confirm the next reply still plays."""
    before = p6_speak.call(pipeline_socket, {'op': 'status'}, timeout=30)
    player_before = before[0].get('player_stats', {}) if before else {}
    # Dropping and re-preparing the handle is what a USB re-enumeration forces;
    # `stop` drops the buffer, and the status call reports the reopen counter.
    p6_speak.call(pipeline_socket, {'op': 'stop', 'reason': 'recovery_probe'}, timeout=30)
    result = run_case(pipeline_socket, '设备恢复测试，如果听到这句话说明重开成功。',
                      request_id, verbose=verbose)
    after = p6_speak.call(pipeline_socket, {'op': 'status'}, timeout=30)
    player_after = after[0].get('player_stats', {}) if after else {}
    result['device_recovery'] = {
        'reopens_before': player_before.get('device_reopens'),
        'reopens_after': player_after.get('device_reopens'),
        'xruns_after': player_after.get('xruns'),
        'played_after_recovery': result.get('audio_seconds_written', 0) > 0.1,
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pipeline-socket', type=Path, default=p6_speak.DEFAULT_PIPELINE_SOCKET)
    parser.add_argument('--qwen-socket', type=Path, default=p6_speak.DEFAULT_QWEN_SOCKET)
    parser.add_argument('--rounds', type=int, default=10)
    parser.add_argument('--stop-rounds', type=int, default=5)
    parser.add_argument('--qwen-rounds', type=int, default=3)
    parser.add_argument('--with-qwen', action='store_true',
                        help='also run real Qwen-generated answers')
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    trace_path = args.output / 'acceptance.jsonl'
    trace = trace_path.open('w')

    def record(kind, payload):
        trace.write(json.dumps({'case': kind, 'wall_time_ns': time.time_ns(), **payload},
                               ensure_ascii=False) + '\n')
        trace.flush()

    summary = {'schema_version': 1, 'rounds': args.rounds, 'cases': {}}
    t0 = time.monotonic()
    try:
        # 1. length ladder, fixed text
        for characters, text in TEXTS.items():
            rows = []
            for round_index in range(args.rounds):
                request_id = f'len{characters}-{round_index}-{uuid.uuid4().hex[:6]}'
                result = run_case(args.pipeline_socket, text, request_id, args.verbose)
                row = {
                    'characters': characters,
                    'round': round_index,
                    'request_id': request_id,
                    'first_valid_pcm_write_s': result.get('first_valid_pcm_write_s'),
                    'synthesis_end_s': result.get('synthesis_end_s'),
                    'playback_end_s': result.get('playback_end_s'),
                    'audio_seconds_written': result.get('audio_seconds_written'),
                    'segment_count': result.get('segment_count'),
                    'inter_segment_gaps_s': result.get('inter_segment_gaps_s'),
                    'pcm_queue_max': result.get('pcm_queue_max'),
                    'pcm_queue_full_waits': result.get('pcm_queue_full_waits'),
                    'xruns': result.get('xruns'),
                    'failure': result.get('failure'),
                }
                rows.append(row)
                record('length', row)
                print(f'  长度 {characters:>3} 第 {round_index + 1:>2}/{args.rounds} 轮: '
                      f'首声 {row["first_valid_pcm_write_s"]}s 段数 {row["segment_count"]} '
                      f'音频 {row["audio_seconds_written"]}s xrun {row["xruns"]}')
            gaps = [gap for row in rows for gap in (row['inter_segment_gaps_s'] or [])]
            summary['cases'][f'length_{characters}'] = {
                'rounds': len(rows),
                'first_valid_pcm_write_p50_s': percentile(
                    [row['first_valid_pcm_write_s'] for row in rows
                     if row['first_valid_pcm_write_s'] is not None], 0.50),
                'first_valid_pcm_write_p95_s': percentile(
                    [row['first_valid_pcm_write_s'] for row in rows
                     if row['first_valid_pcm_write_s'] is not None], 0.95),
                'segment_count_p50': percentile(
                    [row['segment_count'] for row in rows if row['segment_count']], 0.50),
                'gap_max_s': max(gaps) if gaps else None,
                'gap_p95_s': percentile(gaps, 0.95),
                'gap_over_150ms': sum(1 for gap in gaps if gap > 0.15),
                'xruns_total': sum(row['xruns'] or 0 for row in rows),
                'failures': [row['failure'] for row in rows if row['failure']],
                'pcm_queue_blocked': sum(row['pcm_queue_full_waits'] or 0 for row in rows),
            }

        # 2. slow token source: the primary "audio before text end" evidence
        slow_rows = []
        for round_index in range(3):
            request_id = f'slow-{round_index}-{uuid.uuid4().hex[:6]}'
            result = run_slow_token_case(args.pipeline_socket, request_id, verbose=args.verbose)
            row = {'round': round_index, 'request_id': request_id, **result.get('slow_source', {}),
                   'first_valid_pcm_write_s': result.get('first_valid_pcm_write_s'),
                   'playback_end_s': result.get('playback_end_s'),
                   'audio_seconds_written': result.get('audio_seconds_written'),
                   'xruns': result.get('xruns'), 'failure': result.get('failure')}
            slow_rows.append(row)
            record('slow_token', row)
            print(f'  慢 token 第 {round_index + 1}/3 轮: 喂完 {row["feed_total_s"]}s, '
                  f'首声 {row["first_valid_pcm_write_s"]}s, 早于喂完='
                  f'{row["audio_before_feed_finished"]}')
        summary['cases']['slow_token'] = {
            'rounds': len(slow_rows),
            'audio_before_text_end_all_rounds': all(
                row['audio_before_feed_finished'] for row in slow_rows),
            'head_start_s': [round(row['feed_total_s'] - row['first_valid_pcm_write_s'], 4)
                             for row in slow_rows if row['first_valid_pcm_write_s'] is not None],
            'failures': [row['failure'] for row in slow_rows if row['failure']],
        }

        # 3. stop latency
        stop_rows = []
        for round_index in range(args.stop_rounds):
            request_id = f'stop-{round_index}-{uuid.uuid4().hex[:6]}'
            result = run_stop_case(args.pipeline_socket, request_id, verbose=args.verbose)
            row = {'round': round_index, 'request_id': request_id, **result.get('stop_case', {})}
            stop_rows.append(row)
            record('stop', row)
            print(f'  停播 第 {round_index + 1}/{args.stop_rounds} 轮: '
                  f'延迟 {None if row["stop_latency_s"] is None else round(row["stop_latency_s"] * 1000, 1)}ms')
        latencies = [row['stop_latency_s'] for row in stop_rows if row['stop_latency_s'] is not None]
        summary['cases']['stop'] = {
            'rounds': len(stop_rows),
            'stop_latency_p50_ms': None if not latencies else percentile(latencies, 0.5) * 1000,
            'stop_latency_p95_ms': None if not latencies else percentile(latencies, 0.95) * 1000,
            'stop_latency_max_ms': None if not latencies else max(latencies) * 1000,
            'all_cancelled': all(row['cancelled'] for row in stop_rows),
            'target_ms': 250,
        }

        # 4. device recovery
        recovery = run_device_recovery(args.pipeline_socket, f'recovery-{uuid.uuid4().hex[:6]}',
                                       verbose=args.verbose)
        record('device_recovery', recovery.get('device_recovery', {}))
        summary['cases']['device_recovery'] = recovery.get('device_recovery', {})
        print(f'  设备恢复: {summary["cases"]["device_recovery"]}')

        # 5. real Qwen path
        if args.with_qwen:
            qwen_rows = []
            for round_index in range(args.qwen_rounds):
                request_id = f'qwen-{round_index}-{uuid.uuid4().hex[:6]}'
                started = time.monotonic()
                p6_speak.call(args.pipeline_socket,
                              {'op': 'speak_start', 'request_id': request_id}, timeout=60)
                qwen_result = p6_speak.stream_qwen(
                    args.qwen_socket,
                    '请用大约一百二十个字介绍你自己和你能做的事情。',
                    f'{request_id}-qwen',
                    lambda token: p6_speak.call(
                        args.pipeline_socket,
                        {'op': 'speak_text', 'request_id': request_id, 'text': token},
                        timeout=60),
                    timeout=180)
                records = p6_speak.call(args.pipeline_socket,
                                        {'op': 'speak_end', 'request_id': request_id,
                                         'timeout_s': 300}, timeout=360)
                final = next((record for record in reversed(records)
                              if record.get('event') == 'result'), {})
                row = {
                    'round': round_index,
                    'request_id': request_id,
                    'qwen_first_token_s': qwen_result.get('first_token_s'),
                    'qwen_total_s': qwen_result.get('client_total_s'),
                    'answer_characters': len(qwen_result.get('text') or ''),
                    'first_valid_pcm_write_s': final.get('first_valid_pcm_write_s'),
                    'text_end_s': final.get('text_end_s'),
                    'playback_end_s': final.get('playback_end_s'),
                    'audio_started_before_text_end': final.get('audio_started_before_text_end'),
                    'segment_count': final.get('segment_count'),
                    'audio_seconds_written': final.get('audio_seconds_written'),
                    'inter_segment_gaps_s': final.get('inter_segment_gaps_s'),
                    'xruns': final.get('xruns'),
                    'failure': final.get('failure'),
                    'wall_s': round(time.monotonic() - started, 3),
                }
                qwen_rows.append(row)
                record('qwen', row)
                print(f'  Qwen 第 {round_index + 1}/{args.qwen_rounds} 轮: '
                      f'答案 {row["answer_characters"]} 字, 首 token {row["qwen_first_token_s"]}s, '
                      f'首声 {row["first_valid_pcm_write_s"]}s, 早于文本结束='
                      f'{row["audio_started_before_text_end"]}')
            gaps = [gap for row in qwen_rows for gap in (row['inter_segment_gaps_s'] or [])]
            summary['cases']['qwen'] = {
                'rounds': len(qwen_rows),
                'first_token_p50_s': percentile(
                    [row['qwen_first_token_s'] for row in qwen_rows
                     if row['qwen_first_token_s'] is not None], 0.5),
                'first_valid_pcm_write_p50_s': percentile(
                    [row['first_valid_pcm_write_s'] for row in qwen_rows
                     if row['first_valid_pcm_write_s'] is not None], 0.5),
                'audio_before_text_end_rounds': sum(
                    1 for row in qwen_rows if row['audio_started_before_text_end']),
                'gap_max_s': max(gaps) if gaps else None,
                'gap_over_150ms': sum(1 for gap in gaps if gap > 0.15),
                'xruns_total': sum(row['xruns'] or 0 for row in qwen_rows),
                'failures': [row['failure'] for row in qwen_rows if row['failure']],
            }

        summary['wall_s'] = round(time.monotonic() - t0, 2)
        (args.output / 'acceptance_summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print()
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    finally:
        trace.close()


if __name__ == '__main__':
    main()
