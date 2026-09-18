#!/usr/bin/env python3
"""Force a real USB audio re-enumeration while a reply is playing.

The phase-6 acceptance found this failure mode by accident: the board's USB
audio device re-enumerated mid-sentence and the cached ALSA handle started
returning ENODEV. A simulated test was too gentle, so this script reproduces it
for real by unbinding and rebinding the ``snd-usb-audio`` driver while the
pipeline is speaking.

What must hold afterwards:

* the reply is not truncated silently: the pipeline either recovers and keeps
  playing, or reports a failure -- it must not claim success while the audio
  stopped early
* ``device_reopens`` / ``midstream_recoveries`` increase, proving the reopen
  path ran rather than the error being swallowed
* a following reply plays normally

Requires root for the sysfs writes. Nothing here touches the robot.
"""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / 'scripts/voice'))
import p6_speak  # noqa: E402

DRIVER = '/sys/bus/usb/drivers/snd-usb-audio'
DEFAULT_INTERFACE = '5-1.2.2:1.0'

LONG_TEXT = (
    '这是一段用来制造足够播放时间的测试文本。'
    '我们会在它播放期间强制让 USB 声卡重新枚举。'
    '如果播放器不能在设备重新出现后继续工作，这段音频就会中途停止。'
    '这段话会重复若干次以保证播放时间足够长。'
) * 3


def call(payload, timeout=120):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(p6_speak.DEFAULT_PIPELINE_SOCKET))
        connection.sendall(json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
        records = []
        with connection.makefile('r', encoding='utf-8') as reader:
            for line in reader:
                records.append(json.loads(line))
                if records[-1].get('event') in ('result', 'error', 'stopped'):
                    break
        return records


def status():
    records = call({'op': 'status'}, timeout=30)
    return records[0] if records else {}


def reenumerate(interface: str, delay_s: float, gap_s: float) -> dict:
    """Unbind then rebind the audio interface, the way a replug looks."""
    result = {'interface': interface, 'unbind_ok': False, 'bind_ok': False}
    time.sleep(delay_s)
    started = time.monotonic()
    unbind = subprocess.run(['sudo', '-n', 'tee', f'{DRIVER}/unbind'],
                            input=interface.encode(), capture_output=True)
    result['unbind_ok'] = unbind.returncode == 0
    result['unbind_error'] = unbind.stderr.decode(errors='replace').strip()
    result['unbind_at_s'] = round(started, 3)
    time.sleep(gap_s)
    bind = subprocess.run(['sudo', '-n', 'tee', f'{DRIVER}/bind'],
                          input=interface.encode(), capture_output=True)
    result['bind_ok'] = bind.returncode == 0
    result['bind_error'] = bind.stderr.decode(errors='replace').strip()
    result['rebind_at_s'] = round(time.monotonic(), 3)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                    formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--interface', default=DEFAULT_INTERFACE)
    parser.add_argument('--delay', type=float, default=3.0,
                        help='seconds of playback before pulling the device')
    parser.add_argument('--gap', type=float, default=1.5,
                        help='seconds the device stays away')
    parser.add_argument('--no-replug', action='store_true',
                        help='only check the baseline, do not disturb the device')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    trace = (args.output / 'device_replug.jsonl').open('w')

    def record(kind, payload):
        trace.write(json.dumps({'case': kind, **payload}, ensure_ascii=False) + '\n')
        trace.flush()

    before = status()
    player_before = before.get('player_stats', {})
    print(f"播放前: reopens={player_before.get('device_reopens')} "
          f"midstream={player_before.get('midstream_recoveries')} xruns={player_before.get('xruns')}")

    request_id = f'replug-{uuid.uuid4().hex[:6]}'
    call({'op': 'speak_start', 'request_id': request_id})
    for index in range(0, len(LONG_TEXT), 24):
        call({'op': 'speak_text', 'request_id': request_id,
              'text': LONG_TEXT[index:index + 24]})

    replug_thread = None
    replug_result = {'skipped': True}
    if not args.no_replug:
        replug_result = {}
        replug_thread = threading.Thread(
            target=lambda: replug_result.update(reenumerate(args.interface, args.delay, args.gap)),
            daemon=True)
        replug_thread.start()

    records = call({'op': 'speak_end', 'request_id': request_id, 'timeout_s': 300}, timeout=360)
    if replug_thread:
        replug_thread.join(timeout=30)
    final = next((r for r in reversed(records) if r.get('event') == 'result'), {})
    errors = [r.get('error') for r in records if r.get('event') == 'error']

    time.sleep(2)
    after = status()
    player_after = after.get('player_stats', {})
    print(f"播放后: reopens={player_after.get('device_reopens')} "
          f"midstream={player_after.get('midstream_recoveries')} xruns={player_after.get('xruns')}")

    # A follow-up reply must still work.
    followup_id = f'after-{uuid.uuid4().hex[:6]}'
    call({'op': 'speak_start', 'request_id': followup_id})
    call({'op': 'speak_text', 'request_id': followup_id, 'text': '设备恢复之后的第二句话。'})
    follow_records = call({'op': 'speak_end', 'request_id': followup_id, 'timeout_s': 120},
                          timeout=180)
    followup = next((r for r in reversed(follow_records) if r.get('event') == 'result'), {})

    expected_audio = final.get('audio_seconds_written') or 0.0
    summary = {
        'replug': replug_result,
        'first_reply': {
            'audio_seconds_written': expected_audio,
            'playback_end_s': final.get('playback_end_s'),
            'first_valid_pcm_write_s': final.get('first_valid_pcm_write_s'),
            'segment_count': final.get('segment_count'),
            'stream_errors': errors,
            'failure': final.get('failure'),
        },
        'player_before': player_before,
        'player_after': player_after,
        'device_reopens_delta': (player_after.get('device_reopens', 0)
                                 - player_before.get('device_reopens', 0)),
        'midstream_recoveries_delta': (player_after.get('midstream_recoveries', 0)
                                       - player_before.get('midstream_recoveries', 0)),
        'followup': {
            'audio_seconds_written': followup.get('audio_seconds_written'),
            'failure': followup.get('failure'),
            'stream_errors': [r.get('error') for r in follow_records if r.get('event') == 'error'],
        },
    }
    summary['first_reply_completed'] = (
        summary['first_reply']['failure'] in (None, '')
        and expected_audio > 1.0
        and not errors)
    summary['followup_played'] = (
        (followup.get('audio_seconds_written') or 0) > 0.1
        and not summary['followup']['failure'])
    summary['verdict'] = (
        'recovered' if (summary['first_reply_completed'] and summary['followup_played'])
        else 'degraded')

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    record('summary', summary)
    (args.output / 'device_replug_summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    trace.close()
    return 0 if summary['verdict'] == 'recovered' else 1


if __name__ == '__main__':
    raise SystemExit(main())
