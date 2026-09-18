#!/usr/bin/env python3
"""Interactive listening bench for the streaming speech pipeline.

Type a sentence, hear it, and see the timeline. Designed for a human at the
bench, not for automated acceptance:

* ``<text>``        speak the text through the real pipeline
* ``q <prompt>``    ask Qwen and speak its answer as it streams
* ``c <text>``      play a pre-generated phrase (no synthesis)
* ``r <text>``      repeat the text 3 times to produce a long answer
* ``r <N> <text>``  repeat the text N times
* ``s``             stop the current playback and report how long it took
* ``v <0-100>``     set the speaker volume
* ``l``             list pre-generated phrases
* ``t``             show the last timing summary
* ``h`` / ``?``     help
* ``x``             quit (services keep running; use stop_voice_demo.sh)

The stop latency printed here is measured from the stop request to the device
being silenced, which is the same definition the phase-6 acceptance uses.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_PIPELINE_SOCKET = WORKSPACE / 'evidence/voice/P6/runtime/fr3c-tts-pipeline.sock'
DEFAULT_QWEN_SOCKET = WORKSPACE / 'evidence/voice/P3/runtime/fr3c-qwen-service.sock'

HELP = __doc__


def call(socket_path: Path, payload: dict, timeout: float = 600) -> list[dict]:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(socket_path))
        connection.sendall(json.dumps(payload, ensure_ascii=False).encode('utf-8') + b'\n')
        records = []
        with connection.makefile('r', encoding='utf-8') as reader:
            for line in reader:
                records.append(json.loads(line))
                if records[-1].get('event') in ('result', 'error', 'stopped'):
                    break
        return records


def show_timing(final: dict) -> None:
    def value(key, digits=3):
        item = final.get(key)
        return '—' if item is None else f'{item:.{digits}f}'

    if not final or final.get('playback_end_s') is None:
        print('   （本次没有可显示的播放指标）')
        return
    gaps = final.get('inter_segment_gaps_s') or []
    print(f"   首段 {value('first_segment_s')}s | 首声写入 {value('first_valid_pcm_write_s')}s | "
          f"文本结束 {value('text_end_s')}s | 播放结束 {value('playback_end_s')}s")
    print(f"   段数 {final.get('segment_count')} | 音频 {value('audio_seconds_written', 2)}s | "
          f"xrun {final.get('xruns')} | 队列阻塞 {final.get('pcm_queue_full_waits')}")
    if gaps:
        print(f"   段间间隔 最大 {max(gaps) * 1000:.1f}ms | 超150ms {sum(1 for g in gaps if g > 0.15)} 次")
    if final.get('audio_started_before_text_end') is not None:
        early = final['audio_started_before_text_end']
        print(f"   音频早于文本结束: {'是' if early else '否'}"
              + ('' if early or final.get('text_end_s') is None
                 else f"（文本 {final['text_end_s']:.2f}s 结束，首声 {final['first_valid_pcm_write_s']:.2f}s）"))
    if final.get('failure'):
        print(f"   !! 失败: {final['failure']}")


class Bench:
    def __init__(self, pipeline_socket: Path, qwen_socket: Path, volume: int | None) -> None:
        self.pipeline_socket = pipeline_socket
        self.qwen_socket = qwen_socket
        self.last: dict = {}
        self.session: threading.Thread | None = None
        self.request_id = ''
        if volume is not None:
            self.set_volume(volume)

    # -- helpers ---------------------------------------------------------- #

    def set_volume(self, percent: int) -> None:
        subprocess.run(['amixer', '-c', '2', 'sset', 'Speaker', f'{percent}%'],
                       capture_output=True)
        print(f'   扬声器音量 -> {percent}%')

    def status(self) -> dict:
        records = call(self.pipeline_socket, {'op': 'status'}, timeout=30)
        return records[0] if records else {}

    def speak(self, text: str, cached: bool = False, wait: bool = True) -> dict:
        request_id = f'demo-{int(time.time() * 1000)}'
        self.request_id = request_id
        call(self.pipeline_socket,
             {'op': 'speak_start', 'request_id': request_id, 'replace': True}, timeout=60)
        if cached:
            # A cached phrase is short and finishes almost immediately, so
            # `speak_end` must be sent synchronously: if it is handed to a
            # background thread the session is already gone by the time that
            # thread runs.
            call(self.pipeline_socket,
                 {'op': 'speak_cached', 'request_id': request_id, 'text': text}, timeout=30)
            records = call(self.pipeline_socket,
                           {'op': 'speak_end', 'request_id': request_id, 'timeout_s': 600},
                           timeout=660)
            final = next((r for r in reversed(records) if r.get('event') == 'result'), {})
            self.last = self.merge_status(final)
            return self.last
        for index in range(0, len(text), 24):
            call(self.pipeline_socket,
                 {'op': 'speak_text', 'request_id': request_id,
                  'text': text[index:index + 24]}, timeout=60)
        if not wait:
            return {}
        records = call(self.pipeline_socket,
                       {'op': 'speak_end', 'request_id': request_id, 'timeout_s': 600},
                       timeout=660)
        final = next((r for r in reversed(records) if r.get('event') == 'result'), {})
        self.last = self.merge_status(final)
        return self.last

    def merge_status(self, final: dict) -> dict:
        """Fill in metrics that the result record cannot carry.

        A pre-generated phrase has no segments, so the result record is nearly
        empty; the authoritative numbers always live in the session metrics.
        """
        metrics = self.status().get('last_metrics') or {}
        merged = {key: value for key, value in final.items() if value is not None}
        merged.update(metrics)
        return merged

    def speak_qwen(self, prompt: str) -> dict:
        request_id = f'demo-q-{int(time.time() * 1000)}'
        self.request_id = request_id
        # A new question interrupts whatever is still being spoken: that is the
        # behaviour a real caller expects, and it keeps the metrics clean.
        call(self.pipeline_socket,
             {'op': 'speak_start', 'request_id': request_id, 'replace': True}, timeout=60)
        first_token = None
        started = time.monotonic()

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(300)
            connection.connect(str(self.qwen_socket))
            connection.sendall(json.dumps(
                {'op': 'chat_text', 'prompt': prompt, 'request_id': f'{request_id}-qwen'},
                ensure_ascii=False).encode('utf-8') + b'\n')
            answer = ''
            with connection.makefile('r', encoding='utf-8') as reader:
                for line in reader:
                    record = json.loads(line)
                    event = record.get('event')
                    if event == 'token':
                        if first_token is None:
                            first_token = time.monotonic() - started
                            print(f'   [Qwen 首 token {first_token:.3f}s]')
                        token = str(record.get('text') or '')
                        answer += token
                        call(self.pipeline_socket,
                             {'op': 'speak_text', 'request_id': request_id, 'text': token},
                             timeout=60)
                    elif event == 'result':
                        break
                    elif event == 'error':
                        raise RuntimeError(record.get('error', 'qwen error'))
        print(f'   [Qwen 文本 {len(answer)} 字，生成耗时 {time.monotonic() - started:.2f}s]')
        print(f'   回答: {answer}')
        records = call(self.pipeline_socket,
                       {'op': 'speak_end', 'request_id': request_id, 'timeout_s': 600},
                       timeout=660)
        final = next((r for r in reversed(records) if r.get('event') == 'result'), {})
        final = self.merge_status(final)
        final['qwen_first_token_s'] = first_token
        self.last = final
        return final

    def stop(self) -> None:
        started = time.monotonic()
        records = call(self.pipeline_socket, {'op': 'stop', 'reason': 'demo'}, timeout=30)
        elapsed = (time.monotonic() - started) * 1000
        record = records[0] if records else {}
        print(f'   停止请求往返 {elapsed:.1f}ms，设备清空耗时 '
              f'{(record.get("drop_s") or 0) * 1000:.2f}ms')
        for _ in range(40):
            status = self.status()
            if not status.get('session_running'):
                break
            time.sleep(0.05)
        metrics = status.get('last_metrics') or {}
        if metrics.get('stop_latency_s') is not None:
            print(f'   停播延迟（请求→设备静音）{metrics["stop_latency_s"] * 1000:.2f}ms')
        self.last = metrics

    def wait_async(self) -> None:
        """Collect the result of a non-blocking speak in a background thread."""
        request_id = self.request_id

        def collect() -> None:
            records = call(self.pipeline_socket,
                           {'op': 'speak_end', 'request_id': request_id, 'timeout_s': 600},
                           timeout=660)
            final = next((r for r in reversed(records) if r.get('event') == 'result'), {})
            self.last = self.merge_status(final)
            print('\n   [播放结束]')
            final = self.last
            show_timing(final)
            print('> ', end='', flush=True)
        self.session = threading.Thread(target=collect, daemon=True)
        self.session.start()


def main() -> int:
    parser = argparse.ArgumentParser(description='interactive listening bench')
    parser.add_argument('--pipeline-socket', type=Path, default=DEFAULT_PIPELINE_SOCKET)
    parser.add_argument('--qwen-socket', type=Path, default=DEFAULT_QWEN_SOCKET)
    parser.add_argument('--volume', type=int, default=None,
                        help='set the speaker volume before starting, e.g. 45')
    args = parser.parse_args()

    if not args.pipeline_socket.exists():
        print(f'语音服务未运行（找不到 {args.pipeline_socket}）。\n'
              f'先执行: scripts/voice/start_voice_demo.sh', file=sys.stderr)
        return 1

    bench = Bench(args.pipeline_socket, args.qwen_socket, args.volume)
    status = bench.status()
    print(f"候选 {status.get('candidate')} | 采样率 {status.get('sample_rate')} | "
          f"绑核 {status.get('affinity')} | 设备缓冲 {status.get('device_buffer_seconds')}s")
    print(f"预生成短语: {bench_status_cached(bench)}")
    print('输入 h 查看帮助。直接输入文字即可试听。\n')

    while True:
        try:
            line = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        command, _, rest = line.partition(' ')
        command = command.lower()
        try:
            if command in ('x', 'quit', 'exit'):
                break
            if command in ('h', '?', 'help'):
                print(HELP)
            elif command == 's':
                bench.stop()
            elif command == 't':
                if bench.last:
                    show_timing(bench.last)
                else:
                    print('   还没有可显示的结果')
            elif command == 'l':
                print('   已预生成: ' + cached_phrase_list(bench))
            elif command == 'v':
                bench.set_volume(int(rest))
            elif command == 'q':
                if not bench.qwen_socket.exists():
                    print('   Qwen 服务未运行，无法使用 q 模式')
                    continue
                bench.speak_qwen(rest)
                show_timing(bench.last)
            elif command == 'c':
                print(f'   预生成播放（无合成）: {rest!r}' if rest else '   预生成播放: 需要文本')
                bench.speak(rest, cached=True)
                show_timing(bench.last)
            elif command == 'r':
                count_text = rest.split(' ', 1)
                if len(count_text) == 2 and count_text[0].isdigit():
                    text = (count_text[1] + ' ') * int(count_text[0])
                else:
                    text = rest * 3
                print(f'   合成 {len(text)} 字')
                bench.speak(text)
                show_timing(bench.last)
            else:
                # Bare text: speak it, but hand the wait to a thread so the
                # prompt stays responsive and `s` can interrupt.
                bench.speak(line, wait=False)
                bench.wait_async()
        except Exception as error:  # noqa: BLE001 - interactive session
            print(f'   出错: {type(error).__name__}: {error}')
    print('退出。服务仍在运行；停止请执行 scripts/voice/stop_voice_demo.sh')
    return 0


def bench_status_cached(bench: Bench) -> str:
    status = bench.status()
    return f"{status.get('cached_phrases')} 条"


def cached_phrase_list(bench: Bench) -> str:
    records = call(bench.pipeline_socket, {'op': 'preload', 'texts': []}, timeout=30)
    cached = (records[0].get('cached') if records else None) or []
    return ', '.join(cached) if cached else '（无）'


if __name__ == '__main__':
    raise SystemExit(main())
