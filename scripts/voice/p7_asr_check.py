#!/usr/bin/env python3
"""Exercise the real ASR worker with WAV files, or record without sending actions."""
import argparse
import audioop
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import wave

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/fr3c_voice'))
from fr3c_voice.voice_input import VoiceInput


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wav', type=Path, action='append', default=[])
    parser.add_argument('--rounds', type=int, default=1)
    parser.add_argument('--output', type=Path, default=ROOT / 'evidence/voice/P7/asr-check.jsonl')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((ROOT / 'config/voice/asr.json').read_text())
    if not args.wav:
        done = threading.Event()
        def emit(event):
            print(json.dumps(event, ensure_ascii=False), flush=True)
            with args.output.open('a') as out:
                out.write(json.dumps(event, ensure_ascii=False) + '\n')
            if event[0] in ('asr_final', 'asr_empty', 'asr_error'):
                done.set()
        voice = VoiceInput(ROOT, emit)
        try:
            for _ in range(args.rounds):
                done.clear()
                voice.start()
                if not done.wait(50):
                    raise RuntimeError('capture timeout')
                voice._thread.join(1)
        finally:
            voice.close()
        return
    root = Path(cfg['reference_root'])
    lib = root / cfg['c_api_library']
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = f"{root / cfg['runtime_directory']}:{lib.parent}"
    command = [sys.executable, str(ROOT / 'src/fr3c_voice/fr3c_voice/asr_worker.py'),
               '--hotwords-file', str(ROOT / 'config/voice/asr_hotwords.txt'),
               '--hotwords-score', str(cfg.get('hotwords_score', 2.0)),
               '--c-api-library', str(lib), '--model-dir', str(root / cfg['model_directory']),
               '--speech-endpoint-seconds', str(cfg['trailing_silence_s']), '--chunk-samples', '800']
    for round_id in range(args.rounds):
        path = args.wav[round_id % len(args.wav)]
        with wave.open(str(path)) as wav:
            assert wav.getsampwidth() == 2 and wav.getnchannels() == 1
            pcm, _ = audioop.ratecv(wav.readframes(wav.getnframes()), 2, 1,
                                   wav.getframerate(), 16000, None)
        started = time.monotonic()
        result = subprocess.run(command, input=pcm, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, env=env, timeout=20)
        records = []
        for line in result.stdout.splitlines():
            try:
                records.append(json.loads(line))
            except ValueError:
                pass
        final = [e for e in records if e.get('event') == 'final']
        record = dict(round=round_id, wav=str(path), wall_s=time.monotonic()-started,
                      returncode=result.returncode, events=records,
                      unique_final=len(final) == 1)
        with args.output.open('a') as out:
            out.write(json.dumps(record, ensure_ascii=False) + '\n')
        print(json.dumps(record, ensure_ascii=False), flush=True)
        if result.returncode or len(final) != 1:
            raise RuntimeError(result.stderr.decode(errors='replace')[-1000:])


if __name__ == '__main__':
    main()
