#!/usr/bin/env python3
"""P5 acoustic measurement: real audible output, not just a software timestamp.

Why this is not simply "play the whole answer and listen":

* The room's own noise floor here is around -42 dBFS while speech sits near
  -37 dBFS at a comfortable volume, so a long playback measures mostly noise.
* Playing a 21 s test sentence out loud is antisocial on a shared bench.

So the tool separates two things that the original plan conflated:

1. ``calibrate`` derives the fixed *playback path offset* - the delay between
   handing PCM to ALSA and sound physically leaving the speaker - from a short
   gated pilot burst (well under three seconds of actual tone). The pilot's
   start is unambiguous in both the played and the captured signal, so the
   offset is a measurement, not an assumption.
2. ``report`` applies that offset to the software-level ``tts_first_valid_pcm``
   measurements from ``p5_bench.py`` to produce the acoustic first-sound
   figure, with the offset and its spread reported alongside so the arithmetic
   can be audited.

``verify`` is the optional middle ground: it plays one short text, once, to
prove the offset prediction actually matches reality and to check for real
dropouts (played audio loud, capture silent).

The mixer level is set explicitly and restored afterwards, so a measurement can
never leave the bench speaker loud. Requires /dev/snd; run from a normal board
terminal.
"""
import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import wave

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DEVICE = 'plughw:CARD=Device,DEV=0'
CARD = '2'
CAPTURE_RATE = 48000
CAPTURE_CHANNELS = 1

FRAME_MS = 10
# The audibility threshold is not a constant. This room's measured floor sits
# near -42 dBFS, so a fixed -45 dBFS gate would fire on noise and report a fake
# "instant" first sound. The gate is derived from a floor probe and recorded.
ONSET_DBFS_FLOOR = -45.0
ONSET_MARGIN_DB = 12.0
ONSET_DBFS_CEILING = -28.0
ONSET_FRAMES = 2
DROP_LOUD_DBFS = -38.0
DROP_SILENT_MARGIN_DB = 12.0

sys.dont_write_bytecode = True


def derive_onset_threshold(floor_envelope):
    """Audibility gate from the measured room floor, never a bare constant."""
    if floor_envelope.size == 0:
        return ONSET_DBFS_FLOOR, None, None
    floor_p95 = float(np.percentile(floor_envelope, 95))
    threshold = min(ONSET_DBFS_CEILING, max(ONSET_DBFS_FLOOR, floor_p95 + ONSET_MARGIN_DB))
    return threshold, floor_p95, floor_p95 - DROP_SILENT_MARGIN_DB


def frame_envelope(pcm16, sample_rate):
    frame = max(1, int(sample_rate * FRAME_MS / 1000))
    usable = pcm16[: (pcm16.size // frame) * frame].reshape(-1, frame).astype(np.float64)
    if usable.size == 0:
        return np.empty(0)
    return 20.0 * np.log10(np.maximum(np.sqrt(np.mean(usable ** 2, axis=1)), 1e-12) / 32768.0)


def onset_seconds(envelope, threshold, hold=ONSET_FRAMES):
    loud = envelope >= threshold
    for index in range(max(0, len(loud) - hold + 1)):
        if loud[index:index + hold].all():
            return index * FRAME_MS / 1000.0
    return None


class Mixer:
    """Set the USB speaker level explicitly and restore it afterwards."""

    def __init__(self, card=CARD, control='Speaker'):
        self.card = card
        self.control = control
        self.original = None

    def _get(self):
        completed = subprocess.run(['amixer', '-c', self.card, 'sget', self.control],
                                   capture_output=True, text=True)
        if completed.returncode != 0:
            return None
        for line in completed.stdout.splitlines():
            if 'Front Left: Playback' in line:
                return line.split('Playback', 1)[1].strip()
        return None

    def _set(self, value):
        subprocess.run(['amixer', '-c', self.card, 'sset', self.control, value],
                       capture_output=True, text=True)

    def __enter__(self):
        self.original = self._get()
        return self

    def set_percent(self, percent):
        self._set(f'{percent}%')

    def restore(self):
        if self.original:
            # amixer accepts the raw index; take the leading integer.
            index = self.original.split()[0]
            self._set(index)

    def __exit__(self, *exc):
        self.restore()
        return False


class Capture:
    """arecord for a whole number of seconds, started before playback."""

    def __init__(self, device, seconds, path):
        self.device = device
        self.seconds = int(seconds)
        self.path = path
        self.process = None

    def start(self):
        self.process = subprocess.Popen(
            ['arecord', '-D', self.device, '-f', 'S16_LE', '-r', str(CAPTURE_RATE),
             '-c', str(CAPTURE_CHANNELS), '-d', str(self.seconds), '-t', 'wav',
             os.fspath(self.path)],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        return self

    def wait(self, timeout):
        _, stderr = self.process.communicate(timeout=timeout)
        if self.process.returncode != 0:
            raise RuntimeError(f'arecord failed: {stderr.decode(errors="replace").strip()}')
        with wave.open(os.fspath(self.path)) as handle:
            rate = handle.getframerate()
            data = np.frombuffer(handle.readframes(handle.getnframes()), dtype='<i2')
        return data, rate


def play(device, wav_path, blocking=True):
    started_ns = time.monotonic_ns()
    process = subprocess.Popen(['aplay', '-q', '-D', device, os.fspath(wav_path)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    if blocking:
        _, stderr = process.communicate()
        if process.returncode != 0:
            raise RuntimeError(f'aplay failed: {stderr.decode(errors="replace").strip()}')
    return process, started_ns


def write_wav(path, pcm16, sample_rate):
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(os.fspath(path), 'wb') as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(sample_rate)
        handle.writeframes(np.asarray(pcm16, dtype='<i2').tobytes())


def pilot_burst(rate=CAPTURE_RATE, pre_silence_ms=500, tone_ms=250, gap_ms=350, repeats=3,
                freq=1000.0, amplitude=0.4):
    """Short gated pilot: unambiguous onset, minimal total sound energy."""
    total_ms = pre_silence_ms + repeats * (tone_ms + gap_ms)
    signal = np.zeros(int(rate * total_ms / 1000), dtype=np.float64)
    t = np.arange(int(rate * tone_ms / 1000)) / rate
    carrier = amplitude * np.sin(2 * np.pi * freq * t)
    for index in range(repeats):
        begin = int(rate * (pre_silence_ms + index * (tone_ms + gap_ms)) / 1000)
        signal[begin:begin + carrier.size] = carrier
    return np.rint(signal * 32767).astype('<i2')


def calibrate(device, workdir, mixer_percent, repeats=3):
    """Measure the fixed playback path offset with a short pilot burst."""
    workdir.mkdir(parents=True, exist_ok=True)
    pilot = pilot_burst()
    pilot_path = workdir / 'pilot.wav'
    write_wav(pilot_path, pilot, CAPTURE_RATE)

    records = []
    with Mixer() as mixer:
        mixer.set_percent(mixer_percent)
        time.sleep(0.3)
        for index in range(repeats):
            floor_path = workdir / f'pilot_floor_{index}.wav'
            record_path = workdir / f'pilot_rec_{index}.wav'
            floor, _ = Capture(device, 1, floor_path).start().wait(timeout=15)
            floor_env = frame_envelope(floor, CAPTURE_RATE)
            threshold, floor_p95, _ = derive_onset_threshold(floor_env)

            capture = Capture(device, 5, record_path).start()
            time.sleep(0.3)
            process, _ = play(device, pilot_path, blocking=False)
            process.communicate(timeout=15)
            captured, rate = capture.wait(timeout=15)

            played_env = frame_envelope(pilot, CAPTURE_RATE)
            captured_env = frame_envelope(captured, rate)
            played_onset = onset_seconds(played_env, threshold=-60.0, hold=1)
            captured_onset = onset_seconds(captured_env, threshold)
            if captured_onset is None:
                raise RuntimeError('pilot not detected in the capture; check speaker and microphone')
            pre_roll_s = 0.3
            records.append({
                'index': index,
                'pre_roll_s': pre_roll_s,
                'played_onset_s': played_onset,
                'captured_onset_s': captured_onset,
                'offset_s': captured_onset - pre_roll_s - played_onset,
                'threshold_dbfs': threshold,
                'floor_p95_dbfs': floor_p95,
                'pilot_peak_dbfs': float(np.max(played_env)),
                'captured_peak_dbfs': float(np.max(captured_env)),
                'snr_db': float(np.max(captured_env) - floor_p95),
            })
            print(f'  pilot {index}: offset={records[-1]["offset_s"]:.4f}s '
                  f'snr={records[-1]["snr_db"]:.1f}dB threshold={threshold:.1f}dBFS')

    offsets = [record['offset_s'] for record in records]
    return {
        'device': device,
        'mixer_percent': mixer_percent,
        'pilot_total_sound_ms': 250 * 3,
        'repeats': repeats,
        'offset_s_p50': statistics.median(offsets),
        'offset_s_min': min(offsets),
        'offset_s_max': max(offsets),
        'offset_spread_ms': (max(offsets) - min(offsets)) * 1000,
        'snr_db_p50': statistics.median([record['snr_db'] for record in records]),
        'records': records,
    }


def measure_once(device, wav_path, workdir, label, pre_roll_s=0.3, extra_tail_s=0.8):
    """Play one WAV and capture it; used by ``verify`` only."""
    with wave.open(os.fspath(wav_path)) as handle:
        rate = handle.getframerate()
        channels = handle.getnchannels()
        pcm = np.frombuffer(handle.readframes(handle.getnframes()), dtype='<i2')
    if channels > 1:
        pcm = pcm.reshape(-1, channels).mean(axis=1).astype('<i2')
    audio_s = pcm.size / rate
    seconds = int(audio_s + extra_tail_s + 0.5) + 1
    record_path = workdir / f'{label}_rec.wav'

    floor, _ = Capture(device, 1, workdir / f'{label}_floor.wav').start().wait(timeout=15)
    threshold, floor_p95, silent_threshold = derive_onset_threshold(frame_envelope(floor, CAPTURE_RATE))

    capture = Capture(device, seconds, record_path).start()
    time.sleep(pre_roll_s)
    process, started_ns = play(device, wav_path, blocking=False)
    process.communicate(timeout=seconds + 15)
    playback_done_ns = time.monotonic_ns()
    captured, capture_rate = capture.wait(timeout=seconds + 15)

    played_env = frame_envelope(pcm, rate)
    captured_env = frame_envelope(captured, capture_rate)
    pre_roll_frames = int(pre_roll_s / (FRAME_MS / 1000))
    captured_body = captured_env[pre_roll_frames:]
    acoustic = onset_seconds(captured_body, threshold)

    length = min(len(played_env), len(captured_body))
    played_body = played_env[:length]
    captured_body = captured_body[:length]
    loud = played_body >= (floor_p95 + 6.0)
    silent = captured_body <= silent_threshold
    longest = current = total = 0
    for index in range(length):
        if loud[index] and silent[index]:
            current += 1
            total += 1
            longest = max(longest, current)
        else:
            current = 0
    crosstalk = None
    crosstalk_energy = None
    if loud.any() and silent.size:
        # Two estimates are reported because each has a known artefact: the
        # median difference can come out positive when the played envelope
        # ramps up inside the same 10 ms frame in which the capture is already
        # loud, and the energy ratio is skewed by room noise entering the
        # capture during pauses. The P7 echo work should treat both as upper
        # bounds on how much of our own speaker the microphone hears.
        crosstalk = float(np.median(captured_body[loud]) - np.median(played_body[loud]))
        played_energy = float(np.mean(10 ** (played_body[loud] / 10.0)))
        captured_energy = float(np.mean(10 ** (captured_body[loud] / 10.0)))
        if played_energy > 0:
            crosstalk_energy = float(10.0 * np.log10(captured_energy / played_energy))

    return {
        'label': label,
        'audio_duration_s': audio_s,
        'playback_wall_s': (playback_done_ns - started_ns) / 1e9,
        'leading_silence_in_pcm_s': onset_seconds(played_env, threshold=-60.0, hold=1),
        'acoustic_first_sound_s': acoustic,
        'onset_threshold_dbfs': threshold,
        'floor_p95_dbfs': floor_p95,
        'dropout_max_ms': longest * FRAME_MS,
        'dropout_total_ms': total * FRAME_MS,
        'crosstalk_db_median': crosstalk,
        'crosstalk_db_energy': crosstalk_energy,
        'recording': os.fspath(record_path),
    }


def report(workdir, offset_s, offset_spread_ms, output, operating_max_characters=24,
           objective_s=0.8):
    """Combine the software measurements with the calibrated offset.

    The headline numbers are the operating point, not the whole corpus: the
    first segment of a spoken answer is short by construction, so aggregating
    first-segment latency over a 120-character sentence would measure a
    situation the product never creates.  Both views are emitted.
    """
    rows = []
    for candidate_dir in sorted(Path(workdir).glob('*')):
        summary_path = candidate_dir / 'summary.json'
        if not summary_path.is_file():
            continue
        summary = json.loads(summary_path.read_text())
        warm = summary['warm']
        first = warm['first_valid_pcm_p50']
        first95 = warm['first_valid_pcm_p95']

        # Operating point: every corpus item short enough to be a first segment.
        operating = {characters: stats for characters, stats in summary['by_characters'].items()
                     if int(characters) <= operating_max_characters}
        operating_first = [stats['first_valid_pcm_p95'] for stats in operating.values()
                           if stats['first_valid_pcm_p95'] is not None]
        operating_rtf = [stats['rtf_p95'] for stats in operating.values()
                         if stats['rtf_p95'] is not None]
        operating_runs = sum(stats['runs'] for stats in operating.values())
        worst_first = max(operating_first) if operating_first else None
        worst_rtf = max(operating_rtf) if operating_rtf else None
        rows.append({
            'candidate': summary['candidate'],
            'role': summary.get('role'),
            'threads': summary['threads'],
            'sample_rate': summary['sample_rate'],
            'num_speakers': summary['num_speakers'],
            'load_s': summary['load_s'],
            'cold_load_s_p50': summary['cold'].get('load_s_p50'),
            'peak_rss_mib': warm['peak_rss_kib'] / 1024,
            'incremental_delivery': warm['incremental_delivery_any'],
            'whole_corpus': {
                'software_first_valid_pcm_p50_s': first,
                'software_first_valid_pcm_p95_s': first95,
                'acoustic_first_sound_p50_s': None if first is None else first + offset_s,
                'acoustic_first_sound_p95_s': None if first95 is None else first95 + offset_s,
                'rtf_p50': warm['rtf_p50'],
                'rtf_p95': warm['rtf_p95'],
                'note': 'first-segment figures here include the 120-character item and are not the operating point',
            },
            'operating_point': {
                'max_first_segment_characters': operating_max_characters,
                'runs': operating_runs,
                'software_first_valid_pcm_p95_worst_s': worst_first,
                'acoustic_first_sound_p95_worst_s': None if worst_first is None else worst_first + offset_s,
                'rtf_p95_worst': worst_rtf,
                'first_segment_objective_met': worst_first is not None and worst_first + offset_s <= objective_s,
                'rtf_objective_met': worst_rtf is not None and worst_rtf <= objective_s,
            },
            'health': warm['health'],
            'by_characters': summary['by_characters'],
        })
    payload = {
        'schema_version': 1,
        'objective_s': objective_s,
        'offset_s': offset_s,
        'offset_spread_ms': offset_spread_ms,
        'method': 'acoustic_first_sound = software_first_valid_pcm + calibrated playback path offset',
        'candidates': rows,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / 'acoustic_report.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=('calibrate', 'verify', 'report'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workdir', type=Path, default=None,
                        help='p5_bench output root, for verify/report')
    parser.add_argument('--device', default=DEFAULT_DEVICE)
    parser.add_argument('--mixer-percent', type=int, default=45,
                        help='speaker level used during the measurement, restored afterwards')
    parser.add_argument('--calibration', type=Path, default=None,
                        help='calibration.json from calibrate; required by report')
    parser.add_argument('--candidate', action='append', default=None)
    parser.add_argument('--text', action='append', default=None, dest='texts')
    parser.add_argument('--repeat', type=int, default=1)
    args = parser.parse_args()

    if not Path('/dev/snd').is_dir():
        parser.error('/dev/snd is unavailable; run this script from a normal board terminal')
    args.output.mkdir(parents=True, exist_ok=True)

    if args.mode == 'calibrate':
        calibration = calibrate(args.device, args.output / 'acoustic', args.mixer_percent)
        (args.output / 'calibration.json').write_text(
            json.dumps(calibration, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({key: calibration[key] for key in
                          ('offset_s_p50', 'offset_spread_ms', 'snr_db_p50')},
                         ensure_ascii=False, indent=2))
        return

    if args.mode == 'report':
        if not args.calibration or not args.calibration.is_file():
            parser.error('--calibration calibration.json is required for report')
        calibration = json.loads(args.calibration.read_text())
        payload = report(args.workdir, calibration['offset_s_p50'],
                         calibration['offset_spread_ms'], args.output)
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return

    selected = []
    for candidate_dir in sorted(Path(args.workdir).glob('*')):
        if not candidate_dir.is_dir():
            continue
        if args.candidate and candidate_dir.name not in args.candidate:
            continue
        audio_dir = candidate_dir / 'audio'
        if not audio_dir.is_dir():
            continue
        for wav in sorted(audio_dir.glob('*.wav')):
            if args.texts and wav.stem not in set(args.texts):
                continue
            selected.append((candidate_dir.name, wav))
    if not selected:
        parser.error(f'no candidate audio found under {args.workdir}/*/audio/*.wav')

    records = []
    with Mixer() as mixer:
        mixer.set_percent(args.mixer_percent)
        time.sleep(0.3)
        for candidate, wav in selected:
            for index in range(args.repeat):
                record = measure_once(args.device, wav, args.output / 'acoustic',
                                      f'{candidate}_{wav.stem}_{index}')
                record.update(candidate=candidate, text_id=wav.stem, repeat=index)
                records.append(record)
                print(f'  {candidate:<28} {wav.stem:<9} acoustic_first_sound='
                      f'{record["acoustic_first_sound_s"]} dropout={record["dropout_max_ms"]}ms '
                      f'crosstalk_energy={record["crosstalk_db_energy"]}dB')
    (args.output / 'verify.json').write_text(json.dumps(records, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(records, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
