"""Low-latency Chinese TTS backend adapters selected in phase P5.

This module is the boundary between "a TTS model" and "the streaming pipeline"
and deliberately contains no pipeline: no queues, no threads, no player.  The
phase-6 service is expected to drive these objects.

Design notes that came out of the P5 measurements and are encoded here:

* ``sherpa_onnx.OfflineTts.generate`` is *not* streaming.  Every model tested
  delivered one callback per text segment, after that segment finished
  synthesising.  Low latency therefore comes from cutting the first text
  segment short, which is what :class:`FirstSegmentSplitter` is for.
* A callback timestamp is not a latency measurement.  Leading silence inside
  the generated PCM would be counted as "audio started"; :func:`first_audible_s`
  requires the audio to actually be above the noise floor, and
  :class:`TtsResult` reports both numbers.
* Cold model load must not be charged to first-segment latency, so
  :meth:`TtsBackend.load` is separate and its duration is reported separately.

Importing this module does not import numpy or sherpa_onnx; those are pulled in
by the functions that need them, so the ROS service package can import the
module without the TTS virtual environment present.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import time
from typing import Any, Callable, Sequence

# Audibility gate used consistently by the benchmark, the acoustic report and
# this adapter, so the three cannot drift apart.
VALID_FRAME_MS = 10
VALID_ONSET_DBFS = -45.0
VALID_ONSET_FRAMES = 2

# Punctuation that ends a speakable clause, strongest first.  Used by the
# splitter; kept deliberately small because the streaming pipeline (phase 6) is
# responsible for the richer numeral and unit handling.
STRONG_PUNCTUATION = '。！？；\n'
WEAK_PUNCTUATION = '，、,;：:'


class TtsError(RuntimeError):
    """Raised when a backend cannot produce usable audio."""


@dataclass(frozen=True)
class SegmentPolicy:
    """Text-to-segment policy feeding synthesis.

    ``first_max_characters`` is the latency knob: measured first-valid-PCM
    latency is roughly proportional to it.  Later segments are allowed to grow
    so prosody and throughput do not suffer.
    """
    first_min_characters: int = 4
    first_max_characters: int = 24
    segment_target_characters: int = 28
    segment_max_characters: int = 44

    def __post_init__(self) -> None:
        if not 1 <= self.first_min_characters <= self.first_max_characters:
            raise ValueError('first_min_characters must be >= 1 and <= first_max_characters')
        if not self.first_max_characters <= self.segment_max_characters:
            raise ValueError('first_max_characters must not exceed segment_max_characters')
        if not 1 <= self.segment_target_characters <= self.segment_max_characters:
            raise ValueError('segment_target_characters must be within 1..segment_max_characters')
        # The soft cut needs one character of lookahead to keep a clause mark
        # with its clause, so the hard cap must leave room for it.
        if self.segment_max_characters < self.segment_target_characters + 1:
            raise ValueError('segment_max_characters must be at least segment_target_characters + 1')


class FirstSegmentSplitter:
    """Cut a growing text stream into speakable segments.

    This is boundary logic only.  It decides *where* to cut, aggressively for
    the first segment so playback can start early and conservatively afterwards
    so prosody is not chopped into pieces.
    """

    def __init__(self, policy: SegmentPolicy | None = None) -> None:
        self.policy = policy or SegmentPolicy()
        self._buffer = ''
        self._emitted = 0

    @property
    def buffered_characters(self) -> int:
        return len(self._buffer)

    @property
    def emitted_segments(self) -> int:
        return self._emitted

    def reset(self) -> None:
        self._buffer = ''
        self._emitted = 0

    def feed(self, text: str) -> tuple[str, ...]:
        """Append text and return every segment that is now complete."""
        if not text:
            return ()
        self._buffer += text
        return self._drain(force=False)

    def flush(self) -> tuple[str, ...]:
        """Return the remaining text as a final segment, if any."""
        return self._drain(force=True)

    def finish(self) -> tuple[str, ...]:
        """Close the final buffered clause with natural terminal punctuation.

        Streaming chunks deliberately arrive without synthetic punctuation.
        Only the real end of the answer gets a generated full stop, so the TTS
        front-end receives one grammatical sentence instead of one sentence
        per model token.
        """
        stripped = self._buffer.rstrip()
        trailing = self._buffer[len(stripped):]
        if stripped and stripped[-1] not in '。！？；.!?;':
            self._buffer = stripped + '。' + trailing
        return self._drain(force=True)

    def _limits(self) -> tuple[int, int, int, bool]:
        """Return (floor, target, maximum, strict).

        ``floor`` is the shortest useful segment, ``target`` the preferred
        length, ``maximum`` the hard cap.  ``strict`` marks the first segment:
        it carries the latency budget, so it is released at the first sentence
        end or at the hard cap, but never soft-cut early into a half phrase.
        """
        if self._emitted == 0:
            return (self.policy.first_min_characters, self.policy.first_max_characters,
                    self.policy.first_max_characters, True)
        return (max(self.policy.segment_target_characters // 2, 1),
                self.policy.segment_target_characters,
                self.policy.segment_max_characters, False)

    def _drain(self, force: bool) -> tuple[str, ...]:
        segments: list[str] = []
        while self._buffer:
            floor, target, maximum, strict = self._limits()
            cut = self._choose_cut(floor, target, maximum, strict, force)
            if cut is None:
                break
            segment = self._buffer[:cut].strip()
            self._buffer = self._buffer[cut:]
            if not segment:
                continue
            segments.append(segment)
            self._emitted += 1
        return tuple(segments)

    def _choose_cut(self, floor: int, target: int, maximum: int, strict: bool,
                    force: bool) -> int | None:
        """Index just past the cut, or ``None`` when more text is needed.

        Order of preference: sentence-final punctuation, then a clause break
        once the preferred length has been reached, then a hard cut at the cap.
        Punctuation outside the window is never used, so a segment can never
        exceed ``maximum``.
        """
        length = len(self._buffer)
        if length < floor and not force:
            return None
        window = self._buffer[:maximum]
        for index in range(len(window) - 1, floor - 1, -1):
            if window[index] in STRONG_PUNCTUATION:
                return index + 1
        if length >= maximum:
            for index in range(len(window) - 1, floor - 1, -1):
                if window[index] in WEAK_PUNCTUATION:
                    return index + 1
            return maximum
        if length >= target and not strict:
            for index in range(len(window) - 1, floor - 1, -1):
                if window[index] in WEAK_PUNCTUATION:
                    return index + 1
            # One character of lookahead: a clause mark sitting exactly on the
            # boundary stays with its clause instead of being orphaned into the
            # next segment.  If it has not arrived yet, the segment is held
            # until it does, which costs one character of latency at most and
            # never exceeds the hard maximum because maximum >= target + 1.
            if target < length and self._buffer[target] in WEAK_PUNCTUATION:
                return target + 1
            if target < length:
                return target
        if force:
            return length
        return None


@dataclass
class TtsResult:
    """One synthesised segment, with both software and honest latency fields."""
    text: str
    pcm: Any                      # numpy int16 array, 1-D, mono
    sample_rate: int
    synthesis_s: float
    first_callback_s: float | None
    first_audible_s: float | None
    callback_count: int
    peak_dbfs: float
    rms_dbfs: float

    @property
    def audio_duration_s(self) -> float:
        return float(self.pcm.size) / float(self.sample_rate) if self.sample_rate else 0.0

    @property
    def rtf(self) -> float | None:
        duration = self.audio_duration_s
        return self.synthesis_s / duration if duration else None


def first_audible_s(pcm16: Any, sample_rate: int, threshold_dbfs: float = VALID_ONSET_DBFS,
                    frame_ms: int = VALID_FRAME_MS, hold: int = VALID_ONSET_FRAMES) -> float | None:
    """Seconds until the PCM is actually audible, or ``None`` if it never is."""
    import numpy as np

    values = np.asarray(pcm16)
    if values.size == 0:
        return None
    frame = max(1, int(sample_rate * frame_ms / 1000))
    usable = values[: (values.size // frame) * frame].reshape(-1, frame).astype(np.float64)
    if usable.size == 0:
        return None
    rms = np.sqrt(np.mean(usable ** 2, axis=1))
    with np.errstate(divide='ignore'):
        dbfs = 20.0 * np.log10(np.maximum(rms, 1e-12) / 32768.0)
    loud = dbfs >= threshold_dbfs
    for index in range(max(0, len(loud) - hold + 1)):
        if loud[index:index + hold].all():
            return index * frame / sample_rate
    return None


class TtsBackend:
    """A loaded sherpa-onnx TTS engine for one registry candidate.

    The engine is built from ``config/voice/tts_candidates.json`` so the
    parameters used in the P5 report are the parameters used at run time.
    """

    def __init__(self, name: str, candidate: dict, asset_root: Path | str,
                 threads: int = 4, speed: float = 1.0, speaker_id: int | None = None,
                 allow_cache: bool = True) -> None:
        # `allow_cache` exists because a latency measurement must never be
        # served from a previous run's audio: a cache hit turns a 0.44 s first
        # segment into a 0.000 s one and the resulting number is meaningless.
        # Pre-generated status phrases are an explicit, separate path.
        self.name = name
        self.candidate = candidate
        self.asset_root = Path(asset_root)
        self.threads = threads
        self.speed = speed
        self.speaker_id = speaker_id
        self.allow_cache = bool(allow_cache)
        self.load_s: float | None = None
        self._engine: Any = None
        self._cache: dict[str, TtsResult] = {}

    # -- lifecycle -------------------------------------------------------- #

    @property
    def loaded(self) -> bool:
        return self._engine is not None

    def load(self) -> None:
        """Load the model.  Timing is reported separately from synthesis."""
        if self._engine is not None:
            return
        from .tts_engine import build_engine  # local import: keeps numpy optional

        started = time.monotonic()
        self._engine, self.sample_rate, self.num_speakers = build_engine(
            self.name, self.candidate, self.asset_root, self.threads)
        self.load_s = time.monotonic() - started
        if self.speaker_id is None:
            self.speaker_id = 0
        if not 0 <= self.speaker_id < self.num_speakers:
            raise TtsError(
                f'{self.name}: speaker_id {self.speaker_id} outside 0..{self.num_speakers - 1}')

    def warmup(self, text: str = '好的') -> TtsResult:
        """Synthesise one short acknowledgement so the engine is hot."""
        return self.synthesize(text, use_cache=False)

    def precompute(self, text: str) -> TtsResult:
        """Generate and cache a fixed phrase for later zero-cost playback.

        Only fixed status phrases belong here. Dynamic answers must go through
        :meth:`synthesize` with the cache disabled so their latency is real.
        """
        result = self.synthesize(text, use_cache=False)
        self._cache[text.strip()] = result
        return result

    # -- synthesis -------------------------------------------------------- #

    @property
    def cached_texts(self) -> Sequence[str]:
        return tuple(self._cache)

    def synthesize(self, text: str, use_cache: bool = True) -> TtsResult:
        """Synthesise one segment and validate it before returning."""
        clean = text.strip()
        if not clean:
            raise TtsError('TTS text is empty after stripping whitespace')
        if use_cache and self.allow_cache and clean in self._cache:
            return self._cache[clean]
        self.load()
        result = self._generate(clean)
        if use_cache and self.allow_cache:
            self._cache[clean] = result
        return result

    def _generate(self, text: str) -> TtsResult:
        import numpy as np

        engine = self._engine
        sample_rate = int(engine.sample_rate)
        callbacks: list[dict] = []
        blocks: list[Any] = []
        failure: list[str] = []
        started = time.monotonic()

        def on_samples(samples: Any, progress: float) -> int:
            values = np.asarray(samples, dtype=np.float64)
            if values.ndim != 1 or values.size == 0:
                failure.append('empty PCM callback')
                return 0
            non_finite = int(np.count_nonzero(~np.isfinite(values)))
            if non_finite:
                failure.append(f'{non_finite}/{values.size} NaN/Inf samples')
                return 0
            callbacks.append({'t_s': time.monotonic() - started})
            blocks.append(np.rint(np.clip(values, -1.0, 1.0) * 32767.0).astype('<i2'))
            return 1

        engine.generate(text=text, sid=self.speaker_id, speed=self.speed, callback=on_samples)
        synthesis_s = time.monotonic() - started
        if failure:
            raise TtsError(f'{self.name}: {failure[0]}')
        if not blocks:
            raise TtsError(f'{self.name}: engine produced no PCM')

        pcm = np.concatenate(blocks)
        peak = float(np.max(np.abs(pcm)))
        rms = float(np.sqrt(np.mean(pcm.astype(np.float64) ** 2)))
        peak_dbfs = 20.0 * float(np.log10(max(peak, 1e-9) / 32768.0))
        rms_dbfs = 20.0 * float(np.log10(max(rms, 1e-12) / 32768.0))
        audible = first_audible_s(pcm, sample_rate)
        if audible is None:
            raise TtsError(f'{self.name}: synthesised audio is silent')
        first_callback = callbacks[0]['t_s']
        result = TtsResult(
            text=text, pcm=pcm, sample_rate=sample_rate, synthesis_s=synthesis_s,
            first_callback_s=first_callback,
            first_audible_s=first_callback + audible,
            callback_count=len(callbacks), peak_dbfs=peak_dbfs, rms_dbfs=rms_dbfs,
        )
        return result

    def close(self) -> None:
        self._engine = None
        self._cache.clear()


def load_registry(config_path: Path | str) -> dict:
    return json.loads(Path(config_path).read_text())


def resolve(config_path: Path | str, asset_root: Path | str, name: str) -> TtsBackend:
    """Build an unloaded backend for ``name`` from the candidate registry."""
    registry = load_registry(config_path)
    candidates = registry['candidates']
    if name not in candidates:
        raise TtsError(f'candidate {name!r} is not in {config_path}')
    candidate = candidates[name]
    speaker_id = registry.get('speaker_ids', {}).get(name)
    return TtsBackend(name, candidate, Path(asset_root) / name,
                      speaker_id=speaker_id if isinstance(speaker_id, int) else None,
                      allow_cache=False)
