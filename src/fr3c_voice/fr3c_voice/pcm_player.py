"""Persistent ALSA PCM playback with a precise, explicit buffer.

Why this is not ``aplay`` in a pipe: the phase-5 acoustic calibration showed the
playback path adds essentially zero latency on this board, and that result
depends on knowing exactly how much audio is buffered ahead of the speaker.
``snd_pcm_set_params`` lets ALSA pick the buffer, which is fine for one-shot
playback and opaque for a latency budget, so this module sets the hardware
parameters explicitly and reports what it got.

Design points:

* one handle stays open across replies; ``drain`` marks a session boundary and
  ``drop`` makes stop immediate, so a reply never has to close the device
* cancellation is checked between writes, so a stop request is honoured within
  one period rather than after the queued audio has played out
* a USB device that disappears and comes back is reopened on the next
  ``prepare`` instead of disabling every later reply
* ``xrun`` count and written/ahead frame counts are exposed, because "queued"
  is not "heard" and the pipeline must not confuse the two

No TTS logic lives here; this module only moves PCM to the device.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import errno
import threading
import time
from dataclasses import dataclass, field
from typing import Any

# ALSA enumeration values, from <alsa/pcm.h>. Hard-coded so this module needs
# no build-time dependency on ALSA headers.
SND_PCM_STREAM_PLAYBACK = 0
SND_PCM_FORMAT_S16_LE = 2
SND_PCM_ACCESS_RW_INTERLEAVED = 3
SND_PCM_HW_PARAM_PERIOD_SIZE = 13
SND_PCM_HW_PARAM_BUFFER_SIZE = 16


class PcmPlayerError(RuntimeError):
    """Raised when the device cannot be opened, written or flushed."""


class PcmCancelled(PcmPlayerError):
    """Raised when a write/flush observes the cancellation event."""


@dataclass
class PlayerStats:
    sessions: int = 0
    midstream_recoveries: int = 0
    frames_written: int = 0
    written_seconds: float = 0.0
    max_write_block_s: float = 0.0
    xruns: int = 0
    device_reopens: int = 0
    drops: int = 0
    last_error: str = ''
    prepared_sample_rate: int = 0
    period_frames: int = 0
    buffer_frames: int = 0
    buffer_seconds: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            'sessions': self.sessions,
            'midstream_recoveries': self.midstream_recoveries,
            'frames_written': self.frames_written,
            'written_seconds': round(self.written_seconds, 4),
            'max_write_block_s': round(self.max_write_block_s, 4),
            'xruns': self.xruns,
            'device_reopens': self.device_reopens,
            'drops': self.drops,
            'last_error': self.last_error,
            'prepared_sample_rate': self.prepared_sample_rate,
            'period_frames': self.period_frames,
            'buffer_frames': self.buffer_frames,
            'buffer_seconds': round(self.buffer_seconds, 4),
            **self.extra,
        }


class AlsaPcmPlayer:
    """A single ALSA playback handle reused by every speech session."""

    def __init__(self, device: str, sample_rate: int = 22050,
                 target_latency_ms: int = 80, channels: int = 1) -> None:
        self.device = str(device)
        self.sample_rate = int(sample_rate)
        self.target_latency_ms = int(target_latency_ms)
        self.channels = int(channels)
        self.stats = PlayerStats()
        self._lock = threading.RLock()
        self._handle = ctypes.c_void_p()
        self._library: Any | None = None
        self._load_error = ''
        self._write_lock = threading.Lock()
        try:
            name = ctypes.util.find_library('asound') or 'libasound.so.2'
            library = ctypes.CDLL(name)
            self._configure_api(library)
            self._library = library
        except (AttributeError, OSError) as error:  # pragma: no cover - host dependent
            self._load_error = f'libasound unavailable: {error}'

    # -- ctypes surface --------------------------------------------------- #

    @staticmethod
    def _configure_api(library: Any) -> None:
        library.snd_pcm_open.argtypes = [ctypes.POINTER(ctypes.c_void_p),
                                         ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
        library.snd_pcm_open.restype = ctypes.c_int
        library.snd_pcm_close.argtypes = [ctypes.c_void_p]
        library.snd_pcm_close.restype = ctypes.c_int
        library.snd_pcm_hw_params_malloc.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        library.snd_pcm_hw_params_malloc.restype = ctypes.c_int
        library.snd_pcm_hw_params_free.argtypes = [ctypes.c_void_p]
        library.snd_pcm_hw_params_free.restype = None
        library.snd_pcm_hw_params_any.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        library.snd_pcm_hw_params_any.restype = ctypes.c_int
        library.snd_pcm_hw_params_set_access.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
        library.snd_pcm_hw_params_set_access.restype = ctypes.c_int
        library.snd_pcm_hw_params_set_format.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
        library.snd_pcm_hw_params_set_format.restype = ctypes.c_int
        library.snd_pcm_hw_params_set_channels.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint]
        library.snd_pcm_hw_params_set_channels.restype = ctypes.c_int
        library.snd_pcm_hw_params_set_rate.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint, ctypes.c_int]
        library.snd_pcm_hw_params_set_rate.restype = ctypes.c_int
        library.snd_pcm_hw_params_set_period_size_near.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int)]
        library.snd_pcm_hw_params_set_period_size_near.restype = ctypes.c_int
        library.snd_pcm_hw_params_set_buffer_size_near.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong)]
        library.snd_pcm_hw_params_set_buffer_size_near.restype = ctypes.c_int
        library.snd_pcm_hw_params.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        library.snd_pcm_hw_params.restype = ctypes.c_int
        library.snd_pcm_hw_params_get_period_size.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int)]
        library.snd_pcm_hw_params_get_period_size.restype = ctypes.c_int
        library.snd_pcm_hw_params_get_buffer_size.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong)]
        library.snd_pcm_hw_params_get_buffer_size.restype = ctypes.c_int
        library.snd_pcm_prepare.argtypes = [ctypes.c_void_p]
        library.snd_pcm_prepare.restype = ctypes.c_int
        library.snd_pcm_writei.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
        library.snd_pcm_writei.restype = ctypes.c_long
        library.snd_pcm_avail_update.argtypes = [ctypes.c_void_p]
        library.snd_pcm_avail_update.restype = ctypes.c_long
        library.snd_pcm_delay.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_long)]
        library.snd_pcm_delay.restype = ctypes.c_int
        library.snd_pcm_drain.argtypes = [ctypes.c_void_p]
        library.snd_pcm_drain.restype = ctypes.c_int
        library.snd_pcm_drop.argtypes = [ctypes.c_void_p]
        library.snd_pcm_drop.restype = ctypes.c_int
        library.snd_pcm_recover.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
        library.snd_pcm_recover.restype = ctypes.c_int
        library.snd_strerror.argtypes = [ctypes.c_int]
        library.snd_strerror.restype = ctypes.c_char_p

    def _message(self, code: int) -> str:
        if self._library is None:
            return f'ALSA error {code}'
        value = self._library.snd_strerror(int(code))
        return value.decode('utf-8', errors='replace') if value else f'ALSA error {code}'

    def _fail(self, operation: str, code: int) -> PcmPlayerError:
        message = f'ALSA {operation} failed ({code}): {self._message(code)}'
        self.stats.last_error = message
        return PcmPlayerError(message)

    # -- lifecycle -------------------------------------------------------- #

    def validate(self) -> list[str]:
        return [self._load_error] if self._load_error else []

    @property
    def prepared(self) -> bool:
        with self._lock:
            return bool(self._handle.value)

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self._handle.value and self._library is not None:
            self._library.snd_pcm_drop(self._handle)
            self._library.snd_pcm_close(self._handle)
        self._handle = ctypes.c_void_p()
        self.stats.prepared_sample_rate = 0

    def _open_locked(self) -> None:
        """Open and fully configure the device; caller holds ``self._lock``."""
        library = self._library
        if library is None:
            raise PcmPlayerError(self._load_error or 'libasound unavailable')
        handle = ctypes.c_void_p()
        code = int(library.snd_pcm_open(ctypes.byref(handle), self.device.encode('utf-8'),
                                        SND_PCM_STREAM_PLAYBACK, 0))
        if code < 0:
            raise self._fail('open', code)
        params = ctypes.c_void_p()
        try:
            code = int(library.snd_pcm_hw_params_malloc(ctypes.byref(params)))
            if code < 0:
                raise self._fail('hw_params_malloc', code)
            for stage, value in (
                ('hw_params_any', library.snd_pcm_hw_params_any(handle, params)),
                ('set_access', library.snd_pcm_hw_params_set_access(
                    handle, params, SND_PCM_ACCESS_RW_INTERLEAVED)),
                ('set_format', library.snd_pcm_hw_params_set_format(
                    handle, params, SND_PCM_FORMAT_S16_LE)),
                ('set_channels', library.snd_pcm_hw_params_set_channels(
                    handle, params, self.channels)),
            ):
                if int(value) < 0:
                    raise self._fail(stage, int(value))
            rate = ctypes.c_uint(self.sample_rate)
            direction = ctypes.c_int(0)
            code = int(library.snd_pcm_hw_params_set_rate(handle, params, rate, direction))
            if code < 0:
                raise self._fail('set_rate', code)

            # One period is the cancellation granularity, so keep it short;
            # the buffer is the jitter reservoir and is reported, not assumed.
            period = ctypes.c_ulong(max(64, int(self.sample_rate * 0.02)))
            period_dir = ctypes.c_int(0)
            code = int(library.snd_pcm_hw_params_set_period_size_near(
                handle, params, ctypes.byref(period), ctypes.byref(period_dir)))
            if code < 0:
                raise self._fail('set_period_size', code)
            wanted_buffer = ctypes.c_ulong(
                max(int(period.value * 4), int(self.sample_rate * self.target_latency_ms / 1000)))
            code = int(library.snd_pcm_hw_params_set_buffer_size_near(
                handle, params, ctypes.byref(wanted_buffer)))
            if code < 0:
                raise self._fail('set_buffer_size', code)
            code = int(library.snd_pcm_hw_params(handle, params))
            if code < 0:
                raise self._fail('hw_params', code)

            actual_period = ctypes.c_ulong()
            actual_buffer = ctypes.c_ulong()
            library.snd_pcm_hw_params_get_period_size(
                params, ctypes.byref(actual_period), ctypes.byref(period_dir))
            library.snd_pcm_hw_params_get_buffer_size(params, ctypes.byref(actual_buffer))
            self.stats.period_frames = int(actual_period.value)
            self.stats.buffer_frames = int(actual_buffer.value)
            self.stats.buffer_seconds = actual_buffer.value / float(self.sample_rate)
        except PcmPlayerError:
            library.snd_pcm_close(handle)
            raise
        finally:
            if params.value:
                library.snd_pcm_hw_params_free(params)
        self._handle = handle
        self.stats.prepared_sample_rate = self.sample_rate

    def begin_session(self, sample_rate: int | None = None) -> bool:
        """Drop any leftover audio and prepare a clean session boundary.

        Without this, audio still queued from the previous reply is heard (and
        timestamped) as the first sound of the next one, which silently corrupts
        every latency measurement and makes two sentences overlap.
        """
        self.drop()
        return self.prepare(sample_rate)

    def prepare(self, sample_rate: int | None = None) -> bool:
        """(Re)prepare the device. Returns True when an open handle was reused."""
        if sample_rate is not None and int(sample_rate) != self.sample_rate:
            with self._lock:
                self._close_locked()
            self.sample_rate = int(sample_rate)
        library = self._library
        if library is None:
            raise PcmPlayerError(self._load_error or 'libasound unavailable')
        with self._lock:
            if not self._handle.value:
                self._open_locked()
                return False
            code = int(library.snd_pcm_prepare(self._handle))
            if code >= 0:
                return True
            # A USB device that was re-enumerated leaves the cached handle
            # pointing at a node that no longer exists. Reopen once instead of
            # failing every later reply.
            self._close_locked()
            self._open_locked()
            self.stats.device_reopens += 1
            return False

    # -- streaming -------------------------------------------------------- #

    def write(self, pcm: bytes, cancel: threading.Event | None = None,
              first_chunk_callback: Any | None = None) -> float:
        """Write one PCM block, returning the time spent blocked in ALSA.

        The block is written one period at a time. ``snd_pcm_writei`` blocks
        once the device buffer is full, so writing a whole multi-second
        utterance in one call would keep this thread busy for as long as the
        audio lasts and would make "first write" a meaningless timestamp.
        ``first_chunk_callback`` fires after the first period is accepted,
        which is the moment the device actually starts producing sound.
        """
        if not pcm or len(pcm) % 2:
            raise PcmPlayerError('PCM block must be non-empty and 16-bit aligned')
        library = self._library
        with self._lock:
            handle = ctypes.c_void_p(self._handle.value)
        if not handle.value or library is None:
            raise PcmPlayerError('ALSA PCM player is not prepared')

        frames = len(pcm) // 2
        buffer = (ctypes.c_char * len(pcm)).from_buffer_copy(pcm)
        offset = 0
        started = time.monotonic()
        period = max(1, self.stats.period_frames or int(self.sample_rate * 0.02))
        notified = False
        recovery_attempts = 0
        max_recoveries = 3
        with self._write_lock:
            while offset < frames:
                if cancel is not None and cancel.is_set():
                    raise PcmCancelled('playback cancelled')
                chunk = min(period, frames - offset)
                pointer = ctypes.cast(ctypes.byref(buffer, offset * 2), ctypes.c_void_p)
                result = int(library.snd_pcm_writei(handle, pointer, chunk))
                if result > 0:
                    offset += result
                    if not notified and first_chunk_callback is not None:
                        notified = True
                        first_chunk_callback()
                    continue
                if result == -errno.EAGAIN:
                    time.sleep(0.002)
                    continue
                if result == -errno.EPIPE:
                    self.stats.xruns += 1
                recovered = int(library.snd_pcm_recover(handle, result, 1))
                if recovered >= 0:
                    continue
                # A USB audio device can be re-enumerated mid-reply (observed on
                # this board): the cached handle then points at a node that no
                # longer exists and every write fails with ENODEV. Reopen once
                # and keep going instead of ending the answer early.
                if recovered in (-errno.ENODEV, -errno.EBADFD, -errno.ENOENT):
                    if recovery_attempts < max_recoveries:
                        recovery_attempts += 1
                        with self._lock:
                            self._close_locked()
                            try:
                                self._open_locked()
                            except PcmPlayerError as error:
                                raise self._fail('reopen', -errno.ENODEV) from error
                            self.stats.device_reopens += 1
                            self.stats.midstream_recoveries += 1
                            handle = ctypes.c_void_p(self._handle.value)
                        if cancel is not None and cancel.is_set():
                            raise PcmCancelled('playback cancelled')
                        continue
                if cancel is not None and cancel.is_set():
                    raise PcmCancelled('playback cancelled')
                raise self._fail('write/recover', recovered)
        elapsed = time.monotonic() - started
        self.stats.frames_written += frames
        self.stats.written_seconds += frames / float(self.sample_rate)
        self.stats.max_write_block_s = max(self.stats.max_write_block_s, elapsed)
        return elapsed

    def queued_frames(self) -> int | None:
        """Frames still in the device buffer, or ``None`` if unavailable."""
        library = self._library
        with self._lock:
            handle = ctypes.c_void_p(self._handle.value)
        if not handle.value or library is None:
            return None
        delay = ctypes.c_long()
        if int(library.snd_pcm_delay(handle, ctypes.byref(delay))) < 0:
            return None
        return int(delay.value)

    def drain(self) -> float:
        """Let everything already written reach the speaker."""
        library = self._library
        with self._lock:
            handle = ctypes.c_void_p(self._handle.value)
        if not handle.value or library is None:
            raise PcmPlayerError('ALSA PCM player is not prepared')
        started = time.monotonic()
        code = int(library.snd_pcm_drain(handle))
        if code < 0:
            raise self._fail('drain', code)
        return time.monotonic() - started

    def drop(self) -> float:
        """Discard queued audio immediately; used for stop/cancel."""
        library = self._library
        with self._lock:
            handle = ctypes.c_void_p(self._handle.value)
        if not handle.value or library is None:
            return 0.0
        started = time.monotonic()
        code = int(library.snd_pcm_drop(handle))
        if code < 0:
            raise self._fail('drop', code)
        self.stats.drops += 1
        return time.monotonic() - started
