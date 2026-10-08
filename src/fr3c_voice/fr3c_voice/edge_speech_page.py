"""Standalone ASR/TTS tab using the existing workers and local speech service.

There is deliberately no model, intent or robot dispatch dependency here.
Only this tab's queue consumes its ASR results. TTS uses non-replacing sessions
and checks ownership before using the service's global stop operation.
"""
from __future__ import annotations

from pathlib import Path
import queue
import logging
import threading
import time
import uuid

from .speech_client import SpeechClient, SpeechServiceError
from .voice_input import VoiceInput


LOG = logging.getLogger(__name__)


def _active_speech_request(status):
    running = status.get('session_running')
    if running is False:
        return ''
    request = (status.get('last_metrics') or {}).get('request_id')
    if running is not True or not request:
        raise SpeechServiceError('语音服务返回的会话状态不完整')
    return request


class EdgeSpeechPage:
    POLL_MS = 100
    HEALTH_INTERVAL_S = 3.0

    def __init__(self, notebook, workspace, resources, speech_socket,
                 speech_lock=None, voice_factory=VoiceInput, client=None):
        import tkinter as tk
        from tkinter import ttk

        self.tk, self.ttk = tk, ttk
        self.notebook = notebook
        self.resources = resources
        self.events = queue.Queue()
        self.closing = False
        self.generation = 0
        self.asr_request = ''
        self.tts_request = ''
        self._asr_canceling = False
        self._asr_finalizing = False
        self._asr_cleanup = None
        self._tts_thread = None
        self._tts_stop_thread = None
        self._tts_cancel = threading.Event()
        self._speech_lock = speech_lock or threading.Lock()
        self.client = client or SpeechClient(speech_socket, timeout_s=5.0)
        self._health_running = False
        self._next_health = 0.0
        self._tts_ready = False
        self._tts_label = 'TTS：检查中'
        self.final_text = ''
        self.voice = None
        self._asr_label = 'ASR：未配置'
        try:
            self.voice = voice_factory(Path(workspace), self.events.put, self._before_capture)
            # Asset validation only: do not load a model or open an audio device.
            self.voice.validate()
            self._asr_label = 'ASR：本地 Sherpa-onnx / RKNN'
        except Exception as error:
            self.voice = None
            self._asr_label = 'ASR：模型或配置未就绪，请检查现有语音配置'
            LOG.warning('edge speech ASR validation failed: %s', error)

        self.page = ttk.Frame(notebook, style='App.TFrame', padding=(12, 10))
        notebook.add(self.page, text='  端侧语音感知  ')
        self._build()
        # Immediate tab events close the gap before the next periodic GUI poll.
        self._tab_binding = notebook.bind('<<NotebookTabChanged>>', self._tab_changed, add='+')
        self._timer = self.page.after(self.POLL_MS, self._tick)
        self._refresh_controls()

    def _build(self):
        tk, ttk = self.tk, self.ttk
        header = ttk.Frame(self.page, style='Card.TFrame', padding=12)
        header.pack(fill='x')
        ttk.Label(header, text='端侧语音感知', style='Section.TLabel').pack(anchor='w')
        self.device_status = tk.StringVar(value=self._asr_label + ' · ' + self._tts_label)
        ttk.Label(header, textvariable=self.device_status, style='Hint.TLabel',
                  wraplength=700).pack(anchor='w', pady=(5, 0))
        self.resource_status = tk.StringVar()
        ttk.Label(self.page, textvariable=self.resource_status, style='Sub.TLabel',
                  wraplength=700).pack(anchor='w', pady=(8, 5))

        asr = ttk.Frame(self.page, style='Card.TFrame', padding=12)
        asr.pack(fill='both', expand=True, pady=(0, 10))
        ttk.Label(asr, text='语音识别 ASR', style='Section.TLabel').pack(anchor='w')
        ttk.Label(asr, text='单句录音 · 停顿后自动定稿，也可点击结束说话 · 结果仅在本页展示',
                  style='Hint.TLabel', wraplength=700).pack(anchor='w', pady=(4, 6))
        self.asr_text = tk.Text(asr, height=3, wrap='word', relief='flat',
                                font=('Noto Sans CJK SC', 11), background='#f5f7fa',
                                foreground='#1f2a3a', state='disabled')
        self.asr_text.pack(fill='both', expand=True)
        actions = ttk.Frame(asr, style='Card.TFrame')
        actions.pack(fill='x', pady=(8, 0))
        self.record_button = ttk.Button(actions, text='开始录音', style='Primary.TButton',
                                        command=self.start_recording)
        self.record_button.pack(side='left', padx=(0, 6))
        self.finish_button = ttk.Button(actions, text='结束说话', command=self.finish_recording)
        self.finish_button.pack(side='left', padx=4)
        self.cancel_button = ttk.Button(actions, text='取消录音', command=self.cancel_recording)
        self.cancel_button.pack(side='left', padx=4)
        self.copy_button = ttk.Button(actions, text='复制文本', command=self.copy_text)
        self.copy_button.pack(side='right', padx=4)
        self.fill_button = ttk.Button(actions, text='填入 TTS', command=self.fill_tts)
        self.fill_button.pack(side='right', padx=4)
        self.asr_status = tk.StringVar(value='准备录音')
        ttk.Label(asr, textvariable=self.asr_status, style='Hint.TLabel',
                  wraplength=700).pack(anchor='w', pady=(6, 0))

        tts = ttk.Frame(self.page, style='Card.TFrame', padding=12)
        tts.pack(fill='both', expand=True)
        ttk.Label(tts, text='文本转语音 TTS', style='Section.TLabel').pack(anchor='w')
        ttk.Label(tts, text='输入文本或填入识别结果，再点击开始朗读',
                  style='Hint.TLabel').pack(anchor='w', pady=(4, 6))
        self.tts_text = tk.Text(tts, height=3, wrap='word', relief='flat',
                                font=('Noto Sans CJK SC', 11), background='#f5f7fa',
                                foreground='#1f2a3a')
        self.tts_text.pack(fill='both', expand=True)
        actions = ttk.Frame(tts, style='Card.TFrame')
        actions.pack(fill='x', pady=(8, 0))
        self.speak_button = ttk.Button(actions, text='开始朗读', style='Primary.TButton',
                                       command=self.start_speaking)
        self.speak_button.pack(side='left', padx=(0, 6))
        self.stop_button = ttk.Button(actions, text='停止朗读', command=self.stop_speaking)
        self.stop_button.pack(side='left', padx=4)
        self.tts_status = tk.StringVar(value='准备朗读')
        ttk.Label(tts, textvariable=self.tts_status, style='Hint.TLabel',
                  wraplength=700).pack(anchor='w', pady=(6, 0))

    @property
    def audio_active(self):
        return bool(self.asr_request or self.tts_request or self._voice_active()
                    or (self._asr_cleanup and self._asr_cleanup.is_alive())
                    or (self._tts_thread and self._tts_thread.is_alive())
                    or (self._tts_stop_thread and self._tts_stop_thread.is_alive()))

    def _voice_active(self):
        # VoiceInput.close() has a bounded join and no public active property.
        # Read liveness here without altering its original implementation.
        thread = getattr(self.voice, '_thread', None)
        return bool(thread and thread.is_alive())

    def _visible(self):
        return self.notebook.select() == str(self.page)

    def _tab_changed(self, _event=None):
        if self.closing:
            return
        visible = self._visible()
        self.resources.update(visible, self.audio_active)
        if not visible and self.audio_active:
            self.cancel_recording()
            self.stop_speaking()
        self._refresh_controls()

    def _refresh_controls(self):
        idle = not self.audio_active
        ready = self.resources.visible and self.resources.ready
        for button, enabled in (
            (self.record_button, ready and idle and self.voice is not None),
            (self.finish_button, bool(self.asr_request) and not self._asr_canceling
             and not self._asr_finalizing),
            (self.cancel_button, bool(self.asr_request) and not self._asr_canceling),
            (self.speak_button, ready and idle and self._tts_ready),
            (self.stop_button, bool(self.tts_request) and not self._tts_cancel.is_set()),
            (self.copy_button, bool(self.final_text)),
            (self.fill_button, bool(self.final_text)),
        ):
            button.configure(state='normal' if enabled else 'disabled')
        self.resource_status.set(self.resources.reason)
        self.device_status.set(self._asr_label + ' · ' + self._tts_label)

    def _set_asr_text(self, text):
        self.asr_text.configure(state='normal')
        self.asr_text.delete('1.0', 'end')
        self.asr_text.insert('1.0', text)
        self.asr_text.configure(state='disabled')

    def start_recording(self):
        self.resources.update(self._visible(), self.audio_active)
        if not self.resources.ready or self.audio_active or self.voice is None:
            return
        self.generation += 1
        self.final_text = ''
        self._set_asr_text('')
        self._asr_canceling = False
        self._asr_finalizing = False
        self.asr_request = self.voice.start()
        self.asr_status.set('正在准备麦克风…' if self.asr_request else '录音仍在释放，请稍候')
        self._refresh_controls()

    def _before_capture(self):
        if self.closing or not self.resources.ready or not self.resources.visible:
            raise RuntimeError('本页已离开或音频资源尚未释放')
        # Do not use the original before_capture callback: it globally stops TTS.
        # Refuse capture through someone else's playback instead.
        if self.client.socket_path.exists():
            if _active_speech_request(self.client.status()):
                raise RuntimeError('语音服务正在播报，请等待结束后再录音')

    def finish_recording(self):
        if self.asr_request and not self._asr_canceling:
            self._asr_finalizing = True
            self.voice.finish()
            self.asr_status.set('正在定稿…')
            self.finish_button.configure(state='disabled')

    def cancel_recording(self):
        if not self.asr_request or self._asr_canceling:
            return
        self.generation += 1
        self.asr_request = ''  # ignore partial/final events already in the queue
        self._asr_canceling = True
        self.asr_status.set('正在取消录音并释放麦克风…')
        self._asr_cleanup = threading.Thread(target=self.voice.close, daemon=True)
        self._asr_cleanup.start()
        self._refresh_controls()

    def copy_text(self):
        if self.final_text:
            self.page.clipboard_clear()
            self.page.clipboard_append(self.final_text)
            self.asr_status.set('已复制识别文本')

    def fill_tts(self):
        if self.final_text:
            self.tts_text.delete('1.0', 'end')
            self.tts_text.insert('1.0', self.final_text)
            self.tts_status.set('已填入识别结果，点击开始朗读')

    def start_speaking(self):
        self.resources.update(self._visible(), self.audio_active)
        if not self.resources.ready or self.audio_active or not self._tts_ready:
            return
        text = self.tts_text.get('1.0', 'end-1c').strip()
        if not text:
            self.tts_status.set('请先输入需要朗读的文本')
            return
        if len(text) > 2000:
            self.tts_status.set('请将单次朗读文本控制在 2000 字以内')
            return
        self.generation += 1
        epoch = self.generation
        self.tts_request = rid = 'edge-speech-' + uuid.uuid4().hex
        self._tts_cancel.clear()
        self.tts_status.set('正在准备朗读…')
        self._tts_thread = threading.Thread(
            target=self._speak_worker, args=(rid, epoch, text), daemon=True)
        self._tts_thread.start()
        self._refresh_controls()

    def _speak_worker(self, rid, epoch, text):
        attempted = False
        error = ''
        try:
            with self._speech_lock:
                if self._tts_cancel.is_set() or self.closing:
                    return
                attempted = True
                self.client.begin(replace=False, request_id=rid)
            self.events.put(('tts_started', rid, epoch))
            for index in range(0, len(text), 24):
                if self._tts_cancel.is_set():
                    break
                self.client.feed(rid, text[index:index + 24])
            # Even cancellation closes text through the existing end/wait
            # interface, which also joins the service's synthesis workers.
            self.client.finish(rid, timeout_s=600.0)
        except Exception as failure:
            error = str(failure)
        finally:
            if attempted:
                self._release_speech(rid, epoch)
            self.events.put(('tts_done', rid, epoch, error))

    def _release_speech(self, rid, epoch):
        """Keep the UI lease until the server confirms our session has ended.

        Also handles a lost speak_start acknowledgement. Failure to observe the
        server never authorizes a blind global stop or resuming the microphone.
        """
        reported = False
        drain = None

        def finish_input():
            try:
                self.client.finish(rid, timeout_s=600.0)
            except Exception:
                # Reconcile through status and retry if the server still has
                # this session with unclosed input (e.g. a lost start ACK).
                LOG.debug('speech input cleanup failed', exc_info=True)

        while True:
            try:
                with self._speech_lock:
                    status = self.client.status()
                    active_id = _active_speech_request(status)
                    if active_id != rid:
                        if drain is None or not drain.is_alive():
                            return
                    elif (status.get('last_metrics') or {}).get('text_end_s') is None:
                        # End text before stopping: a speak_end handler keeps
                        # its pipeline reference and waits for native synthesis
                        # even after the service marks playback as stopped.
                        if drain is None or not drain.is_alive():
                            drain = threading.Thread(target=finish_input, daemon=True)
                            drain.start()
                    else:
                        self.client.stop('edge_speech_release')
                # Observe playback end and wait for any cleanup connection.
            except Exception:
                if not reported:
                    self.events.put(('tts_releasing', rid, epoch))
                    reported = True
                # On app shutdown the host stops the service under the same
                # start lock; these daemon retries need not outlive the window.
                if self.closing:
                    return
            time.sleep(0.25)

    def stop_speaking(self):
        if not self.tts_request or self._tts_cancel.is_set():
            return
        self._tts_cancel.set()
        self.tts_status.set('正在停止朗读并释放音频资源…')
        # finish() waits for playback on another connection. Interrupt it from
        # a separate worker, serialized against a delayed speak_start ACK.
        rid, epoch = self.tts_request, self.generation
        self._tts_stop_thread = threading.Thread(
            target=self._stop_worker, args=(rid, epoch), daemon=True)
        self._tts_stop_thread.start()
        self._refresh_controls()

    def _stop_worker(self, rid, epoch):
        self._release_speech(rid, epoch)

    def _health_worker(self):
        try:
            health = self.client.health()
            ready = bool(health.get('healthy'))
            label = ('TTS：本地 ' + str(health.get('candidate', '已就绪'))
                     if ready else 'TTS：服务未就绪')
        except SpeechServiceError:
            ready, label = False, 'TTS：服务未运行，可先使用 ASR'
        except Exception:
            ready, label = False, 'TTS：暂时无法确认服务状态'
        self.events.put(('health', ready, label))

    def _drain_events(self):
        for _ in range(100):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            kind = event[0]
            if kind == 'health':
                self._health_running = False
                self._tts_ready, self._tts_label = event[1:]
            elif kind.startswith('asr_'):
                _, rid, data = event
                if rid != self.asr_request or not self.resources.visible:
                    continue
                if kind == 'asr_recording':
                    self.asr_status.set('正在识别本句话…')
                elif kind == 'asr_partial':
                    self._set_asr_text(data.get('text', ''))
                elif kind == 'asr_finalizing':
                    self._asr_finalizing = True
                    self.asr_status.set('正在定稿…')
                    self.finish_button.configure(state='disabled')
                elif kind in ('asr_final', 'asr_empty', 'asr_error'):
                    self.asr_request = ''
                    if kind == 'asr_final':
                        self.final_text = data.get('raw_text', data.get('text', ''))
                        self._set_asr_text(self.final_text)
                        self.asr_status.set(f"识别完成 · 录音 {data.get('audio_s', 0):.1f} 秒")
                    else:
                        self._set_asr_text('')
                        self.asr_status.set(data.get('message') or '没有识别到清晰语音，请再说一次')
            elif kind.startswith('tts_'):
                _, rid, epoch, *details = event
                if rid != self.tts_request or epoch != self.generation:
                    continue
                if kind == 'tts_started' and not self._tts_cancel.is_set():
                    self.tts_status.set('正在朗读…')
                elif kind == 'tts_releasing':
                    self.tts_status.set('暂时无法确认播报状态，正在重试释放…')
                elif kind == 'tts_done':
                    self.tts_request = ''
                    message = ('朗读已停止' if self._tts_cancel.is_set() else
                               ('朗读失败：' + details[0] if details[0] else '朗读完成'))
                    self.tts_status.set(message)

    def _tick(self):
        if self.closing:
            return
        try:
            self._tab_changed()
            self._drain_events()
            if self._asr_canceling and not self._voice_active() and not (
                    self._asr_cleanup and self._asr_cleanup.is_alive()):
                self._asr_canceling = False
                self.asr_status.set('录音已取消')
            # Original robot work may also be started outside this UI.
            if self.resources.reserved and self.resources.visible and not self.resources.ready:
                self.cancel_recording()
                self.stop_speaking()
            self.resources.update(self._visible(), self.audio_active)
            now = time.monotonic()
            if self.resources.visible and not self._health_running and now >= self._next_health:
                self._health_running = True
                self._next_health = now + self.HEALTH_INTERVAL_S
                threading.Thread(target=self._health_worker, daemon=True).start()
            self._refresh_controls()
        except Exception:
            LOG.exception('edge speech page poll failed; retrying next tick')
        finally:
            if not self.closing:
                self._timer = self.page.after(self.POLL_MS, self._tick)

    def close(self):
        if self.closing:
            return
        self.cancel_recording()
        self.stop_speaking()
        self.closing = True
        self.resources.ready = False
        self.page.after_cancel(self._timer)
        self.notebook.unbind('<<NotebookTabChanged>>', self._tab_binding)
        # In particular, --no-speech disables dialogue playback, but the tab
        # can still use a running service. Finish its own shutdown before Tk
        # exits and daemon workers can disappear.
        deadline = time.monotonic() + 6.0
        for worker in (self._tts_stop_thread, self._tts_thread, self._asr_cleanup):
            if worker and worker is not threading.current_thread():
                worker.join(timeout=max(0.0, deadline - time.monotonic()))
