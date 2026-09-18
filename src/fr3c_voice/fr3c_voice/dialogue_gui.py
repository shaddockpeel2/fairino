"""Single-page stage-4 dialogue GUI for safe text-to-Action operation."""

from __future__ import annotations

import argparse
from datetime import datetime
import logging
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid

from .voice_input import VoiceInput
from .wake_input import WakeInput, wake_query_from_asr
from .trace import TraceWriter
from .config import load_config
from .conversation import (
    DialogueSettings,
    ShortConversation,
    build_vision_prompt,
    load_dialogue_settings,
)
from .image_capture import SavedSnapshot, save_snapshot
from .intent import (
    COLOR_DISPLAY,
    DialogueIntent,
    IntentProtocolError,
    classify_text,
    direct_shortcut,
    safe_local_action_route,
    safe_local_non_action_route,
)
from .model_client import CancellableGeneration, ModelServiceClient, ModelServiceError
from .speech_client import SpeechClient, SpeechServiceError, StreamingSpeaker


LOG = logging.getLogger('fr3c_dialogue_gui')


class DialogueGui:
    """One operational page; intentionally contains no jog/manual controls."""

    POLL_MS = 100
    HEALTH_INTERVAL_S = 2.0
    MAX_EVENTS_PER_POLL = 100

    def __init__(self, root, model_client: ModelServiceClient, action_runtime,
                 log_path: Path, speech: StreamingSpeaker | None = None,
                 workspace: Path | None = None,
                 dialogue_settings: DialogueSettings | None = None) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.model_client = model_client
        self.generator = CancellableGeneration(model_client)
        self.dialogue_settings = dialogue_settings or DialogueSettings()
        self.conversation = ShortConversation(
            self.dialogue_settings.max_history_turns,
            self.dialogue_settings.max_history_prompt_bytes)
        # Speech is optional on purpose: the dialogue page must stay usable
        # when the speech service is down, and the checkbox reflects that.
        self.speech = speech
        self._speech_ready = bool(speech is not None and speech.client.available())
        self._playback_busy = False
        self._speech_epoch = 0
        self._speech_start_lock = threading.Lock()
        self._voice_request = ''
        self._reply_request = ''
        self.trace = TraceWriter((workspace / 'evidence/voice/P9/runtime/dialogue-trace.jsonl')
                                 if workspace else None)
        self.voice = (VoiceInput(workspace, lambda e: self.ui_events.put(e),
                                 self._stop_before_capture) if workspace else None)
        self.wake = (WakeInput(workspace, lambda e: self.ui_events.put(e),
                               self._stop_before_capture) if workspace else None)
        self._wake_request = ''
        self._wake_mode = ''
        self._wake_resume_mode = 'wake'
        self._wake_resume_at = 0.0
        self._wake_pending_followup = ''
        self._wake_last_error = ''
        self._wake_followup_deadline = 0.0
        self._wake_followup_heard = False
        self._request_sources: dict[str, str] = {}
        self.action_runtime = action_runtime
        self.bridge = action_runtime.bridge
        self.log_path = log_path
        self.ui_events: queue.Queue[tuple] = queue.Queue()
        self._dialogue_busy = False
        self._closing = False
        self._health_running = False
        self._last_health_started = 0.0
        self._active_action_request = ''
        self._status_values: dict[str, tk.StringVar] = {}
        self._stream_request_id = ''
        self._stream_text = ''
        self._preview_photo = None
        self.capture_directory = (
            workspace / 'evidence/voice/P8/runtime/captures'
            if workspace else Path('/tmp/fr3c_voice_p8_captures'))

        self._configure_window()
        self._build_page()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(self.POLL_MS, self._poll)
        self.root.after(300, self._ensure_wake_listener)

    def _configure_window(self) -> None:
        self.root.title('FR3C 离线智能助手')
        self.root.geometry('940x700')
        self.root.minsize(760, 560)
        self.root.configure(bg='#f3f5f8')
        try:
            if self.root.tk.call('tk', 'windowingsystem') == 'x11':
                self.root.tk.call('tk', 'useinputmethods', True)
        except self.tk.TclError:
            LOG.warning('Tk XIM support is unavailable')
        style = self.ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except self.tk.TclError:
            pass
        style.configure('App.TFrame', background='#f3f5f8')
        style.configure('Card.TFrame', background='#ffffff')
        style.configure(
            'Title.TLabel', background='#f3f5f8', foreground='#152238',
            font=('Noto Sans CJK SC', 18, 'bold'))
        style.configure(
            'Sub.TLabel', background='#f3f5f8', foreground='#65748b',
            font=('Noto Sans CJK SC', 9))
        style.configure(
            'Status.TLabel', background='#ffffff', foreground='#34445b',
            padding=(10, 6), font=('Noto Sans CJK SC', 9))
        style.configure(
            'Wake.TLabel', background='#eef8f3', foreground='#14644f',
            padding=(12, 8), font=('Noto Sans CJK SC', 10, 'bold'))
        style.configure(
            'Primary.TButton', font=('Noto Sans CJK SC', 10, 'bold'),
            padding=(16, 9))
        style.configure('Tool.TButton', padding=(11, 8))

    def _build_page(self) -> None:
        outer = self.ttk.Frame(self.root, style='App.TFrame', padding=18)
        outer.pack(fill='both', expand=True)

        title_row = self.ttk.Frame(outer, style='App.TFrame')
        title_row.pack(fill='x')
        self.ttk.Label(
            title_row, text='FR3C 离线智能助手', style='Title.TLabel').pack(
                side='left')
        self.ttk.Label(
            title_row, text='唤醒语音 · 千问聊天 · 本地知识 · 当前画面 · 阶段 10', style='Sub.TLabel').pack(
                side='right', pady=(8, 0))

        status_card = self.ttk.Frame(outer, style='Card.TFrame', padding=8)
        status_card.pack(fill='x', pady=(12, 10))
        status_top = self.ttk.Frame(status_card, style='Card.TFrame')
        status_top.pack(fill='x')
        for key, title in (
                ('model', '模型'), ('knowledge', '知识'),
                ('vision', '视觉'), ('task', '任务')):
            value = self.tk.StringVar(value=f'{title}：检查中')
            self._status_values[key] = value
            self.ttk.Label(
                status_top, textvariable=value,
                style='Status.TLabel').pack(side='left', padx=4)
        self.ttk.Label(
            status_top, text=f'日志：{self.log_path}',
            style='Status.TLabel').pack(side='right', padx=4)
        wake_value = self.tk.StringVar(value='唤醒：准备中')
        self._status_values['voice'] = wake_value
        self.ttk.Label(
            status_card, textvariable=wake_value,
            style='Wake.TLabel').pack(fill='x', padx=4, pady=(4, 0))

        chat_card = self.ttk.Frame(outer, style='Card.TFrame', padding=12)
        chat_card.pack(fill='both', expand=True)
        self.history = self.tk.Text(
            chat_card, wrap='word', relief='flat', borderwidth=0,
            background='#ffffff', foreground='#1f2a3a',
            font=('Noto Sans CJK SC', 11), padx=12, pady=10,
            state='disabled', cursor='arrow')
        scroll = self.ttk.Scrollbar(
            chat_card, orient='vertical', command=self.history.yview)
        self.history.configure(yscrollcommand=scroll.set)
        self.history.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        self.history.tag_configure(
            'user_name', foreground='#2457a6', font=('Noto Sans CJK SC', 10, 'bold'))
        self.history.tag_configure(
            'assistant_name', foreground='#19705a', font=('Noto Sans CJK SC', 10, 'bold'))
        self.history.tag_configure(
            'system_name', foreground='#996515', font=('Noto Sans CJK SC', 10, 'bold'))
        self.history.tag_configure('body', spacing3=10, lmargin2=12)
        self._append(
            'assistant',
            '你好。可以直接聊天、问“你看到了什么”，也可以发送四色方块抓取命令，'
            '例如“把红色方块放到D区”。')
        if (self.root.tk.call('tk', 'windowingsystem') == 'x11'
                and not os.environ.get('XMODIFIERS')):
            self._append(
                'system',
                '未检测到 XIM 输入法环境；请通过 start_dialogue_gui.sh 启动以使用中文输入。')

        self.preview_card = self.ttk.Frame(
            outer, style='Card.TFrame', padding=8)
        self.preview_label = self.ttk.Label(
            self.preview_card, text='', style='Status.TLabel', compound='left')
        self.preview_label.pack(anchor='w')

        input_row = self.ttk.Frame(outer, style='App.TFrame')
        self.input_row = input_row
        input_row.pack(fill='x', pady=(10, 8))
        self.input_var = self.tk.StringVar()
        self.input_box = self.ttk.Entry(
            input_row, textvariable=self.input_var,
            font=('Noto Sans CJK SC', 11))
        self.input_box.pack(side='left', fill='x', expand=True, ipady=9)
        self.input_box.bind('<Return>', lambda _event: self.send_text())
        self.send_button = self.ttk.Button(
            input_row, text='发送', style='Primary.TButton', command=self.send_text)
        self.send_button.pack(side='left', padx=(10, 0))

        tools = self.ttk.Frame(outer, style='App.TFrame')
        tools.pack(fill='x')
        self.voice_button = self.ttk.Button(
            tools, text='🎙 开始说话', style='Tool.TButton', command=self.toggle_voice)
        self.voice_button.pack(side='left', padx=(0, 7))
        self.wake_enabled = self.tk.BooleanVar(
            value=bool(self.wake is not None and self.wake.enabled_by_default))
        self.wake_check = self.ttk.Checkbutton(
            tools, text='唤醒监听', variable=self.wake_enabled,
            command=self._on_wake_toggle)
        self.wake_check.pack(side='left', padx=7)
        self.ttk.Button(
            tools, text='🖼 截图问答', style='Tool.TButton',
            command=self.ask_image).pack(side='left', padx=7)
        self.tts_enabled = self.tk.BooleanVar(
            value=bool(self.speech is not None))
        self.tts_check = self.ttk.Checkbutton(
            tools, text='自动播报', variable=self.tts_enabled,
            command=self._on_tts_toggle)
        self.tts_check.pack(side='left', padx=12)
        self.stop_speech_button = self.ttk.Button(
            tools, text='停止播报', style='Tool.TButton',
            command=self.stop_playback)
        self.stop_speech_button.pack(side='right', padx=(7, 0))
        self._refresh_speech_controls()
        self.ttk.Button(
            tools, text='停止抓取', style='Tool.TButton',
            command=self.stop_action).pack(side='right', padx=7)
        self.input_box.focus_set()

    def _append(self, role: str, message: str) -> None:
        labels = {'user': '你', 'assistant': '助手', 'system': '系统'}
        name_tag = f'{role}_name' if role in labels else 'system_name'
        label = labels.get(role, '系统')
        self.history.configure(state='normal')
        self.history.insert('end', f'{label}\n', name_tag)
        self.history.insert('end', f'{message.strip()}\n', 'body')
        self.history.configure(state='disabled')
        self.history.see('end')

    def _append_stream_token(self, request_id: str, token: str) -> None:
        """Render model tokens incrementally without touching Tk off-thread."""
        if not token:
            return
        self.history.configure(state='normal')
        if self._stream_request_id != request_id:
            if self._stream_request_id:
                self.history.insert('end', '\n', 'body')
            self._stream_request_id = request_id
            self._stream_text = ''
            self.history.insert('end', '助手\n', 'assistant_name')
        self.history.insert('end', token, 'body')
        self._stream_text += token
        self.history.configure(state='disabled')
        self.history.see('end')

    def _finish_stream(self, request_id: str, answer: str = '') -> None:
        if self._stream_request_id == request_id:
            self.history.configure(state='normal')
            self.history.insert('end', '\n', 'body')
            self.history.configure(state='disabled')
            self.history.see('end')
            self._stream_request_id = ''
            self._stream_text = ''
        elif answer:
            self._append('assistant', answer)

    def _append_knowledge_metadata(self, metadata: dict) -> None:
        mode = metadata.get('mode')
        if mode == 'knowledge':
            lines = [
                f"资料来源 · {metadata.get('version', '未知版本')} · "
                f"检索 {float(metadata.get('elapsed_ms', 0.0)):.0f} ms"
            ]
            for number, source in enumerate(metadata.get('sources', []), 1):
                location = source.get('title') or source.get('source') or '未知资料'
                if source.get('section'):
                    location += ' / ' + str(source['section'])
                if source.get('page') is not None:
                    location += f" / 第{source['page']}页"
                if source.get('ordinal') is not None:
                    location += f" · 片段{source['ordinal']}"
                lines.append(f'[{number}] {location}')
            self._append('system', '\n'.join(lines))
        elif mode == 'no_match':
            self._append('system', '本地知识库未找到可靠出处；回答未引用项目资料。')
        elif mode == 'unavailable' and metadata.get('routed'):
            self._append('system', '知识库暂不可用，本轮已保留普通对话能力。')

    def _show_snapshot(self, snapshot: SavedSnapshot) -> None:
        """Show the exact JPEG sent to Qwen and retain Tk's image reference."""
        try:
            from PIL import Image, ImageTk
            image = Image.open(snapshot.path)
            image.thumbnail((240, 150))
            self._preview_photo = ImageTk.PhotoImage(image)
            caption = (
                f'本轮相机截图  {snapshot.captured_at}  '
                f'{snapshot.width}×{snapshot.height}')
            self.preview_label.configure(
                image=self._preview_photo, text=caption, compound='left')
            if not self.preview_card.winfo_manager():
                self.preview_card.pack(
                    fill='x', pady=(8, 0), before=self.input_row)
        except Exception as error:  # preview failure must not cancel inference
            LOG.warning('unable to display camera snapshot %s: %s', snapshot.path, error)

    def _set_busy(self, busy: bool) -> None:
        self._dialogue_busy = busy
        state = 'disabled' if busy else 'normal'
        self.send_button.configure(state=state)
        self.input_box.configure(state=state)
        if not busy:
            self.input_box.focus_set()

    # -- wake / continuous voice --------------------------------------- #

    def _wake_is_enabled(self) -> bool:
        value = getattr(self, 'wake_enabled', None)
        return bool(self.wake is not None and value is not None and value.get())

    def _pause_wake(self) -> None:
        if self.wake is not None:
            self.wake.cancel(wait=True)
        self._wake_request = ''
        self._wake_mode = ''
        self._wake_followup_deadline = 0.0
        self._wake_followup_heard = False

    def _schedule_wake(self, mode: str = 'wake', delay_s: float = 0.0) -> None:
        if mode not in ('wake', 'followup'):
            mode = 'wake'
        self._wake_resume_mode = mode
        self._wake_resume_at = time.monotonic() + max(0.0, delay_s)

    def _wake_can_listen(self) -> bool:
        if (not self._wake_is_enabled() or self._closing or self._voice_request
                or self._dialogue_busy or self._playback_busy):
            return False
        try:
            robot, _observed = self.bridge.state_snapshot()
            return not (
                self.bridge.guard.snapshot().active
                or robot.get('action_active'))
        except Exception:
            return False

    def _ensure_wake_listener(self) -> None:
        if self.wake is None:
            return
        if not self._wake_is_enabled():
            self._status_values['voice'].set('唤醒：未唤醒 · 监听已关闭')
            return
        if self.wake.active or self._wake_request:
            return
        if not self._wake_can_listen():
            self._status_values['voice'].set('唤醒：未唤醒 · 忙碌时暂停监听')
            return
        now = time.monotonic()
        if now < self._wake_resume_at:
            self._status_values['voice'].set('唤醒：未唤醒 · 即将恢复监听')
            return
        mode = self._wake_resume_mode
        rid = self.wake.start(mode)
        if rid:
            self._wake_request = rid
            self._wake_mode = mode
            self._status_values['voice'].set(
                '唤醒：已唤醒 · 正在准备接收问题' if mode == 'followup'
                else '唤醒：未唤醒 · 正在准备监听 9527')

    def _on_wake_toggle(self) -> None:
        if not self._wake_is_enabled():
            self._pause_wake()
            self._status_values['voice'].set('唤醒：未唤醒 · 监听已关闭')
            return
        self._wake_last_error = ''
        self._schedule_wake('wake')
        self._ensure_wake_listener()

    def _resume_after_request(self, request_id: str,
                              allow_followup: bool = True) -> None:
        source = self._request_sources.pop(request_id, '')
        if source == 'wake' and allow_followup:
            if self._playback_busy:
                self._wake_pending_followup = request_id
                self._status_values['voice'].set('唤醒：已唤醒 · 等待播报结束')
            else:
                delay = float(self.wake.config['resume_delay_s']) if self.wake else 0.0
                self._schedule_wake('followup', delay)
        else:
            self._schedule_wake('wake')

    def _handle_wake_event(self, kind: str, rid: str, data: dict) -> None:
        if rid != self._wake_request:
            return
        if kind == 'wake_waiting':
            phrase = (self.wake.phrases[0] if self.wake and self.wake.phrases
                      else '唤醒词')
            self._status_values['voice'].set(f'唤醒：未唤醒 · 等待 {phrase}')
        elif kind == 'wake_detected':
            self._status_values['voice'].set('唤醒：已唤醒 · 正在接收问题')
        elif kind == 'wake_asr_recording':
            self._status_values['voice'].set('唤醒：已唤醒 · 正在识别本句话')
        elif kind == 'wake_followup_listening':
            self._wake_followup_heard = False
            seconds = float(self.wake.config['followup_initial_silence_s'])
            self._wake_followup_deadline = time.monotonic() + seconds
            self._refresh_wake_countdown(time.monotonic())
        elif kind == 'wake_partial':
            partial = data.get('text', '')
            self.input_var.set(partial)
            if partial.strip():
                self._wake_followup_heard = True
                self._wake_followup_deadline = 0.0
                self._status_values['voice'].set('唤醒：已唤醒 · 正在识别问题')
        elif kind in ('wake_final', 'wake_empty', 'wake_error'):
            mode = data.get('mode') or self._wake_mode
            self._wake_request = ''
            self._wake_mode = ''
            self._wake_followup_deadline = 0.0
            self._wake_followup_heard = False
            if kind == 'wake_error':
                message = data.get('message', '唤醒输入失败')
                self._status_values['voice'].set(f'唤醒：未唤醒 · 暂不可用 · {message}')
                if message != self._wake_last_error:
                    self._append('system', f'唤醒监听暂时失败，将自动重试：{message}')
                    self._wake_last_error = message
                delay = float(self.wake.config['retry_delay_s']) if self.wake else 2.0
                self._schedule_wake('wake', delay)
                return
            self._wake_last_error = ''
            text = data.get('text', '').strip()
            if kind == 'wake_final' and mode == 'wake' and self.wake is not None:
                text = wake_query_from_asr(text, self.wake.phrases)
            self.input_var.set(text)
            if kind == 'wake_final' and text:
                self.send_text(text, request_id=rid, source='wake')
            elif mode == 'wake':
                self._play_wake_acknowledgement(rid)
            else:
                self.input_var.set('')
                self._schedule_wake('wake')

    def _play_wake_acknowledgement(self, request_id: str) -> None:
        """Say one short acknowledgement, then open the question window."""
        self._append('assistant', '我在')
        self._reply_request = request_id
        self._wake_pending_followup = request_id
        self._status_values['voice'].set('唤醒：已唤醒 · 正在提示“我在”')
        self.speak('我在')
        if not self._playback_busy:
            # TTS is disabled or unavailable. Keep the visual acknowledgement
            # and do not hold the microphone window behind a missing service.
            self._wake_pending_followup = ''
            self._schedule_wake('followup')

    def _refresh_wake_countdown(self, now: float) -> None:
        if (self._wake_followup_deadline <= 0.0
                or self._wake_followup_heard
                or self._wake_mode != 'followup'):
            return
        remaining = max(0, math.ceil(self._wake_followup_deadline - now))
        if remaining:
            self._status_values['voice'].set(
                f'唤醒：已唤醒 · 正在接收问题 · {remaining} 秒后退出')
        else:
            self._status_values['voice'].set('唤醒：已唤醒 · 正在结束本次接收')

    def send_text(self, text: str | None = None, request_id: str = '',
                  source: str = '') -> None:
        if self._voice_request:
            return
        text = self.input_var.get().strip() if text is None else text.strip()
        if not text:
            return
        self._pause_wake()
        shortcut = direct_shortcut(text)
        self.input_var.set('')
        self._append('user', text)
        source = source or ('button_voice' if request_id else 'text')
        request_id = request_id or uuid.uuid4().hex
        self._request_sources[request_id] = source
        if shortcut is not None:
            self._reply_request = request_id
            self._handle_intent(request_id, text, shortcut)
            return
        if self._dialogue_busy or self.bridge.guard.snapshot().active:
            self._append('system', '当前请求尚未结束。可随时使用“停止抓取”。')
            self._resume_after_request(request_id, allow_followup=False)
            return
        self._set_busy(True)
        self._reply_request = request_id
        self.trace.write('input_submitted', request_id, source=source)
        local_route = safe_local_non_action_route(text)
        if local_route is not None:
            LOG.info(
                'local_route request_id=%s route=%s',
                request_id, local_route.route)
            self._handle_intent(request_id, text, local_route)
            return
        local_action = safe_local_action_route(text)
        if local_action is not None:
            LOG.info(
                'local_action_route request_id=%s task=%s color=%s zone=%s '
                'support_color=%s', request_id, local_action.task,
                local_action.color, local_action.zone,
                local_action.support_color)
            self.trace.write(
                'local_action_routed', request_id, task=local_action.task,
                color=local_action.color, zone=local_action.zone,
                support_color=local_action.support_color)
            self._handle_intent(request_id, text, local_action)
            return
        threading.Thread(
            target=self._classify_worker,
            args=(request_id, text), daemon=True).start()

    def _classify_worker(self, request_id: str, text: str) -> None:
        try:
            intent, raw = classify_text(
                text,
                lambda prompt: self.generator.generate(
                    prompt, request_id=f'{request_id}-route'))
            LOG.info(
                'intent request_id=%s route=%s task=%s color=%s zone=%s '
                'support_color=%s raw=%r',
                request_id, intent.route, intent.task, intent.color,
                intent.zone, intent.support_color, raw)
            self.ui_events.put(('intent', request_id, text, intent))
        except IntentProtocolError as error:
            LOG.warning(
                'intent_rejected request_id=%s code=%s detail=%s',
                request_id, error.code, error.detail)
            self.ui_events.put(('protocol_error', request_id, error.code))
        except Exception as error:
            LOG.exception('classification_failed request_id=%s', request_id)
            self.ui_events.put(('model_error', request_id, str(error)))

    def _handle_intent(self, request_id: str, text: str,
                       intent: DialogueIntent) -> None:
        if intent.route == 'stop_action':
            self.stop_action(message=intent.reply)
            self._resume_after_request(request_id, allow_followup=False)
            return
        if intent.route == 'stop_playback':
            self.stop_playback(message=intent.reply)
            self._resume_after_request(request_id)
            return
        if intent.route == 'release_ack':
            self.acknowledge_release(message=intent.reply)
            self._resume_after_request(request_id, allow_followup=False)
            return
        if intent.route == 'action':
            self._append('assistant', intent.reply)
            self.speak(intent.reply)
            try:
                self.bridge.send(
                    request_id, intent.task or '', intent.color or '',
                    intent.zone or '', intent.support_color or '')
                self._active_action_request = request_id
                self._set_busy(True)
                LOG.info(
                    'action_dispatched request_id=%s task=%s color=%s zone=%s '
                    'support_color=%s', request_id, intent.task, intent.color,
                    intent.zone, intent.support_color)
            except Exception as error:
                LOG.exception('action_dispatch_failed request_id=%s', request_id)
                self._append('system', f'未发起抓取：{error}')
                self._set_busy(False)
                self._resume_after_request(request_id, allow_followup=False)
            return
        if intent.route == 'vision':
            self._start_vision_answer(request_id, text)
            return
        if intent.route == 'chat':
            self._start_chat_answer(request_id, text)
            return
        self._append('assistant', intent.reply)
        self.speak(intent.reply)
        self._set_busy(False)
        self._resume_after_request(request_id)

    def ask_image(self) -> None:
        from tkinter import simpledialog

        if self._voice_request or self._dialogue_busy or self.bridge.guard.snapshot().active:
            self._append('system', '任务运行中，暂不启动视觉问答。')
            return
        self._pause_wake()
        question = simpledialog.askstring(
            '当前画面问答', '想问当前相机画面什么？', initialvalue='请简要描述当前画面。',
            parent=self.root)
        if not question or not question.strip():
            self._schedule_wake('wake')
            return
        question = question.strip()
        self._append('user', f'[当前相机截图] {question}')
        self._set_busy(True)
        request_id = uuid.uuid4().hex
        self._reply_request = request_id
        self._request_sources[request_id] = 'snapshot_button'
        self.trace.write('input_submitted', request_id, source='snapshot_button')
        self._start_vision_answer(request_id, question)

    def _generation_speech_options(self) -> tuple[bool, int]:
        enabled = bool(
            self.speech is not None
            and self._speech_ready
            and self.tts_enabled.get()
            and not self._voice_request)
        self._speech_epoch += 1
        return enabled, self._speech_epoch

    def _start_chat_answer(self, request_id: str, text: str) -> None:
        try:
            prompt = self.conversation.build_prompt(text)
        except ValueError as error:
            self._append('system', str(error))
            self._set_busy(False)
            self._resume_after_request(request_id, allow_followup=False)
            return
        speak_enabled, speech_epoch = self._generation_speech_options()
        self.trace.write('chat_generation_start', request_id)
        threading.Thread(
            target=self._chat_worker,
            args=(request_id, text, prompt, speak_enabled, speech_epoch),
            daemon=True).start()

    def _start_vision_answer(self, request_id: str, question: str) -> None:
        try:
            frame = self.bridge.latest_color_frame(
                self.dialogue_settings.camera_max_age_s)
        except Exception as error:
            LOG.warning('camera snapshot unavailable request_id=%s: %s', request_id, error)
            self._append('system', f'无法进行当前画面问答：{error}')
            self._set_busy(False)
            self._resume_after_request(request_id, allow_followup=False)
            return
        speak_enabled, speech_epoch = self._generation_speech_options()
        self.trace.write(
            'vision_capture_selected', request_id,
            ros_stamp_ns=frame.ros_stamp_ns,
            age_s=max(0.0, time.monotonic() - frame.observed_monotonic))
        threading.Thread(
            target=self._vision_worker,
            args=(request_id, question, frame, speak_enabled, speech_epoch),
            daemon=True).start()

    def _begin_generation_speech(self, request_id: str,
                                 enabled: bool, epoch: int) -> bool:
        if not enabled or self.speech is None:
            return False
        with self._speech_start_lock:
            if self._closing or epoch != self._speech_epoch or self._voice_request:
                return False
            started = self.speech.start(request_id=request_id)
        if started:
            self.ui_events.put(('speech_stream_started', epoch))
        elif self.speech.last_error:
            LOG.warning('stream speech did not start: %s', self.speech.last_error)
        return started

    def _generate_answer(self, request_id: str, prompt: str,
                         model_request_id: str, image_path: Path | None,
                         speak_enabled: bool, speech_epoch: int,
                         knowledge_question: str = '') -> str:
        speech_started = self._begin_generation_speech(
            request_id, speak_enabled, speech_epoch)

        def on_token(token: str) -> None:
            self.ui_events.put(('answer_token', request_id, token))
            if speech_started and self.speech is not None:
                self.speech.feed(token)

        knowledge_metadata: dict = {}
        arguments = {
            'request_id': model_request_id,
            'image_path': image_path,
            'on_token': on_token,
        }
        if knowledge_question:
            arguments['knowledge_question'] = knowledge_question
            arguments['on_knowledge'] = knowledge_metadata.update
        answer = self.generator.generate(prompt, **arguments).strip()
        if not answer:
            raise ModelServiceError('模型返回空回答')
        self.ui_events.put((
            'answer_result', request_id, answer, dict(knowledge_metadata)))
        if speech_started and self.speech is not None:
            metrics = self.speech.finish(timeout_s=900.0)
            self.trace.write('speech_result', request_id, metrics=metrics)
            self.ui_events.put(('speech_done', speech_epoch, metrics))
        return answer

    def _chat_worker(self, request_id: str, user_text: str, prompt: str,
                     speak_enabled: bool, speech_epoch: int) -> None:
        try:
            answer = self._generate_answer(
                request_id, prompt, f'{request_id}-chat', None,
                speak_enabled, speech_epoch, knowledge_question=user_text)
            self.conversation.record(user_text, answer)
            self.trace.write(
                'chat_generation_end', request_id,
                output_chars=len(answer), history_turns=len(self.conversation.turns))
        except Exception as error:
            LOG.exception('chat_generation_failed request_id=%s', request_id)
            if self.speech is not None:
                self.speech.stop('model_error')
            self.ui_events.put(('generation_error', request_id, str(error)))

    def _vision_worker(self, request_id: str, question: str, frame,
                       speak_enabled: bool, speech_epoch: int) -> None:
        try:
            settings = self.dialogue_settings
            snapshot = save_snapshot(
                frame, self.capture_directory,
                max_edge=settings.capture_max_edge,
                jpeg_quality=settings.capture_jpeg_quality,
                max_saved=settings.max_saved_captures)
            self.ui_events.put(('snapshot_ready', request_id, snapshot))
            prompt = build_vision_prompt(question, snapshot.captured_at)
            answer = self._generate_answer(
                request_id, prompt, f'{request_id}-vision', snapshot.path,
                speak_enabled, speech_epoch)
            self.trace.write(
                'vision_generation_end', request_id,
                image_path=str(snapshot.path), ros_stamp_ns=snapshot.ros_stamp_ns,
                output_chars=len(answer))
        except Exception as error:
            LOG.exception('vision_question_failed request_id=%s', request_id)
            if self.speech is not None:
                self.speech.stop('model_error')
            self.ui_events.put(('generation_error', request_id, str(error)))

    def _stop_before_capture(self) -> None:
        # Serialize against a speech worker which has not opened its session yet.
        with self._speech_start_lock:
            if self.speech is not None and self.speech.client.socket_path.exists():
                self.speech.client.stop('recording')  # errors must prevent capture

    def toggle_voice(self) -> None:
        if self._voice_request:
            self.voice.finish()
            self.voice_button.configure(text='正在定稿…', state='disabled')
            return
        if self.voice is None:
            self._append('system', '语音输入未配置。')
            return
        robot, _ = self.bridge.state_snapshot()
        if (self._dialogue_busy or self.bridge.guard.snapshot().active
                or robot.get('action_active')):
            self._append('system', '当前请求尚未结束。停止抓取按钮始终可用。')
            return
        self._pause_wake()
        self._schedule_wake('wake')
        self._speech_epoch += 1
        self._playback_busy = False
        self._voice_request = self.voice.start()
        if self._voice_request:
            self._set_busy(True)
            self.voice_button.configure(text='准备麦克风…')
            self.input_var.set('')
            self._refresh_speech_controls()

    def _handle_voice_event(self, kind, rid, data) -> None:
        if rid != self._voice_request:
            return
        if kind == 'asr_recording':
            self.voice_button.configure(text='结束说话')
        elif kind == 'asr_partial':
            self.input_var.set(data.get('text', ''))  # replace, never append revisions
        elif kind == 'asr_finalizing':
            self.voice_button.configure(text='正在定稿…', state='disabled')
        elif kind in ('asr_final', 'asr_empty', 'asr_error'):
            self._voice_request = ''  # consume once before dispatch
            self.voice_button.configure(text='🎙 开始说话', state='normal')
            self._set_busy(False)
            self.input_var.set(data.get('text', ''))
            if kind == 'asr_final':
                self.send_text(data['text'], request_id=rid)
            else:
                self._append('system', data.get('message') or '没有识别到清晰语音，请再说一次。')
                self._schedule_wake('wake')

    # -- speech ---------------------------------------------------------- #

    def _refresh_speech_controls(self) -> None:
        """Grey out the speech controls whenever the service is unavailable."""
        if self.speech is None:
            self.tts_check.configure(state='disabled')
            self.stop_speech_button.configure(state='disabled')
            return
        self.tts_check.configure(state='normal')
        self.stop_speech_button.configure(
            state='normal' if (self._speech_ready and self._playback_busy) else 'disabled')

    def _on_tts_toggle(self) -> None:
        if self.speech is None:
            return
        self.speech.enabled = bool(self.tts_enabled.get())
        if not self.speech.enabled and self._playback_busy:
            self.stop_playback('已关闭自动播报。')

    def speak(self, text: str) -> None:
        """Speak a reply that is already known in full (validated intent reply).

        Playback runs on a worker thread so the page stays responsive and the
        stop button can cut the sentence off.
        """
        if self._voice_request or self.speech is None or not self.tts_enabled.get():
            return
        clean = (text or '').strip()
        if not clean:
            return
        if not self._speech_ready:
            self._speech_ready = self.speech.client.available()
            if not self._speech_ready:
                self._append('system', '语音服务未运行，本条回复未播报。')
                LOG.warning('speech service unavailable; reply not spoken')
                return
        self._playback_busy = True
        self._refresh_speech_controls()

        self._speech_epoch += 1
        epoch = self._speech_epoch
        rid = self._reply_request or uuid.uuid4().hex

        def worker() -> None:
            try:
                with self._speech_start_lock:
                    if epoch != self._speech_epoch or self._voice_request or self._closing:
                        return
                    self.speech.client.begin(request_id=rid)
                for index in range(0, len(clean), 24):
                    if epoch != self._speech_epoch:
                        return
                    self.speech.client.feed(rid, clean[index:index + 24])
                result = self.speech.client.finish(rid)
                self.trace.write('speech_result', rid, metrics=result)
                self.ui_events.put(('speech_done', epoch, result))
            except SpeechServiceError as error:
                LOG.warning('speech failed: %s', error)
                self.ui_events.put(('speech_failed', epoch, str(error)))

        threading.Thread(target=worker, daemon=True).start()

    def _on_speech_result(self, result: dict, failed: str = '') -> None:
        self._playback_busy = False
        self._refresh_speech_controls()
        if failed:
            self._append('system', f'播报失败：{failed}')
            if self._wake_pending_followup:
                self._wake_pending_followup = ''
                delay = float(self.wake.config['resume_delay_s']) if self.wake else 0.0
                self._schedule_wake('followup', delay)
            return
        first = result.get('first_valid_pcm_write_s')
        end = result.get('playback_end_s')
        if first is not None and end is not None:
            LOG.info('speech playback first_write=%.3fs playback_end=%.3fs',
                     first, end)
        if self._wake_pending_followup:
            self._wake_pending_followup = ''
            delay = float(self.wake.config['resume_delay_s']) if self.wake else 0.0
            self._schedule_wake('followup', delay)

    def stop_playback(self, message: str = '') -> None:
        self._speech_epoch += 1
        if message:
            self._append('system', message)
        if self.speech is None:
            return
        try:
            with self._speech_start_lock:
                self.speech.stop('gui_stop')
        except SpeechServiceError as error:
            LOG.warning('stop playback failed: %s', error)
        self._playback_busy = False
        self._refresh_speech_controls()
        if self._wake_pending_followup:
            self._wake_pending_followup = ''
            delay = float(self.wake.config['resume_delay_s']) if self.wake else 0.0
            self._schedule_wake('followup', delay)

    def stop_action(self, message: str = '') -> None:
        if message:
            self._append('assistant', message)
        # This call is deliberately synchronous with the button/shortcut path
        # and never waits behind model generation.
        requested = self.bridge.cancel_direct()
        if not requested:
            self._append('system', '停止请求未送达；取消服务当前不可用。')

    def acknowledge_release(self, message: str = '') -> None:
        if message:
            self._append('assistant', message)
        requested = self.bridge.acknowledge_release_direct()
        if not requested:
            LOG.warning('release acknowledgement was not dispatched')

    def _poll(self) -> None:
        if self._closing:
            return
        try:
            self._drain_ui_events()
            self._drain_action_events()
            now = time.monotonic()
            if (not self._health_running
                    and now - self._last_health_started >= self.HEALTH_INTERVAL_S):
                self._health_running = True
                self._last_health_started = now
                threading.Thread(target=self._health_worker, daemon=True).start()
                if self.speech is not None:
                    threading.Thread(
                        target=self._speech_health_worker, daemon=True).start()
            self._refresh_local_status(now)
            self._refresh_wake_countdown(now)
            self._ensure_wake_listener()
        except Exception:  # noqa: BLE001 - one bad event must not stop Tk polling
            LOG.exception('unexpected error in GUI poll; continuing next tick')
        finally:
            if not self._closing:
                self.root.after(self.POLL_MS, self._poll)

    def _health_worker(self) -> None:
        try:
            health_client = ModelServiceClient(
                self.model_client.socket_path, timeout_s=1.5)
            health = health_client.health()
            ready = bool(health.get('healthy')) and health.get('state') == 'ready'
            self.ui_events.put((
                'model_health', ready, health.get('state', 'unknown'),
                health.get('knowledge', {})))
        except Exception as error:
            self.ui_events.put(('model_health', False, str(error)))
        finally:
            self._health_running = False

    def _speech_health_worker(self) -> None:
        try:
            ready = self.speech.client.available() if self.speech else False
        except Exception:  # noqa: BLE001 - a health probe must never raise
            ready = False
        self.ui_events.put(('speech_health', ready))

    def _refresh_local_status(self, now: float) -> None:
        try:
            task_ready = self.bridge.action_servers_ready()
        except Exception:
            task_ready = False
        snapshot = self.bridge.guard.snapshot()
        robot_state, _observed = self.bridge.state_snapshot()
        if snapshot.active:
            self._status_values['task'].set(f'任务：运行中 · {snapshot.phase}')
        elif robot_state.get('action_active'):
            self._status_values['task'].set('任务：运行中')
        elif robot_state.get('state') == 'complete_holding':
            self._status_values['task'].set('任务：等待确认物体已取下')
        elif robot_state.get('state') == 'canceled_holding_state_unchanged':
            self._status_values['task'].set('任务：已取消，等待恢复')
        elif robot_state.get('retryable') is False:
            self._status_values['task'].set('任务：未就绪 · ' + robot_state.get('state', '未知'))
        else:
            self._status_values['task'].set(
                '任务：就绪' if task_ready else '任务：Action 未就绪')
        perception, observed = self.bridge.perception_snapshot()
        if not observed or now - observed > 3.5:
            self._status_values['vision'].set('视觉：状态未知')
        elif perception.get('inference_enabled') is False:
            self._status_values['vision'].set('视觉：推理暂停')
        else:
            self._status_values['vision'].set('视觉：就绪')

    def _drain_ui_events(self) -> None:
        for _ in range(self.MAX_EVENTS_PER_POLL):
            try:
                event = self.ui_events.get_nowait()
            except queue.Empty:
                return
            kind = event[0]
            if kind.startswith('wake_'):
                self._handle_wake_event(kind, event[1], event[2])
            elif kind.startswith('asr_'):
                self._handle_voice_event(kind, event[1], event[2])
            elif kind == 'intent':
                self._handle_intent(event[1], event[2], event[3])
            elif kind == 'protocol_error':
                if event[2] in {
                        'invalid_zone', 'action_missing_zone',
                        'zone_not_unique', 'model_zone_mismatch'}:
                    message = (
                        '没有识别到具体放置区域，本轮未发起机械动作。'
                        '按颜色放到默认区域请说“把红色方块放到分类区”；'
                        '指定区域请说“把红色方块放到D区”或“A3”。')
                else:
                    message = (
                        '这句话未通过安全意图校验，未发起抓取。'
                        '请明确任务和一种颜色。')
                self._append('system', message)
                self._set_busy(False)
                self._resume_after_request(event[1], allow_followup=False)
            elif kind == 'model_error':
                self._append('system', '模型服务暂不可用。本轮没有发起任何机械动作，详情已写入日志。')
                self._set_busy(False)
                self._resume_after_request(event[1], allow_followup=False)
            elif kind == 'answer_token':
                self._append_stream_token(event[1], event[2])
            elif kind == 'answer_result':
                self._finish_stream(event[1], event[2])
                if len(event) > 3 and event[3]:
                    self._append_knowledge_metadata(event[3])
                self._set_busy(False)
                self._resume_after_request(event[1])
            elif kind == 'generation_error':
                self._finish_stream(event[1])
                self._append('system', f'本轮回答失败：{event[2]}')
                self._set_busy(False)
                self._resume_after_request(event[1], allow_followup=False)
            elif kind == 'snapshot_ready':
                self._show_snapshot(event[2])
            elif kind == 'model_health':
                ready, detail = event[1], event[2]
                self._status_values['model'].set(
                    '模型：就绪' if ready else f'模型：未就绪 · {detail}')
                knowledge = event[3] if len(event) > 3 else {}
                if knowledge.get('available'):
                    self._status_values['knowledge'].set(
                        f"知识：就绪 · {knowledge.get('chunks', 0)}片段")
                else:
                    reason = str(knowledge.get('last_error') or '不可用')
                    self._status_values['knowledge'].set(
                        f'知识：已降级 · {reason[:28]}')
            elif kind == 'speech_done':
                if event[1] == self._speech_epoch:
                    self._on_speech_result(event[2])
            elif kind == 'speech_stream_started':
                if event[1] == self._speech_epoch:
                    self._playback_busy = True
                    self._refresh_speech_controls()
            elif kind == 'speech_failed':
                if event[1] == self._speech_epoch:
                    self._on_speech_result({}, str(event[2]))
            elif kind == 'speech_health':
                ready = bool(event[1])
                if ready != self._speech_ready:
                    self._speech_ready = ready
                    LOG.info('speech service availability changed to %s', ready)
                self._refresh_speech_controls()

    def _drain_action_events(self) -> None:
        for _ in range(self.MAX_EVENTS_PER_POLL):
            try:
                event = self.bridge.events.get_nowait()
            except queue.Empty:
                return
            LOG.info(
                'action_event kind=%s request_id=%s phase=%s success=%s '
                'error_code=%s executed=%s detail=%s',
                event.kind, event.request_id, event.phase, event.success,
                event.error_code, event.executed, event.detail)
            if event.kind == 'accepted':
                self._append('system', '抓取请求已接受，正在等待实际执行结果。')
            elif event.kind == 'result':
                if event.success and event.executed:
                    color = COLOR_DISPLAY.get(event.color, event.color)
                    destination = (
                        f'，已放到{event.zone.upper()}区域'
                        if event.zone else
                        f'，已码放到{COLOR_DISPLAY.get(event.support_color, event.support_color)}'
                        '方块上'
                        if event.support_color else '')
                    self._append(
                        'assistant', f'任务已实际完成：{color}方块{destination}。')
                elif event.success:
                    self._append('assistant', '任务规划完成，但未执行真实运动。')
                elif event.phase == 'canceled':
                    self._append('system', '抓取任务已取消。')
                elif event.error_code == 10:
                    self._append(
                        'system',
                        '上一抓取保持任务仍处于持物锁定。确认物体确实已经取下后，'
                        '请输入“物体已取下”，再发送新任务。')
                elif event.error_code == 13:
                    self._append(
                        'system', f'指定区域无效：{event.detail or "请检查区域名"}')
                elif event.error_code == 14:
                    self._append(
                        'system', f'指定区域暂时不可用：{event.detail or "没有空闲槽位"}')
                else:
                    self._append('system', f'抓取未完成：{event.detail or "未知错误"}')
                self._active_action_request = ''
                self._set_busy(False)
                self._resume_after_request(event.request_id, allow_followup=False)
            elif event.kind == 'cancel_requested':
                self._append('system', event.detail)
            elif event.kind in {'cancel_result', 'cancel_failed'}:
                if event.success is False:
                    self._append('system', f'停止失败：{event.detail}')
            elif event.kind == 'release_ack_requested':
                self._append('system', event.detail)
            elif event.kind == 'release_ack_result':
                if event.success:
                    self._append(
                        'assistant',
                        '上一持物状态已解除。请等待视觉重新稳定后发送下一任务。')
                else:
                    self._append('system', f'解除持物状态失败：{event.detail}')
            elif event.kind == 'release_ack_failed':
                self._append('system', event.detail)

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self._speech_epoch += 1
        if self.wake is not None:
            self.wake.close()
        if self.voice is not None:
            self.voice.close()
        try:
            self.generator.cancel()
        except Exception:
            LOG.exception('failed to cancel model request during shutdown')
        try:
            if self.speech is not None:
                self.speech.stop('gui_close')
        except Exception:
            LOG.exception('failed to stop speech during shutdown')
        try:
            self.action_runtime.close()
        except Exception:
            LOG.exception('failed to close ROS runtime')
        self.root.destroy()


def _configure_logging(workspace: Path) -> Path:
    log_directory = workspace / 'evidence/voice/P9/runtime'
    log_directory.mkdir(parents=True, exist_ok=True)
    path = log_directory / f'dialogue-{datetime.now():%Y%m%d_%H%M%S}.log'
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s %(message)s',
        handlers=[logging.FileHandler(path, encoding='utf-8')])
    return path


# The phase-6 speech service is a separate process that owns the speaker. Its
# socket lives under the P6 runtime directory, so neither service can be started
# twice without the second copy noticing.
SPEECH_SOCKET_NAME = 'fr3c-tts-pipeline.sock'


def speech_socket_path(workspace: Path) -> Path:
    return workspace / 'evidence/voice/P6/runtime' / SPEECH_SOCKET_NAME


def validate_runtime(config_path: Path, workspace: Path) -> tuple[Path, Path]:
    """Validate paths without constructing Tk or connecting to the robot."""
    _qwen, service = load_config(config_path, workspace)
    load_dialogue_settings(config_path)
    socket_path = service.runtime_dir / service.service_socket_name
    return socket_path, workspace / 'evidence/voice/P9/runtime'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    workspace_default = Path(__file__).resolve().parents[3]
    parser.add_argument('--workspace', type=Path, default=workspace_default)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--model-socket', type=Path, help='reuse an existing Qwen service')
    parser.add_argument('--speech-socket', type=Path, help='reuse an existing TTS service')
    parser.add_argument(
        '--validate-only', action='store_true',
        help='validate dialogue configuration without starting Tk or ROS')
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    config_path = (
        args.config.resolve() if args.config
        else workspace / 'config/voice/qwen3vl.yaml')
    socket_path, runtime_path = validate_runtime(config_path, workspace)
    dialogue_settings = load_dialogue_settings(config_path)
    socket_path = args.model_socket or socket_path
    if args.validate_only:
        VoiceInput(workspace, lambda event: None).validate()
        WakeInput(workspace, lambda event: None).validate()
        speech_path = args.speech_socket or speech_socket_path(workspace)
        available = SpeechClient(speech_path).available()
        print(f'OK socket={socket_path} gui_runtime={runtime_path} '
              f'speech_socket={speech_path} speech_available={available}')
        return 0

    import tkinter as tk
    from .action_bridge import RosActionRuntime

    log_path = _configure_logging(workspace)
    root = tk.Tk()
    runtime = None
    try:
        runtime = RosActionRuntime(color_topic=dialogue_settings.camera_topic)
        speech_path = args.speech_socket or speech_socket_path(workspace)
        speech_client = SpeechClient(speech_path)
        if speech_client.available():
            LOG.info('speech service ready at %s', speech_path)
        else:
            LOG.warning(
                'speech service not available at %s; replies will not be spoken',
                speech_path)
        speech = StreamingSpeaker(speech_client, enabled=True)
        DialogueGui(
            root, ModelServiceClient(socket_path), runtime, log_path,
            speech=speech, workspace=workspace,
            dialogue_settings=dialogue_settings)
        root.mainloop()
        return 0
    except Exception:
        LOG.exception('dialogue GUI stopped unexpectedly')
        if runtime is not None:
            runtime.close()
        try:
            root.destroy()
        except tk.TclError:
            pass
        raise


if __name__ == '__main__':
    raise SystemExit(main())
