"""Optional Tk page and local client; no imports or calls to robot controls."""

from __future__ import annotations

import base64
from datetime import datetime
import io
import json
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import threading
import time


MODE_LABELS = {'物体分类': 'objects', '颜色识别': 'colors', '人脸追踪': 'faces'}
STATE_LABELS = {
    'stopped': '已停止 · 原始画面', 'starting': '正在准备', 'running': '识别中',
    'tracking': '人脸跟踪中', 'searching': '寻找人脸', 'temporarily_lost': '人脸暂时丢失',
    'paused': '已暂停 · 原有任务优先', 'waiting_camera': '等待相机画面',
    'camera_stale': '相机画面已过期', 'error': '识别失败',
    'waiting_result': '正在处理最新画面',
}
REASON_LABELS = {
    'robot_busy': '机械臂任务运行中', 'servo_busy': '原有视觉跟随运行中',
    'foreground_npu_perception': '原有 RKNN 感知正在使用 NPU',
    'foreground_perception_paused': '原有感知已暂停',
    'foreground_ui_busy': '对话、录音或机械任务运行中',
    'model_busy': '大模型正在推理', 'model_not_ready': '大模型正在准备或恢复',
    'model_status_unavailable': '暂时无法确认大模型状态',
    'checking_model': '正在检查大模型状态', 'model_status_stale': '大模型状态已过期',
    'grasp_status_stale': '机械任务状态已过期',
    'perception_status_stale': '原有感知状态已过期',
    'servo_status_stale': '原有跟随状态已过期',
}


class EdgeVisionClient:
    def __init__(self, workspace, timeout_s=0.6):
        domain = int(os.getenv('ROS_DOMAIN_ID', '0'))
        self.path = Path(workspace) / 'evidence/vision/runtime' / f'edge-vision-{domain}.sock'
        self.timeout_s = timeout_s

    def request(self, operation, **kwargs):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(self.timeout_s)
            connection.connect(str(self.path))
            connection.sendall(json.dumps({'op': operation, **kwargs}).encode('utf-8') + b'\n')
            with connection.makefile('rb') as stream:
                raw = stream.readline(4 * 1024 * 1024 + 1)
        if len(raw) > 4 * 1024 * 1024:
            raise ValueError('edge vision response too large')
        value = json.loads(raw)
        if not value.get('ok'):
            raise RuntimeError(value.get('error', 'edge vision request failed'))
        return value


class EdgeVisionPage:
    """Independent polling, lazy process launch and main-thread-only Tk updates."""

    def __init__(self, notebook, workspace, foreground_busy=None):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk = tk, ttk
        self.notebook = notebook
        self.workspace = Path(workspace)
        self.foreground_busy = foreground_busy or (lambda: False)
        self.client = EdgeVisionClient(workspace)
        self.events = queue.Queue(maxsize=8)
        self.closing = threading.Event()
        self.process = None
        self.process_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.fetching = False
        self.commanding = False
        self.was_visible = False
        self.recognizing = False
        self.generation = 0
        self.photo = None
        self.last_image = None
        self.last_result = None
        self.last_frame_received = 0.0
        self.screenshot_index = 0
        self.font = None
        self.page = ttk.Frame(notebook, style='App.TFrame', padding=(12, 10))
        notebook.add(self.page, text='  端侧视觉感知  ')
        self._build()
        self.timer = self.page.after(200, self._tick)

    def _build(self):
        ttk, tk = self.ttk, self.tk
        tools = ttk.Frame(self.page, style='Card.TFrame', padding=12)
        tools.pack(fill='x')
        ttk.Label(tools, text='端侧视觉感知', style='Section.TLabel').pack(side='left')
        self.mode = tk.StringVar(value='物体分类')
        combo = ttk.Combobox(tools, textvariable=self.mode, values=tuple(MODE_LABELS),
                             state='readonly', width=12)
        combo.pack(side='left', padx=(18, 10))
        combo.bind('<<ComboboxSelected>>', self._mode_changed)
        self.start_button = ttk.Button(tools, text='开始识别', style='Primary.TButton',
                                       command=self.start)
        self.start_button.pack(side='left', padx=4)
        self.stop_button = ttk.Button(tools, text='停止识别', style='Tool.TButton',
                                      command=self.stop)
        self.stop_button.pack(side='left', padx=4)
        self.save_button = ttk.Button(tools, text='保存截图', style='Tool.TButton',
                                      command=self.save)
        self.save_button.pack(side='right')
        ttk.Label(self.page, text='共用当前相机 · 切换展示三个功能 · 离开本页停止识别',
                  style='Sub.TLabel').pack(anchor='w', pady=(9, 6))
        content = ttk.Frame(self.page, style='App.TFrame')
        content.pack(fill='both', expand=True)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=2)
        content.rowconfigure(0, weight=1)
        preview = ttk.Frame(content, style='Card.TFrame', padding=8)
        preview.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        self.image_label = ttk.Label(preview, text='等待当前相机画面',
                                     anchor='center', background='#172338', foreground='#d9e2ef')
        self.image_label.pack(fill='both', expand=True)
        results = ttk.Frame(content, style='Card.TFrame', padding=12)
        results.grid(row=0, column=1, sticky='nsew')
        self.status = tk.StringVar(value='已停止 · 原始画面')
        ttk.Label(results, textvariable=self.status, style='Section.TLabel',
                  wraplength=300).pack(anchor='w', pady=(0, 10))
        self.summary = tk.StringVar(value='当前目标：0')
        ttk.Label(results, textvariable=self.summary, style='Hint.TLabel',
                  wraplength=300).pack(anchor='w', pady=(0, 10))
        self.tree = ttk.Treeview(results, columns=('name', 'category', 'score'),
                                 show='headings', height=10)
        for key, title, width in (('name', '目标', 90), ('category', '类别 / ID', 95),
                                   ('score', '检测置信度', 78)):
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, minwidth=45, stretch=True)
        self.tree.pack(fill='both', expand=True)
        self.detail = tk.StringVar(value='物体：现有 RKNN 模型归并四大类；颜色：红黄蓝绿；人脸：单人轨迹跟踪。')
        ttk.Label(results, textvariable=self.detail, style='Hint.TLabel',
                  wraplength=300, justify='left').pack(anchor='w', pady=(12, 0))
        self.metrics = tk.StringVar(value='相机：等待画面')
        ttk.Label(self.page, textvariable=self.metrics, style='Sub.TLabel',
                  wraplength=820).pack(fill='x', pady=(10, 0))

    def _put(self, item):
        if self.closing.is_set():
            return
        try:
            self.events.put_nowait(item)
        except queue.Full:
            pass

    def _ensure_backend(self):
        try:
            self.client.request('status')
            return
        except (OSError, ValueError, RuntimeError):
            pass
        with self.process_lock:
            if self.closing.is_set():
                raise RuntimeError('page is closing')
            if self.process is None or self.process.poll() is not None:
                script = self.workspace / 'scripts/vision/start_edge_vision.sh'
                folder = self.client.path.parent
                folder.mkdir(parents=True, exist_ok=True, mode=0o700)
                with (folder / 'edge-vision.log').open('a', encoding='utf-8') as log:
                    self.process = subprocess.Popen(
                        ['bash', str(script)], cwd=str(self.workspace),
                        stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                        start_new_session=True)
        deadline = time.monotonic() + 8
        while not self.closing.is_set() and time.monotonic() < deadline:
            try:
                self.client.request('status')
                return
            except (OSError, ValueError, RuntimeError):
                self.closing.wait(0.15)
        raise RuntimeError('视觉服务未就绪，请检查 evidence/vision/runtime/edge-vision.log')

    def _command(self, operation, mode=None):
        self.generation += 1
        generation = self.generation
        self.commanding = True
        self.last_image = self.last_result = None
        self.image_label.configure(image='', text='正在切换画面')
        self.tree.delete(*self.tree.get_children())
        self.summary.set('当前目标：0')
        self.status.set('正在准备' if operation == 'start' else '正在停止')
        self.start_button.configure(state='disabled')
        self.stop_button.configure(state='disabled')

        def run():
            with self.command_lock:
                if self.closing.is_set() or generation != self.generation:
                    return
                try:
                    self._ensure_backend()
                    if self.closing.is_set() or generation != self.generation:
                        return
                    payload = {'mode': mode} if mode else {}
                    value = self.client.request(operation, **payload)
                    self._put(('command', generation, value))
                except Exception as error:
                    self._put(('error', generation, str(error)))
        threading.Thread(target=run, daemon=True).start()

    def start(self):
        self.recognizing = True
        self._command('start', MODE_LABELS[self.mode.get()])

    def stop(self):
        self.recognizing = False
        self._command('stop')

    def _mode_changed(self, _event=None):
        # Selecting a feature stops the previous one; the start button is explicit.
        self.stop()

    def _fetch(self, generation, busy):
        try:
            self._ensure_backend()
            value = self.client.request('snapshot', foreground_busy=busy)
            self._put(('snapshot', generation, value))
        except Exception as error:
            self._put(('error', generation, str(error)))
        finally:
            self.fetching = False

    def _tick(self):
        if self.closing.is_set():
            return
        try:
            visible = self.notebook.select() == str(self.page)
            if self.was_visible and not visible and self.recognizing:
                self.stop()
            self.was_visible = visible
            for _ in range(8):
                try:
                    kind, generation, value = self.events.get_nowait()
                except queue.Empty:
                    break
                if generation != self.generation:
                    continue
                if kind in ('command', 'error'):
                    self.commanding = False
                    self.start_button.configure(state='normal')
                    self.stop_button.configure(state='normal')
                if kind == 'error':
                    self.status.set('视觉服务暂不可用')
                    self.detail.set(value)
                    self.last_image = self.last_result = None
                    self.image_label.configure(image='', text='视觉服务暂不可用')
                    self.tree.delete(*self.tree.get_children())
                elif visible:
                    self._render(value)
            if visible and not self.fetching and not self.commanding:
                self.fetching = True
                busy = bool(self.foreground_busy())
                threading.Thread(target=self._fetch, args=(self.generation, busy), daemon=True).start()
        except Exception as error:
            self.status.set('画面暂不可用')
            self.detail.set(str(error))
        finally:
            if not self.closing.is_set():
                self.timer = self.page.after(200, self._tick)

    def _render(self, value):
        from PIL import Image, ImageDraw, ImageFont, ImageTk
        result = value['result']
        state = result.get('state', 'stopped')
        self.status.set(STATE_LABELS.get(state, state))
        targets = result.get('targets', [])
        counts = result.get('counts', {})
        labels = {'recyclable': '可回收', 'kitchen': '厨余', 'hazardous': '有害', 'other': '其他',
                  'red': '红', 'yellow': '黄', 'blue': '蓝', 'green': '绿'}
        summary = f'当前目标：{len(targets)}'
        if counts:
            summary += '\n' + '  '.join(f'{labels.get(key, key)} {count}' for key, count in counts.items())
        if result.get('mode') == 'faces':
            summary += f'\n画面人脸数：{result.get("faces_detected", 0)} · 跟踪单人'
        self.summary.set(summary)
        self.tree.delete(*self.tree.get_children())
        for target in targets:
            score = target.get('confidence')
            self.tree.insert('', 'end', values=(
                target['label'], target.get('category_label',
                                           f'#{target["track_id"]}' if 'track_id' in target else '颜色'),
                f'{score:.2f}' if score is not None else '—'))
        detail = result.get('error') or REASON_LABELS.get(result.get('reason'), '')
        if not detail and targets:
            detail = '\n'.join(f'{t["label"]}：中心 ({t["center_uv"][0]:.0f}, {t["center_uv"][1]:.0f}) px'
                               for t in targets[:3])
        self.detail.set(detail or '物体分类仅覆盖既有模型的 16 类物品。人脸 ID 表示连续轨迹。')
        self.metrics.set(
            f'处理帧率 {result.get("processing_fps", 0):.1f} FPS · '
            f'处理耗时 {result.get("processing_ms", 0):.1f} ms · '
            f'画面年龄 {result.get("frame_age_s", 0):.2f} s')
        jpeg = value.get('jpeg')
        if not jpeg:
            self.last_image = self.last_result = None
            self.image_label.configure(image='', text=STATE_LABELS.get(state, '等待画面'))
            return
        # Result and JPEG arrive in the same response with the same frame stamp.
        original = Image.open(io.BytesIO(base64.b64decode(jpeg, validate=True))).convert('RGB')
        if self.font is None:
            for candidate in (
                    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
                    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
                try:
                    self.font = ImageFont.truetype(candidate, 15)
                    break
                except OSError:
                    continue
            if self.font is None:
                self.font = ImageFont.load_default()
        draw = ImageDraw.Draw(original)
        for target in targets:
            x, y = target['bbox_xyxy'][:2]
            label = target['label']
            if target.get('category_label'):
                label += ' · ' + target['category_label']
            if 'confidence' in target:
                label += f' {target["confidence"]:.2f}'
            left = max(0, min(round(x), original.width - 30))
            top = max(0, round(y) - 24)
            bounds = draw.textbbox((left + 3, top + 2), label, font=self.font)
            draw.rectangle((left, top, bounds[2] + 3, bounds[3] + 3), fill='#12332b')
            draw.text((left + 3, top + 2), label, font=self.font, fill='white')
        self.last_image, self.last_result = original, result
        self.last_frame_received = time.monotonic()
        display = original.copy()
        display.thumbnail((max(300, self.image_label.winfo_width()-12),
                           max(200, self.image_label.winfo_height()-12)))
        self.photo = ImageTk.PhotoImage(display)
        self.image_label.configure(image=self.photo, text='')

    def save(self):
        if self.last_image is None or time.monotonic()-self.last_frame_received > 1.5:
            self.detail.set('当前没有可保存的新鲜画面')
            return
        folder = self.workspace / 'evidence/vision/captures'
        folder.mkdir(parents=True, exist_ok=True)
        self.screenshot_index += 1
        stem = f'vision-{datetime.now():%Y%m%d_%H%M%S_%f}-{self.screenshot_index}'
        self.last_image.save(folder / f'{stem}.jpg', quality=90)
        (folder / f'{stem}.json').write_text(json.dumps(self.last_result, ensure_ascii=False,
                                                       indent=2) + '\n', encoding='utf-8')
        self.detail.set(f'截图已保存：{folder / (stem + ".jpg")}')

    def close(self):
        self.closing.set()
        self.page.after_cancel(self.timer)
        if self.recognizing:
            # An externally started service survives the window, but the
            # recognition requested by this page must stop when it closes.
            def stop_requested_mode():
                try:
                    self.client.request('stop')
                except (OSError, ValueError, RuntimeError):
                    pass
            threading.Thread(target=stop_requested_mode, daemon=True).start()
        # Terminate only the optional process launched by this page. External
        # edge services and all existing camera/model/robot processes survive.
        with self.process_lock:
            if self.process is not None and self.process.poll() is None:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass


def main():
    """Standalone perception UI without ASR, language model or robot runtime."""
    import argparse
    import tkinter as tk
    from tkinter import ttk
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[3])
    args = parser.parse_args()
    root = tk.Tk()
    root.title('端侧视觉感知')
    root.geometry('980x680')
    notebook = ttk.Notebook(root)
    notebook.pack(fill='both', expand=True, padx=12, pady=12)
    page = EdgeVisionPage(notebook, args.workspace)
    def close():
        page.close()
        root.destroy()
    root.protocol('WM_DELETE_WINDOW', close)
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
