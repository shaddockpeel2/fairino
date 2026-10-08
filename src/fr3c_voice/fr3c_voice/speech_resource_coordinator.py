"""UI audio ownership; recognizers, speech engines and robot routing stay separate.

All transitions run on the Tk thread. Workers may read ``reserved``/``ready``
but never invoke the UI callbacks. Ownership survives leaving the tab until
its workers have finished releasing the microphone and speech session.
"""
from __future__ import annotations


class SpeechResourceCoordinator:
    def __init__(self, foreground_busy, pause_wake, wake_active, resume_wake):
        self.foreground_busy = foreground_busy
        self.pause_wake = pause_wake
        self.wake_active = wake_active
        self.resume_wake = resume_wake
        self.visible = False
        self.reserved = False
        self.ready = False
        self.reason = ''

    def update(self, visible: bool, operation_active: bool = False) -> None:
        self.visible = bool(visible)
        self.ready = False
        if not self.visible:
            if self.reserved and not operation_active:
                try:
                    if self.wake_active():
                        self.reason = '正在等待原唤醒监听释放麦克风'
                        return
                except Exception:
                    self.reason = '暂时无法确认麦克风是否已释放'
                    return
                self.reserved = False
                self.resume_wake()  # scheduler still honours the user's checkbox
            self.reason = '正在释放音频资源' if self.reserved else ''
            return
        try:
            if self.foreground_busy():
                self.reason = '原对话、语音或机械臂任务运行中，请等待结束'
                return
            if not self.reserved:
                self.reserved = True  # block the wake scheduler before cancellation
                try:
                    self.pause_wake()
                except Exception:
                    self.reserved = False
                    raise
            if self.wake_active():
                self.reason = '正在等待原唤醒监听释放麦克风'
                return
        except Exception:
            self.reason = '暂时无法确认原任务或音频状态'
            return
        self.ready = True
        self.reason = '本页使用期间暂停唤醒 · 离开后按原设置恢复'
