import threading
import time

import pytest

from fr3c_voice.coordinator import (
    CoordinationBusy, CoordinationFault, NpuCoordinator, NpuOwner,
    PerceptionSnapshot, RobotSnapshot,
)


class FakeBridge:
    def __init__(self, mode='colors'):
        self.mode = mode
        self.enabled = True
        self.frames = 10
        self.calls = []
        self.robot = RobotSnapshot()
        self.pause_error = None
        self.resume_error = None

    def perception_snapshot(self):
        return PerceptionSnapshot(
            available=True, enabled=self.enabled, frames_inferred=self.frames,
            last_success_ns=1, observed_monotonic=time.monotonic(),
            detection_mode=self.mode, npu_inference=self.mode == 'yolo')

    def robot_snapshot(self):
        return self.robot

    def pause_and_drain(self, timeout_s):
        self.calls.append(('pause', timeout_s))
        if self.pause_error:
            raise self.pause_error
        self.enabled = False
        return self.perception_snapshot()

    def resume_and_verify(self, frames_before, timeout_s):
        self.calls.append(('resume', frames_before, timeout_s))
        if self.resume_error:
            raise self.resume_error
        self.enabled = True
        self.frames = frames_before + 1
        return self.perception_snapshot()


def coordinator(bridge):
    value = NpuCoordinator(bridge, 1, 1, 1, 1)
    value.available_memory_mib = lambda: 4096
    return value


def test_colors_path_is_not_misclassified_as_npu_work():
    bridge = FakeBridge('colors')
    value = coordinator(bridge)
    assert value.run(NpuOwner.QWEN_TEXT, 'text-1', lambda: 'ok') == 'ok'
    assert bridge.calls == []


def test_yolo_path_pauses_and_requires_a_fresh_frame_after_resume():
    bridge = FakeBridge('yolo')
    value = coordinator(bridge)
    assert value.run(NpuOwner.QWEN_VISION, 'vision-1', lambda: 'ok') == 'ok'
    assert [item[0] for item in bridge.calls] == ['pause', 'resume']
    assert bridge.frames == 11


def test_operation_error_still_restores_yolo():
    bridge = FakeBridge('yolo')
    value = coordinator(bridge)
    with pytest.raises(ValueError, match='model failed'):
        value.run(
            NpuOwner.QWEN_TEXT, 'text-2',
            lambda: (_ for _ in ()).throw(ValueError('model failed')))
    assert [item[0] for item in bridge.calls] == ['pause', 'resume']


def test_robot_action_and_concurrent_requests_are_rejected():
    bridge = FakeBridge('colors')
    bridge.robot = RobotSnapshot(
        available=True, active=True, state='executing',
        observed_monotonic=time.monotonic())
    value = coordinator(bridge)
    with pytest.raises(CoordinationBusy, match='robot action'):
        value.run(NpuOwner.QWEN_TEXT, 'text-3', lambda: None)

    bridge.robot = RobotSnapshot()
    entered = threading.Event()
    release = threading.Event()
    thread = threading.Thread(target=lambda: value.run(
        NpuOwner.QWEN_TEXT, 'text-4',
        lambda: (entered.set(), release.wait(1))))
    thread.start()
    assert entered.wait(0.5)
    with pytest.raises(CoordinationBusy, match='NPU is busy'):
        value.run(NpuOwner.QWEN_TEXT, 'text-5', lambda: None)
    release.set()
    thread.join(1)


def test_resume_failure_latches_fault():
    bridge = FakeBridge('yolo')
    bridge.resume_error = TimeoutError('no fresh frame')
    value = coordinator(bridge)
    with pytest.raises(CoordinationFault, match='resume_failed'):
        value.run(NpuOwner.QWEN_TEXT, 'text-6', lambda: None)
    with pytest.raises(CoordinationFault):
        value.run(NpuOwner.QWEN_TEXT, 'text-7', lambda: None)
