import threading
import time

import pytest

from fr3c_camera.inference_gate import InferenceGate


def test_pause_drains_active_work_and_blocks_new_entries():
    gate = InferenceGate()
    assert gate.enter()
    finished = threading.Event()

    def pause():
        gate.set_enabled(False, 1.0)
        finished.set()

    thread = threading.Thread(target=pause)
    thread.start()
    time.sleep(0.02)
    assert not finished.is_set()
    gate.leave(success=True)
    thread.join(1.0)
    assert finished.is_set()
    assert not gate.enter()
    snapshot = gate.snapshot()
    assert snapshot.pause_generation == 1
    assert snapshot.frames_skipped_paused == 1


def test_pause_timeout_keeps_gate_disabled():
    gate = InferenceGate()
    assert gate.enter()
    with pytest.raises(TimeoutError):
        gate.set_enabled(False, 0.01)
    assert gate.snapshot().enabled is False
    gate.leave(success=False)


def test_resume_is_idempotent_and_records_success():
    gate = InferenceGate()
    gate.set_enabled(False, 0.1)
    gate.set_enabled(False, 0.1)
    assert gate.snapshot().pause_generation == 1
    gate.set_enabled(True, 0.1)
    assert gate.enter()
    gate.leave(success=True)
    snapshot = gate.snapshot()
    assert snapshot.enabled is True
    assert snapshot.last_success_ns is not None
