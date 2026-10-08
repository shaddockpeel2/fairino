from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/fr3c_voice'))
from fr3c_camera.edge_runtime import EdgeWorker, PriorityMonitor, EdgeSocketServer, load_settings
from fr3c_voice.edge_vision_page import EdgeVisionClient


WORKSPACE = Path(__file__).resolve().parents[3]


def message():
    array = np.zeros((48, 64, 3), np.uint8)
    return SimpleNamespace(width=64, height=48, step=192, encoding='bgr8',
                           is_bigendian=False, data=array.tobytes(),
                           header=SimpleNamespace(frame_id='test',
                                                  stamp=SimpleNamespace(sec=1, nanosec=2)))


def settings():
    config = load_settings(WORKSPACE)
    config['max_fps'] = 30
    return config


class FakeEngine:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.release.set()
        self.calls = 0
    def close(self):
        pass
    def reset_tracking(self):
        pass
    def process(self, mode, image, now=None):
        self.calls += 1
        self.entered.set()
        self.release.wait(2)
        return image, {'mode': mode, 'state': 'running', 'targets': [{'label': 'old'}]}


def wait_until(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('condition was not reached')


def monitor(config):
    priority = PriorityMonitor(config['priority'])
    priority.set_model('')
    return priority


@pytest.mark.parametrize('mode', ['objects', 'colors', 'faces'])
def test_original_robot_activity_suspends_new_work_without_any_control_calls(mode):
    priority = monitor(settings())
    priority.observe('grasp', {'action_active': True}, now=100)
    priority.set_model('', now=100)
    assert priority.reason(mode, now=100.1) == 'robot_busy'
    priority.observe('grasp', {'action_active': False, 'retryable': True}, now=100.2)
    assert priority.reason(mode, now=100.2) == ''


def test_original_npu_detector_has_priority_and_idle_servo_event_does_not_expire():
    priority = monitor(settings())
    priority.observe('servo', {'state': 'monitoring_manual_ready'}, now=1)
    priority.observe('perception', {'inference_enabled': True, 'npu_inference': True}, now=100)
    priority.set_model('', now=100)
    assert priority.reason('objects', now=100) == 'foreground_npu_perception'
    assert priority.reason('colors', now=100) == ''
    priority.observe('servo', {'state': 'executing_servo_tracking'}, now=100)
    assert priority.reason('faces', now=100) == 'servo_busy'


def test_missing_foreground_services_allow_standalone_but_stale_seen_status_blocks():
    priority = monitor(settings())
    priority.set_model('', now=100)
    assert priority.reason('objects', now=100) == ''
    priority.observe('grasp', {'action_active': False}, now=90)
    assert priority.reason('colors', now=100) == 'grasp_status_stale'


def test_existing_unreachable_model_socket_does_not_allow_new_inference(tmp_path):
    config = settings()['priority']
    config['model_socket'] = str(tmp_path / 'model.sock')
    priority = PriorityMonitor(config)
    priority.poll_model()
    assert priority.reason('objects') == ''
    Path(config['model_socket']).touch()
    priority.poll_model()
    assert priority.reason('objects') == 'model_status_unavailable'


def test_stop_discards_result_of_in_flight_frame():
    config = settings()
    engine = FakeEngine()
    engine.release.clear()
    worker = EdgeWorker(config, monitor(config), engine=engine)
    worker.start()
    try:
        worker.select('colors')
        worker.submit(message())
        assert engine.entered.wait(1)
        stopped = worker.select(None)
        assert stopped['result']['targets'] == []
        engine.release.set()
        time.sleep(0.1)
        assert worker.snapshot()['result']['targets'] == []
        assert worker.snapshot()['result']['mode'] is None
    finally:
        engine.release.set()
        worker.close()


def test_stale_camera_clears_cached_results_and_jpeg():
    config = settings()
    worker = EdgeWorker(config, monitor(config), engine=FakeEngine())
    worker.start()
    try:
        worker.select('colors')
        worker.submit(message())
        wait_until(lambda: bool(worker.snapshot()['result']['targets']))
        assert worker.snapshot()['jpeg']
        worker.submit(message(), received=time.monotonic() - 2)
        snapshot = worker.snapshot()
        assert snapshot['result']['state'] == 'camera_stale'
        assert snapshot['result']['targets'] == []
        assert snapshot['jpeg'] is None
    finally:
        worker.close()


def test_a_new_priority_owner_drops_completed_side_result():
    config = settings()
    priority = monitor(config)
    engine = FakeEngine()
    engine.release.clear()
    worker = EdgeWorker(config, priority, engine=engine)
    worker.start()
    try:
        worker.select('colors')
        worker.submit(message())
        assert engine.entered.wait(1)
        priority.set_model('model_busy')
        engine.release.set()
        wait_until(lambda: worker.result['state'] == 'paused')
        assert worker.snapshot()['result']['targets'] == []
    finally:
        engine.release.set()
        worker.close()


def test_socket_client_controls_only_edge_worker_and_rejects_unknown_mode(tmp_path):
    config = settings()
    worker = EdgeWorker(config, monitor(config), engine=FakeEngine())
    path = tmp_path / 'edge.sock'
    server = EdgeSocketServer(path, worker)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = EdgeVisionClient(WORKSPACE)
    client.path = path
    try:
        assert client.request('start', mode='faces')['result']['mode'] == 'faces'
        assert client.request('stop')['result']['mode'] is None
        with pytest.raises(RuntimeError):
            client.request('start', mode='invalid')
    finally:
        server.shutdown()
        server.server_close()


def test_stopping_inactive_feature_does_not_stop_active_feature():
    config = settings()
    worker = EdgeWorker(config, monitor(config), engine=FakeEngine())
    worker.select('faces')
    assert worker.stop_mode('objects') is False
    assert worker.mode == 'faces'


def test_fresh_camera_arrival_does_not_make_old_processed_image_fresh():
    config = settings()
    worker = EdgeWorker(config, monitor(config), engine=FakeEngine())
    worker.select('colors')
    worker.submit(message())
    worker.jpeg = 'cached_old_jpeg'
    worker.result = {'mode': 'colors', 'state': 'running', 'targets': [{'label': 'old'}]}
    worker.result_received = time.monotonic() - 2
    snapshot = worker.snapshot()
    assert snapshot['result']['state'] == 'waiting_result'
    assert snapshot['result']['targets'] == []
    assert snapshot['jpeg'] is None


def test_foreground_priority_clears_cached_target_without_waiting_for_next_camera_frame():
    config = settings()
    priority = monitor(config)
    worker = EdgeWorker(config, priority, engine=FakeEngine())
    worker.select('colors')
    worker.submit(message())
    worker.result_received = time.monotonic()
    worker.jpeg = 'old_target_overlay'
    worker.result = {'mode': 'colors', 'state': 'running', 'targets': [{'label': 'old'}]}
    priority.set_model('model_busy')
    snapshot = worker.snapshot()
    assert snapshot['result']['state'] == 'paused'
    assert snapshot['result']['targets'] == []
    assert snapshot['jpeg'] is None


def test_stop_is_not_blocked_by_a_slow_output_consumer():
    config = settings()
    publishing = threading.Event()
    release = threading.Event()
    def slow_output(*_args):
        publishing.set()
        release.wait(2)
    worker = EdgeWorker(config, monitor(config), engine=FakeEngine(), on_result=slow_output)
    worker.start()
    try:
        worker.select('colors')
        worker.submit(message())
        assert publishing.wait(1)
        started = time.monotonic()
        result = worker.select(None)
        assert time.monotonic() - started < 0.2
        assert result['result']['targets'] == []
    finally:
        release.set()
        worker.close()
