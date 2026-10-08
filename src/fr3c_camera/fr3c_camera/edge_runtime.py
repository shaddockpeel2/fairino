"""Latest-frame worker and bounded local API for independent edge perception."""

from __future__ import annotations

import base64
import json
from pathlib import Path
import socket
import socketserver
import threading
import time

import cv2
import yaml

from .edge_perception import EdgePerception
from .rgbd import image_array


MODES = ('objects', 'colors', 'faces')
MAX_REQUEST_BYTES = 4096


def socket_path(workspace, domain_id=0):
    return Path(workspace) / 'evidence/vision/runtime' / f'edge-vision-{int(domain_id)}.sock'


def load_settings(workspace, path=None):
    root = Path(workspace).resolve()
    config = yaml.safe_load((Path(path) if path else root / 'config/vision/edge_vision.yaml')
                            .read_text(encoding='utf-8'))
    if config.get('schema_version') != 1:
        raise ValueError('unsupported edge vision config schema')
    if not 0 < float(config['max_fps']) <= 30 or not 0 < config['camera_max_age_s'] <= 5:
        raise ValueError('invalid frame rate or maximum frame age')
    for mode in ('objects', 'faces'):
        config[mode]['model_path'] = str(root / config[mode]['model_path'])
    config['priority']['model_socket'] = str(root / config['priority']['model_socket'])
    original = yaml.safe_load((root / config['color_config']).read_text(encoding='utf-8'))
    params = original['/perception/color_target_2d']['ros__parameters']
    config['colors'] = {
        'intervals': {color: params[f'{color}_hue_intervals']
                      for color in ('red', 'yellow', 'blue', 'green')},
        'segmentation': {key: params[key] for key in
                         ('saturation_min', 'value_min', 'morphology_size')},
        'candidate_filter': {key: params[key] for key in
                             ('min_area_px', 'max_area_px', 'min_aspect_ratio',
                              'max_aspect_ratio', 'min_solidity', 'min_rectangularity',
                              'interior_fraction')},
    }
    return config


class PriorityMonitor:
    """Read-only admission policy. Never pause or command an existing service.

    Foreground work keeps its original scheduling. New work yields on the next
    frame; an already running frame cannot be preempted. This is not an atomic
    device-wide NPU lock, so the side service also uses a low frame-rate cap.
    """

    def __init__(self, config):
        self.config = config
        self.lock = threading.Lock()
        self.states = {}
        self.model = ('checking_model', 0.0)

    def observe(self, key, value, now=None):
        with self.lock:
            self.states[key] = (value, time.monotonic() if now is None else now)

    def set_model(self, reason, now=None):
        with self.lock:
            self.model = (reason, time.monotonic() if now is None else now)

    def reason(self, mode, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            states, model = dict(self.states), self.model
        for key, (value, observed) in states.items():
            # Servo control_state is an event stream, not a heartbeat.
            if key != 'servo' and now - observed > self.config['status_max_age_s']:
                return f'{key}_status_stale'
            if key == 'grasp' and (value.get('action_active') or value.get('auto_stack_active')
                                   or value.get('retryable') is False
                                   or value.get('state') in (
                                       'planning', 'executing', 'approaching', 'descending',
                                       'closing_gripper', 'lifting', 'sorting', 'returning',
                                       'batch_scanning', 'batch_planning', 'batch_executing')):
                return 'robot_busy'
            if key == 'servo' and value.get('state') not in (
                    'idle', 'monitoring_manual_ready', 'unknown', None):
                return 'servo_busy'
            if key == 'perception':
                if value.get('inference_enabled') is False:
                    return 'foreground_perception_paused'
                if mode == 'objects' and value.get('npu_inference'):
                    return 'foreground_npu_perception'
        if now - model[1] > self.config['status_max_age_s']:
            return 'model_status_stale'
        return model[0]

    def poll_model(self):
        path = Path(self.config['model_socket'])
        if not path.exists():
            self.set_model('')
            return
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(0.25)
                connection.connect(str(path))
                connection.sendall(b'{"op":"health"}\n')
                with connection.makefile('rb') as stream:
                    raw = stream.readline(65537)
            if len(raw) > 65536:
                raise ValueError('oversized model health')
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ValueError('model health must be an object')
            if value.get('event') != 'health':
                raise ValueError('invalid model health')
            if value.get('npu_owner') or value.get('active_request_id'):
                reason = 'model_busy'
            elif value.get('state') != 'ready' or not value.get('healthy'):
                reason = 'model_not_ready'
            else:
                reason = ''
            self.set_model(reason)
        except (OSError, ValueError):
            self.set_model('model_status_unavailable')


class EdgeWorker:
    """Only the worker thread owns models; commands invalidate in-flight output."""

    def __init__(self, config, priority, engine=None, on_result=None):
        self.config = config
        self.priority = priority
        self.engine = engine or EdgePerception(config)
        self.on_result = on_result or (lambda _image, _result, _message: None)
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.wake = threading.Event()
        self.mode = None
        self.epoch = 0
        self.frame = None
        self.sequence = 0
        self.preview_until = 0.0
        self.ui_busy_until = 0.0
        self.result = {'state': 'stopped', 'targets': [], 'counts': {}, 'mode': None}
        self.jpeg = None
        self.result_received = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True, name='edge-perception')

    def start(self):
        self.thread.start()

    def submit(self, message, received=None):
        with self.lock:
            self.sequence += 1
            self.frame = (message, time.monotonic() if received is None else received,
                          self.sequence)
        self.wake.set()

    def select(self, mode):
        if mode is not None and mode not in MODES:
            raise ValueError(f'unknown perception mode: {mode}')
        with self.lock:
            self.mode = mode
            self.epoch += 1
            self.result = {'state': 'starting' if mode else 'stopped',
                           'mode': mode, 'targets': [], 'counts': {}, 'epoch': self.epoch}
            self.jpeg = None
        self.wake.set()
        return self.snapshot()

    def stop_mode(self, mode):
        with self.lock:
            if self.mode != mode:
                return False
            self.mode = None
            self.epoch += 1
            self.result = {'state': 'stopped', 'mode': None, 'targets': [],
                           'counts': {}, 'epoch': self.epoch}
            self.jpeg = None
        self.wake.set()
        return True

    def snapshot(self, preview=False, foreground_busy=False):
        now = time.monotonic()
        with self.lock:
            if preview:
                self.preview_until = now + 1.5
            if foreground_busy:
                self.ui_busy_until = now + 1.0
            result = dict(self.result)
            frame = self.frame
            if frame is None or now - frame[1] > self.config['camera_max_age_s']:
                result.update(state='camera_stale' if frame else 'waiting_camera',
                              targets=[], counts={}, faces_detected=0, trail=[],
                              reason='no_fresh_frame')
                jpeg = None
            else:
                jpeg = self.jpeg
                result['frame_age_s'] = now - (self.result_received or frame[1])
                if jpeg and now - self.result_received > self.config['camera_max_age_s']:
                    result.update(state='waiting_result', targets=[], counts={},
                                  faces_detected=0, trail=[], reason='waiting_fresh_result')
                    jpeg = None
                reason = ('foreground_ui_busy' if self.ui_busy_until > now
                          else self.priority.reason(self.mode, now)) if self.mode else ''
                if reason and result.get('state') != 'error':
                    if result.get('state') != 'paused':
                        jpeg = None
                    result.update(state='paused', targets=[], counts={},
                                  faces_detected=0, trail=[], reason=reason)
            return {'schema_version': 1, 'result': result, 'jpeg': jpeg}

    def _commit(self, epoch, message, image, result, jpeg, received):
        with self.lock:
            if epoch != self.epoch or self.done.is_set():
                return False
            self.result, self.jpeg = result, jpeg
            self.result_received = received
        # Never hold the worker lock across a DDS write: a slow subscriber
        # must not block stop commands, camera callbacks or Tk snapshots.
        self.on_result(image, result, message)
        return True

    def _run(self):
        previous_epoch = -1
        last_seq = -1
        failed_epoch = None
        last_processed = None
        last_processed_at = None
        while not self.done.is_set():
            self.wake.wait(1 / float(self.config['max_fps']))
            self.wake.clear()
            cycle = time.monotonic()
            with self.lock:
                mode, epoch, frame = self.mode, self.epoch, self.frame
                preview = self.preview_until > cycle
                ui_busy = self.ui_busy_until > cycle
            if epoch != previous_epoch:
                self.engine.close()
                self.engine.reset_tracking()
                previous_epoch, last_seq, last_processed = epoch, -1, None
                last_processed_at = None
            if frame is None or (mode is None and not preview):
                continue
            message, received, sequence = frame
            if cycle - received > self.config['camera_max_age_s'] or sequence == last_seq:
                continue
            last_seq = sequence
            reason = (('foreground_ui_busy' if ui_busy else self.priority.reason(mode, cycle))
                      if mode else '')
            try:
                image = image_array(message)
                if message.encoding == 'rgb8':
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                elif message.encoding == 'bgr8':
                    image = image.copy()
                else:
                    raise ValueError('edge perception requires rgb8 or bgr8')
                result = {'mode': mode, 'targets': [], 'counts': {},
                          'state': 'paused' if mode and reason else 'stopped', 'reason': reason}
                if mode and not reason and failed_epoch != epoch:
                    if last_processed is not None and cycle-last_processed > self.config['camera_max_age_s']:
                        self.engine.reset_tracking()
                    image, result = self.engine.process(mode, image, now=cycle)
                    # Drop results if foreground work became busy during processing.
                    after = self.priority.reason(mode)
                    with self.lock:
                        if self.ui_busy_until > time.monotonic():
                            after = 'foreground_ui_busy'
                    if after:
                        result = {'mode': mode, 'state': 'paused', 'reason': after,
                                  'targets': [], 'counts': {}}
                        image = image_array(message).copy()
                        if message.encoding == 'rgb8':
                            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                        self.engine.reset_tracking()
                    last_processed = cycle
                elif mode and failed_epoch == epoch:
                    with self.lock:
                        result = {**self.result, 'targets': [], 'counts': {}}
                elif reason:
                    self.engine.reset_tracking()
                stamp = message.header.stamp
                result.update(epoch=epoch, sequence=sequence,
                              ros_stamp_ns=int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
                              frame_id=message.header.frame_id,
                              width=int(message.width), height=int(message.height),
                              frame_age_s=time.monotonic()-received,
                              processing_fps=(1/(cycle-last_processed_at)
                                              if last_processed_at is not None and cycle > last_processed_at
                                              else 0.0))
                last_processed_at = cycle
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY,
                                                          self.config['jpeg_quality']])
                if not ok:
                    raise RuntimeError('JPEG encoding failed')
                jpeg = base64.b64encode(encoded).decode('ascii')
                if time.monotonic() - received <= self.config['camera_max_age_s']:
                    self._commit(epoch, message, image, result, jpeg, received)
            except Exception as error:
                failed_epoch = epoch
                self._commit(epoch, message, None, {
                    'mode': mode, 'epoch': epoch, 'state': 'error', 'targets': [],
                    'counts': {}, 'error': f'{type(error).__name__}: {error}'}, None, received)
            # Enforce the cap even when callbacks keep waking the latest-frame slot.
            self.done.wait(max(0, 1/float(self.config['max_fps']) - (time.monotonic()-cycle)))
        self.engine.close()

    def close(self):
        self.done.set()
        self.wake.set()
        self.thread.join(timeout=5.0)


class EdgeRequestHandler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(1.0)
        try:
            raw = self.rfile.readline(MAX_REQUEST_BYTES + 1)
            if len(raw) > MAX_REQUEST_BYTES:
                raise ValueError('request too large')
            request = json.loads(raw)
            worker = self.server.worker
            operation = request.get('op')
            if operation == 'start':
                mode = request.get('mode')
                if mode not in MODES:
                    raise ValueError('start requires objects, colors or faces')
                value = worker.select(mode)
            elif operation == 'stop':
                value = worker.select(None)
            elif operation in ('snapshot', 'status'):
                value = worker.snapshot(preview=operation == 'snapshot',
                                        foreground_busy=bool(request.get('foreground_busy')))
                if operation == 'status':
                    value.pop('jpeg', None)
            else:
                raise ValueError('unsupported edge vision operation')
            value['ok'] = True
        except (OSError, ValueError, TypeError, AttributeError) as error:
            value = {'ok': False, 'error': str(error)}
        try:
            self.wfile.write(json.dumps(value, ensure_ascii=False).encode('utf-8') + b'\n')
            self.wfile.flush()
        except OSError:
            pass


class EdgeSocketServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    request_queue_size = 8

    def __init__(self, path, worker):
        self.worker = worker
        super().__init__(str(path), EdgeRequestHandler)
