"""Optional edge perception service; subscribes to an already running camera."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .edge_runtime import MODES, EdgeSocketServer, EdgeWorker, PriorityMonitor, load_settings, socket_path


class EdgeVisionNode(Node):
    def __init__(self, settings):
        super().__init__('fr3c_edge_vision')
        self.settings = settings
        self._output_lock = threading.Lock()
        self._pending_output = None
        self.priority = PriorityMonitor(settings['priority'])
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.image_publishers = {mode: self.create_publisher(
            Image, f'/edge_vision/{mode}/debug_image', qos) for mode in MODES}
        self.result_publishers = {mode: self.create_publisher(
            String, f'/edge_vision/{mode}/results', 1) for mode in MODES}
        self.status_publisher = self.create_publisher(String, '/edge_vision/status', 1)
        self.worker = EdgeWorker(settings, self.priority, on_result=self._publish_frame)
        self.create_subscription(Image, settings['image_topic'], self._image, qos)
        # Reading these topics adds no robot/client/service ownership.
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        for key, topic in settings['priority']['status_topics'].items():
            subscription_qos = (QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL) if key == 'servo' else status_qos)
            self.create_subscription(String, topic, self._observer(key), subscription_qos)
        self._edge_services = []
        for mode in MODES:
            for operation in ('start', 'stop'):
                self._edge_services.append(self.create_service(
                    Trigger, f'/edge_vision/{mode}/{operation}',
                    self._control(mode, operation)))
        self.create_timer(0.5, self._publish_status)
        self.create_timer(0.05, self._publish_pending)
        self.model_thread = threading.Thread(target=self._poll_model, daemon=True)
        self.worker.start()
        self.model_thread.start()

    def _image(self, message):
        # Monotonic reception age alone must not validate queued historical data.
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1_000_000_000
        if stamp <= 0 or age > self.settings['camera_max_age_s'] or age < -0.1:
            return
        self.worker.submit(message)

    def _observer(self, key):
        def observe(message):
            try:
                value = ({'state': message.data} if key == 'servo'
                         else json.loads(message.data))
                if isinstance(value, dict):
                    self.priority.observe(key, value)
            except (ValueError, TypeError):
                # A malformed foreground status is not permission to run.
                self.priority.observe(key, {'state': 'invalid', 'action_active': True,
                                            'inference_enabled': False})
        return observe

    def _control(self, mode, operation):
        def callback(_request, response):
            if operation == 'start':
                self.worker.select(mode)
                response.message = f'{mode} accepted; inspect /edge_vision/status for readiness'
            else:
                changed = self.worker.stop_mode(mode)
                response.message = 'stopped' if changed else f'{mode} is already stopped'
            response.success = True
            return response
        return callback

    def _poll_model(self):
        while not self.worker.done.is_set():
            with self.worker.lock:
                active = self.worker.mode is not None
            if active:
                self.priority.poll_model()
            self.worker.done.wait(0.5)

    def _publish_frame(self, image, result, original):
        # The model thread only replaces a slot. DDS publication stays on
        # the ROS executor and never holds the model/control state lock.
        with self._output_lock:
            self._pending_output = (image, result, original)

    def _publish_pending(self):
        with self._output_lock:
            pending = self._pending_output
            self._pending_output = None
        if pending is None:
            return
        image, result, original = pending
        mode = result.get('mode')
        with self.worker.lock:
            current = result.get('epoch') == self.worker.epoch and mode == self.worker.mode
        if mode not in MODES or not current:
            return
        latest = self.worker.snapshot()['result']
        if latest.get('state') in ('paused', 'camera_stale', 'waiting_result', 'waiting_camera'):
            return
        self.result_publishers[mode].publish(String(data=json.dumps(result, ensure_ascii=False)))
        if image is not None:
            message = Image()
            message.header = original.header
            message.height, message.width = image.shape[:2]
            message.encoding = 'bgr8'
            message.step = message.width * 3
            message.data = image.tobytes()
            self.image_publishers[mode].publish(message)

    def _publish_status(self):
        result = self.worker.snapshot()['result']
        self.status_publisher.publish(String(data=json.dumps(result, ensure_ascii=False)))
        # Clear stopped, lost-stream and paused results for independent consumers.
        if result.get('state') not in ('running', 'tracking', 'searching', 'temporarily_lost'):
            for mode, publisher in self.result_publishers.items():
                invalid = {**result, 'mode': mode, 'targets': [], 'counts': {}}
                publisher.publish(String(data=json.dumps(invalid, ensure_ascii=False)))

    def close(self):
        self.worker.close()
        self.model_thread.join(timeout=1.0)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--config', type=Path)
    parser.add_argument('--image-topic')
    parser.add_argument('--validate-only', action='store_true')
    options, ros_args = parser.parse_known_args(args)
    settings = load_settings(options.workspace, options.config)
    if options.image_topic:
        settings['image_topic'] = options.image_topic
    if options.validate_only:
        for mode in ('objects', 'faces'):
            if not Path(settings[mode]['model_path']).is_file():
                parser.error(f'{mode} model is missing')
        print(json.dumps({'ok': True, 'image_topic': settings['image_topic'],
                          'opencv': cv2.__version__, 'yunet_api': hasattr(cv2, 'FaceDetectorYN'),
                          'socket': str(socket_path(options.workspace, os.getenv('ROS_DOMAIN_ID', '0')))},
                         ensure_ascii=False))
        return 0
    # These limits apply only to this optional child process.
    cv2.setNumThreads(1)
    path = socket_path(options.workspace, os.getenv('ROS_DOMAIN_ID', '0'))
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    lock = path.with_suffix('.lock').open('a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Edge vision is already running for this workspace and ROS domain.')
        lock.close()
        return 0
    node = server = None
    stop = threading.Event()
    try:
        lock.seek(0)
        lock.truncate()
        lock.write(str(os.getpid()))
        lock.flush()
        rclpy.init(args=ros_args)
        node = EdgeVisionNode(settings)
        path.unlink(missing_ok=True)
        server = EdgeSocketServer(path, node.worker)
        os.chmod(path, 0o600)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        signal.signal(signal.SIGINT, lambda *_: stop.set())
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        node.get_logger().info(f'Edge vision stopped by default; RGB source={settings["image_topic"]}')
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
            path.unlink(missing_ok=True)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
