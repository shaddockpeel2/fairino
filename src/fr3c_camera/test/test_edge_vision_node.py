import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import Image
from std_msgs.msg import String
from fr3c_camera.edge_runtime import load_settings
from fr3c_camera.edge_vision_node import EdgeVisionNode


WORKSPACE = Path(__file__).resolve().parents[3]


@pytest.fixture
def node():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    settings = load_settings(WORKSPACE)
    settings['priority']['status_topics'] = {'servo': '/edge_vision/test/control_state'}
    rclpy.init()
    node = EdgeVisionNode(settings)
    yield node
    node.close()
    node.destroy_node()
    rclpy.shutdown()


def test_node_constructs_six_independent_services_without_loading_models(node):
    assert len(node._edge_services) == 6
    assert node.worker.mode is None
    assert node.worker.engine._objects is None
    assert node.worker.engine._faces is None
    assert len(list(node.clients)) == 0


def test_original_servo_string_state_is_read_without_a_command(node):
    node.priority.set_model('')
    node._observer('servo')(String(data='monitoring_manual_ready'))
    assert node.priority.reason('faces') == ''
    node._observer('servo')(String(data='executing_servo_tracking'))
    assert node.priority.reason('faces') == 'servo_busy'


def test_stale_ros_timestamp_is_rejected_even_if_just_received(node):
    message = Image()
    message.width, message.height, message.step = 2, 1, 6
    message.encoding = 'rgb8'
    message.data = bytes([255, 0, 0, 0, 255, 0])
    message.header.stamp.sec = 1
    node._image(message)
    assert node.worker.frame is None
    message.header.stamp = node.get_clock().now().to_msg()
    node._image(message)
    assert node.worker.frame is not None


def test_pending_ros_result_is_discarded_after_mode_switch(node):
    node.worker.select('colors')
    epoch = node.worker.epoch
    node._publish_frame(None, {'mode': 'colors', 'epoch': epoch, 'targets': []}, None)
    node.worker.select('faces')
    calls = []
    node.result_publishers['colors'] = type('Publisher', (), {
        'publish': lambda self, message: calls.append(message)})()
    node._publish_pending()
    assert calls == []
