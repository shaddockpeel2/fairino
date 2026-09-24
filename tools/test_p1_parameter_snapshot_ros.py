"""Isolated read-only parameter-service integration test; requires ROS_DOMAIN_ID=97."""

import hashlib
import os
import threading

import pytest

from p1_parameter_snapshot import collect, forward_profile_errors
from test_p1_parameter_snapshot import valid_snapshot


def test_read_only_snapshot_against_mock_parameter_servers():
    if os.environ.get('ROS_DOMAIN_ID') != '97':
        pytest.skip('Run explicitly in isolated ROS_DOMAIN_ID=97')
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    client = None
    thread = None
    try:
        snapshot = valid_snapshot()
        for name, entry in snapshot['nodes'].items():
            values = entry['parameters'].copy()
            for field in ('robot_description', 'robot_description_semantic'):
                if field in values:
                    values[field] = '<robot name="mock"/>'
            node = rclpy.create_node(
                name.lstrip('/'),
                parameter_overrides=[Parameter(key, value=value)
                                     for key, value in values.items()],
                automatically_declare_parameters_from_overrides=True)
            nodes.append(node)
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        client = rclpy.create_node('p1_snapshot_integration_test')
        result = collect(client)
        assert forward_profile_errors(result) == []
        assert result['nodes']['/servo_node']['parameters']['robot_description'][
            'sha256'] == hashlib.sha256(b'<robot name="mock"/>').hexdigest()
        assert result['nodes']['/servo_node']['parameters'][
            'moveit_servo.scale.linear'] is None
        # Removing a server must be explicit evidence failure, not defaults.
        executor.remove_node(nodes[-1])
        nodes[-1].destroy_node()
        nodes.pop()
        result = collect(client, timeout_s=0.2)
        assert forward_profile_errors(result)
    finally:
        executor.shutdown()
        if thread:
            thread.join(timeout=3)
        if client:
            client.destroy_node()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
