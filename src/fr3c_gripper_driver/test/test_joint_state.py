import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from fr3c_gripper_driver.node import GripperNode


@pytest.mark.parametrize('backend', ['mock', 'ft232h'])
def test_startup_position_is_published_for_moveit(backend):
    rclpy.init()
    gripper = GripperNode(parameter_overrides=[
        Parameter('backend', value=backend),
        Parameter('hardware_output_enabled', value=False),
        Parameter('startup_position_m', value=0.130),
    ])
    observer = Node(f'joint_state_observer_{backend}')
    received = []
    observer.create_subscription(
        JointState, '/joint_states', received.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(gripper)
    executor.add_node(observer)

    try:
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert received
        assert received[-1].name == ['gripper_command_joint']
        assert received[-1].position == pytest.approx([0.130])
        expected_source = (
            'mock_startup' if backend == 'mock'
            else 'startup_parameter_unverified')
        assert gripper._position_source == expected_source
        if backend == 'ft232h':
            assert gripper._backend.current_us is None
    finally:
        executor.remove_node(observer)
        executor.remove_node(gripper)
        observer.destroy_node()
        gripper.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
