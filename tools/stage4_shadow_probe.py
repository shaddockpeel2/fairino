#!/usr/bin/env python3
"""Probe isolated Servo output without any robot controller or lease."""

import json
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


class ShadowProbe(Node):
    def __init__(self):
        super().__init__('fr3c_stage4_shadow_probe')
        self.publisher = self.create_publisher(
            TwistStamped, '/fr3c_stage4/shadow_twist', 10)
        self.start = self.create_client(
            Trigger, '/stage4_shadow_servo/start_servo')
        self.stop = self.create_client(
            Trigger, '/stage4_shadow_servo/stop_servo')
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(
            JointTrajectory, '/fr3c_stage4/shadow_joint_trajectory',
            self._on_trajectory, 10)
        self.create_subscription(
            Int8, '/fr3c_stage4/shadow_status', self._on_status, 10)
        self.joints = None
        self.samples = []
        self.statuses = []

    def _on_joints(self, message):
        values = dict(zip(message.name, message.position))
        names = [f'joint{i}' for i in range(1, 7)]
        if all(name in values for name in names):
            self.joints = [values[name] for name in names]

    def _on_trajectory(self, message):
        if not message.points or self.joints is None:
            return
        values = dict(zip(message.joint_names, message.points[-1].positions))
        names = [f'joint{i}' for i in range(1, 7)]
        if all(name in values for name in names):
            self.samples.append({
                't': time.monotonic(),
                'target': [values[name] for name in names],
                'measured': list(self.joints),
            })

    def _on_status(self, message):
        if not self.statuses or self.statuses[-1] != message.data:
            self.statuses.append(message.data)

    def trigger(self, client):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('shadow_servo_service_unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if not future.done() or not future.result().success:
            raise RuntimeError('shadow_servo_trigger_failed')

    def publish(self, x):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = x
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = ShadowProbe()
    try:
        deadline = time.monotonic() + 5
        while node.joints is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.joints is None:
            raise RuntimeError('joint_states_unavailable')
        node.trigger(node.start)
        started = time.monotonic()
        while time.monotonic() - started < 0.5:
            node.publish(0.005)
            rclpy.spin_once(node, timeout_sec=0.02)
        for _ in range(25):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        print(json.dumps({
            'input_x_mps': 0.005,
            'duration_s': 0.5,
            'trajectory_count': len(node.samples),
            'statuses': node.statuses,
            'max_target_minus_measured_rad':
                max((max(abs(a-b) for a, b in zip(
                    row['target'], row['measured']))
                    for row in node.samples), default=None),
            'last_target_minus_measured_rad': (
                [a-b for a, b in zip(
                    node.samples[-1]['target'], node.samples[-1]['measured'])]
                if node.samples else None),
            'max_abs_target_from_first_rad': (
                max(abs(a-b) for row in node.samples
                    for a, b in zip(row['target'], node.samples[0]['target']))
                if node.samples else None),
        }, indent=2))
    finally:
        try:
            node.trigger(node.stop)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
