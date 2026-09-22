#!/usr/bin/env python3
"""Stage-four zero-command lease test; never publishes a nonzero Twist."""

import argparse
import json
import math
import time

from fairino_msgs.srv import ControlLease
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String


ARM_JOINTS = tuple(f'joint{index}' for index in range(1, 7))


class ZeroSpeedTest(Node):
    def __init__(self):
        super().__init__('fr3c_stage4_zero_speed_test')
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lease_client = self.create_client(
            ControlLease, '/fairino/control_lease')
        self.twist_publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(
            String, '/fairino/control_state', self._on_control_state,
            state_qos)
        self.create_subscription(
            Int8, '/servo_node/status', self._on_servo_status, 10)
        self.latest_joints = None
        self.control_state = 'unknown'
        self.servo_status = None
        self.status_history = []

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in ARM_JOINTS):
            values = tuple(float(positions[name]) for name in ARM_JOINTS)
            if all(math.isfinite(value) for value in values):
                self.latest_joints = values

    def _on_control_state(self, message):
        self.control_state = message.data.strip()

    def _on_servo_status(self, message):
        status = int(message.data)
        self.servo_status = status
        if not self.status_history or self.status_history[-1] != status:
            self.status_history.append(status)

    def publish_zero(self):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        self.twist_publisher.publish(message)

    def wait_until(self, predicate, timeout_s, publish_zero=False):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            if publish_zero:
                self.publish_zero()
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def call_lease(self, operation, lease_id='', publish_zero=False):
        request = ControlLease.Request()
        request.operation = operation
        request.owner = 'SERVO_TRACKING'
        request.lease_id = lease_id
        request.heartbeat_timeout_s = 0.5
        future = self.lease_client.call_async(request)
        if not self.wait_until(
                future.done, 12.0, publish_zero=publish_zero):
            raise RuntimeError('control_lease_service_timeout')
        error = future.exception()
        if error is not None:
            raise RuntimeError(f'control_lease_service_error:{error}')
        return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watchdog-test', action='store_true',
                        help='Stop lease heartbeats after the zero window.')
    args = parser.parse_args()
    rclpy.init()
    node = ZeroSpeedTest()
    lease_id = ''
    start_joints = None
    outcome = 'failed_before_acquire'
    detail = ''
    watchdog_release_delay_s = None
    try:
        if not node.lease_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('control_lease_service_unavailable')
        if not node.wait_until(
                lambda: node.latest_joints is not None, 5.0):
            raise RuntimeError('joint_states_unavailable')
        if not node.wait_until(
                lambda: node.control_state == 'monitoring_manual_ready',
                5.0):
            raise RuntimeError(
                f'control_not_idle:{node.control_state}')

        acquired = node.call_lease(ControlLease.Request.ACQUIRE)
        if not acquired.success:
            raise RuntimeError(
                f'control_acquire_rejected:{acquired.owner_state}:'
                f'{acquired.message}')
        lease_id = acquired.lease_id
        outcome = 'lease_acquired'

        started_s = time.monotonic()
        next_heartbeat_s = started_s
        while time.monotonic() - started_s < 3.0:
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=0.02)
            now_s = time.monotonic()
            if now_s >= next_heartbeat_s:
                heartbeat = node.call_lease(
                    ControlLease.Request.HEARTBEAT,
                    lease_id,
                    publish_zero=True)
                if not heartbeat.success:
                    raise RuntimeError(
                        f'heartbeat_rejected:{heartbeat.owner_state}:'
                        f'{heartbeat.message}')
                next_heartbeat_s = now_s + 0.1
            if node.servo_status in (2, 4, 5):
                raise RuntimeError(
                    f'servo_hard_fault:{node.servo_status}')
            # Exclude controller activation from the zero-command drift
            # measurement. The arm has 0.5 s to settle before the baseline.
            if start_joints is None and now_s - started_s >= 0.5:
                start_joints = node.latest_joints

        for _ in range(10):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=0.02)
        if args.watchdog_test:
            stopped_heartbeats_at = time.monotonic()
            if not node.wait_until(
                    lambda: node.control_state == 'monitoring_manual_ready',
                    1.5, publish_zero=True):
                raise RuntimeError(
                    f'heartbeat_watchdog_failed:{node.control_state}')
            watchdog_release_delay_s = (
                time.monotonic() - stopped_heartbeats_at)
        outcome = 'zero_sequence_complete'
    except Exception as error:
        detail = f'{type(error).__name__}:{error}'
    finally:
        if lease_id:
            try:
                for _ in range(10):
                    node.publish_zero()
                    rclpy.spin_once(node, timeout_sec=0.02)
                released = node.call_lease(
                    ControlLease.Request.RELEASE,
                    lease_id,
                    publish_zero=True)
                if not released.success:
                    detail = (detail + ';' if detail else '') + (
                        f'release_rejected:{released.owner_state}:'
                        f'{released.message}')
                elif outcome == 'zero_sequence_complete':
                    outcome = 'passed'
            except Exception as error:
                detail = (detail + ';' if detail else '') + (
                    f'release_exception:{type(error).__name__}:{error}')

        node.wait_until(
            lambda: node.control_state == 'monitoring_manual_ready', 3.0)
        end_joints = node.latest_joints
        delta = None
        if start_joints is not None and end_joints is not None:
            delta = [end - start for start, end in zip(
                start_joints, end_joints)]
        report = {
            'outcome': outcome,
            'detail': detail,
            'control_state': node.control_state,
            'servo_status_history': node.status_history,
            'watchdog_release_delay_s': watchdog_release_delay_s,
            'start_joints_rad': start_joints,
            'end_joints_rad': end_joints,
            'joint_delta_rad': delta,
            'max_abs_joint_delta_rad': (
                max(abs(value) for value in delta)
                if delta is not None else None),
        }
        if (outcome == 'passed' and delta is not None
                and report['max_abs_joint_delta_rad'] > 0.02):
            outcome = 'zero_command_joint_drift'
            report['outcome'] = outcome
        print(json.dumps(report, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
        if outcome != 'passed':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
