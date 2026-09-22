#!/usr/bin/env python3
"""One guarded stage-four +X pulse, with Servo/controller trace summaries."""

import argparse
import json
import math
import time

from control_msgs.msg import JointTrajectoryControllerState
from fairino_msgs.srv import ControlLease
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory


ARM_JOINTS = tuple(f'joint{index}' for index in range(1, 7))
SPEED_MPS = 0.005
PULSE_DURATION_S = 0.5
MAX_PULSE_DURATION_S = 1.0  # 5 mm nominal at the fixed 5 mm/s speed.
ZERO_SETTLE_S = 0.5
HARD_SERVO_FAULTS = (2, 4, 5)
MIN_ACCEPTABLE_X_FRACTION = 0.5
MAX_ACCEPTABLE_X_FRACTION = 1.5


class BaseXPulse(Node):
    def __init__(self):
        super().__init__('fr3c_stage4_base_x_pulse')
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
        self.create_subscription(
            String, '/perception/color_target_2d/status',
            self._on_target_status, 10)
        self.create_subscription(
            JointTrajectory,
            '/fairino3mt_execution_controller/joint_trajectory',
            self._on_servo_trajectory, 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            '/fairino3mt_execution_controller/controller_state',
            self._on_controller_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)
        self.latest_joints = None
        self.control_state = 'unknown'
        self.servo_status = None
        self.status_history = []
        self.latest_target = None
        self.record_pulse = False
        self.servo_trace = []
        self.controller_trace = []
        self.nonzero_command_count = 0
        self.heartbeat_count = 0
        self.pulse_wall_duration_s = None

    @staticmethod
    def _arm_positions(joint_names, positions):
        values = dict(zip(joint_names, positions))
        if all(name in values for name in ARM_JOINTS):
            result = [float(values[name]) for name in ARM_JOINTS]
            if all(math.isfinite(value) for value in result):
                return result
        return None

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

    def _on_target_status(self, message):
        try:
            payload = json.loads(message.data)
            target = payload.get('target_center_px')
            center = payload.get('center_reference_px')
            if (payload.get('valid') is True
                    and isinstance(target, list) and len(target) == 2
                    and isinstance(center, list) and len(center) == 2):
                self.latest_target = {
                    'stamp_s': float(payload.get('source_stamp_s', 0.0)),
                    'target_px': [float(target[0]), float(target[1])],
                    'center_px': [float(center[0]), float(center[1])],
                    'error_px': [
                        float(target[0]) - float(center[0]),
                        float(target[1]) - float(center[1]),
                    ],
                }
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def _on_servo_trajectory(self, message):
        if not self.record_pulse or not message.points:
            return
        positions = self._arm_positions(
            message.joint_names, message.points[-1].positions)
        if positions is not None:
            self.servo_trace.append(positions)

    def _on_controller_state(self, message):
        if not self.record_pulse:
            return
        sample = {
            'reference': self._arm_positions(
                message.joint_names, message.reference.positions),
            'feedback': self._arm_positions(
                message.joint_names, message.feedback.positions),
            'output': self._arm_positions(
                message.joint_names, message.output.positions),
        }
        self.controller_trace.append(sample)

    def publish_twist(self, x_mps=0.0):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = float(x_mps)
        self.twist_publisher.publish(message)
        if self.record_pulse and x_mps != 0.0:
            self.nonzero_command_count += 1

    def wait_until(self, predicate, timeout_s, command_x_mps=0.0):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            self.publish_twist(command_x_mps)
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def call_lease(self, operation, lease_id='', command_x_mps=0.0,
                   timeout_s=12.0, heartbeat_timeout_s=0.5):
        request = ControlLease.Request()
        request.operation = operation
        request.owner = 'SERVO_TRACKING'
        request.lease_id = lease_id
        request.heartbeat_timeout_s = heartbeat_timeout_s
        future = self.lease_client.call_async(request)
        if not self.wait_until(
                future.done, timeout_s, command_x_mps=command_x_mps):
            raise RuntimeError('control_lease_service_timeout')
        error = future.exception()
        if error is not None:
            raise RuntimeError(f'control_lease_service_error:{error}')
        return future.result()

    def sample_tcp(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'grasp_frame', Time(),
                timeout=Duration(seconds=0.2))
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'xyz_m': [translation.x, translation.y, translation.z],
            'quaternion_xyzw': [
                rotation.x, rotation.y, rotation.z, rotation.w],
        }

    def run_command_window(self, duration_s, x_mps, lease_id):
        started_s = time.monotonic()
        next_heartbeat_s = started_s
        if x_mps != 0.0:
            self.record_pulse = True
        try:
            while time.monotonic() - started_s < duration_s:
                self.publish_twist(x_mps)
                rclpy.spin_once(self, timeout_sec=0.02)
                now_s = time.monotonic()
                if now_s >= next_heartbeat_s:
                    heartbeat = self.call_lease(
                        ControlLease.Request.HEARTBEAT,
                        lease_id,
                        command_x_mps=x_mps,
                        timeout_s=0.15)
                    if not heartbeat.success:
                        raise RuntimeError(
                            f'heartbeat_rejected:{heartbeat.owner_state}:'
                            f'{heartbeat.message}')
                    if self.record_pulse:
                        self.heartbeat_count += 1
                    next_heartbeat_s = now_s + 0.1
                if self.servo_status in HARD_SERVO_FAULTS:
                    raise RuntimeError(f'servo_hard_fault:{self.servo_status}')
        finally:
            if x_mps != 0.0:
                self.pulse_wall_duration_s = time.monotonic() - started_s
                self.record_pulse = False


def summarize_trace(samples, start_joints):
    if not samples or start_joints is None:
        return {'sample_count': len(samples)}
    return {
        'sample_count': len(samples),
        'first_delta_rad': [value - start for value, start in zip(
            samples[0], start_joints)],
        'last_delta_rad': [value - start for value, start in zip(
            samples[-1], start_joints)],
        'max_abs_delta_rad': max(
            abs(value - start)
            for sample in samples
            for value, start in zip(sample, start_joints)),
    }


def motion_verdict(actual_x_fraction, servo_target_count,
                   controller_output_count, nonzero_command_count):
    """Separate transport failures from a measured under-travel."""
    if nonzero_command_count == 0:
        return 'no_nonzero_twist_published'
    if servo_target_count == 0:
        return 'no_servo_trajectory_observed'
    if controller_output_count == 0:
        return 'no_controller_output_observed'
    if actual_x_fraction is None or not math.isfinite(actual_x_fraction):
        return 'tcp_feedback_unavailable'
    if actual_x_fraction <= 0.0:
        return 'wrong_direction_or_no_motion'
    if actual_x_fraction < MIN_ACCEPTABLE_X_FRACTION:
        return 'undertravel'
    if actual_x_fraction > MAX_ACCEPTABLE_X_FRACTION:
        return 'overtravel'
    return 'passed'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--duration-s', type=float, default=PULSE_DURATION_S,
        help='Nonzero pulse duration; max 1.0 s (5 mm nominal).')
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or not (
            0.0 < args.duration_s <= MAX_PULSE_DURATION_S):
        parser.error('--duration-s must be finite and in (0, 1.0]')

    rclpy.init()
    node = BaseXPulse()
    lease_id = ''
    outcome = 'failed_before_acquire'
    detail = ''
    start_joints = None
    start_tcp = None
    start_target = None
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
        if not node.wait_until(
                lambda: node.sample_tcp() is not None, 5.0):
            raise RuntimeError('base_to_grasp_frame_tf_unavailable')

        acquired = node.call_lease(ControlLease.Request.ACQUIRE)
        if not acquired.success:
            raise RuntimeError(
                f'control_acquire_rejected:{acquired.owner_state}:'
                f'{acquired.message}')
        lease_id = acquired.lease_id
        outcome = 'lease_acquired'

        node.run_command_window(ZERO_SETTLE_S, 0.0, lease_id)
        # Measure the nonzero pulse from its own baseline, not from before
        # controller activation and the zero-command settling interval.
        start_joints = node.latest_joints
        start_tcp = node.sample_tcp()
        start_target = node.latest_target
        node.run_command_window(args.duration_s, SPEED_MPS, lease_id)
        node.run_command_window(ZERO_SETTLE_S, 0.0, lease_id)
        outcome = 'pulse_complete'
    except Exception as error:
        detail = f'{type(error).__name__}:{error}'
    finally:
        if lease_id:
            try:
                node.run_command_window(ZERO_SETTLE_S, 0.0, lease_id)
                released = node.call_lease(
                    ControlLease.Request.RELEASE, lease_id)
                if not released.success:
                    detail = (detail + ';' if detail else '') + (
                        f'release_rejected:{released.owner_state}:'
                        f'{released.message}')
                elif outcome == 'pulse_complete':
                    outcome = 'passed'
            except Exception as error:
                detail = (detail + ';' if detail else '') + (
                    f'release_exception:{type(error).__name__}:{error}')

        node.wait_until(
            lambda: node.control_state == 'monitoring_manual_ready', 3.0)
        end_joints = node.latest_joints
        end_tcp = node.sample_tcp()
        end_target = node.latest_target
        joint_delta = None
        if start_joints is not None and end_joints is not None:
            joint_delta = [end - start for start, end in zip(
                start_joints, end_joints)]
        tcp_delta = None
        if start_tcp is not None and end_tcp is not None:
            tcp_delta = [end - start for start, end in zip(
                start_tcp['xyz_m'], end_tcp['xyz_m'])]
        actual_x_fraction = None
        if tcp_delta is not None:
            actual_x_fraction = tcp_delta[0] / (
                SPEED_MPS * args.duration_s)
        controller_output_samples = [
            sample['output'] for sample in node.controller_trace
            if sample['output'] is not None]
        verdict = motion_verdict(
            actual_x_fraction, len(node.servo_trace),
            len(controller_output_samples), node.nonzero_command_count)
        if outcome == 'passed' and verdict != 'passed':
            outcome = verdict
        target_error_delta = None
        if start_target is not None and end_target is not None:
            target_error_delta = [end - start for start, end in zip(
                start_target['error_px'], end_target['error_px'])]
        report = {
            'outcome': outcome,
            'detail': detail,
            'command': {
                'frame': 'base_link',
                'linear_x_mps': SPEED_MPS,
                'duration_s': args.duration_s,
                'nominal_displacement_m': SPEED_MPS * args.duration_s,
                'linear_y_mps': 0.0,
                'linear_z_mps': 0.0,
                'angular_xyz_radps': [0.0, 0.0, 0.0],
            },
            'control_state': node.control_state,
            'servo_status_history': node.status_history,
            'pulse_trace': {
                'wall_duration_s': node.pulse_wall_duration_s,
                'nonzero_twist_publish_count': node.nonzero_command_count,
                'heartbeat_count': node.heartbeat_count,
                'servo_target': summarize_trace(
                    node.servo_trace, start_joints),
                'controller_reference': summarize_trace(
                    [sample['reference'] for sample in node.controller_trace
                     if sample['reference'] is not None],
                    start_joints),
                'controller_output': summarize_trace(
                    controller_output_samples,
                    start_joints),
                'controller_feedback': summarize_trace(
                    [sample['feedback'] for sample in node.controller_trace
                     if sample['feedback'] is not None],
                    start_joints),
            },
            'start_joints_rad': start_joints,
            'end_joints_rad': end_joints,
            'joint_delta_rad': joint_delta,
            'start_tcp': start_tcp,
            'end_tcp': end_tcp,
            'tcp_delta_xyz_m': tcp_delta,
            'actual_x_over_nominal_fraction': actual_x_fraction,
            'motion_verdict': verdict,
            'minimum_acceptable_x_fraction': MIN_ACCEPTABLE_X_FRACTION,
            'maximum_acceptable_x_fraction': MAX_ACCEPTABLE_X_FRACTION,
            'start_target': start_target,
            'end_target': end_target,
            'target_error_delta_px': target_error_delta,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
        if outcome != 'passed':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
