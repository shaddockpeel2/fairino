#!/usr/bin/env python3
"""Record the full control chain during one guarded constant P1 Twist."""

import argparse
import json
import math
from pathlib import Path
import time

from control_msgs.msg import DynamicJointState
from fairino_msgs.srv import ControlLease
import rclpy
from std_msgs.msg import Float64MultiArray

from constant_twist_diagnostic_logic import motion_summary, p1_acceptance
from stage4_base_x_pulse import ARM_JOINTS, HARD_SERVO_FAULTS
from stage4_direction_pulse import (
    BASE_DIRECTIONS,
    DirectionPulse,
    orientation_change_deg,
)


DEFAULT_SPEED_MPS = 0.005
MAX_SPEED_MPS = 0.010
MIN_DURATION_S = 0.5
MAX_DURATION_S = 3.0
# P1 requires a two-second zero-command gate before every motion pulse.
PRE_ZERO_S = 2.0
POST_ZERO_S = 1.0
SETTLE_AFTER_RELEASE_S = 0.5
TIMING_FIELDS = {
    'read_position_call_ms', 'read_status_call_ms', 'servo_j_call_ms',
    'servo_j_interval_ms', 'control_period_ms'}


def ros_stamp_s(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def point_payload(point):
    return {
        'positions': list(point.positions),
        'velocities': list(point.velocities),
        'accelerations': list(point.accelerations),
        'effort': list(point.effort),
        'time_from_start_s': ros_stamp_s(point.time_from_start),
    }


class ConstantTwistDiagnostic(DirectionPulse):
    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.origin_s = time.monotonic()
        self.phase = 'preflight'
        self.recording = False
        self.twists = []
        self.servo_trajectories = []
        self.forward_commands = []
        self.controller_states = []
        self.joint_states = []
        self.hardware_timing = []
        self.tcp_trace = []
        self.motion_origin_pose = None
        self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self._on_dynamic_state,
            100,
        )
        self.create_subscription(
            Float64MultiArray,
            '/fairino3mt_servo_position_controller/commands',
            self._on_forward_command,
            100,
        )

    def elapsed(self):
        return time.monotonic() - self.origin_s

    def publish_twist(self, speed_fraction=0.0):
        super().publish_twist(speed_fraction)
        if self.recording:
            self.twists.append({
                't': self.elapsed(), 'phase': self.phase,
                'xy_mps': [self.command_xy[0] * speed_fraction,
                           self.command_xy[1] * speed_fraction],
            })

    def _on_servo_trajectory(self, message):
        super()._on_servo_trajectory(message)
        if self.recording:
            self.servo_trajectories.append({
                't': self.elapsed(), 'phase': self.phase,
                'header_stamp_s': ros_stamp_s(message.header.stamp),
                'joint_names': list(message.joint_names),
                'points': [point_payload(point) for point in message.points],
            })

    def _on_controller_state(self, message):
        super()._on_controller_state(message)
        if self.recording:
            self.controller_states.append({
                't': self.elapsed(), 'phase': self.phase,
                'header_stamp_s': ros_stamp_s(message.header.stamp),
                'joint_names': list(message.joint_names),
                'reference': point_payload(message.reference),
                'feedback': point_payload(message.feedback),
                'error': point_payload(message.error),
                'output': point_payload(message.output),
            })

    def _on_forward_command(self, message):
        if not self.recording:
            return
        values = [float(value) for value in message.data]
        if len(values) == len(ARM_JOINTS) and all(
                math.isfinite(value) for value in values):
            self.forward_commands.append({
                't': self.elapsed(), 'phase': self.phase,
                'positions_rad': values,
            })

    def _on_joint_state(self, message):
        super()._on_joint_state(message)
        if not self.recording:
            return
        lookup = {name: index for index, name in enumerate(message.name)}
        if not all(name in lookup for name in ARM_JOINTS):
            return
        indices = [lookup[name] for name in ARM_JOINTS]
        positions = ([float(message.position[index]) for index in indices]
                     if len(message.position) == len(message.name) else [])
        velocities = ([float(message.velocity[index]) for index in indices]
                      if len(message.velocity) == len(message.name) else [])
        finite_velocities = (
            not velocities
            or all(math.isfinite(value) for value in velocities)
        )
        if (positions
                and all(math.isfinite(value) for value in positions)
                and finite_velocities):
            self.joint_states.append({
                't': self.elapsed(), 'phase': self.phase,
                'header_stamp_s': ros_stamp_s(message.header.stamp),
                'positions_rad': positions, 'velocities_radps': velocities,
            })

    def _on_dynamic_state(self, message):
        if not self.recording:
            return
        for name, values in zip(message.joint_names, message.interface_values):
            if name != 'fairino_status':
                continue
            metrics = {
                key: float(value)
                for key, value in zip(values.interface_names, values.values)
                if key in TIMING_FIELDS and math.isfinite(float(value))
            }
            if metrics:
                self.hardware_timing.append({
                    't': self.elapsed(), 'phase': self.phase,
                    'header_stamp_s': ros_stamp_s(message.header.stamp),
                    'metrics': metrics,
                })

    def sample_tcp_trace(self):
        pose = self.sample_tcp()
        if pose is not None and self.recording:
            self.tcp_trace.append({
                't': self.elapsed(), 'phase': self.phase,
                'xyz_m': pose['xyz_m'],
                'quaternion_xyzw': pose['quaternion_xyzw'],
            })
        return pose

    def run_constant_window(self, duration_s, lease_id, fraction, phase,
                            stop_mode=None, stop_after_s=None):
        self.phase = phase
        started = time.monotonic()
        next_heartbeat = started
        heartbeat_future = None
        heartbeat_sent = None
        stop_injected = False
        publish_commands = True
        renew_heartbeat = True
        if fraction:
            self.record_pulse = True
            self.pulse_started_at = started
        try:
            while rclpy.ok() and time.monotonic() - started < duration_s:
                now = time.monotonic()
                elapsed_s = now - started
                if (stop_mode is not None and not stop_injected
                        and elapsed_s >= stop_after_s):
                    stop_injected = True
                    if stop_mode == 'software_cancel':
                        self.publish_twist(0.0)
                        return 'software_cancel_injected'
                    if stop_mode == 'heartbeat_timeout':
                        renew_heartbeat = False
                    elif stop_mode == 'publisher_exit':
                        publish_commands = False
                if (self.joint_receive_s is None
                        or now - self.joint_receive_s > 0.25):
                    raise RuntimeError('joint_states_stale')
                if self.control_state != 'executing_servo_tracking':
                    if (stop_injected
                            and stop_mode in ('heartbeat_timeout',
                                              'publisher_exit')
                            and self.control_state ==
                            'monitoring_manual_ready'):
                        return f'{stop_mode}_released'
                    raise RuntimeError(
                        f'control_state_changed:{self.control_state}')
                if self.servo_status in HARD_SERVO_FAULTS:
                    raise RuntimeError(f'servo_hard_fault:{self.servo_status}')
                if heartbeat_future is not None:
                    if heartbeat_future.done():
                        if heartbeat_future.exception() is not None:
                            raise RuntimeError('heartbeat_service_error')
                        response = heartbeat_future.result()
                        if response is None or not response.success:
                            raise RuntimeError('heartbeat_rejected')
                        self.heartbeat_latencies_s.append(
                            time.monotonic() - heartbeat_sent)
                        self.heartbeat_count += 1
                        heartbeat_future = None
                        next_heartbeat = time.monotonic() + 0.1
                    elif time.monotonic() - heartbeat_sent > 0.15:
                        raise RuntimeError('heartbeat_service_timeout')
                if (renew_heartbeat and heartbeat_future is None
                        and now >= next_heartbeat):
                    request = ControlLease.Request()
                    request.operation = ControlLease.Request.HEARTBEAT
                    request.owner = 'SERVO_TRACKING'
                    request.lease_id = lease_id
                    request.heartbeat_timeout_s = 0.5
                    heartbeat_future = self.lease_client.call_async(request)
                    heartbeat_sent = time.monotonic()
                if publish_commands:
                    self.publish_twist(fraction)
                pose = self.sample_tcp_trace()
                if fraction and pose is not None:
                    if self.motion_origin_pose is None:
                        self.motion_origin_pose = pose
                    origin = self.motion_origin_pose
                    relative_xy = math.hypot(
                        pose['xyz_m'][0] - origin['xyz_m'][0],
                        pose['xyz_m'][1] - origin['xyz_m'][1])
                    command_speed_mps = math.hypot(*self.command_xy)
                    if relative_xy > command_speed_mps * duration_s + 0.005:
                        raise RuntimeError(
                            'measured_xy_displacement_limit_reached')
                    if abs(pose['xyz_m'][2] - origin['xyz_m'][2]) > 0.003:
                        raise RuntimeError('measured_z_drift_limit_reached')
                    if orientation_change_deg(origin, pose) > 5.0:
                        raise RuntimeError(
                            'measured_orientation_drift_limit_reached')
                # Drain the 125 Hz controller, joint-state, and hardware timing
                # streams between 50 Hz Twist publications. One spin_once per
                # command period would service only one ready subscription and
                # systematically under-sample the exact signals under test.
                drain_deadline = time.monotonic() + 0.02
                while rclpy.ok() and time.monotonic() < drain_deadline:
                    rclpy.spin_once(self, timeout_sec=min(
                        0.002, drain_deadline - time.monotonic()))
            if stop_injected and stop_mode in (
                    'heartbeat_timeout', 'publisher_exit'):
                raise RuntimeError(f'{stop_mode}_did_not_release')
            return 'window_complete'
        finally:
            self.publish_twist(0.0)
            if fraction:
                self.pulse_wall_duration_s = time.monotonic() - started
                self.pulse_ended_at = time.monotonic()
                self.record_pulse = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction', choices=sorted(BASE_DIRECTIONS))
    parser.add_argument('--duration-s', type=float, default=2.0)
    parser.add_argument('--speed-mps', type=float, default=DEFAULT_SPEED_MPS)
    parser.add_argument(
        '--high-speed-validation', action='store_true',
        help='Required for the 0.010 m/s comparison after all 0.005 m/s gates pass.')
    parser.add_argument(
        '--backend', choices=('jtc', 'forward_position'),
        default='forward_position',
        help='Expected MoveIt Servo output backend for this A/B capture.')
    stop_group = parser.add_mutually_exclusive_group()
    stop_group.add_argument('--cancel-after-s', type=float)
    stop_group.add_argument('--heartbeat-timeout-after-s', type=float)
    stop_group.add_argument('--publisher-exit-after-s', type=float)
    parser.add_argument('--operator-ready', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.operator_ready:
        parser.error('motion requires --operator-ready')
    if (not math.isfinite(args.duration_s)
            or not MIN_DURATION_S <= args.duration_s <= MAX_DURATION_S):
        parser.error(
            '--duration-s must be within '
            f'[{MIN_DURATION_S}, {MAX_DURATION_S}]')
    if (not math.isfinite(args.speed_mps)
            or not DEFAULT_SPEED_MPS <= args.speed_mps <= MAX_SPEED_MPS):
        parser.error(
            f'--speed-mps must be within '
            f'[{DEFAULT_SPEED_MPS}, {MAX_SPEED_MPS}]')
    if (args.speed_mps > DEFAULT_SPEED_MPS
            and not args.high_speed_validation):
        parser.error(
            '--high-speed-validation is required above 0.005 m/s')
    stop_options = {
        'software_cancel': args.cancel_after_s,
        'heartbeat_timeout': args.heartbeat_timeout_after_s,
        'publisher_exit': args.publisher_exit_after_s,
    }
    selected_stops = [(name, value) for name, value in stop_options.items()
                      if value is not None]
    stop_mode, stop_after_s = (
        selected_stops[0] if selected_stops else (None, None))
    if (stop_after_s is not None
            and (not math.isfinite(stop_after_s)
                 or not 0.0 < stop_after_s < args.duration_s)):
        parser.error('stop injection time must be within (0, --duration-s)')

    rclpy.init()
    node = ConstantTwistDiagnostic(args.backend)
    node.command_xy = tuple(
        args.speed_mps * value for value in BASE_DIRECTIONS[args.direction])
    lease_id = ''
    outcome = 'failed_before_acquire'
    detail = ''
    start_tcp = stop_tcp = end_tcp = None
    acquired_wall_s = released_wall_s = None
    try:
        if not node.lease_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('control_lease_service_unavailable')
        if not node.wait_until(lambda: node.latest_joints is not None, 5.0):
            raise RuntimeError('joint_states_unavailable')
        if not node.wait_until(
                lambda: node.control_state == 'monitoring_manual_ready', 5.0):
            raise RuntimeError(f'control_not_idle:{node.control_state}')
        if len(node.get_publishers_info_by_topic(
                '/servo_node/delta_twist_cmds')) != 1:
            raise RuntimeError('another_servo_twist_publisher_is_active')
        if not node.wait_until(lambda: node.sample_tcp() is not None, 5.0):
            raise RuntimeError('tcp_tf_unavailable')
        node.check_scene()
        acquired = node.call_lease(ControlLease.Request.ACQUIRE)
        if not acquired.success:
            raise RuntimeError(f'control_acquire_rejected:{acquired.message}')
        lease_id = acquired.lease_id
        acquired_wall_s = time.time()
        node.recording = True
        node.run_constant_window(PRE_ZERO_S, lease_id, 0.0, 'pre_zero')
        start_tcp = node.sample_tcp_trace()
        stop_result = node.run_constant_window(
            args.duration_s, lease_id, 1.0, 'motion',
            stop_mode=stop_mode, stop_after_s=stop_after_s)
        stop_tcp = node.sample_tcp_trace()
        if stop_mode is None:
            node.run_constant_window(
                POST_ZERO_S, lease_id, 0.0, 'post_zero')
            end_tcp = node.sample_tcp_trace()
            outcome = 'capture_complete'
        else:
            end_tcp = stop_tcp
            outcome = stop_result
    except (Exception, KeyboardInterrupt) as error:
        detail = f'{type(error).__name__}:{error}'
    finally:
        if lease_id:
            if (node.control_state == 'executing_servo_tracking'
                    and stop_mode != 'software_cancel'):
                try:
                    node.phase = 'cleanup_zero'
                    node.run_constant_window(
                        0.5, lease_id, 0.0, 'cleanup_zero')
                except Exception as error:
                    detail += f';cleanup_zero:{type(error).__name__}:{error}'
            try:
                release = node.call_lease(
                    ControlLease.Request.RELEASE, lease_id)
                if not release.success:
                    raise RuntimeError(release.message)
                released_wall_s = time.time()
            except Exception as error:
                detail += f';release:{type(error).__name__}:{error}'
        node.phase = 'after_release'
        deadline = time.monotonic() + SETTLE_AFTER_RELEASE_S
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.sample_tcp_trace()
        node.recording = False
        final_tcp = node.sample_tcp()
        node.wait_until(
            lambda: node.control_state == 'monitoring_manual_ready', 3.0)
        record = {
            'schema_version': 2,
            'test': 'constant_twist_control_chain_diagnostic',
            'backend': args.backend,
            'outcome': outcome,
            'detail': detail,
            'direction': args.direction,
            'command_xy_mps': list(node.command_xy),
            'duration_s': args.duration_s,
            'stop_mode': stop_mode,
            'stop_after_s': stop_after_s,
            'nominal_displacement_m': args.speed_mps * args.duration_s,
            'control_state_after': node.control_state,
            'servo_status_history': node.status_history,
            'heartbeat_latencies_s': node.heartbeat_latencies_s,
            'scene_object_ids': node.scene_objects,
            'lease_acquired_wall_s': acquired_wall_s,
            'lease_released_wall_s': released_wall_s,
            'start_tcp': start_tcp,
            'stop_tcp': stop_tcp,
            'end_tcp': end_tcp,
            'final_tcp': final_tcp,
            'twists': node.twists,
            'servo_trajectories': node.servo_trajectories,
            'forward_commands': node.forward_commands,
            'controller_states': node.controller_states,
            'joint_states': node.joint_states,
            'hardware_timing': node.hardware_timing,
            'tcp': node.tcp_trace,
        }
        record['summary'] = motion_summary(record, node.command_xy)
        backend_output_observed = (
            bool(node.forward_commands)
            if args.backend == 'forward_position'
            else bool(node.servo_trajectories))
        record['backend_output_observed'] = backend_output_observed
        record['p1_acceptance'] = p1_acceptance(record, record['summary'])
        if start_tcp and final_tcp:
            delta = [end - start for start, end in zip(
                start_tcp['xyz_m'], final_tcp['xyz_m'])]
            record['total_tcp_delta_xyz_m'] = delta
            record['orientation_change_deg'] = orientation_change_deg(
                start_tcp, final_tcp)
        successful_outcomes = {
            'capture_complete', 'software_cancel_injected',
            'heartbeat_timeout_released', 'publisher_exit_released'}
        safe_capture = (
            outcome in successful_outcomes
            and released_wall_s is not None
            and node.control_state == 'monitoring_manual_ready'
            and backend_output_observed
            and not any(status in HARD_SERVO_FAULTS
                        for status in node.status_history))
        record['capture_safety_passed'] = safe_capture
        payload = json.dumps(record, indent=2, sort_keys=True) + '\n'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding='utf-8')
        print(json.dumps({
            'outcome': outcome,
            'detail': detail,
            'capture_safety_passed': safe_capture,
            'control_state_after': node.control_state,
            'summary': record['summary'],
            'p1_acceptance': record['p1_acceptance'],
            'output': str(args.output),
        }, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
        return 0 if safe_capture else 1


if __name__ == '__main__':
    raise SystemExit(main())
