#!/usr/bin/env python3
"""Run one leased, at-most-2.5-mm nominal stage-four direction pulse.

Run each direction separately. This is deliberately not a closed-loop tracker.
"""

import argparse
import json
import math
from pathlib import Path
import time

from fairino_msgs.srv import ControlLease
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

from stage4_base_x_pulse import BaseXPulse, HARD_SERVO_FAULTS, summarize_trace
from stage4_pulse_logic import (
    MAX_DURATION_S, MAX_SPEED_MPS, TARGET_MAX_AGE_S,
    command_for_side, error_improved, pulse_verdict,
)


BASE_DIRECTIONS = {
    'x+': (MAX_SPEED_MPS, 0.0),
    'x-': (-MAX_SPEED_MPS, 0.0),
    'y+': (0.0, MAX_SPEED_MPS),
    'y-': (0.0, -MAX_SPEED_MPS),
}


class DirectionPulse(BaseXPulse):
    def __init__(self):
        super().__init__()
        self.command_xy = (0.0, 0.0)
        self.target_receive_s = None
        self.joint_receive_s = None
        self.target_source_age_s = math.inf
        self.target_invalid_reason = 'no_observation'
        self.tcp_samples = []
        self.scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene')
        self.scene_objects = []
        self.tcp_tf_age_s = None
        self.camera_tf_age_s = None
        self.heartbeat_latencies_s = []

    def _on_joint_state(self, message):
        super()._on_joint_state(message)
        if self.latest_joints is not None:
            self.joint_receive_s = time.monotonic()

    def _transform_age_s(self, transform):
        stamp = transform.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        now_s = self.get_clock().now().nanoseconds * 1e-9
        return now_s - stamp_s

    def sample_tcp(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'grasp_frame', Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        self.tcp_tf_age_s = self._transform_age_s(transform)
        if not -0.05 <= self.tcp_tf_age_s <= 0.5:
            return None
        p = transform.transform.translation
        q = transform.transform.rotation
        if not all(math.isfinite(v) for v in (
                p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return None
        return {
            'xyz_m': [p.x, p.y, p.z],
            'quaternion_xyzw': [q.x, q.y, q.z, q.w],
        }

    def _on_target_status(self, message):
        try:
            data = json.loads(message.data)
            self.target_invalid_reason = str(data.get('reason', 'invalid'))
            if data.get('target_color') != 'red' or data.get('valid') is not True:
                self.latest_target = None
                return
            target = [float(v) for v in data['target_center_px']]
            center = [float(v) for v in data['center_reference_px']]
            size = [float(data['image_width']), float(data['image_height'])]
            age = float(data['observation_age_s'])
            stamp = float(data['source_stamp_s'])
            if (len(target) != 2 or len(center) != 2 or len(size) != 2
                    or not all(math.isfinite(v) for v in
                               target + center + size + [age, stamp])
                    or min(size) <= 0 or age < 0):
                raise ValueError('invalid_observation_geometry')
            self.latest_target = {
                'stamp_s': stamp,
                'target_px': target,
                'center_px': center,
                'image_size_px': size,
                'error_px': [target[i] - center[i] for i in range(2)],
            }
            self.target_receive_s = time.monotonic()
            self.target_source_age_s = age
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.latest_target = None
            self.target_invalid_reason = 'malformed_observation'

    def target_fresh(self):
        return (self.latest_target is not None
                and self.target_receive_s is not None
                and self.target_source_age_s +
                time.monotonic() - self.target_receive_s <= TARGET_MAX_AGE_S)

    def publish_twist(self, speed_fraction=0.0):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = self.command_xy[0] * speed_fraction
        message.twist.linear.y = self.command_xy[1] * speed_fraction
        self.twist_publisher.publish(message)
        if self.record_pulse and speed_fraction:
            self.nonzero_command_count += 1

    def sample_tcp_now(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'grasp_frame', Time(),
                timeout=Duration(seconds=0.0))
        except Exception:
            return
        if not -0.05 <= self._transform_age_s(transform) <= 0.5:
            return
        p = transform.transform.translation
        self.tcp_samples.append((time.monotonic(), (p.x, p.y, p.z)))

    def camera_quaternion(self):
        transform = self.tf_buffer.lookup_transform(
            'base_link', 'camera_color_optical_frame', Time(),
            timeout=Duration(seconds=0.2))
        self.camera_tf_age_s = self._transform_age_s(transform)
        if not -0.05 <= self.camera_tf_age_s <= 0.5:
            raise RuntimeError('camera_tf_stale')
        q = transform.transform.rotation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError('camera_tf_invalid')
        return values

    def check_scene(self):
        if not self.scene_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('planning_scene_service_unavailable')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = self.scene_client.call_async(request)
        if not self.wait_until(future.done, 3.0):
            raise RuntimeError('planning_scene_timeout')
        response = future.result()
        if response is None:
            raise RuntimeError('planning_scene_failed')
        self.scene_objects = [obj.id for obj in
                              response.scene.world.collision_objects]
        if not any('table' in name.lower() or 'plane' in name.lower()
                   or 'ground' in name.lower()
                   for name in self.scene_objects):
            raise RuntimeError('table_collision_object_missing')

    def run_window(self, duration_s, lease_id, moving=False,
                   renew_heartbeat=True, inject_event=None,
                   inject_after_s=None):
        started = time.monotonic()
        deadline = started + duration_s
        next_heartbeat = started
        heartbeat_future = None
        heartbeat_sent_at = None
        if moving:
            self.record_pulse = True
            self.pulse_started_at = started
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if moving:
                    if (inject_event is not None and
                            time.monotonic() - started >= inject_after_s):
                        if inject_event == 'cancel':
                            raise RuntimeError('test_cancel_requested')
                        if inject_event == 'target_loss':
                            self.latest_target = None
                            self.target_invalid_reason = 'test_injected_target_loss'
                    if (self.joint_receive_s is None or
                            time.monotonic() - self.joint_receive_s > 0.5):
                        raise RuntimeError('joint_states_stale')
                    if not self.target_fresh():
                        target_age = (
                            self.target_source_age_s + time.monotonic()
                            - self.target_receive_s
                            if self.target_receive_s is not None else None)
                        raise RuntimeError(
                            f'red_target_lost:{self.target_invalid_reason}:'
                            f'age_s={target_age}')
                    if self.control_state != 'executing_servo_tracking':
                        raise RuntimeError(
                            f'control_state_changed:{self.control_state}')
                    if self.servo_status in HARD_SERVO_FAULTS:
                        raise RuntimeError(
                            f'servo_hard_fault:{self.servo_status}')
                    if renew_heartbeat and heartbeat_future is not None:
                        if heartbeat_future.done():
                            if heartbeat_future.exception() is not None:
                                raise RuntimeError('heartbeat_service_error')
                            response = heartbeat_future.result()
                            if response is None or not response.success:
                                raise RuntimeError('heartbeat_rejected')
                            self.heartbeat_latencies_s.append(
                                time.monotonic() - heartbeat_sent_at)
                            self.heartbeat_count += 1
                            heartbeat_future = None
                            next_heartbeat = time.monotonic() + 0.1
                        elif time.monotonic() - heartbeat_sent_at > 0.15:
                            raise RuntimeError('heartbeat_service_timeout')
                    if (renew_heartbeat and heartbeat_future is None
                            and time.monotonic() >= next_heartbeat):
                        request = ControlLease.Request()
                        request.operation = ControlLease.Request.HEARTBEAT
                        request.owner = 'SERVO_TRACKING'
                        request.lease_id = lease_id
                        request.heartbeat_timeout_s = 0.5
                        heartbeat_future = self.lease_client.call_async(request)
                        heartbeat_sent_at = time.monotonic()
                self.publish_twist(1.0 if moving else 0.0)
                self.sample_tcp_now()
                rclpy.spin_once(self, timeout_sec=0.02)
                if not moving and time.monotonic() >= next_heartbeat:
                    heartbeat = self.call_lease(
                        ControlLease.Request.HEARTBEAT, lease_id,
                        command_x_mps=0.0, timeout_s=0.15)
                    if not heartbeat.success:
                        raise RuntimeError(
                            f'heartbeat_rejected:{heartbeat.message}')
                    next_heartbeat = time.monotonic() + 0.1
        finally:
            self.publish_twist(0.0)
            if moving:
                self.pulse_wall_duration_s = time.monotonic() - started
                self.pulse_ended_at = time.monotonic()
                self.record_pulse = False


def max_measured_speed(samples):
    """Estimate TCP speed over 100 ms to suppress joint/TF sample jitter."""
    speeds = []
    for index, (t0, p0) in enumerate(samples):
        for t1, p1 in samples[index + 1:]:
            if t1 - t0 >= 0.1:
                speeds.append(math.dist(p0, p1) / (t1 - t0))
                break
    return max(speeds) if speeds else None


def orientation_change_deg(start, end):
    if not start or not end:
        return None
    q0, q1 = start['quaternion_xyzw'], end['quaternion_xyzw']
    dot = abs(sum(a*b for a, b in zip(q0, q1)))
    return math.degrees(2.0 * math.acos(min(1.0, max(0.0, dot))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction', choices=[
        *BASE_DIRECTIONS, 'left', 'right', 'up', 'down'])
    parser.add_argument('--duration-s', type=float, default=MAX_DURATION_S)
    parser.add_argument('--output', type=Path,
                        help='Write JSON evidence to this path.')
    parser.add_argument('--operator-ready', action='store_true',
                        help='Operator has checked clearance and a working stop method.')
    parser.add_argument('--watchdog-test', action='store_true',
                        help='After the pulse, stop heartbeats and verify automatic release.')
    parser.add_argument('--watchdog-during-pulse', action='store_true',
                        help='Stop heartbeats while the bounded pulse is still active.')
    parser.add_argument('--cancel-after-s', type=float,
                        help='Inject a software cancel during the pulse.')
    parser.add_argument('--target-loss-after-s', type=float,
                        help='Inject an invalid target during the pulse.')
    args = parser.parse_args()
    stop_modes = sum((args.watchdog_test, args.watchdog_during_pulse,
                      args.cancel_after_s is not None,
                      args.target_loss_after_s is not None))
    if stop_modes > 1:
        parser.error('choose only one stop test mode')
    if not args.operator_ready:
        parser.error('stage-four motion requires --operator-ready')
    if (not math.isfinite(args.duration_s)
            or not 0.0 < args.duration_s <= MAX_DURATION_S):
        parser.error(f'--duration-s must be within (0, {MAX_DURATION_S}]')
    for option, value in (('--cancel-after-s', args.cancel_after_s),
                          ('--target-loss-after-s', args.target_loss_after_s)):
        if value is not None and (not math.isfinite(value)
                                  or not 0.0 < value < args.duration_s):
            parser.error(f'{option} must be within (0, --duration-s)')

    rclpy.init()
    node = DirectionPulse()
    lease_id = ''
    outcome = 'failed_before_acquire'
    detail = ''
    start_tcp = pulse_end_tcp = end_tcp = settled_tcp = None
    start_target = end_target = None
    start_joints = end_joints = None
    acquired_at = released_at = None
    watchdog_release_delay_s = None
    try:
        if not node.lease_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('control_lease_service_unavailable')
        if not node.wait_until(lambda: node.latest_joints is not None, 5.0):
            raise RuntimeError('joint_states_unavailable')
        if not node.wait_until(
                lambda: node.control_state == 'monitoring_manual_ready', 5.0):
            raise RuntimeError(f'control_not_idle:{node.control_state}')
        if not node.wait_until(node.target_fresh, 5.0):
            raise RuntimeError('fresh_unique_red_target_unavailable')
        if not node.wait_until(
                lambda: node.sample_tcp() is not None, 5.0):
            raise RuntimeError('tcp_tf_unavailable')
        quaternion = None
        camera_tf_deadline = time.monotonic() + 5.0
        while quaternion is None and time.monotonic() < camera_tf_deadline:
            try:
                quaternion = node.camera_quaternion()
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.05)
        if quaternion is None:
            raise RuntimeError('camera_tf_unavailable_or_stale')
        node.check_scene()
        if not node.wait_until(node.target_fresh, 5.0):
            raise RuntimeError('fresh_unique_red_target_unavailable_after_scene_check')
        if args.direction in BASE_DIRECTIONS:
            node.command_xy = BASE_DIRECTIONS[args.direction]
        else:
            node.command_xy = command_for_side(
                args.direction, node.latest_target, quaternion)
        acquired = node.call_lease(
            ControlLease.Request.ACQUIRE,
            heartbeat_timeout_s=(0.4 if args.watchdog_during_pulse else 0.5))
        if not acquired.success:
            raise RuntimeError(f'control_acquire_rejected:{acquired.message}')
        lease_id = acquired.lease_id
        acquired_at = time.time()
        outcome = 'lease_acquired'
        node.run_window(0.1 if args.watchdog_during_pulse else 0.5,
                        lease_id)
        for _ in range(50):
            if (node.target_fresh()
                    and node.target_source_age_s + time.monotonic()
                    - node.target_receive_s <= 0.25):
                break
            node.run_window(0.1, lease_id)
        else:
            raise RuntimeError('fresh_red_target_unavailable_before_pulse')
        start_tcp = node.sample_tcp()
        start_joints = node.latest_joints
        start_target = node.latest_target
        if args.watchdog_during_pulse:
            try:
                node.run_window(args.duration_s, lease_id, moving=True,
                                renew_heartbeat=False)
            except RuntimeError as error:
                if not str(error).startswith('control_state_changed:'):
                    raise
            pulse_end_tcp = node.sample_tcp()
            if not node.wait_until(
                    lambda: node.control_state == 'monitoring_manual_ready',
                    1.0):
                raise RuntimeError(
                    f'watchdog_did_not_release:{node.control_state}')
            watchdog_release_delay_s = (
                time.monotonic() - node.pulse_started_at)
            outcome = 'watchdog_stopped'
        else:
            injected = ('cancel' if args.cancel_after_s is not None else
                        'target_loss' if args.target_loss_after_s is not None
                        else None)
            try:
                node.run_window(
                    args.duration_s, lease_id, moving=True,
                    inject_event=injected,
                    inject_after_s=(args.cancel_after_s
                                    if injected == 'cancel' else
                                    args.target_loss_after_s))
            except RuntimeError as error:
                expected = ('test_cancel_requested' if injected == 'cancel'
                            else 'red_target_lost:test_injected_target_loss'
                            if injected == 'target_loss' else None)
                if expected is None or not str(error).startswith(expected):
                    raise
                outcome = f'{injected}_stopped'
            pulse_end_tcp = node.sample_tcp()
        if args.watchdog_test:
            stopped_heartbeats_at = time.monotonic()
            # The moving window has already sent zero Twist. Continue zero
            # output but intentionally stop renewing this lease.
            if not node.wait_until(
                    lambda: node.control_state == 'monitoring_manual_ready',
                    1.5):
                raise RuntimeError(
                    f'heartbeat_watchdog_failed:{node.control_state}')
            watchdog_release_delay_s = (
                time.monotonic() - stopped_heartbeats_at)
        elif not args.watchdog_during_pulse:
            node.run_window(0.5, lease_id)
        if outcome not in ('watchdog_stopped', 'cancel_stopped',
                           'target_loss_stopped'):
            outcome = 'pulse_complete'
    except (Exception, KeyboardInterrupt) as error:
        detail = f'{type(error).__name__}:{error}'
    finally:
        if lease_id:
            try:
                if watchdog_release_delay_s is None:
                    node.run_window(0.5, lease_id)
                else:
                    for _ in range(10):
                        node.publish_twist(0.0)
                        rclpy.spin_once(node, timeout_sec=0.02)
            except Exception as error:
                detail += f';zero_window_error:{type(error).__name__}:{error}'
            try:
                release = node.call_lease(
                    ControlLease.Request.RELEASE, lease_id)
                if not release.success:
                    raise RuntimeError(
                        f'control_release_rejected:{release.message}')
                released_at = time.time()
            except Exception as error:
                detail += f';release_error:{type(error).__name__}:{error}'
        node.wait_until(
            lambda: node.control_state == 'monitoring_manual_ready', 3.0)
        end_tcp = node.sample_tcp()
        end_joints = node.latest_joints
        end_target = node.latest_target if node.target_fresh() else None
        settle_deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        settled_tcp = node.sample_tcp()
        tcp_delta = ([b-a for a, b in zip(
            start_tcp['xyz_m'], end_tcp['xyz_m'])]
            if start_tcp and end_tcp else None)
        verdict = pulse_verdict(
            node.command_xy, args.duration_s, tcp_delta,
            servo_targets=len(node.servo_trace),
            controller_outputs=sum(
                sample['output'] is not None
                for sample in node.controller_trace),
            nonzero_count=node.nonzero_command_count)
        improved = None
        if (args.direction not in BASE_DIRECTIONS
                and start_target and end_target):
            improved = error_improved(
                args.direction, start_target['error_px'],
                end_target['error_px'])
        angle_change = orientation_change_deg(start_tcp, end_tcp)
        pose_drift_ok = (
            tcp_delta is not None and angle_change is not None
            and abs(tcp_delta[2]) <= 0.003
            and angle_change <= 5.0)
        if outcome == 'pulse_complete':
            outcome = ('passed' if verdict == 'direction_confirmed'
                       and released_at is not None
                       and node.control_state == 'monitoring_manual_ready'
                       and pose_drift_ok
                       and (improved is None or improved)
                       else 'review_required')
        elif outcome in ('watchdog_stopped', 'cancel_stopped',
                         'target_loss_stopped'):
            outcome = (
                'passed' if (outcome != 'watchdog_stopped' or
                             (watchdog_release_delay_s is not None and
                              watchdog_release_delay_s < args.duration_s + 0.3))
                and node.pulse_wall_duration_s < args.duration_s - 0.02
                and node.nonzero_command_count > 0
                and len(node.servo_trace) > 0
                and released_at is not None
                and node.control_state == 'monitoring_manual_ready'
                and tcp_delta is not None
                and math.hypot(*tcp_delta[:2]) <= 0.005
                and settled_tcp is not None and end_tcp is not None
                and math.dist(settled_tcp['xyz_m'], end_tcp['xyz_m'])
                    <= 0.0005
                else 'review_required')
        report = {
            'direction': args.direction, 'outcome': outcome,
            'test_mode': ('watchdog_during_pulse' if args.watchdog_during_pulse
                          else 'watchdog_after_pulse' if args.watchdog_test
                          else 'software_cancel' if args.cancel_after_s is not None
                          else 'injected_target_loss' if args.target_loss_after_s is not None
                          else 'direction'),
            'detail': detail, 'motion_verdict': verdict,
            'image_error_improved': improved,
            'command_xy_mps': list(node.command_xy),
            'duration_s': args.duration_s,
            'nominal_displacement_m':
                math.hypot(*node.command_xy) * args.duration_s,
            'actual_pulse_wall_s': node.pulse_wall_duration_s,
            'pulse_start_elapsed_s': (
                node.pulse_started_at - node.tcp_samples[0][0]
                if node.tcp_samples and hasattr(node, 'pulse_started_at')
                else None),
            'pulse_end_elapsed_s': (
                node.pulse_ended_at - node.tcp_samples[0][0]
                if node.tcp_samples and hasattr(node, 'pulse_ended_at')
                else None),
            'nonzero_twist_count': node.nonzero_command_count,
            'heartbeat_count': node.heartbeat_count,
            'heartbeat_latencies_s': node.heartbeat_latencies_s,
            'servo_status_history': node.status_history,
            'scene_object_ids': node.scene_objects,
            'tcp_tf_age_s_at_end': node.tcp_tf_age_s,
            'camera_tf_age_s_at_start': node.camera_tf_age_s,
            'control_state_after': node.control_state,
            'lease_acquired_wall_s': acquired_at,
            'lease_released_wall_s': released_at,
            'watchdog_release_delay_s': watchdog_release_delay_s,
            'start_target': start_target, 'end_target': end_target,
            'start_tcp': start_tcp, 'end_tcp': end_tcp,
            'pulse_end_tcp': pulse_end_tcp,
            'settled_tcp': settled_tcp,
            'post_zero_tcp_delta_xyz_m': (
                [b-a for a, b in zip(
                    pulse_end_tcp['xyz_m'], end_tcp['xyz_m'])]
                if pulse_end_tcp and end_tcp else None),
            'post_release_tcp_delta_xyz_m': (
                [b-a for a, b in zip(
                    end_tcp['xyz_m'], settled_tcp['xyz_m'])]
                if end_tcp and settled_tcp else None),
            'tcp_delta_xyz_m': tcp_delta,
            'actual_along_command_m': (
                sum(tcp_delta[i] * node.command_xy[i] for i in range(2)) /
                math.hypot(*node.command_xy)
                if tcp_delta is not None and math.hypot(*node.command_xy) > 0
                else None),
            'orientation_change_deg': angle_change,
            'pose_drift_ok': pose_drift_ok,
            'measured_max_tcp_speed_mps':
                max_measured_speed(node.tcp_samples),
            'start_joints_rad': start_joints,
            'end_joints_rad': end_joints,
            'servo_trajectory_count': len(node.servo_trace),
            'controller_state_count': len(node.controller_trace),
            'servo_joint_target_trace': summarize_trace(
                node.servo_trace, start_joints),
            'controller_output_trace': summarize_trace(
                [sample['output'] for sample in node.controller_trace
                 if sample['output'] is not None], start_joints),
            'controller_feedback_trace': summarize_trace(
                [sample['feedback'] for sample in node.controller_trace
                 if sample['feedback'] is not None], start_joints),
            'tcp_sample_count': len(node.tcp_samples),
            'tcp_distinct_positions': len({position
                                           for _, position in node.tcp_samples}),
            'tcp_position_changes': [
                {'elapsed_s': t - node.tcp_samples[0][0], 'xyz_m': list(p)}
                for index, (t, p) in enumerate(node.tcp_samples)
                if index == 0 or p != node.tcp_samples[index - 1][1]
            ],
        }
        payload = json.dumps(report, indent=2, sort_keys=True) + '\n'
        print(payload, end='')
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(payload, encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
        if outcome != 'passed':
            raise SystemExit(1)


if __name__ == '__main__':
    main()
