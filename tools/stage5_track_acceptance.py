#!/usr/bin/env python3
"""Bounded real Action client and JSON evidence recorder; Ctrl-C cancels goal."""
import argparse
import json
from pathlib import Path
import time
import subprocess
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from fairino_msgs.action import TrackTarget2D
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Int8


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--cancel-after', type=float, default=0.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 180:
        parser.error('duration must be in (0, 180] seconds')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('stage5_acceptance_client')
    client = ActionClient(node, TrackTarget2D, '/fr3c_visual_servo/track_2d')
    started = time.monotonic()
    record = {'duration_s': args.duration, 'feedback': [], 'commands': [],
              'control_states': [], 'servo_states': [], 'status': [], 'result': None}
    def elapsed():
        return time.monotonic() - started
    def state(msg):
        if not record['control_states'] or record['control_states'][-1][1] != msg.data:
            record['control_states'].append([elapsed(), msg.data])
    def feedback(msg):
        f = msg.feedback
        record['feedback'].append(dict(t=elapsed(), phase=f.phase, detail=f.detail,
            error_px=[f.error_x_px, f.error_y_px],
            command_xy=[f.command_x_mps, f.command_y_mps], age_s=f.observation_age_s))
    def command(msg):
        v = msg.twist
        record['commands'].append([elapsed(), v.linear.x, v.linear.y, v.linear.z,
                                    v.angular.x, v.angular.y, v.angular.z])
    node.create_subscription(String, '/fairino/control_state', state,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TwistStamped, '/servo_node/delta_twist_cmds', command, 100)
    node.create_subscription(Int8, '/servo_node/status',
        lambda msg: record['servo_states'].append([elapsed(), msg.data]), 100)
    node.create_subscription(String, '/fr3c_visual_servo/dry_run/status',
        lambda msg: record['status'].append([elapsed(), json.loads(msg.data)]), 100)
    handle = None
    result_future = None
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('action_server_unavailable')
        if len(node.get_publishers_info_by_topic('/servo_node/delta_twist_cmds')) != 1:
            raise RuntimeError('expected_exactly_one_servo_command_publisher')
        scene = subprocess.run(
            [sys.executable, str(Path(__file__).with_name('stage4_ground_scene.py')),
             '--require-servo-sync'], capture_output=True, text=True, timeout=30)
        record['scene_preflight'] = scene.stdout + scene.stderr
        if scene.returncode:
            raise RuntimeError('scene_preflight_failed')
        future = client.send_goal_async(TrackTarget2D.Goal(
            color='red', max_duration_s=args.duration), feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError('goal_response_timeout')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('goal_rejected')
        result_future = handle.get_result_async()
        goal_start = time.monotonic()
        canceled = False
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.02)
            age = time.monotonic() - goal_start
            if args.cancel_after and age >= args.cancel_after and not canceled:
                handle.cancel_goal_async()
                record['cancel_requested_s'] = elapsed()
                canceled = True
            if age > args.duration + 15:
                raise RuntimeError('action_result_timeout')
    except (KeyboardInterrupt, Exception) as error:
        record['client_error'] = str(error) or type(error).__name__
    finally:
        if handle and handle.accepted and result_future and not result_future.done():
            handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=12)
        if result_future and result_future.done():
            response = result_future.result()
            r = response.result
            record['result'] = dict(status=response.status, success=r.success,
                error_code=r.error_code, message=r.message, elapsed_s=r.elapsed_s,
                final_error_px=[r.final_error_x_px, r.final_error_y_px])
        until = time.monotonic() + 1
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        record['final_control_state'] = (record['control_states'][-1][1]
                                         if record['control_states'] else 'unknown')
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(record, indent=2))
        print(json.dumps({k: record[k] for k in ('result', 'final_control_state')}))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if record['result'] and record['final_control_state'] == 'monitoring_manual_ready' else 1


if __name__ == '__main__':
    raise SystemExit(main())
