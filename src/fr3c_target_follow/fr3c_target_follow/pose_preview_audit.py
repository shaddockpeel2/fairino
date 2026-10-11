"""Record the read-only pose preview and its sources. No motion commands."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import numpy as np


def preview_report(rows, duration):
    states = [row for row in rows if row['kind'] == 'preview']
    active = [row for row in states if row['status'] in ('preview', 'holding_quality')]
    observations = [row for row in rows if row['kind'] == 'observation']
    report = dict(stage=7, duration_s=duration, preview_states=len(states),
                  trusted_observations=sum(row.get('valid_orientation') is True for row in observations),
                  status_counts=dict(Counter(row['status'] for row in states)),
                  teach_count=max((row['teach_count'] for row in states), default=0),
                  motion_commands_enabled=any(row.get('motion_commands_enabled', False) for row in states),
                  orientation_control_enabled=any(row.get('orientation_control_enabled', False) for row in states),
                  hardware_servo_commands_sent=0, command_scope='this_read_only_preview',
                  reference_kind='one_step_from_actual_tcp',
                  note='Preview only. Desired/error spans include target/robot motion and are not accuracy.')
    if active:
        report.update(
            desired_position_axis_span_mm=(np.ptp([row['desired_tcp_m'] for row in active], axis=0)*1000).tolist(),
            max_rotation_error_deg=max(row['rotation_error_deg'] for row in active),
            max_preview_speed_mm_s=max(np.linalg.norm(row['velocity_m_s'])*1000 for row in active),
            max_preview_angular_speed_deg_s=max(np.linalg.norm(row['angular_velocity_deg_s']) for row in active),
            rotation_mapping_gain=active[-1]['rotation_mapping_gain'])
    if states:
        report['latest_state'] = states[-1]
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=90.)
    options, ros_args = parser.parse_known_args(args)
    if not np.isfinite(options.duration_s) or options.duration_s <= 0:
        parser.error('duration must be positive')
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    options.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = Node('pose_preview_audit')
    rows = []
    stream = (options.output/'samples.jsonl').open('w')

    def save(kind, message):
        record = dict(json.loads(message.data), kind=kind, audit_receive_s=time.monotonic())
        rows.append(record)
        stream.write(json.dumps(record, allow_nan=False)+'\n')

    for topic, kind in (('/target_follow/pose_preview/state', 'preview'),
                        ('/target_follow/pose_observation', 'observation'),
                        ('/target_follow/quality', 'quality'),
                        ('/target_follow/robot_feedback', 'feedback')):
        node.create_subscription(String, topic, lambda message, kind=kind: save(kind, message), 50)
    began = printed = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-began < options.duration_s:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic()-printed >= 1.:
                last = next((row for row in reversed(rows) if row['kind'] == 'preview'), None)
                if last:
                    gap = last.get('rotation_error_deg')
                    angle = f'{gap:.1f} deg' if gap is not None else 'unavailable'
                    print(f"Pose preview: {last['status']} | recorded={last['offset_recorded']} | "
                          f"TCP angle gap={angle} | MOTION OFF", flush=True)
                printed = time.monotonic()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        duration = time.monotonic()-began
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report = preview_report(rows, duration)
    (options.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    if not report['preview_states']:
        raise SystemExit(1)
