"""Record/replay paired base orientation, diagnostic state, and robot feedback."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .audit import percentiles
from .follow_geometry import rotation_from_quaternion
from .marker_pose import rotation_distance_deg


def orientation_report(rows, scenario='unspecified'):
    observations = [r for r in rows if r.get('kind') == 'observation']
    pairs = [r for r in observations if r.get('position_m') is not None]
    trusted = [r for r in pairs if r.get('valid_orientation') is True]
    robot = [r for r in rows if r.get('kind') == 'flange']
    report = dict(stage=6, scenario=scenario, paired_observations=len(pairs),
                  trusted_orientation_observations=len(trusted),
                  orientation_available_fraction=len(trusted)/len(pairs) if pairs else 0.,
                  paired_status_counts=dict(Counter(r['status'] for r in pairs)),
                  observation_status_counts=dict(Counter(r['status'] for r in observations)),
                  state_status_counts=dict(Counter(r['status'] for r in rows if r.get('kind') == 'state')),
                  motion_commands_enabled=any(r.get('motion_commands_enabled', False) for r in rows),
                  orientation_control_enabled=any(r.get('orientation_control_enabled', False) for r in rows),
                  hardware_servo_commands_sent=0,
                  command_scope='this_read_only_pipeline',
                  spread_note=('Contains real movement unless both target and robot were stationary. '
                               'Spread is not absolute pose accuracy; a stable trajectory does not prove the correct PnP branch.'))
    for key in ('orientation_delta_deg', 'angular_speed_deg_s'):
        report[key] = percentiles([r[key] for r in pairs if r.get(key) is not None])
    for key in ('candidate_margin_px', 'candidate_rotation_distance_deg', 'min_edge_px', 'reprojection_error_px'):
        report[key] = percentiles([r['quality'][key] for r in pairs if r.get('quality', {}).get(key) is not None])
    for name, selection in (('all_base', pairs), ('trusted_base', trusted), ('robot_flange', robot)):
        if not selection:
            report[name] = None
            continue
        first = rotation_from_quaternion(selection[0]['quaternion_xyzw'])
        rotations = [rotation_from_quaternion(r['quaternion_xyzw']) for r in selection]
        relative = np.array([np.degrees(cv2.Rodrigues(r @ first.T)[0].reshape(3)) for r in rotations])
        positions = np.array([r['position_m'] for r in selection])
        centre = np.median(positions, axis=0)
        report[name] = dict(
            rotation_deviation_from_first_deg=percentiles([rotation_distance_deg(first, r) for r in rotations]),
            relative_rotation_vector_axis_span_deg=np.ptp(relative, axis=0).tolist(),
            start_end_rotation_vector_deg=relative[-1].tolist(),
            position_deviation_from_median_mm=percentiles((np.linalg.norm(positions-centre, axis=1)*1000).tolist()),
            position_axis_span_mm=(np.ptp(positions, axis=0)*1000).tolist())
    if pairs:
        report['latest_paired_observation'] = pairs[-1]
    states = [r for r in rows if r.get('kind') == 'state']
    if states:
        report['latest_state'] = states[-1]
        report['pairing'] = states[-1].get('pairing')
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=30.)
    parser.add_argument('--scenario', choices=['unspecified', 'static', 'target_rotation', 'robot_rotation'], default='unspecified')
    parser.add_argument('--replay', type=Path)
    options, ros_args = parser.parse_known_args(args)
    if not np.isfinite(options.duration_s) or options.duration_s <= 0:
        parser.error('duration must be positive')
    options.output.mkdir(parents=True, exist_ok=True)
    rows = []
    if options.replay:
        rows = [json.loads(line) for line in options.replay.read_text().splitlines() if line.strip()]
        duration = None
    else:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
        from .orientation_node import exact_stamp

        rclpy.init(args=ros_args)
        node = Node('orientation_audit')
        stream = (options.output/'samples.jsonl').open('w')

        def save(kind, row):
            record = dict(row, kind=kind, audit_receive_s=time.monotonic())
            rows.append(record)
            stream.write(json.dumps(record, allow_nan=False)+'\n')

        node.create_subscription(String, '/target_follow/pose_observation',
                                 lambda m: save('observation', json.loads(m.data)), 50)
        node.create_subscription(String, '/target_follow/orientation_state',
                                 lambda m: save('state', json.loads(m.data)), 50)
        node.create_subscription(String, '/target_follow/quality',
                                 lambda m: save('quality', json.loads(m.data)), 50)
        # Retain controller coordinate context for fixed-target robot-motion
        # diagnostics, using the existing read-only source.
        node.create_subscription(String, '/target_follow/robot_feedback',
                                 lambda m: save('feedback', json.loads(m.data)), 50)

        def flange(message):
            p, q = message.pose.position, message.pose.orientation
            save('flange', dict(stamp=exact_stamp(message.header.stamp), frame_id=message.header.frame_id,
                               position_m=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w]))

        node.create_subscription(PoseStamped, '/target_follow/flange_pose', flange, 50)
        began = last_print = time.monotonic()
        try:
            while rclpy.ok() and time.monotonic()-began < options.duration_s:
                rclpy.spin_once(node, timeout_sec=.1)
                if time.monotonic()-last_print >= 1:
                    recent = next((r for r in reversed(rows) if r['kind'] == 'state'), None)
                    print('Orientation: '+(recent['status'] if recent else 'waiting_for_monitor'), flush=True)
                    last_print = time.monotonic()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            duration = time.monotonic()-began
            stream.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    report = orientation_report(rows, options.scenario)
    report.update(duration_s=duration, replay_source=str(options.replay) if options.replay else None)
    (options.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    if not report['paired_observations']:
        raise SystemExit(1)
