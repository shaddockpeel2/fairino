"""Record a bounded read-only pose audit and a current annotated image."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from fr3c_camera.rgbd import image_array, stamp_seconds


def percentiles(values):
    if not values:
        return None
    return dict(zip(('p50', 'p95', 'max'), np.r_[np.percentile(values, [50, 95]), np.max(values)].tolist()))


class PoseAudit(Node):
    def __init__(self, output):
        super().__init__('target_pose_audit')
        self.output = output
        self.rows = []
        self.pose_stamps = []
        self.snapshot = None
        self.stream = (output / 'samples.jsonl').open('w')
        self.create_subscription(String, '/target_follow/quality', self.quality, 10)
        self.create_subscription(PoseStamped, '/target_follow/tag_pose', self.pose, 10)
        self.create_subscription(Image, '/target_follow/image_annotated', self.image, qos_profile_sensor_data)

    def quality(self, message):
        row = json.loads(message.data)
        row['audit_receive_monotonic_s'] = time.monotonic()
        self.rows.append(row)
        self.stream.write(json.dumps(row, allow_nan=False) + '\n')

    def pose(self, message):
        self.pose_stamps.append(stamp_seconds(message))

    def image(self, message):
        self.snapshot = message

    def report(self, duration):
        rows = self.rows
        processed = [row for row in rows if 'stamp_s' in row]
        valid = [row for row in processed if row['status'] == 'valid']
        span = (processed[-1]['audit_receive_monotonic_s'] - processed[0]['audit_receive_monotonic_s']
                if len(processed) > 1 else 0)
        report = dict(
            duration_s=duration, quality_messages=len(rows), processed_frames=len(processed),
            valid_frames=len(valid), published_pose_messages=len(self.pose_stamps),
            detection_fraction=len(valid) / len(processed) if processed else 0,
            processed_fps=(len(processed) - 1) / span if span > 0 else 0,
            status_counts=dict(Counter(row['status'] for row in rows)),
            pose_timestamp_regressions=int(np.sum(np.diff(self.pose_stamps) <= 0)),
            motion_commands_enabled=any(row['motion_commands_enabled'] for row in rows),
            orientation_ambiguous_frames=sum(row.get('orientation_ambiguous', False) for row in valid),
        )
        for metric in ('processing_ms', 'callback_ms', 'image_age_ms', 'reprojection_error_px', 'min_edge_px'):
            report[metric] = percentiles([row[metric] for row in valid if metric in row])
        if valid:
            positions = np.array([row['position_m'] for row in valid])
            centre = np.median(positions, axis=0)
            report.update(dictionary=valid[-1]['dictionary'], target_id=valid[-1]['target_id'],
                          marker_size_m=valid[-1]['marker_size_m'], frame_id=valid[-1]['frame_id'],
                          median_position_m=centre.tolist(),
                          position_deviation_from_median_mm=percentiles((np.linalg.norm(positions-centre, axis=1)*1000).tolist()),
                          position_axis_std_mm=(np.std(positions, axis=0)*1000).tolist(),
                          latest_pose=valid[-1])
            # q and -q describe the same rotation; compare their absolute inner products.
            q = np.array([row['quaternion_xyzw'] for row in valid])
            angle = np.degrees(2*np.arccos(np.clip(np.abs(q @ q[0]), 0, 1)))
            report['orientation_deviation_from_first_deg'] = percentiles(angle.tolist())
        report['spread_note'] = 'Position/orientation spread includes real target motion; it is noise only if target and camera remained stationary.'
        if self.snapshot is not None:
            cv2.imwrite(str(self.output / 'annotated.png'), image_array(self.snapshot).copy())
            report['snapshot_stamp_s'] = stamp_seconds(self.snapshot)
        self.stream.close()
        (self.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=30.0)
    parser.add_argument('--output', type=Path, required=True)
    options, ros_args = parser.parse_known_args(args)
    if not np.isfinite(options.duration_s) or options.duration_s <= 0:
        parser.error('--duration-s must be positive')
    options.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = PoseAudit(options.output)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < options.duration_s:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.report(time.monotonic() - start)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['valid_frames']:
        raise SystemExit(1)
