"""Pair live color-top observations with stationary TCP contact measurements."""

import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


SCHEMA_VERSION = 1
SESSION_TYPE = 'color_top_tcp_contact'


def _now_iso():
    return datetime.now().astimezone().isoformat()


def _new_session(base_frame, tcp_frame, status_topic):
    return {
        'schema_version': SCHEMA_VERSION,
        'type': SESSION_TYPE,
        'created_at': _now_iso(),
        'base_frame': base_frame,
        'tcp_frame': tcp_frame,
        'status_topic': status_topic,
        'samples': {},
    }


def _load_session(path):
    if not path.exists():
        return None
    try:
        document = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(f'invalid JSON session: {path}') from exc
    if document.get('schema_version') != SCHEMA_VERSION or \
            document.get('type') != SESSION_TYPE:
        raise ValueError(f'not a {SESSION_TYPE} session: {path}')
    if not isinstance(document.get('samples'), dict):
        raise ValueError('session samples must be an object')
    return document


def _atomic_write(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _position_statistics(points_m):
    points = np.asarray(points_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or \
            not np.isfinite(points).all():
        raise ValueError('positions must be a non-empty Nx3 finite array')
    median = np.median(points, axis=0)
    deviations = np.linalg.norm(points - median, axis=1)
    return {
        'median_xyz_m': median.tolist(),
        'mean_xyz_m': np.mean(points, axis=0).tolist(),
        'std_xyz_mm': (np.std(points, axis=0) * 1000.0).tolist(),
        'range_xyz_mm': (np.ptp(points, axis=0) * 1000.0).tolist(),
        'max_deviation_mm': float(np.max(deviations) * 1000.0),
    }


def _comparison(color_top, tcp_contact):
    top = np.asarray(color_top['median_xyz_m'], dtype=float)
    tcp = np.asarray(tcp_contact['median_xyz_m'], dtype=float)
    delta_mm = (tcp - top) * 1000.0
    return {
        'tcp_minus_color_top_xyz_mm': delta_mm.tolist(),
        'xy_distance_mm': float(np.linalg.norm(delta_mm[:2])),
        'z_difference_mm': float(delta_mm[2]),
        'distance_3d_mm': float(np.linalg.norm(delta_mm)),
    }


def _quaternion_z_axis(quaternion_xyzw):
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=float)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError('invalid TCP quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    ])


def _axis_angle_deg(first, second):
    dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def _rpy_deg(quaternion_xyzw):
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    pitch_term = float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw]).tolist()


class ColorTopReader(Node):
    def __init__(self, status_topic):
        super().__init__('surface_color_top_capture')
        self.status_topic = status_topic
        self.messages = []
        self.last_rejection = 'waiting_for_status'
        self.create_subscription(String, status_topic, self._callback, 10)

    def _callback(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.last_rejection = 'malformed_status_json'
            return
        self.messages.append(status)

    def capture(self, count, timeout_s, expected_color, max_age_s):
        accepted = []
        stamps = set()
        start_ros_s = self.get_clock().now().nanoseconds / 1e9
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and len(accepted) < count and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            pending, self.messages = self.messages, []
            for status in pending:
                stamp = status.get('sample_stamp')
                center = status.get('center_base_m')
                now_ros_s = self.get_clock().now().nanoseconds / 1e9
                if not status.get('valid') or not status.get('stable'):
                    self.last_rejection = str(status.get('reason', 'status_not_valid'))
                    continue
                if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                    self.last_rejection = 'missing_sample_stamp'
                    continue
                stamp_key = round(float(stamp), 9)
                if stamp_key in stamps:
                    self.last_rejection = 'duplicate_heartbeat_sample'
                    continue
                age_s = now_ros_s - float(stamp)
                if float(stamp) < start_ros_s - 0.10 or not -0.10 <= age_s <= max_age_s:
                    self.last_rejection = f'stale_sample_age_{age_s:.3f}s'
                    continue
                try:
                    xyz = np.asarray(center, dtype=float)
                except (TypeError, ValueError):
                    self.last_rejection = 'invalid_center_base_m'
                    continue
                if xyz.shape != (3,) or not np.isfinite(xyz).all():
                    self.last_rejection = 'invalid_center_base_m'
                    continue
                color = status.get('color')
                if expected_color and color != expected_color:
                    self.last_rejection = f'expected_{expected_color}_got_{color}'
                    continue
                stamps.add(stamp_key)
                accepted.append(status)
                if len(accepted) == count:
                    break
        if len(accepted) != count:
            if not accepted and self.count_publishers(self.status_topic) == 0:
                raise RuntimeError(
                    f'no publisher on {self.status_topic}; start the perception '
                    'stack or pass the actual topic with --status-topic')
            raise RuntimeError(
                f'only {len(accepted)}/{count} fresh stable color-top samples; '
                f'last status/rejection: {self.last_rejection}')
        return accepted


class TcpContactReader(Node):
    def __init__(self, base_frame, tcp_frame):
        super().__init__('surface_tcp_contact_capture')
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)

    def capture(self, count, timeout_s, max_age_s):
        accepted = []
        stamps = set()
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and len(accepted) < count and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.08)
            try:
                transform = self.buffer.lookup_transform(
                    self.base_frame, self.tcp_frame, Time(),
                    timeout=Duration(seconds=0.02))
            except Exception:
                continue
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if stamp_ns == 0 or stamp_ns in stamps or not -0.10 <= age_s <= max_age_s:
                continue
            p = transform.transform.translation
            q = transform.transform.rotation
            xyz = np.asarray([p.x, p.y, p.z], dtype=float)
            quaternion = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
            if not np.isfinite(xyz).all() or not np.isfinite(quaternion).all():
                raise RuntimeError('non-finite TCP transform')
            accepted.append({
                'tf_stamp_ns': stamp_ns,
                'xyz_m': xyz.tolist(),
                'quaternion_xyzw': quaternion.tolist(),
                'age_s_at_capture': float(age_s),
            })
            stamps.add(stamp_ns)
        if len(accepted) != count:
            raise RuntimeError(
                f'only {len(accepted)}/{count} fresh TCP transforms; check live '
                f'/joint_states and {self.base_frame} -> {self.tcp_frame} TF')
        return accepted


def _capture_color(options):
    rclpy.init()
    node = ColorTopReader(options.status_topic)
    try:
        statuses = node.capture(
            options.count, options.timeout, options.expected_color,
            options.max_age)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    positions = [status['center_base_m'] for status in statuses]
    result = _position_statistics(positions)
    if result['max_deviation_mm'] > options.max_spread_mm:
        raise RuntimeError(
            f'color top moved/noisy during capture: maximum deviation '
            f'{result["max_deviation_mm"]:.2f} mm > {options.max_spread_mm:.2f} mm')
    colors = sorted({status.get('color') for status in statuses}, key=str)
    result.update({
        'captured_at': _now_iso(),
        'base_frame': options.base_frame,
        'status_topic': options.status_topic,
        'sample_count': len(statuses),
        'sample_stamps_s': [float(status['sample_stamp']) for status in statuses],
        'colors': colors,
        'top_z_spread_mm': [
            float(status['top_z_spread_m']) * 1000.0 for status in statuses
            if isinstance(status.get('top_z_spread_m'), (int, float))],
        'depth_valid_fraction': [
            float(status['depth_valid_fraction']) for status in statuses
            if isinstance(status.get('depth_valid_fraction'), (int, float))],
        'observed_short_long_m': [status.get('observed_short_long_m')
                                  for status in statuses],
        'raw_status': statuses,
    })
    return result


def _capture_tcp(options):
    rclpy.init()
    node = TcpContactReader(options.base_frame, options.tcp_frame)
    try:
        transforms = node.capture(options.count, options.timeout, options.max_age)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = _position_statistics([item['xyz_m'] for item in transforms])
    axes = [_quaternion_z_axis(item['quaternion_xyzw']) for item in transforms]
    axis_spread_deg = max(
        [_axis_angle_deg(axes[0], axis) for axis in axes[1:]] or [0.0])
    if result['max_deviation_mm'] > options.max_spread_mm:
        raise RuntimeError(
            f'TCP moved during capture: maximum deviation '
            f'{result["max_deviation_mm"]:.2f} mm > {options.max_spread_mm:.2f} mm')
    if axis_spread_deg > options.max_axis_spread_deg:
        raise RuntimeError(
            f'TCP rotated during capture: Z-axis spread {axis_spread_deg:.2f} deg '
            f'> {options.max_axis_spread_deg:.2f} deg')
    result.update({
        'captured_at': _now_iso(),
        'base_frame': options.base_frame,
        'tcp_frame': options.tcp_frame,
        'sample_count': len(transforms),
        'quaternion_xyzw': transforms[-1]['quaternion_xyzw'],
        'rpy_deg': _rpy_deg(transforms[-1]['quaternion_xyzw']),
        'tcp_z_axis_spread_deg': axis_spread_deg,
        'raw_transforms': transforms,
    })
    return result


def _save_component(options, component, result):
    path = Path(options.session).expanduser().resolve()
    lock_path = path.with_suffix(path.suffix + '.lock')
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        document = _load_session(path)
        if document is None:
            document = _new_session(
                options.base_frame, options.tcp_frame, options.status_topic)
        if document['base_frame'] != options.base_frame or \
                document['tcp_frame'] != options.tcp_frame:
            raise ValueError('session base/TCP frame names differ from requested frames')
        record = dict(document['samples'].get(options.sample, {}))
        if component in record and not options.replace:
            raise ValueError(
                f'{options.sample}.{component} already exists; use --replace to recapture')
        record[component] = result
        if 'color_top' in record and 'tcp_contact' in record:
            record['comparison'] = _comparison(
                record['color_top'], record['tcp_contact'])
        document['samples'][options.sample] = record
        document['updated_at'] = _now_iso()
        _atomic_write(path, document)
    output = {
        'captured': component,
        'sample': options.sample,
        'median_xyz_m': result['median_xyz_m'],
        'max_deviation_mm': result['max_deviation_mm'],
        'comparison': record.get('comparison'),
        'session': str(path),
    }
    print(json.dumps(output, indent=2, ensure_ascii=False))


def _summary(options):
    path = Path(options.session).expanduser().resolve()
    document = _load_session(path)
    if document is None:
        raise ValueError(f'session does not exist: {path}')
    rows = []
    complete_deltas = []
    for name, record in document['samples'].items():
        comparison = record.get('comparison')
        rows.append({
            'sample': name,
            'has_color_top': 'color_top' in record,
            'has_tcp_contact': 'tcp_contact' in record,
            **(comparison or {}),
        })
        if comparison:
            complete_deltas.append(comparison['tcp_minus_color_top_xyz_mm'])
    aggregate = None
    if complete_deltas:
        values = np.asarray(complete_deltas, dtype=float)
        aggregate = {
            'complete_pairs': len(values),
            'median_tcp_minus_color_top_xyz_mm': np.median(values, axis=0).tolist(),
            'mean_tcp_minus_color_top_xyz_mm': np.mean(values, axis=0).tolist(),
            'std_tcp_minus_color_top_xyz_mm': np.std(values, axis=0).tolist(),
            'z_range_mm': [float(np.min(values[:, 2])), float(np.max(values[:, 2]))],
        }
    print(json.dumps({
        'session': str(path),
        'samples': rows,
        'aggregate': aggregate,
    }, indent=2, ensure_ascii=False))


def _reset(options):
    path = Path(options.session).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    archive = None
    lock_path = path.with_suffix(path.suffix + '.lock')
    with lock_path.open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        if path.exists():
            stamp = datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')
            archive = path.with_name(f'{path.stem}.backup-{stamp}{path.suffix}')
            if archive.exists():
                raise RuntimeError(f'archive already exists: {archive}')
            shutil.move(path, archive)
        _atomic_write(path, _new_session(
            options.base_frame, options.tcp_frame, options.status_topic))
    print(json.dumps({
        'reset': True,
        'session': str(path),
        'archived_previous_session': str(archive) if archive else None,
    }, indent=2, ensure_ascii=False))


def _common_capture_arguments(parser):
    parser.add_argument('--session', required=True, help='JSON session path')
    parser.add_argument('--sample', required=True, help='pairing name, e.g. P01')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--tcp-frame', default='grasp_frame')
    parser.add_argument(
        '--status-topic', default='/perception/blue_cube_localizer/status')
    parser.add_argument('--count', type=int, default=8)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--max-age', type=float, default=0.60)
    parser.add_argument('--max-spread-mm', type=float, default=3.0)
    parser.add_argument('--replace', action='store_true')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Capture paired color-top and physical TCP-contact positions')
    commands = parser.add_subparsers(dest='command', required=True)
    color_parser = commands.add_parser(
        'color', help='capture fresh stable color/depth top observations')
    _common_capture_arguments(color_parser)
    color_parser.add_argument(
        '--expected-color', choices=('red', 'green', 'blue', 'yellow'))
    tcp_parser = commands.add_parser(
        'tcp', help='capture fresh stationary base-to-TCP transforms at contact')
    _common_capture_arguments(tcp_parser)
    tcp_parser.set_defaults(count=12, max_spread_mm=2.0)
    tcp_parser.add_argument('--max-axis-spread-deg', type=float, default=2.0)
    summary_parser = commands.add_parser('summary', help='summarize all captured pairs')
    summary_parser.add_argument('--session', required=True)
    reset_parser = commands.add_parser(
        'reset', help='archive the old session and start an empty one')
    reset_parser.add_argument('--session', required=True)
    reset_parser.add_argument('--base-frame', default='base_link')
    reset_parser.add_argument('--tcp-frame', default='grasp_frame')
    reset_parser.add_argument(
        '--status-topic', default='/perception/blue_cube_localizer/status')
    options = parser.parse_args(args)
    if hasattr(options, 'count') and options.count < 1:
        parser.error('--count must be at least 1')
    try:
        if options.command == 'color':
            _save_component(options, 'color_top', _capture_color(options))
        elif options.command == 'tcp':
            _save_component(options, 'tcp_contact', _capture_tcp(options))
        elif options.command == 'summary':
            _summary(options)
        else:
            _reset(options)
    except (RuntimeError, ValueError, OSError) as exc:
        parser.exit(2, f'surface_contact_capture: {exc}\n')


if __name__ == '__main__':
    main()
