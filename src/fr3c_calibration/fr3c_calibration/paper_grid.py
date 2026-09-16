"""Read-only TCP corner capture and live RViz A4 grid publishing."""

import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .paper_grid_geometry import CORNER_NAMES, axis_angle_deg, make_grid, quaternion_z_axis


TOPIC = '/paper_grid/markers'
COLORS = {'A': (0.10, 0.35, 1.00), 'B': (1.00, 0.88, 0.05),
          'C': (0.95, 0.13, 0.13), 'D': (0.10, 0.78, 0.22)}
LABEL_COLORS = {'A': (1., 1., 1.), 'B': (0., 0., 0.),
                'C': (1., 1., 1.), 'D': (1., 1., 1.)}


def _load_session(path):
    if not path.exists():
        return None
    try:
        document = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(
            f'cannot read session {path}: empty or invalid JSON; '
            'use "paper_grid reset --session PATH" to archive it and start again') from exc
    if document.get('schema_version') != 1 or document.get('type') != 'a4_tcp_grid':
        raise ValueError(f'not an A4 TCP grid session: {path}')
    if not isinstance(document.get('corners'), dict) or not set(document['corners']) <= set(CORNER_NAMES):
        raise ValueError('invalid corner records in session')
    return document


def _new_session(base_frame, tcp_frame):
    return {
        'schema_version': 1, 'type': 'a4_tcp_grid',
        'base_frame': base_frame, 'tcp_frame': tcp_frame,
        'corner_order': list(CORNER_NAMES), 'corners': {}, 'grid': None,
        'projection_z_m': 0.0,
    }


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


class TcpReader(Node):
    def __init__(self, base_frame, tcp_frame):
        super().__init__('a4_tcp_corner_capture')
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self.buffer = Buffer(cache_time=Duration(seconds=5))
        self.listener = TransformListener(self.buffer, self)

    def capture_stationary(self, count=12, timeout_s=12.0):
        samples = []
        stamps = set()
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and len(samples) < count and time.monotonic() < deadline:
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
            if stamp_ns == 0 or stamp_ns in stamps or not -0.1 <= age_s <= 0.5:
                continue
            p = transform.transform.translation
            q = transform.transform.rotation
            xyz = np.asarray([p.x, p.y, p.z], dtype=float)
            quat = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
            if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quat)):
                raise RuntimeError('non-finite TCP transform')
            samples.append((xyz, quat, stamp_ns))
            stamps.add(stamp_ns)
        if len(samples) != count:
            raise RuntimeError(f'only {len(samples)}/{count} fresh TCP samples; '
                               'check live /joint_states and base_link -> grasp_frame TF')
        xyz = np.asarray([item[0] for item in samples])
        center = np.median(xyz, axis=0)
        spread_mm = float(np.max(np.linalg.norm(xyz - center, axis=1)) * 1000)
        axes = [quaternion_z_axis(item[1]) for item in samples]
        axis_spread = max(axis_angle_deg(axes[0], axis) for axis in axes[1:])
        if spread_mm > 2.0 or axis_spread > 2.0:
            raise RuntimeError(f'TCP moved during capture: position spread '
                               f'{spread_mm:.2f} mm, Z-axis spread {axis_spread:.2f} deg')
        return {
            'xyz_m': center.tolist(),
            'quaternion_xyzw': samples[-1][1].tolist(),
            'tf_stamp_ns': samples[-1][2],
            'captured_at': datetime.now().astimezone().isoformat(),
            'sample_count': count,
            'position_spread_mm': spread_mm,
            'tcp_z_spread_deg': axis_spread,
        }


def capture(options):
    path = Path(options.session).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(path.suffix + '.lock').open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        document = _load_session(path)
        if document is None:
            document = _new_session(options.base_frame, options.tcp_frame)
        if (document['base_frame'], document['tcp_frame']) != (options.base_frame, options.tcp_frame):
            raise ValueError('session frame names differ from requested frames')
        if options.corner in document['corners'] and not options.replace:
            raise ValueError(f'{options.corner} already captured; use --replace to recapture')

        rclpy.init()
        node = TcpReader(options.base_frame, options.tcp_frame)
        try:
            record = node.capture_stationary()
        finally:
            node.destroy_node()
            rclpy.shutdown()

        updated = dict(document['corners'])
        updated[options.corner] = record
        if len(updated) > 1:
            first = updated[next(name for name in CORNER_NAMES if name in updated)]
            deviation = axis_angle_deg(quaternion_z_axis(first['quaternion_xyzw']),
                                       quaternion_z_axis(record['quaternion_xyzw']))
            if deviation > 8.0:
                raise ValueError(f'TCP Z differs from first corner by {deviation:.1f} deg')
        grid = make_grid(updated, document.get('projection_z_m')) if len(updated) == 4 else None
        document['corners'] = updated
        document['grid'] = grid
        _atomic_write(path, document)
    print(json.dumps({
        'captured': options.corner, 'tcp_xyz_m': record['xyz_m'],
        'stationary_spread_mm': record['position_spread_mm'],
        'session': str(path), 'remaining_corners': [name for name in CORNER_NAMES if name not in updated],
        'grid': grid,
    }, indent=2, ensure_ascii=False))


def reset(options):
    """Archive any old session, including an empty/corrupt one, before restart."""
    path = Path(options.session).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(path.suffix + '.lock').open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        archive = None
        if path.exists():
            stamp = datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')
            for index in range(1000):
                suffix = f'.backup-{stamp}' + (f'-{index}' if index else '')
                candidate = path.with_name(path.stem + suffix + path.suffix)
                if not candidate.exists():
                    os.replace(path, candidate)
                    archive = candidate
                    break
            if archive is None:
                raise RuntimeError('cannot find a free session archive name')
        _atomic_write(path, _new_session(options.base_frame, options.tcp_frame))
    print(json.dumps({'reset': True, 'session': str(path),
                      'archived_previous_session': str(archive) if archive else None,
                      'remaining_corners': list(CORNER_NAMES)}, indent=2))


def project(options):
    """Recalculate the display grid on a horizontal plane without touching raw TCP data."""
    path = Path(options.session).expanduser().resolve()
    with path.with_suffix(path.suffix + '.lock').open('a+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        document = _load_session(path)
        if document is None:
            raise ValueError(f'session does not exist: {path}')
        grid = (make_grid(document['corners'], options.plane_z)
                if len(document['corners']) == 4 else None)
        stamp = datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')
        for index in range(1000):
            suffix = f'.before-project-{stamp}' + (f'-{index}' if index else '')
            backup = path.with_name(path.stem + suffix + path.suffix)
            if not backup.exists():
                shutil.copy2(path, backup)
                break
        else:
            raise RuntimeError('cannot find a free backup name')
        document['projection_z_m'] = options.plane_z
        document['grid'] = grid
        _atomic_write(path, document)
    print(json.dumps({
        'projected': True, 'session': str(path), 'backup': str(backup),
        'projection_z_m': options.plane_z,
        'measured_tcp_z_span_mm': grid['measured_tcp_z_span_mm'] if grid else None,
        'region_centers_m': ({name: item['center_m'] for name, item in grid['regions'].items()}
                             if grid else None),
    }, indent=2))


def _point(xyz):
    point = Point()
    point.x, point.y, point.z = map(float, xyz)
    return point


def _marker(frame, stamp, marker_id, marker_type, rgb, scale, points=None,
            text='', position=None, alpha=1.0):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = stamp
    marker.ns = 'a4_tcp_grid'
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    if points:
        marker.points = [_point(value) for value in points]
    if position is not None:
        marker.pose.position = _point(position)
    marker.text = text
    return marker


def session_markers(document, stamp):
    frame = document['base_frame']
    deletion = Marker()
    deletion.header.frame_id = frame
    deletion.header.stamp = stamp
    deletion.action = Marker.DELETEALL
    markers = [deletion]
    grid = document.get('grid')
    normal = np.asarray(grid['normal']) if grid else np.asarray([0., 0., 1.])
    display_up = normal if normal[2] >= 0 else -normal
    plane_z = document.get('projection_z_m')
    for index, name in enumerate(CORNER_NAMES):
        if name not in document['corners']:
            continue
        xyz = np.asarray(document['corners'][name]['xyz_m']).copy()
        if plane_z is not None:
            xyz[2] = plane_z
        markers.append(_marker(frame, stamp, index * 2, Marker.SPHERE,
                               (1., 1., 1.), (.012, .012, .012), position=xyz))
        markers.append(_marker(frame, stamp, index * 2 + 1, Marker.TEXT_VIEW_FACING,
                               (1., 1., 1.), (0., 0., .023), text=name,
                               position=xyz + .018 * display_up))
    if grid:
        for index, name in enumerate(('A', 'B', 'C', 'D')):
            region = grid['regions'][name]
            rgb = COLORS[name]
            fill_corners = [np.asarray(point) + .001 * display_up
                       for point in region['corners_m']]
            if np.dot(np.cross(fill_corners[1] - fill_corners[0],
                               fill_corners[2] - fill_corners[0]), display_up) >= 0:
                triangles = [fill_corners[i] for i in (0, 1, 2, 0, 2, 3)]
            else:
                triangles = [fill_corners[i] for i in (0, 2, 1, 0, 3, 2)]
            markers.append(_marker(frame, stamp, 10 + index * 3,
                                   Marker.TRIANGLE_LIST, rgb, (1., 1., 1.),
                                   points=triangles, alpha=.45))
            outline = [point + .001 * display_up for point in fill_corners]
            markers.append(_marker(frame, stamp, 11 + index * 3,
                                   Marker.LINE_STRIP, rgb, (.004, 0., 0.),
                                   points=outline + [outline[0]]))
            center = np.asarray(region['center_m']) + .012 * display_up
            markers.append(_marker(frame, stamp, 12 + index * 3,
                                   Marker.TEXT_VIEW_FACING, LABEL_COLORS[name], (0., 0., .05),
                                   text=name, position=center))
    return MarkerArray(markers=markers)


class PaperGridPublisher(Node):
    def __init__(self, session):
        super().__init__('a4_paper_grid_publisher')
        self.session = Path(session).expanduser().resolve()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, TOPIC, qos)
        self.last_contents = None
        self.create_timer(0.5, self.refresh)
        self.refresh()

    def refresh(self):
        try:
            if not self.session.exists():
                return
            contents = self.session.read_bytes()
            if contents == self.last_contents:
                return
            document = _load_session(self.session)
            if len(document['corners']) == 4:
                document['grid'] = make_grid(
                    document['corners'], document.get('projection_z_m'))
            self.publisher.publish(session_markers(document, self.get_clock().now().to_msg()))
            self.last_contents = contents
            self.get_logger().info(f'published {len(document["corners"])} corners to {TOPIC}')
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'cannot publish paper grid: {exc}')


def serve(options):
    rclpy.init()
    node = PaperGridPublisher(options.session)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='Capture A4 TCP corners and display a 2x2 RViz grid')
    commands = parser.add_subparsers(dest='command', required=True)
    capture_parser = commands.add_parser('capture', help='record one stationary TCP paper corner')
    capture_parser.add_argument('--session', required=True, help='JSON session path')
    capture_parser.add_argument('--corner', choices=CORNER_NAMES, required=True)
    capture_parser.add_argument('--base-frame', default='base_link')
    capture_parser.add_argument('--tcp-frame', default='grasp_frame')
    capture_parser.add_argument('--replace', action='store_true')
    reset_parser = commands.add_parser('reset', help='archive old session and start with zero corners')
    reset_parser.add_argument('--session', required=True, help='JSON session path')
    reset_parser.add_argument('--base-frame', default='base_link')
    reset_parser.add_argument('--tcp-frame', default='grasp_frame')
    project_parser = commands.add_parser('project', help='project recorded corners onto base XY plane')
    project_parser.add_argument('--session', required=True, help='existing JSON session path')
    project_parser.add_argument('--plane-z', type=float, default=0.0,
                                help='Z height of RViz XY plane in base frame (default: 0 m)')
    serve_parser = commands.add_parser('serve', help='publish recorded corners and grid to RViz')
    serve_parser.add_argument('--session', required=True, help='same JSON session path as capture')
    options = parser.parse_args(args)
    try:
        if options.command == 'capture':
            capture(options)
        elif options.command == 'reset':
            reset(options)
        elif options.command == 'project':
            project(options)
        else:
            serve(options)
    except (RuntimeError, ValueError, OSError) as exc:
        parser.exit(2, f'paper_grid: {exc}\n')


if __name__ == '__main__':
    main()
