"""Pair image quality with image-time base poses; publish read-only observations."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .follow_geometry import quaternion_from_rotation, transform
from .marker_pose import rpy_degrees
from .orientation_quality import ExactPosePairer, OrientationGate, measurement_problem, stamp_ns


def exact_stamp(stamp):
    return dict(sec=stamp.sec, nanosec=stamp.nanosec)


class OrientationCheck(Node):
    def __init__(self):
        super().__init__('orientation_check')
        defaults = dict(target_timeout_s=.35, feedback_timeout_s=.20, confirmation_frames=5,
                        max_angular_speed_deg_s=60., jitter_allowance_deg=3., publish_hz=20.,
                        target_id=1, source_frame_id='camera_color_optical_frame')
        self.options = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in ('feedback_timeout_s', 'publish_hz'):
            if not math.isfinite(self.options[key]) or self.options[key] <= 0:
                raise ValueError('invalid_' + key)
        self.gate = OrientationGate(self.options['confirmation_frames'], self.options['target_timeout_s'],
                                    self.options['max_angular_speed_deg_s'], self.options['jitter_allowance_deg'])
        self.pairer = ExactPosePairer()
        self.watermark = 0
        self.last_pair_wall = self.feedback_wall = 0.
        self.feedback_ns = 0
        self.state = self.empty('waiting_for_paired_pose')
        self.observation_pub = self.create_publisher(String, '/target_follow/pose_observation', 10)
        self.state_pub = self.create_publisher(String, '/target_follow/orientation_state', 10)
        self.create_subscription(PoseStamped, '/target_follow/tag_pose_base', self.pose, 10)
        self.create_subscription(String, '/target_follow/quality', self.quality, 10)
        self.create_subscription(PoseStamped, '/target_follow/flange_pose', self.feedback, 50)
        self.create_timer(1/self.options['publish_hz'], self.tick)
        self.get_logger().info('Stage 6: exact image stamps, base-frame rotation, read-only; no motion client.')

    @staticmethod
    def empty(reason):
        return dict(schema_version=1, status=reason, frame_id='base_link', stamp=None,
                    valid_position=False, valid_orientation=False, orientation_measurement_valid=False,
                    position_m=None, quaternion_xyzw=None, motion_commands_enabled=False,
                    orientation_control_enabled=False, confirmation_count=0)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def feedback_fresh(self):
        age = (self.now_ns()-self.feedback_ns)/1e9
        return (self.feedback_ns > 0 and -.05 <= age <= self.options['feedback_timeout_s'] and
                time.monotonic()-self.feedback_wall <= self.options['feedback_timeout_s'])

    def feedback(self, message):
        try:
            ns = stamp_ns(exact_stamp(message.header.stamp))
            if message.header.frame_id != 'base_link':
                raise ValueError('unexpected_feedback_frame')
            if ns <= self.feedback_ns:
                return
            p, q = message.pose.position, message.pose.orientation
            transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            if (ns-self.now_ns())/1e9 > .05:
                return
            self.feedback_ns, self.feedback_wall = ns, time.monotonic()
        except ValueError:
            self.invalidate('invalid_robot_feedback')

    def invalidate(self, reason, ns=None, preserve_history=False):
        if ns is not None and ns < self.watermark:
            return
        if ns is not None:
            self.watermark = max(ns, self.watermark)
            self.pairer.discard_before(ns-1)
        if preserve_history:
            self.gate.confirmed = 0
        else:
            self.gate.reset()
        self.state = self.empty(reason)
        if ns is not None:
            self.state['stamp'] = dict(sec=ns//10**9, nanosec=ns%10**9)
        self.publish(self.observation_pub, self.state)

    def pose(self, message):
        try:
            if message.header.frame_id != 'base_link':
                raise ValueError('unexpected_base_frame')
            stamp = exact_stamp(message.header.stamp)
            ns = stamp_ns(stamp)
            if ns < self.watermark or ns <= self.pairer.last_paired_ns:
                return
            if not -.05 <= (self.now_ns()-ns)/1e9 <= self.options['target_timeout_s']:
                raise ValueError('stale_base_pose')
            p, q = message.pose.position, message.pose.orientation
            matrix = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            self.consume(self.pairer.add('pose', stamp, matrix))
        except ValueError as exc:
            self.invalidate(str(exc))

    def quality(self, message):
        try:
            row = json.loads(message.data)
            if not isinstance(row, dict):
                raise ValueError('invalid_quality_json')
            if row.get('target_id') != self.options['target_id']:
                raise ValueError('unexpected_target_id')
            if row.get('status') == 'no_fresh_image':
                self.invalidate('no_fresh_image')
                return
            ns = stamp_ns(row.get('stamp'))
            if ns < self.watermark or ns <= self.pairer.last_paired_ns:
                return
            if not -.05 <= (self.now_ns()-ns)/1e9 <= self.options['target_timeout_s']:
                raise ValueError('stale_quality')
            if row.get('frame_id') != self.options['source_frame_id']:
                raise ValueError('unexpected_quality_frame')
            problem = measurement_problem(row)
            # A known rejection invalidates immediately, even if its pose arrives later.
            if problem:
                self.invalidate(problem, ns, preserve_history=True)
            self.consume(self.pairer.add('quality', row['stamp'], row))
        except (ValueError, TypeError) as exc:
            self.invalidate(str(exc))

    def consume(self, pair):
        if pair is None:
            return
        ns, matrix, quality = pair
        if ns < self.watermark:
            return
        age = (self.now_ns()-ns)/1e9
        if not -.05 <= age <= self.options['target_timeout_s']:
            self.invalidate('stale_paired_pose')
            return
        if not self.feedback_fresh():
            self.invalidate('stale_robot_feedback', ns)
            return
        gate = self.gate.observe(ns, matrix[:3, :3], measurement_problem(quality))
        self.watermark = ns
        self.last_pair_wall = time.monotonic()
        self.state = dict(schema_version=1, frame_id='base_link', source_frame_id=quality['frame_id'],
                          stamp=quality['stamp'], stamp_s=ns/1e9, target_id=quality['target_id'],
                          position_m=matrix[:3, 3].tolist(),
                          quaternion_xyzw=quaternion_from_rotation(matrix[:3, :3]).tolist(),
                          rpy_deg=rpy_degrees(matrix[:3, :3]),
                          valid_position=quality.get('valid_position') is True,
                          orientation_measurement_valid=measurement_problem(quality) is None,
                          quality=quality, motion_commands_enabled=False, orientation_control_enabled=False,
                          **gate)
        self.publish(self.observation_pub, self.state)

    def publish(self, publisher, row):
        output = dict(row, published_stamp_s=self.now_ns()/1e9, pairing=self.pairer.stats())
        publisher.publish(String(data=json.dumps(output, allow_nan=False)))

    def tick(self):
        now = self.now_ns()
        self.pairer.expire(now, self.options['target_timeout_s'])
        if self.state['stamp'] is not None and self.state.get('position_m') is not None:
            age = (now-stamp_ns(self.state['stamp']))/1e9
            if not self.feedback_fresh():
                self.invalidate('stale_robot_feedback')
            elif (not -.05 <= age <= self.options['target_timeout_s'] or
                  time.monotonic()-self.last_pair_wall > self.options['target_timeout_s']):
                self.invalidate('stale_target')
        state = dict(self.state, feedback_age_ms=(now-self.feedback_ns)/1e6 if self.feedback_ns else None)
        if self.state['stamp'] is not None:
            state['target_age_ms'] = (now-stamp_ns(self.state['stamp']))/1e6
        self.publish(self.state_pub, state)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OrientationCheck()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
