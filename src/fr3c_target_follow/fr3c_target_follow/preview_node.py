"""Transform target at image time and publish a position-only PBVS preview.

All outputs are /target_follow/* preview data; this node has no SDK or motion client.
"""
import json
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from .follow_geometry import PoseHistory, PositionPreview, quaternion_from_rotation, transform, transform_rpy


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def seconds(stamp):
    return stamp.sec + stamp.nanosec/1e9


class FollowPreview(Node):
    def __init__(self):
        super().__init__('follow_preview')
        defaults = dict(tcp_file='', transform_only=False, target_timeout_s=.35, feedback_timeout_s=.20,
                        max_interpolation_gap_s=.05, kp=1.5, max_speed_m_s=.01,
                        max_accel_m_s2=.03, publish_hz=50.0)
        self.options = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key in defaults.keys()-{'tcp_file', 'transform_only'}:
            if not np.isfinite(self.options[key]) or self.options[key] <= 0:
                raise ValueError('invalid parameter: '+key)
        tcp_data = yaml.safe_load(Path(self.options['tcp_file']).read_text())['ydi_j22_tcp']
        if not tcp_data.get('calibrated') or tcp_data['parent_frame'] != 'flange_link' or tcp_data['child_frame'] != 'grasp_frame':
            raise ValueError('a calibrated flange_link -> grasp_frame TCP is required')
        self.flange_tcp = transform_rpy(tcp_data['xyz_m'], tcp_data['rpy_rad'])
        self.history = PoseHistory()
        self.preview = PositionPreview(self.options['kp'], self.options['max_speed_m_s'], self.options['max_accel_m_s2'])
        self.buffer = Buffer(cache_time=Duration(seconds=5))
        self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.target = None
        self.target_stamp = self.last_input_stamp = 0.0
        self.target_matrix = None
        self.interpolation_gap = 0.0
        self.last_tick = time.monotonic()
        self.robot_error = None
        self.pose_pub = self.create_publisher(PoseStamped, '/target_follow/tag_pose_base', 10)
        self.tcp_pub = self.create_publisher(PoseStamped, '/target_follow/tcp_pose_base', 10)
        self.goal_pub = None if self.options['transform_only'] else self.create_publisher(PoseStamped, '/target_follow/desired_tcp_pose', 10)
        self.ref_pub = None if self.options['transform_only'] else self.create_publisher(PoseStamped, '/target_follow/preview_reference', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/target_follow/preview_velocity', 10)
        self.state_pub = self.create_publisher(String, '/target_follow/preview_state', 10)
        self.create_subscription(PoseStamped, '/target_follow/flange_pose', self.flange, 50)
        self.create_subscription(PoseStamped, '/target_follow/tag_pose', self.tag, 5)
        if not self.options['transform_only']:
            self.create_service(Trigger, '/target_follow/teach_offset', self.teach)
        self.create_timer(1/self.options['publish_hz'], self.tick)
        self.get_logger().info('Image-time base-frame conversion only; offset and motion belong to the follow driver.'
                               if self.options['transform_only'] else
                               'Base-frame position PBVS preview; robot motion disabled; use teach_offset to record current offset.')

    def flange(self, message):
        if message.header.frame_id != 'base_link':
            self.robot_error = 'unexpected_robot_frame'
            return
        try:
            p, q = message.pose.position, message.pose.orientation
            self.history.add(seconds(message.header.stamp), [p.x,p.y,p.z], [q.x,q.y,q.z,q.w])
            self.robot_error = None
        except ValueError as exc:
            self.robot_error = str(exc)

    def tag(self, message):
        stamp = seconds(message.header.stamp)
        if message.header.frame_id != 'camera_color_optical_frame' or stamp <= self.last_input_stamp:
            return
        self.last_input_stamp = stamp
        self.pending = message

    def output_pose(self, publisher, matrix, stamp):
        message = PoseStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp = stamp
        message.pose.position.x, message.pose.position.y, message.pose.position.z = matrix[:3,3].tolist()
        q = quaternion_from_rotation(matrix[:3,:3]).tolist()
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = q
        publisher.publish(message)

    def current_tcp(self, now):
        if self.robot_error:
            raise ValueError(self.robot_error)
        stamp, flange = self.history.latest()
        if not -.05 <= now-stamp <= self.options['feedback_timeout_s']:
            raise ValueError('stale_robot_feedback')
        return flange @ self.flange_tcp

    def accept_pending(self, now):
        if self.pending is None:
            return
        message = self.pending
        stamp = seconds(message.header.stamp)
        if not -.05 <= now-stamp <= self.options['target_timeout_s']:
            self.pending = None
            raise ValueError('stale_target')
        flange, gap = self.history.at(stamp, self.options['max_interpolation_gap_s'])
        static_tf = self.buffer.lookup_transform('flange_link', message.header.frame_id, Time())
        t, q = static_tf.transform.translation, static_tf.transform.rotation
        flange_camera = transform([t.x,t.y,t.z], [q.x,q.y,q.z,q.w])
        target = flange @ flange_camera @ pose_matrix(message.pose)
        self.target_matrix, self.target = target, target[:3,3]
        self.target_stamp, self.interpolation_gap = stamp, gap
        self.pending = None
        self.output_pose(self.pose_pub, target, message.header.stamp)

    def teach(self, _request, response):
        now = self.get_clock().now().nanoseconds/1e9
        try:
            tcp = self.current_tcp(now)
            if self.target is None or now-self.target_stamp > self.options['target_timeout_s']:
                raise ValueError('fresh base-frame target is required')
            self.preview.teach(self.target, tcp)
            response.success = True
            response.message = json.dumps(dict(offset_m=self.preview.offset.tolist(), orientation_control_enabled=False, motion_commands_enabled=False))
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def tick(self):
        wall_now = time.monotonic()
        dt, self.last_tick = wall_now-self.last_tick, wall_now
        stamp = self.get_clock().now()
        now = stamp.nanoseconds/1e9
        state = dict(motion_commands_enabled=False, orientation_control_enabled=False,
                     offset_recorded=self.preview.offset is not None,
                     reference_kind='one_step_from_actual_tcp', stamp_s=now,
                     velocity_m_s=[0.0,0.0,0.0])
        velocity = np.zeros(3)
        try:
            tcp = self.current_tcp(now)
            state['actual_tcp_m'] = tcp[:3,3].tolist()
            self.output_pose(self.tcp_pub, tcp, stamp.to_msg())
            self.accept_pending(now)
            if self.target is None or now-self.target_stamp > self.options['target_timeout_s']:
                raise ValueError('stale_or_missing_target')
            state.update(target_base_m=self.target.tolist(), target_age_ms=(now-self.target_stamp)*1000,
                         interpolation_gap_ms=self.interpolation_gap*1000,
                         offset_m=None if self.preview.offset is None else self.preview.offset.tolist())
            if self.options['transform_only']:
                state['status'] = 'base_transform'
                self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))
                return
            goal, reference, error, velocity = self.preview.step(self.target, tcp, dt)
            self.output_pose(self.goal_pub, goal, stamp.to_msg())
            self.output_pose(self.ref_pub, reference, stamp.to_msg())
            state.update(status='preview', desired_tcp_m=goal[:3,3].tolist(),
                         position_error_m=error.tolist(), error_norm_mm=float(np.linalg.norm(error)*1000),
                         preview_reference_m=reference[:3,3].tolist(), velocity_m_s=velocity.tolist())
        except (ValueError, TransformException) as exc:
            self.preview.stop()
            state['status'] = str(exc)
        output = TwistStamped()
        output.header.stamp = stamp.to_msg(); output.header.frame_id = 'base_link'
        output.twist.linear.x, output.twist.linear.y, output.twist.linear.z = velocity.tolist()
        self.velocity_pub.publish(output)
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FollowPreview()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can invalidate a subscription while Ctrl-C shuts down its context.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            try: node.destroy_node()
            except KeyboardInterrupt: pass
        if rclpy.ok(): rclpy.shutdown()
