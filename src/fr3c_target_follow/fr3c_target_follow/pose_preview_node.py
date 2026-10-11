"""Stage 7: paired trusted observations -> full pose preview, never SDK commands."""
import json
from pathlib import Path
import time

import numpy as np
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .follow_geometry import quaternion_from_rotation, transform, transform_rpy
from .marker_pose import rpy_degrees
from .orientation_quality import measurement_problem, stamp_ns
from .pose_preview import PosePreview


PREFIX = '/target_follow/pose_preview/'
SOFT_REASONS = {'orientation_ambiguous', 'acquiring_orientation'}


class PosePreviewNode(Node):
    def __init__(self):
        super().__init__('pose_preview')
        defaults = dict(tcp_file='', target_id=1, auto_teach=False, publish_hz=50.,
                        target_timeout_s=.35, feedback_timeout_s=.20,
                        position_tau_s=.15, rotation_tau_s=.4, deadband_enter_deg=3.,
                        deadband_exit_deg=2., rotation_mapping_gain=1., kp_position=2., kp_rotation=1.,
                        max_speed_m_s=.02, max_accel_m_s2=.06,
                        max_angular_speed_deg_s=3., max_angular_accel_deg_s2=10.)
        self.options = {key: self.declare_parameter(key, value, ParameterDescriptor(read_only=True)).value
                        for key, value in defaults.items()}
        for key in ('feedback_timeout_s', 'publish_hz'):
            if not np.isfinite(self.options[key]) or self.options[key] <= 0:
                raise ValueError('invalid_'+key)
        config = yaml.safe_load(Path(self.options['tcp_file']).read_text())['ydi_j22_tcp']
        if (config.get('calibrated') is not True or config['parent_frame'] != 'flange_link' or
                config['child_frame'] != 'grasp_frame'):
            raise ValueError('calibrated flange_link -> grasp_frame TCP is required')
        self.flange_tcp = transform_rpy(config['xyz_m'], config['rpy_rad'])
        self.preview = PosePreview(**{key: value for key, value in self.options.items()
                                      if key not in ('tcp_file', 'target_id', 'auto_teach',
                                                     'publish_hz', 'feedback_timeout_s')})
        self.feedback_ns = self.event_ns = self.good_ns = 0
        self.feedback_wall = self.good_wall = 0.
        self.flange_matrix = None
        self.feedback_error = None
        self.reason = 'waiting_for_trusted_pose'
        self.latest_raw = None
        self.latest_raw_ns = 0
        self.last_tick = time.monotonic()
        self.auto_pending = self.options['auto_teach']
        self.teach_count = 0
        self.state_pub = self.create_publisher(String, PREFIX+'state', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, PREFIX+'velocity', 10)
        self.pose_pubs = {name: self.create_publisher(PoseStamped, PREFIX+name, 10) for name in
                          ('actual_tcp', 'raw_target', 'filtered_target', 'mapped_target',
                           'desired_tcp', 'reference_tcp', 'reference_flange')}
        self.create_subscription(PoseStamped, '/target_follow/flange_pose', self.feedback, 50)
        self.create_subscription(String, '/target_follow/pose_observation', self.observation, 50)
        self.create_service(Trigger, PREFIX+'teach', self.teach)
        self.create_timer(1/self.options['publish_hz'], self.tick)
        self.get_logger().info('Stage 7 READ ONLY. T records pose; no start/stop motion services or SDK client.')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def fresh(self, ns, arrival, timeout):
        return ns > 0 and -.05 <= (self.now_ns()-ns)/1e9 <= timeout and time.monotonic()-arrival <= timeout

    def feedback(self, message):
        try:
            ns = stamp_ns(dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec))
            if ns <= self.feedback_ns:
                return
            if message.header.frame_id != 'base_link':
                raise ValueError('unexpected_robot_frame')
            if (ns-self.now_ns())/1e9 > .05:
                raise ValueError('future_robot_feedback')
            p, q = message.pose.position, message.pose.orientation
            matrix = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            self.flange_matrix = matrix
            self.feedback_ns, self.feedback_wall = ns, time.monotonic()
            self.feedback_error = None
        except (ValueError, TypeError) as exc:
            self.feedback_error = str(exc)
            self.preview.stop()

    def current_tcp(self):
        if self.feedback_error:
            raise ValueError(self.feedback_error)
        if not self.fresh(self.feedback_ns, self.feedback_wall, self.options['feedback_timeout_s']):
            raise ValueError('stale_robot_feedback')
        return self.flange_matrix @ self.flange_tcp

    def trusted_fresh(self):
        return self.fresh(self.good_ns, self.good_wall, self.options['target_timeout_s'])

    def observation(self, message):
        try:
            row = json.loads(message.data)
            if not isinstance(row, dict) or row.get('schema_version') != 1 or row.get('frame_id') != 'base_link':
                raise ValueError('invalid_pose_observation')
            if row.get('motion_commands_enabled') is not False or row.get('orientation_control_enabled') is not False:
                raise ValueError('unexpected_motion_source')
            ns = stamp_ns(row['stamp']) if row.get('stamp') is not None else None
            if ns is not None:
                if ns < self.event_ns or ns <= self.good_ns:
                    return
                if not -.05 <= (self.now_ns()-ns)/1e9 <= self.options['target_timeout_s']:
                    raise ValueError('stale_pose_observation')
                self.event_ns = ns
            if row.get('position_m') is not None:
                if ns is None:
                    raise ValueError('missing_raw_pose_stamp')
                self.latest_raw = transform(row['position_m'], row['quaternion_xyzw'])
                self.latest_raw_ns = ns
            if row.get('valid_orientation') is not True:
                self.reason = row.get('status', 'invalid_pose_observation')
                if self.reason == 'valid':
                    raise ValueError('invalid_pose_observation')
                if self.reason not in SOFT_REASONS:
                    self.preview.stop()
                return
            quality = row.get('quality')
            if (ns is None or row.get('status') != 'valid' or row.get('valid_position') is not True or
                    row.get('orientation_measurement_valid') is not True or
                    row.get('target_id') != self.options['target_id'] or
                    row.get('source_frame_id') != 'camera_color_optical_frame' or
                    not isinstance(quality, dict) or stamp_ns(quality.get('stamp')) != ns or
                    measurement_problem(quality) is not None):
                raise ValueError('invalid_trusted_pose')
            if row.get('position_m') is None:
                raise ValueError('missing_trusted_pose')
            target = transform(row['position_m'], row['quaternion_xyzw'])
            self.preview.observe(ns, target)
            self.good_ns, self.good_wall = ns, time.monotonic()
            self.reason = None
            if self.auto_pending:
                try:
                    self.record()
                    self.auto_pending = False
                except ValueError:
                    pass  # Wait for a valid feedback packet, never manufacture TCP.
        except (ValueError, TypeError, KeyError) as exc:
            self.reason = str(exc)
            self.preview.stop()

    def record(self):
        actual_tcp = self.current_tcp()
        if self.reason is not None or not self.trusted_fresh():
            raise ValueError('fresh_trusted_pose_required')
        self.preview.teach(self.good_ns, self.preview.raw, actual_tcp)
        self.teach_count += 1
        self.auto_pending = False
        self.get_logger().info('Recorded target-to-TCP pose for preview only.')
        return dict(offset_matrix=self.preview.offset.tolist(), rotation_mapping_gain=self.preview.mapping_gain,
                    motion_commands_enabled=False, orientation_control_enabled=False)

    def teach(self, _request, response):
        try:
            response.message = json.dumps(self.record(), allow_nan=False)
            response.success = True
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def output_pose(self, name, matrix, ns):
        message = PoseStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)
        message.pose.position.x, message.pose.position.y, message.pose.position.z = matrix[:3, 3].tolist()
        q = quaternion_from_rotation(matrix[:3, :3]).tolist()
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = q
        self.pose_pubs[name].publish(message)

    @staticmethod
    def add_pose(state, name, matrix):
        state[name+'_m'] = matrix[:3, 3].tolist()
        state[name+'_rpy_deg'] = rpy_degrees(matrix[:3, :3])
        state[name+'_quaternion_xyzw'] = quaternion_from_rotation(matrix[:3, :3]).tolist()

    def tick(self):
        wall = time.monotonic()
        dt, self.last_tick = wall-self.last_tick, wall
        now = self.now_ns()
        state = dict(schema_version=1, stage=7, stamp=dict(sec=now//10**9, nanosec=now%10**9),
                     motion_commands_enabled=False, orientation_control_enabled=False,
                     reference_kind='one_step_from_actual_tcp', offset_recorded=self.preview.offset is not None,
                     teach_count=self.teach_count, quality_reason=self.reason,
                     rotation_mapping_gain=self.preview.mapping_gain,
                     deadband_enter_deg=self.options['deadband_enter_deg'],
                     rotation_tau_s=self.options['rotation_tau_s'],
                     velocity_m_s=[0., 0., 0.], angular_velocity_deg_s=[0., 0., 0.],
                     target_age_ms=(now-self.good_ns)/1e6 if self.good_ns else None,
                     feedback_age_ms=(now-self.feedback_ns)/1e6 if self.feedback_ns else None,
                     trusted_image_stamp=dict(sec=self.good_ns//10**9, nanosec=self.good_ns%10**9) if self.good_ns else None)
        if self.latest_raw is not None:
            self.add_pose(state, 'raw_target', self.latest_raw)
            self.output_pose('raw_target', self.latest_raw, self.latest_raw_ns)
            state['raw_image_stamp'] = dict(sec=self.latest_raw_ns//10**9, nanosec=self.latest_raw_ns%10**9)
        if self.preview.filtered is not None:
            self.add_pose(state, 'filtered_target', self.preview.filtered)
        v, w = np.zeros(3), np.zeros(3)
        try:
            actual = self.current_tcp()
            self.add_pose(state, 'actual_tcp', actual)
            self.output_pose('actual_tcp', actual, self.feedback_ns)
            if not self.trusted_fresh():
                raise ValueError('stale_trusted_pose' if self.good_ns else self.reason or 'waiting_for_trusted_pose')
            if self.reason is not None and self.reason not in SOFT_REASONS:
                raise ValueError(self.reason)
            self.output_pose('filtered_target', self.preview.filtered, self.good_ns)
            if self.preview.offset is None:
                state['status'] = 'ready_to_teach' if self.reason is None else 'waiting_for_trusted_pose'
            else:
                output = self.preview.step(actual, dt)
                mapped = self.preview.mapped_target()
                reference_flange = output['reference'] @ np.linalg.inv(self.flange_tcp)
                for name, matrix in [('mapped_target', mapped), ('desired_tcp', output['goal']),
                                     ('reference_tcp', output['reference']), ('reference_flange', reference_flange)]:
                    self.add_pose(state, name, matrix)
                    self.output_pose(name, matrix, self.good_ns if name == 'mapped_target' else now)
                v, w = output['velocity'], output['omega']
                state.update(status='holding_quality' if self.reason in SOFT_REASONS else 'preview',
                             position_error_m=output['position_error'].tolist(),
                             error_norm_mm=float(np.linalg.norm(output['position_error'])*1000),
                             rotation_error_deg=float(np.degrees(np.linalg.norm(output['angular_error']))),
                             velocity_m_s=v.tolist(), angular_velocity_deg_s=np.degrees(w).tolist(),
                             offset_matrix=self.preview.offset.tolist(),
                             rotation_active=self.preview.rotation_active)
        except ValueError as exc:
            self.preview.stop()
            state['status'] = str(exc)
        velocity = TwistStamped()
        velocity.header.frame_id = 'base_link'
        velocity.header.stamp.sec, velocity.header.stamp.nanosec = divmod(now, 10**9)
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z = v.tolist()
        velocity.twist.angular.x, velocity.twist.angular.y, velocity.twist.angular.z = w.tolist()
        self.velocity_pub.publish(velocity)
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PosePreviewNode()
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
