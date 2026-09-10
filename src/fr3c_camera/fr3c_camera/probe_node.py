"""A pixel probe for integration, not an object detector or robot command source."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from message_filters import Subscriber, ApproximateTimeSynchronizer
from .rgbd import validate_pair, project_color_pixel


class RGBDProbe(Node):
    def __init__(self):
        super().__init__('fr3c_rgbd_probe')
        defaults = {'camera_namespace': '/camera', 'pixel_u': 320.0, 'pixel_v': 240.0,
                    'patch_radius': 2, 'min_depth_m': 0.1, 'max_depth_m': 3.0,
                    'max_age_s': 0.5, 'max_skew_s': 0.06}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        ns = self.get_parameter('camera_namespace').value.rstrip('/')
        self.subs = [Subscriber(self, typ, ns + topic, qos_profile=qos_profile_sensor_data)
                     for typ, topic in [(Image,'/color/image_raw'), (Image,'/depth/image_raw'),
                                        (CameraInfo,'/color/camera_info'), (CameraInfo,'/depth/camera_info')]]
        self.sync = ApproximateTimeSynchronizer(self.subs, 12, self.get_parameter('max_skew_s').value)
        self.sync.registerCallback(self.pair)
        self.pub = self.create_publisher(PointStamped, '/fr3c_camera/probe_point', 10)
        self.diag = self.create_publisher(DiagnosticArray, '/fr3c_camera/diagnostics', 10)
        self.last_valid = None
        self.reason = 'waiting_for_rgbd'
        self.create_timer(0.5, self.status)

    def pair(self, color, depth, ci, di):
        p = lambda key: self.get_parameter(key).value
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            metres = validate_pair(color, depth, ci, di, now, p('max_age_s'), p('max_skew_s'))
            xyz, _, _ = project_color_pixel(metres, ci, di, p('pixel_u'), p('pixel_v'),
                radius=p('patch_radius'), min_depth=p('min_depth_m'), max_depth=p('max_depth_m'))
            point = PointStamped(header=depth.header)
            point.point.x, point.point.y, point.point.z = xyz
            self.pub.publish(point)
            self.last_valid = now
            self.reason = 'valid_camera_frame_probe_not_grasp_target'
        except (ValueError, RuntimeError, cv2.error) as exc:
            self.reason = str(exc)

    def status(self):
        now = self.get_clock().now()
        fresh = self.last_valid is not None and now.nanoseconds*1e-9 - self.last_valid <= self.get_parameter('max_age_s').value
        status = DiagnosticStatus(name='fr3c_camera/rgbd_probe', hardware_id='camera',
            level=DiagnosticStatus.OK if fresh and self.reason.startswith('valid_') else DiagnosticStatus.WARN,
            message=self.reason if fresh else 'no_fresh_valid_point: ' + self.reason,
            values=[KeyValue(key='hand_eye_calibrated', value='false'),
                    KeyValue(key='output_units', value='metres'),
                    KeyValue(key='purpose', value='pixel probe only')])
        self.diag.publish(DiagnosticArray(header=Header(stamp=now.to_msg()), status=[status]))


def main(args=None):
    rclpy.init(args=args)
    node = RGBDProbe()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
