"""Read-only cross-check of ROS FK against the controller flange pose."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import socket
import time
import xmlrpc.client

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from .geometry import (
    invert_transform,
    transform_matrix,
    transform_spread,
)
from .handeye_session import json_matrix, tf_to_matrix


JOINT_NAMES = [f'joint{index}' for index in range(1, 7)]


def rpy_degrees_to_rotation(values):
    roll, pitch, yaw = np.radians(np.asarray(values, dtype=float))
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rotation_x = np.asarray([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rotation_y = np.asarray([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rotation_z = np.asarray([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rotation_z @ rotation_y @ rotation_x


class PoseReader(Node):
    def __init__(self, base_frame, wrist_frame):
        super().__init__('fr3c_robot_pose_crosscheck')
        self.base_frame = base_frame
        self.wrist_frame = wrist_frame
        self.joint_state = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in JOINT_NAMES):
            self.joint_state = message

    def read(self, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is None:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.wrist_frame, Time(),
                    timeout=Duration(seconds=0.1))
                return self.joint_state, tf_to_matrix(transform)
            except Exception:
                continue
        raise RuntimeError('timed out waiting for /joint_states and base-to-wrist TF')


def controller_read(robot_ip):
    socket.setdefaulttimeout(3.0)
    proxy = xmlrpc.client.ServerProxy(f'http://{robot_ip}:20003/RPC2')
    flange = proxy.GetActualToolFlangePose(0)
    joints = proxy.GetActualJointPosDegree(0)
    if not flange or int(flange[0]) != 0:
        raise RuntimeError(f'GetActualToolFlangePose failed: {flange}')
    if not joints or int(joints[0]) != 0:
        raise RuntimeError(f'GetActualJointPosDegree failed: {joints}')
    xyz_m = np.asarray(flange[1:4], dtype=float) / 1000.0
    base_from_flange = transform_matrix(
        rpy_degrees_to_rotation(flange[4:7]), xyz_m)
    return base_from_flange, [float(value) for value in joints[1:7]], [
        float(value) for value in flange[1:7]]


def capture(output_path, robot_ip='192.168.58.2', base_frame='base_link',
            wrist_frame='flange_link'):
    output_path = Path(output_path).resolve()
    rclpy.init()
    node = PoseReader(base_frame, wrist_frame)
    try:
        joint_message, base_from_wrist = node.read()
        base_from_flange, controller_joint_deg, controller_flange = controller_read(robot_ip)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    ros_positions = dict(zip(joint_message.name, joint_message.position))
    ros_joint_deg = [float(np.degrees(ros_positions[name])) for name in JOINT_NAMES]
    wrist_from_flange = invert_transform(base_from_wrist) @ base_from_flange
    record = {
        'index': 0,
        'captured_at': datetime.now().astimezone().isoformat(),
        'robot_ip': robot_ip,
        'base_frame': base_frame,
        'wrist_frame': wrist_frame,
        'ros_joint_deg': ros_joint_deg,
        'controller_joint_deg': controller_joint_deg,
        'joint_max_absolute_difference_deg': float(np.max(np.abs(
            np.asarray(ros_joint_deg) - np.asarray(controller_joint_deg)))),
        'controller_flange_xyz_mm_rpy_deg': controller_flange,
        'base_from_wrist_ros': json_matrix(base_from_wrist),
        'base_from_flange_controller': json_matrix(base_from_flange),
        'wrist_from_flange_snapshot': json_matrix(wrist_from_flange),
    }
    if output_path.exists():
        document = json.loads(output_path.read_text())
    else:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        document = {'schema_version': 1, 'samples': []}
    record['index'] = len(document['samples']) + 1
    document['samples'].append(record)

    transforms = [
        np.asarray(sample['wrist_from_flange_snapshot'], dtype=float)
        for sample in document['samples']
    ]
    _, translation_mm, rotation_deg = transform_spread(transforms)
    document['summary'] = {
        'sample_count': len(transforms),
        'wrist_from_flange_translation_spread_max_mm': float(np.max(translation_mm)),
        'wrist_from_flange_rotation_spread_max_deg': float(np.max(rotation_deg)),
        'joint_max_absolute_difference_deg': float(max(
            sample['joint_max_absolute_difference_deg'] for sample in document['samples'])),
        'interpretation': (
            'A fixed ROS wrist-to-controller-flange relationship should have near-zero spread. '
            'Large spread indicates that ROS FK and controller flange kinematics are not equivalent.'
        ),
    }
    output_path.write_text(json.dumps(document, indent=2) + '\n')
    return record, document['summary']


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Append one stopped-pose ROS/controller FK cross-check sample.')
    parser.add_argument('output', help='JSON file to create or append')
    parser.add_argument('--robot-ip', default='192.168.58.2')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument(
        '--wrist-frame', default='flange_link',
        help='ROS robot-end frame to compare with controller ToolFlangePose')
    options = parser.parse_args(args)
    record, summary = capture(
        options.output, options.robot_ip, options.base_frame, options.wrist_frame)
    print(json.dumps({
        'captured': True,
        'sample_index': record['index'],
        'output': str(Path(options.output).resolve()),
        'this_joint_max_absolute_difference_deg': (
            record['joint_max_absolute_difference_deg']),
        'summary': summary,
    }, indent=2))


if __name__ == '__main__':
    main()
