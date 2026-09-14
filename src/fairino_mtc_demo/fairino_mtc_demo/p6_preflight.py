"""Collect a read-only ROS/controller P6 snapshot and issue an execution gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import yaml

from .p6_checks import (
    ARM_JOINTS, evaluate, invalidate_gate, pose_from_xyz_rpy, write_evidence)


def _pose(transform: TransformStamped):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        'translation_m': [translation.x, translation.y, translation.z],
        'quaternion_xyzw': [rotation.x, rotation.y, rotation.z, rotation.w],
        'parent_frame': transform.header.frame_id,
        'child_frame': transform.child_frame_id,
    }


class Collector(Node):
    def __init__(self):
        super().__init__('fr3c_p6_preflight')
        volatile = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        transient = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.arm_joint_state = None
        self.arm_joint_received = None
        self.controller_status = None
        self.control_state = None
        self.static_camera_sources = set()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self._joint, volatile)
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._dynamic, volatile)
        self.create_subscription(String, '/fairino/control_state', self._control, transient)
        self.create_subscription(TFMessage, '/tf_static', self._static_tf, transient)
        self.controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.hardware_client = self.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components')

    def _joint(self, message):
        if all(name in message.name for name in ARM_JOINTS):
            self.arm_joint_state = message
            self.arm_joint_received = self.get_clock().now()

    def _dynamic(self, message):
        for name, values in zip(message.joint_names, message.interface_values):
            if name == 'fairino_status':
                self.controller_status = {
                    key: float(value)
                    for key, value in zip(values.interface_names, values.values)
                }

    def _control(self, message):
        self.control_state = message.data

    def _static_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == 'camera_link':
                pose = _pose(transform)
                self.static_camera_sources.add((
                    pose['parent_frame'],
                    *[round(value, 12) for value in pose['translation_m']],
                    *[round(value, 12) for value in pose['quaternion_xyzw']],
                ))

    def collect(self, duration_s, service_timeout_s):
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        controllers = self._call(self.controllers_client, ListControllers.Request(),
                                 service_timeout_s)
        hardware = self._call(self.hardware_client, ListHardwareComponents.Request(),
                              service_timeout_s)
        joint_publishers = sorted(
            ('/' + endpoint.node_name if endpoint.node_namespace == '/' else
             endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name)
            for endpoint in self.get_publishers_info_by_topic('/joint_states'))
        snapshot = {
            'controller_status': self.controller_status,
            'control_state': self.control_state,
            'arm_joint_source_count': joint_publishers.count('/joint_state_broadcaster'),
            'joint_state_publishers': joint_publishers,
            'arm_joint_state_age_s': (
                (self.get_clock().now() - self.arm_joint_received).nanoseconds / 1e9
                if self.arm_joint_received is not None else math.inf),
            'nodes': sorted('/' + name if namespace == '/' else
                            namespace.rstrip('/') + '/' + name
                            for name, namespace in self.get_node_names_and_namespaces()),
            'camera_link_static_source_count': len(self.static_camera_sources),
            'controllers': ({item.name: item.state for item in controllers.controller}
                            if controllers else {}),
            'hardware': self._hardware_dict(hardware),
            'base_to_flange': self._lookup('base_link', 'flange_link'),
            'flange_to_grasp': self._lookup('flange_link', 'grasp_frame'),
            'flange_to_camera': self._lookup('flange_link', 'camera_link'),
        }
        if self.arm_joint_state is not None:
            values = dict(zip(self.arm_joint_state.name, self.arm_joint_state.position))
            snapshot['arm_joint_positions_rad'] = {
                name: values[name] for name in ARM_JOINTS}
        return snapshot

    def _call(self, client, request, timeout_s):
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return future.result() if future.done() and future.exception() is None else None

    def _lookup(self, parent, child):
        try:
            return _pose(self.tf_buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=0.5)))
        except Exception as error:
            return {'error': f'{type(error).__name__}: {error}'}

    @staticmethod
    def _hardware_dict(response):
        if response is None:
            return {}
        result = {}
        for component in response.component:
            arm = {}
            for interface in component.command_interfaces:
                if interface.name in [f'{name}/position' for name in ARM_JOINTS]:
                    arm[interface.name] = {
                        'available': interface.is_available,
                        'claimed': interface.is_claimed,
                    }
            result[component.name] = {
                'state': component.state.label,
                'type': component.type,
                'class_type': component.class_type,
                'arm_command_interfaces': arm,
            }
        return result


def _load_config(path):
    path = Path(path).expanduser().resolve()
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('P6 config must be a YAML mapping')
    for key in ('tcp_file', 'handeye_file'):
        value = Path(document[key]).expanduser()
        if not value.is_absolute():
            value = (path.parent / value).resolve()
        source = yaml.safe_load(value.read_text(encoding='utf-8'))
        if key == 'tcp_file':
            data = source['ydi_j22_tcp']
            document['expected_tcp'] = pose_from_xyz_rpy(
                data['xyz_m'], data['rpy_rad'])
        else:
            document['expected_handeye'] = pose_from_xyz_rpy(
                source['xyz'], source['rpy'])
    return document


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Read-only P6 preflight; never sends robot or gripper commands.')
    parser.add_argument('--config', required=True)
    parser.add_argument('--evidence-root', required=True)
    parser.add_argument('--sample-duration-s', type=float, default=2.0)
    parser.add_argument('--service-timeout-s', type=float, default=3.0)
    parser.add_argument('--confirm-workcell-clear', action='store_true')
    parser.add_argument('--confirm-lift-clearance-50mm', action='store_true')
    parser.add_argument('--confirm-cable-slack', action='store_true')
    parser.add_argument('--confirm-stop-method-ready', action='store_true')
    parser.add_argument('--confirm-no-external-motion-source', action='store_true')
    raw_args = sys.argv if args is None else list(args)
    parser_args = remove_ros_args(args=raw_args)
    if args is None:
        parser_args = parser_args[1:]
    options = parser.parse_args(parser_args)
    invalidate_gate(options.evidence_root)
    confirmations = {
        'workcell_clear': options.confirm_workcell_clear,
        'lift_clearance_50mm': options.confirm_lift_clearance_50mm,
        'cable_slack': options.confirm_cable_slack,
        'stop_method_ready': options.confirm_stop_method_ready,
        'no_external_motion_source': options.confirm_no_external_motion_source,
    }
    try:
        config = _load_config(options.config)
        rclpy.init(args=None)
        node = Collector()
        try:
            snapshot = node.collect(
                options.sample_duration_s, options.service_timeout_s)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        checks = evaluate(snapshot, config, confirmations)
    except Exception as error:
        config = {'control_path': 'unverified'}
        snapshot = {
            'collection_error': f'{type(error).__name__}: {error}',
        }
        checks = [{
            'name': 'preflight_collection_completed',
            'passed': False,
            'observed': snapshot['collection_error'],
            'expected': 'complete read-only P6 snapshot',
            'detail': 'The execution gate remains failed.',
        }]
    passed, report_path, gate_path = write_evidence(
        options.evidence_root, snapshot, config, confirmations, checks)
    summary = {
        'passed': passed,
        'failed_checks': [check['name'] for check in checks if not check['passed']],
        'report_path': str(report_path),
        'gate_path': str(gate_path),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
