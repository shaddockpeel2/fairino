"""Read-only ROS parameter evidence; no publishers, leases or motion commands."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import time


PARAMETERS = {
    '/servo_node': [
        'moveit_servo.command_in_type', 'moveit_servo.publish_period',
        'moveit_servo.low_latency_mode', 'moveit_servo.command_out_type',
        'moveit_servo.command_out_topic', 'moveit_servo.publish_joint_positions',
        'moveit_servo.publish_joint_velocities', 'moveit_servo.check_collisions',
        'moveit_servo.incoming_command_timeout',
        'moveit_servo.smoothing_filter_plugin_name', 'butterworth_filter_coeff',
        'moveit_servo.scale.linear', 'moveit_servo.scale.rotational',
        'robot_description', 'robot_description_semantic',
    ],
    '/controller_manager': ['update_rate'],
    '/fairino3mt_servo_position_controller': ['joints', 'interface_name'],
    '/fr3c_trajectory_control_bridge': [
        'servo_controller', 'servo_command_topic', 'servo_command_timeout_s',
        'tracking_heartbeat_timeout_s',
    ],
}


def collect(node, timeout_s=2.0):
    """Bounded service reads. Missing services/parameters remain explicit."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python

    snapshot = {'captured_wall_s': time.time(), 'nodes': {}}
    for name, parameters in PARAMETERS.items():
        client = node.create_client(GetParameters, name + '/get_parameters')
        entry = {'parameters': {}, 'error': None}
        future = None
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RuntimeError('parameter_service_unavailable')
            deadline = time.monotonic() + timeout_s

            def read(names):
                nonlocal future
                future = client.call_async(GetParameters.Request(names=names))
                while not future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                if not future.done():
                    raise RuntimeError('parameter_service_timeout')
                return future.result().values

            values = read(parameters)
            if len(values) != len(parameters):
                # Humble may reject the whole batch when one optional parameter
                # is undeclared. Retry individually within the SAME deadline.
                from rcl_interfaces.msg import ParameterValue
                values = []
                for key in parameters:
                    response = read([key])
                    values.append(response[0] if len(response) == 1
                                  else ParameterValue())
            for key, value in zip(parameters, values):
                value = parameter_value_to_python(value)
                if key in ('robot_description', 'robot_description_semantic'):
                    value = ({'sha256': hashlib.sha256(value.encode()).hexdigest(),
                              'length': len(value)} if isinstance(value, str) else None)
                entry['parameters'][key] = value
        except Exception as error:
            entry['error'] = f'{type(error).__name__}:{error}'
        finally:
            if future is not None and not future.done():
                future.cancel()
            node.destroy_client(client)
        snapshot['nodes'][name] = entry
    return snapshot


def forward_profile_errors(snapshot):
    """Fail closed on missing essential settings before acquiring a lease."""
    errors = []
    nodes = snapshot.get('nodes', {})
    for name in PARAMETERS:
        if name not in nodes or nodes[name].get('error'):
            errors.append(f'{name}:snapshot_unavailable')

    def params(name):
        return nodes.get(name, {}).get('parameters', {})

    expected = {
        '/servo_node': {
            'moveit_servo.command_in_type': 'speed_units',
            'moveit_servo.command_out_type': 'std_msgs/Float64MultiArray',
            'moveit_servo.command_out_topic':
                '/p1/servo_position_candidates',
            'moveit_servo.publish_joint_positions': True,
            'moveit_servo.publish_joint_velocities': False,
            'moveit_servo.check_collisions': True,
        },
        '/fairino3mt_servo_position_controller': {
            'joints': [f'joint{i}' for i in range(1, 7)],
            'interface_name': 'position',
        },
        '/fr3c_trajectory_control_bridge': {
            'servo_controller': 'fairino3mt_servo_position_controller',
            'servo_command_topic': '/fairino3mt_servo_position_controller/commands',
        },
    }
    for node, values in expected.items():
        for key, value in values.items():
            actual = params(node).get(key)
            if type(actual) is not type(value) or actual != value:
                errors.append(f'{node}:{key}:unexpected')
    servo = params('/servo_node')
    period = servo.get('moveit_servo.publish_period')
    if (not isinstance(period, (int, float)) or isinstance(period, bool)
            or not math.isfinite(period) or not 0.008 <= period <= 0.020):
        errors.append('/servo_node:publish_period:outside_P1_matrix')
    for key in ('robot_description', 'robot_description_semantic'):
        if not isinstance(servo.get(key), dict) or not servo[key].get('length'):
            errors.append(f'/servo_node:{key}:missing')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    import rclpy
    rclpy.init()
    node = rclpy.create_node('p1_read_only_parameter_snapshot')
    try:
        snapshot = collect(node)
        snapshot['forward_profile_errors'] = forward_profile_errors(snapshot)
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(snapshot, stream, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps(snapshot, indent=2))
        return 1 if snapshot['forward_profile_errors'] else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
