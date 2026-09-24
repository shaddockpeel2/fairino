#!/usr/bin/env python3
"""Read-only P1 path guard / stop integration preflight. Never acquires a lease."""

import argparse
import json
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from p1_parameter_snapshot import collect, forward_profile_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; choose a new evidence path')
    rclpy.init()
    node = rclpy.create_node('p1_stop_path_preflight')
    states = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/fairino/control_state',
                             lambda message: states.append(message.data), qos)
    report = {'read_only': True, 'errors': []}

    def call(service, kind, timeout=3.0):
        client = node.create_client(kind, service)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(service + ':unavailable')
            future = client.call_async(kind.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                future.cancel()
                raise RuntimeError(service + ':timeout')
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        report['parameters'] = collect(node)
        report['errors'].extend(forward_profile_errors(report['parameters']))
        response = call('/controller_manager/list_controllers', ListControllers)
        controllers = {c.name: c.state for c in response.controller}
        report['controllers'] = controllers
        for name, expected in (
            ('joint_state_broadcaster', 'active'),
            ('fairino3mt_execution_controller', 'inactive'),
            ('fairino3mt_servo_position_controller', 'inactive'),
        ):
            if controllers.get(name) != expected:
                report['errors'].append(name + ':unexpected_state')
        response = call('/p1/path_guard/ready', Trigger)
        report['path_guard'] = {'success': response.success, 'message': response.message}
        if not response.success:
            report['errors'].append('path_guard:' + response.message)
        report['control_state'] = states[-1] if states else None
        if report['control_state'] != 'monitoring_manual_ready':
            report['errors'].append('control_state_not_idle')
        client = node.create_client(Trigger, '/fairino/stop_servo')
        report['stop_service_available'] = client.wait_for_service(timeout_sec=2)
        node.destroy_client(client)
        if not report['stop_service_available']:
            report['errors'].append('verified_stop_service_missing')
        for topic in ('/p1/servo_position_candidates',
                      '/fairino3mt_servo_position_controller/commands'):
            publishers = node.get_publishers_info_by_topic(topic)
            report[topic] = [p.node_name for p in publishers]
            expected = 'servo_node' if 'candidates' in topic else 'p1_path_guard'
            if len(publishers) != 1 or publishers[0].node_name != expected:
                report['errors'].append(topic + ':unexpected_publishers')
    except Exception as error:
        report['errors'].append(f'{type(error).__name__}:{error}')
    finally:
        report['passed'] = not report['errors']
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
