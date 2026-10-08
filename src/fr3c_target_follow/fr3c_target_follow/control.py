"""Small bounded command client; status and stop results come from the driver."""
import argparse
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'teach', 'status'])
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Node('follow_control_client')
    latest = {}
    def receive(message):
        latest.update(state=json.loads(message.data), arrival=time.monotonic())
    node.create_subscription(String, '/target_follow/follow_state', receive, 10)
    def wait_for(predicate):
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return True
        return False
    try:
        if options.action == 'status':
            if not wait_for(lambda: 'state' in latest):
                raise RuntimeError('No follow_state received; check the Demo and ROS domain.')
            print(json.dumps(latest['state'], ensure_ascii=False, indent=2))
            return
        service = {'start': 'follow_start', 'stop': 'servo_stop', 'teach': 'teach_offset'}[options.action]
        client = node.create_client(Trigger, '/target_follow/' + service)
        if not wait_for(client.service_is_ready):
            raise RuntimeError('Service unavailable; check the Demo and ROS domain: ' + service)
        future = client.call_async(Trigger.Request())
        if not wait_for(future.done):
            raise RuntimeError('Service response timed out: ' + service)
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)
        print(result.message)
        if options.action == 'stop':
            acknowledged = time.monotonic()
            if not wait_for(lambda: latest.get('arrival', 0) > acknowledged and
                            latest['state']['status'] in ('stopped', 'fault') and
                            not latest['state'].get('following_requested') and
                            not latest['state'].get('motion_commands_enabled')):
                raise RuntimeError('Stop requested, but no stopped driver state received; check the driver.')
            state = latest['state']
            if state.get('stop_motion_code', 0) or state.get('servo_end_code', 0):
                raise RuntimeError('SDK stop/end failed: ' + json.dumps(state))
            print('Driver stopped. StopMotion=' + str(state.get('stop_motion_code', 'not active')) +
                  ' ServoMoveEnd=' + str(state.get('servo_end_code', 'not active')))
    except (RuntimeError, KeyboardInterrupt, ExternalShutdownException) as exc:
        node.get_logger().error(str(exc) or 'Interrupted')
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
