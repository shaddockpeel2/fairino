import os
import queue
import threading
import time

import pytest


rclpy = pytest.importorskip('rclpy')
pytest.importorskip('fairino_msgs.action')
ColorActionBridge = pytest.importorskip(
    'fr3c_voice.action_bridge').ColorActionBridge
ActionServer = pytest.importorskip('rclpy.action').ActionServer
MultiThreadedExecutor = pytest.importorskip(
    'rclpy.executors').MultiThreadedExecutor
Node = pytest.importorskip('rclpy.node').Node
PickColor = pytest.importorskip('fairino_msgs.action').PickColor
Trigger = pytest.importorskip('std_srvs.srv').Trigger
Image = pytest.importorskip('sensor_msgs.msg').Image
String = pytest.importorskip('std_msgs.msg').String


def test_bridge_constructs_and_missing_server_fails_closed():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    node = ColorActionBridge()
    try:
        assert isinstance(node._action_clients, dict)
        assert isinstance(node._clients, list)  # rclpy.Node internal storage
        node.send(
            'no-server-smoke', 'pick_hold', 'red', server_timeout_s=0.01)
        event = node.events.get(timeout=1.0)
        assert event.kind == 'result'
        assert event.success is False
        assert not node.guard.snapshot().active
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_bridge_caches_a_fresh_rgb_frame_and_rejects_stale_frame():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    node = ColorActionBridge()
    try:
        message = Image()
        message.width = 2
        message.height = 1
        message.step = 6
        message.encoding = 'rgb8'
        message.data = bytes([255, 0, 0, 0, 255, 0])
        message.header.stamp.sec = 123
        node._on_color_image(message)
        assert node.latest_color_frame(max_age_s=1.0).ros_stamp_ns == 123_000_000_000
        node._latest_color_frame = node._latest_color_frame.__class__(
            **{**node._latest_color_frame.__dict__,
               'observed_monotonic': time.monotonic() - 2.0})
        with pytest.raises(RuntimeError, match='过期'):
            node.latest_color_frame(max_age_s=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_periodic_status_updates_refresh_snapshots_without_flooding_ui_queue():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    node = ColorActionBridge()
    try:
        node._on_state(String(data='{"state":"ready","detail":"same"}'))
        node._on_perception(String(data=(
            '{"inference_enabled":true,"reason":"stable_target_updated"}')))
        assert node.state_snapshot()[0]['state'] == 'ready'
        assert node.perception_snapshot()[0]['inference_enabled'] is True
        with pytest.raises(queue.Empty):
            node.events.get_nowait()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_bridge_waits_for_final_action_result_and_blocks_duplicate():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    server_node = Node('p4_mock_pick_server')

    def execute(goal_handle):
        time.sleep(0.15)
        goal_handle.succeed()
        result = PickColor.Result()
        result.success = True
        result.error_code = PickColor.Result.OK
        result.message = 'plan-only complete'
        result.color = goal_handle.request.color
        result.zone = goal_handle.request.zone
        result.support_color = goal_handle.request.support_color
        result.executed = False
        return result

    def acknowledge(_request, response):
        response.success = True
        response.message = 'released state synchronized'
        return response

    action_server = ActionServer(
        server_node, PickColor, ColorActionBridge.HOLD_ACTION, execute)
    release_service = server_node.create_service(
        Trigger, ColorActionBridge.RELEASE_ACK_SERVICE, acknowledge)
    bridge = ColorActionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        bridge.send('first', 'pick_hold', 'red', server_timeout_s=2.0)
        with pytest.raises(RuntimeError):
            bridge.send('duplicate', 'pick_sort', 'blue', server_timeout_s=0.01)
        events = []
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                event = bridge.events.get(timeout=0.2)
            except queue.Empty:
                continue
            events.append(event)
            if event.kind == 'result':
                break
        accepted = [event for event in events if event.kind == 'accepted']
        results = [event for event in events if event.kind == 'result']
        assert len(accepted) == 1
        assert len(results) == 1
        assert results[0].success is True
        assert results[0].executed is False
        assert results[0].color == 'red'
        assert results[0].zone == ''
        assert not bridge.guard.snapshot().active
        assert bridge.acknowledge_release_direct(service_timeout_s=2.0)
        release_events = []
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                event = bridge.events.get(timeout=0.5)
            except queue.Empty:
                continue
            release_events.append(event)
            if event.kind == 'release_ack_result':
                break
        release_results = [
            event for event in release_events
            if event.kind == 'release_ack_result']
        assert len(release_results) == 1
        assert release_results[0].success is True
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        server_node.destroy_service(release_service)
        action_server.destroy()
        bridge.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()


def test_bridge_sends_explicit_zone_only_to_pick_place_server():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    server_node = Node('pick_place_mock_server')
    requests = []

    def execute(goal_handle):
        requests.append((goal_handle.request.color, goal_handle.request.zone))
        goal_handle.succeed()
        result = PickColor.Result()
        result.success = True
        result.error_code = PickColor.Result.OK
        result.message = 'placed'
        result.color = goal_handle.request.color
        result.zone = goal_handle.request.zone
        result.support_color = goal_handle.request.support_color
        result.executed = True
        return result

    action_server = ActionServer(
        server_node, PickColor, ColorActionBridge.PLACE_ACTION, execute)
    bridge = ColorActionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        bridge.send(
            'place-red-in-d', 'pick_place', 'red', ' D ',
            server_timeout_s=2.0)
        result_event = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                event = bridge.events.get(timeout=0.2)
            except queue.Empty:
                continue
            if event.kind == 'result':
                result_event = event
                break
        assert requests == [('red', 'd')]
        assert result_event is not None
        assert result_event.success is True
        assert result_event.color == 'red'
        assert result_event.zone == 'd'
        with pytest.raises(ValueError, match='does not accept'):
            bridge.send('bad-sort-zone', 'pick_sort', 'red', 'd')
        with pytest.raises(ValueError, match='invalid placement zone'):
            bridge.send('bad-place-zone', 'pick_place', 'red', '')
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        action_server.destroy()
        bridge.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()


def test_bridge_sends_source_and_support_to_pick_stack_server():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    server_node = Node('pick_stack_mock_server')
    requests = []

    def execute(goal_handle):
        requests.append((
            goal_handle.request.color, goal_handle.request.support_color,
            goal_handle.request.zone))
        goal_handle.succeed()
        result = PickColor.Result()
        result.success = True
        result.error_code = PickColor.Result.OK
        result.message = 'stacked'
        result.color = goal_handle.request.color
        result.zone = goal_handle.request.zone
        result.support_color = goal_handle.request.support_color
        result.executed = True
        return result

    action_server = ActionServer(
        server_node, PickColor, ColorActionBridge.STACK_ACTION, execute)
    bridge = ColorActionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        bridge.send(
            'stack-red-on-blue', 'pick_stack', 'red', support_color='blue',
            server_timeout_s=2.0)
        result_event = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                event = bridge.events.get(timeout=0.2)
            except queue.Empty:
                continue
            if event.kind == 'result':
                result_event = event
                break
        assert requests == [('red', 'blue', '')]
        assert result_event is not None
        assert result_event.success is True
        assert result_event.color == 'red'
        assert result_event.support_color == 'blue'
        with pytest.raises(ValueError, match='must differ'):
            bridge.send(
                'same-color', 'pick_stack', 'red', support_color='red')
        with pytest.raises(ValueError, match='support color'):
            bridge.send(
                'missing-support', 'pick_stack', 'red')
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        action_server.destroy()
        bridge.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()
