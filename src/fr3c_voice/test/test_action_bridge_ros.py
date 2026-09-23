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
MoveToPose = pytest.importorskip('fairino_msgs.action').MoveToPose
GripperCommand = pytest.importorskip('control_msgs.action').GripperCommand
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


def test_bridge_manual_gripper_exposes_only_calibrated_open_and_close():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    server_node = Node('manual_gripper_mock_server')
    positions = []

    def execute(goal_handle):
        positions.append(goal_handle.request.command.position)
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = goal_handle.request.command.position
        result.reached_goal = True
        return result

    action_server = ActionServer(
        server_node, GripperCommand, ColorActionBridge.GRIPPER_ACTION, execute)
    bridge = ColorActionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        deadline = time.monotonic() + 2.0
        while not bridge.gripper_server_ready() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert bridge.command_gripper('open-request', 'open')
        events = []
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                event = bridge.events.get(timeout=0.2)
            except queue.Empty:
                continue
            events.append(event)
            if event.kind == 'gripper_result':
                break
        results = [event for event in events if event.kind == 'gripper_result']
        assert len(results) == 1
        assert results[0].success is True
        assert positions == [ColorActionBridge.GRIPPER_OPEN_POSITION_M]
        assert not bridge.gripper_command_active()
        with pytest.raises(ValueError, match='unsupported gripper command'):
            bridge.command_gripper('bad-request', 'half-open')
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        action_server.destroy()
        bridge.destroy_node()
        server_node.destroy_node()
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


def test_bridge_sends_empty_goal_to_pick_stack_auto_server():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    server_node = Node('pick_auto_stack_mock_server')
    requests = []

    def execute(goal_handle):
        requests.append((
            goal_handle.request.color, goal_handle.request.zone,
            goal_handle.request.support_color))
        # Give the client time to attach its feedback subscription before the
        # first feedback is published.
        time.sleep(0.3)
        feedback = PickColor.Feedback()
        feedback.phase = 'stacking'
        feedback.detail = '正在按红、黄、蓝、绿顺序自动码垛。'
        goal_handle.publish_feedback(feedback)
        time.sleep(0.3)
        goal_handle.succeed()
        result = PickColor.Result()
        result.success = True
        result.error_code = PickColor.Result.OK
        result.message = '自动码垛完成，共码放3层。'
        result.color = goal_handle.request.color
        result.zone = goal_handle.request.zone
        result.support_color = goal_handle.request.support_color
        result.executed = True
        return result

    action_server = ActionServer(
        server_node, PickColor, ColorActionBridge.AUTO_STACK_ACTION, execute)
    bridge = ColorActionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        bridge.send(
            'auto-stack-all', 'pick_auto_stack', '', server_timeout_s=2.0)
        feedback_event = None
        result_event = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                event = bridge.events.get(timeout=0.2)
            except queue.Empty:
                continue
            if event.kind == 'feedback' and feedback_event is None:
                feedback_event = event
            if event.kind == 'result':
                result_event = event
                break
        assert requests == [('', '', '')]
        assert feedback_event is not None
        assert feedback_event.phase == 'stacking'
        assert feedback_event.detail == '正在按红、黄、蓝、绿顺序自动码垛。'
        assert result_event is not None
        assert result_event.success is True
        assert result_event.error_code == int(PickColor.Result.OK)
        assert result_event.detail == '自动码垛完成，共码放3层。'
        assert result_event.color == ''
        assert result_event.zone == ''
        assert result_event.support_color == ''
        assert not bridge.guard.snapshot().active
        with pytest.raises(ValueError, match='does not accept'):
            bridge.send('auto-stack-red', 'pick_auto_stack', 'red')
        with pytest.raises(ValueError, match='does not accept'):
            bridge.send('auto-stack-zone', 'pick_auto_stack', '', 'd')
        with pytest.raises(ValueError, match='does not accept'):
            bridge.send(
                'auto-stack-support', 'pick_auto_stack', '', '',
                support_color='blue')
        assert not bridge.guard.snapshot().active
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        action_server.destroy()
        bridge.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()


def test_bridge_sends_xyz_goal_and_reports_collision_review_feedback():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    server_node = Node('move_to_pose_mock_server')
    requests = []

    def execute(goal_handle):
        requests.append((goal_handle.request.x, goal_handle.request.y,
                         goal_handle.request.z))
        feedback = MoveToPose.Feedback()
        feedback.phase = 'collision_check'
        feedback.detail = 'collision-free'
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result = MoveToPose.Result()
        result.success = True
        result.error_code = MoveToPose.Result.OK
        result.message = 'arrived'
        result.executed = True
        return result

    action_server = ActionServer(
        server_node, MoveToPose, ColorActionBridge.MOVE_TO_POSE_ACTION, execute)
    bridge = ColorActionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        bridge.send_pose('xyz', 0.10, -0.40, 0.35, server_timeout_s=2.0)
        events = []
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                event = bridge.events.get(timeout=0.2)
            except queue.Empty:
                continue
            events.append(event)
            if event.kind == 'pose_result':
                break
        assert requests == [(0.10, -0.40, 0.35)]
        assert any(event.kind == 'pose_accepted' for event in events)
        assert any(event.kind == 'pose_feedback' for event in events)
        result = [event for event in events if event.kind == 'pose_result'][0]
        assert result.success is True
        assert result.executed is True
        assert not bridge.guard.snapshot().active
        with pytest.raises(ValueError, match='finite'):
            bridge.send_pose('bad', float('nan'), 0.0, 0.0)
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        action_server.destroy()
        bridge.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('service_success', [True, False])
def test_tabletop_reset_reports_backend_result(service_success):
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    server = Node('fake_tabletop_reset')
    calls = []

    def reset(request, response):
        calls.append(request)
        response.success = service_success
        response.message = 'cleared' if service_success else 'task active'
        return response

    server.create_service(Trigger, ColorActionBridge.RESET_TABLETOP_SERVICE, reset)
    bridge = ColorActionBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    executor.add_node(bridge)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert bridge.reset_tabletop_direct(service_timeout_s=2.0)
        event = bridge.events.get(timeout=3.0)
        assert event.kind == 'tabletop_reset_result'
        assert event.success is service_success
        assert event.detail == ('cleared' if service_success else 'task active')
        assert len(calls) == 1
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        bridge.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_tabletop_reset_missing_service_and_busy_gripper_fail_closed():
    os.environ.setdefault('ROS_DOMAIN_ID', '231')
    rclpy.init()
    bridge = ColorActionBridge()
    try:
        assert not bridge.reset_tabletop_direct(service_timeout_s=0.01)
        assert bridge.events.get(timeout=1.0).success is False
        bridge.gripper_command_active = lambda: True
        assert not bridge.reset_tabletop_direct(service_timeout_s=0.01)
        event = bridge.events.get(timeout=1.0)
        assert event.success is False
        assert '仍在运行' in event.detail
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
