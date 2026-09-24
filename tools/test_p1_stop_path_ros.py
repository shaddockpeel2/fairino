"""Real ROS/FCL guard + real bridge, simulated controller and measured feedback.

Only runs in localhost ROS_DOMAIN_ID=97. Never loads the FAIRINO hardware plugin.
"""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_live_guard_and_verified_stop_transaction(tmp_path):
    if (os.environ.get('ROS_DOMAIN_ID') != '97'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        pytest.skip('requires isolated localhost ROS_DOMAIN_ID=97')
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import Float64MultiArray, String
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from fairino_msgs.srv import ControlLease
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene
    from moveit_msgs.srv import GetPlanningScene
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose

    urdf = (ROOT / 'evidence/P1_execution/reanalysis_robot_20260923.urdf').read_text()
    srdf = (ROOT / 'src/fairino3mt_v6_moveit2_config/config/fairino3mt_v6_robot.srdf').read_text()
    config = tmp_path / 'guard.yaml'
    config.write_text(yaml.safe_dump({'p1_path_guard': {'ros__parameters': {
        'robot_description': urdf, 'robot_description_semantic': srdf,
    }}}))
    q = json.loads((ROOT / 'evidence/P1_execution/forward_020_xplus_0p5s.json').read_text())[
        'joint_states'][0]['positions_rad']
    names = [f'joint{i}' for i in range(1, 7)] + ['gripper_command_joint']
    positions = q + [0.0]
    spec = importlib.util.spec_from_file_location(
        'p1_bridge_live_test', ROOT / 'src/fairino3mt_v6_moveit2_config/scripts/trajectory_control_bridge.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    mock = rclpy.create_node('p1_mock_stack')
    bridge = module.TrajectoryControlBridge()
    client_node = rclpy.create_node('p1_test_client')
    thread = None
    process = None
    log = (tmp_path / 'guard.log').open('w+')
    running = False
    invalid = False
    states = {'joint_state_broadcaster': 'active',
              'fairino3mt_execution_controller': 'inactive',
              'fairino3mt_servo_position_controller': 'inactive'}
    events, outputs, reports = [], [], []
    joint_pub = mock.create_publisher(JointState, '/joint_states', 10)
    candidate_pub = mock.create_publisher(Float64MultiArray, '/p1/servo_position_candidates', 1)

    def joints():
        return JointState(header=__import__('std_msgs.msg', fromlist=['Header']).Header(
            stamp=mock.get_clock().now().to_msg()), name=names, position=positions, velocity=[0.0] * 7)

    def tick():
        joint_pub.publish(joints())
        if running:
            candidate_pub.publish(Float64MultiArray(data=[float('nan')] * 6 if invalid else q))

    mock.create_timer(0.02, tick)

    def trigger_start(_req, res):
        nonlocal running
        running = True
        res.success = True
        return res

    def trigger_stop(_req, res):
        nonlocal running
        events.append('servo_generator_stop')
        running = False
        res.success = True
        return res

    mock.create_service(Trigger, '/servo_node/start_servo', trigger_start)
    mock.create_service(Trigger, '/servo_node/stop_servo', trigger_stop)

    def list_controllers(_req, res):
        res.controller = [ControllerState(name=name, state=state) for name, state in states.items()]
        return res

    def switch(req, res):
        for name in req.deactivate_controllers:
            states[name] = 'inactive'
            events.append('hardware_interface_revoked')
        for name in req.activate_controllers:
            states[name] = 'active'
        res.ok = True
        return res

    mock.create_service(ListControllers, '/controller_manager/list_controllers', list_controllers)
    mock.create_service(SwitchController, '/controller_manager/switch_controller', switch)
    scene = PlanningScene()
    scene.robot_model_name = 'fairino3mt_v6_robot'
    pairs = [(entry.get('link1'), entry.get('link2'))
             for entry in ET.fromstring(srdf).findall('disable_collisions')]
    acm_names = sorted({name for pair in pairs for name in pair})
    scene.allowed_collision_matrix.entry_names = acm_names
    scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[(a, b) in pairs or (b, a) in pairs for b in acm_names])
        for a in acm_names]
    ground = CollisionObject(id='rviz_ground_plane', operation=CollisionObject.ADD)
    ground.header.frame_id = 'base_link'
    ground.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.02])]
    ground_pose = Pose()
    ground_pose.orientation.w = 1.0
    ground_pose.position.z = -0.011
    ground.primitive_poses = [ground_pose]
    scene.world.collision_objects = [ground]

    def get_scene(_req, res):
        scene.robot_state.joint_state = joints()
        res.scene = scene
        return res

    mock.create_service(GetPlanningScene, '/get_planning_scene', get_scene)
    mock.create_subscription(Float64MultiArray, '/fairino3mt_servo_position_controller/commands',
                             lambda msg: outputs.append(msg.data), 10)
    mock.create_subscription(String, '/fairino/stop_report',
                             lambda msg: reports.append(json.loads(msg.data)), 10)

    def call(kind, service, request=None):
        client = client_node.create_client(kind, service)
        try:
            assert client.wait_for_service(timeout_sec=10), service
            future = client.call_async(request or kind.Request())
            rclpy.spin_until_future_complete(client_node, future, timeout_sec=10)
            assert future.done(), service
            return future.result()
        finally:
            client_node.destroy_client(client)

    try:
        executor.add_node(mock)
        executor.add_node(bridge)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        process = subprocess.Popen([
            str(ROOT / 'install/fairino3mt_v6_moveit2_config/lib/fairino3mt_v6_moveit2_config/servo_path_guard'),
            '--ros-args', '--params-file', str(config)], stdout=log, stderr=log)
        deadline = time.monotonic() + 15
        ready = None
        while time.monotonic() < deadline:
            ready = call(Trigger, '/p1/path_guard/ready')
            if ready.success:
                break
            time.sleep(0.1)
        assert ready.success, ready.message
        request = ControlLease.Request(operation=ControlLease.Request.ACQUIRE,
                                       owner='SERVO_TRACKING', heartbeat_timeout_s=2.0)
        acquired = call(ControlLease, '/fairino/control_lease', request)
        assert acquired.success, acquired.message
        deadline = time.monotonic() + 1
        while not outputs and time.monotonic() < deadline:
            time.sleep(0.02)
        assert outputs, 'No validated target forwarded'
        stopped = call(Trigger, '/fairino/stop_servo')
        assert stopped.success, stopped.message
        assert events.index('hardware_interface_revoked') < events.index('servo_generator_stop')
        assert reports[-1]['stationary_verified'] and reports[-1]['released']
        # Reacquire, then malformed target must never escape the guard.
        acquired = call(ControlLease, '/fairino/control_lease', request)
        assert acquired.success
        invalid = True
        deadline = time.monotonic() + 3
        while len(reports) < 2 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert len(reports) == 2
        assert reports[-1]['reason'].startswith('path_guard:')
        assert reports[-1]['stationary_verified'] and reports[-1]['released']
        assert all(all(value == value for value in output) for output in outputs)
        assert not call(Trigger, '/p1/path_guard/ready').success
        assert states['fairino3mt_servo_position_controller'] == 'inactive'
    finally:
        if process:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        executor.shutdown()
        if thread:
            thread.join(timeout=3)
        client_node.destroy_node()
        bridge.destroy_node()
        mock.destroy_node()
        rclpy.shutdown()
        log.seek(0)
        print(log.read()[-6000:])
        log.close()
