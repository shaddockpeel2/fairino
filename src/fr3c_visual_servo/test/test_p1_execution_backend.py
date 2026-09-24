from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
MOVEIT_CONFIG = WORKSPACE / 'src' / 'fairino3mt_v6_moveit2_config'


def test_both_arm_controllers_are_configured_but_idle_by_launch_default():
    controllers = yaml.safe_load(
        (MOVEIT_CONFIG / 'config/ros2_controllers.yaml').read_text(
            encoding='utf-8'))
    manager = controllers['controller_manager']['ros__parameters']
    assert manager['fairino3mt_execution_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController')
    assert manager['fairino3mt_servo_position_controller']['type'] == (
        'forward_command_controller/ForwardCommandController')
    servo = controllers['fairino3mt_servo_position_controller'][
        'ros__parameters']
    assert servo['joints'] == [f'joint{index}' for index in range(1, 7)]
    assert servo['interface_name'] == 'position'

    launch = (MOVEIT_CONFIG / 'launch/spawn_controllers.launch.py').read_text(
        encoding='utf-8')
    assert 'arguments=["fairino3mt_execution_controller", "--inactive"]' in launch
    assert ('arguments=["fairino3mt_servo_position_controller", '
            '"--inactive"]') in launch


def test_forward_servo_profile_is_position_only_and_preserves_jtc_baseline():
    forward = yaml.safe_load(
        (MOVEIT_CONFIG / 'config/fr3c_servo_forward.yaml').read_text(
            encoding='utf-8'))
    baseline = yaml.safe_load(
        (MOVEIT_CONFIG / 'config/fr3c_servo.yaml').read_text(
            encoding='utf-8'))
    assert forward['command_out_type'] == 'std_msgs/Float64MultiArray'
    assert forward['publish_joint_positions'] is True
    assert forward['publish_joint_velocities'] is False
    assert forward['publish_joint_accelerations'] is False
    assert forward['command_out_topic'] == (
        '/p1/servo_position_candidates')
    assert forward['publish_period'] == 0.020
    assert baseline['command_out_type'] == 'trajectory_msgs/JointTrajectory'
    assert baseline['command_out_topic'] == (
        '/fairino3mt_execution_controller/joint_trajectory')

    p1_launch = (WORKSPACE / 'src/fr3c_visual_servo/launch/'
                 'p1_servo_forward.launch.py').read_text(encoding='utf-8')
    legacy_launch = (WORKSPACE / 'src/fr3c_visual_servo/launch/'
                     'track_2d_servo.launch.py').read_text(encoding='utf-8')
    assert "config/fr3c_servo_forward.yaml" in p1_launch
    assert "config/fr3c_servo.yaml" in legacy_launch
    assert "config/fr3c_servo_forward.yaml" not in legacy_launch


def test_bridge_profile_has_separate_controller_and_command_watchdog():
    bridge = yaml.safe_load(
        (MOVEIT_CONFIG / 'config/control_bridge.yaml').read_text(
            encoding='utf-8'))['/fr3c_trajectory_control_bridge']['ros__parameters']
    assert bridge['execution_controller'] != bridge['servo_controller']
    assert bridge['servo_controller'] == (
        'fairino3mt_servo_position_controller')
    assert bridge['servo_command_topic'] == (
        '/fairino3mt_servo_position_controller/commands')
    assert 0.0 < bridge['servo_command_timeout_s'] < (
        bridge['tracking_heartbeat_timeout_s'])
