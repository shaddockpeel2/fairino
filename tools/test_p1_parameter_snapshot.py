import copy

from p1_parameter_snapshot import PARAMETERS, forward_profile_errors


def valid_snapshot():
    return {'nodes': {
        '/servo_node': {'error': None, 'parameters': {
            'moveit_servo.command_in_type': 'speed_units',
            'moveit_servo.command_out_type': 'std_msgs/Float64MultiArray',
            'moveit_servo.command_out_topic':
                '/p1/servo_position_candidates',
            'moveit_servo.publish_joint_positions': True,
            'moveit_servo.publish_joint_velocities': False,
            'moveit_servo.check_collisions': True,
            'moveit_servo.publish_period': 0.02,
            'robot_description': {'length': 10, 'sha256': 'example'},
            'robot_description_semantic': {'length': 10, 'sha256': 'example'},
        }},
        '/controller_manager': {'error': None, 'parameters': {'update_rate': 125}},
        '/fairino3mt_servo_position_controller': {'error': None, 'parameters': {
            'joints': [f'joint{i}' for i in range(1, 7)],
            'interface_name': 'position',
        }},
        '/fr3c_trajectory_control_bridge': {'error': None, 'parameters': {
            'servo_controller': 'fairino3mt_servo_position_controller',
            'servo_command_topic': '/fairino3mt_servo_position_controller/commands',
        }},
    }}


def test_snapshot_validation_rejects_missing_and_wrong_semantics():
    assert forward_profile_errors(valid_snapshot()) == []
    assert len(forward_profile_errors({})) >= len(PARAMETERS)
    for key, value in (
        ('moveit_servo.command_in_type', 'unitless'),
        ('moveit_servo.publish_period', float('nan')),
        ('moveit_servo.publish_period', 0.1),
        ('moveit_servo.check_collisions', False),
        ('moveit_servo.check_collisions', 1),
        ('robot_description', None),
    ):
        snapshot = copy.deepcopy(valid_snapshot())
        snapshot['nodes']['/servo_node']['parameters'][key] = value
        assert forward_profile_errors(snapshot), key
    snapshot = valid_snapshot()
    snapshot['nodes']['/fairino3mt_servo_position_controller']['parameters'][
        'joints'].reverse()
    assert forward_profile_errors(snapshot)
