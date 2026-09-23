"""Start the retained V1 JTC red-cube tracking stack.

The base MoveIt/ros2_control stack (including the trajectory control bridge)
must already be running.  This launch never sends a motion command by itself;
an explicit TrackTarget2D goal is still required.

P1 uses p1_servo_forward.launch.py instead.  Do not use this V1 launch with
the V2 forward-controller bridge for real motion.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    camera_share = Path(get_package_share_directory('fr3c_camera'))
    tracking_share = Path(get_package_share_directory('fr3c_visual_servo'))
    moveit_share = Path(get_package_share_directory(
        'fairino3mt_v6_moveit2_config'))
    moveit_config = MoveItConfigsBuilder(
        'fairino3mt_v6_robot',
        package_name='fairino3mt_v6_moveit2_config').to_moveit_configs()
    with (moveit_share / 'config/fr3c_servo.yaml').open(
            encoding='utf-8') as stream:
        servo_config = yaml.safe_load(stream)
    servo_config['publish_period'] = ParameterValue(
        LaunchConfiguration('servo_publish_period'), value_type=float)
    servo_params = {'moveit_servo': servo_config}

    color = LaunchConfiguration('target_color')
    execute = LaunchConfiguration('execute')
    return LaunchDescription([
        DeclareLaunchArgument('observation_stale_s', default_value='0.40'),
        DeclareLaunchArgument(
            'dropout_grace_s', default_value='0.16',
            description='Brief target detection gap allowance, capped at 5 mm nominal travel.'),
        DeclareLaunchArgument(
            'lost_timeout_s', default_value='0.0',
            description='0 waits at zero speed for target recovery until cancel or goal expiry.'),
        DeclareLaunchArgument(
            'max_speed_mps', default_value='0.005',
            description='Tracking Twist speed limit. Default 0.005 m/s; '
                        '0.075 m/s is available only after staged direction '
                        'and stop validation.'),
        DeclareLaunchArgument(
            'gain_mps', default_value='0.080',
            description='Camera-plane proportional gain.'),
        DeclareLaunchArgument(
            'max_acceleration_mps2', default_value='0.030',
            description='Tracking acceleration limit, at most 0.090 m/s^2.'),
        DeclareLaunchArgument(
            'target_color', default_value='red',
            description='Perception and action color.'),
        DeclareLaunchArgument(
            'execute', default_value='false',
            description=(
                'Enable leased Servo output after direction verification. '
                'An explicit Action goal is still required.')),
        DeclareLaunchArgument(
            'start_perception', default_value='true'),
        DeclareLaunchArgument(
            'start_action_server', default_value='true',
            description='Set false to launch only MoveIt Servo for stage-four pulses.'),
        DeclareLaunchArgument(
            'servo_publish_period', default_value='0.10',
            description='Servo calculation period in seconds.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                camera_share / 'launch/color_target_2d.launch.py')),
            launch_arguments={'target_color': color, 'observation_stale_s':
                              LaunchConfiguration('observation_stale_s')}.items(),
            condition=IfCondition(LaunchConfiguration('start_perception')),
        ),
        Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        ),
        Node(
            package='fr3c_visual_servo',
            executable='track_2d_action_server',
            name='track_2d_action_server',
            namespace='fr3c_visual_servo',
            output='screen',
            parameters=[
                str(tracking_share / 'config/track_2d.yaml'),
                {
                    'execute': ParameterValue(execute, value_type=bool),
                    'perception_target_color': color,
                    'observation_stale_s': ParameterValue(
                        LaunchConfiguration('observation_stale_s'), value_type=float),
                    'dropout_grace_s': ParameterValue(
                        LaunchConfiguration('dropout_grace_s'), value_type=float),
                    'lost_timeout_s': ParameterValue(
                        LaunchConfiguration('lost_timeout_s'), value_type=float),
                    'max_speed_mps': ParameterValue(
                        LaunchConfiguration('max_speed_mps'), value_type=float),
                    'gain_x_mps': ParameterValue(
                        LaunchConfiguration('gain_mps'), value_type=float),
                    'gain_y_mps': ParameterValue(
                        LaunchConfiguration('gain_mps'), value_type=float),
                    'max_acceleration_mps2': ParameterValue(
                        LaunchConfiguration('max_acceleration_mps2'),
                        value_type=float),
                },
            ],
            condition=IfCondition(LaunchConfiguration('start_action_server')),
        ),
    ])
