"""
Start only MoveIt Servo with the P1 forward-position output profile.

The base MoveIt/ros2_control stack and trajectory control bridge must already
be running.  This launch starts no perception or Action server and publishes
no motion by itself; a guarded P1 diagnostic must acquire the Servo lease and
publish Twist commands.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Build the isolated P1 MoveIt Servo launch description."""
    share = Path(get_package_share_directory(
        'fairino3mt_v6_moveit2_config'))
    with (share / 'config/fr3c_servo_forward.yaml').open(
            encoding='utf-8') as stream:
        servo = yaml.safe_load(stream)

    servo['publish_period'] = ParameterValue(
        LaunchConfiguration('publish_period'), value_type=float)
    servo['low_latency_mode'] = ParameterValue(
        LaunchConfiguration('low_latency_mode'), value_type=bool)
    servo['smoothing_filter_plugin_name'] = ParameterValue(
        LaunchConfiguration('smoothing_filter_plugin'), value_type=str)

    moveit = MoveItConfigsBuilder(
        'fairino3mt_v6_robot',
        package_name='fairino3mt_v6_moveit2_config').to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_period', default_value='0.020',
            description='P1 matrix values: 0.020, 0.010 or 0.008 seconds.'),
        DeclareLaunchArgument(
            'low_latency_mode', default_value='false'),
        DeclareLaunchArgument(
            'smoothing_filter_plugin',
            default_value='online_signal_smoothing::ButterworthFilterPlugin',
            description='Humble requires a loadable smoothing plugin; empty is invalid.'),
        DeclareLaunchArgument(
            'butterworth_filter_coeff', default_value='1.5'),
        Node(
            package='fairino3mt_v6_moveit2_config',
            executable='servo_path_guard', name='p1_path_guard', output='screen',
            parameters=[moveit.robot_description, moveit.robot_description_semantic],
        ),
        Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            parameters=[
                {
                    'moveit_servo': servo,
                    'butterworth_filter_coeff': ParameterValue(
                        LaunchConfiguration('butterworth_filter_coeff'),
                        value_type=float),
                },
                moveit.robot_description,
                moveit.robot_description_semantic,
                moveit.robot_description_kinematics,
            ],
        ),
    ])
