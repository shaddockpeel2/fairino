"""Read live joint state but publish Servo trajectories to an isolated topic."""

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
    share = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    with (share / 'config/fr3c_servo.yaml').open(encoding='utf-8') as stream:
        servo = yaml.safe_load(stream)
    servo['cartesian_command_in_topic'] = '/fr3c_stage4/shadow_twist'
    servo['joint_command_in_topic'] = '/fr3c_stage4/shadow_joint_cmd'
    servo['command_out_topic'] = '/fr3c_stage4/shadow_joint_trajectory'
    servo['status_topic'] = '/fr3c_stage4/shadow_status'
    servo['publish_period'] = ParameterValue(
        LaunchConfiguration('publish_period'), value_type=float)
    moveit = MoveItConfigsBuilder(
        'fairino3mt_v6_robot',
        package_name='fairino3mt_v6_moveit2_config').to_moveit_configs()
    coefficient = LaunchConfiguration('butterworth_filter_coeff')
    return LaunchDescription([
        DeclareLaunchArgument(
            'butterworth_filter_coeff', default_value='1.5'),
        DeclareLaunchArgument('publish_period', default_value='0.02'),
        Node(
            package='moveit_servo', executable='servo_node_main',
            name='stage4_shadow_servo', output='screen',
            parameters=[
                {'moveit_servo': servo,
                 'butterworth_filter_coeff': ParameterValue(
                     coefficient, value_type=float)},
                moveit.robot_description,
                moveit.robot_description_semantic,
                moveit.robot_description_kinematics,
            ],
        ),
    ])
