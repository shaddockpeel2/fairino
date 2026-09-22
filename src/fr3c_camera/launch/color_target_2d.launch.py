"""Start color-only 2-D cube perception against an existing camera stream."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('fr3c_camera'))
    default_config = str(share / 'config/color_target_2d.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('observation_stale_s', default_value='0.40'),
        DeclareLaunchArgument(
            'target_color', default_value='red',
            description='Target cube color: red, yellow, blue, or green.'),
        Node(
            package='fr3c_camera',
            executable='color_target_2d',
            name='color_target_2d',
            namespace='perception',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'target_color': LaunchConfiguration('target_color'),
                 'stale_after_s': ParameterValue(
                     LaunchConfiguration('observation_stale_s'), value_type=float)},
            ],
        ),
    ])
