"""Launch red-cube perception and stage-two TrackTarget2D dry-run."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = Path(get_package_share_directory('fr3c_camera'))
    servo_share = Path(get_package_share_directory('fr3c_visual_servo'))
    color = LaunchConfiguration('target_color')
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_color', default_value='red',
            description='Perception and action color: red/yellow/blue/green.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                camera_share / 'launch/color_target_2d.launch.py')),
            launch_arguments={'target_color': color}.items(),
        ),
        Node(
            package='fr3c_visual_servo',
            executable='track_2d_action_server',
            name='track_2d_action_server',
            namespace='fr3c_visual_servo',
            output='screen',
            parameters=[
                str(servo_share / 'config/track_2d.yaml'),
                {'execute': False, 'perception_target_color': color},
            ],
        ),
    ])
