"""Start P3 blue-cube localization against an already-running RGB-D/robot stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fr3c_camera'))
    default_config = str(share / 'config/blue_cube_localizer.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument(
            'hand_eye_file', default_value='',
            description='Required accepted flange-to-camera_link calibration YAML.',
        ),
        DeclareLaunchArgument(
            'tcp_file', default_value='',
            description='Required calibrated flange_link-to-grasp_frame TCP YAML.',
        ),
        DeclareLaunchArgument('fixed_yaw_rad', default_value='0.0'),
        Node(
            package='fr3c_camera',
            executable='blue_cube_localizer',
            name='blue_cube_localizer',
            namespace='perception',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'hand_eye_file': LaunchConfiguration('hand_eye_file'),
                    'tcp_file': LaunchConfiguration('tcp_file'),
                    'fixed_yaw_rad': LaunchConfiguration('fixed_yaw_rad'),
                },
            ],
        ),
    ])
