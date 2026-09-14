"""Start P3 localization with the accepted hand-eye and TCP calibration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = Path(get_package_share_directory('fr3c_camera'))
    moveit_share = Path(
        get_package_share_directory('fairino3mt_v6_moveit2_config'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'perception_config',
            default_value=str(camera_share / 'config/blue_cube_localizer.yaml'),
        ),
        DeclareLaunchArgument(
            'hand_eye_file',
            default_value=str(moveit_share / 'config/dabai_dcw2_handeye.yaml'),
        ),
        DeclareLaunchArgument(
            'tcp_file',
            default_value=str(moveit_share / 'config/ydi_j22_tcp.yaml'),
        ),
        DeclareLaunchArgument('fixed_yaw_rad', default_value='0.0'),
        Node(
            package='fr3c_camera',
            executable='blue_cube_localizer',
            name='blue_cube_localizer',
            namespace='perception',
            output='screen',
            parameters=[
                LaunchConfiguration('perception_config'),
                {
                    'hand_eye_file': LaunchConfiguration('hand_eye_file'),
                    'tcp_file': LaunchConfiguration('tcp_file'),
                    'fixed_yaw_rad': LaunchConfiguration('fixed_yaw_rad'),
                },
            ],
        ),
    ])
