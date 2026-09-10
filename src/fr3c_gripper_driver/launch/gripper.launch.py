from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fr3c_gripper_driver'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=str(share / 'config/ydi_j22_ft232h.yaml')),
        DeclareLaunchArgument('backend', default_value='mock'),
        DeclareLaunchArgument('hardware_output_enabled', default_value='false'),
        Node(
            package='fr3c_gripper_driver',
            executable='gripper_node',
            name='ydi_j22_gripper',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'backend': LaunchConfiguration('backend'),
                    'hardware_output_enabled': LaunchConfiguration(
                        'hardware_output_enabled'),
                },
            ],
        ),
    ])
