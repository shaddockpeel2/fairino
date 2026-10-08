"""Camera pose measurement only. An existing camera can be reused."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    camera = Path(get_package_share_directory('fr3c_camera'))
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_detector', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(follow / 'config/tag_pose.yaml')),
        DeclareLaunchArgument('camera_config', default_value=str(camera / 'config/dabai_dcw2_15fps.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(camera / 'launch/bench.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_camera')),
            launch_arguments={'config': LaunchConfiguration('camera_config'), 'probe': 'false'}.items()),
        Node(package='fr3c_target_follow', executable='tag_pose', name='tag_pose',
             condition=IfCondition(LaunchConfiguration('start_detector')),
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
