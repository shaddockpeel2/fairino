"""Launch the validated DaBai streams, RGB point cloud, and camera-only RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = Path(get_package_share_directory('fr3c_camera'))

    config = LaunchConfiguration('config')
    probe = LaunchConfiguration('probe')
    rviz_config = LaunchConfiguration('rviz_config')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / 'launch/bench.launch.py')),
        launch_arguments={'config': config, 'probe': probe}.items(),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='camera_rviz',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=str(camera_share / 'config/dabai_dcw2_rviz.yaml'),
            description='Camera parameter YAML. The default enables RGB point cloud.',
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(camera_share / 'rviz/dabai_dcw2.rviz'),
        ),
        DeclareLaunchArgument('probe', default_value='false'),
        camera,
        rviz,
    ])
