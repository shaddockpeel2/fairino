"""Report the nearest self/scene collision distances without commanding motion."""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'fairino3mt_v6_robot',
        package_name='fairino3mt_v6_moveit2_config',
    ).to_moveit_configs()

    return LaunchDescription([
        Node(
            package='fairino3mt_v6_moveit2_config',
            executable='collision_distance_diagnostic',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        ),
    ])
