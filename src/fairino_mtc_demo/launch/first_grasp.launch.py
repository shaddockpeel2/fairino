"""Launch the one-shot P5 task against an already running MoveIt/perception stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package = 'fairino_mtc_demo'
    moveit_package = 'fairino3mt_v6_moveit2_config'
    config = Path(get_package_share_directory(package)) / 'config/first_grasp.yaml'
    moveit_config = MoveItConfigsBuilder(
        'fairino3mt_v6_robot', package_name=moveit_package
    ).to_dict()

    return LaunchDescription([
        DeclareLaunchArgument(
            'task_config',
            default_value=str(config),
            description='P5 geometry, ground-plane, grasp and execution-gate parameters.',
        ),
        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='Arm real motion/gripper steps. Default only plans and publishes preview.',
        ),
        Node(
            package=package,
            executable='first_grasp_node',
            name='fr3c_first_grasp',
            output='screen',
            parameters=[
                moveit_config,
                LaunchConfiguration('task_config'),
                {
                    'execute': ParameterValue(
                        LaunchConfiguration('execute'), value_type=bool),
                },
            ],
        ),
    ])
