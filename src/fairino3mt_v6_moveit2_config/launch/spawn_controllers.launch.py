from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory(
        "fairino3mt_v6_moveit2_config"))
    # The public fairino3mt_controller action is provided by the bridge below.
    # The real ros2_control controller stays inactive until an execution goal
    # arrives, leaving the FAIRINO flange drag button usable while MoveIt/RViz
    # are only planning or monitoring.
    state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    execution_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fairino3mt_execution_controller", "--inactive"],
        output="screen",
    )
    trajectory_bridge = Node(
        package="fairino3mt_v6_moveit2_config",
        executable="trajectory_control_bridge.py",
        output="screen",
        parameters=[str(package_share / "config/control_bridge.yaml")],
    )

    return LaunchDescription([
        state_broadcaster,
        RegisterEventHandler(OnProcessExit(
            target_action=execution_controller,
            on_exit=[trajectory_bridge],
        )),
        execution_controller,
    ])
