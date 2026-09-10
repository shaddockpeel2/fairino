from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

robotname = "fairino3mt_v6_robot"
packagename = "fairino3mt_v6_moveit2_config"


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        robotname, package_name=packagename
    ).to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="fairino_mtc_demo",
        executable="mtc_node",
        output="screen",
        #prefix=['gdb -ex run --args'],
        parameters=[
            moveit_config,
            {
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
                "cycle_count": ParameterValue(
                    LaunchConfiguration("cycle_count"), value_type=int
                ),
                "tip_frame": LaunchConfiguration("tip_frame"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description="Execute planned trajectories. Keep false until mock planning is verified.",
            ),
            DeclareLaunchArgument(
                "cycle_count",
                default_value="1",
                description="Number of forward/reverse pick-place cycles.",
            ),
            DeclareLaunchArgument(
                "tip_frame",
                default_value="grasp_frame",
                description="Cartesian IK/attach frame. Defaults to the YDI-J22 fingertip TCP.",
            ),
            pick_place_demo,
        ]
    )
