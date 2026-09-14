"""Start the complete physical first-grasp stack with one launch command."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from fairino_mtc_demo.p6_checks import invalidate_gate
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _invalidate_previous_gate(context):
    evidence_root = LaunchConfiguration('evidence_root').perform(context)
    gate = invalidate_gate(evidence_root, 'full_stack_startup')
    return [LogInfo(msg=f'Invalidated previous P6 gate: {gate}')]


def generate_launch_description():
    moveit_share = Path(get_package_share_directory(
        'fairino3mt_v6_moveit2_config'))
    task_share = Path(get_package_share_directory('fairino_mtc_demo'))
    camera_share = Path(get_package_share_directory('fr3c_camera'))
    gripper_share = Path(get_package_share_directory('fr3c_gripper_driver'))

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            moveit_share / 'launch/camera_moveit_demo.launch.py')),
        launch_arguments={
            'camera_config': LaunchConfiguration('camera_config'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'start_gripper_driver': 'true',
            'gripper_config': LaunchConfiguration('gripper_config'),
            'gripper_backend': LaunchConfiguration('gripper_backend'),
            'gripper_hardware_output_enabled': LaunchConfiguration(
                'gripper_hardware_output_enabled'),
            'publish_calibrated_tf': 'true',
            'hand_eye_file': LaunchConfiguration('hand_eye_file'),
        }.items(),
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            moveit_share / 'launch/first_grasp_perception.launch.py')),
        launch_arguments={
            'perception_config': LaunchConfiguration('perception_config'),
            'hand_eye_file': LaunchConfiguration('hand_eye_file'),
            'tcp_file': LaunchConfiguration('tcp_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_blue_cube_localizer')),
    )
    task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            task_share / 'launch/first_grasp.launch.py')),
        launch_arguments={
            'task_config': LaunchConfiguration('task_config'),
            'execute': LaunchConfiguration('first_grasp_execute'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_first_grasp')),
    )
    preflight = TimerAction(
        period=LaunchConfiguration('p6_start_delay_s'),
        actions=[Node(
            package='fairino_mtc_demo',
            executable='p6_preflight',
            name='fr3c_p6_startup_preflight',
            output='screen',
            arguments=[
                '--config', LaunchConfiguration('p6_config'),
                '--evidence-root', LaunchConfiguration('evidence_root'),
                '--sample-duration-s', '5.0',
                '--service-timeout-s', '5.0',
                '--confirm-workcell-clear',
                '--confirm-lift-clearance-50mm',
                '--confirm-cable-slack',
                '--confirm-stop-method-ready',
                '--confirm-no-external-motion-source',
            ],
        )],
        condition=IfCondition(LaunchConfiguration('run_p6_preflight')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config',
            default_value=str(camera_share / 'config/dabai_dcw2_rviz.yaml')),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(moveit_share / 'config/moveit_camera.rviz')),
        DeclareLaunchArgument(
            'gripper_config',
            default_value=str(gripper_share / 'config/ydi_j22_ft232h.yaml')),
        DeclareLaunchArgument('gripper_backend', default_value='ft232h'),
        DeclareLaunchArgument(
            'gripper_hardware_output_enabled', default_value='true'),
        DeclareLaunchArgument(
            'hand_eye_file',
            default_value=str(moveit_share / 'config/dabai_dcw2_handeye.yaml')),
        DeclareLaunchArgument(
            'tcp_file',
            default_value=str(moveit_share / 'config/ydi_j22_tcp.yaml')),
        DeclareLaunchArgument(
            'perception_config',
            default_value=str(camera_share / 'config/blue_cube_localizer.yaml')),
        DeclareLaunchArgument(
            'task_config',
            default_value=str(task_share / 'config/first_grasp.yaml')),
        DeclareLaunchArgument(
            'p6_config',
            default_value=str(task_share / 'config/p6_preflight.yaml')),
        DeclareLaunchArgument(
            'evidence_root',
            default_value='/home/industio/fairino_ws/evidence/P6'),
        DeclareLaunchArgument(
            'start_blue_cube_localizer', default_value='true',
            description='Start P3 blue-cube localization by default.'),
        DeclareLaunchArgument(
            'start_first_grasp', default_value='true',
            description='Start the P5 service node by default.'),
        DeclareLaunchArgument(
            'first_grasp_execute', default_value='true',
            description='Arm P5 for real execution; motion still requires /start.'),
        DeclareLaunchArgument(
            'run_p6_preflight', default_value='true',
            description='Run the startup P6 gate after the stack settles.'),
        DeclareLaunchArgument(
            'p6_start_delay_s', default_value='12.0',
            description='Delay before the startup P6 snapshot.'),
        OpaqueFunction(function=_invalidate_previous_gate),
        base_stack,
        perception,
        task,
        preflight,
        LogInfo(msg=(
            'Full grasp stack requested. Wait for P3 valid=true/stable=true and the P6 '
            'passed=true log, then call /fr3c_first_grasp/start.')),
    ])
