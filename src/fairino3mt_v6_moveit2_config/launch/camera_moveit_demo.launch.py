"""Run MoveIt and DaBai in one RViz, with an optional calibrated hand-eye TF."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_tf(context):
    publish = LaunchConfiguration('publish_calibrated_tf').perform(context).lower()
    if publish not in ('1', 'true', 'yes', 'on'):
        preview = LaunchConfiguration('publish_mount_preview_tf').perform(context).lower()
        if preview not in ('1', 'true', 'yes', 'on'):
            return []

        # The URDF already contains a photo-estimated camera housing. The
        # housing axes are image-right, image-down and optical-forward, while
        # the driver's camera_link follows the ROS body-frame convention. This
        # rotation is the inverse of the nominal camera_link -> optical-frame
        # rotation, so the downstream optical frame points along housing +Z.
        # It is still only a visualization estimate, not hand-eye calibration.
        return [
            LogInfo(msg=(
                'Publishing visualization-only camera_housing_link -> camera_link TF. '
                'Point-cloud alignment is approximate until hand-eye calibration is supplied.'
            )),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='dabai_mount_preview_tf',
                output='screen',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--qx', '0.5', '--qy', '-0.5',
                    '--qz', '0.5', '--qw', '0.5',
                    '--frame-id', 'camera_housing_link', '--child-frame-id', 'camera_link',
                ],
            ),
        ]

    path_text = LaunchConfiguration('hand_eye_file').perform(context).strip()
    if not path_text:
        raise RuntimeError('publish_calibrated_tf=true requires hand_eye_file')
    path = Path(path_text)
    if not path.is_file():
        raise RuntimeError(f'hand-eye calibration file does not exist: {path}')

    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('calibrated') is not True:
        raise RuntimeError(f'hand-eye file is not marked calibrated: {path}')
    if data.get('handeye_accepted') is not True or data.get('status') != 'accepted':
        raise RuntimeError(f'hand-eye file is not accepted: {path}')
    if str(data.get('camera_serial', '')).strip() != 'CH82C5201C9':
        raise RuntimeError('hand-eye file is not for the deployed DaBai DCW2 serial')

    parent = str(data.get('parent_frame', '')).strip()
    child = str(data.get('child_frame', '')).strip()
    xyz = data.get('xyz')
    rpy = data.get('rpy')
    if not parent or not child or parent == child:
        raise RuntimeError('hand-eye parent_frame and child_frame must be distinct and non-empty')
    if child != 'camera_link':
        raise RuntimeError('hand-eye child_frame must be camera_link for the Orbbec TF tree')
    if not isinstance(xyz, list) or len(xyz) != 3 or not isinstance(rpy, list) or len(rpy) != 3:
        raise RuntimeError('hand-eye xyz and rpy must each contain three numbers')
    try:
        values = [float(value) for value in xyz + rpy]
    except (TypeError, ValueError) as exc:
        raise RuntimeError('hand-eye xyz and rpy must contain finite numbers') from exc
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError('hand-eye xyz and rpy must contain finite numbers')

    x, y, z, roll, pitch, yaw = values
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dabai_hand_eye_tf',
        output='screen',
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )]


def generate_launch_description():
    moveit_share = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    camera_share = Path(get_package_share_directory('fr3c_camera'))
    gripper_share = Path(get_package_share_directory('fr3c_gripper_driver'))

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_share / 'launch/demo.launch.py')),
        launch_arguments={'use_rviz': 'false'}.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / 'launch/bench.launch.py')),
        launch_arguments={
            'config': LaunchConfiguration('camera_config'),
            'probe': LaunchConfiguration('probe'),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_share / 'launch/moveit_rviz.launch.py')),
        launch_arguments={'rviz_config': LaunchConfiguration('rviz_config')}.items(),
    )
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gripper_share / 'launch/gripper.launch.py')),
        launch_arguments={
            'config': LaunchConfiguration('gripper_config'),
            'backend': LaunchConfiguration('gripper_backend'),
            'hardware_output_enabled': LaunchConfiguration(
                'gripper_hardware_output_enabled'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_gripper_driver')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config', default_value=str(camera_share / 'config/dabai_dcw2_rviz_15fps.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(moveit_share / 'config/moveit_camera.rviz'),
        ),
        DeclareLaunchArgument('probe', default_value='false'),
        DeclareLaunchArgument(
            'start_gripper_driver', default_value='true',
            description='Start the gripper server by default.',
        ),
        DeclareLaunchArgument(
            'gripper_config',
            default_value=str(gripper_share / 'config/ydi_j22_ft232h.yaml'),
        ),
        DeclareLaunchArgument('gripper_backend', default_value='mock'),
        DeclareLaunchArgument(
            'gripper_hardware_output_enabled', default_value='false',
            description='Must be true together with the hardware confirmation environment token.',
        ),
        DeclareLaunchArgument(
            'publish_calibrated_tf', default_value='true',
            description=(
                'Publish wrist/tool-to-camera_link only from a validated '
                'calibration file.'),
        ),
        DeclareLaunchArgument(
            'publish_mount_preview_tf', default_value='true',
            description=(
                'Attach camera_link to the photo-estimated camera_housing_link for RViz only. '
                'Automatically ignored when publish_calibrated_tf=true.'
            ),
        ),
        DeclareLaunchArgument(
            'hand_eye_file',
            default_value=str(moveit_share / 'config/dabai_dcw2_handeye.yaml'),
            description='YAML with calibrated, parent_frame, child_frame, xyz and rpy fields.',
        ),
        # Validate calibration before starting the camera or real ros2_control
        # processes when calibrated TF publication was requested.
        OpaqueFunction(function=_camera_tf),
        moveit_demo,
        camera,
        gripper,
        rviz,
    ])
