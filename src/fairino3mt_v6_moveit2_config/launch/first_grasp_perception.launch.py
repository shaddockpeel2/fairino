"""Start P3 localization with the accepted hand-eye and TCP calibration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument(
            'top_pose_z_compensation_m', default_value='-0.004',
            description='Signed base-Z correction added to every accepted top pose.'),
        DeclareLaunchArgument('detection_mode', default_value='blue'),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=str(camera_share / 'models/best.rknn')),
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
                    'top_pose_z_compensation_m': ParameterValue(
                        LaunchConfiguration('top_pose_z_compensation_m'),
                        value_type=float),
                    'detection_mode': LaunchConfiguration('detection_mode'),
                    'yolo_model_path': LaunchConfiguration('yolo_model_path'),
                },
            ],
        ),
    ])
