"""Read-only feedback, calibrated TF, and position PBVS preview; no actuator."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    path = Path(LaunchConfiguration('handeye_file').perform(context))
    data = yaml.safe_load(path.read_text())
    if (data.get('calibrated') is not True or data.get('handeye_accepted') is not True or
            data.get('parent_frame') != 'flange_link' or data.get('child_frame') != 'camera_link'):
        raise RuntimeError('accepted flange_link -> camera_link handeye calibration is required')
    x,y,z = map(str, data['xyz']); roll,pitch,yaw = map(str, data['rpy'])
    return [
        Node(package='tf2_ros', executable='static_transform_publisher', name='follow_handeye_tf',
             arguments=['--x',x,'--y',y,'--z',z,'--roll',roll,'--pitch',pitch,'--yaw',yaw,
                        '--frame-id','flange_link','--child-frame-id','camera_link']),
        Node(package='fr3c_follow_feedback', executable='robot_feedback',
             condition=IfCondition(LaunchConfiguration('start_feedback')), output='screen'),
        Node(package='fr3c_target_follow', executable='follow_preview', name='follow_preview',
             condition=IfCondition(LaunchConfiguration('start_preview')),
             parameters=[LaunchConfiguration('params_file'),
                         {'tcp_file': LaunchConfiguration('tcp_file'),
                          'transform_only': ParameterValue(LaunchConfiguration('transform_only'), value_type=bool)}], output='screen'),
    ]


def generate_launch_description():
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    robot = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    return LaunchDescription([
        DeclareLaunchArgument('start_feedback', default_value='true'),
        DeclareLaunchArgument('start_preview', default_value='true'),
        DeclareLaunchArgument('transform_only', default_value='false'),
        DeclareLaunchArgument('handeye_file', default_value=str(robot/'config/dabai_dcw2_handeye.yaml')),
        DeclareLaunchArgument('tcp_file', default_value=str(robot/'config/ydi_j22_tcp.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(follow/'config/preview.yaml')),
        OpaqueFunction(function=nodes),
    ])
