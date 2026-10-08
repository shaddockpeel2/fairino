"""Stage 4: reuse image-time conversion; the C++ driver owns feedback and motion."""
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def collision_enabled(context):
    """CLI overrides the selected parameter file; auto inherits that file."""
    selection = LaunchConfiguration('collision_enabled').perform(context).lower()
    if selection != 'auto':
        return selection == 'true'
    config = yaml.safe_load(Path(LaunchConfiguration('driver_params_file').perform(context)).read_text()) or {}
    enabled = False
    for key in ('/**', 'position_follow', '/position_follow'):
        enabled = config.get(key, {}).get('ros__parameters', {}).get('collision_enabled', enabled)
    if not isinstance(enabled, bool):
        raise RuntimeError('collision_enabled must be a YAML boolean (true/false)')
    return enabled


def driver(context):
    path = Path(LaunchConfiguration('tcp_file').perform(context))
    tcp = yaml.safe_load(path.read_text())['ydi_j22_tcp']
    if not tcp.get('calibrated') or tcp['parent_frame'] != 'flange_link' or tcp['child_frame'] != 'grasp_frame':
        raise RuntimeError('calibrated flange_link -> grasp_frame TCP is required')
    enabled = collision_enabled(context)
    collision = {'collision_enabled': enabled}
    if enabled:
        robot = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
        collision.update({
            'robot_description': xacro.process_file(str(robot/'config/fairino3mt_v6_robot.urdf.xacro'), mappings={
                'initial_positions_file': str(robot/'config/initial_positions.yaml'),
                'hardware_plugin': 'mock_components/GenericSystem',
            }).toxml(),
            'robot_description_semantic': (robot/'config/fairino3mt_v6_robot.srdf').read_text(),
        })
    return [Node(package='fr3c_follow_driver', executable='position_follow', name='position_follow',
                 parameters=[LaunchConfiguration('driver_params_file'), {
                     'tcp_xyz_m': [float(v) for v in tcp['xyz_m']],
                     'tcp_rpy_rad': [float(v) for v in tcp['rpy_rad']],
                     **collision,
                     'mock': ParameterValue(LaunchConfiguration('mock'), value_type=bool),
                     'robot_ip': LaunchConfiguration('robot_ip'),
                     'output_dir': LaunchConfiguration('output_dir'),
                 }], output='screen')]


def generate_launch_description():
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    robot = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    execution = Path(get_package_share_directory('fr3c_follow_driver'))
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('collision_enabled', default_value='auto', choices=['auto', 'true', 'false'],
                              description='true/false overrides config; auto uses collision_enabled in driver_params_file'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.58.2'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('tcp_file', default_value=str(robot/'config/ydi_j22_tcp.yaml')),
        DeclareLaunchArgument('handeye_file', default_value=str(robot/'config/dabai_dcw2_handeye.yaml')),
        DeclareLaunchArgument('driver_params_file', default_value=str(execution/'config/position_follow.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(follow/'launch/preview.launch.py')),
                                 launch_arguments={
                                     'start_feedback': 'false', 'transform_only': 'true',
                                     'tcp_file': LaunchConfiguration('tcp_file'),
                                     'handeye_file': LaunchConfiguration('handeye_file'),
                                 }.items()),
        OpaqueFunction(function=driver),
    ])
