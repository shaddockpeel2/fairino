"""Stage 9 pilot: paired full pose -> sole native SDK owner, starts idle."""
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from fr3c_target_follow.demo_graph import discover, pose_follow_source_plan


def component_exit(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason='Pose follow component exited; closing owned nodes and driver.'))]


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mock = value('mock') == 'true'
    graph = discover()
    plan = pose_follow_source_plan(graph, mock=mock, viewer=value('viewer') == 'true')
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    robot = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    params = yaml.safe_load(Path(value('driver_params_file')).read_text())['pose_follow']['ros__parameters']
    enabled = params.get('collision_enabled', False) if value('collision_enabled') == 'auto' else value('collision_enabled') == 'true'
    if not isinstance(enabled, bool):
        raise RuntimeError('collision_enabled must be a YAML boolean')
    tcp = yaml.safe_load(Path(value('tcp_file')).read_text())['ydi_j22_tcp']
    if tcp.get('calibrated') is not True or tcp.get('parent_frame') != 'flange_link' or tcp.get('child_frame') != 'grasp_frame':
        raise RuntimeError('calibrated flange_link -> grasp_frame TCP required')
    overrides = dict(mock=mock, robot_ip=value('robot_ip'), output_dir=value('output_dir'), collision_enabled=enabled,
                     tcp_xyz_m=[float(x) for x in tcp['xyz_m']], tcp_rpy_rad=[float(x) for x in tcp['rpy_rad']])
    if value('rotation_mapping_gain'):
        overrides['rotation_mapping_gain'] = float(value('rotation_mapping_gain'))
    if enabled:
        overrides.update(robot_description=xacro.process_file(str(robot/'config/fairino3mt_v6_robot.urdf.xacro'), mappings={
            'initial_positions_file': str(robot/'config/initial_positions.yaml'),
            'hardware_plugin': 'mock_components/GenericSystem'}).toxml(),
            robot_description_semantic=(robot/'config/fairino3mt_v6_robot.srdf').read_text())
    if plan['start_camera']:
        camera = yaml.safe_load(Path(value('camera_config')).read_text())
        serial = camera.get('serial_number') or camera['camera']['serial_number']
        subprocess.run(['ros2', 'run', 'fr3c_camera', 'dabai_camera_preflight', '--serial', str(serial)], check=True)
    output = Path(value('output_dir'))
    output.mkdir(parents=True, exist_ok=True)
    (output/'session.json').write_text(json.dumps(dict(
        stage=9, follow_mode='pose', started_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        mock=mock, ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'), motion_starts_automatically=False,
        sources_before_start=graph, owned_sources=plan, driver_params_file=value('driver_params_file'),
        driver_parameters={**params, **{k: v for k, v in overrides.items() if not k.startswith('robot_description')}},
        tag_parameters=yaml.safe_load(Path(value('tag_params_file')).read_text()),
        orientation_parameters=yaml.safe_load(Path(value('orientation_params_file')).read_text()),
        tcp_file=value('tcp_file'), handeye_file=value('handeye_file'),
        collision_enabled=enabled, detector_params_applied=plan['start_detector']), ensure_ascii=False, indent=2)+'\n')
    actions = [RegisterEventHandler(OnProcessExit(on_exit=component_exit)),
        LogInfo(msg='Stage 9 POSE FOLLOW starts IDLE | T teach | S start | X stop. Report: '+str(output)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(follow/'launch/pose_only.launch.py')),
            launch_arguments=dict(start_camera=str(plan['start_camera']).lower(), start_detector=str(plan['start_detector']).lower(),
                                  camera_config=value('camera_config'), params_file=value('tag_params_file')).items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(follow/'launch/preview.launch.py')),
            launch_arguments=dict(start_feedback='false', transform_only='true',
                                  tcp_file=value('tcp_file'), handeye_file=value('handeye_file')).items()),
        Node(package='fr3c_target_follow', executable='orientation_check',
             parameters=[value('orientation_params_file')], output='screen'),
        Node(package='fr3c_follow_driver', executable='pose_follow', name='pose_follow',
             parameters=[value('driver_params_file'), overrides], output='screen')]
    if not plan['start_detector']:
        actions.append(LogInfo(msg='Reusing external detector; its ambiguity threshold is unchanged.'))
    if plan['start_viewer']:
        actions.append(Node(package='fr3c_target_follow', executable='pose_view', name='target_pose_viewer', output='screen'))
    return actions


def generate_launch_description():
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    robot = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    camera = Path(get_package_share_directory('fr3c_camera'))
    driver = Path(get_package_share_directory('fr3c_follow_driver'))
    defaults = dict(mock='false', viewer='true', robot_ip='192.168.58.2', collision_enabled='auto', rotation_mapping_gain='',
        output_dir='/tmp/fr3c_pose_follow_'+str(os.getpid()),
        driver_params_file=str(driver/'config/pose_follow.yaml'),
        orientation_params_file=str(follow/'config/orientation_check_pose_demo.yaml'),
        tag_params_file=str(follow/'config/tag_pose_demo_follow.yaml'),
        camera_config=str(camera/'config/dabai_dcw2_15fps.yaml'),
        tcp_file=str(robot/'config/ydi_j22_tcp.yaml'), handeye_file=str(robot/'config/dabai_dcw2_handeye.yaml'))
    return LaunchDescription([*[DeclareLaunchArgument(key, default_value=default,
        **({'choices': ['true', 'false']} if key in ('mock', 'viewer') else
           {'choices': ['auto', 'true', 'false']} if key == 'collision_enabled' else {})) for key, default in defaults.items()],
        OpaqueFunction(function=start)])
