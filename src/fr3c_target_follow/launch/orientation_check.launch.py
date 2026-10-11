"""Stage 6: missing read-only sources, monitor, audit and optional read-only view."""
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from fr3c_target_follow.demo_graph import discover, orientation_source_plan


def component_exit(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason='Orientation check component exited; closing owned read-only sources.'))]


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    graph = discover()
    plan = orientation_source_plan(graph, value('start_sources') == 'true')
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    demo_quality = value('quality_profile') == 'demo'
    params_file = value('params_file') or str(follow/'config'/(
        'orientation_check_demo.yaml' if demo_quality else 'orientation_check.yaml'))
    tag_params_file = value('tag_params_file') or str(follow/'config'/(
        'tag_pose_demo.yaml' if demo_quality else 'tag_pose.yaml'))
    if plan['start_camera']:
        camera = yaml.safe_load(Path(value('camera_config')).read_text())
        serial = camera.get('serial_number') or camera['camera']['serial_number']
        subprocess.run(['ros2', 'run', 'fr3c_camera', 'dabai_camera_preflight', '--serial', str(serial)], check=True)
    output = Path(value('output_dir'))
    output.mkdir(parents=True, exist_ok=True)
    (output/'session.json').write_text(json.dumps(dict(
        stage=6, started_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        sources_before_start=graph, owned_sources=plan, motion_commands_enabled=False,
        ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'), scenario=value('scenario'),
        quality_profile=value('quality_profile'), orientation_params_file=params_file,
        tag_params_file=tag_params_file, detector_params_applied=plan['start_detector']), indent=2)+'\n')
    actions = [
        RegisterEventHandler(OnProcessExit(on_exit=component_exit)),
        LogInfo(msg='Stage 6 READ ONLY. Report: '+str(output)),
        LogInfo(msg='Orientation quality profile: '+value('quality_profile')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(follow/'launch/pose_only.launch.py')),
            launch_arguments=dict(start_camera=str(plan['start_camera']).lower(),
                                  start_detector=str(plan['start_detector']).lower(),
                                  camera_config=value('camera_config'), params_file=tag_params_file).items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(follow/'launch/preview.launch.py')),
            launch_arguments=dict(start_feedback=str(plan['start_feedback']).lower(),
                                  start_preview=str(plan['start_base']).lower(), transform_only='true',
                                  tcp_file=value('tcp_file'), handeye_file=value('handeye_file')).items()),
        Node(package='fr3c_target_follow', executable='orientation_check',
             parameters=[params_file], output='screen'),
        Node(package='fr3c_target_follow', executable='orientation_audit', output='screen',
             arguments=['--output', str(output), '--duration-s', value('duration_s'), '--scenario', value('scenario')]),
    ]
    if demo_quality and not plan['start_detector']:
        actions.append(LogInfo(msg='Reusing external detector: its ambiguity threshold is unchanged; '
                               'Demo confirmation applies to this monitor. To apply Demo detector quality, '
                               'stop the external detector and restart this check.'))
    if value('viewer') == 'true':
        actions.append(Node(package='fr3c_target_follow', executable='pose_view', output='screen', arguments=['--read-only']))
    return actions


def generate_launch_description():
    camera = Path(get_package_share_directory('fr3c_camera'))
    robot = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    defaults = dict(start_sources='true', viewer='true', duration_s='30', scenario='unspecified',
                    quality_profile='conservative',
                    output_dir='/tmp/fr3c_orientation_'+str(os.getpid()),
                    params_file='', tag_params_file='',
                    camera_config=str(camera/'config/dabai_dcw2_15fps.yaml'),
                    tcp_file=str(robot/'config/ydi_j22_tcp.yaml'),
                    handeye_file=str(robot/'config/dabai_dcw2_handeye.yaml'))
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v,
             **({'choices': ['true', 'false']} if k in ('viewer', 'start_sources') else
                {'choices': ['conservative', 'demo']} if k == 'quality_profile' else
                {'choices': ['unspecified', 'static', 'target_rotation', 'robot_rotation']} if k == 'scenario' else {}))
          for k, v in defaults.items()], OpaqueFunction(function=start)])
