"""Stage 5: one owned launch, reuse existing vision, start the driver idle."""
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
from fr3c_target_follow.demo_graph import discover, source_plan


def component_exit(event, context):
    # Includes only processes owned by this launch. Reused sources are left running.
    if context.is_shutdown:
        return []
    return [LogInfo(msg='Demo component exited; stopping the Demo and its driver.'),
            EmitEvent(event=Shutdown(reason='Demo component exited: ' + event.action.process_details['name']))]


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mock = value('mock').lower() == 'true'
    viewer = value('viewer').lower() == 'true'
    graph = discover()
    plan = source_plan(graph, mock=mock, viewer=viewer)
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    if plan['start_camera']:
        camera = yaml.safe_load(Path(value('camera_config')).read_text())
        serial = camera.get('serial_number') or camera['camera']['serial_number']
        subprocess.run(['ros2', 'run', 'fr3c_camera', 'dabai_camera_preflight', '--serial', str(serial)],
                       check=True)
    output = Path(value('output_dir'))
    output.mkdir(parents=True, exist_ok=True)
    session = {
        'stage': 5, 'started_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        'mock': mock, 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'motion_starts_automatically': False, 'sources_before_start': graph, 'owned_sources': plan,
        'driver_params_file': value('driver_params_file'), 'output_dir': str(output),
        'collision_enabled_selection': value('collision_enabled'),
    }
    (output / 'session.json').write_text(json.dumps(session, ensure_ascii=False, indent=2) + '\n')
    return [
        RegisterEventHandler(OnProcessExit(on_exit=component_exit)),
        LogInfo(msg='Demo report directory: ' + str(output)),
        LogInfo(msg='Camera: ' + ('start' if plan['start_camera'] else 'reuse' if graph['publishers']['camera'] else 'mock, no device')),
        LogInfo(msg='Tag detector: ' + ('start' if plan['start_detector'] else 'reuse')),
        LogInfo(msg='Driver starts IDLE. T teach | S start (records offset again) | X stop. '
                    'Ctrl-C or closing the Demo-owned viewer ends the Demo.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(follow / 'launch/pose_only.launch.py')),
            launch_arguments={
                'start_camera': str(plan['start_camera']).lower(),
                'start_detector': str(plan['start_detector']).lower(),
                'camera_config': value('camera_config'), 'params_file': value('tag_params_file'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(follow / 'launch/position_follow.launch.py')),
            launch_arguments={name: value(name) for name in (
                'mock', 'robot_ip', 'output_dir', 'tcp_file', 'handeye_file', 'driver_params_file',
                'collision_enabled')}.items()),
        *([Node(package='fr3c_target_follow', executable='pose_view', name='target_pose_viewer',
                output='screen')] if plan['start_viewer'] else []),
    ]


def generate_launch_description():
    follow = Path(get_package_share_directory('fr3c_target_follow'))
    camera = Path(get_package_share_directory('fr3c_camera'))
    robot = Path(get_package_share_directory('fairino3mt_v6_moveit2_config'))
    driver = Path(get_package_share_directory('fr3c_follow_driver'))
    default_output = '/tmp/fr3c_follow_demo_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid())
    defaults = {
        'mock': 'false', 'viewer': 'true', 'robot_ip': '192.168.58.2', 'output_dir': default_output,
        'collision_enabled': 'auto',
        'camera_config': str(camera / 'config/dabai_dcw2_15fps.yaml'),
        'tag_params_file': str(follow / 'config/tag_pose.yaml'),
        'driver_params_file': str(driver / 'config/position_follow.yaml'),
        'tcp_file': str(robot / 'config/ydi_j22_tcp.yaml'),
        'handeye_file': str(robot / 'config/dabai_dcw2_handeye.yaml'),
    }
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value)
                                for name, value in defaults.items()], OpaqueFunction(function=start)])
