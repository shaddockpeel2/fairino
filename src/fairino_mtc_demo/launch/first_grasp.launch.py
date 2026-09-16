"""Launch the one-shot P5 task against an already running MoveIt/perception stack."""

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _load_sort_regions(path):
    regions = {}
    document = json.loads(path.read_text(encoding='utf-8'))
    if document.get('schema_version') != 1 or document.get('type') != 'a4_tcp_grid' or \
            document.get('base_frame') != 'base_link' or \
            document.get('grid', {}).get('projection', {}).get('mode') != 'base_z':
        raise RuntimeError(f'Invalid projected A4 grid: {path}')
    for color, region in (('blue', 'A'), ('yellow', 'B'),
                          ('red', 'C'), ('green', 'D')):
        entry = document['grid']['regions'][region]
        center = entry['center_m']
        if len(center) != 3 or not all(isinstance(v, (int, float)) and
                                       math.isfinite(v) for v in center):
            raise RuntimeError(f'Invalid {region} center in {path}')
        if not all(math.isfinite(float(entry[k])) and float(entry[k]) > 0.04
                   for k in ('width_m', 'height_m')):
            raise RuntimeError(f'{region} is too small for a 30 mm cube: {path}')
        center = [float(v) for v in center]
        width = float(entry['width_m'])
        height = float(entry['height_m'])
        regions[f'place_{color}_center_m'] = center
        # Keep placement deterministic and simple. A region that can safely
        # hold a 2x2 layout gets four slots; smaller regions retain one center.
        cube_side = 0.030
        edge_margin = 0.005
        if width >= 2.0 * (cube_side + edge_margin) and \
                height >= 2.0 * (cube_side + edge_margin):
            offset_x = (width - cube_side - 2.0 * edge_margin) * 0.5
            offset_y = (height - cube_side - 2.0 * edge_margin) * 0.5
            slots = [
                [center[0] + dx, center[1] + dy, center[2]]
                for dy in (-offset_y, offset_y)
                for dx in (-offset_x, offset_x)
            ]
        else:
            slots = [center]
        regions[f'place_{color}_slots_m'] = [
            coordinate for slot in slots for coordinate in slot]
    regions['sort_grid_file'] = str(path.resolve())
    return regions


def _task_node(context, moveit_config, package):
    sort_enabled = LaunchConfiguration('sort_enabled').perform(context).lower() in (
        'true', '1', 'yes')
    regions = {}
    if sort_enabled:
        path = Path(LaunchConfiguration('grid_file').perform(context))
        regions = _load_sort_regions(path)
    return [Node(
        package=package,
        executable='first_grasp_node',
        name='fr3c_first_grasp',
        output='screen',
        parameters=[moveit_config, LaunchConfiguration('task_config'), {
            'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
            'sort_enabled': sort_enabled, **regions,
        }],
    )]


def generate_launch_description():
    package = 'fairino_mtc_demo'
    moveit_package = 'fairino3mt_v6_moveit2_config'
    config = Path(get_package_share_directory(package)) / 'config/first_grasp.yaml'
    moveit_config = MoveItConfigsBuilder(
        'fairino3mt_v6_robot', package_name=moveit_package
    ).to_dict()

    return LaunchDescription([
        DeclareLaunchArgument(
            'task_config',
            default_value=str(config),
            description='P5 geometry, ground-plane, grasp and execution-gate parameters.',
        ),
        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='Arm real motion/gripper steps. Default only plans and publishes preview.',
        ),
        DeclareLaunchArgument('sort_enabled', default_value='false'),
        DeclareLaunchArgument('grid_file', default_value=''),
        OpaqueFunction(function=_task_node, args=[moveit_config, package]),
    ])
