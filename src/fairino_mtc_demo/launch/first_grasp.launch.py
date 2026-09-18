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


# Field colour-to-region mapping for the verified four-colour sorting flow.
_COLOR_REGIONS = (('blue', 'A'), ('yellow', 'B'), ('red', 'C'), ('green', 'D'))


def _region_center_and_slots(label, entry, path):
    center = entry['center_m']
    if len(center) != 3 or not all(isinstance(v, (int, float)) and
                                   math.isfinite(v) for v in center):
        raise RuntimeError(f'Invalid {label} center in {path}')
    if not all(math.isfinite(float(entry[k])) and float(entry[k]) > 0.04
               for k in ('width_m', 'height_m')):
        raise RuntimeError(f'{label} is too small for a 30 mm cube: {path}')
    center = [float(v) for v in center]
    width = float(entry['width_m'])
    height = float(entry['height_m'])
    # Keep placement deterministic and simple. A region that can safely hold a
    # 2x2 layout gets four slots; smaller regions retain one center slot.
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
    flat = [coordinate for slot in slots for coordinate in slot]
    return center, flat


def _load_sort_regions(path):
    regions = {}
    document = json.loads(path.read_text(encoding='utf-8'))
    if document.get('schema_version') != 1 or document.get('type') != 'a4_tcp_grid' or \
            document.get('base_frame') != 'base_link' or \
            document.get('grid', {}).get('projection', {}).get('mode') != 'base_z':
        raise RuntimeError(f'Invalid projected A4 grid: {path}')
    for color, region in _COLOR_REGIONS:
        entry = document['grid']['regions'][region]
        center, slots = _region_center_and_slots(region, entry, path)
        regions[f'place_{color}_center_m'] = center
        regions[f'place_{color}_slots_m'] = slots
    # pick_place addresses zones directly, so every calibrated region is exposed
    # under its own name instead of only the four colour-mapped ones. This keeps
    # the placement target independent of the detected colour.
    zone_names = []
    for region, entry in sorted(document['grid']['regions'].items()):
        if not isinstance(region, str) or not region.isalnum():
            raise RuntimeError(f'Invalid region name {region!r} in {path}')
        zone = region.lower()
        if zone in zone_names:
            raise RuntimeError(f'Duplicate region name {region!r} in {path}')
        center, slots = _region_center_and_slots(region, entry, path)
        zone_names.append(zone)
        regions[f'place_zone_{zone}_center_m'] = center
        regions[f'place_zone_{zone}_slots_m'] = slots
    regions['place_zone_names'] = zone_names
    regions['sort_grid_file'] = str(path.resolve())
    return regions


def _task_node(context, moveit_config, package):
    sort_enabled = LaunchConfiguration('sort_enabled').perform(context).lower() in (
        'true', '1', 'yes')
    stack_enabled = LaunchConfiguration('stack_enabled').perform(context).lower() in (
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
            'sort_enabled': sort_enabled,
            'stack_enabled': stack_enabled,
            **regions,
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
        DeclareLaunchArgument(
            'stack_enabled',
            default_value='false',
            description='Enable the pick_stack action. Requires sort_enabled.',
        ),
        DeclareLaunchArgument('grid_file', default_value=''),
        OpaqueFunction(function=_task_node, args=[moveit_config, package]),
    ])
