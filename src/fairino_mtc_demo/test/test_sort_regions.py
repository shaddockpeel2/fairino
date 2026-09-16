"""Keep the launch-time placement mapping aligned with the workcell labels."""

import importlib.util
import json
from pathlib import Path


def test_field_region_colors_are_mapped_to_their_centers(tmp_path):
    launch_path = Path(__file__).parents[1] / 'launch' / 'first_grasp.launch.py'
    spec = importlib.util.spec_from_file_location('first_grasp_launch', launch_path)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)

    centers = {
        'A': [0.1, -0.1, 0.0], 'B': [0.2, -0.1, 0.0],
        'C': [0.2, -0.2, 0.0], 'D': [0.1, -0.2, 0.0],
    }
    document = {
        'schema_version': 1,
        'type': 'a4_tcp_grid',
        'base_frame': 'base_link',
        'grid': {
            'projection': {'mode': 'base_z'},
            'regions': {
                name: {'center_m': center, 'width_m': 0.1, 'height_m': 0.1}
                for name, center in centers.items()
            },
        },
    }
    path = tmp_path / 'a4_grid.json'
    path.write_text(json.dumps(document), encoding='utf-8')

    result = launch_module._load_sort_regions(path)
    assert result['place_blue_center_m'] == centers['A']
    assert result['place_yellow_center_m'] == centers['B']
    assert result['place_red_center_m'] == centers['C']
    assert result['place_green_center_m'] == centers['D']
    for color in ('red', 'yellow', 'blue', 'green'):
        slots = result[f'place_{color}_slots_m']
        assert len(slots) == 12
        assert all(slots[index] == 0.0 for index in range(2, len(slots), 3))


def test_small_sort_region_falls_back_to_one_center_slot(tmp_path):
    launch_path = Path(__file__).parents[1] / 'launch' / 'first_grasp.launch.py'
    spec = importlib.util.spec_from_file_location('first_grasp_launch_small', launch_path)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    center = [0.2, -0.3, 0.0]
    document = {
        'schema_version': 1,
        'type': 'a4_tcp_grid',
        'base_frame': 'base_link',
        'grid': {
            'projection': {'mode': 'base_z'},
            'regions': {
                name: {'center_m': center, 'width_m': 0.05, 'height_m': 0.05}
                for name in ('A', 'B', 'C', 'D')
            },
        },
    }
    path = tmp_path / 'small_grid.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    result = launch_module._load_sort_regions(path)
    assert result['place_red_slots_m'] == center
