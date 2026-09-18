"""Keep the launch-time placement mapping aligned with the workcell labels."""

import importlib.util
import json
from pathlib import Path

import pytest


def _launch_module(name):
    launch_path = Path(__file__).parents[1] / 'launch' / 'first_grasp.launch.py'
    spec = importlib.util.spec_from_file_location(name, launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _grid_document(regions):
    return {
        'schema_version': 1,
        'type': 'a4_tcp_grid',
        'base_frame': 'base_link',
        'grid': {
            'projection': {'mode': 'base_z'},
            'regions': regions,
        },
    }


def test_every_calibrated_region_is_addressable_as_a_zone(tmp_path):
    """pick_place zones must cover the grid, not just the four colour regions."""
    launch_module = _launch_module('first_grasp_launch_zones')
    wide = {'center_m': [0.15, -0.15, 0.0], 'width_m': 0.12, 'height_m': 0.12}
    document = _grid_document({
        'A': wide, 'B': wide, 'C': wide, 'D': wide,
        'E': {'center_m': [0.3, -0.3, 0.0], 'width_m': 0.05, 'height_m': 0.05},
    })
    path = tmp_path / 'a4_grid.json'
    path.write_text(json.dumps(document), encoding='utf-8')

    result = launch_module._load_sort_regions(path)

    assert sorted(result['place_zone_names']) == ['a', 'b', 'c', 'd', 'e']
    for zone, label in (('a', 'A'), ('b', 'B'), ('c', 'C'), ('d', 'D'), ('e', 'E')):
        assert result[f'place_zone_{zone}_center_m'] == document['grid']['regions'][label][
            'center_m']
        assert len(result[f'place_zone_{zone}_slots_m']) % 3 == 0
    # A zone is an independent handle: its slots are its own, not a colour alias.
    assert result['place_zone_a_slots_m'] == result['place_blue_slots_m']
    assert result['place_zone_e_slots_m'] == document['grid']['regions']['E']['center_m']
    # The verified colour mapping is untouched by the zone parameters.
    assert result['place_blue_center_m'] == document['grid']['regions']['A']['center_m']
    assert result['place_red_center_m'] == document['grid']['regions']['C']['center_m']


def test_zone_names_reject_non_identifier_regions(tmp_path):
    """A region name that cannot become a ROS parameter must fail loudly."""
    launch_module = _launch_module('first_grasp_launch_bad_zone')
    document = _grid_document({
        'A': {'center_m': [0.1, -0.1, 0.0], 'width_m': 0.12, 'height_m': 0.12},
        'B': {'center_m': [0.2, -0.1, 0.0], 'width_m': 0.12, 'height_m': 0.12},
        'C': {'center_m': [0.2, -0.2, 0.0], 'width_m': 0.12, 'height_m': 0.12},
        'D': {'center_m': [0.1, -0.2, 0.0], 'width_m': 0.12, 'height_m': 0.12},
        'A-1': {'center_m': [0.3, -0.3, 0.0], 'width_m': 0.12, 'height_m': 0.12},
    })
    path = tmp_path / 'bad_grid.json'
    path.write_text(json.dumps(document), encoding='utf-8')

    with pytest.raises(RuntimeError, match='Invalid region name'):
        launch_module._load_sort_regions(path)


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
