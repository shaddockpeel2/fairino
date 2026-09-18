from pathlib import Path


def _source():
    return (Path(__file__).parents[1] / 'src' / 'first_grasp_node.cpp').read_text()


def test_ordinary_actions_do_not_require_the_source_bottom_at_z_zero():
    source = _source()
    snapshot = source[
        source.index('bool targetSnapshot('):
        source.index('std::string calibratedZoneList()')
    ]

    assert 'current target is not on the configured table plane' not in snapshot
    assert 'target_z_bounds_' in snapshot


def test_pick_stack_checks_same_tabletop_before_building_the_scene():
    source = _source()
    validation = source[
        source.index('bool validateStackPair('):
        source.index('bool slotOccupied(')
    ]
    scene = source[
        source.index('bool applyWorldScene('):
        source.index('moveit_msgs::msg::AttachedCollisionObject attachedCube(')
    ]

    assert 'std::abs(source_plane_gap) > maximum_cube_plane_gap_m_' in validation
    assert 'std::abs(tabletop_difference) > stack_height_tolerance_m_' in validation
    assert 'both cubes must start on the tabletop' in validation
    assert 'stack source bottom must be within' not in scene
    assert 'stackDestinationHeight()' not in source
