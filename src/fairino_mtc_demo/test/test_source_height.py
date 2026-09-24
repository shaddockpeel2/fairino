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


def test_pick_stack_checks_tier_height_and_only_compares_two_tabletop_cubes():
    source = _source()
    validation = source[
        source.index('bool validateStackPair('):
        source.index('bool slotOccupied(')
    ]
    scene = source[
        source.index('bool applyWorldScene('):
        source.index('moveit_msgs::msg::AttachedCollisionObject attachedCube(')
    ]

    assert 'stackCubeTierHeightAcceptable(*source, "source", reason)' in validation
    # The shared-tabletop comparison applies only when both cubes are tier 0.
    assert 'source->stack_tier == 0 && destination.stack_tier == 0 &&' in validation
    assert 'std::abs(tabletop_difference) > stack_height_tolerance_m_' in validation
    assert 'it has to be picked from the tabletop' not in validation
    assert 'stack source bottom must be within' not in scene
    assert 'stackDestinationHeight()' not in source
