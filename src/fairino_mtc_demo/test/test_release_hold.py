"""Regression checks for the redundant fixed-duration placement release sequence."""
from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def _source():
    return (PACKAGE / 'src' / 'first_grasp_node.cpp').read_text()


def test_release_finishes_before_scene_detach_and_retreat():
    source = _source()
    retreat_start = source.index('        auto retreat = preview[kPlaceRetreatIndex]')
    cycle = source[
        retreat_start:
        source.index('        auto home = planHome(', retreat_start)
    ]

    ordered_steps = [
        'for (int repetition = 1; repetition <= release_command_repetitions_; ++repetition)',
        'commandGripper(pre_open_position_m_, name)',
        'waitForGripperState(pre_open_position_m_, name)',
        'releaseCubeScene(reason)',
        'verifyValidatedSegmentStart(',
        'executeMotion(retreat, activePlaceApproachPose())',
    ]
    offsets = [cycle.index(step) for step in ordered_steps]
    assert offsets == sorted(offsets)
    assert 'beginGripperCommand(pre_open_position_m_, "release")' not in cycle
    assert '"release_confirm_" + std::to_string(repetition)' in cycle


def test_release_redundancy_defaults_to_two_bounded_open_commands():
    source = _source()
    config = yaml.safe_load((PACKAGE / 'config' / 'first_grasp.yaml').read_text())

    assert config['fr3c_first_grasp']['ros__parameters']['release_command_repetitions'] == 2
    assert 'release_command_repetitions_{2}' in source
    assert 'release_command_repetitions_ < 1' in source
    assert 'release_command_repetitions_ > 3' in source


def test_p6_gate_can_be_first_accepted_for_100_hours():
    config = yaml.safe_load((PACKAGE / 'config' / 'first_grasp.yaml').read_text())
    parameters = config['fr3c_first_grasp']['ros__parameters']

    assert parameters['p6_gate_max_age_s'] == 360000.0
    assert 'p6_gate_max_age_s_{360000.0}' in _source()


def test_release_overlap_timing_contract_is_removed():
    source = _source()
    assert 'waitForReleaseLead' not in source
    assert 'release_motion_lead_s_' not in source
    assert 'release_pwm_hold_s_' not in source
    assert 'release_hold_margin_s_' not in source


def test_pre_open_is_bounded_before_any_arm_motion_for_open_loop_hardware():
    source = _source()
    cycle_start = source.index('bool runCycleBody()')
    cycle = source[cycle_start:source.index('void fail(', cycle_start)]
    loop = cycle.index('pre_open_command_repetitions_')
    planning = cycle.index('preview = previewAll()')
    assert loop < planning
    assert '"pre_open_confirm_" + std::to_string(repetition)' in cycle
    assert 'pre_open_command_repetitions_{1}' in source
