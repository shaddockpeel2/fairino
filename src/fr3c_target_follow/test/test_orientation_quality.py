import numpy as np
import pytest

from fr3c_target_follow.follow_geometry import transform_rpy
from fr3c_target_follow.orientation_quality import (
    ExactPosePairer, OrientationGate, measurement_problem, stamp_ns,
)
from fr3c_target_follow.orientation_audit import orientation_report


def rotation(degrees):
    return transform_rpy([0, 0, 0], [0, 0, np.radians(degrees)])[:3, :3]


def quality():
    return dict(status='valid', valid_position=True, orientation_ambiguous=False,
                orientation_measurement_valid=True, solver='IPPE_SQUARE', candidate_count=2,
                candidate_errors_px=[.01, .6], candidate_rotation_distance_deg=30.,
                selected_candidate_index=0)


def test_exact_pairing_distinguishes_nanoseconds_lost_by_epoch_float_and_reversed_arrival():
    pairer = ExactPosePairer()
    stamp = dict(sec=1791500000, nanosec=123456001)
    other = dict(sec=1791500000, nanosec=123456002)
    assert stamp_ns(stamp)/1e9 == stamp_ns(other)/1e9
    assert pairer.add('pose', stamp, 'base_pose') is None
    assert pairer.add('quality', other, 'wrong_quality') is None
    assert pairer.add('quality', stamp, 'correct_quality') == (stamp_ns(stamp), 'base_pose', 'correct_quality')
    assert pairer.add('pose', other, 'second_pose') == (stamp_ns(other), 'second_pose', 'wrong_quality')
    assert pairer.add('pose', stamp, 'old_pose') is None
    assert pairer.paired == 2 and pairer.late == 1


@pytest.mark.parametrize('stamp', [None, {'stamp_s': 100.1}, {'sec': 100, 'nanosec': 1.},
                                    {'sec': 0, 'nanosec': 0}, {'sec': 1, 'nanosec': 10**9},
                                    {'sec': True, 'nanosec': 1}])
def test_float_or_invalid_stamps_are_never_guessed(stamp):
    with pytest.raises(ValueError):
        stamp_ns(stamp)


def test_pair_caches_are_bounded_and_old_missing_partners_expire():
    pairer = ExactPosePairer(capacity=2)
    for ns in (1, 2, 3):
        pairer.add('pose', dict(sec=100, nanosec=ns), str(ns))
    assert len(pairer.poses) == 2 and pairer.evicted == 1
    pairer.expire(101*10**9, .35)
    assert not pairer.poses and pairer.expired == 2


def test_quality_cannot_promote_ambiguity_fallback_or_wrong_continuity_branch():
    row = quality()
    assert measurement_problem(row) is None
    for field, value in [('orientation_ambiguous', True), ('orientation_measurement_valid', False),
                         ('solver', 'ITERATIVE_FALLBACK'), ('selected_candidate_index', 1),
                         ('candidate_errors_px', [float('nan'), .3]), ('candidate_count', 3)]:
        assert measurement_problem(dict(row, **{field: value})) is not None
    del row['orientation_ambiguous']
    assert measurement_problem(row) == 'missing_orientation_quality'


def test_approximate_alternative_requires_explicit_bounded_policy_and_valid_measurement():
    row = dict(quality(), selected_candidate_index=1)
    assert measurement_problem(row) == 'continuity_selected_alternative'
    row.update(orientation_candidate_tolerance_deg=35., orientation_approximate=True)
    assert measurement_problem(row) is None
    for field, value in [('orientation_candidate_tolerance_deg', float('nan')),
                         ('orientation_candidate_tolerance_deg', 46.),
                         ('orientation_candidate_tolerance_deg', True),
                         ('candidate_rotation_distance_deg', 36.), ('orientation_approximate', False),
                         ('orientation_ambiguous', True), ('orientation_measurement_valid', False),
                         ('solver', 'ITERATIVE_FALLBACK')]:
        assert measurement_problem(dict(row, **{field: value})) is not None


def test_five_distinct_good_frames_and_reconfirmation_after_rejection():
    gate = OrientationGate()
    for i in range(1, 6):
        output = gate.observe(100*10**9+i*60_000_000, rotation(0))
        assert output['valid_orientation'] is (i == 5)
    output = gate.observe(100*10**9+6*60_000_000, rotation(0), 'orientation_ambiguous')
    assert not output['valid_orientation'] and output['confirmation_count'] == 0
    for i in range(7, 12):
        output = gate.observe(100*10**9+i*60_000_000, rotation(0))
        assert output['valid_orientation'] is (i == 11)


def test_rate_bound_uses_real_image_interval_and_jump_does_not_replace_anchor():
    slow = OrientationGate(confirmation_frames=1)
    slow.observe(100*10**9, rotation(0))
    assert slow.observe(100*10**9+100_000_000, rotation(8))['valid_orientation']
    fast = OrientationGate(confirmation_frames=1)
    fast.observe(100*10**9, rotation(0))
    assert fast.observe(100*10**9+20_000_000, rotation(8))['status'] == 'orientation_jump'
    assert fast.observe(100*10**9+40_000_000, rotation(0))['valid_orientation']


def test_pose_demo_tolerates_tilt_noise_but_rejects_large_isolated_flip():
    gate = OrientationGate(confirmation_frames=2, jitter_allowance_deg=25.)
    gate.observe(100*10**9, rotation(0))
    assert gate.observe(100*10**9+60_000_000, rotation(0))['valid_orientation']
    assert gate.observe(100*10**9+120_000_000, rotation(22))['valid_orientation']
    assert gate.observe(100*10**9+180_000_000, rotation(-2))['valid_orientation']
    rejected = gate.observe(100*10**9+240_000_000, rotation(120))
    assert rejected['status'] == 'orientation_jump' and not rejected['valid_orientation']
    assert gate.observe(100*10**9+300_000_000, rotation(-1))['status'] == 'acquiring_orientation'
    assert gate.observe(100*10**9+360_000_000, rotation(0))['valid_orientation']
    original = OrientationGate(confirmation_frames=2, jitter_allowance_deg=6.)
    original.observe(100*10**9, rotation(0))
    assert original.observe(100*10**9+60_000_000, rotation(22))['status'] == 'orientation_jump'


def test_cross_half_turn_is_short_rotation_and_timeout_requires_new_confirmation():
    gate = OrientationGate(confirmation_frames=2)
    gate.observe(100*10**9, rotation(179))
    out = gate.observe(100*10**9+100_000_000, rotation(-179))
    assert out['valid_orientation'] and out['orientation_delta_deg'] == pytest.approx(2)
    out = gate.observe(101*10**9, rotation(-179))
    assert not out['valid_orientation'] and out['confirmation_count'] == 1
    with pytest.raises(ValueError, match='increasing'):
        gate.observe(101*10**9, rotation(-179))


def test_report_separates_measured_from_trusted_orientation_and_signed_base_axes():
    q = lambda a: [0., 0., float(np.sin(np.radians(a)/2)), float(np.cos(np.radians(a)/2))]
    rows = [dict(kind='observation', status=status, valid_orientation=valid, position_m=[0., 0., 0.],
                 quaternion_xyzw=q(angle), quality={}) for status, valid, angle in
            [('orientation_ambiguous', False, 0), ('valid', True, 10), ('valid', True, 20)]]
    report = orientation_report(rows, 'target_rotation')
    assert report['paired_observations'] == 3 and report['trusted_orientation_observations'] == 2
    assert report['all_base']['start_end_rotation_vector_deg'] == pytest.approx([0, 0, 20])
    assert report['trusted_base']['start_end_rotation_vector_deg'] == pytest.approx([0, 0, 10])
    assert report['motion_commands_enabled'] is False
