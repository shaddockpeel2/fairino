import numpy as np
import pytest
from pathlib import Path
import yaml

from fr3c_target_follow.follow_geometry import transform_rpy, transform, quaternion_from_rotation
from fr3c_target_follow.marker_pose import rotation_distance_deg
from fr3c_target_follow.pose_preview import PosePreview


def target(degrees=0., position=(.1, -.3, .02), axis=2):
    angles = np.zeros(3)
    angles[axis] = np.radians(degrees)
    return transform_rpy(position, angles)


def exact(**kwargs):
    return PosePreview(position_tau_s=0., rotation_tau_s=0., deadband_enter_deg=0.,
                       deadband_exit_deg=0., **kwargs)


def test_record_no_jump_and_full_relative_transform_in_combined_motion():
    start = transform_rpy([.15, -.4, .03], [.2, -.1, .8])
    tcp = transform_rpy([.2, -.3, .2], [2.9, .1, -.5])
    p = exact()
    p.teach(1_000_000_000, start, tcp)
    assert p.goal() == pytest.approx(tcp)
    assert p.step(tcp, .02)['reference'] == pytest.approx(tcp)
    moved = transform_rpy([.17, -.42, .04], [.3, -.2, 1.1])
    p.observe(1_100_000_000, moved)
    assert p.goal() == pytest.approx(moved @ np.linalg.inv(start) @ tcp)


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_each_rotation_axis_rotates_position_offset_and_tcp_orientation(axis):
    start = target()
    tcp = target(position=(.3, -.2, .22))
    p = exact()
    p.teach(1_000_000_000, start, tcp)
    moved = target(10., axis=axis)
    p.observe(1_100_000_000, moved)
    expected = moved @ np.linalg.inv(start) @ tcp
    assert p.goal() == pytest.approx(expected)
    assert np.linalg.norm(p.goal()[:3, 3]-tcp[:3, 3]) > .01


def test_half_rotation_mapping_reduces_both_orientation_and_orbit():
    start = target()
    tcp = target(position=(.3, -.3, .02))
    p = exact(rotation_mapping_gain=.5)
    p.teach(1_000_000_000, start, tcp)
    p.observe(1_100_000_000, target(20.))
    assert p.goal() == pytest.approx(target(10.) @ np.linalg.inv(start) @ tcp)
    assert rotation_distance_deg(tcp[:3, :3], p.goal()[:3, :3]) == pytest.approx(10.)


def test_deadband_ignores_small_jitter_but_accumulates_slow_motion():
    p = PosePreview(position_tau_s=0., rotation_tau_s=0.)
    p.teach(1_000_000_000, target(), target(position=(.3, -.3, .02)))
    for i, angle in enumerate([1., -1., 2., -2., 1.], 1):
        p.observe(1_000_000_000+i*50_000_000, target(angle))
        assert rotation_distance_deg(p.goal()[:3, :3], np.eye(3)) < 1e-5
    for i, angle in enumerate(np.arange(1., 11.), 6):
        p.observe(1_000_000_000+i*50_000_000, target(angle))
    assert rotation_distance_deg(p.goal()[:3, :3], np.eye(3)) == pytest.approx(8.)


def test_filter_uses_real_observation_time_and_wraps_shortest_rotation():
    p = PosePreview(position_tau_s=.2, rotation_tau_s=.4, deadband_enter_deg=0., deadband_exit_deg=0.)
    start = target(179.)
    p.teach(1_000_000_000, start, start)
    end = target(-179., position=(.2, -.3, .02))
    p.observe(1_100_000_000, end)
    assert p.filtered[0, 3] == pytest.approx(.1+.1*(1-np.exp(-.1/.2)))
    assert rotation_distance_deg(start[:3, :3], p.filtered[:3, :3]) == pytest.approx(2*(1-np.exp(-.1/.4)))
    q = quaternion_from_rotation(end[:3, :3])
    p.observe(1_200_000_000, transform(end[:3, 3], -q))
    assert rotation_distance_deg(start[:3, :3], p.filtered[:3, :3]) == pytest.approx(2*(1-np.exp(-.2/.4)))
    with pytest.raises(ValueError, match='increasing'):
        p.observe(1_200_000_000, end)


def test_reference_is_bounded_from_actual_feedback_never_simulated_progress():
    tcp = target(position=(.3, -.3, .02))
    p = exact()
    p.teach(1_000_000_000, target(), tcp)
    p.observe(1_100_000_000, target(40., position=(.4, -.3, .02)))
    last_v, last_w = np.zeros(3), np.zeros(3)
    for _ in range(100):
        output = p.step(tcp, .02)
        assert np.linalg.norm(output['velocity']) <= .02+1e-12
        assert np.linalg.norm(output['omega']) <= np.radians(3.)+1e-12
        assert np.linalg.norm(output['velocity']-last_v) <= .06*.02+1e-12
        assert np.linalg.norm(output['omega']-last_w) <= np.radians(10.)*.02+1e-12
        assert np.linalg.norm(output['reference'][:3, 3]-tcp[:3, 3]) <= .02*.02+1e-12
        assert rotation_distance_deg(output['reference'][:3, :3], tcp[:3, :3]) <= 3.*.02+1e-6
        last_v, last_w = output['velocity'], output['omega']
    with pytest.raises(ValueError, match='period_gap'):
        p.step(tcp, .2)
    assert p.velocity == pytest.approx([0, 0, 0])
    assert p.omega == pytest.approx([0, 0, 0])


def test_nonzero_rotated_tcp_is_compensated_in_flange_reference():
    flange_tcp = transform_rpy([.002, -.003, .185], [.1, -.2, .3])
    flange = transform_rpy([.2, -.3, .4], [np.pi, .2, .4])
    actual_tcp = flange @ flange_tcp
    p = exact()
    p.teach(1_000_000_000, target(), actual_tcp)
    p.observe(1_100_000_000, target(12., axis=0))
    reference = p.step(actual_tcp, .02)['reference']
    flange_reference = reference @ np.linalg.inv(flange_tcp)
    assert flange_reference @ flange_tcp == pytest.approx(reference)
    assert np.linalg.norm(flange_reference[:3, 3]-flange[:3, 3]) > 1e-6


def test_quality_gap_cannot_reseed_rotation_and_bypass_deadband():
    p = PosePreview()
    tcp = target(position=(.3, -.3, .02))
    p.teach(1_000_000_000, target(), tcp)
    p.observe(1_100_000_000, target(1.))
    p.stop()
    p.observe(2_000_000_000, target(2.))
    assert p.goal() == pytest.approx(tcp)
    p.observe(3_000_000_000, target(20.))
    assert rotation_distance_deg(p.filtered[:3, :3], target(20.)[:3, :3]) > 10.


def test_stable_demo_profile_absorbs_several_degree_jitter_then_responds_and_reanchors():
    config = yaml.safe_load((Path(__file__).parents[1]/'config/pose_preview_demo_stable.yaml').read_text())['pose_preview']['ros__parameters']
    preview = PosePreview(**{k: v for k, v in config.items()
                             if k not in ('target_id', 'auto_teach', 'feedback_timeout_s', 'publish_hz')})
    tcp = transform_rpy([.3, -.2, .18], [2.9, -.2, .7])
    preview.teach(1_000_000_000, target(), tcp)
    for i in range(1, 61):
        preview.observe(1_000_000_000+i*100_000_000, target([3., -3., 4.5, -4.5][i % 4]))
        assert preview.goal() == pytest.approx(tcp, abs=1e-10)
    for i in range(61, 101):
        preview.observe(1_000_000_000+i*100_000_000, target(20.))
    angle = rotation_distance_deg(preview.goal()[:3, :3], tcp[:3, :3])
    assert 6. < angle < 10.
    assert np.linalg.norm(preview.goal()[:3, 3]-tcp[:3, 3]) > .01
    moved_tcp = transform_rpy([.2, -.35, .25], [3., .3, -.5])
    preview.teach(12_000_000_000, target(-8., position=(.15, -.25, .03)), moved_tcp)
    assert preview.goal() == pytest.approx(moved_tcp, abs=1e-10)
