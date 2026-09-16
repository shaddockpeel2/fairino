from collections import deque

import numpy as np

from fr3c_camera.blue_cube_localizer import (
    _periodic_angle_mean_std,
    _update_sample_window,
)


def update(samples, stamp, xyz):
    return _update_sample_window(
        samples=samples,
        stamp=stamp,
        center=np.asarray(xyz, dtype=np.float64),
        payload=f'observation-{stamp}',
        maximum_gap=0.75,
        maximum_jump=0.020,
        required_samples=4,
        xy_std_limit=0.003,
        z_std_limit=0.003,
    )


def test_short_rejection_does_not_erase_accepted_samples():
    samples = deque(maxlen=4)
    for stamp in (0.0, 0.2, 0.4):
        stable, _ = update(samples, stamp, [0.20, -0.40, 0.035])
        assert not stable

    # A rejected frame is intentionally not added. The next good frame can
    # complete the rolling window instead of restarting from zero.
    stable, deviation = update(samples, 0.7, [0.201, -0.400, 0.035])

    assert stable
    assert len(samples) == 4
    assert float(np.max(deviation)) < 0.003


def test_long_gap_restarts_the_window():
    samples = deque(maxlen=4)
    for stamp in (0.0, 0.2, 0.4):
        update(samples, stamp, [0.20, -0.40, 0.035])

    stable, _ = update(samples, 1.3, [0.20, -0.40, 0.035])

    assert not stable
    assert len(samples) == 1


def test_target_jump_restarts_the_window():
    samples = deque(maxlen=4)
    for stamp in (0.0, 0.2, 0.4):
        update(samples, stamp, [0.20, -0.40, 0.035])

    stable, _ = update(samples, 0.6, [0.24, -0.40, 0.035])

    assert not stable
    assert len(samples) == 1


def test_rectangle_axis_mean_is_continuous_across_pi_equivalence():
    angles = np.deg2rad([89.0, -89.0, 88.0, -88.0])

    mean, deviation = _periodic_angle_mean_std(angles)

    assert abs(abs(np.rad2deg(mean)) - 90.0) < 0.1
    assert np.rad2deg(deviation) < 2.0


def test_cube_edge_mean_is_continuous_across_quarter_turn_equivalence():
    angles = np.deg2rad([44.0, -44.0, 43.0, -43.0])

    mean, deviation = _periodic_angle_mean_std(angles, period=np.pi / 2)

    assert abs(abs(np.rad2deg(mean)) - 45.0) < 0.1
    assert np.rad2deg(deviation) < 2.0


def test_large_rectangle_yaw_jump_restarts_the_window():
    samples = deque(maxlen=4)
    common = dict(
        maximum_gap=0.75,
        maximum_jump=0.020,
        required_samples=4,
        xy_std_limit=0.003,
        z_std_limit=0.003,
        maximum_yaw_jump=np.deg2rad(20.0),
    )
    center = np.array([0.20, -0.40, 0.035])
    for stamp in (0.0, 0.2, 0.4):
        _update_sample_window(
            samples, stamp, center, 'observation', yaw=np.deg2rad(10.0), **common)

    stable, _ = _update_sample_window(
        samples, 0.6, center, 'observation', yaw=np.deg2rad(45.0), **common)

    assert not stable
    assert len(samples) == 1
