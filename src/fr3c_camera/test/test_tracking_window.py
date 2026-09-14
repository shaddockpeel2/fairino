from collections import deque

import numpy as np

from fr3c_camera.blue_cube_localizer import _update_sample_window


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
