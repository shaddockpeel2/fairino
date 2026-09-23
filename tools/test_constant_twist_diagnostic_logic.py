import pytest

from constant_twist_diagnostic_logic import (
    interval_summary, motion_summary, p1_acceptance, percentile,
    phase_travel_within, projected_displacement, tcp_window_speeds,
    vector_norm)


def test_percentile_and_interval_summary():
    assert percentile([3, 1, 2], 0.5) == 2
    summary = interval_summary([{'t': 0.0}, {'t': 0.1}, {'t': 0.2}])
    assert summary['mean_rate_hz'] == pytest.approx(10.0)
    assert summary['interval_ms_p99'] == pytest.approx(100.0)


def test_tcp_speed_is_projected_along_command():
    samples = [
        {'t': 0.0, 'xyz_m': [0.0, 0.0, 0.2]},
        {'t': 0.1, 'xyz_m': [0.001, 0.002, 0.2]},
        {'t': 0.2, 'xyz_m': [0.002, 0.004, 0.2]},
    ]
    assert tcp_window_speeds(samples, (0.005, 0.0)) == pytest.approx(
        [0.01, 0.01])
    assert tcp_window_speeds(samples, (0.0, 0.005)) == pytest.approx(
        [0.02, 0.02])


def test_vector_norm_rejects_missing_or_invalid_vectors():
    assert vector_norm([3.0, 4.0]) == pytest.approx(5.0)
    assert vector_norm([]) is None
    assert vector_norm([float('nan')]) is None


def test_forward_backend_summary_uses_float64_command_stream():
    record = {
        'backend': 'forward_position',
        'forward_commands': [
            {'t': 0.0, 'phase': 'motion', 'positions_rad': [0.0] * 6},
            {'t': 0.02, 'phase': 'motion', 'positions_rad': [0.001] * 6},
        ],
    }
    summary = motion_summary(record, (0.005, 0.0))
    assert summary['expected_backend'] == 'forward_position'
    assert summary['expected_backend_command_count'] == 2
    assert summary['forward_position_command_timing'][
        'mean_rate_hz'] == pytest.approx(50.0)


def test_p1_displacement_and_stop_metrics():
    start = {'xyz_m': [0.0, 0.0, 0.2]}
    stop = {'xyz_m': [0.009, 0.001, 0.2]}
    final = {'xyz_m': [0.0092, 0.001, 0.2]}
    assert projected_displacement(start, stop, (0.005, 0.0)) == pytest.approx(
        0.009)
    samples = [
        {'t': 3.0, 'phase': 'post_zero', 'xyz_m': stop['xyz_m']},
        {'t': 3.5, 'phase': 'post_zero', 'xyz_m': final['xyz_m']},
        {'t': 4.0, 'phase': 'post_zero', 'xyz_m': [0.02, 0.0, 0.2]},
    ]
    assert phase_travel_within(samples, 'post_zero', 0.5) == pytest.approx(
        0.0002)
    summary = motion_summary({
        'duration_s': 2.0,
        'start_tcp': start,
        'stop_tcp': stop,
        'final_tcp': final,
        'tcp': samples,
    }, (0.005, 0.0))
    assert summary['tcp_command_speed_realization_ratio'] == pytest.approx(0.9)
    assert summary['tcp_final_along_displacement_m'] == pytest.approx(0.0092)


def test_p1_acceptance_requires_every_automated_gate():
    summary = {
        'tcp_final_along_displacement_m': 0.009,
        'tcp_command_speed_realization_ratio': 0.9,
        'tcp_post_zero_0_5s_travel_m': 0.0002,
        'hardware_metrics_ms': {
            'servo_j_interval_ms': {'p99': 10.0, 'max': 15.0},
        },
    }
    record = {
        'duration_s': 2.0,
        'command_xy_mps': [0.005, 0.0],
        'control_state_after': 'monitoring_manual_ready',
        'servo_status_history': [0],
        'backend_output_observed': True,
    }
    acceptance = p1_acceptance(record, summary)
    assert acceptance['automated_passed']
    summary['tcp_post_zero_0_5s_travel_m'] = 0.00031
    assert not p1_acceptance(record, summary)['automated_passed']
