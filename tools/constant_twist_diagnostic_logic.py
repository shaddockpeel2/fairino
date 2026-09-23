"""ROS-independent summaries for the bounded constant-Twist diagnostic."""

import math
import statistics


def percentile(values, fraction):
    values = sorted(float(value) for value in values if math.isfinite(value))
    if not values:
        return None
    position = fraction * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def interval_summary(samples):
    times = [float(sample['t']) for sample in samples]
    intervals = [
        end - start for start, end in zip(times, times[1:])
        if math.isfinite(start) and math.isfinite(end) and end > start
    ]
    if not intervals:
        return {'sample_count': len(samples)}
    return {
        'sample_count': len(samples),
        'mean_rate_hz': 1.0 / statistics.mean(intervals),
        'interval_ms_p50': 1000.0 * percentile(intervals, 0.50),
        'interval_ms_p90': 1000.0 * percentile(intervals, 0.90),
        'interval_ms_p99': 1000.0 * percentile(intervals, 0.99),
        'interval_ms_max': 1000.0 * max(intervals),
    }


def scalar_summary(values):
    values = [float(value) for value in values if math.isfinite(value)]
    if not values:
        return {'sample_count': 0}
    return {
        'sample_count': len(values),
        'min': min(values),
        'p10': percentile(values, 0.10),
        'p50': percentile(values, 0.50),
        'p90': percentile(values, 0.90),
        'p99': percentile(values, 0.99),
        'max': max(values),
        'mean': statistics.mean(values),
    }


def vector_norm(values):
    if not values or not all(math.isfinite(float(value)) for value in values):
        return None
    return math.sqrt(sum(float(value) ** 2 for value in values))


def projected_displacement(start, end, command_xy):
    if not start or not end:
        return None
    magnitude = math.hypot(*command_xy)
    if magnitude <= 0.0:
        return None
    unit = (command_xy[0] / magnitude, command_xy[1] / magnitude)
    return sum(
        (float(end['xyz_m'][index]) - float(start['xyz_m'][index]))
        * unit[index]
        for index in range(2))


def phase_travel_within(samples, phase, window_s):
    phase_samples = [sample for sample in samples
                     if sample.get('phase') == phase]
    if len(phase_samples) < 2:
        return None
    start = phase_samples[0]
    deadline = float(start['t']) + window_s
    candidates = [sample for sample in phase_samples
                  if float(sample['t']) <= deadline]
    end = candidates[-1] if candidates else phase_samples[0]
    return math.dist(start['xyz_m'], end['xyz_m'])


def tcp_window_speeds(samples, command_xy, minimum_window_s=0.08,
                      maximum_window_s=0.14):
    magnitude = math.hypot(*command_xy)
    if magnitude <= 0.0:
        return []
    unit = (command_xy[0] / magnitude, command_xy[1] / magnitude)
    speeds = []
    for index, start in enumerate(samples):
        for end in samples[index + 1:]:
            dt = end['t'] - start['t']
            if dt >= minimum_window_s:
                if dt <= maximum_window_s:
                    delta = (end['xyz_m'][0] - start['xyz_m'][0],
                             end['xyz_m'][1] - start['xyz_m'][1])
                    along = delta[0] * unit[0] + delta[1] * unit[1]
                    speeds.append(along / dt)
                break
    return speeds


def motion_summary(record, command_xy):
    def motion(samples):
        return [sample for sample in samples
                if sample.get('phase') == 'motion']

    servo = motion(record.get('servo_trajectories', []))
    forward = motion(record.get('forward_commands', []))
    controller = motion(record.get('controller_states', []))
    joints = motion(record.get('joint_states', []))
    hardware = motion(record.get('hardware_timing', []))
    tcp = motion(record.get('tcp', []))
    reference_velocity_norms = [
        vector_norm(sample['reference'].get('velocities', []))
        for sample in controller]
    feedback_velocity_norms = [
        vector_norm(sample['feedback'].get('velocities', []))
        for sample in controller]
    velocity_error_norms = [
        vector_norm(sample['error'].get('velocities', []))
        for sample in controller]
    tcp_speeds = tcp_window_speeds(tcp, command_xy)
    timing_names = (
        'read_position_call_ms', 'read_status_call_ms', 'servo_j_call_ms',
        'servo_j_interval_ms', 'control_period_ms')
    final_displacement = projected_displacement(
        record.get('start_tcp'), record.get('final_tcp'), command_xy)
    command_displacement = projected_displacement(
        record.get('start_tcp'), record.get('stop_tcp'), command_xy)
    duration_s = float(record.get('duration_s', 0.0))
    commanded_speed = math.hypot(*command_xy)
    speed_realization = (
        command_displacement / duration_s / commanded_speed
        if command_displacement is not None and duration_s > 0.0
        and commanded_speed > 0.0 else None)
    return {
        'command_publish_timing': interval_summary(
            motion(record.get('twists', []))),
        'servo_trajectory_timing': interval_summary(servo),
        'servo_trajectory_has_velocity_count': sum(
            bool(sample['points'] and sample['points'][-1].get('velocities'))
            for sample in servo),
        'forward_position_command_timing': interval_summary(forward),
        'expected_backend': record.get('backend', 'jtc'),
        'expected_backend_command_count': (
            len(forward) if record.get('backend') == 'forward_position'
            else len(servo)),
        'controller_state_timing': interval_summary(controller),
        'joint_state_timing': interval_summary(joints),
        'hardware_timing_message_rate': interval_summary(hardware),
        'controller_reference_velocity_norm_radps': scalar_summary(
            value for value in reference_velocity_norms if value is not None),
        'controller_feedback_velocity_norm_radps': scalar_summary(
            value for value in feedback_velocity_norms if value is not None),
        'controller_velocity_error_norm_radps': scalar_summary(
            value for value in velocity_error_norms if value is not None),
        'tcp_along_command_speed_mps': scalar_summary(tcp_speeds),
        'tcp_command_displacement_m': command_displacement,
        'tcp_final_along_displacement_m': final_displacement,
        'tcp_command_speed_realization_ratio': speed_realization,
        'tcp_post_zero_0_5s_travel_m': phase_travel_within(
            record.get('tcp', []), 'post_zero', 0.5),
        'hardware_metrics_ms': {
            name: scalar_summary(
                sample['metrics'][name] for sample in hardware
                if name in sample['metrics'])
            for name in timing_names
        },
    }


def p1_acceptance(record, summary):
    """Evaluate automatable P1 thresholds; operator smoothness stays manual."""
    hardware = summary.get('hardware_metrics_ms', {})
    servo_interval = hardware.get('servo_j_interval_ms', {})
    checks = {
        'final_displacement_8_to_12_mm': (
            summary.get('tcp_final_along_displacement_m') is not None
            and 0.008 <= summary['tcp_final_along_displacement_m'] <= 0.012),
        'speed_realization_at_least_80_percent': (
            summary.get('tcp_command_speed_realization_ratio') is not None
            and summary['tcp_command_speed_realization_ratio'] >= 0.80),
        'post_zero_0_5s_travel_at_most_0_3_mm': (
            summary.get('tcp_post_zero_0_5s_travel_m') is not None
            and summary['tcp_post_zero_0_5s_travel_m'] <= 0.0003),
        'servoj_interval_p99_at_most_12_ms': (
            servo_interval.get('p99') is not None
            and servo_interval['p99'] <= 12.0),
        'servoj_interval_max_below_20_ms': (
            servo_interval.get('max') is not None
            and servo_interval['max'] < 20.0),
        'control_released': (
            record.get('control_state_after') == 'monitoring_manual_ready'),
        'no_hard_servo_fault': not any(
            status in (2, 4, 5)
            for status in record.get('servo_status_history', [])),
        'backend_output_observed': bool(
            record.get('backend_output_observed')),
    }
    applicable = (
        math.isclose(float(record.get('duration_s', 0.0)), 2.0,
                     abs_tol=1e-9)
        and math.isclose(
            math.hypot(*record.get('command_xy_mps', [0.0, 0.0])),
            0.005, abs_tol=1e-9))
    return {
        'applicable_to_5mmps_2s_gate': applicable,
        'automated_checks': checks,
        'automated_passed': applicable and all(checks.values()),
        'operator_smoothness_observation_required': True,
    }
