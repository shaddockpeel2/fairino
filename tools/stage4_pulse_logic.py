"""Small, ROS-independent calculations for stage-four short pulses."""

import math


DEFAULT_SPEED_MPS = 0.005
VERIFIED_SPEED_MPS = 0.015
MAX_SPEED_MPS = 0.075
MAX_DURATION_S = 0.5
# The camera normally delivers 6-8 Hz but has occasional longer frame gaps.
# A missing/invalid target still stops immediately; this only covers a brief
# gap between otherwise valid frames during a 0.5 s diagnostic pulse.
TARGET_MAX_AGE_S = 0.65


def rotate_xy(vector, quaternion):
    """Rotate a camera-plane vector into base XY, discarding base Z."""
    x, y = vector
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid_camera_transform')
    qx, qy, qz, qw = (v / norm for v in quaternion)
    tx = -2.0 * qz * y
    ty = 2.0 * qz * x
    tz = 2.0 * (qx*y - qy*x)
    return (x + qw*tx + qy*tz - qz*ty,
            y + qw*ty + qz*tx - qx*tz)


def command_for_side(side, target, quaternion, *, gain_mps=0.020,
                     sign_x=1.0, sign_y=1.0,
                     max_speed_mps=DEFAULT_SPEED_MPS):
    """Use the stage-two proportional law for one off-center red target."""
    ex, ey = target['error_px']
    width, height = target['image_size_px']
    if side == 'left' and ex >= -40.0:
        raise ValueError('red_cube_not_left_of_center')
    if side == 'right' and ex <= 40.0:
        raise ValueError('red_cube_not_right_of_center')
    if side == 'up' and ey >= -30.0:
        raise ValueError('red_cube_not_above_center')
    if side == 'down' and ey <= 30.0:
        raise ValueError('red_cube_not_below_center')
    if (min(width, height) <= 0
            or not 0.0 < max_speed_mps <= MAX_SPEED_MPS
            or not all(math.isfinite(v) for v in (
                ex, ey, width, height, gain_mps, sign_x, sign_y,
                max_speed_mps))):
        raise ValueError('invalid_target_geometry')
    # Isolate the requested image axis so each pulse tests one sign at a time.
    camera_xy = ((sign_x * gain_mps * ex / width, 0.0)
                 if side in ('left', 'right') else
                 (0.0, sign_y * gain_mps * ey / height))
    base_xy = rotate_xy(camera_xy, quaternion)
    magnitude = math.hypot(*base_xy)
    if magnitude < 1e-6:
        raise ValueError('camera_command_has_no_base_xy_component')
    if magnitude > max_speed_mps:
        base_xy = tuple(value * max_speed_mps / magnitude
                        for value in base_xy)
    return base_xy


def error_improved(side, before, after, tolerance_px=0.5):
    """Require a small signed improvement from one bounded short pulse."""
    axis = 0 if side in ('left', 'right') else 1
    return abs(after[axis]) < abs(before[axis]) - tolerance_px


def pulse_nominal_limit_m(speed_mps):
    if speed_mps <= 0.010:
        return 0.0025
    if speed_mps <= VERIFIED_SPEED_MPS:
        return 0.005
    return 0.015


def pulse_observed_limit_m(speed_mps, duration_s):
    if speed_mps <= VERIFIED_SPEED_MPS:
        return 0.005
    # Allow a bounded post-zero tail when validating an untested speed.
    return min(0.020, speed_mps * duration_s + 0.005)


def pulse_verdict(command_xy, duration_s, tcp_delta, *,
                  servo_targets, controller_outputs, nonzero_count):
    if nonzero_count == 0:
        return 'no_nonzero_twist_published'
    if servo_targets == 0:
        return 'no_servo_trajectory_observed'
    if controller_outputs == 0:
        return 'no_controller_output_observed'
    if tcp_delta is None or not all(math.isfinite(v) for v in tcp_delta):
        return 'tcp_feedback_unavailable'
    along = sum(a*b for a, b in zip(tcp_delta[:2], command_xy))
    speed = math.hypot(*command_xy)
    nominal = speed * duration_s
    actual_along = along / speed
    if nominal <= 0 or along <= 0:
        return 'wrong_direction_or_no_motion'
    if math.hypot(*tcp_delta[:2]) > pulse_observed_limit_m(
            speed, duration_s):
        return 'displacement_limit_exceeded'
    if speed > VERIFIED_SPEED_MPS and servo_targets < 2:
        return 'insufficient_servo_trajectories'
    # Be tolerant of short-pulse smoothing, but a tiny response cannot verify
    # that the command scale and controller chain are working as intended.
    if actual_along < nominal * 0.2:
        return 'undertravel'
    return 'direction_confirmed'
