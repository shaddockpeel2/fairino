"""Offline reference A/B with recorded FR3C pose and hypothetical lag plants.

No ROS, network or SDK imports. This is a mechanism experiment, not a robot
digital twin, collision checker, ServoJ benchmark or P1 hardware acceptance.
"""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from p1_bounded_reference import BoundedReference, Limits
from p1_forward_replay_analysis import ForwardKinematics


NAMES = tuple(f'joint{i}' for i in range(1, 7))


def pose(fk, q):
    return fk.transform(dict(zip(NAMES, q)))


def differential_velocity(fk, q, speed=0.005):
    """Central-difference geometric Jacobian; fixed orientation x+ velocity."""
    origin = pose(fk, q)
    jacobian = np.zeros((6, 6))
    epsilon = 1e-6
    for i in range(6):
        offset = np.zeros(6)
        offset[i] = epsilon
        plus, minus = pose(fk, q + offset), pose(fk, q - offset)
        jacobian[:3, i] = (plus[:3, 3] - minus[:3, 3]) / (2 * epsilon)
        skew = ((plus[:3, :3] - minus[:3, :3]) / (2 * epsilon)
                @ origin[:3, :3].T)
        jacobian[3:, i] = [skew[2, 1], skew[0, 2], skew[1, 0]]
    desired = np.array([speed, 0, 0, 0, 0, 0])
    condition = float(np.linalg.cond(jacobian))
    if not np.isfinite(condition) or condition >= 30:
        raise ValueError('Starting Jacobian is singular or ill-conditioned')
    qdot = np.linalg.solve(jacobian, desired)
    return qdot, condition, float(np.linalg.norm(jacobian @ qdot - desired))


def limits_from_urdf(urdf):
    joints = {j.get('name'): j for j in ET.fromstring(urdf).findall('joint')}
    bounds = [joints[name].find('limit') for name in NAMES]
    return Limits(
        lower=tuple(float(b.get('lower')) + 0.08 for b in bounds),
        upper=tuple(float(b.get('upper')) - 0.08 for b in bounds),
        velocity=tuple(min(float(b.get('velocity')), 0.1) for b in bounds),
        acceleration=(0.2,) * 6, following_error=(0.015,) * 6,
        travel=(0.10,) * 6)


def run_case(fk, initial, qdot, limits, *, mode, tau, duration,
             period=0.02, injection=None, frozen=False):
    """Plant: qdot=(target-q)/tau, rate limited; stop uses mock deceleration.

    Bounds here are geometric only. No world/self collision model is supplied.
    Actual/final TCP and stop travel are recomputed from FK, not joint norms.
    """
    start_pose = pose(fk, initial)
    origin = start_pose[:3, 3]

    def segment_allowed(actual, candidate):
        actual_tcp = pose(fk, actual)[:3, 3]
        for fraction in np.linspace(0, 1, 5):
            q = np.asarray(actual) + fraction * (np.asarray(candidate) - actual)
            tcp_pose = pose(fk, q)
            if np.linalg.norm(tcp_pose[:3, 3] - origin) > 0.012:
                return False
            if np.linalg.norm(tcp_pose[:3, 3] - actual_tcp) > 0.002:
                return False
            rotation = start_pose[:3, :3].T @ tcp_pose[:3, :3]
            angle = np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))
            if angle > np.deg2rad(1):
                return False
        return True

    controller = BoundedReference(limits, segment_allowed, mode)
    result = controller.arm(0, initial, 0, lease_active=True)
    if result.state != 'active':
        raise ValueError(f'Initial pose rejected: {result.reason}')
    q = initial.copy()
    velocity = np.zeros(6)
    target = initial.copy()
    step_s = 0.004
    stride = round(period / step_s)
    if abs(stride * step_s - period) > 1e-9:
        raise ValueError('Controller period must be a multiple of mock step')
    stop_t = None
    stop_q = None
    stop_distance = 0.0
    command_stamp = feedback_stamp = 0.0
    feedback = q.copy()
    rows = []
    max_lead = 0.0
    fault_at = duration / 2
    for i in range(1, round((duration + 0.5) / step_s) + 1):
        now = i * step_s
        if i % stride == 0 and controller.state == 'active':
            if injection != 'command_timeout' or now < fault_at:
                command_stamp = now
            if injection != 'feedback_timeout' or now < fault_at:
                feedback_stamp = now
                feedback = q.copy()
            requested = qdot if now < duration - 1e-10 else np.zeros(6)
            result = controller.step(
                now, feedback, feedback_stamp, requested, command_stamp,
                lease_active=not (injection == 'lease_lost' and now >= fault_at),
                cancel=injection == 'cancel' and now >= fault_at)
            if result.stop_required:
                # Explicitly revoke target. The mock then brakes its velocity.
                # Real hardware MUST supply and validate this separate stop path.
                target = None
                stop_t, stop_q = now, q.copy()
            else:
                target = np.array(result.position)
                max_lead = max(max_lead, float(np.linalg.norm(
                    pose(fk, target)[:3, 3] - pose(fk, q)[:3, 3])))
        previous_tcp = pose(fk, q)[:3, 3]
        if frozen:
            velocity[:] = 0
        elif target is None:
            brake = np.array(limits.acceleration) * step_s
            velocity -= np.clip(velocity, -brake, brake)
        else:
            desired = np.clip((target - q) / tau,
                              -np.array(limits.velocity), limits.velocity)
            delta = np.array(limits.acceleration) * step_s
            velocity += np.clip(desired - velocity, -delta, delta)
        q = q + velocity * step_s
        tcp = pose(fk, q)[:3, 3]
        if stop_t is not None and now - stop_t <= 0.5 + 1e-10:
            stop_distance += float(np.linalg.norm(tcp - previous_tcp))
        if i % stride == 0:
            rows.append({'t': now, 'actual_rad': q.tolist(),
                         'target_rad': target.tolist() if target is not None else None,
                         'tcp_m': tcp.tolist(), 'state': controller.state})
    displacement = float((pose(fk, q)[:3, 3] - origin)[0])
    return {
        'mode': mode, 'hypothetical_lag_tau_s': tau,
        'duration_s': duration, 'controller_period_s': period,
        'frozen_plant': frozen, 'injection': injection,
        'end_state': controller.state, 'stop_reason': controller.reason,
        'stop_time_s': stop_t, 'max_tcp_reference_lead_mm': 1000 * max_lead,
        'final_x_displacement_mm': 1000 * displacement,
        'final_displacement_ratio': displacement / (0.005 * duration),
        'command_window_x_displacement_mm': (
            float((pose(fk, stop_q)[:3, 3] - origin)[0] * 1000)
            if stop_q is not None else None),
        'mock_stop_path_travel_mm': stop_distance * 1000,
        'trace': rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', required=True, type=Path)
    parser.add_argument('--urdf', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    raw_record, raw_urdf = args.record.read_bytes(), args.urdf.read_bytes()
    record = json.loads(raw_record)
    initial = np.array(record['joint_states'][0]['positions_rad'])
    fk = ForwardKinematics(raw_urdf)
    qdot, condition, residual = differential_velocity(fk, initial)
    limits = limits_from_urdf(raw_urdf)
    cases = [run_case(fk, initial, qdot, limits, mode=mode, tau=tau,
                      duration=duration, period=period)
             for period in (0.020, 0.008)
             for duration in (0.5, 2.0)
             for tau in (0.02, 0.08, 0.30, 0.80)
             for mode in ('feedback_step', 'integrated')]
    for injection in ('command_timeout', 'feedback_timeout', 'cancel', 'lease_lost'):
        cases.append(run_case(fk, initial, qdot, limits, mode='integrated',
                              tau=0.08, duration=2, injection=injection))
    cases.append(run_case(fk, initial, qdot, limits, mode='integrated',
                          tau=0.08, duration=2, frozen=True))
    report = {
        'kind': 'offline_hypothetical_plant_only', 'hardware_gate_passed': False,
        'limitations': [
            'Lag values are hypotheses, not fitted FAIRINO dynamics.',
            'Fixed starting-pose Jacobian; not the MoveIt Servo implementation.',
            'No Butterworth, SDK/DDS delay, collision world or controller box model.',
            'Mock target revocation and braking require a real stop implementation.',
            'Geometric guard samples a path; it is not a continuous collision proof.',
        ],
        'record_sha256': hashlib.sha256(raw_record).hexdigest(),
        'urdf_sha256': hashlib.sha256(raw_urdf).hexdigest(),
        'initial_rad': initial.tolist(), 'requested_joint_velocity_radps': qdot.tolist(),
        'jacobian_condition': condition, 'jacobian_residual': residual,
        'cases': cases,
    }
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    for case in cases:
        print(f"{case['mode']:13} T={case['controller_period_s']:.3f} "
              f"tau={case['hypothetical_lag_tau_s']:.2f} "
              f"duration={case['duration_s']:.1f} "
              f"x={case['final_x_displacement_mm']:.3f}mm "
              f"stop={case['stop_reason']} injection={case['injection']} "
              f"frozen={case['frozen_plant']}")


if __name__ == '__main__':
    main()
