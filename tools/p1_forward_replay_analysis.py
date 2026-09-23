"""Offline FK analysis of P1 forward commands; never connects to ROS/hardware."""

import argparse
from bisect import bisect_right
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from constant_twist_diagnostic_logic import scalar_summary


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return (np.eye(3) + np.sin(angle) * skew
            + (1 - np.cos(angle)) * (skew @ skew))


class ForwardKinematics:
    """Evaluate a URDF serial chain, including fixed TCP offsets."""

    def __init__(self, urdf, base='base_link', tip='grasp_frame'):
        root = ET.fromstring(urdf)
        joints = {j.find('child').get('link'): j
                  for j in root.findall('joint')}
        self.chain = []
        visited = set()
        while tip != base:
            if tip in visited or tip not in joints:
                raise ValueError('No valid serial chain from base to tip')
            visited.add(tip)
            joint = joints[tip]
            self.chain.insert(0, joint)
            tip = joint.find('parent').get('link')

    def position(self, positions):
        result = np.eye(4)
        for joint in self.chain:
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = np.fromstring(
                    origin.get('xyz', '0 0 0'), sep=' ')
                roll, pitch, yaw = np.fromstring(
                    origin.get('rpy', '0 0 0'), sep=' ')
                transform[:3, :3] = (
                    rotation([0, 0, 1], yaw)
                    @ rotation([0, 1, 0], pitch)
                    @ rotation([1, 0, 0], roll))
            result = result @ transform
            kind = joint.get('type')
            if kind == 'fixed':
                continue
            value = float(positions[joint.get('name')])
            if not np.isfinite(value):
                raise ValueError('Non-finite joint position')
            axis_node = joint.find('axis')
            axis = np.fromstring(
                axis_node.get('xyz') if axis_node is not None else '1 0 0',
                sep=' ')
            transform = np.eye(4)
            if kind in ('revolute', 'continuous'):
                transform[:3, :3] = rotation(axis, value)
            elif kind == 'prismatic':
                transform[:3, 3] = axis * value
            else:
                raise ValueError(f'Unsupported joint type: {kind}')
            result = result @ transform
        return result[:3, 3]


def analyze(record, fk, period_s=0.02, warmup_s=0.1,
            max_feedback_age_s=0.04):
    if record.get('backend') != 'forward_position':
        raise ValueError('This analysis requires forward_position evidence')
    command = np.array([*record['command_xy_mps'], 0.0], dtype=float)
    speed = np.linalg.norm(command)
    if not np.isfinite(speed) or speed <= 0 or period_s <= 0:
        raise ValueError('A nonzero finite command and positive period are required')
    unit = command / speed
    # Diagnostic schema v2 explicitly reorders joint feedback to joint1..joint6.
    # ForwardCommandController is configured in the same order.
    names = [f'joint{i}' for i in range(1, 7)]

    def tcp(sample):
        values = sample['positions_rad']
        if len(values) != len(names):
            raise ValueError('Expected six joint positions in joint1..joint6 order')
        return fk.position(dict(zip(names, values)))

    joints = sorted(record['joint_states'], key=lambda s: s['t'])
    times = [s['t'] for s in joints]
    commands = sorted((s for s in record['forward_commands']
                       if s['phase'] == 'motion'), key=lambda s: s['t'])
    if not commands or not joints:
        raise ValueError('Missing motion commands or joint feedback')
    rows = []
    skipped = 0
    for sample in commands:
        index = bisect_right(times, sample['t']) - 1
        if index < 0 or sample['t'] - times[index] > max_feedback_age_s:
            skipped += 1
            continue
        actual = tcp(joints[index])
        target = tcp(sample)
        rows.append({
            't': sample['t'],
            'feedback_receive_age_ms': 1000 * (sample['t'] - times[index]),
            'target_tcp_m': target.tolist(),
            'feedback_tcp_m': actual.tolist(),
            'target_lead_along_mm': float((target - actual) @ unit * 1000),
        })
    settled = [row for row in rows
               if row['t'] >= commands[0]['t'] + warmup_s]
    return {
        'joint_order_assumption': names,
        'alignment': 'most recent received feedback; not Servo internal state',
        'publish_period_s_assumption': period_s,
        'warmup_excluded_s': warmup_s,
        'max_feedback_receive_age_s': max_feedback_age_s,
        'skipped_command_count': skipped,
        'nominal_single_cycle_step_mm': float(speed * period_s * 1000),
        'target_lead_along_mm_after_warmup': scalar_summary(
            row['target_lead_along_mm'] for row in settled),
        'feedback_receive_age_ms': scalar_summary(
            row['feedback_receive_age_ms'] for row in rows),
        'rows': rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', type=Path)
    parser.add_argument('--urdf', type=Path, required=True,
                        help='Expanded URDF matching the tested robot')
    parser.add_argument('--period-s', type=float, default=0.020)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    record_bytes = args.record.read_bytes()
    urdf_bytes = args.urdf.read_bytes()
    result = analyze(json.loads(record_bytes), ForwardKinematics(urdf_bytes),
                     period_s=args.period_s)
    result['record_sha256'] = hashlib.sha256(record_bytes).hexdigest()
    result['urdf_sha256'] = hashlib.sha256(urdf_bytes).hexdigest()
    result['record_path'] = str(args.record.resolve())
    result['urdf_path'] = str(args.urdf.resolve())
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}, indent=2))


if __name__ == '__main__':
    main()
