import math

import numpy as np
import pytest

from p1_forward_replay_analysis import ForwardKinematics, analyze


def test_fk_rotates_tcp_offset_and_applies_fixed_origin():
    fk = ForwardKinematics('''<robot>
      <joint name="joint1" type="revolute">
        <parent link="base_link"/><child link="arm"/>
        <origin xyz="1 2 3"/><axis xyz="0 0 1"/>
      </joint>
      <joint name="tcp" type="fixed">
        <parent link="arm"/><child link="grasp_frame"/>
        <origin xyz="1 0 0"/>
      </joint></robot>''')
    assert np.allclose(fk.position({'joint1': math.pi / 2}), [1, 3, 3])


def test_feedback_relative_step_is_not_misread_as_scaled_velocity():
    fk = ForwardKinematics('''<robot>
      <joint name="joint1" type="prismatic">
        <parent link="base_link"/><child link="grasp_frame"/>
        <axis xyz="1 0 0"/>
      </joint></robot>''')
    record = {
        'backend': 'forward_position', 'command_xy_mps': [0.005, 0],
        'joint_states': [], 'forward_commands': [],
    }
    # Slow plant, but the requested single-cycle step is the full 0.1 mm.
    for i in range(20):
        actual = i * 0.000001
        record['joint_states'].append({
            't': i * 0.02, 'positions_rad': [actual, 0, 0, 0, 0, 0]})
        record['forward_commands'].append({
            't': i * 0.02 + 0.001, 'phase': 'motion',
            'positions_rad': [actual + 0.0001, 0, 0, 0, 0, 0]})
    result = analyze(record, fk)
    assert result['nominal_single_cycle_step_mm'] == pytest.approx(0.1)
    assert result['target_lead_along_mm_after_warmup']['p50'] == pytest.approx(0.1)
    assert result['feedback_receive_age_ms']['p50'] == pytest.approx(1)
    # Old or future feedback must not be used to manufacture a paired sample.
    record['joint_states'] = [record['joint_states'][5]]
    result = analyze(record, fk)
    assert result['skipped_command_count'] == 18


def test_invalid_chain_and_empty_evidence_fail_explicitly():
    with pytest.raises(ValueError, match='serial chain'):
        ForwardKinematics('<robot/>')
    with pytest.raises(ValueError, match='forward_position'):
        analyze({'backend': 'jtc'}, None)
