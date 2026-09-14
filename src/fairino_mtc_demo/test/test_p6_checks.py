from pathlib import Path

from fairino_mtc_demo.p6_checks import (
    STATUS_FIELDS, evaluate, invalidate_gate, pose_from_xyz_rpy,
    write_evidence)


def _fixture():
    status = {name: 0.0 for name in STATUS_FIELDS}
    status.update({
        'program_state': 1.0,
        'robot_state': 1.0,
        'motion_done': 1.0,
        'enabled': 1.0,
        'payload_kg': 0.5,
        'payload_cog_x_mm': 1.0,
        'payload_cog_y_mm': 2.0,
        'payload_cog_z_mm': 3.0,
    })
    identity = pose_from_xyz_rpy([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    required = ['/controller_manager', '/joint_state_broadcaster']
    config = {
        'control_path': 'moveit_joint_servoj',
        'execution_controller': 'execution',
        'joint_state_broadcaster': 'joint_state_broadcaster',
        'hardware_component': 'hardware',
        'maximum_state_age_s': 0.5,
        'expected_joint_state_publishers': ['/joint_state_broadcaster'],
        'flange_translation_tolerance_mm': 0.5,
        'flange_rotation_tolerance_deg': 0.1,
        'transform_tolerances': {'translation_mm': 0.2, 'rotation_deg': 0.02},
        'payload': {
            'expected_mass_kg': 0.5,
            'expected_cog_mm': [1.0, 2.0, 3.0],
            'basis': 'measured fixture',
        },
        'expected_tcp': identity,
        'expected_handeye': identity,
        'required_nodes': required,
        'forbidden_nodes': ['/ros2_cmd_server'],
    }
    snapshot = {
        'controller_status': status,
        'controllers': {'execution': 'inactive', 'joint_state_broadcaster': 'active'},
        'hardware': {'hardware': {
            'state': 'active',
            'arm_command_interfaces': {
                f'joint{index}/position': {'available': True, 'claimed': False}
                for index in range(1, 7)},
        }},
        'control_state': 'monitoring_manual_ready',
        'arm_joint_source_count': 1,
        'joint_state_publishers': ['/joint_state_broadcaster'],
        'arm_joint_state_age_s': 0.01,
        'base_to_flange': identity,
        'flange_to_grasp': identity,
        'flange_to_camera': identity,
        'camera_link_static_source_count': 1,
        'nodes': required,
    }
    confirmations = {
        'workcell_clear': True,
        'lift_clearance_50mm': True,
        'cable_slack': True,
        'stop_method_ready': True,
        'no_external_motion_source': True,
    }
    return snapshot, config, confirmations


def test_complete_safe_snapshot_passes():
    snapshot, config, confirmations = _fixture()
    checks = evaluate(snapshot, config, confirmations)
    assert checks
    assert all(check['passed'] for check in checks)


def test_unresolved_payload_and_safety_stop_fail_closed():
    snapshot, config, confirmations = _fixture()
    config['payload']['expected_mass_kg'] = None
    config['payload']['expected_cog_mm'] = None
    config['payload']['basis'] = ''
    snapshot['controller_status']['safety_stop_1'] = 1.0
    failed = {
        check['name'] for check in evaluate(snapshot, config, confirmations)
        if not check['passed']}
    assert 'payload_basis_defined' in failed
    assert 'controller_safety_stops_clear' in failed


def test_operator_directed_payload_waiver_allows_prototype():
    snapshot, config, confirmations = _fixture()
    config['payload'] = {
        'required': False,
        'expected_mass_kg': None,
        'expected_cog_mm': None,
        'basis': '',
        'waiver_reason': 'operator requested approximate first grasp',
    }
    snapshot['controller_status']['payload_kg'] = 0.0
    snapshot['controller_status']['payload_cog_x_mm'] = 0.0
    snapshot['controller_status']['payload_cog_y_mm'] = 0.0
    snapshot['controller_status']['payload_cog_z_mm'] = 0.0
    checks = evaluate(snapshot, config, confirmations)
    assert all(check['passed'] for check in checks)
    assert any(
        check['name'] == 'payload_prototype_waiver_recorded'
        for check in checks)


def test_manual_confirmation_and_forbidden_node_fail_closed():
    snapshot, config, confirmations = _fixture()
    confirmations['cable_slack'] = False
    snapshot['nodes'].append('/ros2_cmd_server')
    failed = {
        check['name'] for check in evaluate(snapshot, config, confirmations)
        if not check['passed']}
    assert 'operator_confirmation:cable_slack' in failed
    assert 'forbidden_motion_nodes_absent' in failed


def test_evidence_gate_is_always_overwritten(tmp_path):
    snapshot, config, confirmations = _fixture()
    checks = evaluate(snapshot, config, confirmations)
    passed, report, gate = write_evidence(
        tmp_path, snapshot, config, confirmations, checks)
    assert passed
    assert Path(report).is_file()
    assert 'passed=true' in Path(gate).read_text()

    confirmations['workcell_clear'] = False
    checks = evaluate(snapshot, config, confirmations)
    passed, _, gate = write_evidence(
        tmp_path, snapshot, config, confirmations, checks)
    assert not passed
    assert 'passed=false' in Path(gate).read_text()


def test_gate_is_invalidated_before_collection(tmp_path):
    gate = invalidate_gate(tmp_path)
    content = Path(gate).read_text()
    assert 'passed=false' in content
    assert 'control_path=unverified' in content
