"""Pure evaluation and evidence helpers for the P6 execution preflight."""

from __future__ import annotations

from datetime import datetime
import json
import math
import os
from pathlib import Path
import tempfile


ARM_JOINTS = [f'joint{index}' for index in range(1, 7)]
STATUS_FIELDS = [
    'program_state', 'robot_state', 'main_error_code', 'sub_error_code',
    'robot_mode', 'tool_id', 'work_object_id', 'emergency_stop',
    'motion_done', 'collision_state', 'safety_stop_0', 'safety_stop_1',
    'enabled', 'payload_kg', 'payload_cog_x_mm', 'payload_cog_y_mm',
    'payload_cog_z_mm', 'tool_x_mm', 'tool_y_mm', 'tool_z_mm',
    'tool_rx_deg', 'tool_ry_deg', 'tool_rz_deg', 'wobj_x_mm',
    'wobj_y_mm', 'wobj_z_mm', 'wobj_rx_deg', 'wobj_ry_deg',
    'wobj_rz_deg', 'flange_x_mm', 'flange_y_mm', 'flange_z_mm',
    'flange_rx_deg', 'flange_ry_deg', 'flange_rz_deg',
]


def _rotation_from_rpy(roll, pitch, yaw):
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]


def _rotation_from_quaternion(quaternion):
    x, y, z, w = [float(value) for value in quaternion]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ]


def pose_error(first, second):
    """Return translation (mm) and rotation (deg) between pose dictionaries."""
    translation_first = [float(value) for value in first['translation_m']]
    translation_second = [float(value) for value in second['translation_m']]
    translation_mm = 1000.0 * math.sqrt(sum(
        (a - b) ** 2 for a, b in zip(translation_first, translation_second)))
    rotation_first = _rotation_from_quaternion(first['quaternion_xyzw'])
    rotation_second = _rotation_from_quaternion(second['quaternion_xyzw'])
    trace = sum(
        rotation_first[row][column] * rotation_second[row][column]
        for row in range(3) for column in range(3))
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return translation_mm, math.degrees(math.acos(cosine))


def pose_from_xyz_rpy(xyz_m, rpy_rad):
    rotation = _rotation_from_rpy(*[float(value) for value in rpy_rad])
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = [
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
            0.25 * scale,
        ]
    else:
        largest = max(range(3), key=lambda index: rotation[index][index])
        if largest == 0:
            scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
            quaternion = [0.25 * scale, (rotation[0][1] + rotation[1][0]) / scale,
                          (rotation[0][2] + rotation[2][0]) / scale,
                          (rotation[2][1] - rotation[1][2]) / scale]
        elif largest == 1:
            scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
            quaternion = [(rotation[0][1] + rotation[1][0]) / scale, 0.25 * scale,
                          (rotation[1][2] + rotation[2][1]) / scale,
                          (rotation[0][2] - rotation[2][0]) / scale]
        else:
            scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
            quaternion = [(rotation[0][2] + rotation[2][0]) / scale,
                          (rotation[1][2] + rotation[2][1]) / scale, 0.25 * scale,
                          (rotation[1][0] - rotation[0][1]) / scale]
    return {'translation_m': list(map(float, xyz_m)), 'quaternion_xyzw': quaternion}


def controller_flange_pose(status):
    xyz = [status[f'flange_{axis}_mm'] / 1000.0 for axis in ('x', 'y', 'z')]
    rpy = [math.radians(status[f'flange_r{axis}_deg']) for axis in ('x', 'y', 'z')]
    return pose_from_xyz_rpy(xyz, rpy)


def _finite_vector(value, length):
    return (isinstance(value, list) and len(value) == length and
            all(isinstance(item, (int, float)) and math.isfinite(item)
                for item in value))


def _valid_pose(value):
    return (isinstance(value, dict) and
            _finite_vector(value.get('translation_m'), 3) and
            _finite_vector(value.get('quaternion_xyzw'), 4))


def evaluate(snapshot, config, confirmations):
    """Evaluate a collected P6 snapshot without touching the robot."""
    checks = []

    def add(name, passed, observed, expected, detail=''):
        checks.append({
            'name': name, 'passed': bool(passed), 'observed': observed,
            'expected': expected, 'detail': detail,
        })

    add('control_path', config.get('control_path') == 'moveit_joint_servoj',
        config.get('control_path'), 'moveit_joint_servoj',
        'Tool/work-object IDs do not transform joint ServoJ trajectories.')

    status = snapshot.get('controller_status') or {}
    missing = [name for name in STATUS_FIELDS if name not in status]
    add('controller_status_complete', not missing, missing, [],
        'Status must come from the existing fairino_hardware SDK connection.')
    if not missing:
        add('controller_program_stopped', int(status['program_state']) == 1,
            status['program_state'], 1)
        add('controller_robot_stopped', int(status['robot_state']) == 1,
            status['robot_state'], 1)
        add('controller_error_clear',
            int(status['main_error_code']) == 0 and int(status['sub_error_code']) == 0,
            [status['main_error_code'], status['sub_error_code']], [0, 0])
        add('controller_emergency_stop_clear', int(status['emergency_stop']) == 0,
            status['emergency_stop'], 0)
        add('controller_safety_stops_clear',
            int(status['safety_stop_0']) == 0 and int(status['safety_stop_1']) == 0,
            [status['safety_stop_0'], status['safety_stop_1']], [0, 0])
        add('controller_collision_clear', int(status['collision_state']) == 0,
            status['collision_state'], 0)
        add('controller_motion_done', int(status['motion_done']) == 1,
            status['motion_done'], 1)
        add('controller_enabled', int(status['enabled']) == 1,
            status['enabled'], 1)

        payload = config.get('payload') or {}
        payload_required = bool(payload.get('required', True))
        expected_mass = payload.get('expected_mass_kg')
        expected_cog = payload.get('expected_cog_mm')
        basis = str(payload.get('basis', '')).strip()
        actual_payload = {
            'mass_kg': status['payload_kg'],
            'cog_mm': [status[f'payload_cog_{axis}_mm'] for axis in ('x', 'y', 'z')],
        }
        if not payload_required:
            waiver_reason = str(payload.get('waiver_reason', '')).strip()
            add('payload_prototype_waiver_recorded', bool(waiver_reason),
                {'controller': actual_payload, 'reason': waiver_reason},
                'non-empty operator-directed prototype waiver')
        else:
            defined = (
                isinstance(expected_mass, (int, float)) and expected_mass > 0.0 and
                _finite_vector(expected_cog, 3) and bool(basis))
            add('payload_basis_defined', defined,
                {'mass_kg': expected_mass, 'cog_mm': expected_cog, 'basis': basis},
                'measured total end-effector mass and flange-relative CoG')
            if defined:
                mass_tolerance = float(payload.get('mass_tolerance_kg', 0.02))
                cog_tolerance = float(payload.get('cog_tolerance_mm', 3.0))
                add('payload_mass_matches',
                    abs(status['payload_kg'] - expected_mass) <= mass_tolerance,
                    status['payload_kg'], expected_mass)
                add('payload_cog_matches',
                    max(abs(a - b) for a, b in zip(
                        actual_payload['cog_mm'], expected_cog)) <= cog_tolerance,
                    actual_payload['cog_mm'], expected_cog)

        add('controller_tool_recorded_not_required_by_joint_path', True,
            {'tool_id': status['tool_id'],
             'tool_coord_mm_deg': [status[f'tool_{axis}_{unit}'] for axis, unit in (
                 ('x', 'mm'), ('y', 'mm'), ('z', 'mm'), ('rx', 'deg'),
                 ('ry', 'deg'), ('rz', 'deg'))]},
            'record only for moveit_joint_servoj')
        add('controller_work_object_recorded_not_required_by_joint_path', True,
            {'work_object_id': status['work_object_id']},
            'record only for moveit_joint_servoj')

    controllers = snapshot.get('controllers') or {}
    execution_name = str(config.get('execution_controller', ''))
    broadcaster_name = str(config.get('joint_state_broadcaster', ''))
    add('execution_controller_idle', controllers.get(execution_name) == 'inactive',
        controllers.get(execution_name), 'inactive')
    add('joint_state_broadcaster_active', controllers.get(broadcaster_name) == 'active',
        controllers.get(broadcaster_name), 'active')

    hardware = snapshot.get('hardware') or {}
    component_name = str(config.get('hardware_component', ''))
    component = hardware.get(component_name) or {}
    arm_interfaces = component.get('arm_command_interfaces') or {}
    add('hardware_component_active', component.get('state') == 'active',
        component.get('state'), 'active')
    add('all_arm_commands_available_unclaimed',
        len(arm_interfaces) == 6 and all(
            value.get('available') and not value.get('claimed')
            for value in arm_interfaces.values()), arm_interfaces,
        'six available and unclaimed joint position interfaces')

    add('trajectory_bridge_idle',
        snapshot.get('control_state') == 'monitoring_manual_ready',
        snapshot.get('control_state'), 'monitoring_manual_ready')
    add('single_arm_joint_state_source', snapshot.get('arm_joint_source_count') == 1,
        snapshot.get('arm_joint_source_count'), 1)
    expected_joint_publishers = sorted(config.get('expected_joint_state_publishers', []))
    add('joint_state_publishers_expected',
        sorted(snapshot.get('joint_state_publishers') or []) == expected_joint_publishers,
        sorted(snapshot.get('joint_state_publishers') or []), expected_joint_publishers)
    add('arm_joint_state_fresh',
        snapshot.get('arm_joint_state_age_s', math.inf) <=
        float(config.get('maximum_state_age_s', 0.5)),
        snapshot.get('arm_joint_state_age_s'), config.get('maximum_state_age_s', 0.5))

    tf_limits = config.get('transform_tolerances') or {}
    translation_limit = float(tf_limits.get('translation_mm', 0.2))
    rotation_limit = float(tf_limits.get('rotation_deg', 0.02))
    if not missing and _valid_pose(snapshot.get('base_to_flange')):
        translation, rotation = pose_error(
            controller_flange_pose(status), snapshot['base_to_flange'])
        add('ros_fk_matches_controller_flange',
            translation <= float(config.get('flange_translation_tolerance_mm', 0.5)) and
            rotation <= float(config.get('flange_rotation_tolerance_deg', 0.1)),
            {'translation_mm': translation, 'rotation_deg': rotation},
            {'translation_mm_max': config.get('flange_translation_tolerance_mm', 0.5),
             'rotation_deg_max': config.get('flange_rotation_tolerance_deg', 0.1)})
    else:
        add('ros_fk_matches_controller_flange', False, None, 'live base_link->flange_link')

    for name, observed_key, expected_key, count_key in (
            ('tcp_transform', 'flange_to_grasp', 'expected_tcp', None),
            ('handeye_transform', 'flange_to_camera', 'expected_handeye',
             'camera_link_static_source_count')):
        observed = snapshot.get(observed_key)
        expected = config.get(expected_key)
        if _valid_pose(observed) and _valid_pose(expected):
            translation, rotation = pose_error(observed, expected)
            add(name + '_matches',
                translation <= translation_limit and rotation <= rotation_limit,
                {'translation_mm': translation, 'rotation_deg': rotation},
                {'translation_mm_max': translation_limit,
                 'rotation_deg_max': rotation_limit})
        else:
            add(name + '_matches', False, observed, expected)
        if count_key:
            add('handeye_transform_unique', snapshot.get(count_key) == 1,
                snapshot.get(count_key), 1)

    nodes = snapshot.get('nodes') or []
    for required in config.get('required_nodes', []):
        add('required_node:' + required, nodes.count(required) == 1,
            nodes.count(required), 1)
    forbidden_present = sorted(set(nodes).intersection(config.get('forbidden_nodes', [])))
    add('forbidden_motion_nodes_absent', not forbidden_present,
        forbidden_present, [])

    for confirmation in (
            'workcell_clear', 'lift_clearance_50mm', 'cable_slack',
            'stop_method_ready', 'no_external_motion_source'):
        add('operator_confirmation:' + confirmation,
            confirmations.get(confirmation) is True,
            confirmations.get(confirmation, False), True,
            'Physical conditions cannot be inferred from RViz or the ROS graph.')

    return checks


def atomic_write(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def invalidate_gate(root, reason='preflight_in_progress'):
    """Fail closed before collecting so an interrupted run cannot leave PASS."""
    now = datetime.now().astimezone()
    gate_path = Path(root).expanduser().resolve() / 'p6_execution_gate.txt'
    gate = (
        'P6_EXECUTION_GATE_V1\n'
        'passed=false\n'
        f'generated_at={now.isoformat()}\n'
        'control_path=unverified\n'
        f'report_path={reason}\n'
    )
    atomic_write(gate_path, gate)
    return gate_path


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def write_evidence(root, snapshot, config, confirmations, checks):
    now = datetime.now().astimezone()
    run_id = str(int(now.timestamp() * 1_000_000_000))
    run_directory = Path(root).expanduser().resolve() / 'runs' / run_id
    report_path = run_directory / 'report.json'
    passed = all(check['passed'] for check in checks)
    report = {
        'schema_version': 1,
        'generated_at': now.isoformat(),
        'passed': passed,
        'control_path': config.get('control_path'),
        'confirmations': confirmations,
        'checks': checks,
        'snapshot': snapshot,
    }
    atomic_write(
        report_path,
        json.dumps(_json_safe(report), ensure_ascii=False, indent=2) + '\n')
    gate_path = Path(root).expanduser().resolve() / 'p6_execution_gate.txt'
    gate = (
        'P6_EXECUTION_GATE_V1\n'
        f'passed={str(passed).lower()}\n'
        f'generated_at={now.isoformat()}\n'
        f'control_path={config.get("control_path", "")}\n'
        f'report_path={report_path}\n'
    )
    atomic_write(gate_path, gate)
    return passed, report_path, gate_path
