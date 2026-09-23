#!/usr/bin/env python3
"""Isolated, opt-in tilted pick_hold experiment using the existing P5 pipeline."""

import argparse
import copy
import math
import os
from pathlib import Path
import subprocess
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / 'evidence/tilted_red_grasp/runs'


def tilted_parameters(angle_deg, depth_mm=10.0, contact_toward_flange_mm=0.0):
    """R_nominal * Rx(tilt): retain local X closing direction, tilt in YZ.

    The calibrated grasp_frame remains the planning TCP. A nonzero contact
    offset tells the planner which physical point on the finger, rather than
    the TCP itself, must coincide with the requested point on the cube. The
    existing attachedCube() stores the full inverse grasp transform and thus
    preserves the upright cube during lift.
    """
    if not math.isfinite(angle_deg) or not 5.0 <= abs(angle_deg) <= 45.0:
        raise ValueError('Test angle must satisfy 5 <= abs(angle_deg) <= 45 degrees')
    if not math.isfinite(depth_mm) or not 5.0 <= depth_mm <= 15.0:
        raise ValueError('Test depth must satisfy 5 <= depth_mm <= 15 mm')
    if (not math.isfinite(contact_toward_flange_mm) or
            not 0.0 <= contact_toward_flange_mm <= 55.0):
        raise ValueError('Contact offset must satisfy 0 <= offset <= 55 mm')
    half = math.radians(angle_deg) / 2.0
    return {
        'grasp_orientation_xyzw': [0.0, math.cos(half), -math.sin(half), 0.0],
        # Production now searches 0, +/-10, +/-20 and +/-30 degrees relative
        # to its nominal tool pose. This isolated diagnostic requests one exact
        # absolute tilt, so disable additional relative candidates here.
        'grasp_tilt_candidates_rad': [0.0],
        'use_target_yaw': True,
        'target_yaw_offset_rad': -math.pi / 2.0,
        'use_target_size': True,
        'object_size_m': [0.030, 0.030, 0.030],
        # Only the first tabletop layer; reject the previous four-layer tower.
        'target_top_z_bounds_m': [0.025, 0.040],
        'grasp_depth_m': depth_mm / 1000.0,
        # grasp_frame +Z points from flange toward the fingertips. A positive
        # user value selects a point back along the finger face, toward flange.
        'grasp_contact_offset_xyz_m': [
            0.0, 0.0, -contact_toward_flange_mm / 1000.0],
        'approach_clearance_m': 0.050,
        'lift_distance_m': 0.050,
        'velocity_scale': 0.030,
        'acceleration_scale': 0.030,
        'sort_enabled': False,
        'stack_enabled': False,
        'log_directory': str(RUNS),
    }


def make_config(angle_deg, depth_mm=10.0, contact_toward_flange_mm=0.0):
    source = ROOT / 'src/fairino_mtc_demo/config/first_grasp.yaml'
    config = copy.deepcopy(yaml.safe_load(source.read_text()))
    config['fr3c_first_grasp']['ros__parameters'].update(
        tilted_parameters(angle_deg, depth_mm, contact_toward_flange_mm))
    return config


def check_runtime(actual, angle_deg, depth_mm, contact_toward_flange_mm, execute):
    expected = {
        **tilted_parameters(angle_deg, depth_mm, contact_toward_flange_mm),
        'execute': execute,
    }
    for name, value in expected.items():
        observed = actual.get(name)
        if isinstance(value, list):
            matches = isinstance(observed, list) and len(value) == len(observed) and all(
                math.isclose(a, b, abs_tol=1e-9) for a, b in zip(value, observed))
        elif isinstance(value, float):
            matches = isinstance(observed, (int, float)) and math.isclose(
                value, observed, abs_tol=1e-9)
        else:
            matches = observed == value
        if not matches:
            raise RuntimeError(f'Refusing goal: {name}={observed!r}, expected {value!r}. '
                               'Start the matching tilted test launch first.')


def ros_node():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('tilted_red_test_client')
    # Allow graph discovery before checking for an existing stack.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node


def wait_result(node, future, timeout=10.0):
    import rclpy
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError('ROS request timed out')
    return future.result()


def wait_for_task_node(node, timeout=30.0):
    import rclpy
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        count = node.get_node_names_and_namespaces().count(('fr3c_first_grasp', '/'))
        if count == 1:
            return
        if count > 1:
            raise RuntimeError('发现多个 /fr3c_first_grasp 节点，请先退出重复的 launch')
        rclpy.spin_once(node, timeout_sec=0.2)
    raise RuntimeError('等待 30 秒仍未发现 /fr3c_first_grasp。请先在另一终端运行 '
                       'test_tilted_red.sh launch --angle-deg 20，并检查该终端启动错误。'
                       '两终端应使用相同 ROS_DOMAIN_ID。')


def launch_test(args):
    import rclpy
    node = ros_node()
    try:
        active = set(node.get_node_names()) & {
            'fr3c_first_grasp', 'move_group', 'controller_manager',
        }
        if active:
            raise RuntimeError('Existing robot stack detected: ' + ', '.join(sorted(active)) +
                               '. Stop its launch normally before starting this isolated test.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    config_dir = ROOT / 'evidence/tilted_red_grasp/configs'
    config_dir.mkdir(parents=True, exist_ok=True)
    config_path = config_dir / f'{time.time_ns()}.yaml'
    config_path.write_text(yaml.safe_dump(
        make_config(args.angle_deg, args.depth_mm, args.contact_toward_flange_mm),
        sort_keys=False))
    print(f'Tilt={args.angle_deg:g} deg; depth={args.depth_mm:g} mm; '
          f'contact_toward_flange={args.contact_toward_flange_mm:g} mm; '
          f'execute={args.execute}; config={config_path}', flush=True)
    print('After startup, use the run command in a second terminal with matching options.',
          flush=True)
    launch_env = os.environ.copy()
    if args.execute:
        launch_env['FR3C_ENABLE_FT232H_PWM'] = 'I_UNDERSTAND_OPEN_LOOP_PWM'
    else:
        launch_env.pop('FR3C_ENABLE_FT232H_PWM', None)
    return subprocess.call([
        'ros2', 'launch', 'fairino_mtc_demo', 'full_first_grasp.launch.py',
        'detection_mode:=colors', 'sort_enabled:=false', 'stack_enabled:=false',
        'start_first_grasp:=true', 'start_blue_cube_localizer:=true',
        f'task_config:={config_path}',
        f'first_grasp_execute:={str(args.execute).lower()}',
        # Keep the normal hardware/controller configuration and startup gate.
        # No action goal or gripper command is sent by this launcher.
        'gripper_backend:=ft232h', 'gripper_hardware_output_enabled:=true',
        'run_p6_preflight:=true',
    ], env=launch_env)


def run_test(args):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters
    from fairino_msgs.action import PickColor

    node = ros_node()
    goal_handle = None
    try:
        wait_for_task_node(node)
        client = node.create_client(GetParameters, '/fr3c_first_grasp/get_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Grasp parameter service unavailable')
        names = list(tilted_parameters(
            args.angle_deg, args.depth_mm, args.contact_toward_flange_mm)) + ['execute']
        response = wait_result(node, client.call_async(GetParameters.Request(names=names)))
        actual = dict(zip(names, map(parameter_value_to_python, response.values)))
        check_runtime(actual, args.angle_deg, args.depth_mm,
                      args.contact_toward_flange_mm, args.execute)
        action = ActionClient(node, PickColor, '/fr3c_first_grasp/pick_hold')
        if not action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('pick_hold action unavailable')
        goal = PickColor.Goal(color='red', zone='', support_color='')
        goal_handle = wait_result(node, action.send_goal_async(
            goal, feedback_callback=lambda msg: print(
                f'{msg.feedback.phase}: {msg.feedback.detail}', flush=True)))
        if not goal_handle.accepted:
            raise RuntimeError('Red pick_hold goal rejected')
        result = wait_result(node, goal_handle.get_result_async(), timeout=180.0).result
        print(f'success={result.success}, executed={result.executed}, '
              f'error={result.error_code}: {result.message}')
        print(f'Trajectories and target snapshot: {RUNS}')
        return 0 if result.success else 1
    except (KeyboardInterrupt, RuntimeError):
        if goal_handle is not None and goal_handle.accepted:
            wait_result(node, goal_handle.cancel_goal_async(), timeout=5.0)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['config', 'launch', 'run'])
    parser.add_argument('--angle-deg', type=float, default=30.0,
                        help='signed local-X tilt, 5..45 degrees in magnitude (default: 30)')
    parser.add_argument('--depth-mm', type=float, default=10.0,
                        help='contact depth below cube top, 5..15 mm (default: 10)')
    parser.add_argument('--contact-toward-flange-mm', type=float, default=0.0,
                        help='contact point offset from TCP along the finger face toward flange, '
                             '0..55 mm (default: 0)')
    parser.add_argument('--execute', action='store_true',
                        help='launch/run real pick_hold; omitted means plan only')
    args = parser.parse_args()
    try:
        config = make_config(
            args.angle_deg, args.depth_mm, args.contact_toward_flange_mm)
        if args.command == 'config':
            print(yaml.safe_dump(config, sort_keys=False), end='')
            return 0
        return launch_test(args) if args.command == 'launch' else run_test(args)
    except (ValueError, RuntimeError) as error:
        parser.exit(2, f'{error}\n')
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
