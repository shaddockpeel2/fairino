"""ROS 2 GripperCommand server for the open-loop YDI-J22 PWM gripper."""

from __future__ import annotations

import math
import os
import threading
import time

from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .ft232h_backend import Ft232hPwmBackend
from .mapping import GripperCalibration


HARDWARE_CONFIRMATION_ENV = 'FR3C_ENABLE_FT232H_PWM'
HARDWARE_CONFIRMATION_VALUE = 'I_UNDERSTAND_OPEN_LOOP_PWM'


class GripperNode(Node):
    """Expose an estimated GripperCommand interface over mock or FT232H PWM."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            'ydi_j22_gripper', parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._calibration = GripperCalibration(
            self.get_parameter('closed_position_m').value,
            self.get_parameter('open_position_m').value,
            self.get_parameter('operating_close_us').value,
            self.get_parameter('operating_open_us').value,
            self.get_parameter('electrical_min_us').value,
            self.get_parameter('electrical_max_us').value,
            tuple(self.get_parameter('calibration_positions_m').value),
            tuple(self.get_parameter('calibration_pulses_us').value),
        )
        self._calibration.validate()
        self._allowed_positions_m = tuple(
            float(value) for value in
            self.get_parameter('allowed_positions_m').value)
        if (len(self._allowed_positions_m) != 2
                or not math.isclose(
                    self._allowed_positions_m[0],
                    self._calibration.closed_position_m)
                or not math.isclose(
                    self._allowed_positions_m[1],
                    self._calibration.open_position_m)):
            raise ValueError(
                'allowed_positions_m must contain exactly [close, open]')
        self._backend_name = str(self.get_parameter('backend').value).lower()
        if self._backend_name not in ('mock', 'ft232h'):
            raise ValueError('backend must be mock or ft232h')
        self._hardware_output_enabled = bool(
            self.get_parameter('hardware_output_enabled').value)
        self._hardware_gate_open = (
            self._hardware_output_enabled
            and os.environ.get(HARDWARE_CONFIRMATION_ENV)
            == HARDWARE_CONFIRMATION_VALUE
        )
        self._assume_reached = bool(
            self.get_parameter('assume_reached_without_feedback').value)
        self._settle_time_s = float(self.get_parameter('settle_time_s').value)
        self._command_timeout_s = float(
            self.get_parameter('command_timeout_s').value)
        self._slew_us_per_s = float(
            self.get_parameter('slew_us_per_s').value)
        if self._settle_time_s < 0.0 or self._command_timeout_s <= 0.0:
            raise ValueError('settle_time_s must be >= 0 and command_timeout_s > 0')

        self._backend = None
        if self._backend_name == 'ft232h':
            self._backend = Ft232hPwmBackend(
                url=self.get_parameter('ftdi_url').value,
                sample_rate_hz=self.get_parameter('sample_rate_hz').value,
                frame_us=self.get_parameter('frame_us').value,
                signal_pin=self.get_parameter('signal_pin').value,
                command_duration_s=self.get_parameter(
                    'command_duration_s').value,
                post_write_delay_s=self.get_parameter(
                    'post_write_delay_s').value,
                electrical_min_us=self.get_parameter(
                    'electrical_min_us').value,
                electrical_max_us=self.get_parameter(
                    'electrical_max_us').value,
                pyftdi_site_packages=self.get_parameter(
                    'pyftdi_site_packages').value,
            )

        self._state_lock = threading.Lock()
        self._goal_reserved = False
        # This gripper has no position feedback, including after a process
        # restart. MoveIt nevertheless requires a complete joint state before
        # it can plan. Publish the configured startup aperture for both
        # backends as an explicitly unverified estimate. The default is fully
        # release position. This assignment does not open the FTDI device and
        # does not emit PWM.
        self._estimated_position_m = float(
            self.get_parameter('startup_position_m').value)
        self._calibration.position_to_pulse(self._estimated_position_m)
        self._position_source = (
            'mock_startup' if self._backend_name == 'mock'
            else 'startup_parameter_unverified')
        self._last_target_m = None
        self._last_result = 'never_commanded'

        self._joint_name = str(self.get_parameter('joint_name').value)
        callback_group = ReentrantCallbackGroup()
        self._joint_publisher = self.create_publisher(
            JointState, '/joint_states', 10)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self._action_server = ActionServer(
            self,
            GripperCommand,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self._timer = self.create_timer(
            0.2, self._publish_state, callback_group=callback_group)

        if self._backend_name == 'ft232h' and not self._hardware_gate_open:
            self.get_logger().warning(
                'FT232H backend is DISARMED: action goals will be rejected. '
                f'Set hardware_output_enabled:=true and {HARDWARE_CONFIRMATION_ENV}='
                f'{HARDWARE_CONFIRMATION_VALUE} to arm it.')
        elif self._backend_name == 'ft232h':
            self.get_logger().warning(
                'FT232H finite-burst PWM is ARMED. Starting this node emits no '
                'PWM; each accepted action opens FTDI, sends one bounded burst, '
                'pulls AD7 low and closes FTDI.')
        else:
            self.get_logger().info('YDI-J22 mock GripperCommand server ready.')
        if self._backend_name == 'ft232h':
            self.get_logger().warning(
                'Publishing startup_position_m='
                f'{self._estimated_position_m:.3f} '
                'as an UNVERIFIED joint-state '
                'estimate for MoveIt; send an explicit pre-open command '
                'before using the grasp opening for planning.')

    def _declare_parameters(self):
        defaults = {
            'action_name': '/ydi_j22_gripper_controller/gripper_cmd',
            'joint_name': 'gripper_command_joint',
            'backend': 'mock',
            'closed_position_m': 0.027,
            'open_position_m': 0.060,
            'startup_position_m': 0.060,
            'allowed_positions_m': [0.027, 0.060],
            'operating_close_us': 820,
            'operating_open_us': 1300,
            'electrical_min_us': 500,
            'electrical_max_us': 2500,
            'calibration_positions_m': [0.027, 0.060],
            'calibration_pulses_us': [820, 1300],
            'aperture_calibrated': False,
            'hardware_output_enabled': False,
            'assume_reached_without_feedback': True,
            'settle_time_s': 0.6,
            'command_timeout_s': 8.0,
            'slew_us_per_s': 400.0,
            'ftdi_url': 'ftdi://ftdi:232h/1',
            'pyftdi_site_packages': '',
            'sample_rate_hz': 50000,
            'frame_us': 20000,
            'signal_pin': 7,
            'command_duration_s': 0.8,
            'post_write_delay_s': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _goal_callback(self, goal_request):
        position = float(goal_request.command.position)
        effort = float(goal_request.command.max_effort)
        try:
            self._calibration.position_to_pulse(position)
        except (TypeError, ValueError) as error:
            self.get_logger().error(f'Rejecting invalid gripper position: {error}')
            return GoalResponse.REJECT
        if not any(
                math.isclose(
                    position, allowed, rel_tol=0.0, abs_tol=1e-6)
                for allowed in self._allowed_positions_m):
            self.get_logger().error(
                'Rejecting unsupported gripper position: only 0.027 m '
                'and 0.060 m are enabled')
            return GoalResponse.REJECT
        if not math.isfinite(effort) or effort < 0.0:
            self.get_logger().error('Rejecting invalid max_effort')
            return GoalResponse.REJECT
        if effort > 0.0:
            self.get_logger().error(
                'Rejecting max_effort > 0: YDI-J22 has no force/current feedback')
            return GoalResponse.REJECT
        if self._backend_name == 'ft232h' and not self._hardware_gate_open:
            self.get_logger().error('Rejecting goal: FT232H hardware output is disarmed')
            return GoalResponse.REJECT
        with self._state_lock:
            if self._goal_reserved:
                self.get_logger().warning('Rejecting concurrent gripper goal: busy')
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        target_m = float(goal_handle.request.command.position)
        target_us = self._calibration.position_to_pulse(target_m)
        self.get_logger().warning(
            f'GOAL accepted: aperture={target_m * 1000.0:.1f} mm, '
            f'mapped_pulse={target_us} us, backend={self._backend_name}')
        try:
            with self._state_lock:
                start_m = self._estimated_position_m
                self._last_target_m = target_m
                self._last_result = 'executing'
            if self._backend_name == 'mock':
                return self._execute_mock(goal_handle, start_m, target_m)
            return self._execute_ft232h(goal_handle, target_m, target_us)
        except Exception as error:
            self.get_logger().error(
                f'Gripper command failed: {type(error).__name__}: {error}')
            with self._state_lock:
                self._last_result = f'fault:{type(error).__name__}'
            goal_handle.abort()
            return self._make_result(False)
        finally:
            with self._state_lock:
                self._goal_reserved = False

    def _execute_mock(self, goal_handle, start_m, target_m):
        if start_m is None:
            start_m = target_m
        start_us = self._calibration.position_to_pulse(start_m)
        target_us = self._calibration.position_to_pulse(target_m)
        duration = abs(target_us - start_us) / self._slew_us_per_s
        duration = max(0.05, duration)
        started = time.monotonic()
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self._state_lock:
                    self._last_result = 'canceled'
                return self._make_result(False)
            fraction = min(1.0, (time.monotonic() - started) / duration)
            position = start_m + fraction * (target_m - start_m)
            self._set_position(position)
            goal_handle.publish_feedback(self._make_feedback(False))
            if fraction >= 1.0:
                break
            time.sleep(0.05)
        time.sleep(self._settle_time_s)
        goal_handle.succeed()
        with self._state_lock:
            self._last_result = 'estimated_reached_mock'
        return self._make_result(True)

    def _execute_ft232h(self, goal_handle, target_m, requested_us):
        prior_us = self._backend.current_us
        effective_duration_s = self._backend.command_duration_s
        self.get_logger().warning(
            f'PWM burst starting: requested_pulse={requested_us} us, '
            f'aperture={target_m * 1000.0:.1f} mm, '
            f'burst_duration_s={effective_duration_s:.3f}')
        target_us = self._backend.command(requested_us)
        self.get_logger().warning(
            f'PWM burst completed: requested_pulse={requested_us} us, '
            f'effective_target={target_us} us, '
            f'aperture={target_m * 1000.0:.1f} mm, '
            f'burst_duration_s={effective_duration_s:.3f}')
        expected_motion_s = (
            abs(target_us - prior_us) / self._slew_us_per_s
            if prior_us is not None and self._slew_us_per_s > 0 else 0.0
        )
        deadline = time.monotonic() + max(
            self._command_timeout_s,
            expected_motion_s + self._settle_time_s + 1.0,
        )
        reached_at = None
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._backend.freeze()
                goal_handle.canceled()
                with self._state_lock:
                    self._last_result = 'canceled_pwm_frozen'
                return self._make_result(False)
            if self._backend.error:
                raise RuntimeError(self._backend.error)
            current_us = self._backend.current_us
            if current_us is not None:
                # There is no position feedback. The actual FTDI clock may
                # quantize the pulse a few microseconds away from the nominal
                # 820/1300 us calibration, so publish the commanded two-state
                # aperture rather than treating clock quantization as motion.
                self._set_position(target_m)
            goal_handle.publish_feedback(self._make_feedback(False))
            if (current_us == target_us
                    and self._backend.actual_rate_hz is not None):
                if reached_at is None:
                    reached_at = time.monotonic()
                elif time.monotonic() - reached_at >= self._settle_time_s:
                    break
            else:
                reached_at = None
            time.sleep(0.05)
        else:
            raise TimeoutError('PWM command did not settle before timeout')

        self._set_position(target_m)
        if self._assume_reached:
            goal_handle.succeed()
            with self._state_lock:
                self._last_result = 'commanded_position_estimated'
            return self._make_result(True)
        goal_handle.abort()
        with self._state_lock:
            self._last_result = 'unconfirmed_no_feedback'
        return self._make_result(False)

    def _set_position(self, position_m):
        with self._state_lock:
            self._estimated_position_m = float(position_m)
            self._position_source = 'commanded_open_loop_estimate'

    def _make_feedback(self, reached):
        feedback = GripperCommand.Feedback()
        with self._state_lock:
            feedback.position = (
                float(self._estimated_position_m)
                if self._estimated_position_m is not None else math.nan)
        feedback.effort = math.nan
        feedback.stalled = False
        feedback.reached_goal = bool(reached)
        return feedback

    def _make_result(self, reached):
        result = GripperCommand.Result()
        with self._state_lock:
            result.position = (
                float(self._estimated_position_m)
                if self._estimated_position_m is not None else math.nan)
        result.effort = math.nan
        result.stalled = False
        result.reached_goal = bool(reached)
        return result

    def _publish_state(self):
        with self._state_lock:
            position = self._estimated_position_m
            last_target = self._last_target_m
            last_result = self._last_result
            busy = self._goal_reserved
            position_source = self._position_source
        stamp = self.get_clock().now().to_msg()
        if position is not None:
            joint_state = JointState()
            joint_state.header.stamp = stamp
            joint_state.name = [self._joint_name]
            joint_state.position = [float(position)]
            self._joint_publisher.publish(joint_state)

        status = DiagnosticStatus()
        status.name = 'ydi_j22_gripper/open_loop_driver'
        status.hardware_id = 'FT232H/YDI-J22' if self._backend_name == 'ft232h' else 'mock'
        backend_error = self._backend.error if self._backend is not None else None
        if backend_error:
            status.level = DiagnosticStatus.ERROR
            status.message = backend_error
        elif self._backend_name == 'ft232h' and not self._hardware_gate_open:
            status.level = DiagnosticStatus.WARN
            status.message = 'hardware output disarmed'
        elif self._backend_name == 'ft232h':
            status.level = DiagnosticStatus.WARN
            status.message = 'open-loop output; position is estimated'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'mock backend'
        values = {
            'backend': self._backend_name,
            'armed': str(self._hardware_gate_open).lower(),
            'busy': str(busy).lower(),
            'feedback_mode': 'open_loop_estimated',
            'position_source': position_source,
            'aperture_calibrated': str(
                bool(self.get_parameter('aperture_calibrated').value)).lower(),
            'last_result': last_result,
            'estimated_position_m': 'unknown' if position is None else f'{position:.6f}',
            'last_target_m': 'none' if last_target is None else f'{last_target:.6f}',
            'current_pulse_us': (
                'none' if self._backend is None or self._backend.current_us is None
                else str(self._backend.current_us)),
            'actual_sample_rate_hz': (
                'none' if self._backend is None or self._backend.actual_rate_hz is None
                else f'{self._backend.actual_rate_hz:.1f}'),
            'frames_written': (
                '0' if self._backend is None
                else str(self._backend.frames_written)),
            'bytes_written': (
                '0' if self._backend is None
                else str(self._backend.bytes_written)),
            'usb_recovery_count': (
                '0' if self._backend is None
                else str(self._backend.recovery_count)),
        }
        status.values = [KeyValue(key=key, value=value)
                         for key, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = [status]
        self._diagnostic_publisher.publish(array)

    def destroy_node(self):
        if self._backend is not None:
            self._backend.close()
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
