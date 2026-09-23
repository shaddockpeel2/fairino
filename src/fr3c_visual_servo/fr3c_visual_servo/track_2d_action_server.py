"""ROS 2 TrackTarget2D action server for dry-run or leased MoveIt Servo."""

from dataclasses import asdict, replace
import json
import math
import threading
import time
import uuid

from fairino_msgs.action import TrackTarget2D
from fairino_msgs.srv import ControlLease
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

from fr3c_visual_servo.control import (
    ControllerConfig,
    Observation,
    SUPPORTED_COLORS,
    Termination,
    TrackingController,
)


class Track2DActionServer(Node):
    """Execute one bounded dry-run or leased Servo tracking goal at a time."""

    def __init__(self):
        super().__init__('track_2d_action_server')
        self._declare_parameters()
        self._config = self._read_controller_config()
        self._execute_enabled = bool(self.get_parameter('execute').value)
        self._perception_color = str(
            self.get_parameter('perception_target_color').value
        ).strip().lower()
        if self._perception_color not in SUPPORTED_COLORS:
            raise ValueError('invalid_perception_target_color')
        self._dry_run_command_frame = str(
            self.get_parameter('dry_run_command_frame').value)
        self._camera_frame = str(
            self.get_parameter('camera_command_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tcp_origin = None
        self._measured_xy_m = 0.0
        self._tf_timeout_s = float(self.get_parameter('tf_timeout_s').value)
        self._heartbeat_period_s = float(
            self.get_parameter('heartbeat_period_s').value)
        self._heartbeat_timeout_s = float(
            self.get_parameter('heartbeat_timeout_s').value)
        self._stop_publish_cycles = int(
            self.get_parameter('stop_publish_cycles').value)
        if self._stop_publish_cycles < 1:
            raise ValueError('stop_publish_cycles_must_be_positive')
        if (self._tf_timeout_s <= 0.0 or self._heartbeat_period_s <= 0.0
                or self._heartbeat_timeout_s <= self._heartbeat_period_s):
            raise ValueError('invalid_stage_three_timeout_parameters')

        callback_group = ReentrantCallbackGroup()
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._expected_twist_pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('expected_twist_topic').value), 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('dry_run_status_topic').value),
            state_qos)
        self._servo_twist_pub = None
        self._lease_client = None
        self._tf_buffer = None
        self._tf_listener = None
        self._servo_fault = None
        self._control_state = 'unknown'
        if self._execute_enabled:
            self._servo_twist_pub = self.create_publisher(
                TwistStamped,
                str(self.get_parameter('servo_twist_topic').value), 10)
            self._lease_client = self.create_client(
                ControlLease,
                str(self.get_parameter('control_lease_service').value),
                callback_group=callback_group)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(
                self._tf_buffer, self, spin_thread=False)
            self.create_subscription(
                Int8, str(self.get_parameter('servo_status_topic').value),
                self._on_servo_status, 10, callback_group=callback_group)
            self.create_subscription(
                String, str(self.get_parameter('control_state_topic').value),
                self._on_control_state, state_qos,
                callback_group=callback_group)
        self.create_subscription(
            String, str(self.get_parameter('observation_topic').value),
            self._on_observation, 10, callback_group=callback_group)

        self._lock = threading.Lock()
        self._goal_reserved = False
        self._latest_observation = None
        self._observation_sequence = 0
        self._last_source_key = None
        self._last_parse_error = None
        self._shutdown = threading.Event()
        self._action_server = ActionServer(
            self,
            TrackTarget2D,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        if self._execute_enabled:
            self.get_logger().warning(
                'TrackTarget2D stage-three Servo output is ENABLED; motion '
                'still requires an explicit accepted Action goal')
        else:
            self.get_logger().warning(
                'TrackTarget2D ready in DRY-RUN mode; no Servo command '
                'publisher was created')

    def _declare_parameters(self):
        defaults = {
            'execute': False,
            'action_name': '/fr3c_visual_servo/track_2d',
            'observation_topic': '/perception/color_target_2d/status',
            'expected_twist_topic':
                '/fr3c_visual_servo/dry_run/expected_twist',
            'dry_run_status_topic': '/fr3c_visual_servo/dry_run/status',
            'dry_run_command_frame': 'camera_color_optical_frame',
            'camera_command_frame': 'camera_color_optical_frame',
            'base_frame': 'base_link',
            'tf_timeout_s': 0.05,
            'tcp_frame': 'grasp_frame',
            'max_tf_age_s': 0.5,
            'max_z_drift_m': 0.005,
            'max_orientation_drift_rad': 0.05,
            'servo_twist_topic': '/servo_node/delta_twist_cmds',
            'servo_status_topic': '/servo_node/status',
            'control_state_topic': '/fairino/control_state',
            'control_lease_service': '/fairino/control_lease',
            'heartbeat_period_s': 0.10,
            'heartbeat_timeout_s': 0.50,
            'perception_target_color': 'red',
            'control_rate_hz': 50.0,
            'gain_x_mps': 0.020,
            'gain_y_mps': 0.020,
            'axis_sign_x': 1.0,
            'axis_sign_y': 1.0,
            'max_speed_mps': 0.005,
            'max_acceleration_mps2': 0.030,
            'initial_center_half_width_px': 12.0,
            'initial_center_half_height_px': 9.0,
            'enter_half_width_px': 40.0,
            'enter_half_height_px': 30.0,
            'exit_half_width_px': 55.0,
            'exit_half_height_px': 45.0,
            'enter_frames': 3,
            'exit_frames': 2,
            'observation_stale_s': 0.40,
            'dropout_grace_s': 0.16,
            'lost_timeout_s': 0.0,
            'acquisition_timeout_s': 5.0,
            'stop_publish_cycles': 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_controller_config(self):
        def value(name):
            return self.get_parameter(name).value

        config = ControllerConfig(
            control_rate_hz=float(value('control_rate_hz')),
            gain_x_mps=float(value('gain_x_mps')),
            gain_y_mps=float(value('gain_y_mps')),
            axis_sign_x=float(value('axis_sign_x')),
            axis_sign_y=float(value('axis_sign_y')),
            max_speed_mps=float(value('max_speed_mps')),
            max_acceleration_mps2=float(value('max_acceleration_mps2')),
            initial_center_half_width_px=float(
                value('initial_center_half_width_px')),
            initial_center_half_height_px=float(
                value('initial_center_half_height_px')),
            enter_half_width_px=float(value('enter_half_width_px')),
            enter_half_height_px=float(value('enter_half_height_px')),
            exit_half_width_px=float(value('exit_half_width_px')),
            exit_half_height_px=float(value('exit_half_height_px')),
            enter_frames=int(value('enter_frames')),
            exit_frames=int(value('exit_frames')),
            observation_stale_s=float(value('observation_stale_s')),
            dropout_grace_s=float(value('dropout_grace_s')),
            lost_timeout_s=float(value('lost_timeout_s')),
            acquisition_timeout_s=float(value('acquisition_timeout_s')),
        )
        config.validate()
        return config

    def _goal_callback(self, goal_request):
        color = goal_request.color.strip().lower()
        duration = float(goal_request.max_duration_s)
        if color not in SUPPORTED_COLORS:
            self.get_logger().error(f'Rejecting unsupported color: {color!r}')
            return GoalResponse.REJECT
        if color != self._perception_color:
            self.get_logger().error(
                f'Rejecting {color} goal: perception is configured for '
                f'{self._perception_color}')
            return GoalResponse.REJECT
        if not math.isfinite(duration) or duration < 0.0:
            self.get_logger().error('Rejecting invalid max_duration_s')
            return GoalResponse.REJECT
        with self._lock:
            if (self._execute_enabled
                    and self._control_state in (
                        'acquiring_automatic_control',
                        'executing_automatic',
                        'releasing_automatic_control',
                        'acquiring_servo_tracking_control',
                        'executing_servo_tracking',
                        'releasing_servo_tracking_control',
                        'fault_control_not_released')):
                self.get_logger().warning(
                    'Rejecting tracking goal: arm control is busy or faulted '
                    f'({self._control_state})')
                return GoalResponse.REJECT
            if self._goal_reserved:
                self.get_logger().warning('Rejecting concurrent tracking goal')
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _on_observation(self, message):
        received_s = time.monotonic()
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('status_is_not_an_object')
            color = str(payload.get('target_color', '')).strip().lower()
            valid = payload.get('valid') is True
            reason = str(payload.get('reason', 'missing_reason'))
            source_stamp = self._finite_number(
                payload.get('source_stamp_s'), None)
            source_key = (color, source_stamp)
            target = self._pair(payload.get('target_center_px'))
            center = self._pair(payload.get('center_reference_px'))
            width = self._finite_number(payload.get('image_width'), 0.0)
            height = self._finite_number(payload.get('image_height'), 0.0)
            age_default = math.inf if valid else 0.0
            age_s = self._finite_number(
                payload.get('observation_age_s'), age_default)
            if age_s < 0.0:
                age_s = 0.0
            with self._lock:
                previous = self._latest_observation
                repeated_source = (
                    source_stamp is not None
                    and source_key == self._last_source_key
                    and previous is not None)
                if not repeated_source:
                    self._observation_sequence += 1
                    self._last_source_key = source_key
                sequence = self._observation_sequence
                # Replaying one message must not refresh that image forever.
                # Preserve its first receipt origin so monotonic age grows.
                if repeated_source:
                    received_s = previous.received_monotonic_s
                    age_s = previous.age_at_receive_s
                observation = Observation(
                    sample_id=sequence,
                    color=color,
                    valid=valid,
                    reason=reason,
                    target_u_px=target[0],
                    target_v_px=target[1],
                    center_u_px=center[0],
                    center_v_px=center[1],
                    image_width_px=width,
                    image_height_px=height,
                    age_at_receive_s=age_s,
                    received_monotonic_s=received_s,
                )
                self._latest_observation = observation
            self._last_parse_error = None
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            signature = f'{type(error).__name__}:{error}'
            if signature != self._last_parse_error:
                self.get_logger().error(
                    f'Ignoring malformed perception status: {signature}')
                self._last_parse_error = signature

    def _on_servo_status(self, message):
        status = int(message.data)
        # Servo itself halts at singularities and joint bounds (2, 5), then
        # retries the current command when the target moves back into reach.
        # A collision halt (4) still aborts the Action and releases control.
        if status == 4:
            with self._lock:
                self._servo_fault = status

    def _on_control_state(self, message):
        with self._lock:
            self._control_state = message.data.strip()

    @staticmethod
    def _finite_number(value, default):
        if value is None:
            return default
        number = float(value)
        return number if math.isfinite(number) else default

    @classmethod
    def _pair(cls, value):
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            return 0.0, 0.0
        return (cls._finite_number(value[0], 0.0),
                cls._finite_number(value[1], 0.0))

    async def _execute(self, goal_handle):
        started_s = time.monotonic()
        run_id = str(uuid.uuid4())
        color = goal_handle.request.color.strip().lower()
        controller = TrackingController(
            color, self._config, started_s,
            float(goal_handle.request.max_duration_s))
        mode = 'SERVO' if self._execute_enabled else 'DRY-RUN'
        self.get_logger().info(
            f'{mode} {run_id} started for {color}; parameters='
            f'{json.dumps(asdict(self._config), sort_keys=True)}')
        self._publish_run_status(
            run_id, controller, None, 'goal_started', started_s)
        final_output = None
        lease_id = ''
        self._tcp_origin = None
        self._measured_xy_m = 0.0
        last_heartbeat_s = 0.0
        period_s = 1.0 / self._config.control_rate_hz
        with self._lock:
            self._servo_fault = None
        try:
            while rclpy.ok() and not self._shutdown.is_set():
                now_s = time.monotonic()
                with self._lock:
                    servo_fault = self._servo_fault
                if lease_id and servo_fault is not None:
                    final_output = controller.stop(
                        Termination.SERVO_FAULT,
                        f'moveit_servo_hard_fault:{servo_fault}')
                elif goal_handle.is_cancel_requested:
                    final_output = controller.stop(
                        Termination.CANCELED, 'cancel_requested')
                else:
                    with self._lock:
                        observation = self._latest_observation
                    final_output = controller.update(observation, now_s)

                if (self._execute_enabled and not lease_id
                        and not final_output.done
                        and final_output.phase.value != 'ACQUIRING'):
                    self._check_tcp_bounds()
                    lease = await self._lease_operation(
                        ControlLease.Request.ACQUIRE)
                    if not lease.success:
                        termination = (
                            Termination.BUSY if lease.message.startswith(
                                'busy:') else Termination.CONTROL_UNAVAILABLE)
                        final_output = controller.stop(
                            termination,
                            f'control_acquisition_failed:{lease.message}')
                    else:
                        lease_id = lease.lease_id
                        last_heartbeat_s = time.monotonic()
                        # Acquisition can take longer than an image lifetime.
                        # Recompute on the next iteration before commanding.
                        continue

                if (lease_id and not final_output.done
                        and now_s - last_heartbeat_s
                        >= self._heartbeat_period_s):
                    heartbeat = await self._lease_operation(
                        ControlLease.Request.HEARTBEAT, lease_id)
                    last_heartbeat_s = time.monotonic()
                    if not heartbeat.success:
                        final_output = controller.stop(
                            Termination.CONTROL_UNAVAILABLE,
                            f'control_heartbeat_failed:{heartbeat.message}')

                if lease_id and not final_output.done:
                    try:
                        self._check_tcp_bounds()
                    except (TransformException, ValueError) as error:
                        final_output = controller.stop(
                            Termination.SAFETY_LIMIT, str(error))
                    # Heartbeat service waits can also age the observation.
                    with self._lock:
                        latest = self._latest_observation
                    if (not final_output.done and (latest is None or
                            not latest.valid or latest.age_s(time.monotonic())
                            > self._config.observation_stale_s)):
                        final_output = controller.update(latest, time.monotonic())

                command_output = final_output
                if lease_id and not final_output.done:
                    try:
                        command_output = self._transform_output_to_base(
                            final_output)
                    except TransformException as error:
                        final_output = controller.stop(
                            Termination.CONTROL_UNAVAILABLE,
                            f'command_transform_unavailable:{error}')
                        command_output = final_output
                self._publish_expected_twist(command_output)
                if lease_id:
                    self._publish_servo_twist(command_output)
                self._publish_feedback(goal_handle, command_output)
                self._publish_run_status(
                    run_id, controller, command_output,
                    final_output.detail, started_s)
                if final_output.done:
                    break
                # rclpy drives Action callbacks with its own executor task,
                # not an asyncio event loop. A short sleep is safe here: the
                # MultiThreadedExecutor has other threads for observations,
                # lease replies and cancellation.
                time.sleep(period_s)
            if final_output is None or not final_output.done:
                final_output = controller.stop(
                    Termination.INTERNAL_ERROR, 'node_shutdown')
        except Exception as error:  # pragma: no cover - ROS containment path
            self.get_logger().error(
                f'{mode} {run_id} failed: {type(error).__name__}: {error}')
            final_output = controller.stop(
                Termination.INTERNAL_ERROR,
                f'exception:{type(error).__name__}:{error}')
        finally:
            try:
                await self._publish_stop_sequence(
                    goal_handle, final_output, publish_servo=bool(lease_id))
            except Exception as error:
                self.get_logger().error(
                    f'Stop publication failed: {type(error).__name__}: '
                    f'{error}')
                final_output = controller.stop(
                    Termination.INTERNAL_ERROR,
                    f'stop_publication_failed:{type(error).__name__}:{error}')
            if lease_id:
                release = await self._lease_operation(
                    ControlLease.Request.RELEASE, lease_id)
                if not release.success:
                    self.get_logger().error(
                        f'Control lease release failed: {release.message}')
                    if (final_output is None
                            or final_output.termination in (
                                Termination.OK, Termination.CANCELED)):
                        final_output = controller.stop(
                            Termination.CONTROL_UNAVAILABLE,
                            f'control_release_failed:{release.message}')
            with self._lock:
                self._goal_reserved = False

        result = self._make_result(final_output, started_s)
        if final_output.termination == Termination.OK:
            goal_handle.succeed()
        elif final_output.termination == Termination.CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().info(
            f'{mode} {run_id} stopped: {final_output.termination.value}; '
            f'{final_output.detail}')
        return result

    async def _lease_operation(self, operation, lease_id=''):
        if not self._lease_client.wait_for_service(timeout_sec=2.0):
            response = ControlLease.Response()
            response.success = False
            response.message = 'control_lease_service_unavailable'
            response.owner_state = 'UNKNOWN'
            return response
        request = ControlLease.Request()
        request.operation = operation
        request.owner = 'SERVO_TRACKING'
        request.lease_id = lease_id
        request.heartbeat_timeout_s = float(self._heartbeat_timeout_s)
        try:
            return await self._lease_client.call_async(request)
        except Exception as error:  # pragma: no cover
            response = ControlLease.Response()
            response.success = False
            response.message = (
                f'control_lease_exception:{type(error).__name__}:{error}')
            response.owner_state = 'UNKNOWN'
            return response

    async def _publish_stop_sequence(self, goal_handle, output,
                                     publish_servo=False):
        if output is None:
            return
        for _ in range(self._stop_publish_cycles):
            self._publish_expected_twist(output, force_zero=True)
            if publish_servo:
                self._publish_servo_twist(output, force_zero=True)
            self._publish_feedback(goal_handle, output)
            time.sleep(1.0 / self._config.control_rate_hz)

    def _publish_expected_twist(self, output, force_zero=False):
        frame = (self._base_frame if self._execute_enabled
                 else self._dry_run_command_frame)
        self._publish_twist(
            self._expected_twist_pub, output, frame, force_zero)

    def _publish_servo_twist(self, output, force_zero=False):
        self._publish_twist(
            self._servo_twist_pub, output, self._base_frame, force_zero)

    def _publish_twist(self, publisher, output, frame, force_zero=False):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame
        if not force_zero:
            message.twist.linear.x = float(output.command_x_mps)
            message.twist.linear.y = float(output.command_y_mps)
        publisher.publish(message)

    @staticmethod
    def _rotate_vector_by_quaternion(vector, quaternion):
        x, y, z = vector
        qx, qy, qz, qw = quaternion
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12 or not math.isfinite(norm):
            raise TransformException('invalid_transform_quaternion')
        qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    def _check_transform_age(self, transform):
        stamp = transform.header.stamp
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            stamp.sec + stamp.nanosec * 1e-9)
        if not -0.05 <= age <= float(self.get_parameter('max_tf_age_s').value):
            raise TransformException('robot_transform_stale')

    def _check_tcp_bounds(self):
        transform = self._tf_buffer.lookup_transform(
            self._base_frame, str(self.get_parameter('tcp_frame').value),
            Time(), timeout=Duration(seconds=self._tf_timeout_s))
        self._check_transform_age(transform)
        p, q = transform.transform.translation, transform.transform.rotation
        pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('invalid_tcp_pose')
        if self._tcp_origin is None:
            self._tcp_origin = pose
        origin = self._tcp_origin
        self._measured_xy_m = math.hypot(p.x - origin[0], p.y - origin[1])
        if abs(p.z - origin[2]) > float(self.get_parameter('max_z_drift_m').value):
            raise ValueError('tcp_height_drift_limit_reached')
        norm = math.sqrt(sum(v*v for v in pose[3:]) *
                         sum(v*v for v in origin[3:]))
        if norm < 1e-12:
            raise ValueError('invalid_tcp_orientation')
        dot = abs(sum(a*b for a, b in zip(pose[3:], origin[3:])) / norm)
        angle = 2 * math.acos(min(1.0, dot))
        if angle > float(self.get_parameter('max_orientation_drift_rad').value):
            raise ValueError('tcp_orientation_drift_limit_reached')

    def _transform_output_to_base(self, output):
        if (output.command_x_mps == 0.0 and output.command_y_mps == 0.0):
            return output
        transform = self._tf_buffer.lookup_transform(
            self._base_frame, self._camera_frame, Time(),
            timeout=Duration(seconds=self._tf_timeout_s))
        self._check_transform_age(transform)
        rotation = transform.transform.rotation
        base_vector = self._rotate_vector_by_quaternion(
            (output.command_x_mps, output.command_y_mps, 0.0),
            (rotation.x, rotation.y, rotation.z, rotation.w))
        # Stage three deliberately commands only table-plane translation.
        return replace(
            output,
            command_x_mps=float(base_vector[0]),
            command_y_mps=float(base_vector[1]))

    @staticmethod
    def _publish_feedback(goal_handle, output):
        feedback = TrackTarget2D.Feedback()
        feedback.phase = output.phase.value
        feedback.detail = output.detail
        feedback.target_u_px = float(output.target_u_px)
        feedback.target_v_px = float(output.target_v_px)
        feedback.error_x_px = float(output.error_x_px)
        feedback.error_y_px = float(output.error_y_px)
        feedback.command_x_mps = float(output.command_x_mps)
        feedback.command_y_mps = float(output.command_y_mps)
        feedback.observation_age_s = float(output.observation_age_s)
        feedback.centered = bool(output.centered)
        goal_handle.publish_feedback(feedback)

    def _publish_run_status(self, run_id, controller, output, detail,
                            started_s):
        command = [0.0, 0.0] if output is None else [
            output.command_x_mps, output.command_y_mps]
        status = {
            'schema_version': 1,
            'run_id': run_id,
            'execute': self._execute_enabled,
            'measured_relative_xy_m': self._measured_xy_m,
            'control_state': self._control_state,
            'color': controller.color,
            'phase': controller.phase.value,
            'detail': detail,
            'elapsed_s': max(0.0, time.monotonic() - started_s),
            'expected_command_xy_mps': command,
            'estimated_displacement_xy_m':
                list(controller.estimated_displacement_xy_m),
            'parameter_snapshot': asdict(self._config),
        }
        self._status_pub.publish(String(
            data=json.dumps(status, sort_keys=True, separators=(',', ':'))))

    @staticmethod
    def _result_code(termination):
        codes = {
            Termination.OK: TrackTarget2D.Result.OK,
            Termination.BUSY: TrackTarget2D.Result.BUSY,
            Termination.TARGET_NOT_FOUND:
                TrackTarget2D.Result.TARGET_NOT_FOUND,
            Termination.TARGET_NOT_UNIQUE:
                TrackTarget2D.Result.TARGET_NOT_UNIQUE,
            Termination.TARGET_STALE: TrackTarget2D.Result.TARGET_STALE,
            Termination.CONTROL_UNAVAILABLE:
                TrackTarget2D.Result.CONTROL_UNAVAILABLE,
            Termination.SERVO_FAULT: TrackTarget2D.Result.SERVO_FAULT,
            Termination.SAFETY_LIMIT: TrackTarget2D.Result.SAFETY_LIMIT,
            Termination.CANCELED: TrackTarget2D.Result.CANCELED,
            Termination.INTERNAL_ERROR: TrackTarget2D.Result.INTERNAL_ERROR,
        }
        return codes[termination]

    def _make_result(self, output, started_s):
        result = TrackTarget2D.Result()
        result.success = output.termination == Termination.OK
        result.error_code = self._result_code(output.termination)
        result.message = output.detail
        result.final_error_x_px = float(output.error_x_px)
        result.final_error_y_px = float(output.error_y_px)
        result.elapsed_s = float(max(0.0, time.monotonic() - started_s))
        return result

    def destroy_node(self):
        self._shutdown.set()
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = Track2DActionServer()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
