#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace fairino_hardware
{
namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("FairinoHardwareInterface");

constexpr std::array<std::string_view, kStatusInterfaceCount> kStatusInterfaceNames{
  "program_state", "robot_state", "main_error_code", "sub_error_code", "robot_mode",
  "tool_id", "work_object_id", "emergency_stop", "motion_done", "collision_state",
  "safety_stop_0", "safety_stop_1", "enabled", "payload_kg", "payload_cog_x_mm",
  "payload_cog_y_mm", "payload_cog_z_mm", "tool_x_mm", "tool_y_mm", "tool_z_mm",
  "tool_rx_deg", "tool_ry_deg", "tool_rz_deg", "wobj_x_mm", "wobj_y_mm", "wobj_z_mm",
  "wobj_rx_deg", "wobj_ry_deg", "wobj_rz_deg", "flange_x_mm", "flange_y_mm",
  "flange_z_mm", "flange_rx_deg", "flange_ry_deg", "flange_rz_deg",
  "read_position_call_ms", "read_status_call_ms", "servo_j_call_ms",
  "servo_j_interval_ms", "control_period_ms"};

constexpr std::size_t kReadPositionCallMsIndex = 35;
constexpr std::size_t kReadStatusCallMsIndex = 36;
constexpr std::size_t kServoJCallMsIndex = 37;
constexpr std::size_t kServoJIntervalMsIndex = 38;
constexpr std::size_t kControlPeriodMsIndex = 39;

double milliseconds_since(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

double parse_positive_double(
  const std::unordered_map<std::string, std::string> & parameters, const char * name,
  double default_value)
{
  const auto parameter = parameters.find(name);
  if (parameter == parameters.end()) {
    return default_value;
  }

  std::size_t parsed_length = 0;
  const double value = std::stod(parameter->second, &parsed_length);
  if (
    parsed_length != parameter->second.size() || !std::isfinite(value) ||
    value <= 0.0)
  {
    throw std::invalid_argument(std::string{name} + " must be a finite positive number");
  }
  return value;
}

class FairinoRobotClient final : public RobotClient
{
public:
  errno_t rpc(const char * ip) override {return robot_.RPC(ip);}
  errno_t close_rpc() override {return robot_.CloseRPC();}
  errno_t get_actual_joint_position_degree(uint8_t flag, JointPos * position) override
  {
    return robot_.GetActualJointPosDegree(flag, position);
  }
  errno_t get_robot_realtime_state(ROBOT_STATE_PKG * state) override
  {
    return robot_.GetRobotRealTimeState(state);
  }
  errno_t is_in_drag_teach(uint8_t * state) override {return robot_.IsInDragTeach(state);}
  errno_t servo_move_start() override {return robot_.ServoMoveStart();}
  errno_t servo_move_end() override {return robot_.ServoMoveEnd();}
  errno_t stop_motion() override {return robot_.StopMotion();}
  errno_t servo_j(
    JointPos * position, ExaxisPos * external_axes, float command_period_sec) override
  {
    return robot_.ServoJ(position, external_axes, 0, 0, command_period_sec, 0, 0);
  }

private:
  FRRobot robot_;
};

}  // namespace

namespace detail
{

bool all_commands_finite(const std::array<double, kJointCount> & commands)
{
  return std::all_of(
    commands.cbegin(), commands.cend(), [](double command) {
      return std::isfinite(command);
    });
}

}  // namespace detail

FairinoHardwareInterface::FairinoHardwareInterface(std::unique_ptr<RobotClient> robot_client)
: robot_client_(std::move(robot_client))
{
}

std::unique_ptr<RobotClient> FairinoHardwareInterface::make_robot_client()
{
  return std::make_unique<FairinoRobotClient>();
}

bool FairinoHardwareInterface::load_hardware_parameters()
{
  try {
    const auto controller_ip = info_.hardware_parameters.find("controller_ip");
    if (controller_ip != info_.hardware_parameters.end()) {
      if (controller_ip->second.empty()) {
        throw std::invalid_argument("controller_ip must not be empty");
      }
      controller_ip_ = controller_ip->second;
    }
    servo_command_period_sec_ = parse_positive_double(
      info_.hardware_parameters, "servo_command_period_sec", kDefaultServoCommandPeriodSec);
    control_period_tolerance_sec_ = parse_positive_double(
      info_.hardware_parameters, "control_period_tolerance_sec", 0.002);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(kLogger, "Invalid hardware parameter: %s", error.what());
    return false;
  }
  return true;
}

bool FairinoHardwareInterface::load_status_interfaces()
{
  status_interfaces_enabled_ = false;
  if (info_.gpios.empty()) {
    RCLCPP_WARN(
      kLogger,
      "No fairino_status GPIO is configured; P6 controller-state preflight will be unavailable");
    return true;
  }
  if (info_.gpios.size() != 1U || info_.gpios.front().name != "fairino_status") {
    RCLCPP_FATAL(kLogger, "Expected exactly one GPIO named 'fairino_status'");
    return false;
  }
  const auto & interfaces = info_.gpios.front().state_interfaces;
  if (interfaces.size() != kStatusInterfaceNames.size()) {
    RCLCPP_FATAL(
      kLogger, "fairino_status has %zu state interfaces; expected %zu", interfaces.size(),
      kStatusInterfaceNames.size());
    return false;
  }
  for (std::size_t index = 0; index < interfaces.size(); ++index) {
    if (interfaces[index].name != kStatusInterfaceNames[index]) {
      RCLCPP_FATAL(
        kLogger, "fairino_status interface %zu is '%s'; expected '%s'", index,
        interfaces[index].name.c_str(), kStatusInterfaceNames[index].data());
      return false;
    }
  }
  status_interfaces_enabled_ = true;
  return true;
}

void FairinoHardwareInterface::update_status_state(const ROBOT_STATE_PKG & state)
{
  const std::array<double, 5> timing_metrics{
    status_state_[kReadPositionCallMsIndex], status_state_[kReadStatusCallMsIndex],
    status_state_[kServoJCallMsIndex], status_state_[kServoJIntervalMsIndex],
    status_state_[kControlPeriodMsIndex]};
  status_state_ = {
    static_cast<double>(state.program_state), static_cast<double>(state.robot_state),
    static_cast<double>(state.main_code), static_cast<double>(state.sub_code),
    static_cast<double>(state.robot_mode), static_cast<double>(state.tool),
    static_cast<double>(state.user), static_cast<double>(state.EmergencyStop),
    static_cast<double>(state.motion_done), static_cast<double>(state.collisionState),
    static_cast<double>(state.safety_stop0_state), static_cast<double>(state.safety_stop1_state),
    static_cast<double>(state.rbtEnableState), state.load,
    state.loadCog[0], state.loadCog[1], state.loadCog[2],
    state.toolCoord[0], state.toolCoord[1], state.toolCoord[2],
    state.toolCoord[3], state.toolCoord[4], state.toolCoord[5],
    state.wobjCoord[0], state.wobjCoord[1], state.wobjCoord[2],
    state.wobjCoord[3], state.wobjCoord[4], state.wobjCoord[5],
    state.flange_cur_pos[0], state.flange_cur_pos[1], state.flange_cur_pos[2],
    state.flange_cur_pos[3], state.flange_cur_pos[4], state.flange_cur_pos[5],
    timing_metrics[0], timing_metrics[1], timing_metrics[2], timing_metrics[3], timing_metrics[4]};
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & system_info)
{
  if (
    hardware_interface::SystemInterface::on_init(system_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount) {
    RCLCPP_FATAL(
      kLogger, "Expected exactly %zu joints, but received %zu", kJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!load_hardware_parameters() || !load_status_interfaces()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  velocity_state_enabled_ = info_.joints.front().state_interfaces.size() == 2U;
  if (velocity_state_enabled_ && !status_interfaces_enabled_) {
    RCLCPP_FATAL(kLogger, "Velocity state requires FAIRINO realtime status feedback");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        kLogger, "Joint '%s' has %zu command interfaces; exactly 1 is required",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        kLogger, "Joint '%s' command interface is '%s'; '%s' is required",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != (velocity_state_enabled_ ? 2U : 1U)) {
      RCLCPP_FATAL(
        kLogger, "Joint '%s' has %zu state interfaces; inconsistent velocity state configuration",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        kLogger, "Joint '%s' state interface is '%s'; '%s' is required", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (velocity_state_enabled_ &&
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        kLogger, "Joint '%s' second state interface must be velocity",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(
    kJointCount * (velocity_state_enabled_ ? 2U : 1U) +
    (status_interfaces_enabled_ ? kStatusInterfaceCount : 0U));
  for (std::size_t index = 0; index < kJointCount; ++index) {
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION,
      &joint_position_state_[index]);
    if (velocity_state_enabled_) {
      state_interfaces.emplace_back(
        info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_state_[index]);
    }
  }
  if (status_interfaces_enabled_) {
    for (std::size_t index = 0; index < kStatusInterfaceNames.size(); ++index) {
      state_interfaces.emplace_back(
        "fairino_status", std::string{kStatusInterfaceNames[index]}, &status_state_[index]);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kJointCount);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    command_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION,
      &joint_position_command_[index]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  using namespace std::chrono_literals;

  RCLCPP_INFO(kLogger, "Connecting to FAIRINO controller at %s", controller_ip_.c_str());
  if (!robot_client_) {
    robot_client_ = make_robot_client();
  }

  write_failed_ = false;
  period_reported_ = false;
  position_command_active_ = false;
  have_last_servo_j_call_start_ = false;
  const errno_t rpc_result = robot_client_->rpc(controller_ip_.c_str());
  if (rpc_result != 0) {
    RCLCPP_ERROR(kLogger, "FAIRINO RPC connection failed, SDK error code: %d", rpc_result);
    robot_client_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  rpc_connected_ = true;
  rclcpp::sleep_for(200ms);

  JointPos current_position{};
  const errno_t read_result =
    robot_client_->get_actual_joint_position_degree(0, &current_position);
  if (read_result != 0) {
    RCLCPP_ERROR(
      kLogger, "Cannot read initial joint positions, SDK error code: %d", read_result);
    stop_and_disconnect("activation read failure");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double position_rad = current_position.jPos[index] / 180.0 * M_PI;
    joint_position_state_[index] = position_rad;
    joint_position_command_[index] = position_rad;
  }

  if (status_interfaces_enabled_) {
    ROBOT_STATE_PKG controller_state{};
    const errno_t status_result = robot_client_->get_robot_realtime_state(&controller_state);
    if (status_result != 0) {
      RCLCPP_ERROR(
        kLogger, "Cannot read initial FAIRINO status, SDK error code: %d", status_result);
      stop_and_disconnect("activation status read failure");
      return hardware_interface::CallbackReturn::ERROR;
    }
    update_status_state(controller_state);
    if (velocity_state_enabled_) {
      for (std::size_t index = 0; index < kJointCount; ++index) {
        const double velocity = controller_state.actual_qd[index] * M_PI / 180.0;
        if (!std::isfinite(velocity)) {
          RCLCPP_ERROR(kLogger, "Non-finite FAIRINO joint velocity at index %zu", index);
          write_failed_ = true;
          stop_and_disconnect("invalid joint velocity feedback");
          return hardware_interface::CallbackReturn::ERROR;
        }
        joint_velocity_state_[index] = velocity;
      }
    }
  }

  RCLCPP_INFO(
    kLogger,
    "Hardware active in monitoring mode with cmdT=%.6f s; ServoJ is inhibited until a "
    "trajectory controller claims all six arm position interfaces",
    servo_command_period_sec_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::stop_and_disconnect(
  const char * reason)
{
  bool succeeded = true;
  if (!robot_client_) {
    rpc_connected_ = false;
    servo_mode_active_ = false;
    position_command_active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  position_command_active_ = false;
  if (servo_mode_active_) {
    const errno_t result = robot_client_->servo_move_end();
    if (result != 0) {
      RCLCPP_ERROR(
        kLogger, "ServoMoveEnd failed during %s, SDK error code: %d", reason, result);
      succeeded = false;
    }
    servo_mode_active_ = false;
  }
  if (rpc_connected_) {
    const errno_t stop_result = robot_client_->stop_motion();
    if (stop_result != 0) {
      RCLCPP_ERROR(
        kLogger, "StopMotion failed during %s, SDK error code: %d", reason, stop_result);
      succeeded = false;
    }
    const errno_t close_result = robot_client_->close_rpc();
    if (close_result != 0) {
      RCLCPP_ERROR(
        kLogger, "CloseRPC failed during %s, SDK error code: %d", reason, close_result);
      succeeded = false;
    }
    rpc_connected_ = false;
  }
  robot_client_.reset();
  return succeeded ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(kLogger, "Deactivating FAIRINO hardware");
  return stop_and_disconnect("deactivate");
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  return stop_and_disconnect("shutdown");
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_ERROR(kLogger, "Hardware error latched; ending servo mode and disconnecting");
  return stop_and_disconnect("error handling");
}

hardware_interface::return_type FairinoHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::unordered_set<std::string> expected_interfaces;
  for (const auto & joint : info_.joints) {
    expected_interfaces.insert(joint.name + "/" + hardware_interface::HW_IF_POSITION);
  }

  const auto relevant_interfaces = [&expected_interfaces](
    const std::vector<std::string> & requested_interfaces)
    {
      std::unordered_set<std::string> relevant;
      for (const auto & interface : requested_interfaces) {
        if (expected_interfaces.count(interface) != 0U) {
          relevant.insert(interface);
        }
      }
      return relevant;
    };

  const auto starting = relevant_interfaces(start_interfaces);
  const auto stopping = relevant_interfaces(stop_interfaces);
  if (!starting.empty() && starting != expected_interfaces) {
    RCLCPP_ERROR(
      kLogger, "Rejecting partial arm-controller start: received %zu of %zu position interfaces",
      starting.size(), expected_interfaces.size());
    return hardware_interface::return_type::ERROR;
  }
  if (!stopping.empty() && stopping != expected_interfaces) {
    RCLCPP_ERROR(
      kLogger, "Rejecting partial arm-controller stop: received %zu of %zu position interfaces",
      stopping.size(), expected_interfaces.size());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto contains_arm_interface = [this](const std::vector<std::string> & interfaces) {
      for (const auto & joint : info_.joints) {
        const std::string name = joint.name + "/" + hardware_interface::HW_IF_POSITION;
        if (std::find(interfaces.begin(), interfaces.end(), name) != interfaces.end()) {
          return true;
        }
      }
      return false;
    };

  if (contains_arm_interface(stop_interfaces)) {
    position_command_active_ = false;
    have_last_servo_j_call_start_ = false;
    if (servo_mode_active_) {
      const errno_t end_result = robot_client_->servo_move_end();
      servo_mode_active_ = false;
      const errno_t stop_result = robot_client_->stop_motion();
      if (end_result != 0 || stop_result != 0) {
        // Inhibit all future writes/acquisitions after an unconfirmed stop.
        // A later successful switch must not silently clear this SDK failure.
        write_failed_ = true;
        RCLCPP_ERROR(
          kLogger, "Failed stopping arm controller: ServoMoveEnd=%d StopMotion=%d",
          end_result, stop_result);
        return hardware_interface::return_type::ERROR;
      }
    }
    RCLCPP_INFO(
      kLogger, "Arm trajectory controller stopped; monitoring-only mode restored");
  }

  if (contains_arm_interface(start_interfaces)) {
    if (!robot_client_ || !rpc_connected_ || write_failed_) {
      RCLCPP_ERROR(kLogger, "Cannot start arm controller while FAIRINO hardware is unavailable");
      return hardware_interface::return_type::ERROR;
    }

    uint8_t drag_state = 0;
    const errno_t drag_result = robot_client_->is_in_drag_teach(&drag_state);
    if (drag_result != 0) {
      RCLCPP_ERROR(
        kLogger, "Cannot query drag-teach state before automatic control, SDK error code: %d",
        drag_result);
      return hardware_interface::return_type::ERROR;
    }
    if (drag_state != 0) {
      RCLCPP_WARN(
        kLogger, "Rejecting automatic control while the flange drag button is active");
      return hardware_interface::return_type::ERROR;
    }

    JointPos current_position{};
    const errno_t read_result =
      robot_client_->get_actual_joint_position_degree(0, &current_position);
    if (read_result != 0) {
      RCLCPP_ERROR(
        kLogger, "Cannot synchronize arm-controller start, SDK error code: %d", read_result);
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double position_rad = current_position.jPos[index] / 180.0 * M_PI;
      joint_position_state_[index] = position_rad;
      joint_position_command_[index] = position_rad;
    }

    if (!servo_mode_active_) {
      const errno_t start_result = robot_client_->servo_move_start();
      if (start_result != 0) {
        RCLCPP_ERROR(
          kLogger, "ServoMoveStart failed while starting arm controller, SDK error code: %d",
          start_result);
        return hardware_interface::return_type::ERROR;
      }
      servo_mode_active_ = true;
    }
    period_reported_ = false;
    have_last_servo_j_call_start_ = false;
    position_command_active_ = true;
    RCLCPP_INFO(
      kLogger, "Arm trajectory controller acquired all six joints; ServoJ enabled");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!robot_client_ || !rpc_connected_ || write_failed_) {
    return hardware_interface::return_type::ERROR;
  }

  JointPos state_data{};
  const auto position_read_started = std::chrono::steady_clock::now();
  const errno_t return_code =
    robot_client_->get_actual_joint_position_degree(1, &state_data);
  status_state_[kReadPositionCallMsIndex] = milliseconds_since(position_read_started);
  if (return_code != 0) {
    RCLCPP_ERROR(kLogger, "Joint-state read failed, SDK error code: %d", return_code);
    write_failed_ = true;
    stop_and_disconnect("joint-state read failure");
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    joint_position_state_[index] = state_data.jPos[index] / 180.0 * M_PI;
  }
  if (status_interfaces_enabled_) {
    ROBOT_STATE_PKG controller_state{};
    const auto status_read_started = std::chrono::steady_clock::now();
    const errno_t status_result = robot_client_->get_robot_realtime_state(&controller_state);
    status_state_[kReadStatusCallMsIndex] = milliseconds_since(status_read_started);
    if (status_result != 0) {
      RCLCPP_ERROR(kLogger, "FAIRINO status read failed, SDK error code: %d", status_result);
      write_failed_ = true;
      stop_and_disconnect("controller status read failure");
      return hardware_interface::return_type::ERROR;
    }
    update_status_state(controller_state);
    if (velocity_state_enabled_) {
      for (std::size_t index = 0; index < kJointCount; ++index) {
        const double velocity = controller_state.actual_qd[index] * M_PI / 180.0;
        if (!std::isfinite(velocity)) {
          RCLCPP_ERROR(kLogger, "Non-finite FAIRINO joint velocity at index %zu", index);
          write_failed_ = true;
          stop_and_disconnect("invalid joint velocity feedback");
          return hardware_interface::return_type::ERROR;
        }
        joint_velocity_state_[index] = velocity;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & period)
{
  if (!robot_client_ || !rpc_connected_ || write_failed_) {
    return hardware_interface::return_type::ERROR;
  }
  // controller_manager keeps calling write() in monitoring-only mode.  A
  // successful no-op here preserves live joint states without occupying the
  // robot's motion channel, so the native flange drag button remains usable.
  if (!position_command_active_) {
    return hardware_interface::return_type::OK;
  }
  if (!servo_mode_active_) {
    RCLCPP_ERROR(kLogger, "Arm controller is active but FAIRINO servo mode is not running");
    return hardware_interface::return_type::ERROR;
  }

  if (!period_reported_) {
    const double actual_period_sec = period.seconds();
    if (
      !std::isfinite(actual_period_sec) || actual_period_sec <= 0.0 ||
      std::abs(actual_period_sec - servo_command_period_sec_) > control_period_tolerance_sec_)
    {
      RCLCPP_WARN(
        kLogger,
        "Measured control period %.6f s differs from ServoJ cmdT %.6f s by more than %.6f s",
        actual_period_sec, servo_command_period_sec_, control_period_tolerance_sec_);
    } else {
      RCLCPP_INFO(
        kLogger, "Measured control period %.6f s matches ServoJ cmdT %.6f s",
        actual_period_sec, servo_command_period_sec_);
    }
    period_reported_ = true;
  }
  status_state_[kControlPeriodMsIndex] = period.seconds() * 1000.0;

  if (control_mode_ == 0) {
    if (!detail::all_commands_finite(joint_position_command_)) {
      RCLCPP_ERROR(kLogger, "Rejected non-finite position command across the six-joint vector");
      write_failed_ = true;
      stop_and_disconnect("invalid position command");
      return hardware_interface::return_type::ERROR;
    }

    JointPos command{};
    ExaxisPos external_axes{0, 0, 0, 0};
    for (std::size_t index = 0; index < kJointCount; ++index) {
      command.jPos[index] = joint_position_command_[index] / M_PI * 180.0;
    }
    const auto servo_call_started = std::chrono::steady_clock::now();
    if (have_last_servo_j_call_start_) {
      status_state_[kServoJIntervalMsIndex] = std::chrono::duration<double, std::milli>(
        servo_call_started - last_servo_j_call_start_).count();
    }
    last_servo_j_call_start_ = servo_call_started;
    have_last_servo_j_call_start_ = true;
    const errno_t return_code = robot_client_->servo_j(
      &command, &external_axes, static_cast<float>(servo_command_period_sec_));
    status_state_[kServoJCallMsIndex] = milliseconds_since(servo_call_started);
    if (return_code != 0) {
      write_failed_ = true;
      RCLCPP_ERROR(
        kLogger,
        "ServoJ failed with SDK error code %d; hardware write is latched failed and no further "
        "commands will be sent",
        return_code);
      stop_and_disconnect("ServoJ failure");
      return hardware_interface::return_type::ERROR;
    }
  } else if (control_mode_ == 1) {
    if (!detail::all_commands_finite(joint_torque_command_)) {
      RCLCPP_ERROR(kLogger, "Rejected non-finite torque command across the six-joint vector");
      write_failed_ = true;
      stop_and_disconnect("invalid torque command");
      return hardware_interface::return_type::ERROR;
    }
  } else {
    RCLCPP_ERROR(kLogger, "Unsupported hardware control mode: %d", control_mode_);
    write_failed_ = true;
    stop_and_disconnect("unsupported control mode");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace fairino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
