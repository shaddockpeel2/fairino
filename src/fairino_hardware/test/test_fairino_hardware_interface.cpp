#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace fairino_hardware
{
namespace
{

struct FakeRobotState
{
  int rpc_result{0};
  int read_result{0};
  int drag_query_result{0};
  uint8_t drag_state{0};
  int servo_start_result{0};
  int servo_end_result{0};
  int stop_result{0};
  int close_result{0};
  int servo_j_result{0};
  int servo_j_calls{0};
  float last_command_period_sec{0.0F};
  ROBOT_STATE_PKG controller_status{};
  std::vector<std::string> calls;
};

class FakeRobotClient final : public RobotClient
{
public:
  explicit FakeRobotClient(std::shared_ptr<FakeRobotState> state)
  : state_(std::move(state)) {}

  errno_t rpc(const char * ip) override
  {
    state_->calls.emplace_back(std::string{"rpc:"} + ip);
    return state_->rpc_result;
  }

  errno_t close_rpc() override
  {
    state_->calls.emplace_back("close");
    return state_->close_result;
  }

  errno_t get_actual_joint_position_degree(uint8_t flag, JointPos * position) override
  {
    state_->calls.emplace_back("read:" + std::to_string(flag));
    for (std::size_t index = 0; index < kJointCount; ++index) {
      position->jPos[index] = static_cast<double>(index + 1) * 10.0;
    }
    return state_->read_result;
  }

  errno_t get_robot_realtime_state(ROBOT_STATE_PKG * state) override
  {
    *state = state_->controller_status;
    return state_->read_result;
  }

  errno_t servo_move_start() override
  {
    state_->calls.emplace_back("start");
    return state_->servo_start_result;
  }

  errno_t is_in_drag_teach(uint8_t * state) override
  {
    state_->calls.emplace_back("is_drag");
    *state = state_->drag_state;
    return state_->drag_query_result;
  }

  errno_t servo_move_end() override
  {
    state_->calls.emplace_back("end");
    return state_->servo_end_result;
  }

  errno_t stop_motion() override
  {
    state_->calls.emplace_back("stop");
    return state_->stop_result;
  }

  errno_t servo_j(
    JointPos * /* position */, ExaxisPos * /* external_axes */,
    float command_period_sec) override
  {
    state_->calls.emplace_back("servo_j");
    ++state_->servo_j_calls;
    state_->last_command_period_sec = command_period_sec;
    return state_->servo_j_result;
  }

private:
  std::shared_ptr<FakeRobotState> state_;
};

hardware_interface::HardwareInfo make_hardware_info(
  bool include_status = false, bool include_velocity = false)
{
  hardware_interface::HardwareInfo info;
  info.name = "FairinoTestSystem";
  info.type = "system";
  info.hardware_class_type = "fairino_hardware/FairinoHardwareInterface";
  info.hardware_parameters["controller_ip"] = "127.0.0.1";
  info.hardware_parameters["servo_command_period_sec"] = "0.008";
  info.hardware_parameters["control_period_tolerance_sec"] = "0.002";
  for (std::size_t index = 0; index < kJointCount; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(index + 1);
    joint.type = "joint";
    hardware_interface::InterfaceInfo command_interface;
    command_interface.name = hardware_interface::HW_IF_POSITION;
    joint.command_interfaces.push_back(command_interface);
    hardware_interface::InterfaceInfo state_interface;
    state_interface.name = hardware_interface::HW_IF_POSITION;
    joint.state_interfaces.push_back(state_interface);
    if (include_velocity) {
      hardware_interface::InterfaceInfo velocity_interface;
      velocity_interface.name = hardware_interface::HW_IF_VELOCITY;
      joint.state_interfaces.push_back(velocity_interface);
    }
    info.joints.push_back(std::move(joint));
  }
  if (include_status) {
    hardware_interface::ComponentInfo gpio;
    gpio.name = "fairino_status";
    gpio.type = "gpio";
    const std::vector<std::string> names{
      "program_state", "robot_state", "main_error_code", "sub_error_code", "robot_mode",
      "tool_id", "work_object_id", "emergency_stop", "motion_done", "collision_state",
      "safety_stop_0", "safety_stop_1", "enabled", "payload_kg", "payload_cog_x_mm",
      "payload_cog_y_mm", "payload_cog_z_mm", "tool_x_mm", "tool_y_mm", "tool_z_mm",
      "tool_rx_deg", "tool_ry_deg", "tool_rz_deg", "wobj_x_mm", "wobj_y_mm", "wobj_z_mm",
      "wobj_rx_deg", "wobj_ry_deg", "wobj_rz_deg", "flange_x_mm", "flange_y_mm",
      "flange_z_mm", "flange_rx_deg", "flange_ry_deg", "flange_rz_deg",
      "read_position_call_ms", "read_status_call_ms", "servo_j_call_ms",
      "servo_j_interval_ms", "control_period_ms"};
    for (const auto & name : names) {
      hardware_interface::InterfaceInfo state_interface;
      state_interface.name = name;
      gpio.state_interfaces.push_back(std::move(state_interface));
    }
    info.gpios.push_back(std::move(gpio));
  }
  return info;
}

std::vector<std::string> arm_position_interfaces()
{
  std::vector<std::string> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    interfaces.push_back(
      "joint" + std::to_string(index + 1) + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interfaces;
}

TEST(CommandValidation, IncludesTheSixthPositionAndTorqueElement)
{
  std::array<double, kJointCount> commands{};
  EXPECT_TRUE(detail::all_commands_finite(commands));

  commands[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(detail::all_commands_finite(commands));

  commands[5] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(detail::all_commands_finite(commands));
}

TEST(FairinoHardwareInterface, ExportsRealtimeControllerStatusWithoutAnotherSdkClient)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  fake_state->controller_status.program_state = 1;
  fake_state->controller_status.robot_state = 1;
  fake_state->controller_status.EmergencyStop = 0;
  fake_state->controller_status.motion_done = 1;
  fake_state->controller_status.rbtEnableState = 1;
  fake_state->controller_status.load = 0.625;
  fake_state->controller_status.loadCog[0] = 1.5;
  fake_state->controller_status.flange_cur_pos[0] = -32.5;
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(
    hardware.on_init(make_hardware_info(true)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto interfaces = hardware.export_state_interfaces();
  ASSERT_EQ(interfaces.size(), kJointCount + kStatusInterfaceCount);
  EXPECT_DOUBLE_EQ(interfaces[kJointCount + 0].get_value(), 1.0);
  EXPECT_DOUBLE_EQ(interfaces[kJointCount + 8].get_value(), 1.0);
  EXPECT_DOUBLE_EQ(interfaces[kJointCount + 13].get_value(), 0.625);
  EXPECT_DOUBLE_EQ(interfaces[kJointCount + 14].get_value(), 1.5);
  EXPECT_DOUBLE_EQ(interfaces[kJointCount + 29].get_value(), -32.5);
  EXPECT_EQ(
    hardware.perform_command_mode_switch(arm_position_interfaces(), {}),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::OK);
  const auto timing_interfaces = hardware.export_state_interfaces();
  EXPECT_NEAR(timing_interfaces[kJointCount + 39].get_value(), 8.0, 1e-9);
  EXPECT_GE(timing_interfaces[kJointCount + 37].get_value(), 0.0);

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(FairinoHardwareInterface, ExportsSdkVelocityStateInRadiansPerSecond)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  fake_state->controller_status.actual_qd[0] = 90.0;
  fake_state->controller_status.actual_qd[5] = -45.0;
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(
    hardware.on_init(make_hardware_info(true, true)),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::OK);
  const auto interfaces = hardware.export_state_interfaces();
  ASSERT_EQ(interfaces.size(), 2U * kJointCount + kStatusInterfaceCount);
  EXPECT_EQ(interfaces[1].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  EXPECT_DOUBLE_EQ(interfaces[1].get_value(), M_PI / 2.0);
  EXPECT_DOUBLE_EQ(interfaces[11].get_value(), -M_PI / 4.0);
  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(FairinoHardwareInterface, RejectsNonFiniteVelocityFeedback)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(
    hardware.on_init(make_hardware_info(true, true)),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  fake_state->controller_status.actual_qd[5] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    hardware.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::ERROR);
  EXPECT_NE(
    std::find(fake_state->calls.begin(), fake_state->calls.end(), "close"),
    fake_state->calls.end());
}

TEST(FairinoHardwareInterface, MonitorsUntilControllerClaimsThenRunsServoLifecycle)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  // Before a trajectory goal, controller_manager still calls write().  It
  // must remain a no-op so the native flange drag button can acquire control.
  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(fake_state->servo_j_calls, 0);

  const auto arm_interfaces = arm_position_interfaces();
  EXPECT_EQ(
    hardware.prepare_command_mode_switch(arm_interfaces, {}),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    hardware.perform_command_mode_switch(arm_interfaces, {}),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(fake_state->servo_j_calls, 1);
  EXPECT_NEAR(fake_state->last_command_period_sec, 0.008F, 1e-6F);

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    fake_state->calls,
    (std::vector<std::string>{
        "rpc:127.0.0.1", "read:0", "is_drag", "read:0", "start", "servo_j", "end",
        "stop", "close"}));
}

TEST(FairinoHardwareInterface, RejectsPartialArmControllerClaims)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(
    hardware.prepare_command_mode_switch({"joint1/position"}, {}),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(fake_state->servo_j_calls, 0);
  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(FairinoHardwareInterface, RejectsAutomaticControlWhileDragTeachIsActive)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  fake_state->drag_state = 1;
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(
    hardware.perform_command_mode_switch(arm_position_interfaces(), {}),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(fake_state->servo_j_calls, 0);
  EXPECT_EQ(std::count(fake_state->calls.begin(), fake_state->calls.end(), "start"), 0);
  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(FairinoHardwareInterface, ReleasesServoAndKeepsLiveStateMonitoring)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto arm_interfaces = arm_position_interfaces();
  ASSERT_EQ(
    hardware.perform_command_mode_switch(arm_interfaces, {}),
    hardware_interface::return_type::OK);
  ASSERT_EQ(
    hardware.perform_command_mode_switch({}, arm_interfaces),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(
    hardware.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(fake_state->servo_j_calls, 0);

  EXPECT_EQ(
    hardware.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    fake_state->calls,
    (std::vector<std::string>{
        "rpc:127.0.0.1", "read:0", "is_drag", "read:0", "start", "end", "stop",
        "read:1", "stop", "close"}));
}

TEST(FairinoHardwareInterface, RejectsNonFiniteSixthJointBeforeSdkCall)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.perform_command_mode_switch(arm_position_interfaces(), {}),
    hardware_interface::return_type::OK);

  auto command_interfaces = hardware.export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), kJointCount);
  command_interfaces[5].set_value(std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(fake_state->servo_j_calls, 0);
  EXPECT_EQ(
    hardware.on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
}

TEST(FairinoHardwareInterface, PropagatesAndLatchesServoJFailure)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  fake_state->servo_j_result = 14;
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hardware.perform_command_mode_switch(arm_position_interfaces(), {}),
    hardware_interface::return_type::OK);

  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(fake_state->servo_j_calls, 1);

  // Even if an older controller_manager calls write again, the failed SDK path sends nothing.
  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(fake_state->servo_j_calls, 1);

  EXPECT_EQ(
    hardware.on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    fake_state->calls,
    (std::vector<std::string>{
        "rpc:127.0.0.1", "read:0", "is_drag", "read:0", "start", "servo_j", "end",
        "stop", "close"}));
}

TEST(FairinoHardwareInterface, FailedStopLatchesAndPreventsReacquisition)
{
  for (const bool fail_end : {false, true}) {
    auto fake_state = std::make_shared<FakeRobotState>();
    FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
    ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.perform_command_mode_switch(arm_position_interfaces(), {}),
      hardware_interface::return_type::OK);
    fake_state->servo_end_result = fail_end ? 14 : 0;
    fake_state->stop_result = fail_end ? 0 : 14;
    EXPECT_EQ(hardware.perform_command_mode_switch({}, arm_position_interfaces()),
      hardware_interface::return_type::ERROR);
    // Both stop calls are attempted even if the first one fails.
    EXPECT_EQ(fake_state->calls[fake_state->calls.size() - 2], "end");
    EXPECT_EQ(fake_state->calls.back(), "stop");
    fake_state->servo_end_result = fake_state->stop_result = 0;
    EXPECT_EQ(hardware.perform_command_mode_switch(arm_position_interfaces(), {}),
      hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008)),
      hardware_interface::return_type::ERROR);
    EXPECT_EQ(fake_state->servo_j_calls, 0);
  }
}

TEST(FairinoHardwareInterface, MakesAllArmCommandsUnavailableToRos2ControlAfterFailure)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  fake_state->servo_j_result = 14;
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>();
  const auto hardware_info = make_hardware_info();
  resource_manager->import_component(
    std::make_unique<FairinoHardwareInterface>(
      std::make_unique<FakeRobotClient>(fake_state)),
    hardware_info);

  rclcpp_lifecycle::State active_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    resource_manager->set_component_state(hardware_info.name, active_state),
    hardware_interface::return_type::OK);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    EXPECT_TRUE(
      resource_manager->command_interface_is_available(
        "joint" + std::to_string(index + 1) + "/position"));
  }

  const auto arm_interfaces = arm_position_interfaces();
  ASSERT_TRUE(resource_manager->prepare_command_mode_switch(arm_interfaces, {}));
  ASSERT_TRUE(resource_manager->perform_command_mode_switch(arm_interfaces, {}));

  const auto write_status = resource_manager->write(
    rclcpp::Time(0), rclcpp::Duration::from_seconds(0.008));
  EXPECT_FALSE(write_status.ok);
  EXPECT_EQ(write_status.failed_hardware_names, (std::vector<std::string>{hardware_info.name}));
  for (std::size_t index = 0; index < kJointCount; ++index) {
    EXPECT_FALSE(
      resource_manager->command_interface_is_available(
        "joint" + std::to_string(index + 1) + "/position"));
  }
  EXPECT_EQ(fake_state->servo_j_calls, 1);
}

TEST(FairinoHardwareInterface, FailsControllerAcquisitionWhenServoModeCannotStart)
{
  auto fake_state = std::make_shared<FakeRobotState>();
  fake_state->servo_start_result = 14;
  FairinoHardwareInterface hardware(std::make_unique<FakeRobotClient>(fake_state));
  ASSERT_EQ(hardware.on_init(make_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(
    hardware.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    hardware.perform_command_mode_switch(arm_position_interfaces(), {}),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(
    fake_state->calls,
    (std::vector<std::string>{
        "rpc:127.0.0.1", "read:0", "is_drag", "read:0", "start"}));
}

}  // namespace
}  // namespace fairino_hardware
