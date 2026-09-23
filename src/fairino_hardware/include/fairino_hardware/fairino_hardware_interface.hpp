#ifndef FAIRINO_HARDWARE__FAIRINO_HARDWARE_INTERFACE_HPP_
#define FAIRINO_HARDWARE__FAIRINO_HARDWARE_INTERFACE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "fairino_hardware/visibility_control.h"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "libfairino/include/robot.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fairino_hardware
{

inline constexpr std::size_t kJointCount = 6;
inline constexpr std::size_t kStatusInterfaceCount = 40;
inline constexpr char kDefaultControllerIp[] = "192.168.58.2";
inline constexpr double kDefaultServoCommandPeriodSec = 0.008;

namespace detail
{
bool all_commands_finite(const std::array<double, kJointCount> & commands);
}  // namespace detail

// A small SDK boundary keeps production on FRRobot while allowing deterministic offline failures.
class RobotClient
{
public:
  virtual ~RobotClient() = default;

  virtual errno_t rpc(const char * ip) = 0;
  virtual errno_t close_rpc() = 0;
  virtual errno_t get_actual_joint_position_degree(uint8_t flag, JointPos * position) = 0;
  virtual errno_t get_robot_realtime_state(ROBOT_STATE_PKG * state) = 0;
  virtual errno_t is_in_drag_teach(uint8_t * state) = 0;
  virtual errno_t servo_move_start() = 0;
  virtual errno_t servo_move_end() = 0;
  virtual errno_t stop_motion() = 0;
  virtual errno_t servo_j(
    JointPos * position, ExaxisPos * external_axes, float command_period_sec) = 0;
};

class FairinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FairinoHardwareInterface() = default;
  explicit FairinoHardwareInterface(std::unique_ptr<RobotClient> robot_client);

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<RobotClient> make_robot_client();
  hardware_interface::CallbackReturn stop_and_disconnect(const char * reason);
  bool load_hardware_parameters();
  bool load_status_interfaces();
  void update_status_state(const ROBOT_STATE_PKG & state);

  std::array<double, kJointCount> joint_position_command_{};
  std::array<double, kJointCount> joint_velocity_command_{};
  std::array<double, kJointCount> joint_torque_command_{};
  std::array<double, kJointCount> joint_position_state_{};
  std::array<double, kJointCount> joint_velocity_state_{};
  std::array<double, kJointCount> joint_torque_state_{};
  std::array<double, kStatusInterfaceCount> status_state_{};
  bool status_interfaces_enabled_{false};
  bool velocity_state_enabled_{false};
  int control_mode_{0};
  std::string controller_ip_{kDefaultControllerIp};
  double servo_command_period_sec_{kDefaultServoCommandPeriodSec};
  double control_period_tolerance_sec_{0.002};
  bool rpc_connected_{false};
  bool servo_mode_active_{false};
  bool position_command_active_{false};
  bool write_failed_{false};
  bool period_reported_{false};
  std::chrono::steady_clock::time_point last_servo_j_call_start_{};
  bool have_last_servo_j_call_start_{false};
  std::unique_ptr<RobotClient> robot_client_;
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__FAIRINO_HARDWARE_INTERFACE_HPP_
