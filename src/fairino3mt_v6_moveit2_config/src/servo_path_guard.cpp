#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include "servo_path_check.hpp"

using namespace std::chrono_literals;
namespace
{
double steadyNow()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
const std::vector<std::string> joints{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
constexpr const char* candidate_topic = "/p1/servo_position_candidates";
constexpr const char* output_topic = "/fairino3mt_servo_position_controller/commands";

class Guard
{
public:
  explicit Guard(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", "p1_path_guard");
    if (!monitor_->getPlanningScene())
      throw std::runtime_error("path guard cannot load robot model");
    monitor_->startStateMonitor("/joint_states");
    monitor_->startSceneMonitor("/monitored_planning_scene");
    output_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, 1);
    fault_ = node_->create_publisher<std_msgs::msg::String>("/p1/path_guard/fault", 10);
    emergency_switch_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    feedback_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (msg->position.size() != msg->name.size())
            return;
          for (const auto& name : joints)
          {
            auto it = std::find(msg->name.begin(), msg->name.end(), name);
            if (it == msg->name.end() || !std::isfinite(msg->position[it - msg->name.begin()]))
              return;
          }
          const double stamp = rclcpp::Time(msg->header.stamp).seconds();
          if (stamp > arm_stamp_)
          {
            arm_stamp_ = stamp;
            arm_receive_ = steadyNow();
          }
        });
    state_ = node_->create_subscription<std_msgs::msg::String>(
        "/fairino/control_state", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          const bool next = msg->data == "executing_servo_tracking";
          idle_verified_ = msg->data == "monitoring_manual_ready";
          if (next && !active_)
          {
            auto current = monitor_->getStateMonitor()->getCurrentState();
            current->update();
            origin_ = current->getGlobalLinkTransform("grasp_frame");
            current->copyJointGroupPositions("fairino3mt_v6_group", last_target_);
            last_command_ = steadyNow();
          }
          active_ = next;
        });
    input_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        candidate_topic, 1, [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          if (!active_ || latched_)
            return;
          const double started = steadyNow();
          std::string error = readiness();
          if (!error.empty())
            return fail(error);
          auto result = validate(msg->data, true);
          if (!result.valid)
            return fail(result.reason);
          if (steadyNow() - started > 0.04)
            return fail("path_check_deadline");
          // Only checked targets are ever sent to the hardware-facing topic.
          output_->publish(*msg);
          last_target_ = msg->data;
          last_command_ = steadyNow();
        });
    ready_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/p1/path_guard/ready", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                      std_srvs::srv::Trigger::Response::SharedPtr response) {
          response->message = readiness();
          if (response->message.empty())
          {
            auto current = monitor_->getStateMonitor()->getCurrentState();
            std::vector<double> q;
            current->copyJointGroupPositions("fairino3mt_v6_group", q);
            auto result = validate(q, false);
            response->success = result.valid;
            response->message = result.reason;
          }
        });
    check_service_ = node_->create_service<moveit_msgs::srv::GetStateValidity>(
        "/p1/path_guard/check_target",
        [this](const moveit_msgs::srv::GetStateValidity::Request::SharedPtr request,
               moveit_msgs::srv::GetStateValidity::Response::SharedPtr response) {
          // Read-only path query, not a motion authorization or cached permit.
          if (request->robot_state.joint_state.name != joints || !readiness().empty())
            return;
          response->valid = validate(request->robot_state.joint_state.position, false).valid;
        });
    timer_ = node_->create_wall_timer(20ms, [this]() {
      if (active_ && !latched_)
      {
        auto error = readiness();
        if (!error.empty())
          fail(error);
        else if (steadyNow() - last_command_ > 0.10)
          fail("candidate_timeout");
        else if (!last_target_.empty())
        {
          auto result = validate(last_target_, true);
          if (!result.valid)
            fail(result.reason);
        }
      }
      // Repeat until bridge observes the fault, including startup message races.
      if (latched_)
      {
        fault_->publish(std_msgs::msg::String().set__data(reason_));
        // If the bridge died or did not complete release, revoke interfaces
        // directly through controller_manager. This invokes the same hardware
        // ServoMoveEnd/StopMotion sequence, never another SDK connection.
        if (!idle_verified_ && !fallback_sent_ && steadyNow() - fault_time_ > 0.30)
        {
          fallback_sent_ = true;
          if (emergency_switch_->service_is_ready())
          {
            auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            request->deactivate_controllers = {"fairino3mt_servo_position_controller",
                                               "fairino3mt_execution_controller"};
            request->strictness = request->BEST_EFFORT;
            request->activate_asap = true;
            request->timeout.sec = 1;
            emergency_switch_->async_send_request(request, [this](
                rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
              RCLCPP_ERROR(node_->get_logger(), "Fallback controller stop acknowledged=%s; inspect stop feedback",
                           future.get()->ok ? "true" : "false");
            });
          }
          else
            RCLCPP_FATAL(node_->get_logger(), "Stop service unavailable; operator must use physical stop");
        }
      }
    });
    sync_thread_ = std::thread([this]() {
      while (!quit_ && rclcpp::ok())
      {
        if (monitor_->requestPlanningSceneState("/get_planning_scene"))
          last_scene_sync_ = steadyNow();
        for (int i = 0; i < 10 && !quit_; ++i)
          std::this_thread::sleep_for(50ms);
      }
    });
  }

  ~Guard()
  {
    quit_ = true;
    if (sync_thread_.joinable())
      sync_thread_.join();
  }

private:
  std::string readiness()
  {
    if (latched_)
      return "latched:" + reason_;
    if (node_->count_publishers("/fairino/control_state") != 1)
      return "control_supervisor_missing_or_duplicated";
    if (steadyNow() - last_scene_sync_.load() > 2.0)
      return "planning_scene_sync_stale";
    auto state_monitor = monitor_->getStateMonitor();
    const double age = node_->now().seconds() - arm_stamp_;
    if (!state_monitor->haveCompleteState() || age < 0 || age > 0.10 ||
        steadyNow() - arm_receive_ > 0.10)
      return "joint_state_stale_or_incomplete";
    if (node_->count_publishers(output_topic) != 1 || node_->count_publishers(candidate_topic) != 1)
      return "unexpected_command_publishers";
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
    if (!scene->getWorld()->hasObject("rviz_ground_plane"))
      return "ground_scene_missing";
    return "";
  }

  p1::PathResult validate(const std::vector<double>& target, bool streaming)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
    // Preserve scene attached bodies, but refresh all measured variable positions.
    auto measured = scene->getCurrentState();
    auto live = monitor_->getStateMonitor()->getCurrentState();
    measured.setVariablePositions(live->getVariablePositions());
    measured.update();
    std::vector<double> actual;
    measured.copyJointGroupPositions("fairino3mt_v6_group", actual);
    const auto origin = streaming ? origin_ : measured.getGlobalLinkTransform("grasp_frame");
    auto result = p1::checkPath(*scene.operator->(), measured, actual, target, origin);
    if (result.valid && streaming && !last_target_.empty())
    {
      auto previous = p1::checkPath(*scene.operator->(), measured, last_target_, target, origin);
      previous.samples += result.samples;
      return previous;
    }
    return result;
  }

  void fail(const std::string& reason)
  {
    latched_ = true;
    fault_time_ = steadyNow();
    reason_ = reason;
    fault_->publish(std_msgs::msg::String().set__data(reason));
    RCLCPP_ERROR(node_->get_logger(), "Rejecting Servo output and requesting stop: %s", reason.c_str());
  }
  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
  rclcpp::Service<moveit_msgs::srv::GetStateValidity>::SharedPtr check_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr emergency_switch_;
  std::thread sync_thread_;
  std::atomic<bool> quit_{false};
  std::atomic<double> last_scene_sync_{0};
  bool active_{false}, latched_{false};
  bool idle_verified_{false}, fallback_sent_{false};
  std::string reason_;
  double last_command_{0};
  double fault_time_{0};
  double arm_stamp_{0}, arm_receive_{0};
  Eigen::Isometry3d origin_{Eigen::Isometry3d::Identity()};
  std::vector<double> last_target_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "p1_path_guard", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try
  {
    Guard guard(node);
    rclcpp::spin(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
