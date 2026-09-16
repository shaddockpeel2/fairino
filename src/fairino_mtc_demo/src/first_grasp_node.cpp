#include <control_msgs/action/gripper_command.hpp>
#include <fairino_msgs/action/pick_color.hpp>
#include <fairino_msgs/srv/set_target_color.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;
using PickColor = fairino_msgs::action::PickColor;
using GoalHandlePickColor = rclcpp_action::ServerGoalHandle<PickColor>;

constexpr double kPi = 3.14159265358979323846;

bool isFinite(double value)
{
  return std::isfinite(value);
}

bool finiteVector(const std::vector<double> & values, std::size_t expected)
{
  return values.size() == expected &&
         std::all_of(values.begin(), values.end(), [](double value) {return isFinite(value);});
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

geometry_msgs::msg::Quaternion normalizedQuaternion(const std::vector<double> & xyzw)
{
  if (!finiteVector(xyzw, 4)) {
    throw std::runtime_error("grasp_orientation_xyzw must contain four finite values");
  }
  const double norm = std::sqrt(
    xyzw[0] * xyzw[0] + xyzw[1] * xyzw[1] +
    xyzw[2] * xyzw[2] + xyzw[3] * xyzw[3]);
  if (norm < 1e-9) {
    throw std::runtime_error("grasp_orientation_xyzw has zero norm");
  }
  geometry_msgs::msg::Quaternion result;
  result.x = xyzw[0] / norm;
  result.y = xyzw[1] / norm;
  result.z = xyzw[2] / norm;
  result.w = xyzw[3] / norm;
  return result;
}

geometry_msgs::msg::Quaternion normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & input)
{
  return normalizedQuaternion(std::vector<double>{input.x, input.y, input.z, input.w});
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(yaw * 0.5);
  result.w = std::cos(yaw * 0.5);
  return result;
}

geometry_msgs::msg::Quaternion conjugate(const geometry_msgs::msg::Quaternion & q)
{
  geometry_msgs::msg::Quaternion result;
  result.x = -q.x;
  result.y = -q.y;
  result.z = -q.z;
  result.w = q.w;
  return result;
}

geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  geometry_msgs::msg::Quaternion result;
  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return result;
}

geometry_msgs::msg::Vector3 rotate(
  const geometry_msgs::msg::Quaternion & q,
  const geometry_msgs::msg::Vector3 & vector)
{
  geometry_msgs::msg::Quaternion input;
  input.x = vector.x;
  input.y = vector.y;
  input.z = vector.z;
  input.w = 0.0;
  const auto output = multiply(multiply(q, input), conjugate(q));
  geometry_msgs::msg::Vector3 result;
  result.x = output.x;
  result.y = output.y;
  result.z = output.z;
  return result;
}

double quaternionDistance(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  const double dot = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
  return 2.0 * std::acos(clamp(dot, 0.0, 1.0));
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & input)
{
  const auto q = normalizedQuaternion(input);
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double positionDistance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream output;
  for (const char character : input) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string & id, const std::string & frame_id,
  const std::vector<double> & size, const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame_id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(size.begin(), size.end());
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  return object;
}

std::string stateName(int state)
{
  switch (state) {
    case 0: return "waiting_for_target";
    case 1: return "planning";
    case 2: return "waiting_for_descend_command";
    case 3: return "descending_and_closing";
    case 4: return "waiting_for_manual_grip_confirmation";
    case 5: return "attaching_and_lifting";
    case 6: return "plan_only_complete";
    case 7: return "complete_holding";
    case 8: return "failed_holding_state_unchanged";
    case 9: return "canceled_holding_state_unchanged";
    case 10: return "retryable_failure_before_arm_motion";
    case 11: return "placing";
    case 12: return "returning_home";
    case 13: return "complete_sorted_home";
    case 14: return "batch_scanning";
    case 15: return "batch_complete";
    case 16: return "batch_complete_with_skips";
    default: return "unknown";
  }
}
}  // namespace

class FirstGraspTask
{
public:
  explicit FirstGraspTask(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    readParameters();
    validateParameters();
    locked_grasp_orientation_ = grasp_orientation_;
    locked_object_size_m_ = object_size_m_;

    move_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(tip_frame_);
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group_->allowReplanning(false);
    move_group_->startStateMonitor(2.0);

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(node_, gripper_action_);
    display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 1);
    state_publisher_ = node_->create_publisher<std_msgs::msg::String>("~/state", 10);

    pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {onTargetPose(*message);});
    valid_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      target_valid_topic_, 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {onTargetValid(message->data);});
    status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      target_status_topic_, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {onTargetStatus(message->data);});
    if (use_target_size_) {
      size_subscription_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        target_size_topic_, 10,
        [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message) {
          onTargetSize(*message);
        });
    }

    start_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    start_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {start(response, false);},
      rmw_qos_profile_services_default, start_callback_group_);
    continuous_start_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/start_continuous",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {start(response, true);},
      rmw_qos_profile_services_default, start_callback_group_);
    descend_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/descend_and_close",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {authorizeDescend(response);});
    lift_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/confirm_grip_and_lift",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {confirmGrip(response);});
    cancel_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {cancel(response);});
    release_ack_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/acknowledge_object_released",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        acknowledgeObjectReleased(response);
      }, rmw_qos_profile_services_default, start_callback_group_);
    perception_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(
      perception_reset_service_);
    perception_skip_client_ = node_->create_client<std_srvs::srv::Trigger>(
      perception_skip_service_);
    perception_color_client_ = node_->create_client<fairino_msgs::srv::SetTargetColor>(
      perception_color_service_);
    pick_hold_server_ = createPickServer("~/pick_hold", false);
    pick_sort_server_ = createPickServer("~/pick_sort", true);
    state_timer_ = node_->create_wall_timer(500ms, [this]() {publishState();});

    RCLCPP_INFO(
      node_->get_logger(),
      "P5 first-grasp task ready in %s mode. /start accepts one current stable target snapshot; no action is automatic.",
      execute_ ? "EXECUTION" : "PLAN-ONLY");
    publishState();
  }

  ~FirstGraspTask()
  {
    canceled_.store(true);
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
  }

private:
  enum State
  {
    WAITING_TARGET = 0,
    PLANNING = 1,
    WAITING_DESCEND = 2,
    DESCENDING = 3,
    WAITING_GRIP_CONFIRMATION = 4,
    LIFTING = 5,
    PLAN_ONLY_COMPLETE = 6,
    COMPLETE = 7,
    FAILED = 8,
    CANCELED = 9,
    RETRYABLE_FAILED = 10,
    PLACING = 11,
    RETURNING_HOME = 12,
    SORT_COMPLETE = 13,
    BATCH_SCANNING = 14,
    BATCH_COMPLETE = 15,
    BATCH_COMPLETE_WITH_SKIPS = 16,
  };

  struct PlannedMotion
  {
    std::string name;
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::RobotState start_state;
    moveit::core::RobotStatePtr end_state;
  };

  struct JointPathMetrics
  {
    double path_length_rad{0.0};
    std::vector<std::string> joint_names;
    std::vector<double> joint_travel_rad;
  };

  struct GraspCandidate
  {
    double yaw_rad{0.0};
    geometry_msgs::msg::Quaternion orientation;
  };

  struct IgnoredTarget
  {
    std::string color;
    geometry_msgs::msg::Point center;
    bool completed_source{false};
    bool observed_after_cycle{false};
  };

  static bool supportedColor(const std::string & color)
  {
    return color == "red" || color == "yellow" || color == "blue" || color == "green";
  }

  rclcpp_action::Server<PickColor>::SharedPtr createPickServer(
    const std::string & name, bool sort_task)
  {
    return rclcpp_action::create_server<PickColor>(
      node_, name,
      [](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const PickColor::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandlePickColor>) {
        canceled_.store(true);
        move_group_->stop();
        if (gripper_client_) {
          gripper_client_->async_cancel_all_goals();
        }
        condition_.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this, sort_task](const std::shared_ptr<GoalHandlePickColor> goal_handle) {
        if (action_goal_claimed_.exchange(true)) {
          auto result = std::make_shared<PickColor::Result>();
          result->success = false;
          result->error_code = PickColor::Result::BUSY;
          result->message = "another grasp task is active";
          result->color = goal_handle->get_goal()->color;
          result->executed = false;
          goal_handle->abort(result);
          return;
        }
        if (action_thread_.joinable()) {
          action_thread_.join();
        }
        action_thread_ = std::thread(
          [this, goal_handle, sort_task]() {executePickAction(goal_handle, sort_task);});
      });
  }

  void finishPickAction(
    const std::shared_ptr<GoalHandlePickColor> & goal_handle,
    uint16_t error_code, const std::string & message, bool success)
  {
    auto result = std::make_shared<PickColor::Result>();
    result->success = success;
    result->error_code = error_code;
    result->message = message;
    result->color = goal_handle->get_goal()->color;
    result->executed = execute_ && success;
    if (goal_handle->is_canceling() || error_code == PickColor::Result::CANCELED) {
      goal_handle->canceled(result);
    } else if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      active_action_goal_.reset();
    }
    worker_running_.store(false);
    automatic_task_ = false;
    action_goal_claimed_.store(false);
  }

  bool setPerceptionTargetColor(
    const std::string & color, uint64_t & generation, std::string & reason)
  {
    if (!perception_color_client_->wait_for_service(1s)) {
      reason = "set_target_color service is unavailable";
      return false;
    }
    auto request = std::make_shared<fairino_msgs::srv::SetTargetColor::Request>();
    request->color = color;
    auto future = perception_color_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      reason = "set_target_color service timed out";
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      reason = response->message;
      return false;
    }
    generation = response->generation;
    recordEvent(
      "target_filter_set", "color=" + color + ",generation=" + std::to_string(generation));
    return true;
  }

  bool waitForRequestedTarget(
    const std::string & requested_color, uint64_t filter_generation,
    uint16_t & error_code, std::string & reason)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(target_acquisition_timeout_s_));
    bool observed_fresh_status = false;
    bool observed_not_found = false;
    while (rclcpp::ok() && !canceled_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::string status;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status = latest_target_status_;
      }
      const auto data = nlohmann::json::parse(status, nullptr, false);
      if (!data.is_discarded() && data.is_object() &&
        data.value("detection_mode", "") == "colors" &&
        data.value("target_filter_color", "") == requested_color &&
        data.value("filter_generation", uint64_t{0}) == filter_generation)
      {
        observed_fresh_status = true;
        const std::string perception_reason = data.value("reason", "");
        if (perception_reason.rfind("requested_color_not_unique", 0) == 0) {
          error_code = PickColor::Result::TARGET_NOT_UNIQUE;
          reason = "more than one " + requested_color + " target is physically valid";
          return false;
        }
        if (perception_reason.rfind("requested_color_not_found", 0) == 0 ||
          perception_reason == "color_target_count_0")
        {
          observed_not_found = true;
        }
        geometry_msgs::msg::PoseStamped target;
        std::vector<double> target_object_size;
        std::string target_status;
        std::string target_color;
        std::string snapshot_reason;
        if (targetSnapshot(
            target, target_object_size, target_status, target_color, snapshot_reason,
            requested_color, filter_generation))
        {
          if (!lockTargetSnapshot(
              target, target_object_size, target_status, target_color, reason))
          {
            error_code = PickColor::Result::PRECONDITION_FAILED;
            return false;
          }
          publishState();
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    if (canceled_.load()) {
      error_code = PickColor::Result::CANCELED;
      reason = "task canceled while waiting for a fresh target";
    } else if (observed_not_found) {
      error_code = PickColor::Result::TARGET_NOT_FOUND;
      reason = "no physically valid " + requested_color + " target was found";
    } else if (observed_fresh_status) {
      error_code = PickColor::Result::TARGET_STALE;
      reason = "requested target did not become stable and fresh before timeout";
    } else {
      error_code = PickColor::Result::PERCEPTION_UNAVAILABLE;
      reason = "no status arrived for the requested perception filter generation";
    }
    return false;
  }

  void executePickAction(
    const std::shared_ptr<GoalHandlePickColor> goal_handle, bool sort_task)
  {
    const std::string color = goal_handle->get_goal()->color;
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      active_action_goal_ = goal_handle;
    }
    if (goal_handle->is_canceling()) {
      canceled_.store(true);
      finishPickAction(
        goal_handle, PickColor::Result::CANCELED,
        "task canceled before precondition checks", false);
      return;
    }
    if (!supportedColor(color)) {
      finishPickAction(
        goal_handle, PickColor::Result::INVALID_COLOR,
        "color must be one of: red, yellow, blue, green", false);
      return;
    }

    std::unique_lock<std::mutex> start_lock(start_mutex_);
    std::string reason;
    if (sort_task && !sort_enabled_) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "pick_sort requires sort_enabled=true and configured placement regions", false);
      return;
    }
    if (!p6Gate(reason)) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "P6 execution gate refused the task: " + reason, false);
      return;
    }
    State prior_state;
    if (!stateAllowsStart(prior_state, reason)) {
      finishPickAction(goal_handle, PickColor::Result::BUSY, reason, false);
      return;
    }
    if (prior_state == COMPLETE) {
      finishPickAction(
        goal_handle, PickColor::Result::HOLDING_OBJECT,
        "a previous pick_hold task still owns the held object; release it before another task",
        false);
      return;
    }
    if (prior_state == SORT_COMPLETE && !archivePreviousSortedCubeScene(reason)) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "cannot prepare the previous sorted scene: " + reason, false);
      return;
    }

    try {
      locked_home_state_ = currentStartState(false);
      locked_home_pose_ = move_group_->getCurrentPose(tip_frame_).pose;
    } catch (const std::exception & error) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        std::string("cannot snapshot the task start state: ") + error.what(), false);
      return;
    }

    batch_mode_ = false;
    continuous_requested_ = false;
    automatic_task_ = true;
    task_sort_enabled_ = sort_task;
    canceled_.store(false);
    if (goal_handle->is_canceling()) {
      canceled_.store(true);
      finishPickAction(
        goal_handle, PickColor::Result::CANCELED,
        "task canceled before target acquisition", false);
      return;
    }
    worker_running_.store(true);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      locked_color_ = color;
    }
    setState(
      WAITING_TARGET,
      std::string(sort_task ? "pick_sort" : "pick_hold") +
      " accepted; requesting a fresh unique " + color + " target");
    start_lock.unlock();

    uint64_t filter_generation = 0;
    if (!setPerceptionTargetColor(color, filter_generation, reason)) {
      finishPickAction(
        goal_handle, PickColor::Result::PERCEPTION_UNAVAILABLE, reason, false);
      return;
    }
    uint16_t acquisition_error = PickColor::Result::INTERNAL_ERROR;
    if (!waitForRequestedTarget(color, filter_generation, acquisition_error, reason)) {
      if (acquisition_error == PickColor::Result::CANCELED) {
        setState(CANCELED, reason);
      } else {
        setState(RETRYABLE_FAILED, reason);
      }
      finishPickAction(goal_handle, acquisition_error, reason, false);
      return;
    }

    const bool completed = runCycle();
    State final_state;
    std::string final_detail;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      final_state = state_;
      final_detail = state_detail_;
    }
    if (completed &&
      (final_state == PLAN_ONLY_COMPLETE ||
      (!sort_task && final_state == COMPLETE) ||
      (sort_task && final_state == SORT_COMPLETE)))
    {
      const std::string completion = execute_ ?
        (sort_task ?
        "pick_sort motion completed once; the selected cube was released in its color region" :
        "pick_hold motion completed once; the gripper close was open-loop and the cube remains held") :
        (sort_task ? "pick_sort full route planned; no command was sent" :
        "pick_hold route planned; no command was sent");
      finishPickAction(goal_handle, PickColor::Result::OK, completion, true);
      return;
    }
    if (final_state == CANCELED || canceled_.load()) {
      finishPickAction(goal_handle, PickColor::Result::CANCELED, final_detail, false);
      return;
    }
    finishPickAction(
      goal_handle,
      arm_motion_started_.load() ? PickColor::Result::EXECUTION_FAILED :
      PickColor::Result::PLANNING_FAILED,
      final_detail, false);
  }

  void readParameters()
  {
    auto declare_string = [this](const std::string & name, const std::string & value) {
        return node_->declare_parameter<std::string>(name, value);
      };
    auto declare_double = [this](const std::string & name, double value) {
        return node_->declare_parameter<double>(name, value);
      };
    auto declare_vector = [this](const std::string & name, const std::vector<double> & value) {
        return node_->declare_parameter<std::vector<double>>(name, value);
      };

    arm_group_ = declare_string("arm_group", "fairino3mt_v6_group");
    base_frame_ = declare_string("base_frame", "base_link");
    tip_frame_ = declare_string("tip_frame", "grasp_frame");
    gripper_joint_ = declare_string("gripper_joint", "gripper_command_joint");
    gripper_action_ = declare_string(
      "gripper_action", "/ydi_j22_gripper_controller/gripper_cmd");
    target_pose_topic_ = declare_string(
      "target_pose_topic", "/perception/blue_cube_localizer/top_pose");
    target_valid_topic_ = declare_string(
      "target_valid_topic", "/perception/blue_cube_localizer/valid");
    target_status_topic_ = declare_string(
      "target_status_topic", "/perception/blue_cube_localizer/status");
    target_size_topic_ = declare_string(
      "target_size_topic", "/perception/blue_cube_localizer/object_size");
    perception_reset_service_ = declare_string(
      "perception_reset_service", "/perception/blue_cube_localizer/reset");
    perception_skip_service_ = declare_string(
      "perception_skip_service", "/perception/blue_cube_localizer/skip_current_sort_target");
    perception_color_service_ = declare_string(
      "perception_color_service", "/perception/blue_cube_localizer/set_target_color");
    cube_id_ = declare_string("cube_id", "target_cube");
    ground_plane_id_ = declare_string("ground_plane_id", "rviz_ground_plane");
    log_directory_ = declare_string("log_directory", "evidence/P5/runs");
    p6_gate_file_ = declare_string(
      "p6_gate_file", "/home/industio/fairino_ws/evidence/P6/p6_execution_gate.txt");
    sort_enabled_ = node_->declare_parameter<bool>("sort_enabled", false);
    sort_grid_file_ = declare_string("sort_grid_file", "");
    for (const auto & color : {"red", "green", "blue", "yellow"}) {
      place_centers_[color] = declare_vector(
        std::string("place_") + color + "_center_m", std::vector<double>{});
      place_slots_[color] = declare_vector(
        std::string("place_") + color + "_slots_m", std::vector<double>{});
    }
    release_bottom_gap_m_ = declare_double("release_bottom_gap_m", 0.005);
    batch_empty_timeout_s_ = declare_double("batch_empty_timeout_s", 10.0);
    batch_target_match_distance_m_ = declare_double(
      "batch_target_match_distance_m", 0.025);

    execute_ = node_->declare_parameter<bool>("execute", false);
    approach_clearance_m_ = declare_double("approach_clearance_m", 0.010);
    grasp_depth_m_ = declare_double("grasp_depth_m", 0.015);
    lift_distance_m_ = declare_double("lift_distance_m", 0.050);
    pre_open_position_m_ = declare_double("pre_open_position_m", 0.060);
    close_position_m_ = declare_double("close_position_m", 0.026);
    object_size_m_ = declare_vector("object_size_m", {0.060, 0.030, 0.030});
    use_target_size_ = node_->declare_parameter<bool>("use_target_size", true);
    square_aspect_ratio_max_ = declare_double("square_aspect_ratio_max", 1.10);
    grasp_orientation_ = normalizedQuaternion(
      declare_vector("grasp_orientation_xyzw", std::vector<double>{}));
    use_target_yaw_ = node_->declare_parameter<bool>("use_target_yaw", true);
    target_yaw_offset_rad_ = declare_double("target_yaw_offset_rad", -kPi * 0.5);
    target_yaw_symmetry_rad_ = declare_double("target_yaw_symmetry_rad", kPi);
    maximum_target_tilt_rad_ = declare_double("maximum_target_tilt_rad", 5.0 * kPi / 180.0);
    target_x_bounds_ = declare_vector("target_x_bounds_m", std::vector<double>{});
    target_y_bounds_ = declare_vector("target_y_bounds_m", std::vector<double>{});
    target_z_bounds_ = declare_vector("target_top_z_bounds_m", std::vector<double>{});
    ground_plane_xy_size_ = declare_vector("ground_plane_xy_size_m", {2.0, 2.0});
    ground_plane_z_m_ = declare_double("ground_plane_z_m", 0.0);
    ground_plane_thickness_m_ = declare_double("ground_plane_thickness_m", 0.020);
    ground_plane_contact_epsilon_m_ = declare_double(
      "ground_plane_contact_epsilon_m", 0.00001);

    velocity_scale_ = declare_double("velocity_scale", 0.065);
    acceleration_scale_ = declare_double("acceleration_scale", 0.065);
    planning_time_s_ = declare_double("planning_time_s", 10.0);
    planning_attempts_ = node_->declare_parameter<int>("planning_attempts", 5);
    cartesian_step_m_ = declare_double("cartesian_step_m", 0.001);
    cartesian_jump_threshold_ = declare_double("cartesian_jump_threshold", 0.0);
    maximum_cartesian_joint_step_rad_ = declare_double(
      "maximum_cartesian_joint_step_rad", 0.10);
    minimum_cartesian_fraction_ = declare_double("minimum_cartesian_fraction", 0.999999);
    target_message_timeout_s_ = declare_double("target_message_timeout_s", 1.0);
    target_acquisition_timeout_s_ = declare_double("target_acquisition_timeout_s", 4.0);
    arrival_position_tolerance_m_ = declare_double("arrival_position_tolerance_m", 0.003);
    arrival_orientation_tolerance_rad_ = declare_double(
      "arrival_orientation_tolerance_rad", 2.0 * kPi / 180.0);
    action_timeout_s_ = declare_double("action_timeout_s", 12.0);
    state_timeout_s_ = declare_double("state_timeout_s", 3.0);
    gripper_state_tolerance_m_ = declare_double("gripper_state_tolerance_m", 0.003);
    maximum_cube_plane_gap_m_ = declare_double("maximum_cube_plane_gap_m", 0.010);
    p6_gate_max_age_s_ = declare_double("p6_gate_max_age_s", 600.0);

    touch_links_ = node_->declare_parameter<std::vector<std::string>>(
      "touch_links",
      {"gripper_finger_px_py_link", "gripper_finger_px_ny_link",
        "gripper_finger_nx_py_link", "gripper_finger_nx_ny_link"});
  }

  void validateParameters() const
  {
    const auto positive = [](double value) {return isFinite(value) && value > 0.0;};
    if (arm_group_.empty() || base_frame_.empty() || tip_frame_.empty() ||
      gripper_action_.empty() ||
      cube_id_.empty() || ground_plane_id_.empty() ||
      (use_target_size_ && target_size_topic_.empty()))
    {
      throw std::runtime_error("frame, group, action and scene object names must not be empty");
    }
    if (!positive(approach_clearance_m_) || !positive(grasp_depth_m_) ||
      !positive(lift_distance_m_) ||
      !positive(release_bottom_gap_m_) ||
      !positive(cartesian_step_m_) || !positive(planning_time_s_) ||
      !positive(target_message_timeout_s_) || !positive(action_timeout_s_) ||
      !positive(target_acquisition_timeout_s_) ||
      !positive(state_timeout_s_) || !positive(gripper_state_tolerance_m_) ||
      !positive(ground_plane_thickness_m_) || !positive(maximum_cube_plane_gap_m_) ||
      !positive(maximum_cartesian_joint_step_rad_) ||
      !positive(p6_gate_max_age_s_) ||
      !positive(batch_empty_timeout_s_) ||
      !positive(batch_target_match_distance_m_) ||
      planning_attempts_ < 1)
    {
      throw std::runtime_error("distance, timeout and planning parameters must be positive");
    }
    if (!finiteVector(object_size_m_, 3) ||
      std::any_of(
        object_size_m_.begin(), object_size_m_.end(),
        [](double value) {return value <= 0.0;}))
    {
      throw std::runtime_error("object_size_m must contain three positive finite dimensions");
    }
    if (!isFinite(square_aspect_ratio_max_) || square_aspect_ratio_max_ < 1.0) {
      throw std::runtime_error("square_aspect_ratio_max must be finite and >= 1");
    }
    if (!isFinite(target_yaw_offset_rad_) || !positive(target_yaw_symmetry_rad_) ||
      target_yaw_symmetry_rad_ > 2.0 * kPi || !positive(maximum_target_tilt_rad_) ||
      maximum_target_tilt_rad_ >= kPi * 0.5)
    {
      throw std::runtime_error("target-yaw offset, symmetry or tilt parameters are invalid");
    }
    const int symmetry_count = static_cast<int>(std::lround(2.0 * kPi / target_yaw_symmetry_rad_));
    if (symmetry_count < 1 || symmetry_count > 8 ||
      std::abs(symmetry_count * target_yaw_symmetry_rad_ - 2.0 * kPi) > 1e-6)
    {
      throw std::runtime_error("target_yaw_symmetry_rad must divide one full turn");
    }
    if (!isFinite(cartesian_jump_threshold_) || cartesian_jump_threshold_ < 0.0) {
      throw std::runtime_error("cartesian_jump_threshold must be finite and >= 0");
    }
    if (!isFinite(pre_open_position_m_) || !isFinite(close_position_m_) ||
      pre_open_position_m_ <= close_position_m_ || close_position_m_ < 0.0)
    {
      throw std::runtime_error("gripper parameters must satisfy pre_open > close >= 0");
    }
    if (!(velocity_scale_ > 0.0 && velocity_scale_ <= 1.0) ||
      !(acceleration_scale_ > 0.0 && acceleration_scale_ <= 1.0))
    {
      throw std::runtime_error("velocity_scale and acceleration_scale must be in (0, 1]");
    }
    for (const auto * bounds : { & target_x_bounds_, & target_y_bounds_, & target_z_bounds_}) {
      if (!finiteVector(*bounds, 2) || (*bounds)[0] >= (*bounds)[1]) {
        throw std::runtime_error("all target bounds must contain increasing finite min/max pairs");
      }
    }
    if (!finiteVector(ground_plane_xy_size_, 2) ||
      std::any_of(
        ground_plane_xy_size_.begin(), ground_plane_xy_size_.end(),
        [](double value) {return value <= 0.0;}) ||
      !isFinite(ground_plane_z_m_) || !isFinite(ground_plane_contact_epsilon_m_) ||
      ground_plane_contact_epsilon_m_ < 0.0 ||
      ground_plane_contact_epsilon_m_ >= ground_plane_thickness_m_)
    {
      throw std::runtime_error(
              "ground plane size/Z/thickness/contact epsilon parameters are invalid");
    }
    if (touch_links_.empty()) {
      throw std::runtime_error("touch_links must name only the intended gripper contact links");
    }
    if (sort_enabled_) {
      if (sort_grid_file_.empty()) {
        throw std::runtime_error("sorting requires a projected A4 grid file");
      }
      for (const auto & color : {"red", "green", "blue", "yellow"}) {
        const auto & center = place_centers_.at(color);
        const auto & slots = place_slots_.at(color);
        if (!finiteVector(center, 3) ||
          std::abs(center[2] - ground_plane_z_m_) > 0.001)
        {
          throw std::runtime_error(
                  std::string("invalid ") + color + " region center or grid plane Z");
        }
        if (slots.empty() || slots.size() % 3 != 0 ||
          !finiteVector(slots, slots.size()))
        {
          throw std::runtime_error(std::string("invalid ") + color + " placement slots");
        }
        for (std::size_t index = 2; index < slots.size(); index += 3) {
          if (std::abs(slots[index] - ground_plane_z_m_) > 0.001) {
            throw std::runtime_error(
                    std::string("invalid ") + color + " placement slot plane Z");
          }
        }
      }
    }
    if (execute_ && p6_gate_file_.empty()) {
      throw std::runtime_error("execution mode requires p6_gate_file");
    }
  }

  bool p6Gate(std::string & reason) const
  {
    if (!execute_) {
      return true;
    }
    const std::filesystem::path path = std::filesystem::absolute(p6_gate_file_);
    std::error_code error;
    const auto modified = std::filesystem::last_write_time(path, error);
    if (error) {
      reason = "P6 gate is missing or unreadable: " + path.string();
      return false;
    }

    std::ifstream stream(path);
    std::string line;
    if (!std::getline(stream, line) || line != "P6_EXECUTION_GATE_V1") {
      reason = "P6 gate has an invalid signature";
      return false;
    }
    std::unordered_map<std::string, std::string> values;
    while (std::getline(stream, line)) {
      const auto separator = line.find('=');
      if (separator == std::string::npos || separator == 0U ||
        !values.emplace(line.substr(0, separator), line.substr(separator + 1)).second)
      {
        reason = "P6 gate contains malformed or duplicate fields";
        return false;
      }
    }
    if (values["passed"] != "true") {
      reason = "P6 gate reports failed; inspect " + values["report_path"];
      return false;
    }
    if (values["control_path"] != "moveit_joint_servoj") {
      reason = "P6 gate was not issued for the MoveIt joint ServoJ path";
      return false;
    }
    if (values["generated_at"].empty() || values["report_path"].empty()) {
      reason = "P6 gate is missing its generation time or evidence report";
      return false;
    }
    const std::string identity = values["generated_at"] + "\n" + values["report_path"];
    {
      std::lock_guard<std::mutex> lock(p6_mutex_);
      if (identity == accepted_p6_gate_identity_) {
        return true;
      }
    }

    const double age = std::chrono::duration<double>(
      std::filesystem::file_time_type::clock::now() - modified).count();
    if (!isFinite(age) || age < -1.0 || age > p6_gate_max_age_s_) {
      std::ostringstream message;
      message << "P6 gate has not yet been accepted in this grasp session and is stale: age_s="
              << age << ", initial_limit_s=" << p6_gate_max_age_s_;
      reason = message.str();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(p6_mutex_);
      accepted_p6_gate_identity_ = identity;
    }
    return true;
  }

  void onTargetPose(const geometry_msgs::msg::PoseStamped & message)
  {
    const auto & orientation = message.pose.orientation;
    const double orientation_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (message.header.frame_id != base_frame_ ||
      !isFinite(message.pose.position.x) || !isFinite(message.pose.position.y) ||
      !isFinite(message.pose.position.z) ||
      !isFinite(orientation_norm) || orientation_norm < 1e-9)
    {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Rejecting target pose with wrong frame, position or orientation");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_target_ = message;
    target_pose_received_ = node_->now();
    have_target_pose_ = true;
    recent_target_poses_.push_back(message);
    if (recent_target_poses_.size() > 32) {
      recent_target_poses_.pop_front();
    }
  }

  void onTargetValid(bool valid)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_valid_ = valid;
    target_valid_received_ = node_->now();
  }

  void onTargetStatus(const std::string & status)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_target_status_ = status;
    target_status_received_ = node_->now();
  }

  void onTargetSize(const geometry_msgs::msg::Vector3Stamped & message)
  {
    const auto & size = message.vector;
    if (message.header.frame_id != base_frame_ ||
      !isFinite(size.x) || !isFinite(size.y) || !isFinite(size.z) ||
      size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0)
    {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Rejecting target size with wrong frame or invalid dimensions");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_target_size_ = message;
    target_size_received_ = node_->now();
    have_target_size_ = true;
    recent_target_sizes_.push_back(message);
    if (recent_target_sizes_.size() > 32) {
      recent_target_sizes_.pop_front();
    }
  }

  bool targetSnapshot(
    geometry_msgs::msg::PoseStamped & result, std::vector<double> & object_size,
    std::string & status, std::string & color,
    std::string & reason, const std::string & requested_color = "",
    uint64_t filter_generation = 0) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = node_->now();
    if (!have_target_pose_) {
      reason = "no target pose received";
      return false;
    }
    if (!target_valid_) {
      reason = "perception is not currently stable";
      if (!latest_target_status_.empty()) {
        reason += "; status=" + latest_target_status_;
      }
      return false;
    }
    if (use_target_size_ && !have_target_size_) {
      reason = "no target object size received";
      return false;
    }
    if ((now - target_pose_received_).seconds() > target_message_timeout_s_ ||
      (now - target_valid_received_).seconds() > target_message_timeout_s_ ||
      (use_target_size_ &&
      (now - target_size_received_).seconds() > target_message_timeout_s_))
    {
      reason = "perception heartbeat/pose is stale";
      return false;
    }
    status = latest_target_status_;
    const geometry_msgs::msg::PoseStamped * selected_pose = &latest_target_;
    const geometry_msgs::msg::Vector3Stamped * selected_size = &latest_target_size_;
    if (sort_enabled_ || automatic_task_) {
      if ((now - target_status_received_).seconds() > target_message_timeout_s_) {
        reason = "four-color status is stale";
        return false;
      }
      const auto data = nlohmann::json::parse(status, nullptr, false);
      if (data.is_discarded() || !data.is_object() ||
        !data.contains("color") || !data["color"].is_string() ||
        !data.contains("sample_stamp") || !data["sample_stamp"].is_number() ||
        !data.contains("detection_mode") || data["detection_mode"] != "colors" ||
        !data.contains("valid") || data["valid"] != true)
      {
        reason = "sorting requires a current valid four-color status";
        return false;
      }
      color = data["color"].get<std::string>();
      if (place_centers_.count(color) == 0) {
        reason = "detected color has no configured placement region";
        return false;
      }
      if (!requested_color.empty()) {
        if (!data.contains("target_filter_color") ||
          !data["target_filter_color"].is_string() ||
          data["target_filter_color"].get<std::string>() != requested_color ||
          !data.contains("filter_generation") ||
          !data["filter_generation"].is_number_unsigned() ||
          data["filter_generation"].get<uint64_t>() != filter_generation)
        {
          reason = "target status belongs to a different color filter generation";
          return false;
        }
        if (color != requested_color) {
          reason = "perception selected a color different from the action request";
          return false;
        }
        if (!data.contains("requested_color_physical_count") ||
          !data["requested_color_physical_count"].is_number_integer() ||
          data["requested_color_physical_count"].get<int>() != 1)
        {
          reason = "requested color target is not unique";
          return false;
        }
      }
      const double status_stamp = data["sample_stamp"].get<double>();
      if (!isFinite(status_stamp)) {
        reason = "four-color status has a nonfinite sample stamp";
        return false;
      }
      const auto matching_stamp = [status_stamp](const auto & message) {
          return std::abs(
            rclcpp::Time(message.header.stamp, RCL_ROS_TIME).seconds() -
            status_stamp) <= 0.005;
        };
      const auto pose_match = std::find_if(
        recent_target_poses_.rbegin(), recent_target_poses_.rend(), matching_stamp);
      const auto size_match = std::find_if(
        recent_target_sizes_.rbegin(), recent_target_sizes_.rend(), matching_stamp);
      if (pose_match == recent_target_poses_.rend() ||
        (use_target_size_ && size_match == recent_target_sizes_.rend()))
      {
        reason = "waiting for pose/size carrying the current color status timestamp";
        return false;
      }
      selected_pose = &*pose_match;
      if (use_target_size_) {
        selected_size = &*size_match;
      }
    }
    const rclcpp::Time sample_time(selected_pose->header.stamp, RCL_ROS_TIME);
    object_size = object_size_m_;
    if (use_target_size_) {
      const rclcpp::Time size_sample_time(selected_size->header.stamp, RCL_ROS_TIME);
      if (std::abs((size_sample_time - sample_time).seconds()) > 0.02) {
        reason = "target pose and object size timestamps do not match";
        return false;
      }
      object_size = {
        selected_size->vector.x,
        selected_size->vector.y,
        selected_size->vector.z};
    }
    const double sample_age = (now - sample_time).seconds();
    if (!isFinite(sample_age) || sample_age < -0.1 ||
      sample_age > target_message_timeout_s_)
    {
      std::ostringstream message;
      message << "stable target sample is stale: age_s=" << sample_age;
      reason = message.str();
      return false;
    }
    const auto & point = selected_pose->pose.position;
    if (point.x < target_x_bounds_[0] || point.x > target_x_bounds_[1] ||
      point.y<target_y_bounds_[0] || point.y> target_y_bounds_[1] ||
      point.z<target_z_bounds_[0] || point.z> target_z_bounds_[1])
    {
      reason = "target is outside configured first-grasp bounds";
      return false;
    }
    if (use_target_yaw_) {
      const auto object_orientation = normalizedQuaternion(selected_pose->pose.orientation);
      geometry_msgs::msg::Vector3 object_z;
      object_z.z = 1.0;
      const auto object_z_base = rotate(object_orientation, object_z);
      const double tilt = std::acos(clamp(object_z_base.z, -1.0, 1.0));
      if (!isFinite(tilt) || tilt > maximum_target_tilt_rad_) {
        reason = "target orientation is not a horizontal top-surface yaw";
        return false;
      }
    }
    const double cube_plane_gap = point.z - object_size[2] - ground_plane_z_m_;
    if (cube_plane_gap < 0.0 || cube_plane_gap > maximum_cube_plane_gap_m_) {
      std::ostringstream message;
      message << "current target is not on the configured table plane: gap_m="
              << cube_plane_gap;
      reason = message.str();
      return false;
    }
    if (sort_enabled_ || automatic_task_) {
      if (std::abs(object_size[0] - object_size[1]) > 0.003 ||
        std::abs(object_size[1] - object_size[2]) > 0.003)
      {
        reason = "four-color sorting currently accepts only equal-size cubes";
        return false;
      }
    }
    result = *selected_pose;
    return true;
  }

  bool callPerceptionService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & operation, std::string & reason)
  {
    if (!client->wait_for_service(1s)) {
      reason = operation + " service is unavailable";
      return false;
    }
    auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(3s) != std::future_status::ready) {
      reason = operation + " service timed out";
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      reason = operation + " refused: " + response->message;
      return false;
    }
    recordEvent(operation, response->message);
    return true;
  }

  bool currentActionableCount(int & count) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((node_->now() - target_status_received_).seconds() > target_message_timeout_s_) {
      return false;
    }
    const auto data = nlohmann::json::parse(latest_target_status_, nullptr, false);
    if (data.is_discarded() || !data.is_object() ||
      !data.contains("detection_mode") || data["detection_mode"] != "colors" ||
      !data.contains("actionable_cube_count") ||
      !data["actionable_cube_count"].is_number_integer())
    {
      return false;
    }
    count = data["actionable_cube_count"].get<int>();
    return count >= 0;
  }

  bool assignPlacementSlot(const std::string & color, std::string & reason)
  {
    if (!task_sort_enabled_) {
      locked_place_center_.clear();
      return true;
    }
    if (!batch_mode_) {
      locked_place_center_ = place_centers_.at(color);
      return true;
    }
    const auto & slots = place_slots_.at(color);
    const std::size_t used = place_slot_use_count_[color];
    if (used >= slots.size() / 3) {
      reason = "no unused placement slot remains for color " + color;
      return false;
    }
    locked_place_center_ = {
      slots[used * 3], slots[used * 3 + 1], slots[used * 3 + 2]};
    return true;
  }

  bool lockTargetSnapshot(
    const geometry_msgs::msg::PoseStamped & target,
    const std::vector<double> & target_object_size,
    const std::string & target_status, const std::string & target_color,
    std::string & reason)
  {
    double object_yaw = 0.0;
    double yaw_symmetry = target_yaw_symmetry_rad_;
    if (use_target_size_) {
      const double planar_aspect =
        std::max(target_object_size[0], target_object_size[1]) /
        std::min(target_object_size[0], target_object_size[1]);
      yaw_symmetry = planar_aspect <= square_aspect_ratio_max_ ? kPi * 0.5 : kPi;
    }
    try {
      object_yaw = quaternionYaw(target.pose.orientation);
    } catch (const std::exception & error) {
      reason = std::string("invalid target yaw: ") + error.what();
      return false;
    }
    if (!assignPlacementSlot(target_color, reason)) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    locked_target_ = target;
    locked_target_status_ = target_status;
    locked_color_ = target_color;
    locked_object_size_m_ = target_object_size;
    locked_grasp_orientation_ = grasp_orientation_;
    locked_object_yaw_rad_ = object_yaw;
    locked_applied_yaw_rad_ = 0.0;
    locked_yaw_symmetry_rad_ = yaw_symmetry;
    descend_authorized_ = batch_mode_ || continuous_requested_ || automatic_task_;
    grip_confirmed_ = batch_mode_ || continuous_requested_ || automatic_task_;
    arm_motion_started_.store(false);
    cycle_failure_skippable_ = false;
    state_ = PLANNING;
    state_detail_ = batch_mode_ ?
      "batch selected " + target_color + " target" :
      "start accepted; copied the current stable P3 target";
    return true;
  }

  bool ignoredBatchTarget(const std::string & color, const geometry_msgs::msg::Point & point)
  {
    for (auto & ignored : ignored_batch_targets_) {
      const double xy_distance = std::hypot(
        point.x - ignored.center.x, point.y - ignored.center.y);
      if (color == ignored.color && xy_distance <= batch_target_match_distance_m_) {
        if (ignored.completed_source && !ignored.observed_after_cycle) {
          ignored.observed_after_cycle = true;
          std::lock_guard<std::mutex> lock(mutex_);
          if (batch_succeeded_ > 0) {
            --batch_succeeded_;
          }
          ++batch_skipped_;
          state_detail_ = "a previously handled source target is still visible; skipping it";
        }
        return true;
      }
    }
    return false;
  }

  static bool batchSnapshotFailureSkippable(const std::string & reason)
  {
    for (const auto & prefix : {
      "detected color has no configured placement region",
      "target is outside configured first-grasp bounds",
      "target orientation is not a horizontal top-surface yaw",
      "current target is not on the configured table plane",
      "four-color sorting currently accepts only equal-size cubes"
    })
    {
      if (reason.rfind(prefix, 0) == 0) {
        return true;
      }
    }
    return false;
  }

  bool waitForBatchTarget(std::string & reason)
  {
    std::optional<std::chrono::steady_clock::time_point> empty_since;
    std::ostringstream scan_detail;
    scan_detail << "arm is home; waiting for the next stable target or "
                << batch_empty_timeout_s_ << " seconds of no actionable cubes";
    setState(BATCH_SCANNING, scan_detail.str());
    while (rclcpp::ok() && !canceled_.load()) {
      geometry_msgs::msg::PoseStamped target;
      std::vector<double> target_object_size;
      std::string target_status;
      std::string target_color;
      std::string snapshot_reason;
      if (targetSnapshot(
          target, target_object_size, target_status, target_color, snapshot_reason))
      {
        empty_since.reset();
        if (ignoredBatchTarget(target_color, target.pose.position)) {
          if (!callPerceptionService(
              perception_skip_client_, "skip_ignored_sort_target", reason))
          {
            return false;
          }
          continue;
        }
        if (!lockTargetSnapshot(
            target, target_object_size, target_status, target_color, reason))
        {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            ++batch_skipped_;
          }
          recordEvent("batch_target_skipped", reason);
          if (!callPerceptionService(
              perception_skip_client_, "skip_unplaceable_sort_target", reason))
          {
            return false;
          }
          continue;
        }
        publishState();
        return true;
      }
      if (batchSnapshotFailureSkippable(snapshot_reason)) {
        empty_since.reset();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++batch_skipped_;
        }
        recordEvent("batch_target_skipped", snapshot_reason);
        if (!callPerceptionService(
            perception_skip_client_, "skip_invalid_sort_target", reason))
        {
          return false;
        }
        continue;
      }

      int actionable_count = -1;
      if (currentActionableCount(actionable_count)) {
        if (actionable_count == 0) {
          if (!empty_since) {
            empty_since = std::chrono::steady_clock::now();
          }
          const double empty_for = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - *empty_since).count();
          if (empty_for >= batch_empty_timeout_s_) {
            batch_empty_confirmed_ = true;
            reason = "no actionable cube was observed for " +
              std::to_string(batch_empty_timeout_s_) + " seconds";
            return false;
          }
        } else {
          empty_since.reset();
        }
      } else {
        // Camera/TF/stationarity errors are not an empty scene. Keep waiting
        // so a perception outage cannot be reported as successful clearing.
        empty_since.reset();
      }
      std::this_thread::sleep_for(100ms);
    }
    reason = canceled_.load() ? "batch canceled" : "ROS shutdown";
    return false;
  }

  bool stateAllowsStart(State & state, std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = state_;
    }
    if (worker_running_.load()) {
      reason = "previous task worker is still finishing";
      return false;
    }
    if (state != WAITING_TARGET && state != RETRYABLE_FAILED &&
      state != PLAN_ONLY_COMPLETE && state != COMPLETE && state != SORT_COMPLETE &&
      state != BATCH_COMPLETE && state != BATCH_COMPLETE_WITH_SKIPS)
    {
      reason = "Task is not idle; current state=" + stateName(state);
      return false;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    return true;
  }

  bool clearPreviousCubeScene(std::string & reason)
  {
    if (planning_scene_.getAttachedObjects({cube_id_}).count(cube_id_) != 0) {
      if (!planning_scene_.applyAttachedCollisionObject(attachedCube(false))) {
        reason = "failed to remove the previous trial's attached cube";
        return false;
      }
    }
    if (planning_scene_.getAttachedObjects({cube_id_}).count(cube_id_) != 0) {
      reason = "the previous trial's cube is still attached in the planning scene";
      return false;
    }

    moveit_msgs::msg::CollisionObject remove;
    remove.header.frame_id = base_frame_;
    remove.id = cube_id_;
    remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!planning_scene_.applyCollisionObject(remove)) {
      reason = "failed to remove the previous trial's world cube";
      return false;
    }
    if (planning_scene_.getObjects({cube_id_}).count(cube_id_) != 0) {
      reason = "the previous trial's world cube is still present in the planning scene";
      return false;
    }
    return true;
  }

  bool archivePreviousSortedCubeScene(std::string & reason)
  {
    if (planning_scene_.getAttachedObjects({cube_id_}).count(cube_id_) != 0) {
      reason = "previously sorted cube is still attached in the planning scene";
      return false;
    }
    const std::string placed_id =
      cube_id_ + "_sorted_" + run_directory_.filename().string();
    if (run_directory_.empty()) {
      reason = "previous sorted run has no evidence directory";
      return false;
    }
    const auto previous = planning_scene_.getObjects({cube_id_});
    if (planning_scene_.getObjects({placed_id}).count(placed_id) == 0) {
      if (previous.count(cube_id_) == 0) {
        reason = "previously sorted cube is missing from the planning scene";
        return false;
      }
      auto archived = previous.at(cube_id_);
      archived.id = placed_id;
      archived.operation = moveit_msgs::msg::CollisionObject::ADD;
      if (!planning_scene_.applyCollisionObject(archived) ||
        planning_scene_.getObjects({placed_id}).count(placed_id) == 0)
      {
        reason = "failed to retain the previous sorted cube as an obstacle";
        return false;
      }
    }
    if (previous.count(cube_id_) != 0) {
      moveit_msgs::msg::CollisionObject remove;
      remove.header.frame_id = base_frame_;
      remove.id = cube_id_;
      remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!planning_scene_.applyCollisionObject(remove) ||
        planning_scene_.getObjects({cube_id_}).count(cube_id_) != 0)
      {
        reason = "failed to free the active cube ID for the next sort";
        return false;
      }
    }
    recordEvent("sorted_cube_archived", "obstacle_id=" + placed_id);
    return true;
  }

  std::vector<GraspCandidate> graspCandidates() const
  {
    if (!use_target_yaw_) {
      return {{0.0, grasp_orientation_}};
    }
    std::vector<GraspCandidate> candidates;
    const int symmetry_count = static_cast<int>(std::lround(2.0 * kPi / locked_yaw_symmetry_rad_));
    for (int equivalent = 0; equivalent < symmetry_count; ++equivalent) {
      const double raw_yaw = locked_object_yaw_rad_ + target_yaw_offset_rad_ +
        equivalent * locked_yaw_symmetry_rad_;
      const double candidate_yaw = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));
      const auto candidate = normalizedQuaternion(
        multiply(yawQuaternion(candidate_yaw), grasp_orientation_));
      candidates.push_back({candidate_yaw, candidate});
    }
    return candidates;
  }

  geometry_msgs::msg::Pose approachPose() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = locked_target_.pose.position;
    pose.position.z += approach_clearance_m_;
    pose.orientation = locked_grasp_orientation_;
    return pose;
  }

  geometry_msgs::msg::Pose graspPose() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = locked_target_.pose.position;
    pose.position.z -= grasp_depth_m_;
    pose.orientation = locked_grasp_orientation_;
    return pose;
  }

  geometry_msgs::msg::Pose liftPose() const
  {
    auto pose = graspPose();
    pose.position.z += lift_distance_m_;
    return pose;
  }

  geometry_msgs::msg::Pose placePose() const
  {
    auto pose = graspPose();
    const auto & center = locked_place_center_;
    pose.position.x = center[0];
    pose.position.y = center[1];
    // The attached cube center is grasp_depth - height/2 above the TCP.
    // Thus this puts its bottom 5 mm above the projected paper plane.
    pose.position.z = center[2] + release_bottom_gap_m_ +
      locked_object_size_m_[2] - grasp_depth_m_;
    return pose;
  }

  geometry_msgs::msg::Pose placeApproachPose() const
  {
    auto pose = placePose();
    pose.position.z += lift_distance_m_;
    return pose;
  }

  moveit_msgs::msg::CollisionObject placedCubeObject() const
  {
    auto pose = cubeObject().primitive_poses.front();
    const auto & center = locked_place_center_;
    pose.position.x = center[0];
    pose.position.y = center[1];
    pose.position.z = center[2] + release_bottom_gap_m_ +
      locked_object_size_m_[2] * 0.5;
    // After opening, the cube settles onto the paper plane.
    pose.position.z -= release_bottom_gap_m_;
    return makeBox(cube_id_, base_frame_, locked_object_size_m_, pose);
  }

  moveit_msgs::msg::CollisionObject cubeObject() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = locked_target_.pose.position;
    pose.position.z -= locked_object_size_m_[2] * 0.5;
    pose.orientation = locked_target_.pose.orientation;
    const double norm = std::sqrt(
      pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
    if (norm < 1e-9) {
      pose.orientation.w = 1.0;
    }
    return makeBox(cube_id_, base_frame_, locked_object_size_m_, pose);
  }

  bool applyWorldScene(std::string & reason)
  {
    geometry_msgs::msg::Pose plane_pose;
    plane_pose.position.z = ground_plane_z_m_ - ground_plane_contact_epsilon_m_ -
      ground_plane_thickness_m_ * 0.5;
    plane_pose.orientation.w = 1.0;
    const double cube_bottom = locked_target_.pose.position.z - locked_object_size_m_[2];
    const double cube_plane_gap = cube_bottom - ground_plane_z_m_;
    if (cube_plane_gap < 0.0 || cube_plane_gap > maximum_cube_plane_gap_m_) {
      std::ostringstream stream;
      stream << "visual box bottom must be on/above the RViz plane and within "
             << maximum_cube_plane_gap_m_ * 1000.0 << " mm; gap="
             << cube_plane_gap * 1000.0 << " mm";
      reason = stream.str();
      return false;
    }
    for (const auto & pose : {approachPose(), graspPose(), liftPose()}) {
      if (pose.position.z < ground_plane_z_m_) {
        reason = "a task TCP target is below the configured RViz ground plane";
        return false;
      }
    }
    if (task_sort_enabled_ && (placePose().position.z < ground_plane_z_m_ ||
      placeApproachPose().position.z < ground_plane_z_m_))
    {
      reason = "place TCP target is below the ground plane";
      return false;
    }
    const auto plane = makeBox(
      ground_plane_id_, base_frame_,
      {ground_plane_xy_size_[0], ground_plane_xy_size_[1], ground_plane_thickness_m_},
      plane_pose);
    if (!planning_scene_.applyCollisionObjects({plane, cubeObject()})) {
      reason = "MoveIt rejected RViz-ground/cube collision objects";
      return false;
    }
    recordEvent(
      "scene_applied",
      "RViz XY plane converted to a collision slab below Z=" +
      std::to_string(ground_plane_z_m_) + "; visual target cube added");
    return true;
  }

  moveit_msgs::msg::AttachedCollisionObject attachedCube(bool add) const
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tip_frame_;
    attached.touch_links = touch_links_;
    attached.object.id = cube_id_;
    attached.object.operation = add ?
      moveit_msgs::msg::CollisionObject::ADD : moveit_msgs::msg::CollisionObject::REMOVE;
    if (!add) {
      return attached;
    }

    const auto tip = graspPose();
    const auto world_cube = cubeObject().primitive_poses.front();
    geometry_msgs::msg::Vector3 world_delta;
    world_delta.x = world_cube.position.x - tip.position.x;
    world_delta.y = world_cube.position.y - tip.position.y;
    world_delta.z = world_cube.position.z - tip.position.z;
    const auto relative_delta = rotate(conjugate(tip.orientation), world_delta);

    geometry_msgs::msg::Pose relative_pose;
    relative_pose.position.x = relative_delta.x;
    relative_pose.position.y = relative_delta.y;
    relative_pose.position.z = relative_delta.z;
    relative_pose.orientation = multiply(conjugate(tip.orientation), world_cube.orientation);
    attached.object.header.frame_id = tip_frame_;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.assign(
      locked_object_size_m_.begin(), locked_object_size_m_.end());
    attached.object.primitives.push_back(primitive);
    attached.object.primitive_poses.push_back(relative_pose);
    return attached;
  }

  bool attachCube(std::string & reason)
  {
    if (!planning_scene_.applyAttachedCollisionObject(attachedCube(true))) {
      reason = "MoveIt rejected attached target cube";
      return false;
    }
    if (planning_scene_.getAttachedObjects({cube_id_}).count(cube_id_) != 1) {
      reason = "target cube attachment was not observable in the planning scene";
      return false;
    }
    recordEvent("cube_attached", "touch permission limited to configured finger links");
    return true;
  }

  bool restoreWorldCube(std::string & reason)
  {
    if (planning_scene_.getAttachedObjects({cube_id_}).count(cube_id_) != 0) {
      if (!planning_scene_.applyAttachedCollisionObject(attachedCube(false))) {
        reason = "failed to remove preview attachment";
        return false;
      }
    }
    if (!planning_scene_.applyCollisionObject(cubeObject())) {
      reason = "failed to restore cube after preview attachment";
      return false;
    }
    return true;
  }

  bool releaseCubeScene(std::string & reason)
  {
    if (!planning_scene_.applyAttachedCollisionObject(attachedCube(false))) {
      reason = "failed to detach cube after opening gripper";
      return false;
    }
    if (planning_scene_.getAttachedObjects({cube_id_}).count(cube_id_) != 0) {
      reason = "cube remained attached after release";
      return false;
    }
    if (!planning_scene_.applyCollisionObject(placedCubeObject())) {
      reason = "failed to add released cube at region center";
      return false;
    }
    recordEvent("cube_released_scene", "color=" + locked_color_);
    return true;
  }

  moveit::core::RobotStatePtr currentStartState(bool force_pre_open)
  {
    auto state = move_group_->getCurrentState(state_timeout_s_);
    if (!state) {
      throw std::runtime_error("MoveIt current robot state is unavailable");
    }
    if (force_pre_open) {
      const auto & variable_names = state->getRobotModel()->getVariableNames();
      if (std::find(variable_names.begin(), variable_names.end(), gripper_joint_) ==
        variable_names.end())
      {
        throw std::runtime_error("robot model has no configured gripper joint");
      }
      state->setVariablePosition(gripper_joint_, pre_open_position_m_);
      state->update();
    }
    return state;
  }

  bool waitForGripperState(double expected, const std::string & phase)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(state_timeout_s_);
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok() && !canceled_.load()) {
      auto state = move_group_->getCurrentState(0.25);
      if (state) {
        const auto & names = state->getRobotModel()->getVariableNames();
        if (std::find(names.begin(), names.end(), gripper_joint_) != names.end()) {
          const double value = state->getVariablePosition(gripper_joint_);
          if (isFinite(value) && std::abs(value - expected) <= gripper_state_tolerance_m_) {
            recordEvent(
              "gripper_state_" + phase,
              "estimated_position_m=" + std::to_string(value));
            return true;
          }
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    fail(
      "gripper action succeeded but MoveIt joint state did not reach " +
      std::to_string(expected) + " m for " + phase);
    return false;
  }

  moveit::core::RobotStatePtr endState(
    const moveit::core::RobotState & start,
    const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    robot_trajectory::RobotTrajectory converted(move_group_->getRobotModel(), arm_group_);
    converted.setRobotTrajectoryMsg(start, trajectory);
    if (converted.empty()) {
      throw std::runtime_error("planned trajectory is empty");
    }
    return std::make_shared<moveit::core::RobotState>(converted.getLastWayPoint());
  }

  PlannedMotion planPose(
    const std::string & name, const moveit::core::RobotStatePtr & start,
    const geometry_msgs::msg::Pose & target)
  {
    move_group_->setStartState(*start);
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target, tip_frame_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto code = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    recordReturn("plan_" + name, code.val);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error(
              name + " planning failed with MoveIt code " +
              std::to_string(code.val));
    }
    PlannedMotion motion;
    motion.name = name;
    motion.trajectory = plan.trajectory_;
    moveit::core::robotStateToRobotStateMsg(*start, motion.start_state);
    motion.end_state = endState(*start, motion.trajectory);
    saveTrajectory(motion);
    return motion;
  }

  PlannedMotion planHome(
    const std::string & name, const moveit::core::RobotStatePtr & start)
  {
    move_group_->setStartState(*start);
    std::vector<double> home_joints;
    locked_home_state_->copyJointGroupPositions(arm_group_, home_joints);
    move_group_->clearPoseTargets();
    move_group_->setJointValueTarget(home_joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto code = move_group_->plan(plan);
    recordReturn("plan_" + name, code.val);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error(
              name + " planning failed with MoveIt code " +
              std::to_string(code.val));
    }
    PlannedMotion motion;
    motion.name = name;
    motion.trajectory = plan.trajectory_;
    moveit::core::robotStateToRobotStateMsg(*start, motion.start_state);
    motion.end_state = endState(*start, motion.trajectory);
    saveTrajectory(motion);
    return motion;
  }

  PlannedMotion planCartesian(
    const std::string & name, const moveit::core::RobotStatePtr & start,
    const geometry_msgs::msg::Pose & target)
  {
    move_group_->setStartState(*start);
    move_group_->setPoseReferenceFrame(base_frame_);
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error;
    const double fraction = move_group_->computeCartesianPath(
      {target}, cartesian_step_m_, cartesian_jump_threshold_, trajectory, true, &error);
    double maximum_joint_step = 0.0;
    const auto & points = trajectory.joint_trajectory.points;
    for (std::size_t point_index = 1; point_index < points.size(); ++point_index) {
      const auto & previous = points[point_index - 1].positions;
      const auto & current = points[point_index].positions;
      if (previous.size() != current.size()) {
        throw std::runtime_error(name + " Cartesian trajectory has inconsistent joint dimensions");
      }
      for (std::size_t joint_index = 0; joint_index < current.size(); ++joint_index) {
        maximum_joint_step = std::max(
          maximum_joint_step, std::abs(current[joint_index] - previous[joint_index]));
      }
    }
    {
      std::ostringstream detail;
      detail << std::setprecision(9) << "fraction=" << fraction << ",moveit_code=" << error.val
             << ",maximum_joint_step_rad=" << maximum_joint_step;
      recordEvent("plan_" + name, detail.str());
    }
    if (error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      fraction < minimum_cartesian_fraction_)
    {
      std::ostringstream message;
      message << name << " Cartesian path incomplete: fraction=" << fraction
              << ", code=" << error.val;
      throw std::runtime_error(message.str());
    }
    if (points.size() < 2 || maximum_joint_step > maximum_cartesian_joint_step_rad_) {
      std::ostringstream message;
      message << name << " Cartesian joint-step gate failed: points=" << points.size()
              << ", maximum_step_rad=" << maximum_joint_step
              << ", limit_rad=" << maximum_cartesian_joint_step_rad_;
      throw std::runtime_error(message.str());
    }

    robot_trajectory::RobotTrajectory timed(move_group_->getRobotModel(), arm_group_);
    timed.setRobotTrajectoryMsg(*start, trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(timed, velocity_scale_, acceleration_scale_)) {
      throw std::runtime_error(name + " Cartesian time parameterization failed");
    }
    timed.getRobotTrajectoryMsg(trajectory);

    PlannedMotion motion;
    motion.name = name;
    motion.trajectory = trajectory;
    moveit::core::robotStateToRobotStateMsg(*start, motion.start_state);
    motion.end_state = endState(*start, motion.trajectory);
    saveTrajectory(motion);
    return motion;
  }

  JointPathMetrics jointPathMetrics(const PlannedMotion & motion) const
  {
    const auto & trajectory = motion.trajectory.joint_trajectory;
    const auto & names = trajectory.joint_names;
    const auto & state = motion.start_state.joint_state;
    if (names.empty() || trajectory.points.empty()) {
      throw std::runtime_error(motion.name + " has no joint trajectory");
    }
    std::vector<double> previous;
    previous.reserve(names.size());
    for (const auto & name : names) {
      const auto entry = std::find(state.name.begin(), state.name.end(), name);
      if (entry == state.name.end()) {
        throw std::runtime_error(motion.name + " start state is missing " + name);
      }
      const std::size_t index = std::distance(state.name.begin(), entry);
      if (index >= state.position.size() || !isFinite(state.position[index])) {
        throw std::runtime_error(motion.name + " start state has invalid " + name);
      }
      previous.push_back(state.position[index]);
    }
    JointPathMetrics result;
    result.joint_names = names;
    result.joint_travel_rad.assign(names.size(), 0.0);
    for (const auto & point : trajectory.points) {
      if (point.positions.size() != names.size()) {
        throw std::runtime_error(motion.name + " has inconsistent joint dimensions");
      }
      double squared_segment_length = 0.0;
      for (std::size_t index = 0; index < names.size(); ++index) {
        const double position = point.positions[index];
        if (!isFinite(position)) {
          throw std::runtime_error(motion.name + " has nonfinite joint positions");
        }
        const double step = std::abs(position - previous[index]);
        result.joint_travel_rad[index] += step;
        squared_segment_length += step * step;
        previous[index] = position;
      }
      result.path_length_rad += std::sqrt(squared_segment_length);
    }
    return result;
  }

  JointPathMetrics combinedJointPathMetrics(const std::vector<PlannedMotion> & motions) const
  {
    JointPathMetrics total;
    for (const auto & motion : motions) {
      const auto part = jointPathMetrics(motion);
      if (total.joint_names.empty()) {
        total.joint_names = part.joint_names;
        total.joint_travel_rad.assign(part.joint_travel_rad.size(), 0.0);
      } else if (part.joint_names != total.joint_names) {
        throw std::runtime_error("planned motions use inconsistent joint ordering");
      }
      total.path_length_rad += part.path_length_rad;
      for (std::size_t index = 0; index < part.joint_travel_rad.size(); ++index) {
        total.joint_travel_rad[index] += part.joint_travel_rad[index];
      }
    }
    return total;
  }

  std::vector<PlannedMotion> previewAll()
  {
    std::string reason;
    if (!applyWorldScene(reason)) {
      throw std::runtime_error(reason);
    }
    auto start = currentStartState(!execute_);
    const auto candidates = graspCandidates();
    std::vector<PlannedMotion> best_motions;
    GraspCandidate best_candidate;
    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (canceled_.load()) {
        throw std::runtime_error("candidate planning canceled");
      }
      const auto & candidate = candidates[index];
      locked_grasp_orientation_ = candidate.orientation;
      locked_applied_yaw_rad_ = candidate.yaw_rad;
      const std::string prefix = "candidate_" + std::to_string(index) + "_";
      bool attachment_attempted = false;
      try {
        std::vector<PlannedMotion> motions;
        motions.push_back(planPose(prefix + "approach", start, approachPose()));
        motions.push_back(
          planCartesian(
            prefix + "descend", motions.back().end_state, graspPose()));
        attachment_attempted = true;
        if (!attachCube(reason)) {
          throw std::runtime_error(reason);
        }
        motions.push_back(
          planCartesian(
            prefix + "lift", motions.back().end_state, liftPose()));
        if (task_sort_enabled_) {
          motions.push_back(
            planPose(
              prefix + "place_approach", motions.back().end_state,
              placeApproachPose()));
          motions.push_back(
            planCartesian(
              prefix + "place_descend", motions.back().end_state, placePose()));
          if (!releaseCubeScene(reason)) {
            throw std::runtime_error(reason);
          }
          auto open_state = std::make_shared<moveit::core::RobotState>(
            *motions.back().end_state);
          open_state->setVariablePosition(gripper_joint_, pre_open_position_m_);
          open_state->update();
          motions.push_back(
            planCartesian(
              prefix + "place_retreat", open_state,
              placeApproachPose()));
          motions.push_back(
            planHome(
              prefix + "return_home", motions.back().end_state));
        }
        if (!restoreWorldCube(reason)) {
          throw std::runtime_error(reason);
        }
        attachment_attempted = false;
        const auto metrics = combinedJointPathMetrics(motions);
        // Compare equivalent grasp orientations using the standard joint-space
        // path length. All six arm joints have equal weight; joint6 is no
        // longer constrained or preferred separately.
        const double score = metrics.path_length_rad;
        std::ostringstream detail;
        detail << std::setprecision(9) << "index=" << index << ",yaw_rad=" << candidate.yaw_rad
               << ",joint_space_path_length_rad=" << score;
        for (std::size_t joint = 0; joint < metrics.joint_names.size(); ++joint) {
          detail << "," << metrics.joint_names[joint] << "_travel_rad="
                 << metrics.joint_travel_rad[joint];
        }
        recordEvent("grasp_candidate_valid", detail.str());
        if (score < best_score) {
          best_score = score;
          best_candidate = candidate;
          best_motions = std::move(motions);
        }
      } catch (const std::exception & error) {
        if (attachment_attempted) {
          std::string restore_reason;
          if (!restoreWorldCube(restore_reason)) {
            throw std::runtime_error(
                    "candidate cleanup failed: " + restore_reason + "; original: " + error.what());
          }
        }
        recordEvent(
          "grasp_candidate_rejected",
          "index=" + std::to_string(index) + ",yaw_rad=" +
          std::to_string(candidate.yaw_rad) + ",reason=" + error.what());
      }
    }
    if (best_motions.empty()) {
      throw std::runtime_error("no equivalent grasp has a complete collision-free trajectory");
    }
    locked_grasp_orientation_ = best_candidate.orientation;
    locked_applied_yaw_rad_ = best_candidate.yaw_rad;
    best_motions[0].name = "approach";
    best_motions[1].name = "descend";
    best_motions[2].name = "lift";
    if (task_sort_enabled_) {
      best_motions[3].name = "place_approach";
      best_motions[4].name = "place_descend";
      best_motions[5].name = "place_retreat";
      best_motions[6].name = "return_home";
    }
    for (const auto & motion : best_motions) {
      saveTrajectory(motion);
    }
    recordEvent(
      "grasp_candidate_selected", "yaw_rad=" +
      std::to_string(best_candidate.yaw_rad) + ",joint_space_path_length_rad=" +
      std::to_string(best_score));
    publishPreview(best_motions);
    recordEvent(
      "preview_complete", task_sort_enabled_ ?
      "grasp, place, release, retreat and return-home paths planned" :
      "approach, full descend and attached full lift planned");
    return best_motions;
  }

  void publishPreview(const std::vector<PlannedMotion> & motions)
  {
    if (motions.empty()) {
      return;
    }
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = move_group_->getRobotModel()->getName();
    display.trajectory_start = motions.front().start_state;
    for (const auto & motion : motions) {
      display.trajectory.push_back(motion.trajectory);
    }
    display_publisher_->publish(display);
  }

  bool executeMotion(const PlannedMotion & motion, const geometry_msgs::msg::Pose & expected)
  {
    if (canceled_.load()) {
      return false;
    }
    try {
      const auto metrics = jointPathMetrics(motion);
      std::ostringstream detail;
      detail << std::setprecision(9) << "segment=" << motion.name
             << ",joint_space_path_length_rad=" << metrics.path_length_rad;
      for (std::size_t joint = 0; joint < metrics.joint_names.size(); ++joint) {
        detail << "," << metrics.joint_names[joint] << "_travel_rad="
               << metrics.joint_travel_rad[joint];
      }
      recordEvent("trajectory_execution_metrics", detail.str());
      arm_motion_started_.store(true);
      const auto code = move_group_->execute(motion.trajectory);
      recordReturn("execute_" + motion.name, code.val);
      if (code != moveit::core::MoveItErrorCode::SUCCESS) {
        fail(motion.name + " execution failed with MoveIt code " + std::to_string(code.val));
        return false;
      }
    } catch (const std::exception & error) {
      fail(motion.name + " trajectory validation failed: " + error.what());
      return false;
    }
    const auto actual = move_group_->getCurrentPose(tip_frame_).pose;
    const double position_error = positionDistance(actual.position, expected.position);
    const double orientation_error = quaternionDistance(actual.orientation, expected.orientation);
    std::ostringstream detail;
    detail << std::setprecision(9) << "position_error_m=" << position_error
           << ",orientation_error_rad=" << orientation_error;
    recordEvent("arrival_" + motion.name, detail.str());
    if (position_error > arrival_position_tolerance_m_ ||
      orientation_error > arrival_orientation_tolerance_rad_)
    {
      fail(motion.name + " returned success but did not pass arrival tolerance: " + detail.str());
      return false;
    }
    return true;
  }

  bool commandGripper(double position, const std::string & name)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::duration<double>(action_timeout_s_)))
    {
      fail("gripper action server unavailable for " + name);
      return false;
    }
    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = 0.0;
    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::duration<double>(action_timeout_s_)) !=
      std::future_status::ready)
    {
      fail("gripper goal acknowledgement timed out for " + name);
      return false;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      fail("gripper rejected " + name);
      return false;
    }
    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::duration<double>(action_timeout_s_)) !=
      std::future_status::ready)
    {
      gripper_client_->async_cancel_goal(goal_handle);
      fail("gripper result timed out for " + name);
      return false;
    }
    const auto wrapped = result_future.get();
    const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result && wrapped.result->reached_goal;
    std::ostringstream detail;
    detail << "result_code=" << static_cast<int>(wrapped.code)
           << ",reached_goal=" << (wrapped.result && wrapped.result->reached_goal)
           << ",requested_position_m=" << position;
    if (wrapped.result) {
      detail << ",reported_position_m=" << wrapped.result->position
             << ",stalled=" << wrapped.result->stalled;
    }
    recordEvent("gripper_" + name, detail.str());
    if (!success) {
      fail("gripper " + name + " failed: " + detail.str());
      return false;
    }
    return true;
  }

  void start(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response,
    bool continuous)
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    std::string reason;
    if (!p6Gate(reason)) {
      response->success = false;
      response->message = "Start refused: " + reason;
      return;
    }
    State prior_state;
    if (!stateAllowsStart(prior_state, reason)) {
      response->success = false;
      response->message = reason;
      return;
    }
    if (prior_state == COMPLETE && !clearPreviousCubeScene(reason)) {
      response->success = false;
      response->message = "Next trial refused: " + reason;
      return;
    }
    if (prior_state == SORT_COMPLETE && !archivePreviousSortedCubeScene(reason)) {
      response->success = false;
      response->message = "Next trial refused: " + reason;
      return;
    }
    moveit::core::RobotStatePtr home_state;
    geometry_msgs::msg::Pose home_pose;
    try {
      home_state = currentStartState(false);
      home_pose = move_group_->getCurrentPose(tip_frame_).pose;
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("Start refused: cannot snapshot home: ") + error.what();
      return;
    }

    batch_mode_ = continuous && sort_enabled_ && execute_;
    continuous_requested_ = continuous;
    automatic_task_ = false;
    task_sort_enabled_ = sort_enabled_;
    if (batch_mode_ && !callPerceptionService(
        perception_reset_client_, "reset_sort_batch", reason))
    {
      response->success = false;
      response->message = "Batch start refused: " + reason;
      batch_mode_ = false;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ != WAITING_TARGET && state_ != RETRYABLE_FAILED &&
        state_ != PLAN_ONLY_COMPLETE && state_ != COMPLETE && state_ != SORT_COMPLETE &&
        state_ != BATCH_COMPLETE && state_ != BATCH_COMPLETE_WITH_SKIPS)
      {
        response->success = false;
        response->message = "Task is not idle; current state=" + stateName(state_);
        return;
      }
      locked_home_state_ = home_state;
      locked_home_pose_ = home_pose;
      canceled_.store(false);
      batch_succeeded_ = 0;
      batch_skipped_ = 0;
      ignored_batch_targets_.clear();
      state_ = batch_mode_ ? BATCH_SCANNING : WAITING_TARGET;
      state_detail_ = batch_mode_ ?
        "batch accepted; perception reset and arm home recorded" :
        "preparing current stable target";
    }

    if (!batch_mode_) {
      geometry_msgs::msg::PoseStamped target;
      std::vector<double> target_object_size;
      std::string target_status;
      std::string target_color;
      if (!targetSnapshot(target, target_object_size, target_status, target_color, reason) ||
        !lockTargetSnapshot(
          target, target_object_size, target_status, target_color, reason))
      {
        response->success = false;
        response->message = "Start refused now: " + reason;
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_running_.store(true);
      worker_ = std::thread(
        [this]() {
          run();
          worker_running_.store(false);
        });
    }
    response->success = true;
    if (batch_mode_) {
      std::ostringstream message;
      message << "Accepted color-sort batch; it ends after " << batch_empty_timeout_s_
              << " seconds with no actionable cube.";
      response->message = message.str();
    } else {
      response->message = execute_ ?
        (continuous ?
        "Accepted continuous grasp; monitor /fr3c_first_grasp/state for completion." :
        "Accepted current stable target copy: pre-open, preview, then approach.") :
        "Accepted current stable target copy for plan-only preview.";
    }
    publishState();
  }

  void authorizeDescend(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::string reason;
    if (!p6Gate(reason)) {
      response->success = false;
      response->message = "Descend refused: " + reason;
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != WAITING_DESCEND) {
      response->success = false;
      response->message = "Refused: current state=" + stateName(state_);
      return;
    }
    descend_authorized_ = true;
    response->success = true;
    response->message = "Descend and close authorized.";
    condition_.notify_all();
  }

  void confirmGrip(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::string reason;
    if (!p6Gate(reason)) {
      response->success = false;
      response->message = "Lift refused: " + reason;
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != WAITING_GRIP_CONFIRMATION) {
      response->success = false;
      response->message = "Refused: current state=" + stateName(state_);
      return;
    }
    grip_confirmed_ = true;
    response->success = true;
    response->message = "Manual grip confirmation accepted; attach and lift authorized.";
    condition_.notify_all();
  }

  void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    canceled_.store(true);
    move_group_->stop();
    if (gripper_client_) {
      gripper_client_->async_cancel_all_goals();
    }
    condition_.notify_all();
    setState(CANCELED, "operator cancellation; gripper command/scene attachment not reversed");
    response->success = true;
    response->message = "Cancel requested. No automatic release or retry will occur.";
  }

  void acknowledgeObjectReleased(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    if (worker_running_.load() || action_goal_claimed_.load()) {
      response->success = false;
      response->message = "Release acknowledgement refused: a task is still active.";
      return;
    }
    State current_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_state = state_;
    }
    if (current_state != COMPLETE) {
      response->success = false;
      response->message =
        "Release acknowledgement is valid only after complete_holding; current state=" +
        stateName(current_state);
      return;
    }
    std::string reason;
    uint64_t filter_generation = 0;
    if (!setPerceptionTargetColor("", filter_generation, reason)) {
      response->success = false;
      response->message =
        "Release acknowledgement could not clear the perception color filter: " + reason;
      return;
    }
    if (!clearPreviousCubeScene(reason)) {
      response->success = false;
      response->message = "Release acknowledgement failed to synchronize MoveIt: " + reason;
      return;
    }
    automatic_task_ = false;
    task_sort_enabled_ = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      locked_color_.clear();
    }
    setState(
      WAITING_TARGET,
      "operator confirmed the held object was physically released; planning scene cleared");
    response->success = true;
    response->message = "Held-object state and perception color filter generation " +
      std::to_string(filter_generation) +
      " cleared. No gripper or arm command was sent; a new Action may start.";
  }

  bool runCycle()
  {
    try {
      initializeRunLog();
      recordEvent("target_snapshot", "stable target copied; equivalent yaws will be planned");
      if (execute_ && !commandGripper(pre_open_position_m_, "pre_open")) {
        return false;
      }
      if (execute_ && !waitForGripperState(pre_open_position_m_, "pre_open")) {
        return false;
      }
      std::vector<PlannedMotion> preview;
      try {
        preview = previewAll();
      } catch (const std::exception & error) {
        cycle_failure_skippable_ = true;
        fail(error.what());
        return false;
      }
      std::ostringstream target_detail;
      target_detail << std::setprecision(9) << "locked top=["
                    << locked_target_.pose.position.x << ","
                    << locked_target_.pose.position.y << ","
                    << locked_target_.pose.position.z << "] m, object_yaw="
                    << locked_object_yaw_rad_ << " rad, applied_grasp_yaw="
                    << locked_applied_yaw_rad_ << " rad";
      recordEvent("target_locked", target_detail.str());
      RCLCPP_INFO(node_->get_logger(), "%s", target_detail.str().c_str());
      recordTarget();
      if (!execute_) {
        setState(
          PLAN_ONLY_COMPLETE, task_sort_enabled_ ?
          "full color-sort route planned; no command was sent" :
          "three motions planned; no command was sent");
        return true;
      }
      std::string p6_reason;
      if (!p6Gate(p6_reason)) {
        fail("before approach: " + p6_reason);
        return false;
      }
      recordEvent("p6_gate", "before approach passed");
      recordEvent("target_lock", "before approach uses immutable start snapshot");
      if (!executeMotion(preview.front(), approachPose())) {
        return false;
      }
      setState(WAITING_DESCEND, "inspect high-clearance alignment, then call descend_and_close");
      if (!waitForAuthorization(descend_authorized_)) {
        return false;
      }
      setState(DESCENDING, "planning from actual state, then descending and closing");
      recordEvent("target_lock", "before descend uses immutable start snapshot");
      auto descend = planCartesian("descend_execute", currentStartState(false), graspPose());
      if (!executeMotion(descend, graspPose())) {
        return false;
      }
      if (!commandGripper(close_position_m_, "close")) {
        return false;
      }
      if (!waitForGripperState(close_position_m_, "close")) {
        return false;
      }
      setState(
        WAITING_GRIP_CONFIRMATION,
        automatic_task_ ?
        "26 mm close was commanded open-loop; continuing without a grip sensor claim" :
        "26 mm was commanded open-loop; physically confirm grip before lift");
      if (!waitForAuthorization(grip_confirmed_)) {
        return false;
      }
      setState(
        LIFTING, automatic_task_ ?
        "close command completed; attaching cube in the scene and planning lift" :
        "manual grip confirmed; attaching cube and planning lift");
      recordEvent("target_lock", "before lift uses immutable start snapshot");
      std::string reason;
      if (!attachCube(reason)) {
        fail(reason);
        return false;
      }
      auto lift = planCartesian("lift_execute", currentStartState(false), liftPose());
      if (!executeMotion(lift, liftPose())) {
        return false;
      }
      if (task_sort_enabled_) {
        if (!p6Gate(p6_reason)) {
          fail("before placement: " + p6_reason);
          return false;
        }
        setState(PLACING, "moving the grasped cube to its color region");
        auto place_approach = planPose(
          "place_approach_execute", currentStartState(false), placeApproachPose());
        if (!executeMotion(place_approach, placeApproachPose())) {
          return false;
        }
        auto place_descend = planCartesian(
          "place_descend_execute", currentStartState(false), placePose());
        if (!executeMotion(place_descend, placePose())) {
          return false;
        }
        if (!commandGripper(pre_open_position_m_, "release")) {
          return false;
        }
        if (!waitForGripperState(pre_open_position_m_, "release")) {
          return false;
        }
        if (!releaseCubeScene(reason)) {
          fail(reason);
          return false;
        }
        setState(RETURNING_HOME, "retreating from the placed cube and returning to start joints");
        auto retreat = planCartesian(
          "place_retreat_execute", currentStartState(false), placeApproachPose());
        if (!executeMotion(retreat, placeApproachPose())) {
          return false;
        }
        auto home = planHome("return_home_execute", currentStartState(false));
        if (!executeMotion(home, locked_home_pose_)) {
          return false;
        }
        setState(
          SORT_COMPLETE, "cube placed in " + locked_color_ +
          " region and arm returned to start joints");
        return true;
      }
      setState(
        COMPLETE,
        "lift complete; cube remains attached in the scene; after physically placing it, "
        "a fresh stable target may start the next trial");
      return true;
    } catch (const std::exception & error) {
      fail(error.what());
      return false;
    }
  }

  void runBatch()
  {
    while (rclcpp::ok() && !canceled_.load()) {
      std::string reason;
      batch_empty_confirmed_ = false;
      if (!waitForBatchTarget(reason)) {
        if (canceled_.load()) {
          setState(CANCELED, reason);
          return;
        }
        if (!batch_empty_confirmed_) {
          fail("batch target acquisition failed: " + reason);
          return;
        }
        int succeeded;
        int skipped;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          succeeded = batch_succeeded_;
          skipped = batch_skipped_;
        }
        std::ostringstream detail;
        detail << "batch finished after " << batch_empty_timeout_s_
               << " seconds with no actionable cube; succeeded=" << succeeded
               << ", skipped=" << skipped;
        setState(
          skipped == 0 ? BATCH_COMPLETE : BATCH_COMPLETE_WITH_SKIPS,
          detail.str());
        return;
      }

      const auto source_point = locked_target_.pose.position;
      const auto source_color = locked_color_;
      const auto destination = locked_place_center_;
      if (!runCycle()) {
        State failed_state;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          failed_state = state_;
        }
        if (!cycle_failure_skippable_ || arm_motion_started_.load() ||
          failed_state != RETRYABLE_FAILED)
        {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++batch_skipped_;
        }
        recordEvent(
          "batch_target_skipped",
          "full-route planning failed before arm motion; color=" + source_color);
        if (!clearPreviousCubeScene(reason) ||
          !callPerceptionService(
            perception_skip_client_, "skip_unreachable_sort_target", reason))
        {
          fail("cannot continue batch after a skipped target: " + reason);
          return;
        }
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++batch_succeeded_;
      }
      ++place_slot_use_count_[source_color];
      ignored_batch_targets_.push_back(
        {source_color, source_point, true, false});
      geometry_msgs::msg::Point placed_top;
      placed_top.x = destination[0];
      placed_top.y = destination[1];
      placed_top.z = destination[2] + locked_object_size_m_[2];
      ignored_batch_targets_.push_back(
        {source_color, placed_top, false, false});
      if (!archivePreviousSortedCubeScene(reason)) {
        fail("cannot archive the placed cube before the next target: " + reason);
        return;
      }
    }
  }

  void run()
  {
    if (batch_mode_) {
      runBatch();
    } else {
      runCycle();
    }
  }

  bool waitForAuthorization(bool & flag)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, &flag]() {return flag || canceled_.load() || !rclcpp::ok();});
    if (canceled_.load() || !rclcpp::ok()) {
      lock.unlock();
      setState(CANCELED, "canceled while waiting; no release or retry");
      return false;
    }
    return true;
  }

  void fail(const std::string & reason)
  {
    if (canceled_.load()) {
      setState(CANCELED, reason);
      return;
    }
    move_group_->stop();
    if (arm_motion_started_.load()) {
      setState(FAILED, reason);
    } else {
      setState(
        RETRYABLE_FAILED,
        reason + "; no arm motion started, fix the condition and call /start again");
    }
    RCLCPP_ERROR(node_->get_logger(), "P5 task stopped: %s", reason.c_str());
  }

  void setState(State state, const std::string & detail)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = state;
      state_detail_ = detail;
    }
    recordEvent("state", stateName(state) + ": " + detail);
    std::shared_ptr<GoalHandlePickColor> goal_handle;
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      goal_handle = active_action_goal_;
    }
    if (goal_handle && goal_handle->is_active()) {
      auto feedback = std::make_shared<PickColor::Feedback>();
      feedback->phase = stateName(state);
      feedback->detail = detail;
      goal_handle->publish_feedback(feedback);
    }
    publishState();
  }

  void publishState()
  {
    State state;
    std::string detail;
    int batch_succeeded;
    int batch_skipped;
    std::string locked_color;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = state_;
      detail = state_detail_;
      batch_succeeded = batch_succeeded_;
      batch_skipped = batch_skipped_;
      locked_color = locked_color_;
    }
    const bool retryable =
      state == WAITING_TARGET || state == RETRYABLE_FAILED ||
      state == PLAN_ONLY_COMPLETE || state == COMPLETE || state == SORT_COMPLETE ||
      state == BATCH_COMPLETE || state == BATCH_COMPLETE_WITH_SKIPS;
    std_msgs::msg::String message;
    message.data = "{\"state\":\"" + stateName(state) + "\",\"detail\":\"" +
      jsonEscape(detail) + "\",\"execute\":" + (execute_ ? "true" : "false") +
      ",\"batch_mode\":" + (batch_mode_ ? "true" : "false") +
      ",\"batch_succeeded\":" + std::to_string(batch_succeeded) +
      ",\"batch_skipped\":" + std::to_string(batch_skipped) +
      ",\"task_mode\":\"" + (task_sort_enabled_ ? "pick_sort" : "pick_hold") + "\"" +
      ",\"target_color\":\"" + jsonEscape(locked_color) + "\"" +
      ",\"action_active\":" + (action_goal_claimed_.load() ? "true" : "false") +
      ",\"retryable\":" + (retryable ? "true" : "false") + "}";
    state_publisher_->publish(message);
  }

  void initializeRunLog()
  {
    const auto nanoseconds = node_->now().nanoseconds();
    const auto root = std::filesystem::absolute(log_directory_);
    run_directory_ = root / std::to_string(nanoseconds);
    std::filesystem::create_directories(run_directory_);
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      if (event_stream_.is_open()) {
        event_stream_.close();
      }
      event_stream_.clear();
      event_stream_.open(run_directory_ / "events.jsonl", std::ios::out | std::ios::app);
      if (!event_stream_) {
        throw std::runtime_error("cannot create P5 run evidence in " + run_directory_.string());
      }
    }
    recordEvent("run_started", execute_ ? "execution" : "plan_only");
  }

  void recordTarget()
  {
    std::string perception_status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      perception_status = locked_target_status_;
    }
    std::ofstream stream(run_directory_ / "target_and_parameters.yaml");
    stream << std::setprecision(12)
           << "frame: " << base_frame_ << "\n"
           << "target_top_xyz_m: [" << locked_target_.pose.position.x << ", "
           << locked_target_.pose.position.y << ", " << locked_target_.pose.position.z << "]\n"
           << "target_object_orientation_xyzw: [" << locked_target_.pose.orientation.x << ", "
           << locked_target_.pose.orientation.y << ", "
           << locked_target_.pose.orientation.z << ", "
           << locked_target_.pose.orientation.w << "]\n"
           << "object_long_axis_yaw_rad: " << locked_object_yaw_rad_ << "\n"
           << "applied_grasp_yaw_rad: " << locked_applied_yaw_rad_ << "\n"
           << "selected_grasp_orientation_xyzw: [" << locked_grasp_orientation_.x << ", "
           << locked_grasp_orientation_.y << ", " << locked_grasp_orientation_.z << ", "
           << locked_grasp_orientation_.w << "]\n"
           << "nominal_grasp_orientation_xyzw: [" << grasp_orientation_.x << ", "
           << grasp_orientation_.y << ", " << grasp_orientation_.z << ", "
           << grasp_orientation_.w << "]\n"
           << "use_target_yaw: " << (use_target_yaw_ ? "true" : "false") << "\n"
           << "use_target_size: " << (use_target_size_ ? "true" : "false") << "\n"
           << "target_yaw_offset_rad: " << target_yaw_offset_rad_ << "\n"
           << "selected_yaw_symmetry_rad: " << locked_yaw_symmetry_rad_ << "\n"
           << "equivalent_grasp_selection: minimum_joint_space_path_length\n"
           << "approach_clearance_m: " << approach_clearance_m_ << "\n"
           << "grasp_depth_m: " << grasp_depth_m_ << "\n"
           << "lift_distance_m: " << lift_distance_m_ << "\n"
           << "pre_open_position_m: " << pre_open_position_m_ << "\n"
           << "close_position_m: " << close_position_m_ << "\n"
           << "object_size_m: [" << locked_object_size_m_[0] << ", "
           << locked_object_size_m_[1] << ", " << locked_object_size_m_[2] << "]\n"
           << "sort_enabled: " << (task_sort_enabled_ ? "true" : "false") << "\n"
           << "sort_color: " << locked_color_ << "\n"
           << "sort_grid_file: " << sort_grid_file_ << "\n"
           << "release_bottom_gap_m: " << release_bottom_gap_m_ << "\n"
           << "place_region_center_m: [";
    if (task_sort_enabled_) {
      const auto & center = locked_place_center_;
      stream << center[0] << ", " << center[1] << ", " << center[2];
    }
    stream << "]\n"
           << "ground_plane_z_m: " << ground_plane_z_m_ << "\n"
           << "ground_plane_xy_size_m: [" << ground_plane_xy_size_[0] << ", "
           << ground_plane_xy_size_[1] << "]\n"
           << "ground_plane_thickness_m: " << ground_plane_thickness_m_ << "\n"
           << "execute: " << (execute_ ? "true" : "false") << "\n"
           << "p6_gate_file: " << p6_gate_file_ << "\n"
           << "p6_gate_max_age_s: " << p6_gate_max_age_s_ << "\n"
           << "perception_status_json: '" << perception_status << "'\n";
  }

  void recordEvent(const std::string & event, const std::string & detail)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (!event_stream_) {
      return;
    }
    event_stream_ << "{\"time_ns\":" << node_->now().nanoseconds()
                  << ",\"event\":\"" << jsonEscape(event)
                  << "\",\"detail\":\"" << jsonEscape(detail) << "\"}\n";
    event_stream_.flush();
  }

  void recordReturn(const std::string & operation, int code)
  {
    recordEvent(operation, "return_code=" + std::to_string(code));
  }

  void saveTrajectory(const PlannedMotion & motion)
  {
    if (run_directory_.empty()) {
      return;
    }
    std::ofstream stream(run_directory_ / (motion.name + "_trajectory.yaml"));
    stream << moveit_msgs::msg::to_yaml(motion.trajectory);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr perception_reset_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr perception_skip_client_;
  rclcpp::Client<fairino_msgs::srv::SetTargetColor>::SharedPtr perception_color_client_;
  rclcpp_action::Server<PickColor>::SharedPtr pick_hold_server_;
  rclcpp_action::Server<PickColor>::SharedPtr pick_sort_server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr size_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr continuous_start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr descend_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lift_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_ack_service_;
  rclcpp::CallbackGroup::SharedPtr start_callback_group_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::string arm_group_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string gripper_joint_;
  std::string gripper_action_;
  std::string target_pose_topic_;
  std::string target_valid_topic_;
  std::string target_status_topic_;
  std::string target_size_topic_;
  std::string perception_reset_service_;
  std::string perception_skip_service_;
  std::string perception_color_service_;
  std::string cube_id_;
  std::string ground_plane_id_;
  std::string log_directory_;
  std::string p6_gate_file_;
  std::string sort_grid_file_;
  std::unordered_map<std::string, std::vector<double>> place_centers_;
  std::unordered_map<std::string, std::vector<double>> place_slots_;
  std::unordered_map<std::string, std::size_t> place_slot_use_count_;
  bool sort_enabled_{false};
  double release_bottom_gap_m_{0.005};
  double batch_empty_timeout_s_{10.0};
  double batch_target_match_distance_m_{0.025};
  bool execute_{false};
  double approach_clearance_m_{0.010};
  double grasp_depth_m_{0.015};
  double lift_distance_m_{0.050};
  double pre_open_position_m_{0.060};
  double close_position_m_{0.026};
  std::vector<double> object_size_m_{0.060, 0.030, 0.030};
  bool use_target_size_{true};
  double square_aspect_ratio_max_{1.10};
  geometry_msgs::msg::Quaternion grasp_orientation_;
  bool use_target_yaw_{true};
  double target_yaw_offset_rad_{-kPi * 0.5};
  double target_yaw_symmetry_rad_{kPi};
  double maximum_target_tilt_rad_{5.0 * kPi / 180.0};
  std::vector<double> target_x_bounds_;
  std::vector<double> target_y_bounds_;
  std::vector<double> target_z_bounds_;
  std::vector<double> ground_plane_xy_size_;
  std::vector<std::string> touch_links_;
  double velocity_scale_{0.065};
  double acceleration_scale_{0.065};
  double planning_time_s_{10.0};
  int planning_attempts_{5};
  double cartesian_step_m_{0.001};
  double cartesian_jump_threshold_{0.0};
  double maximum_cartesian_joint_step_rad_{0.10};
  double minimum_cartesian_fraction_{0.999999};
  double target_message_timeout_s_{1.0};
  double target_acquisition_timeout_s_{4.0};
  double arrival_position_tolerance_m_{0.003};
  double arrival_orientation_tolerance_rad_{2.0 * kPi / 180.0};
  double action_timeout_s_{12.0};
  double state_timeout_s_{3.0};
  double gripper_state_tolerance_m_{0.003};
  double ground_plane_z_m_{0.0};
  double ground_plane_thickness_m_{0.020};
  double ground_plane_contact_epsilon_m_{0.00001};
  double maximum_cube_plane_gap_m_{0.010};
  double p6_gate_max_age_s_{600.0};

  mutable std::mutex mutex_;
  std::mutex start_mutex_;
  std::mutex action_mutex_;
  mutable std::mutex p6_mutex_;
  mutable std::string accepted_p6_gate_identity_;
  std::condition_variable condition_;
  State state_{WAITING_TARGET};
  std::string state_detail_{"waiting for a current stable P3 target"};
  geometry_msgs::msg::PoseStamped latest_target_;
  std::deque<geometry_msgs::msg::PoseStamped> recent_target_poses_;
  geometry_msgs::msg::PoseStamped locked_target_;
  geometry_msgs::msg::Vector3Stamped latest_target_size_;
  std::deque<geometry_msgs::msg::Vector3Stamped> recent_target_sizes_;
  std::vector<double> locked_object_size_m_{0.060, 0.030, 0.030};
  geometry_msgs::msg::Quaternion locked_grasp_orientation_;
  double locked_object_yaw_rad_{0.0};
  double locked_applied_yaw_rad_{0.0};
  double locked_yaw_symmetry_rad_{kPi};
  std::string locked_target_status_;
  std::string locked_color_;
  std::vector<double> locked_place_center_;
  moveit::core::RobotStatePtr locked_home_state_;
  geometry_msgs::msg::Pose locked_home_pose_;
  rclcpp::Time target_pose_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_size_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_valid_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_status_received_{0, 0, RCL_ROS_TIME};
  bool have_target_pose_{false};
  bool have_target_size_{false};
  bool target_valid_{false};
  std::string latest_target_status_;
  bool descend_authorized_{false};
  bool grip_confirmed_{false};
  bool batch_mode_{false};
  bool continuous_requested_{false};
  bool automatic_task_{false};
  std::atomic<bool> task_sort_enabled_{false};
  bool cycle_failure_skippable_{false};
  bool batch_empty_confirmed_{false};
  int batch_succeeded_{0};
  int batch_skipped_{0};
  std::vector<IgnoredTarget> ignored_batch_targets_;
  std::atomic<bool> arm_motion_started_{false};
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> canceled_{false};
  std::atomic<bool> action_goal_claimed_{false};
  std::thread worker_;
  std::thread action_thread_;
  std::shared_ptr<GoalHandlePickColor> active_action_goal_;

  std::mutex log_mutex_;
  std::filesystem::path run_directory_;
  std::ofstream event_stream_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "fr3c_first_grasp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
  try {
    auto task = std::make_shared<FirstGraspTask>(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "P5 configuration/startup rejected: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
