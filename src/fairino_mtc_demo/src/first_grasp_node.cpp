#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
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

    start_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {start(response, false);});
    continuous_start_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/start_continuous",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {start(response, true);});
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
  };

  struct PlannedMotion
  {
    std::string name;
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::RobotState start_state;
    moveit::core::RobotStatePtr end_state;
  };

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
    cube_id_ = declare_string("cube_id", "target_cube");
    ground_plane_id_ = declare_string("ground_plane_id", "rviz_ground_plane");
    log_directory_ = declare_string("log_directory", "evidence/P5/runs");
    p6_gate_file_ = declare_string(
      "p6_gate_file", "/home/industio/fairino_ws/evidence/P6/p6_execution_gate.txt");

    execute_ = node_->declare_parameter<bool>("execute", false);
    approach_clearance_m_ = declare_double("approach_clearance_m", 0.015);
    grasp_depth_m_ = declare_double("grasp_depth_m", 0.015);
    lift_distance_m_ = declare_double("lift_distance_m", 0.050);
    pre_open_position_m_ = declare_double("pre_open_position_m", 0.060);
    close_position_m_ = declare_double("close_position_m", 0.026);
    cube_side_m_ = declare_double("cube_side_m", 0.030);
    grasp_orientation_ = normalizedQuaternion(
      declare_vector("grasp_orientation_xyzw", std::vector<double>{}));
    target_x_bounds_ = declare_vector("target_x_bounds_m", std::vector<double>{});
    target_y_bounds_ = declare_vector("target_y_bounds_m", std::vector<double>{});
    target_z_bounds_ = declare_vector("target_top_z_bounds_m", std::vector<double>{});
    ground_plane_xy_size_ = declare_vector("ground_plane_xy_size_m", {2.0, 2.0});
    ground_plane_z_m_ = declare_double("ground_plane_z_m", 0.0);
    ground_plane_thickness_m_ = declare_double("ground_plane_thickness_m", 0.020);
    ground_plane_contact_epsilon_m_ = declare_double(
      "ground_plane_contact_epsilon_m", 0.00001);

    velocity_scale_ = declare_double("velocity_scale", 0.05);
    acceleration_scale_ = declare_double("acceleration_scale", 0.05);
    planning_time_s_ = declare_double("planning_time_s", 10.0);
    planning_attempts_ = node_->declare_parameter<int>("planning_attempts", 5);
    cartesian_step_m_ = declare_double("cartesian_step_m", 0.001);
    cartesian_jump_threshold_ = declare_double("cartesian_jump_threshold", 0.0);
    maximum_cartesian_joint_step_rad_ = declare_double(
      "maximum_cartesian_joint_step_rad", 0.10);
    minimum_cartesian_fraction_ = declare_double("minimum_cartesian_fraction", 0.999999);
    target_message_timeout_s_ = declare_double("target_message_timeout_s", 1.0);
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
      cube_id_.empty() || ground_plane_id_.empty())
    {
      throw std::runtime_error("frame, group, action and scene object names must not be empty");
    }
    if (!positive(approach_clearance_m_) || !positive(grasp_depth_m_) ||
      !positive(lift_distance_m_) || !positive(cube_side_m_) ||
      !positive(cartesian_step_m_) || !positive(planning_time_s_) ||
      !positive(target_message_timeout_s_) || !positive(action_timeout_s_) ||
      !positive(state_timeout_s_) || !positive(gripper_state_tolerance_m_) ||
      !positive(ground_plane_thickness_m_) || !positive(maximum_cube_plane_gap_m_) ||
      !positive(maximum_cartesian_joint_step_rad_) ||
      !positive(p6_gate_max_age_s_) ||
      planning_attempts_ < 1)
    {
      throw std::runtime_error("distance, timeout and planning parameters must be positive");
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
    if (message.header.frame_id != base_frame_ ||
      !isFinite(message.pose.position.x) || !isFinite(message.pose.position.y) ||
      !isFinite(message.pose.position.z))
    {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Rejecting target pose with wrong frame or non-finite position");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_target_ = message;
    target_pose_received_ = node_->now();
    have_target_pose_ = true;
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
  }

  bool targetSnapshot(
    geometry_msgs::msg::PoseStamped & result, std::string & status,
    std::string & reason) const
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
    if ((now - target_pose_received_).seconds() > target_message_timeout_s_ ||
      (now - target_valid_received_).seconds() > target_message_timeout_s_)
    {
      reason = "perception heartbeat/pose is stale";
      return false;
    }
    const rclcpp::Time sample_time(latest_target_.header.stamp, RCL_ROS_TIME);
    const double sample_age = (now - sample_time).seconds();
    if (!isFinite(sample_age) || sample_age < -0.1 ||
      sample_age > target_message_timeout_s_)
    {
      std::ostringstream message;
      message << "stable target sample is stale: age_s=" << sample_age;
      reason = message.str();
      return false;
    }
    const auto & point = latest_target_.pose.position;
    if (point.x < target_x_bounds_[0] || point.x > target_x_bounds_[1] ||
      point.y<target_y_bounds_[0] || point.y> target_y_bounds_[1] ||
      point.z<target_z_bounds_[0] || point.z> target_z_bounds_[1])
    {
      reason = "target is outside configured first-grasp bounds";
      return false;
    }
    const double cube_plane_gap = point.z - cube_side_m_ - ground_plane_z_m_;
    if (cube_plane_gap < 0.0 || cube_plane_gap > maximum_cube_plane_gap_m_) {
      std::ostringstream message;
      message << "current target is not on the configured table plane: gap_m="
              << cube_plane_gap;
      reason = message.str();
      return false;
    }
    result = latest_target_;
    status = latest_target_status_;
    return true;
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
      state != PLAN_ONLY_COMPLETE && state != COMPLETE)
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

  geometry_msgs::msg::Pose approachPose() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = locked_target_.pose.position;
    pose.position.z += approach_clearance_m_;
    pose.orientation = grasp_orientation_;
    return pose;
  }

  geometry_msgs::msg::Pose graspPose() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = locked_target_.pose.position;
    pose.position.z -= grasp_depth_m_;
    pose.orientation = grasp_orientation_;
    return pose;
  }

  geometry_msgs::msg::Pose liftPose() const
  {
    auto pose = graspPose();
    pose.position.z += lift_distance_m_;
    return pose;
  }

  moveit_msgs::msg::CollisionObject cubeObject() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = locked_target_.pose.position;
    pose.position.z -= cube_side_m_ * 0.5;
    pose.orientation = locked_target_.pose.orientation;
    const double norm = std::sqrt(
      pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
    if (norm < 1e-9) {
      pose.orientation.w = 1.0;
    }
    return makeBox(cube_id_, base_frame_, {cube_side_m_, cube_side_m_, cube_side_m_}, pose);
  }

  bool applyWorldScene(std::string & reason)
  {
    geometry_msgs::msg::Pose plane_pose;
    plane_pose.position.z = ground_plane_z_m_ - ground_plane_contact_epsilon_m_ -
      ground_plane_thickness_m_ * 0.5;
    plane_pose.orientation.w = 1.0;
    const double cube_bottom = locked_target_.pose.position.z - cube_side_m_;
    const double cube_plane_gap = cube_bottom - ground_plane_z_m_;
    if (cube_plane_gap < 0.0 || cube_plane_gap > maximum_cube_plane_gap_m_) {
      std::ostringstream stream;
      stream << "visual cube bottom must be on/above the RViz plane and within "
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
    primitive.dimensions = {cube_side_m_, cube_side_m_, cube_side_m_};
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
    if (!planning_scene_.applyAttachedCollisionObject(attachedCube(false))) {
      reason = "failed to remove preview attachment";
      return false;
    }
    if (!planning_scene_.applyCollisionObject(cubeObject())) {
      reason = "failed to restore cube after preview attachment";
      return false;
    }
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

  PlannedMotion planApproach(const moveit::core::RobotStatePtr & start)
  {
    move_group_->setStartState(*start);
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(approachPose(), tip_frame_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto code = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    recordReturn("plan_approach", code.val);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error(
              "approach planning failed with MoveIt code " +
              std::to_string(code.val));
    }
    PlannedMotion motion;
    motion.name = "approach";
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

  std::vector<PlannedMotion> previewAll()
  {
    std::string reason;
    if (!applyWorldScene(reason)) {
      throw std::runtime_error(reason);
    }
    auto start = currentStartState(!execute_);
    auto approach = planApproach(start);
    auto descend = planCartesian("descend", approach.end_state, graspPose());
    if (!attachCube(reason)) {
      throw std::runtime_error(reason);
    }
    PlannedMotion lift;
    try {
      lift = planCartesian("lift", descend.end_state, liftPose());
    } catch (...) {
      std::string restore_reason;
      restoreWorldCube(restore_reason);
      throw;
    }
    if (!restoreWorldCube(reason)) {
      throw std::runtime_error(reason);
    }
    std::vector<PlannedMotion> motions;
    motions.push_back(std::move(approach));
    motions.push_back(std::move(descend));
    motions.push_back(std::move(lift));
    publishPreview(motions);
    recordEvent("preview_complete", "approach, full descend and attached full lift planned");
    return motions;
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
    const auto code = move_group_->execute(motion.trajectory);
    recordReturn("execute_" + motion.name, code.val);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      fail(motion.name + " execution failed with MoveIt code " + std::to_string(code.val));
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
           << ",position=" << position;
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
    geometry_msgs::msg::PoseStamped target;
    std::string target_status;
    if (!targetSnapshot(target, target_status, reason)) {
      response->success = false;
      response->message = "Start refused now: " + reason;
      return;
    }
    if (prior_state == COMPLETE && !clearPreviousCubeScene(reason)) {
      response->success = false;
      response->message = "Next trial refused: " + reason;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ != WAITING_TARGET && state_ != RETRYABLE_FAILED &&
        state_ != PLAN_ONLY_COMPLETE && state_ != COMPLETE)
      {
        response->success = false;
        response->message = "Task is not idle; current state=" + stateName(state_);
        return;
      }
      locked_target_ = target;
      locked_target_status_ = target_status;
      state_ = PLANNING;
      state_detail_ = prior_state == COMPLETE ?
        "next trial accepted; cleared the previous attachment and copied the current stable target"
        :
        "start accepted; copied the current stable P3 target";
      // Continuous mode pre-authorizes both transitions. The worker still
      // advances only after each preceding action reports success, so there
      // is no fixed delay and failures remain fail-closed.
      descend_authorized_ = continuous;
      grip_confirmed_ = continuous;
      arm_motion_started_.store(false);
      canceled_.store(false);
      worker_running_.store(true);
      worker_ = std::thread(
        [this]() {
          run();
          worker_running_.store(false);
        });
    }
    response->success = true;
    response->message = execute_ ?
      (continuous ?
      "Accepted continuous grasp: pre-open, approach, descend, close, and lift." :
      "Accepted current stable target copy: pre-open, preview, then approach.") :
      "Accepted current stable target copy for plan-only preview.";
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

  void run()
  {
    try {
      initializeRunLog();
      std::ostringstream target_detail;
      target_detail << std::setprecision(9) << "locked top=["
                    << locked_target_.pose.position.x << ","
                    << locked_target_.pose.position.y << ","
                    << locked_target_.pose.position.z << "] m";
      recordEvent("target_locked", target_detail.str());
      RCLCPP_INFO(node_->get_logger(), "%s", target_detail.str().c_str());
      recordTarget();
      if (execute_ && !commandGripper(pre_open_position_m_, "pre_open")) {
        return;
      }
      if (execute_ && !waitForGripperState(pre_open_position_m_, "pre_open")) {
        return;
      }
      auto preview = previewAll();
      if (!execute_) {
        setState(PLAN_ONLY_COMPLETE, "three motions planned; no command was sent");
        return;
      }
      std::string p6_reason;
      if (!p6Gate(p6_reason)) {
        fail("before approach: " + p6_reason);
        return;
      }
      recordEvent("p6_gate", "before approach passed");
      recordEvent("target_lock", "before approach uses immutable start snapshot");
      arm_motion_started_.store(true);
      if (!executeMotion(preview.front(), approachPose())) {
        return;
      }
      setState(WAITING_DESCEND, "inspect high-clearance alignment, then call descend_and_close");
      if (!waitForAuthorization(descend_authorized_)) {
        return;
      }
      setState(DESCENDING, "planning from actual state, then descending and closing");
      recordEvent("target_lock", "before descend uses immutable start snapshot");
      auto descend = planCartesian("descend_execute", currentStartState(false), graspPose());
      if (!executeMotion(descend, graspPose())) {
        return;
      }
      if (!commandGripper(close_position_m_, "close")) {
        return;
      }
      if (!waitForGripperState(close_position_m_, "close")) {
        return;
      }
      setState(
        WAITING_GRIP_CONFIRMATION,
        "26 mm was commanded open-loop; physically confirm grip before lift");
      if (!waitForAuthorization(grip_confirmed_)) {
        return;
      }
      setState(LIFTING, "manual grip confirmed; attaching cube and planning lift");
      recordEvent("target_lock", "before lift uses immutable start snapshot");
      std::string reason;
      if (!attachCube(reason)) {
        fail(reason);
        return;
      }
      auto lift = planCartesian("lift_execute", currentStartState(false), liftPose());
      if (!executeMotion(lift, liftPose())) {
        return;
      }
      setState(
        COMPLETE,
        "lift complete; cube remains attached in the scene; after physically placing it, "
        "a fresh stable target may start the next trial");
    } catch (const std::exception & error) {
      fail(error.what());
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
    publishState();
  }

  void publishState()
  {
    State state;
    std::string detail;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = state_;
      detail = state_detail_;
    }
    const bool retryable =
      state == WAITING_TARGET || state == RETRYABLE_FAILED ||
      state == PLAN_ONLY_COMPLETE || state == COMPLETE;
    std_msgs::msg::String message;
    message.data = "{\"state\":\"" + stateName(state) + "\",\"detail\":\"" +
      jsonEscape(detail) + "\",\"execute\":" + (execute_ ? "true" : "false") +
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
           << "grasp_orientation_xyzw: [" << grasp_orientation_.x << ", "
           << grasp_orientation_.y << ", " << grasp_orientation_.z << ", "
           << grasp_orientation_.w << "]\n"
           << "approach_clearance_m: " << approach_clearance_m_ << "\n"
           << "grasp_depth_m: " << grasp_depth_m_ << "\n"
           << "lift_distance_m: " << lift_distance_m_ << "\n"
           << "pre_open_position_m: " << pre_open_position_m_ << "\n"
           << "close_position_m: " << close_position_m_ << "\n"
           << "cube_side_m: " << cube_side_m_ << "\n"
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr continuous_start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr descend_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lift_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::string arm_group_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string gripper_joint_;
  std::string gripper_action_;
  std::string target_pose_topic_;
  std::string target_valid_topic_;
  std::string target_status_topic_;
  std::string cube_id_;
  std::string ground_plane_id_;
  std::string log_directory_;
  std::string p6_gate_file_;
  bool execute_{false};
  double approach_clearance_m_{0.015};
  double grasp_depth_m_{0.015};
  double lift_distance_m_{0.050};
  double pre_open_position_m_{0.060};
  double close_position_m_{0.026};
  double cube_side_m_{0.030};
  geometry_msgs::msg::Quaternion grasp_orientation_;
  std::vector<double> target_x_bounds_;
  std::vector<double> target_y_bounds_;
  std::vector<double> target_z_bounds_;
  std::vector<double> ground_plane_xy_size_;
  std::vector<std::string> touch_links_;
  double velocity_scale_{0.05};
  double acceleration_scale_{0.05};
  double planning_time_s_{10.0};
  int planning_attempts_{5};
  double cartesian_step_m_{0.001};
  double cartesian_jump_threshold_{0.0};
  double maximum_cartesian_joint_step_rad_{0.10};
  double minimum_cartesian_fraction_{0.999999};
  double target_message_timeout_s_{1.0};
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
  mutable std::mutex p6_mutex_;
  mutable std::string accepted_p6_gate_identity_;
  std::condition_variable condition_;
  State state_{WAITING_TARGET};
  std::string state_detail_{"waiting for a current stable P3 target"};
  geometry_msgs::msg::PoseStamped latest_target_;
  geometry_msgs::msg::PoseStamped locked_target_;
  std::string locked_target_status_;
  rclcpp::Time target_pose_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_valid_received_{0, 0, RCL_ROS_TIME};
  bool have_target_pose_{false};
  bool target_valid_{false};
  std::string latest_target_status_;
  bool descend_authorized_{false};
  bool grip_confirmed_{false};
  std::atomic<bool> arm_motion_started_{false};
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> canceled_{false};
  std::thread worker_;

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
