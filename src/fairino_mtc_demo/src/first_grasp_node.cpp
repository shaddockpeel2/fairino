#include <control_msgs/action/gripper_command.hpp>
#include <fairino_msgs/action/move_to_pose.hpp>
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
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
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
using MoveToPose = fairino_msgs::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
using PickColor = fairino_msgs::action::PickColor;
using GoalHandlePickColor = rclcpp_action::ServerGoalHandle<PickColor>;
using ClientGoalHandlePickColor = rclcpp_action::ClientGoalHandle<PickColor>;

constexpr double kPi = 3.14159265358979323846;
// pick_stack places exactly one cube per request, so a four-cube tower is built
// by calling it again with the cube that the previous call left on top as the
// new support. This is the default number of cubes a tower may contain: the
// support cube of a new pair may already be the top of a shorter tower, up to
// tier maximum_stack_cubes - 1. The perception side has to offer one
// stacking_cube_height_offsets_m entry per stacked tier, i.e.
// maximum_stack_cubes - 1 entries, or the tower top stops being visible as a
// cube (see fr3c_camera/config/blue_cube_localizer.yaml).
constexpr int kMaximumStackCubesDefault = 4;
// A tower this tall is already far outside the cell; the limit only exists so a
// typo in the parameter cannot disable the tier check entirely.
constexpr int kMaximumStackCubesLimit = 8;
// pick_stack_auto reuses the stack implementation through a private Action
// endpoint.  The marker prevents an accidental caller of that private endpoint
// from being treated as an orchestrator step; the public pick_stack endpoint
// remains locked for the whole automatic tower.
constexpr const char * kAutoStackInternalZone = "__fr3c_auto_stack_internal__";
// When both cubes are on the table, this tolerance absorbs the independent
// depth/top-surface estimates of their bottom heights. It also bounds the
// average per-cube height implied by a reported tower tier.
constexpr double kStackTabletopToleranceDefault = 0.012;
// Keep a small numerical gap between an attached source cube and the source
// tower obstacle at the instant the gripper closes.
constexpr double kStackSourceColumnTopGapM = 0.001;
// pick_stack_auto builds one tower from the colours that are on the tabletop,
// always in this order and always with the first entry on top: red, then
// yellow, blue and finally green on the table. A colour that is missing is
// skipped, so red/blue/green gives a three-cube tower and red/green a two-cube
// one. The order is fixed here and never taken from the caller, so the same
// spoken command always produces the same tower.
constexpr std::array<const char *, 4> kAutoStackOrder{"red", "yellow", "blue", "green"};

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

geometry_msgs::msg::Quaternion rollQuaternion(double roll)
{
  geometry_msgs::msg::Quaternion result;
  result.x = std::sin(roll * 0.5);
  result.w = std::cos(roll * 0.5);
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

// A cube's top face is square, so a grasp yaw, that yaw plus one symmetry step,
// and every further multiple describe the same physical grip. pick_stack picks
// the equivalent whose placement rotation is smallest, which keeps the wrist
// close to the source orientation and gives the planner the best chance of a
// collision-free route onto the lower cube.
//
// The result is the requested rotation minus the nearest whole number of
// symmetry steps, so it never exceeds half a symmetry step. For a square top
// that worst case is 45 degrees, and it is reached when the two measured yaws
// differ by half a step. Callers that need a tighter bound must reject the
// candidate rather than assume the reduction removed more than it can.
//
// The step count is taken from the raw difference, not from a folded angle:
// folding reports the angle inside one turn, which loses how many turns the
// difference already covered. Both measured yaws are wrapped into [-pi, pi), so
// a quarter-turn symmetry can appear up to four times in the difference.
double closestEquivalentAngle(double target_yaw, double reference_yaw, double symmetry)
{
  const double raw = target_yaw - reference_yaw;
  const double steps = std::round(raw / symmetry);
  return raw - steps * symmetry;
}

double positionDistance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

// MoveIt may normalize a CollisionObject when it crosses the planning-scene
// service boundary: a pose originally stored in primitive_poses can come back
// in the object's outer pose with an identity primitive pose. Always compose
// both layers before comparing a retained obstacle with a newly perceived
// target, otherwise the same physical cube can exist twice in the scene.
geometry_msgs::msg::Point collisionPrimitivePosition(
  const moveit_msgs::msg::CollisionObject & object,
  const geometry_msgs::msg::Pose & primitive_pose)
{
  auto orientation = object.pose.orientation;
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (norm < 1e-9) {
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;
  } else {
    orientation = normalizedQuaternion(orientation);
  }
  geometry_msgs::msg::Vector3 local;
  local.x = primitive_pose.position.x;
  local.y = primitive_pose.position.y;
  local.z = primitive_pose.position.z;
  const auto rotated = rotate(orientation, local);
  geometry_msgs::msg::Point world;
  world.x = object.pose.position.x + rotated.x;
  world.y = object.pose.position.y + rotated.y;
  world.z = object.pose.position.z + rotated.z;
  return world;
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

// A placement zone names a calibrated A4 grid region and, optionally, one
// 1-based slot inside it: "A", "A3", "c". Whitespace is ignored.
struct ZoneRequest
{
  std::string region;
  bool explicit_slot{false};
  std::size_t slot_index{0};
};

bool parseZone(const std::string & raw, ZoneRequest & request, std::string & reason)
{
  // Build into a local and publish only on success, so a rejected zone can
  // never leave a half-filled request for a caller to act on.
  ZoneRequest parsed;
  std::string text;
  for (const char character : raw) {
    const auto value = static_cast<unsigned char>(character);
    if (std::isspace(value) == 0) {
      text.push_back(static_cast<char>(std::tolower(value)));
    }
  }
  if (text.empty()) {
    reason = "zone is empty";
    return false;
  }
  std::size_t split = 0;
  while (split < text.size() &&
    std::isalpha(static_cast<unsigned char>(text[split])) != 0)
  {
    ++split;
  }
  if (split == 0) {
    reason = "zone must start with a region name, for example A or A3";
    return false;
  }
  for (std::size_t index = split; index < text.size(); ++index) {
    if (std::isdigit(static_cast<unsigned char>(text[index])) == 0) {
      reason = "zone must be a region name with an optional slot number, "
        "for example A or A3";
      return false;
    }
  }
  parsed.region = text.substr(0, split);
  parsed.explicit_slot = split < text.size();
  if (parsed.explicit_slot) {
    const std::string digits = text.substr(split);
    if (digits.size() > 2) {
      reason = "zone slot number is out of range; slots are numbered from 1";
      return false;
    }
    const int slot = std::stoi(digits);
    if (slot < 1) {
      reason = "zone slot number is out of range; slots are numbered from 1";
      return false;
    }
    parsed.slot_index = static_cast<std::size_t>(slot - 1);
  }
  request = parsed;
  return true;
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
    case 17: return "complete_stacked_home";
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
    reset_tabletop_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/reset_tabletop_objects",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
          resetTabletopObjects(response);
        } catch (const std::exception & error) {
          response->success = false;
          response->message = std::string("重置失败：") + error.what();
        }
      }, rmw_qos_profile_services_default, start_callback_group_);
    perception_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(
      perception_reset_service_);
    perception_skip_client_ = node_->create_client<std_srvs::srv::Trigger>(
      perception_skip_service_);
    perception_color_client_ = node_->create_client<fairino_msgs::srv::SetTargetColor>(
      perception_color_service_);
    pick_hold_server_ = createPickServer("~/pick_hold", PickTaskMode::HOLD);
    pick_sort_server_ = createPickServer("~/pick_sort", PickTaskMode::SORT);
    pick_place_server_ = createPickServer("~/pick_place", PickTaskMode::PLACE);
    pick_stack_server_ = createPickServer("~/pick_stack", PickTaskMode::STACK);
    // Automatic steps use a private endpoint backed by the same implementation.
    // Keeping them off the public endpoint lets that endpoint stay locked for
    // the entire tower without a process-wide "internal call" race window.
    pick_stack_internal_server_ = createPickServer(
      "~/internal/pick_stack", PickTaskMode::STACK, true);
    pick_stack_client_ = rclcpp_action::create_client<PickColor>(
      node_, "~/internal/pick_stack");
    pick_auto_server_ = createAutoStackServer("~/pick_stack_auto");
    move_to_pose_server_ = createMoveToPoseServer("~/move_to_pose");
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
    cancelInternalStackGoal();
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
    if (auto_action_thread_.joinable()) {
      auto_action_thread_.join();
    }
    if (pose_action_thread_.joinable()) {
      pose_action_thread_.join();
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
    STACK_COMPLETE = 17,
  };

  struct PlannedMotion
  {
    std::string name;
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::RobotState start_state;
    moveit::core::RobotStatePtr end_state;
  };

  struct PendingGripperCommand
  {
    double position{0.0};
    std::string name;
    GoalHandleGripper::SharedPtr goal_handle;
    std::shared_future<GoalHandleGripper::WrappedResult> result_future;
  };

  struct JointPathMetrics
  {
    double duration_s{0.0};
    double path_length_rad{0.0};
    std::vector<std::string> joint_names;
    std::vector<double> joint_travel_rad;
  };

  struct GraspCandidate
  {
    double yaw_rad{0.0};
    geometry_msgs::msg::Quaternion orientation;
    double tilt_rad{0.0};
  };

  // One physically valid cube reported by the same perception frame as the
  // selected target. pick_stack needs the lower cube's pose and yaw, which the
  // single-colour filter never selects, so those values travel in the status.
  struct DetectedCube
  {
    std::string color;
    geometry_msgs::msg::Point center;
    double top_z{0.0};
    double grasp_axis_yaw_rad{0.0};
    double grasp_axis_yaw_period_rad{kPi * 0.5};
    std::vector<double> object_size_m;
    int stack_tier{0};
  };

  // The immutable lower-cube snapshot of one pick_stack task. `stack_tier` is
  // the tier the support cube was already at when the pair was frozen: 0 for a
  // cube on the table, k for the top of a k-cube tower. The new cube becomes
  // tier k + 1.
  struct StackDestination
  {
    bool valid{false};
    std::string color;
    geometry_msgs::msg::Pose top_pose;
    std::vector<double> object_size_m;
    double grasp_axis_yaw_rad{0.0};
    double grasp_axis_yaw_period_rad{kPi * 0.5};
    int stack_tier{0};
  };

  // One finished single-cube step of an automatic stack. The step is a real
  // pick_stack goal, so its outcome is whatever that task reported.
  struct AutoStackStepOutcome
  {
    bool success{false};
    bool canceled{false};
    uint16_t error_code{PickColor::Result::INTERNAL_ERROR};
    std::string message;
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

  // Action goals are free-form text, so colour values are trimmed and lowered
  // before any comparison. An empty result means "not specified".
  static std::string normalizedColor(const std::string & raw)
  {
    const auto first = raw.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return {};
    }
    const auto last = raw.find_last_not_of(" \t\r\n");
    std::string text = raw.substr(first, last - first + 1);
    std::transform(
      text.begin(), text.end(), text.begin(),
      [](unsigned char character) {return static_cast<char>(std::tolower(character));});
    return text;
  }

  // The four action names share one message type, so the task mode has to be
  // carried explicitly instead of being inferred from the goal fields.
  enum class PickTaskMode
  {
    HOLD,
    SORT,
    PLACE,
    STACK,
  };

  static bool modeIsSorting(PickTaskMode mode)
  {
    return mode == PickTaskMode::SORT || mode == PickTaskMode::PLACE ||
           mode == PickTaskMode::STACK;
  }

  static const char * modeName(PickTaskMode mode)
  {
    switch (mode) {
      case PickTaskMode::HOLD: return "pick_hold";
      case PickTaskMode::SORT: return "pick_sort";
      case PickTaskMode::PLACE: return "pick_place";
      case PickTaskMode::STACK: return "pick_stack";
    }
    return "unknown";
  }

  // Abort a goal before any work was done. The result is filled exactly like
  // finishPickAction() fills it, so a caller cannot tell the two apart.
  void abortGoal(
    const std::shared_ptr<GoalHandlePickColor> & goal_handle,
    uint16_t error_code, const std::string & message)
  {
    auto result = std::make_shared<PickColor::Result>();
    result->success = false;
    result->error_code = error_code;
    result->message = message;
    result->color = goal_handle->get_goal()->color;
    result->zone = goal_handle->get_goal()->zone;
    result->support_color = goal_handle->get_goal()->support_color;
    result->executed = false;
    goal_handle->abort(result);
  }

  void abortMoveToPoseGoal(
    const std::shared_ptr<GoalHandleMoveToPose> & goal_handle,
    uint16_t error_code, const std::string & message)
  {
    auto result = std::make_shared<MoveToPose::Result>();
    result->success = false;
    result->error_code = error_code;
    result->message = message;
    result->executed = false;
    goal_handle->abort(result);
  }

  rclcpp_action::Server<MoveToPose>::SharedPtr createMoveToPoseServer(
    const std::string & name)
  {
    return rclcpp_action::create_server<MoveToPose>(
      node_, name,
      [](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const MoveToPose::Goal> goal) {
        return goal && isFinite(goal->x) && isFinite(goal->y) && isFinite(goal->z) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
        rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose>) {
        canceled_.store(true);
        move_group_->stop();
        condition_.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        std::lock_guard<std::mutex> start_lock(start_mutex_);
        if (auto_stack_active_.load() || action_goal_claimed_.exchange(true)) {
          abortMoveToPoseGoal(
            goal_handle, MoveToPose::Result::BUSY,
            "another arm task is active");
          return;
        }
        if (pose_action_thread_.joinable()) {
          pose_action_thread_.join();
        }
        markActionStart();
        pose_action_thread_ = std::thread(
          [this, goal_handle]() {executeMoveToPoseAction(goal_handle);});
      });
  }

  void publishMoveToPoseFeedback(
    const std::shared_ptr<GoalHandleMoveToPose> & goal_handle,
    const std::string & phase, const std::string & detail)
  {
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->phase = phase;
    feedback->detail = detail;
    goal_handle->publish_feedback(feedback);
  }

  void finishMoveToPoseAction(
    const std::shared_ptr<GoalHandleMoveToPose> & goal_handle,
    uint16_t error_code, const std::string & message, bool success,
    bool executed, State previous_state, bool restore_state)
  {
    auto result = std::make_shared<MoveToPose::Result>();
    result->success = success;
    result->error_code = error_code;
    result->message = message;
    result->executed = executed;
    if (goal_handle->is_canceling() || error_code == MoveToPose::Result::CANCELED) {
      goal_handle->canceled(result);
    } else if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    action_goal_claimed_.store(false);
    if (restore_state) {
      setState(
        previous_state,
        "manual XYZ motion finished; previous task state preserved: " + message);
    } else {
      publishState();
    }
  }

  bool manualPoseTargetValid(
    const MoveToPose::Goal & goal, std::string & reason) const
  {
    const std::array<double, 3> coordinates{goal.x, goal.y, goal.z};
    const std::array<const std::vector<double> *, 3> bounds{
      & manual_pose_x_bounds_, & manual_pose_y_bounds_, & manual_pose_z_bounds_};
    const std::array<const char *, 3> names{"x", "y", "z"};
    for (std::size_t index = 0; index < coordinates.size(); ++index) {
      if (!isFinite(coordinates[index]) || coordinates[index] < (*bounds[index])[0] ||
        coordinates[index] > (*bounds[index])[1])
      {
        std::ostringstream message;
        message << names[index] << "=" << coordinates[index]
                << " m is outside [" << (*bounds[index])[0] << ", "
                << (*bounds[index])[1] << "] m";
        reason = message.str();
        return false;
      }
    }
    return true;
  }

  void executeMoveToPoseAction(
    const std::shared_ptr<GoalHandleMoveToPose> & goal_handle)
  {
    canceled_.store(false);
    State previous_state = WAITING_TARGET;
    bool restore_state = false;
    auto finish = [this, &goal_handle, &previous_state, &restore_state](
      uint16_t error_code, const std::string & message, bool success = false,
      bool executed = false) {
        finishMoveToPoseAction(
          goal_handle, error_code, message, success, executed,
          previous_state, restore_state);
      };
    if (goal_handle->is_canceling()) {
      finish(MoveToPose::Result::CANCELED, "XYZ motion canceled before validation");
      return;
    }
    std::string reason;
    if (!stateAllowsStart(previous_state, reason)) {
      finish(MoveToPose::Result::BUSY, reason);
      return;
    }
    restore_state = true;
    if (previous_state == COMPLETE || cubeIsStillAttached()) {
      finish(
        MoveToPose::Result::PRECONDITION_FAILED,
        "XYZ motion refused while the gripper has a held-object record");
      return;
    }
    if (!p6Gate(reason)) {
      finish(
        MoveToPose::Result::PRECONDITION_FAILED,
        "P6 execution gate refused XYZ motion: " + reason);
      return;
    }
    if (!manualPoseTargetValid(*goal_handle->get_goal(), reason)) {
      finish(MoveToPose::Result::INVALID_TARGET, reason);
      return;
    }

    setState(PLANNING, "validating collision-free XYZ motion");
    publishMoveToPoseFeedback(
      goal_handle, "collision_check",
      "正在使用当前末端姿态审核工作空间、逆解和整条碰撞轨迹");
    try {
      initializeRunLog();
      if (planning_scene_.getObjects({ground_plane_id_}).count(ground_plane_id_) == 0 &&
        !planning_scene_.applyCollisionObject(groundPlaneObject()))
      {
        finish(
          MoveToPose::Result::PRECONDITION_FAILED,
          "failed to add the configured tabletop collision plane");
        return;
      }
      const auto start = currentStartState(false);
      auto target = move_group_->getCurrentPose(tip_frame_).pose;
      target.position.x = goal_handle->get_goal()->x;
      target.position.y = goal_handle->get_goal()->y;
      target.position.z = goal_handle->get_goal()->z;
      auto motion = planPoseFromSeededIk(
        "manual_xyz", start, target, manual_pose_ik_timeout_s_);
      publishPreview({motion});
      recordEvent(
        "manual_xyz_collision_check_passed",
        "x=" + std::to_string(target.position.x) +
        ",y=" + std::to_string(target.position.y) +
        ",z=" + std::to_string(target.position.z));
      if (canceled_.load() || goal_handle->is_canceling()) {
        finish(MoveToPose::Result::CANCELED, "XYZ motion canceled after planning");
        return;
      }
      if (!execute_) {
        finish(
          MoveToPose::Result::OK,
          "XYZ collision-free trajectory planned; execution mode is disabled",
          true, false);
        return;
      }
      publishMoveToPoseFeedback(
        goal_handle, "executing",
        "碰撞审核通过，正在执行规划轨迹");
      if (!executeMotion(motion, target)) {
        finish(
          canceled_.load() ? MoveToPose::Result::CANCELED :
          MoveToPose::Result::EXECUTION_FAILED,
          canceled_.load() ? "XYZ motion canceled during execution" :
          "XYZ trajectory did not complete or pass arrival tolerance");
        return;
      }
      finish(
        MoveToPose::Result::OK,
        "XYZ motion completed with collision-checked planning and arrival validation",
        true, true);
    } catch (const std::exception & error) {
      finish(
        canceled_.load() ? MoveToPose::Result::CANCELED :
        MoveToPose::Result::PLANNING_FAILED,
        canceled_.load() ? "XYZ motion canceled during planning" :
        std::string("collision-aware XYZ planning failed: ") + error.what());
    }
  }

  rclcpp_action::Server<PickColor>::SharedPtr createPickServer(
    const std::string & name, PickTaskMode mode, bool internal_only = false)
  {
    return rclcpp_action::create_server<PickColor>(
      node_, name,
      [this, internal_only](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const PickColor::Goal> goal) {
        if (internal_only &&
        (!auto_stack_active_.load() || !goal ||
        goal->zone != kAutoStackInternalZone))
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
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
      [this, mode, internal_only](const std::shared_ptr<GoalHandlePickColor> goal_handle) {
        // Serialize ownership changes with the legacy /start service and the
        // automatic-stack accept callback. Without this lock, a manual goal and
        // an automatic goal could both observe the other's flag as false.
        std::lock_guard<std::mutex> start_lock(start_mutex_);
        // The public servers remain locked for the whole automatic tower. Its
        // own steps arrive only through the separate private server above, so
        // an unrelated public goal can never be mistaken for an internal one.
        if (auto_stack_active_.load() && !internal_only) {
          abortGoal(
            goal_handle, PickColor::Result::BUSY,
            "pick_stack_auto is building a tower; wait for it to finish");
          return;
        }
        if (action_goal_claimed_.exchange(true)) {
          abortGoal(goal_handle, PickColor::Result::BUSY, "another grasp task is active");
          return;
        }
        if (action_thread_.joinable()) {
          action_thread_.join();
        }
        markActionStart();
        action_thread_ = std::thread(
          [this, goal_handle, mode, internal_only]() {
            executePickAction(goal_handle, mode, internal_only);
          });
      });
  }

  // -----------------------------------------------------------------------
  // pick_stack_auto: one goal builds the whole tower
  //
  // The order is fixed (kAutoStackOrder) and the caller supplies no colour.
  // Each layer is the very same single-cube task an operator would send by
  // hand, issued internally through the pick_stack action server, so both paths
  // share one implementation and one set of geometry checks. The automatic
  // stack keeps one attempt per layer and never retries or releases after a
  // failure. Successful intermediate layers end at their safe retreat pose;
  // the final layer alone returns to the home snapshot of the whole tower.
  // -----------------------------------------------------------------------
  rclcpp_action::Server<PickColor>::SharedPtr createAutoStackServer(
    const std::string & name)
  {
    return rclcpp_action::create_server<PickColor>(
      node_, name,
      [](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const PickColor::Goal> goal) {
        // This action has no request parameters. Reject malformed direct ROS
        // clients here as well as in the GUI bridge instead of silently
        // ignoring caller-supplied fields.
        if (!goal || !goal->color.empty() || !goal->zone.empty() ||
        !goal->support_color.empty() || !goal->source_location.empty())
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandlePickColor>) {
        // Cancel the layer that is running right now. The orchestrator stops
        // after it and reports the tower as unfinished; whatever is already
        // standing stays standing.
        canceled_.store(true);
        cancelInternalStackGoal();
        move_group_->stop();
        if (gripper_client_) {
          gripper_client_->async_cancel_all_goals();
        }
        condition_.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandlePickColor> goal_handle) {
        std::lock_guard<std::mutex> start_lock(start_mutex_);
        // Refuse up front when another task already owns the arm, instead of
        // discovering the colours and failing on the first layer.
        if (action_goal_claimed_.load() || worker_running_.load()) {
          abortGoal(
            goal_handle, PickColor::Result::BUSY,
            "another grasp task is active; the automatic stack needs the whole arm");
          return;
        }
        {
          std::lock_guard<std::mutex> lock(auto_stack_mutex_);
          if (auto_stack_running_) {
            abortGoal(
              goal_handle, PickColor::Result::BUSY,
              "another automatic stack is already running");
            return;
          }
          auto_stack_running_ = true;
        }
        auto_stack_active_.store(true);
        if (auto_action_thread_.joinable()) {
          auto_action_thread_.join();
        }
        markActionStart();
        auto_action_thread_ = std::thread(
          [this, goal_handle]() {
            executeAutoStackAction(goal_handle);
          });
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
    result->zone = goal_handle->get_goal()->zone;
    result->support_color = goal_handle->get_goal()->support_color;
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
    task_sort_area_source_ = false;
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
    const std::shared_ptr<GoalHandlePickColor> goal_handle, PickTaskMode mode,
    bool internal_stack_step = false)
  {
    const std::string color = normalizedColor(goal_handle->get_goal()->color);
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
    const std::string raw_zone = internal_stack_step ? std::string{} :
    goal_handle->get_goal()->zone;
    const std::string raw_support_color = goal_handle->get_goal()->support_color;
    const std::string source_location = goal_handle->get_goal()->source_location;
    const bool source_from_sort_area = source_location == "sort_area";
    if (!source_location.empty() && !source_from_sort_area) {
      finishPickAction(goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "source_location must be empty or sort_area", false);
      return;
    }
    if (internal_stack_step && source_from_sort_area) {
      finishPickAction(goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "automatic stack steps cannot change camera view", false);
      return;
    }
    ZoneRequest zone_request;
    std::string zone_reason;
    if (mode == PickTaskMode::PLACE) {
      if (!parseZone(raw_zone, zone_request, zone_reason)) {
        finishPickAction(
          goal_handle, PickColor::Result::INVALID_ZONE,
          "invalid pick_place zone: " + zone_reason, false);
        return;
      }
      if (zone_centers_.count(zone_request.region) == 0) {
        finishPickAction(
          goal_handle, PickColor::Result::INVALID_ZONE,
          "unknown placement zone '" + zone_request.region +
          "'; calibrated zones are " + calibratedZoneList(), false);
        return;
      }
    } else if (!raw_zone.empty()) {
      finishPickAction(
        goal_handle, PickColor::Result::INVALID_ZONE,
        std::string(modeName(mode)) + " does not accept a zone; " +
        "use /fr3c_first_grasp/pick_place for an explicit placement zone", false);
      return;
    }

    std::string support_color;
    if (mode == PickTaskMode::STACK) {
      support_color = normalizedColor(raw_support_color);
      if (support_color.empty()) {
        finishPickAction(
          goal_handle, PickColor::Result::INVALID_SUPPORT_COLOR,
          "pick_stack requires support_color: the colour of the cube that stays "
          "on the table and becomes the lower cube of the stack", false);
        return;
      }
      if (!supportedColor(support_color)) {
        finishPickAction(
          goal_handle, PickColor::Result::INVALID_SUPPORT_COLOR,
          "support_color must be one of: red, yellow, blue, green", false);
        return;
      }
      if (support_color == color) {
        finishPickAction(
          goal_handle, PickColor::Result::INVALID_SUPPORT_COLOR,
          "pick_stack needs two different colours; color and support_color are both " +
          color, false);
        return;
      }
    } else if (!raw_support_color.empty()) {
      finishPickAction(
        goal_handle, PickColor::Result::INVALID_SUPPORT_COLOR,
        std::string(modeName(mode)) + " does not accept a support_color; " +
        "use /fr3c_first_grasp/pick_stack to stack one cube on another", false);
      return;
    }

    std::unique_lock<std::mutex> start_lock(start_mutex_);
    std::string reason;
    if (modeIsSorting(mode) && !sort_enabled_) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        std::string(modeName(mode)) +
        " requires sort_enabled=true and configured placement regions", false);
      return;
    }
    if (mode == PickTaskMode::STACK && !stack_enabled_) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "pick_stack requires stack_enabled=true", false);
      return;
    }
    if (source_from_sort_area && (!sort_enabled_ ||
      sort_area_observation_joints_rad_.empty()))
    {
      finishPickAction(goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "sort-area source requires colors perception and a configured "
        "observation joint pose", false);
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
    // A stack that failed after closing the gripper returns home with the cube
    // still attached. The next task's pre-open step would drop it, so the task
    // is refused with the same code the hold flow uses until the operator has
    // physically released the cube and cleared it (acknowledge_object_released
    // for a hold, or a fresh release of the planning scene).
    if (cubeIsStillAttached()) {
      finishPickAction(
        goal_handle, PickColor::Result::HOLDING_OBJECT,
        "the previous task left the cube attached to the gripper; remove it and "
        "clear the scene before starting another task", false);
      return;
    }
    if (prior_state == SORT_COMPLETE && !archivePreviousSortedCubeScene(reason)) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "cannot prepare the previous sorted scene: " + reason, false);
      return;
    }
    // In an automatic tower the previous source is exactly this layer's
    // support.  Do not archive a second obstacle at the same pose: the normal
    // source cleanup below removes target_cube and applyWorldScene() recreates
    // that physical cube as the support plus its compact tower column.
    if (prior_state == STACK_COMPLETE && !internal_stack_step &&
      !archivePreviousSortedCubeScene(reason))
    {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        "cannot prepare the previous stacked scene: " + reason, false);
      return;
    }
    if (prior_state == STACK_COMPLETE && !clearStackSupportObject(reason)) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED, reason, false);
      return;
    }

    try {
      // Every layer of one automatic tower shares the home snapshot taken
      // before the first motion.  Only the final layer uses it; ordinary
      // pick_stack requests keep their original one-request/one-home contract.
      bool reused_auto_home = false;
      if (internal_stack_step) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto_stack_home_state_) {
          locked_home_state_ = std::make_shared<moveit::core::RobotState>(
            *auto_stack_home_state_);
          locked_home_pose_ = auto_stack_home_pose_;
          reused_auto_home = true;
        }
      }
      if (!reused_auto_home) {
        locked_home_state_ = currentStartState(false);
        locked_home_pose_ = move_group_->getCurrentPose(tip_frame_).pose;
      }
    } catch (const std::exception & error) {
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED,
        std::string("cannot snapshot the task start state: ") + error.what(), false);
      return;
    }

    batch_mode_ = false;
    continuous_requested_ = false;
    automatic_task_ = true;
    task_sort_enabled_ = modeIsSorting(mode);
    task_stack_enabled_ = mode == PickTaskMode::STACK;
    task_sort_area_source_ = source_from_sort_area;
    task_return_home_ = !internal_stack_step ||
      auto_stack_step_number_.load() >= auto_stack_step_count_.load();
    task_pre_open_ = !internal_stack_step || auto_stack_step_number_.load() <= 1;
    task_direct_approach_ = internal_stack_step && auto_stack_step_number_.load() > 1;
    canceled_.store(false);
    if (goal_handle->is_canceling()) {
      canceled_.store(true);
      finishPickAction(
        goal_handle, PickColor::Result::CANCELED,
        "task canceled before target acquisition", false);
      return;
    }
    // A finished or abandoned stack may leave virtual columns in the scene.
    // Every new task clears them before installing its own fresh snapshot.
    if (!clearStackSupportObject(reason)) {
      setState(RETRYABLE_FAILED, reason);
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED, reason, false);
      return;
    }
    // A stack that failed before closing the gripper left the source cube where
    // the perception node still sees it, so the stale planning-scene copy must
    // not become an obstacle for this task.
    if (mode == PickTaskMode::STACK && !clearPreviousCubeScene(reason)) {
      setState(RETRYABLE_FAILED, reason);
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED, reason, false);
      return;
    }
    worker_running_.store(true);
    observation_log_started_ = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      locked_color_ = color;
      locked_zone_ = zone_request;
      locked_place_center_.clear();
      locked_stack_support_color_ = support_color;
      locked_stack_destination_ = StackDestination();
      locked_stack_source_ = DetectedCube();
    }
    // Resolve the zone before any target acquisition or arm motion so an
    // unusable slot is reported as such instead of a planning failure.
    if (mode == PickTaskMode::PLACE && !assignZoneSlot(zone_reason)) {
      setState(RETRYABLE_FAILED, zone_reason);
      finishPickAction(
        goal_handle, PickColor::Result::ZONE_UNAVAILABLE, zone_reason, false);
      return;
    }
    const std::string task_name = modeName(mode);
    if (mode == PickTaskMode::STACK) {
      setState(
        WAITING_TARGET,
        task_name + " accepted; locating one fresh " + color + " cube and the " +
        support_color + " cube it will be placed on");
    } else {
      setState(
        WAITING_TARGET,
        mode == PickTaskMode::PLACE ?
        task_name + " accepted; requesting a fresh unique " + color +
        " target for zone " + zone_request.region :
        task_name + " accepted; requesting a fresh unique " + color + " target");
    }
    start_lock.unlock();

    uint64_t filter_generation = 0;
    uint16_t acquisition_error = PickColor::Result::INTERNAL_ERROR;
    if (internal_stack_step) {
      if (!lockAutoStackPairFromInventory(
          color, support_color, acquisition_error, reason))
      {
        setState(RETRYABLE_FAILED, reason);
        finishPickAction(goal_handle, acquisition_error, reason, false);
        return;
      }
      publishState();
    } else {
      if (source_from_sort_area && mode == PickTaskMode::STACK) {
        // The support is measured in the current/start view. Its immutable
        // base-frame pose remains the destination after the camera moves.
        if (!setPerceptionTargetColor(support_color, filter_generation, reason) ||
          !waitForRequestedTarget(
            support_color, filter_generation, acquisition_error, reason))
        {
          setState(RETRYABLE_FAILED, reason);
          finishPickAction(goal_handle, acquisition_error, reason, false);
          return;
        }
        if (!lockVisibleStackSupport(support_color, filter_generation,
          acquisition_error, reason))
        {
          setState(RETRYABLE_FAILED, reason);
          finishPickAction(goal_handle, acquisition_error, reason, false);
          return;
        }
      }
      if (source_from_sort_area) {
        try {
          initializeRunLog();
          observation_log_started_ = true;
          std::vector<moveit_msgs::msg::CollisionObject> observation_objects{
            groundPlaneObject()};
          if (mode == PickTaskMode::STACK) {
            observation_objects.push_back(stackSupportObject());
            if (stackColumnPresent()) {
              observation_objects.push_back(stackColumnObject());
            }
          }
          if (!planning_scene_.applyCollisionObjects(observation_objects)) {
            throw std::runtime_error("cannot install initial-view obstacles before observation move");
          }
          setState(PLANNING, "moving to the sort-area observation joints");
          auto observation = planSortAreaObservation(currentStartState(false));
          if (!execute_) {
            publishPreview({observation});
            reason = "sort-area observation route planned without motion; source "
              "acquisition and the complete task require execution mode";
            setState(RETRYABLE_FAILED, reason);
            finishPickAction(goal_handle, PickColor::Result::PRECONDITION_FAILED,
              reason, false);
            return;
          }
          reason.clear();
          if (!p6Gate(reason)) {
            setState(RETRYABLE_FAILED, reason);
            finishPickAction(goal_handle, PickColor::Result::PRECONDITION_FAILED,
              reason, false);
            return;
          }
          if (!executeMotion(observation, robotStateTipPose(*observation.end_state))) {
            reason = "sort-area observation motion failed; inspect the task state and run log";
            if (arm_motion_started_.load() && !canceled_.load()) {
              recoverToHomeAfterFailedStack();
            }
            finishPickAction(goal_handle, PickColor::Result::EXECUTION_FAILED,
              reason, false);
            return;
          }
          recordEvent("sort_area_observation_arrived", "source_color=" + color);
        } catch (const std::exception & error) {
          reason = std::string("sort-area observation planning failed: ") + error.what();
          setState(RETRYABLE_FAILED, reason);
          finishPickAction(goal_handle, PickColor::Result::PLANNING_FAILED,
            reason, false);
          return;
        }
      }
      if (!setPerceptionTargetColor(color, filter_generation, reason)) {
        if (source_from_sort_area && !canceled_.load()) {
          recoverToHomeAfterFailedStack();
        }
        finishPickAction(
          goal_handle, PickColor::Result::PERCEPTION_UNAVAILABLE, reason, false);
        return;
      }
      if (!waitForRequestedTarget(color, filter_generation, acquisition_error, reason)) {
        if (acquisition_error == PickColor::Result::CANCELED) {
          setState(CANCELED, reason);
        } else {
          setState(RETRYABLE_FAILED, reason);
        }
        if (source_from_sort_area && !canceled_.load()) {
          recoverToHomeAfterFailedStack();
        }
        finishPickAction(goal_handle, acquisition_error, reason, false);
        return;
      }
      // lockTargetSnapshot resets this flag because ordinary tasks have not
      // moved yet. This task already moved to its second observation view.
      if (source_from_sort_area) {
        arm_motion_started_.store(true);
      }
    }
    if (mode == PickTaskMode::STACK && !internal_stack_step) {
      uint16_t pair_error = PickColor::Result::INTERNAL_ERROR;
      if (source_from_sort_area ?
        !validateStackPair(reason, true) :
        !lockStackDestinationFromSourceSnapshot(
          support_color, color, filter_generation, pair_error, reason))
      {
        if (source_from_sort_area) {
          pair_error = PickColor::Result::STACK_GEOMETRY_REJECTED;
        }
        setState(RETRYABLE_FAILED, reason);
        if (source_from_sort_area && !canceled_.load()) {
          recoverToHomeAfterFailedStack();
        }
        finishPickAction(goal_handle, pair_error, reason, false);
        return;
      }
      if (!source_from_sort_area && !validateStackPair(reason)) {
        setState(RETRYABLE_FAILED, reason);
        finishPickAction(
          goal_handle, PickColor::Result::STACK_GEOMETRY_REJECTED, reason, false);
        return;
      }
      publishState();
    }
    // A completed sort/stack retains its placed cube as a MoveIt obstacle.
    // Reclaim that copy only after a new source has passed its stack geometry
    // checks, so a rejected request leaves the original scene untouched.
    if (!clearArchivedObstacleAtTarget(reason)) {
      setState(RETRYABLE_FAILED, reason);
      if (source_from_sort_area && !canceled_.load()) {
        recoverToHomeAfterFailedStack();
      }
      finishPickAction(
        goal_handle, PickColor::Result::PRECONDITION_FAILED, reason, false);
      return;
    }

    recordLatency("target_acquired");

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
      (mode == PickTaskMode::HOLD && final_state == COMPLETE) ||
      (mode != PickTaskMode::HOLD && final_state == SORT_COMPLETE) ||
      (mode == PickTaskMode::STACK && final_state == STACK_COMPLETE)))
    {
      std::string completion;
      if (mode == PickTaskMode::PLACE) {
        completion = execute_ ?
          "pick_place motion completed once; the selected cube was released in zone " + raw_zone :
          "pick_place full route to zone " + raw_zone + " planned; no command was sent";
      } else if (mode == PickTaskMode::STACK) {
        int tower_cubes = 2;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          tower_cubes = locked_stack_destination_.stack_tier + 2;
        }
        completion = execute_ ?
          "pick_stack motion completed once; the " + color + " cube was released on top of " +
          "the " + support_color + " cube as cube " + std::to_string(tower_cubes) + " of " +
          std::to_string(maximum_stack_cubes_) +
          (task_return_home_ ? " and the arm returned to its start joints" :
          " and the arm stayed at the safe retreat pose for the next automatic layer") :
          "pick_stack full route planned; no command was sent";
      } else if (mode == PickTaskMode::SORT) {
        completion = execute_ ?
          "pick_sort motion completed once; the cube was released in its color region" :
          "pick_sort full route planned; no command was sent";
      } else {
        completion = execute_ ?
          "pick_hold motion completed once; the gripper close was open-loop and the cube held" :
          "pick_hold route planned; no command was sent";
      }
      finishPickAction(goal_handle, PickColor::Result::OK, completion, true);
      return;
    }
    if (final_state == CANCELED || canceled_.load()) {
      finishPickAction(goal_handle, PickColor::Result::CANCELED, final_detail, false);
      return;
    }
    // pick_stack promises "one attempt, then the task slot is free again". That
    // includes the failure path: the arm was already sent home by runCycle()'s
    // recovery step, and the state below is a startable one, so the caller can
    // immediately issue the next stack without any manual state reset.
    finishPickAction(
      goal_handle,
      arm_motion_started_.load() ? PickColor::Result::EXECUTION_FAILED :
      PickColor::Result::PLANNING_FAILED,
      final_detail, false);
  }

  // -----------------------------------------------------------------------
  // pick_stack_auto: discovery, ordering and the step loop
  // -----------------------------------------------------------------------

  static std::string colorLabel(const std::string & color)
  {
    if (color == "red") {return "红色";}
    if (color == "yellow") {return "黄色";}
    if (color == "blue") {return "蓝色";}
    if (color == "green") {return "绿色";}
    return color;
  }

  // "红色、黄色、蓝色、绿色", i.e. the planned order from the top downwards.
  static std::string joinAutoStackOrder(const std::vector<std::string> & order)
  {
    std::string text;
    for (const auto & color : order) {
      if (!text.empty()) {
        text += "、";
      }
      text += colorLabel(color);
    }
    return text;
  }

  // The part of the tower that is standing after `placed_cubes` cubes were put
  // down, listed from the table upwards.
  static std::string standingTowerText(
    const std::vector<std::string> & order, std::size_t placed_cubes)
  {
    std::string text;
    for (std::size_t index = 0; index < placed_cubes && index < order.size(); ++index) {
      if (!text.empty()) {
        text += "→";
      }
      text += colorLabel(order[order.size() - 1 - index]);
    }
    return text.empty() ? std::string("(空)") : text;
  }

  // The two colours of step `step` (0-based): `upper` is the cube that is picked
  // up and `lower` the one that stays, both read from the fixed order, which
  // starts at the top of the tower. Step 0 therefore takes the two lowest
  // colours, and the last step puts the topmost colour on the finished tower.
  static void autoStackStepColors(
    const std::vector<std::string> & order, std::size_t step,
    std::string & upper, std::string & lower)
  {
    upper = order[order.size() - 2 - step];
    lower = order[order.size() - 1 - step];
  }

  static void setPromiseValue(
    const std::shared_ptr<std::promise<AutoStackStepOutcome>> & promise,
    const AutoStackStepOutcome & value)
  {
    try {
      promise->set_value(value);
    } catch (const std::future_error &) {
      // The wait already gave up on this step and reported its own reason.
    }
  }

  void publishAutoStackFeedback(
    const std::shared_ptr<GoalHandlePickColor> & goal_handle,
    const std::string & phase, const std::string & detail)
  {
    if (!goal_handle || !goal_handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<PickColor::Feedback>();
    feedback->phase = phase;
    feedback->detail = detail;
    goal_handle->publish_feedback(feedback);
  }

  // The automatic stack has no requested colour, so the two colour fields of the
  // result report the ends of the tower instead: `color` is the topmost colour
  // and `support_color` the one that stayed on the table.
  void finishAutoStackAction(
    const std::shared_ptr<GoalHandlePickColor> & goal_handle,
    uint16_t error_code, const std::string & message, bool success,
    const std::vector<std::string> & order)
  {
    auto result = std::make_shared<PickColor::Result>();
    result->success = success;
    result->error_code = error_code;
    result->message = message;
    result->color = order.empty() ? std::string{} : order.front();
    result->zone.clear();
    result->support_color = order.empty() ? std::string{} : order.back();
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto_stack_order_.clear();
      auto_stack_inventory_.clear();
      auto_stack_home_state_.reset();
    }
  }

  void cancelInternalStackGoal()
  {
    ClientGoalHandlePickColor::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(auto_stack_mutex_);
      handle = auto_stack_step_goal_;
    }
    if (handle) {
      pick_stack_client_->async_cancel_goal(handle);
    }
  }

  // Wait for one four-colour status of the requested filter generation and
  // return every physically valid cube it lists. A filter colour that is not on
  // the table is not an error here: that frame still reports the whole cube
  // list, just with valid=false and requested_color_physical_count=0.
  bool waitForColourInventory(
    const std::string & filter_color, uint64_t filter_generation,
    std::vector<DetectedCube> & cubes, uint16_t & error_code, std::string & reason)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(target_acquisition_timeout_s_);
    std::string last_reason = "no four-colour status with a cube list arrived";
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
        data.value("target_filter_color", "") == filter_color &&
        data.value("filter_generation", uint64_t{0}) == filter_generation &&
        data.contains("detected_targets"))
      {
        std::vector<DetectedCube> found;
        std::string parse_reason;
        if (detectedCubesFromStatus(
            status, filter_color, filter_generation, found, parse_reason))
        {
          cubes = found;
          return true;
        }
        last_reason = parse_reason;
      }
      std::this_thread::sleep_for(50ms);
    }
    if (canceled_.load()) {
      error_code = PickColor::Result::CANCELED;
      reason = "自动码垛在识别桌面方块颜色时被取消";
      return false;
    }
    error_code = PickColor::Result::PERCEPTION_UNAVAILABLE;
    reason = "自动码垛无法获得桌面方块颜色清单：" + last_reason;
    return false;
  }

  // The colours of the cubes lying on the table right now, in the fixed
  // automatic order with the future top of the tower first. A colour that
  // appears twice on the table is refused instead of guessed, and so is a set of
  // colours that cannot form a tower within maximum_stack_cubes.
  bool discoverAutoStackOrder(
    std::vector<std::string> & order, std::vector<DetectedCube> & inventory,
    uint16_t & error_code, std::string & reason)
  {
    const std::string filter_color = kAutoStackOrder.front();
    uint64_t filter_generation = 0;
    if (!setPerceptionTargetColor(filter_color, filter_generation, reason)) {
      error_code = PickColor::Result::PERCEPTION_UNAVAILABLE;
      reason = "自动码垛无法设置感知颜色过滤器：" + reason;
      return false;
    }
    std::vector<DetectedCube> cubes;
    if (!waitForColourInventory(
        filter_color, filter_generation, cubes, error_code, reason))
    {
      return false;
    }
    // Only cubes on the table can be picked up; a cube that is already part of a
    // tower is somebody else's work and is reported at a higher tier.
    std::map<std::string, int> tabletop_counts;
    for (const auto & cube : cubes) {
      if (cube.stack_tier == 0) {
        ++tabletop_counts[cube.color];
      }
    }
    std::vector<std::string> available;
    for (const char * color : kAutoStackOrder) {
      const int count = tabletop_counts[color];
      if (count > 1) {
        error_code = PickColor::Result::TARGET_NOT_UNIQUE;
        reason = "桌面上有 " + std::to_string(count) + " 个" + colorLabel(color) +
          "方块，自动码垛无法确定该用哪一个";
        return false;
      }
      if (count == 1) {
        available.emplace_back(color);
      }
    }
    if (available.size() < 2) {
      error_code = PickColor::Result::TARGET_NOT_FOUND;
      reason = "桌面上只有 " + std::to_string(available.size()) +
        " 种颜色的方块，至少需要两种颜色才能码垛";
      return false;
    }
    if (available.size() > static_cast<std::size_t>(maximum_stack_cubes_)) {
      error_code = PickColor::Result::STACK_GEOMETRY_REJECTED;
      reason = "桌面上有 " + std::to_string(available.size()) +
        " 种颜色的方块，超过 maximum_stack_cubes=" +
        std::to_string(maximum_stack_cubes_) +
        " 的塔高上限；请抬高该参数或先拿走多余的方块";
      return false;
    }
    order = available;
    inventory = cubes;
    recordEvent(
      "auto_stack_order_locked",
      "top_to_bottom=" + joinAutoStackOrder(order) +
      ",filter_generation=" + std::to_string(filter_generation));
    return true;
  }

  std::string autoStackSnapshotStatus(
    const std::vector<DetectedCube> & cubes, const std::string & source_color) const
  {
    nlohmann::json targets = nlohmann::json::array();
    int source_count = 0;
    for (const auto & cube : cubes) {
      if (cube.color == source_color) {
        ++source_count;
      }
      targets.push_back(
      {
        {"color", cube.color},
        {"center_base_m", {cube.center.x, cube.center.y, cube.center.z}},
        {"top_z_base_m", cube.top_z},
        {"grasp_axis_yaw_base_rad", cube.grasp_axis_yaw_rad},
        {"grasp_axis_yaw_period_rad", cube.grasp_axis_yaw_period_rad},
        {"nominal_object_size_m", cube.object_size_m},
        {"stack_tier", cube.stack_tier}});
    }
    return nlohmann::json(
    {
      {"detection_mode", "colors"},
      {"target_filter_color", source_color},
      {"filter_generation", uint64_t{0}},
      {"requested_color_physical_count", source_count},
      {"detected_targets", targets},
      {"snapshot_source", "pick_stack_auto_initial_inventory"}}).dump();
  }

  // Lock one internal layer from the inventory captured before the first arm
  // motion.  The source cubes keep their measured tabletop poses.  After each
  // successful placement advanceAutoStackInventory() moves only the new tower
  // top, so no camera revisit or return to the camera pose is needed.
  bool lockAutoStackPairFromInventory(
    const std::string & source_color, const std::string & support_color,
    uint16_t & error_code, std::string & reason)
  {
    std::vector<DetectedCube> cubes;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cubes = auto_stack_inventory_;
    }
    const auto matches = [&cubes](const std::string & color) {
        return static_cast<int>(std::count_if(
                 cubes.begin(), cubes.end(), [&color](const DetectedCube & cube) {
                   return cube.color == color;
                 }));
      };
    const int source_matches = matches(source_color);
    const int support_matches = matches(support_color);
    if (source_matches != 1 || support_matches != 1) {
      error_code = (source_matches > 1 || support_matches > 1) ?
        PickColor::Result::TARGET_NOT_UNIQUE : PickColor::Result::TARGET_NOT_FOUND;
      reason = "首次采集的自动码垛清单不再包含唯一的 " + source_color +
        " 源方块和 " + support_color + " 支撑方块";
      return false;
    }
    const auto source = std::find_if(
      cubes.begin(), cubes.end(), [&source_color](const DetectedCube & cube) {
        return cube.color == source_color;
      });
    const auto support = std::find_if(
      cubes.begin(), cubes.end(), [&support_color](const DetectedCube & cube) {
        return cube.color == support_color;
      });
    if (source->stack_tier != 0) {
      error_code = PickColor::Result::STACK_GEOMETRY_REJECTED;
      reason = "首次采集的 " + source_color + " 源方块不在桌面第 0 层";
      return false;
    }
    const auto & point = source->center;
    if (point.x < target_x_bounds_[0] || point.x > target_x_bounds_[1] ||
      point.y<target_y_bounds_[0] || point.y> target_y_bounds_[1] ||
      point.z<target_z_bounds_[0] || point.z> target_z_bounds_[1])
    {
      error_code = PickColor::Result::PRECONDITION_FAILED;
      reason = "首次采集的 " + source_color + " 方块超出抓取边界";
      return false;
    }
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = base_frame_;
    target.header.stamp = node_->now();
    target.pose.position = source->center;
    target.pose.orientation.z = std::sin(source->grasp_axis_yaw_rad * 0.5);
    target.pose.orientation.w = std::cos(source->grasp_axis_yaw_rad * 0.5);
    const std::string status = autoStackSnapshotStatus(cubes, source_color);
    if (!lockTargetSnapshot(
        target, source->object_size_m, status, source_color, reason))
    {
      error_code = PickColor::Result::PRECONDITION_FAILED;
      return false;
    }
    if (!lockStackDestinationSnapshot(cubes, *support, reason)) {
      error_code = PickColor::Result::STACK_GEOMETRY_REJECTED;
      return false;
    }
    if (!validateStackPair(reason)) {
      error_code = PickColor::Result::STACK_GEOMETRY_REJECTED;
      return false;
    }
    recordEvent(
      "auto_stack_pair_snapshot_reused",
      "source=" + source_color + ",support=" + support_color +
      ",inventory_from_first_rgbd_status=true,support_tier=" +
      std::to_string(support->stack_tier));
    return true;
  }

  bool advanceAutoStackInventory(
    const std::string & source_color, const std::string & support_color,
    std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto source = std::find_if(
      auto_stack_inventory_.begin(), auto_stack_inventory_.end(),
      [&source_color](const DetectedCube & cube) {return cube.color == source_color;});
    const auto support = std::find_if(
      auto_stack_inventory_.begin(), auto_stack_inventory_.end(),
      [&support_color](const DetectedCube & cube) {return cube.color == support_color;});
    if (source == auto_stack_inventory_.end() || support == auto_stack_inventory_.end() ||
      !locked_stack_destination_.valid ||
      locked_stack_destination_.color != support_color)
    {
      reason = "当前层与冻结清单中的源方块/支撑方块不一致";
      return false;
    }
    source->center.x = locked_stack_destination_.top_pose.position.x;
    source->center.y = locked_stack_destination_.top_pose.position.y;
    source->top_z = locked_stack_destination_.top_pose.position.z +
      source->object_size_m[2];
    source->center.z = source->top_z;
    source->grasp_axis_yaw_rad = locked_stack_destination_.grasp_axis_yaw_rad;
    source->grasp_axis_yaw_period_rad =
      locked_stack_destination_.grasp_axis_yaw_period_rad;
    source->stack_tier = locked_stack_destination_.stack_tier + 1;
    recordEvent(
      "auto_stack_inventory_advanced",
      "new_top=" + source_color + ",tier=" + std::to_string(source->stack_tier) +
      ",top_xyz_m=[" + std::to_string(source->center.x) + "," +
      std::to_string(source->center.y) + "," + std::to_string(source->top_z) + "]");
    return true;
  }

  // Run one single-cube stack as an internal pick_stack goal. The step is a real
  // action goal, so it goes through exactly the same preconditions, geometry
  // checks and recovery as an operator-issued request.
  bool runAutoStackStep(
    const std::shared_ptr<GoalHandlePickColor> & auto_goal,
    const std::string & upper_color, const std::string & lower_color,
    int step, int total_steps,
    AutoStackStepOutcome & outcome, std::string & reason)
  {
    // The internal task reads these before it snapshots its start state.  Steps
    // are serialized by this function, so they describe exactly the goal that
    // is about to be sent.
    auto_stack_step_number_.store(step);
    auto_stack_step_count_.store(total_steps);
    if (!pick_stack_client_->wait_for_action_server(2s)) {
      outcome.error_code = PickColor::Result::INTERNAL_ERROR;
      reason = "内部 pick_stack Action 服务不可用，自动码垛无法继续";
      return false;
    }
    auto goal = PickColor::Goal();
    goal.color = upper_color;
    goal.zone = kAutoStackInternalZone;
    goal.support_color = lower_color;
    auto promise = std::make_shared<std::promise<AutoStackStepOutcome>>();
    auto future = promise->get_future();
    rclcpp_action::Client<PickColor>::SendGoalOptions options;
    options.goal_response_callback =
      [this, promise](const ClientGoalHandlePickColor::SharedPtr & handle) {
        if (handle) {
          std::lock_guard<std::mutex> lock(auto_stack_mutex_);
          auto_stack_step_goal_ = handle;
          return;
        }
        AutoStackStepOutcome rejected;
        rejected.error_code = PickColor::Result::BUSY;
        rejected.message = "内部 pick_stack 目标被拒绝：任务槽被其它请求占用";
        setPromiseValue(promise, rejected);
      };
    options.feedback_callback =
      [this, auto_goal, step, total_steps](
      ClientGoalHandlePickColor::SharedPtr,
      const std::shared_ptr<const PickColor::Feedback> feedback) {
        if (!feedback) {
          return;
        }
        publishAutoStackFeedback(
          auto_goal, "auto_stack_step",
          "第 " + std::to_string(step) + "/" + std::to_string(total_steps) + " 步：" +
          feedback->detail);
      };
    options.result_callback =
      [this, promise](const ClientGoalHandlePickColor::WrappedResult & result) {
        AutoStackStepOutcome value;
        if (result.result) {
          value.success = result.result->success;
          value.error_code = result.result->error_code;
          value.message = result.result->message;
        } else {
          value.message = "内部 pick_stack 没有返回结果";
        }
        if (result.code == rclcpp_action::ResultCode::CANCELED) {
          value.success = false;
          value.canceled = true;
          value.error_code = PickColor::Result::CANCELED;
          if (value.message.empty()) {
            value.message = "内部 pick_stack 已取消";
          }
        }
        setPromiseValue(promise, value);
        std::lock_guard<std::mutex> lock(auto_stack_mutex_);
        auto_stack_step_goal_.reset();
      };
    pick_stack_client_->async_send_goal(goal, options);

    // A step always ends with a result: pick_stack promises "one attempt, then
    // the slot is free again" on every path, including its own recovery. This
    // loop therefore only has to keep cancellation responsive.
    while (future.wait_for(100ms) != std::future_status::ready) {
      if (!rclcpp::ok()) {
        cancelInternalStackGoal();
        outcome.canceled = true;
        outcome.error_code = PickColor::Result::CANCELED;
        outcome.message = "ROS shutdown interrupted the internal pick_stack step";
        reason = outcome.message;
        return false;
      }
      if (auto_goal->is_canceling() || canceled_.load()) {
        cancelInternalStackGoal();
      }
    }
    outcome = future.get();
    return true;
  }

  void executeAutoStackAction(const std::shared_ptr<GoalHandlePickColor> & goal_handle)
  {
    // Clears the ownership flag on every exit path, so a failed or canceled
    // automatic stack never keeps the cell locked for the next request.
    struct RunningGuard
    {
      std::mutex & mutex;
      bool & running;
      std::atomic<bool> & active;
      ~RunningGuard()
      {
        active.store(false);
        std::lock_guard<std::mutex> lock(mutex);
        running = false;
      }
    } running_guard{auto_stack_mutex_, auto_stack_running_, auto_stack_active_};
    // Each single-cube task clears this flag when it starts; the automatic stack
    // does the same, so a cancellation that was meant for an earlier task cannot
    // abort this one. From here on a set flag means "stop the tower".
    canceled_.store(false);

    std::vector<std::string> order;
    std::vector<DetectedCube> inventory;
    const auto reject = [this, &goal_handle, &order](
      uint16_t error_code, const std::string & message) {
        finishAutoStackAction(goal_handle, error_code, message, false, order);
      };

    if (!stack_enabled_) {
      reject(
        PickColor::Result::PRECONDITION_FAILED,
        "自动码垛需要 stack_enabled=true：四色感知与方块几何都没有打开");
      return;
    }
    if (!execute_) {
      // Without real motion the tower never grows, so the second step would be
      // planned against a support cube that is still lying on the table. That
      // plan is not the plan of the tower, and reporting it as verified would be
      // a lie; refuse instead of planning something that cannot happen.
      reject(
        PickColor::Result::PRECONDITION_FAILED,
        "自动码垛必须实际执行（execute=true）：只规划时塔不会真的变高，"
        "第二步之后的支撑位置不成立；逐对预演请用 send_color_task.sh stack");
      return;
    }
    publishAutoStackFeedback(
      goal_handle, "auto_stack_discovery", "正在识别桌面上现有的方块颜色");
    uint16_t discovery_error = PickColor::Result::INTERNAL_ERROR;
    std::string reason;
    if (!discoverAutoStackOrder(order, inventory, discovery_error, reason)) {
      reject(discovery_error, reason);
      return;
    }
    moveit::core::RobotStatePtr auto_home_state;
    geometry_msgs::msg::Pose auto_home_pose;
    try {
      auto_home_state = currentStartState(false);
      auto_home_pose = move_group_->getCurrentPose(tip_frame_).pose;
    } catch (const std::exception & error) {
      reject(
        PickColor::Result::PRECONDITION_FAILED,
        std::string("自动码垛无法保存整项任务的起始关节位：") + error.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto_stack_order_ = order;
      auto_stack_inventory_ = inventory;
      // The final layer returns here.  Intermediate layers deliberately keep
      // the arm at the tower retreat pose so the next pick starts nearby.
      auto_stack_home_state_ = auto_home_state;
      auto_stack_home_pose_ = auto_home_pose;
    }

    const int total_steps = static_cast<int>(order.size()) - 1;
    const std::string planned = joinAutoStackOrder(order);
    const auto canceled_message = [&order, &planned, total_steps](int completed_steps) {
        return "自动码垛已取消：计划自上而下 " + planned + " 共 " +
               std::to_string(total_steps + 1) + " 层，已完成 " +
               std::to_string(completed_steps) + " 步，塔上现有 " +
               std::to_string(completed_steps + 1) + " 个方块（自下而上 " +
               standingTowerText(order, static_cast<std::size_t>(completed_steps) + 1) + "）";
      };

    for (int step = 0; step < total_steps; ++step) {
      // `is_canceling` catches a cancel request on this goal; `canceled_` also
      // catches the operator cancel service, which reaches the arm and the
      // running step but not this goal handle.
      if (goal_handle->is_canceling() || canceled_.load()) {
        finishAutoStackAction(
          goal_handle, PickColor::Result::CANCELED, canceled_message(step), false, order);
        return;
      }
      std::string upper = order.front();
      std::string lower = order.back();
      autoStackStepColors(order, static_cast<std::size_t>(step), upper, lower);
      publishAutoStackFeedback(
        goal_handle, "auto_stack_step",
        "第 " + std::to_string(step + 1) + "/" + std::to_string(total_steps) + " 步：" +
        colorLabel(upper) + "放到" + colorLabel(lower) + "上面");
      AutoStackStepOutcome outcome;
      std::string step_reason;
      if (!runAutoStackStep(
          goal_handle, upper, lower, step + 1, total_steps, outcome, step_reason))
      {
        finishAutoStackAction(
          goal_handle,
          outcome.error_code == PickColor::Result::OK ?
          PickColor::Result::INTERNAL_ERROR : outcome.error_code,
          step_reason, false, order);
        return;
      }
      if (!outcome.success) {
        if (outcome.canceled || goal_handle->is_canceling()) {
          finishAutoStackAction(
            goal_handle, PickColor::Result::CANCELED, canceled_message(step), false, order);
          return;
        }
        finishAutoStackAction(
          goal_handle, outcome.error_code,
          "自动码垛在第 " + std::to_string(step + 1) + "/" + std::to_string(total_steps) +
          " 步失败（" + colorLabel(upper) + "放到" + colorLabel(lower) + "）：" +
          outcome.message + "。计划自上而下 " + planned + " 共 " +
          std::to_string(total_steps + 1) + " 层，已完成 " + std::to_string(step) +
          " 步，塔上现有 " + std::to_string(step + 1) + " 个方块（自下而上 " +
          standingTowerText(order, static_cast<std::size_t>(step) + 1) +
          "）；未自动重试，必要时可用 pick_stack 手工继续",
          false, order);
        return;
      }
      if (!advanceAutoStackInventory(upper, lower, reason)) {
        finishAutoStackAction(
          goal_handle, PickColor::Result::INTERNAL_ERROR,
          "自动码垛已完成第 " + std::to_string(step + 1) +
          " 步，但无法更新首次采集的塔顶坐标：" + reason,
          false, order);
        return;
      }
    }
    finishAutoStackAction(
      goal_handle, PickColor::Result::OK,
      "自动码垛完成：自上而下 " + planned + "，共 " + std::to_string(order.size()) +
      " 层，机械臂已回到起始关节位",
      true, order);
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
    sort_area_observation_joints_rad_ = declare_vector(
      "sort_area_observation_joints_rad", std::vector<double>{});
    cube_id_ = declare_string("cube_id", "target_cube");
    stack_support_id_ = declare_string("stack_support_id", "stack_support_cube");
    ground_plane_id_ = declare_string("ground_plane_id", "rviz_ground_plane");
    log_directory_ = declare_string("log_directory", "evidence/P5/runs");
    p6_gate_file_ = declare_string(
      "p6_gate_file", "/home/industio/fairino_ws/evidence/P6/p6_execution_gate.txt");
    sort_enabled_ = node_->declare_parameter<bool>("sort_enabled", false);
    stack_enabled_ = node_->declare_parameter<bool>("stack_enabled", false);
    maximum_stack_placement_rotation_rad_ = declare_double(
      "maximum_stack_placement_rotation_rad", kPi * 0.25);
    stack_approach_clearance_m_ = declare_double("stack_approach_clearance_m", 0.030);
    stack_height_tolerance_m_ = declare_double(
      "stack_height_tolerance_m", kStackTabletopToleranceDefault);
    stack_footprint_tolerance_m_ = declare_double(
      "stack_footprint_tolerance_m", 0.020);
    maximum_stack_cubes_ = node_->declare_parameter<int>(
      "maximum_stack_cubes", kMaximumStackCubesDefault);
    maximum_stack_tier_ = maximum_stack_cubes_ - 1;
    sort_grid_file_ = declare_string("sort_grid_file", "");
    for (const auto & color : {"red", "green", "blue", "yellow"}) {
      place_centers_[color] = declare_vector(
        std::string("place_") + color + "_center_m", std::vector<double>{});
      place_slots_[color] = declare_vector(
        std::string("place_") + color + "_slots_m", std::vector<double>{});
    }
    // pick_place resolves its target from the calibrated grid by region name, so
    // the placement target never depends on the detected colour.
    for (const auto & zone : node_->declare_parameter<std::vector<std::string>>(
        "place_zone_names", std::vector<std::string>{}))
    {
      zone_centers_[zone] = declare_vector(
        std::string("place_zone_") + zone + "_center_m", std::vector<double>{});
      zone_slots_[zone] = declare_vector(
        std::string("place_zone_") + zone + "_slots_m", std::vector<double>{});
    }
    zone_slot_clearance_m_ = declare_double("zone_slot_clearance_m", 0.025);
    release_bottom_gap_m_ = declare_double("release_bottom_gap_m", 0.005);
    batch_empty_timeout_s_ = declare_double("batch_empty_timeout_s", 10.0);
    batch_target_match_distance_m_ = declare_double(
      "batch_target_match_distance_m", 0.025);

    execute_ = node_->declare_parameter<bool>("execute", false);
    approach_clearance_m_ = declare_double("approach_clearance_m", 0.010);
    grasp_depth_m_ = declare_double("grasp_depth_m", 0.010);
    grasp_tilt_candidates_rad_ = declare_vector(
      "grasp_tilt_candidates_rad",
      {0.0, 10.0 * kPi / 180.0, -10.0 * kPi / 180.0,
        20.0 * kPi / 180.0, -20.0 * kPi / 180.0,
        30.0 * kPi / 180.0, -30.0 * kPi / 180.0});
    candidate_planning_time_s_ = declare_double("candidate_planning_time_s", 0.25);
    candidate_planning_attempts_ = node_->declare_parameter<int>(
      "candidate_planning_attempts", 1);
    candidate_ik_timeout_s_ = declare_double("candidate_ik_timeout_s", 0.05);
    grasp_contact_offset_xyz_m_ = declare_vector(
      "grasp_contact_offset_xyz_m", {0.0, 0.0, 0.0});
    lift_distance_m_ = declare_double("lift_distance_m", 0.050);
    pre_open_position_m_ = declare_double("pre_open_position_m", 0.060);
    close_position_m_ = declare_double("close_position_m", 0.027);
    pre_open_command_repetitions_ = node_->declare_parameter<int>(
      "pre_open_command_repetitions", 1);
    release_command_repetitions_ = node_->declare_parameter<int>(
      "release_command_repetitions", 2);
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
    manual_pose_x_bounds_ = declare_vector(
      "manual_pose_x_bounds_m", {-0.45, 0.45});
    manual_pose_y_bounds_ = declare_vector(
      "manual_pose_y_bounds_m", {-0.80, -0.10});
    manual_pose_z_bounds_ = declare_vector(
      "manual_pose_z_bounds_m", {0.12, 0.65});
    manual_pose_ik_timeout_s_ = declare_double("manual_pose_ik_timeout_s", 0.25);
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
    validated_segment_start_tolerance_rad_ = declare_double(
      "validated_segment_start_tolerance_rad", 0.010);
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
    p6_gate_max_age_s_ = declare_double("p6_gate_max_age_s", 360000.0);

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
      cube_id_.empty() || stack_support_id_.empty() || ground_plane_id_.empty() ||
      cube_id_ == stack_support_id_ || cube_id_ == stackColumnId() ||
      cube_id_ == stackSourceColumnId() ||
      ground_plane_id_ == stack_support_id_ || ground_plane_id_ == stackColumnId() ||
      ground_plane_id_ == stackSourceColumnId() ||
      (use_target_size_ && target_size_topic_.empty()))
    {
      throw std::runtime_error("frame, group, action and scene object names must not be empty");
    }
    if (!positive(approach_clearance_m_) || !positive(grasp_depth_m_) ||
      !positive(candidate_planning_time_s_) ||
      !positive(candidate_ik_timeout_s_) || !positive(manual_pose_ik_timeout_s_) ||
      !positive(lift_distance_m_) ||
      !positive(release_bottom_gap_m_) ||
      !positive(cartesian_step_m_) || !positive(planning_time_s_) ||
      !positive(target_message_timeout_s_) || !positive(action_timeout_s_) ||
      !positive(target_acquisition_timeout_s_) ||
      !positive(state_timeout_s_) || !positive(gripper_state_tolerance_m_) ||
      !positive(ground_plane_thickness_m_) || !positive(maximum_cube_plane_gap_m_) ||
      !positive(maximum_cartesian_joint_step_rad_) ||
      !positive(validated_segment_start_tolerance_rad_) ||
      !positive(p6_gate_max_age_s_) ||
      !positive(batch_empty_timeout_s_) ||
      !positive(batch_target_match_distance_m_) ||
      planning_attempts_ < 1 || candidate_planning_attempts_ < 1 ||
      pre_open_command_repetitions_ < 1 ||
      pre_open_command_repetitions_ > 3 || release_command_repetitions_ < 1 ||
      release_command_repetitions_ > 3)
    {
      throw std::runtime_error("distance, timeout and planning parameters must be positive");
    }
    if (!finiteVector(
        grasp_tilt_candidates_rad_, grasp_tilt_candidates_rad_.size()) ||
      grasp_tilt_candidates_rad_.empty() ||
      std::none_of(
        grasp_tilt_candidates_rad_.begin(), grasp_tilt_candidates_rad_.end(),
        [](double value) {return std::abs(value) <= 1e-12;}) ||
      std::any_of(
        grasp_tilt_candidates_rad_.begin(), grasp_tilt_candidates_rad_.end(),
        [](double value) {return std::abs(value) > 30.0 * kPi / 180.0 + 1e-9;}))
    {
      throw std::runtime_error(
              "grasp_tilt_candidates_rad must be a finite non-empty list, include "
              "vertical 0 rad, and stay within +/-30 degrees");
    }
    if (!finiteVector(object_size_m_, 3) ||
      std::any_of(
        object_size_m_.begin(), object_size_m_.end(),
        [](double value) {return value <= 0.0;}))
    {
      throw std::runtime_error("object_size_m must contain three positive finite dimensions");
    }
    if (!sort_area_observation_joints_rad_.empty() &&
      !finiteVector(sort_area_observation_joints_rad_, 6))
    {
      throw std::runtime_error(
              "sort_area_observation_joints_rad must contain six finite joint values");
    }
    if (!finiteVector(grasp_contact_offset_xyz_m_, 3) ||
      std::sqrt(
        grasp_contact_offset_xyz_m_[0] * grasp_contact_offset_xyz_m_[0] +
        grasp_contact_offset_xyz_m_[1] * grasp_contact_offset_xyz_m_[1] +
        grasp_contact_offset_xyz_m_[2] * grasp_contact_offset_xyz_m_[2]) > 0.055 + 1e-9)
    {
      throw std::runtime_error(
              "grasp_contact_offset_xyz_m must be a finite XYZ vector no longer than 55 mm");
    }
    if (sort_enabled_ && std::any_of(
        grasp_contact_offset_xyz_m_.begin(), grasp_contact_offset_xyz_m_.end(),
        [](double value) {return std::abs(value) > 1e-12;}))
    {
      throw std::runtime_error(
              "nonzero grasp_contact_offset_xyz_m is currently limited to pick_hold tests");
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
    for (const auto * bounds : {
      & manual_pose_x_bounds_, & manual_pose_y_bounds_, & manual_pose_z_bounds_
    })
    {
      if (!finiteVector(*bounds, 2) || (*bounds)[0] >= (*bounds)[1]) {
        throw std::runtime_error(
                "all manual_pose bounds must contain increasing finite min/max pairs");
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
    if (!positive(maximum_stack_placement_rotation_rad_) ||
      maximum_stack_placement_rotation_rad_ > kPi * 0.5 ||
      !positive(stack_approach_clearance_m_) ||
      !positive(stack_height_tolerance_m_) ||
      !positive(stack_footprint_tolerance_m_))
    {
      throw std::runtime_error(
              "stack rotation limit, approach clearance and tolerances must be positive "
              "and the rotation limit must not exceed half a turn of yaw symmetry");
    }
    if (maximum_stack_cubes_ < 2 || maximum_stack_cubes_ > kMaximumStackCubesLimit) {
      throw std::runtime_error(
              "maximum_stack_cubes must be between 2 and " +
              std::to_string(kMaximumStackCubesLimit) +
              ": it counts the cubes of one tower, including the one on the table");
    }
    if (stack_enabled_ && !sort_enabled_) {
      throw std::runtime_error(
              "stack_enabled requires sort_enabled so the four-color "
              "perception mode and cube geometry are configured");
    }
    if (stack_enabled_ && !use_target_yaw_) {
      throw std::runtime_error(
              "stack_enabled requires use_target_yaw: without a measured top-face yaw "
              "the upper cube cannot be aligned with the lower cube");
    }
    if (stack_enabled_ && !use_target_size_) {
      throw std::runtime_error(
              "stack_enabled requires use_target_size: the stack geometry needs the "
              "measured cube height");
    }
    if (stack_enabled_) {
      // The allowance has to cover the smallest symmetry step that can occur at
      // run time, not the configured fallback. With use_target_size enabled the
      // locked step is derived from the measured top shape: a square top uses
      // half a turn of symmetry steps (kPi * 0.5) and a 60 x 30 mm top uses a
      // full turn. `target_yaw_symmetry_rad` only applies when no measured size
      // is available, so it bounds the smallest possible step from below.
      //
      // Half the smallest step is exactly the worst case the reduction can
      // leave: graspCandidates() wraps its candidates, so the candidates are one
      // whole step apart in the reduced space and the nearest of them is never
      // further than half a step from the requested placement rotation. A limit
      // below that half step would refuse even two cubes that already share a
      // yaw, which is the failure this guard exists to prevent.
      const double symmetry = std::min(target_yaw_symmetry_rad_, kPi * 0.5);
      if (!(symmetry > 0.0) ||
        maximum_stack_placement_rotation_rad_ < 0.5 * symmetry - 1e-9)
      {
        throw std::runtime_error(
                "maximum_stack_placement_rotation_rad must allow at least half the "
                "smallest run-time yaw-symmetry step (" +
                std::to_string(0.5 * symmetry) +
                " rad for this configuration), otherwise rotating a cube onto the "
                "same yaw would always be refused");
      }
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
      for (const auto & entry : zone_centers_) {
        const auto & zone = entry.first;
        const auto center = entry.second;
        const auto slots = zone_slots_.find(zone);
        if (!finiteVector(center, 3) ||
          std::abs(center[2] - ground_plane_z_m_) > 0.001)
        {
          throw std::runtime_error(
                  "invalid " + zone + " placement zone center or grid plane Z");
        }
        if (slots == zone_slots_.end() || slots->second.empty() ||
          slots->second.size() % 3 != 0 ||
          !finiteVector(slots->second, slots->second.size()))
        {
          throw std::runtime_error("invalid " + zone + " placement zone slots");
        }
        for (std::size_t index = 2; index < slots->second.size(); index += 3) {
          if (std::abs(slots->second[index] - ground_plane_z_m_) > 0.001) {
            throw std::runtime_error(
                    "invalid " + zone + " placement zone slot plane Z");
          }
        }
      }
    }
    if (!isFinite(zone_slot_clearance_m_) || zone_slot_clearance_m_ <= 0.0) {
      throw std::runtime_error("zone_slot_clearance_m must be a positive length");
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
    // Do not require the source cube bottom to coincide with the RViz/A4
    // ground plane. In the real cell source cubes can sit on a raised fixture
    // while sort destinations remain on the calibrated Z=0 paper plane. The
    // broad target Z bounds and MoveIt collision checks still apply.
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

  std::string calibratedZoneList() const
  {
    std::string list;
    for (const auto & entry : zone_centers_) {
      if (!list.empty()) {
        list += ", ";
      }
      list += entry.first;
    }
    return list.empty() ? std::string("(none)") : list;
  }

  // ---------------------------------------------------------------------
  // pick_stack target acquisition
  //
  // The source-colour filter stabilizes the cube that will be grasped, while
  // `detected_targets` in that very same RGB-D status still describes every
  // physically valid coloured cube in view. pick_stack accepts only a status
  // that contains exactly one source and one support cube, then freezes both
  // poses before planning. No perception result is consulted after motion.
  // ---------------------------------------------------------------------

  // Parse the cubes the perception node reports for the frame that also
  // produced the selected target. Entries are only trusted while the status
  // belongs to the expected filter generation.
  bool detectedCubesFromStatus(
    const std::string & status, const std::string & requested_color,
    uint64_t filter_generation, std::vector<DetectedCube> & cubes,
    std::string & reason) const
  {
    cubes.clear();
    const auto data = nlohmann::json::parse(status, nullptr, false);
    if (data.is_discarded() || !data.is_object() ||
      data.value("detection_mode", "") != "colors")
    {
      reason = "no four-color perception status is available";
      return false;
    }
    if (!requested_color.empty()) {
      if (data.value("target_filter_color", std::string{}) != requested_color ||
        data.value("filter_generation", uint64_t{0}) != filter_generation)
      {
        reason = "perception status belongs to a different colour filter generation";
        return false;
      }
    }
    if (!data.contains("detected_targets") || !data["detected_targets"].is_array()) {
      reason = "perception status carries no per-cube target list";
      return false;
    }
    for (const auto & entry : data["detected_targets"]) {
      if (!entry.is_object() || !entry.contains("color") ||
        !entry["color"].is_string() || !entry.contains("center_base_m") ||
        !entry["center_base_m"].is_array() || entry["center_base_m"].size() != 3)
      {
        reason = "perception reported a malformed per-cube target entry";
        return false;
      }
      DetectedCube cube;
      cube.color = normalizedColor(entry["color"].get<std::string>());
      if (!supportedColor(cube.color)) {
        reason = "perception reported an unsupported colour " + cube.color;
        return false;
      }
      std::vector<double> center;
      for (const auto & value : entry["center_base_m"]) {
        if (!value.is_number()) {
          reason = "perception reported a non-numeric cube centre";
          return false;
        }
        center.push_back(value.get<double>());
      }
      if (!finiteVector(center, 3)) {
        reason = "perception reported a non-finite cube centre";
        return false;
      }
      cube.center.x = center[0];
      cube.center.y = center[1];
      cube.center.z = center[2];
      cube.top_z = entry.value("top_z_base_m", center[2]);
      cube.grasp_axis_yaw_rad = entry.value("grasp_axis_yaw_base_rad", 0.0);
      cube.grasp_axis_yaw_period_rad = entry.value(
        "grasp_axis_yaw_period_rad", kPi * 0.5);
      cube.stack_tier = entry.value("stack_tier", 0);
      if (!isFinite(cube.top_z) || !isFinite(cube.grasp_axis_yaw_rad) ||
        !(cube.grasp_axis_yaw_period_rad > 0.0) ||
        !isFinite(cube.grasp_axis_yaw_period_rad) ||
        cube.grasp_axis_yaw_period_rad > 2.0 * kPi ||
        cube.stack_tier < 0 || cube.stack_tier > maximum_stack_tier_)
      {
        reason = "perception reported an unusable cube orientation, or a stack tier "
          "above the configured maximum_stack_cubes of " +
          std::to_string(maximum_stack_cubes_);
        return false;
      }
      cube.object_size_m = object_size_m_;
      if (entry.contains("nominal_object_size_m") &&
        entry["nominal_object_size_m"].is_array() &&
        entry["nominal_object_size_m"].size() == 3)
      {
        std::vector<double> size;
        for (const auto & value : entry["nominal_object_size_m"]) {
          if (!value.is_number()) {
            reason = "perception reported a non-numeric cube size";
            return false;
          }
          size.push_back(value.get<double>());
        }
        if (finiteVector(size, 3) &&
          std::all_of(size.begin(), size.end(), [](double value) {return value > 0.0;}))
        {
          cube.object_size_m = size;
        }
      }
      cubes.push_back(cube);
    }
    return true;
  }

  // The cube that stays behind becomes the lower cube of the new pair. It may
  // A reported tier must agree with the measured bottom height for either
  // cube in a stack pair. Only a destination has the separate capacity limit.
  bool stackCubeTierHeightAcceptable(
    const DetectedCube & cube, const std::string & role, std::string & reason) const
  {
    const double bottom = cube.top_z - cube.object_size_m[2];
    const double plane_gap = bottom - ground_plane_z_m_;
    if (cube.stack_tier == 0) {
      // A cube on the table is measured against the plane itself, which is the
      // tightest check available and the one the two-cube stack always used.
      if (plane_gap < -maximum_cube_plane_gap_m_ ||
        plane_gap > maximum_cube_plane_gap_m_)
      {
        std::ostringstream message;
        message << "the " << cube.color << " cube that should stay on the table is "
                << "not resting on the table plane: bottom_gap_m=" << plane_gap;
        reason = message.str();
        return false;
      }
      return true;
    }
    // A cube reported at tier k sits on k cubes, so its bottom has to be k cube
    // heights above the plane. Real cubes are a little shorter or taller than
    // their nominal size and every added cube carries its own estimate, so the
    // check is on the average height the tier implies rather than on one
    // absolute millimetre figure that would drift with the tower.
    const double implied_cube_height = plane_gap / cube.stack_tier;
    if (implied_cube_height < cube.object_size_m[2] - stack_height_tolerance_m_ ||
      implied_cube_height > cube.object_size_m[2] + stack_height_tolerance_m_)
    {
      std::ostringstream message;
      message << "the " << cube.color << " " << role << " cube is reported at tier "
              << cube.stack_tier << " but its bottom is measured " << plane_gap
              << " m above the table plane, which implies "
              << implied_cube_height << " m per stacked cube instead of the nominal "
              << cube.object_size_m[2] << " m";
      reason = message.str();
      return false;
    }
    return true;
  }

  bool stackSupportTierAcceptable(const DetectedCube & cube, std::string & reason) const
  {
    if (cube.stack_tier + 1 >= maximum_stack_cubes_) {
      reason = "the " + cube.color + " support cube already sits at tier " +
        std::to_string(cube.stack_tier) + ", so placing on it would make a " +
        std::to_string(cube.stack_tier + 2) + "-cube tower and maximum_stack_cubes is " +
        std::to_string(maximum_stack_cubes_);
      return false;
    }
    return stackCubeTierHeightAcceptable(cube, "support", reason);
  }

  // The lower cube must be a cube, and its top must be free. This is the check
  // that stops "stack A on B" from silently becoming a taller tower than the
  // caller asked for when B already carries a cube.
  bool lockStackDestinationSnapshot(
    const std::vector<DetectedCube> & cubes, const DetectedCube & cube,
    std::string & reason)
  {
    if (!stackSupportTierAcceptable(cube, reason)) {
      return false;
    }
    if (std::abs(cube.object_size_m[0] - cube.object_size_m[1]) > 0.003 ||
      std::abs(cube.object_size_m[1] - cube.object_size_m[2]) > 0.003)
    {
      reason = "pick_stack currently accepts only equal-size cubes";
      return false;
    }
    for (const auto & other : cubes) {
      if (other.color == cube.color &&
        positionDistance(other.center, cube.center) < 1e-9)
      {
        continue;
      }
      const double xy = std::hypot(
        other.center.x - cube.center.x, other.center.y - cube.center.y);
      if (xy <= stack_footprint_tolerance_m_ &&
        other.center.z >= cube.top_z + 0.5 * cube.object_size_m[2])
      {
        reason = "another detected cube already sits on the " + cube.color +
          " support cube; pick_stack adds to a tower only from its top face";
        return false;
      }
    }
    StackDestination destination;
    destination.valid = true;
    destination.color = cube.color;
    destination.top_pose.position = cube.center;
    destination.top_pose.orientation.z = std::sin(cube.grasp_axis_yaw_rad * 0.5);
    destination.top_pose.orientation.w = std::cos(cube.grasp_axis_yaw_rad * 0.5);
    destination.object_size_m = cube.object_size_m;
    destination.grasp_axis_yaw_rad = cube.grasp_axis_yaw_rad;
    destination.grasp_axis_yaw_period_rad = cube.grasp_axis_yaw_period_rad;
    destination.stack_tier = cube.stack_tier;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      locked_stack_destination_ = destination;
    }
    std::ostringstream detail;
    detail << std::setprecision(9) << "color=" << cube.color
           << ",top_xyz_m=[" << cube.center.x << "," << cube.center.y << ","
           << cube.center.z << "],yaw_rad=" << cube.grasp_axis_yaw_rad
           << ",tier=" << cube.stack_tier
           << ",bottom_z_m=" << cube.top_z - cube.object_size_m[2]
           << ",tower_cubes_after_placement=" << cube.stack_tier + 2;
    recordEvent("stack_destination_locked", detail.str());
    return true;
  }

  // Lock the support from the exact status that was already used to lock the
  // source pose. This is deliberately not another perception request: the
  // camera is still at the initial pose and both coordinates share one frame.
  bool lockStackDestinationFromSourceSnapshot(
    const std::string & support_color, const std::string & source_color,
    uint64_t filter_generation, uint16_t & error_code, std::string & reason)
  {
    std::string status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status = locked_target_status_;
    }
    std::vector<DetectedCube> cubes;
    if (!detectedCubesFromStatus(
        status, source_color, filter_generation, cubes, reason))
    {
      error_code = PickColor::Result::PERCEPTION_UNAVAILABLE;
      reason = "the source snapshot cannot provide the complete stack pair: " + reason;
      return false;
    }
    int source_matches = 0;
    int support_matches = 0;
    DetectedCube support;
    for (const auto & cube : cubes) {
      if (cube.color == source_color) {
        ++source_matches;
      }
      if (cube.color == support_color) {
        ++support_matches;
        support = cube;
      }
    }
    if (source_matches != 1) {
      error_code = source_matches == 0 ? PickColor::Result::TARGET_NOT_FOUND :
        PickColor::Result::TARGET_NOT_UNIQUE;
      reason = "the initial snapshot must contain exactly one " + source_color +
        " source cube; count=" + std::to_string(source_matches);
      return false;
    }
    if (support_matches != 1) {
      error_code = support_matches == 0 ? PickColor::Result::TARGET_NOT_FOUND :
        PickColor::Result::TARGET_NOT_UNIQUE;
      reason = "the initial snapshot must contain exactly one " + support_color +
        " support cube; count=" + std::to_string(support_matches);
      return false;
    }
    if (!lockStackDestinationSnapshot(cubes, support, reason)) {
      error_code = PickColor::Result::STACK_GEOMETRY_REJECTED;
      return false;
    }
    recordEvent(
      "stack_pair_snapshot_locked",
      "source=" + source_color + ",support=" + support_color +
      ",filter_generation=" + std::to_string(filter_generation) +
      ",same_rgbd_status=true");
    return true;
  }

  bool lockVisibleStackSupport(
    const std::string & support_color, uint64_t filter_generation,
    uint16_t & error_code, std::string & reason)
  {
    std::string status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status = locked_target_status_;
    }
    std::vector<DetectedCube> cubes;
    if (!detectedCubesFromStatus(
        status, support_color, filter_generation, cubes, reason))
    {
      error_code = PickColor::Result::PERCEPTION_UNAVAILABLE;
      return false;
    }
    const auto matches = std::count_if(
      cubes.begin(), cubes.end(), [&support_color](const DetectedCube & cube) {
        return cube.color == support_color;
      });
    if (matches != 1) {
      error_code = matches == 0 ? PickColor::Result::TARGET_NOT_FOUND :
        PickColor::Result::TARGET_NOT_UNIQUE;
      reason = "start-view stack support count=" + std::to_string(matches);
      return false;
    }
    const auto support = std::find_if(
      cubes.begin(), cubes.end(), [&support_color](const DetectedCube & cube) {
        return cube.color == support_color;
      });
    if (!lockStackDestinationSnapshot(cubes, *support, reason)) {
      error_code = PickColor::Result::STACK_GEOMETRY_REJECTED;
      return false;
    }
    recordEvent("stack_support_start_view_locked", "color=" + support_color);
    return true;
  }

  // Decide whether the pair ordered by the caller can physically be stacked in
  // the direction the caller asked for, objecting before any arm motion.
  bool validateStackPair(std::string & reason, bool cross_view = false)
  {
    StackDestination destination;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      destination = locked_stack_destination_;
    }
    if (!destination.valid) {
      reason = "the stack support cube was never snapshotted";
      return false;
    }
    std::string status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status = locked_target_status_;
    }
    const auto data = nlohmann::json::parse(status, nullptr, false);
    const uint64_t generation = data.is_object() ?
      data.value("filter_generation", uint64_t{0}) : uint64_t{0};
    std::vector<DetectedCube> cubes;
    if (!detectedCubesFromStatus(status, locked_color_, generation, cubes, reason)) {
      reason = "cannot validate the frozen stack pair: " + reason;
      return false;
    }
    const auto source = std::find_if(
      cubes.begin(), cubes.end(), [this](const DetectedCube & cube) {
        return cube.color == locked_color_;
      });
    if (source == cubes.end()) {
      reason = "the frozen snapshot has no " + locked_color_ + " source cube";
      return false;
    }
    if (!stackCubeTierHeightAcceptable(*source, "source", reason)) {
      return false;
    }
    for (const auto & other : cubes) {
      if (other.color == source->color &&
        positionDistance(other.center, source->center) < 1e-9)
      {
        continue;
      }
      const double xy = std::hypot(
        other.center.x - source->center.x, other.center.y - source->center.y);
      if (xy <= stack_footprint_tolerance_m_ &&
        other.center.z >= source->top_z + 0.5 * source->object_size_m[2])
      {
        reason = "another detected cube already sits on the " + source->color +
          " source cube; pick_stack can only take a cube from a tower's top";
        return false;
      }
    }
    const auto support = std::find_if(
      cubes.begin(), cubes.end(), [&destination](const DetectedCube & cube) {
        return cube.color == destination.color;
      });
    if (!cross_view && support == cubes.end()) {
      reason = "the frozen snapshot has no " + destination.color + " support cube";
      return false;
    }
    if (!cross_view && support->stack_tier != destination.stack_tier) {
      reason = "the " + destination.color + " support cube changed tier between " +
        std::to_string(destination.stack_tier) + " and " +
        std::to_string(support->stack_tier) + " inside one frozen snapshot";
      return false;
    }
    if (!cross_view && !stackSupportTierAcceptable(*support, reason)) {
      return false;
    }
    const double source_bottom = source->top_z - source->object_size_m[2];
    const double destination_bottom = destination.top_pose.position.z -
      destination.object_size_m[2];
    // Only two tier-0 cubes should have bottoms on the same tabletop.
    const double tabletop_difference =
      source->stack_tier == 0 && destination.stack_tier == 0 ?
      source_bottom - destination_bottom : 0.0;
    if (source->stack_tier == 0 && destination.stack_tier == 0 &&
      std::abs(tabletop_difference) > stack_height_tolerance_m_)
    {
      std::ostringstream message;
      message << "the two cubes do not start on the same tabletop: bottom_difference_m="
              << tabletop_difference;
      reason = message.str();
      return false;
    }
    const double xy_separation = std::hypot(
      source->center.x - destination.top_pose.position.x,
      source->center.y - destination.top_pose.position.y);
    if (xy_separation <= stack_footprint_tolerance_m_) {
      reason = "the source and support cube footprints overlap in the initial snapshot";
      return false;
    }
    const auto & grasp_target = locked_target_.pose.position;
    if (std::hypot(
        grasp_target.x - source->center.x, grasp_target.y - source->center.y) >
      stack_height_tolerance_m_ ||
      std::abs(grasp_target.z - source->top_z) > stack_height_tolerance_m_)
    {
      reason = "the selected source pose disagrees with the frozen stack snapshot";
      return false;
    }
    if (std::hypot(
        grasp_target.x - destination.top_pose.position.x,
        grasp_target.y - destination.top_pose.position.y) <=
      stack_footprint_tolerance_m_)
    {
      reason = "the stable source pose overlaps the stack destination footprint";
      return false;
    }
    locked_stack_source_ = *source;
    // The grasp uses the stable target pose and size. Anchor the remaining
    // source tower to that same pose so the two collision objects meet cleanly.
    locked_stack_source_.center = grasp_target;
    locked_stack_source_.top_z = grasp_target.z;
    locked_stack_source_.object_size_m = locked_object_size_m_;
    std::ostringstream detail;
    detail << std::setprecision(9) << "lower=" << destination.color
           << ",upper=" << locked_color_
           << ",same_snapshot=" << (cross_view ? "false" : "true")
           << ",support_tier=" << destination.stack_tier
           << ",source_tier=" << source->stack_tier
           << ",tower_cubes_after_placement=" << destination.stack_tier + 2
           << ",maximum_stack_cubes=" << maximum_stack_cubes_
           << ",xy_separation_m=" << xy_separation
           << ",tabletop_bottom_difference_m=" << tabletop_difference
           << ",source_bottom_m=" << source_bottom
           << ",destination_bottom_m=" << destination_bottom;
    recordEvent("stack_pair_validated", detail.str());
    RCLCPP_INFO(node_->get_logger(), "pick_stack pair: %s", detail.str().c_str());
    return true;
  }

  // A slot is unusable when a known MoveIt obstacle already sits on it. This
  // reads the live planning scene instead of keeping a counter, so the answer
  // stays correct after an operator clears the table and restarts the stack.
  bool slotOccupied(const std::vector<double> & center)
  {
    for (const auto & entry : planning_scene_.getObjects()) {
      if (entry.first == cube_id_ || entry.first == ground_plane_id_) {
        continue;
      }
      for (const auto & pose : entry.second.primitive_poses) {
        if (std::hypot(
            pose.position.x - center[0], pose.position.y - center[1]) <
          zone_slot_clearance_m_)
        {
          return true;
        }
      }
    }
    return false;
  }

  bool assignZoneSlot(std::string & reason)
  {
    const auto slots_entry = zone_slots_.find(locked_zone_.region);
    if (slots_entry == zone_slots_.end() || slots_entry->second.empty()) {
      reason = "placement zone has no configured slots: " + locked_zone_.region;
      return false;
    }
    const auto & slots = slots_entry->second;
    const std::size_t count = slots.size() / 3;
    const auto slot_center = [&slots](std::size_t index) {
        return std::vector<double>{
        slots[index * 3], slots[index * 3 + 1], slots[index * 3 + 2]};
      };
    if (locked_zone_.explicit_slot) {
      if (locked_zone_.slot_index >= count) {
        std::ostringstream message;
        message << "placement zone " << locked_zone_.region << " has " << count
                << " slot(s); requested slot " << (locked_zone_.slot_index + 1);
        reason = message.str();
        return false;
      }
      if (slotOccupied(slot_center(locked_zone_.slot_index))) {
        std::ostringstream message;
        message << "requested placement slot " << locked_zone_.region
                << (locked_zone_.slot_index + 1) << " is already occupied";
        reason = message.str();
        return false;
      }
      locked_place_center_ = slot_center(locked_zone_.slot_index);
      return true;
    }
    for (std::size_t index = 0; index < count; ++index) {
      if (!slotOccupied(slot_center(index))) {
        locked_place_center_ = slot_center(index);
        return true;
      }
    }
    reason = "every slot in placement zone " + locked_zone_.region + " is already occupied";
    return false;
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
    if (!locked_zone_.region.empty()) {
      return assignZoneSlot(reason);
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
    locked_grasp_tilt_rad_ = 0.0;
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
      state != STACK_COMPLETE &&
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

  // True while the planning scene still carries the grasped cube as an object
  // attached to the gripper, which means software cannot assume the gripper is
  // empty even though the arm may have returned home.
  bool cubeIsStillAttached()
  {
    return planning_scene_.getAttachedObjects({cube_id_}).count(cube_id_) != 0;
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

    // MoveIt reports a REMOVE for an object that is not in the scene as a
    // FAILURE, not as a no-op:
    //   PlanningScene::processCollisionObjectRemove() logs
    //   "Tried to remove world object '<id>', but it does not exist in this
    //   scene." and returns false, which becomes ApplyPlanningScene.success =
    //   false. Treating that as an error would refuse the very first task on a
    //   freshly started stack, when the scene is legitimately empty and there is
    //   nothing to clean up. Removing a cube that is not there is already the
    //   desired state, so it is not requested in the first place.
    if (planning_scene_.getObjects({cube_id_}).count(cube_id_) == 0) {
      recordEvent("previous_cube_absent", "world_id=" + cube_id_);
      return true;
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
    // The lower cube of a finished stack is a virtual obstacle for planning, not
    // a real scene object the operator created, so it must not outlive the task.
    return clearStackSupportObject(reason);
  }

  bool clearArchivedObstacleAtTarget(std::string & reason)
  {
    const auto target_object = cubeObject();
    const auto target = collisionPrimitivePosition(
      target_object, target_object.primitive_poses.front());
    const double match_tolerance_m = std::min(
      locked_object_size_m_[2] * 0.45,
      std::max(maximum_cube_plane_gap_m_, stack_height_tolerance_m_));
    const std::string prefix = cube_id_ + "_sorted_";
    std::vector<std::string> matches;
    for (const auto & entry : planning_scene_.getObjects()) {
      if (entry.first.rfind(prefix, 0) != 0) {
        continue;
      }
      if (std::any_of(
          entry.second.primitive_poses.begin(), entry.second.primitive_poses.end(),
          [&entry, &target, match_tolerance_m](const geometry_msgs::msg::Pose & pose) {
            const auto archived = collisionPrimitivePosition(entry.second, pose);
            return positionDistance(archived, target) <= match_tolerance_m;
          }))
      {
        matches.push_back(entry.first);
      }
    }
    for (const auto & id : matches) {
      moveit_msgs::msg::CollisionObject remove;
      remove.header.frame_id = base_frame_;
      remove.id = id;
      remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!planning_scene_.applyCollisionObject(remove) ||
        planning_scene_.getObjects({id}).count(id) != 0)
      {
        reason = "failed to reclaim the selected cube from archived obstacle " + id;
        return false;
      }
      recordEvent(
        "archived_target_reclaimed",
        "obstacle_id=" + id + ",distance_tolerance_m=" +
        std::to_string(match_tolerance_m));
    }
    return true;
  }

  // Remove the virtual stack obstacles from the planning scene: the support
  // cube and any columns below the support or the previous source.
  // Safe to call when no stack obstacle is present.
  bool clearStackSupportObject(std::string & reason)
  {
    for (const auto & id : {stack_support_id_, stackColumnId(), stackSourceColumnId()}) {
      if (planning_scene_.getObjects({id}).count(id) == 0) {
        continue;
      }
      moveit_msgs::msg::CollisionObject remove;
      remove.header.frame_id = base_frame_;
      remove.id = id;
      remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!planning_scene_.applyCollisionObject(remove) ||
        planning_scene_.getObjects({id}).count(id) != 0)
      {
        reason = "failed to clear the previous stack obstacle " + id + " from the scene";
        return false;
      }
      recordEvent("stack_support_cleared", "obstacle_id=" + id);
    }
    return true;
  }

  std::vector<GraspCandidate> graspCandidates() const
  {
    std::vector<GraspCandidate> candidates;
    if (!use_target_yaw_) {
      for (const double tilt : grasp_tilt_candidates_rad_) {
        candidates.push_back(
        {
          0.0,
          normalizedQuaternion(multiply(grasp_orientation_, rollQuaternion(tilt))),
          tilt});
      }
      return candidates;
    }
    const int symmetry_count = static_cast<int>(std::lround(2.0 * kPi / locked_yaw_symmetry_rad_));
    // Angle is the primary fallback. Exhaust all symmetry-equivalent vertical
    // grasps before adding any tilt, then exhaust +10 before -10, and so on in
    // the configured order. This prevents a tilted grip at the first yaw from
    // hiding an equally reachable vertical grip at another equivalent yaw.
    for (const double tilt : grasp_tilt_candidates_rad_) {
      const auto tilted_nominal = normalizedQuaternion(
        multiply(grasp_orientation_, rollQuaternion(tilt)));
      for (int equivalent = 0; equivalent < symmetry_count; ++equivalent) {
        const double raw_yaw = locked_object_yaw_rad_ + target_yaw_offset_rad_ +
          equivalent * locked_yaw_symmetry_rad_;
        const double candidate_yaw = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));
        const auto candidate = normalizedQuaternion(
          multiply(yawQuaternion(candidate_yaw), tilted_nominal));
        candidates.push_back({candidate_yaw, candidate, tilt});
      }
    }
    return candidates;
  }

  double activeGraspDepth() const
  {
    return grasp_depth_m_;
  }

  geometry_msgs::msg::Pose approachPose() const
  {
    geometry_msgs::msg::Pose pose;
    pose.orientation = locked_grasp_orientation_;
    auto contact = locked_target_.pose.position;
    contact.z += approach_clearance_m_;
    geometry_msgs::msg::Vector3 local_contact_offset;
    local_contact_offset.x = grasp_contact_offset_xyz_m_[0];
    local_contact_offset.y = grasp_contact_offset_xyz_m_[1];
    local_contact_offset.z = grasp_contact_offset_xyz_m_[2];
    const auto world_contact_offset = rotate(pose.orientation, local_contact_offset);
    pose.position.x = contact.x - world_contact_offset.x;
    pose.position.y = contact.y - world_contact_offset.y;
    pose.position.z = contact.z - world_contact_offset.z;
    return pose;
  }

  geometry_msgs::msg::Pose graspPose() const
  {
    geometry_msgs::msg::Pose pose;
    pose.orientation = locked_grasp_orientation_;
    auto contact = locked_target_.pose.position;
    contact.z -= activeGraspDepth();
    // p_contact = p_tcp + R * offset. Solving this lets a configured point on
    // the finger contact the cube while the calibrated TCP remains elsewhere.
    geometry_msgs::msg::Vector3 local_contact_offset;
    local_contact_offset.x = grasp_contact_offset_xyz_m_[0];
    local_contact_offset.y = grasp_contact_offset_xyz_m_[1];
    local_contact_offset.z = grasp_contact_offset_xyz_m_[2];
    const auto world_contact_offset = rotate(pose.orientation, local_contact_offset);
    pose.position.x = contact.x - world_contact_offset.x;
    pose.position.y = contact.y - world_contact_offset.y;
    pose.position.z = contact.z - world_contact_offset.z;
    return pose;
  }

  geometry_msgs::msg::Pose liftPose() const
  {
    auto pose = graspPose();
    pose.position.z += lift_distance_m_;
    if (task_stack_enabled_ && locked_stack_destination_.valid) {
      // A fixed 50 mm tabletop lift is lower than the top of a three-cube
      // tower.  In that state OMPL has to discover a narrow path that first
      // moves away below the tower top and only then climbs; the fourth red
      // layer timed out in all four equivalent IK branches that way.  Lift the
      // source vertically until its bottom already clears the current tower by
      // the same margin used at the placement approach.
      const double tower_clearance_tcp_z =
        locked_stack_destination_.top_pose.position.z + release_bottom_gap_m_ +
        stack_approach_clearance_m_ + locked_object_size_m_[2] - activeGraspDepth();
      pose.position.z = std::max(pose.position.z, tower_clearance_tcp_z);
      if (locked_stack_source_.stack_tier > 0) {
        // Lift the attached cube completely above the tower it came from
        // before planning any horizontal motion to the destination.
        const double source_clearance_tcp_z =
          locked_stack_source_.top_z - locked_stack_source_.object_size_m[2] +
          stack_approach_clearance_m_ + locked_object_size_m_[2] - activeGraspDepth();
        pose.position.z = std::max(pose.position.z, source_clearance_tcp_z);
      }
    }
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
      locked_object_size_m_[2] - activeGraspDepth();
    return pose;
  }

  geometry_msgs::msg::Pose placeApproachPose() const
  {
    auto pose = placePose();
    pose.position.z += lift_distance_m_;
    return pose;
  }

  // -----------------------------------------------------------------------
  // pick_stack geometry
  //
  // The upper cube must come down with the same top-face yaw as the lower cube,
  // otherwise the two cubes touch only along an edge and the tower is not
  // stable. The lower cube's yaw is known before any motion, so the required
  // placement rotation is fixed during planning and the source grasp yaw is
  // chosen from the equivalent grasps that need the smallest rotation.
  // -----------------------------------------------------------------------

  // Yaw of the lower cube's top face in the base frame.
  double stackDestinationYaw() const
  {
    return locked_stack_destination_.grasp_axis_yaw_rad;
  }

  // TCP rotation that lands the upper cube with its top face aligned to the
  // lower cube. The applied grasp yaw of a candidate already carries the edge
  // offset, so the placement yaw is (destination - offset) mapped onto the
  // candidate's equivalent grasp set.
  geometry_msgs::msg::Quaternion stackPlacementOrientation(
    const GraspCandidate & candidate) const
  {
    if (!use_target_yaw_ || !locked_stack_destination_.valid) {
      return candidate.orientation;
    }
    const double symmetry = locked_yaw_symmetry_rad_;
    const double desired_applied =
      stackDestinationYaw() - target_yaw_offset_rad_;
    const double placement_yaw = closestEquivalentAngle(
      desired_applied, candidate.yaw_rad, symmetry);
    if (std::abs(placement_yaw) < 1e-9) {
      return candidate.orientation;
    }
    return normalizedQuaternion(
      multiply(yawQuaternion(placement_yaw), candidate.orientation));
  }

  // Rotation the wrist has to add to the source grasp to align the upper cube
  // with the lower one. A square cube allows four equally valid choices; the
  // planner still has to confirm that the chosen one is reachable.
  double stackPlacementYawOffset(const GraspCandidate & candidate) const
  {
    if (!use_target_yaw_ || !locked_stack_destination_.valid) {
      return 0.0;
    }
    return closestEquivalentAngle(
      stackDestinationYaw() - target_yaw_offset_rad_, candidate.yaw_rad,
      locked_yaw_symmetry_rad_);
  }

  // TCP height that leaves the attached cube's bottom release_bottom_gap_m_
  // above the lower cube's top face. The attached cube centre sits
  // (grasp_depth - height/2) above the TCP, so the TCP has to be that much
  // lower than the intended resting centre.
  geometry_msgs::msg::Pose stackPlacePose() const
  {
    auto pose = graspPose();
    pose.position.x = locked_stack_destination_.top_pose.position.x;
    pose.position.y = locked_stack_destination_.top_pose.position.y;
    const double destination_top_z = locked_stack_destination_.top_pose.position.z;
    pose.position.z = destination_top_z + release_bottom_gap_m_ +
      locked_object_size_m_[2] - activeGraspDepth();
    pose.orientation = stackPlacementOrientation(
      {locked_applied_yaw_rad_, locked_grasp_orientation_});
    return pose;
  }

  // Approach and retreat for a stack stay higher than a table placement: the
  // grasped cube has to clear the lower cube while the wrist rotates.
  geometry_msgs::msg::Pose stackPlaceApproachPose() const
  {
    auto pose = stackPlacePose();
    pose.position.z += stack_approach_clearance_m_;
    return pose;
  }

  geometry_msgs::msg::Quaternion stackedCubeOrientation() const
  {
    // Preserve the cube-to-tool orientation captured at the source grasp.
    // For a tilted grasp this is essential: using the tool orientation itself
    // would incorrectly leave a tilted cube in the world scene after release,
    // although the planned placement keeps the physical cube upright.
    const auto source_cube_orientation = cubeObject().primitive_poses.front().orientation;
    const auto cube_in_tool = normalizedQuaternion(
      multiply(conjugate(locked_grasp_orientation_), source_cube_orientation));
    const auto placement_tool_orientation = stackPlacementOrientation(
      {locked_applied_yaw_rad_, locked_grasp_orientation_, locked_grasp_tilt_rad_});
    return normalizedQuaternion(
      multiply(placement_tool_orientation, cube_in_tool));
  }

  // The upper cube after the 5 mm free drop: its bottom rests on the lower
  // cube's top. The release pose itself remains release_bottom_gap_m_ higher.
  moveit_msgs::msg::CollisionObject stackedCubeObject() const
  {
    auto pose = cubeObject().primitive_poses.front();
    pose.position.x = locked_stack_destination_.top_pose.position.x;
    pose.position.y = locked_stack_destination_.top_pose.position.y;
    pose.position.z = locked_stack_destination_.top_pose.position.z +
      locked_object_size_m_[2] * 0.5;
    pose.orientation = stackedCubeOrientation();
    return makeBox(cube_id_, base_frame_, locked_object_size_m_, pose);
  }

  // The lower cube stays in the scene for the whole stack task, so the planner
  // must route the grasped upper cube around it.
  moveit_msgs::msg::CollisionObject stackSupportObject() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = locked_stack_destination_.top_pose.position.x;
    pose.position.y = locked_stack_destination_.top_pose.position.y;
    pose.position.z = locked_stack_destination_.top_pose.position.z -
      locked_stack_destination_.object_size_m[2] * 0.5;
    pose.orientation = locked_stack_destination_.top_pose.orientation;
    return makeBox(
      stack_support_id_, base_frame_, locked_stack_destination_.object_size_m, pose);
  }

  // Only the top face of a tower is visible, so the cubes underneath the support
  // cube never appear in `detected_targets`. Once the support is itself stacked
  // they still occupy the cell, and a path that dove below the support cube
  // would be planned straight through them. This virtual column stands in for
  // everything between the table plane and the support cube's bottom; for a
  // support cube on the table it has no height and is not added at all, so a
  // plain two-cube stack keeps exactly the scene it had before.
  moveit_msgs::msg::CollisionObject stackColumnObject() const
  {
    const double column_height = locked_stack_destination_.top_pose.position.z -
      locked_stack_destination_.object_size_m[2] - ground_plane_z_m_;
    geometry_msgs::msg::Pose pose;
    pose.position.x = locked_stack_destination_.top_pose.position.x;
    pose.position.y = locked_stack_destination_.top_pose.position.y;
    pose.position.z = ground_plane_z_m_ + column_height * 0.5;
    pose.orientation.w = 1.0;
    return makeBox(
      stackColumnId(), base_frame_,
      {locked_stack_destination_.object_size_m[0],
        locked_stack_destination_.object_size_m[1], column_height},
      pose);
  }

  // True when the locked support cube is the top of an already-stacked tower and
  // therefore has invisible cubes below it.
  bool stackColumnPresent() const
  {
    return locked_stack_destination_.valid && locked_stack_destination_.stack_tier > 0;
  }

  std::string stackColumnId() const
  {
    return stack_support_id_ + "_column";
  }

  // The picked cube may also start on a tower. Keep the tower below it as an
  // obstacle from the first approach through the release and return path.
  moveit_msgs::msg::CollisionObject stackSourceColumnObject() const
  {
    const double column_height = locked_stack_source_.top_z -
      locked_stack_source_.object_size_m[2] - ground_plane_z_m_ -
      kStackSourceColumnTopGapM;
    geometry_msgs::msg::Pose pose;
    pose.position.x = locked_stack_source_.center.x;
    pose.position.y = locked_stack_source_.center.y;
    pose.position.z = ground_plane_z_m_ + column_height * 0.5;
    // pick_stack aligns each new cube to its support, so a tower built by this
    // action has the same footprint orientation as its visible top cube.
    pose.orientation = yawQuaternion(locked_stack_source_.grasp_axis_yaw_rad);
    return makeBox(
      stackSourceColumnId(), base_frame_,
      {locked_stack_source_.object_size_m[0],
        locked_stack_source_.object_size_m[1], column_height},
      pose);
  }

  bool stackSourceColumnPresent() const
  {
    return task_stack_enabled_ && locked_stack_source_.stack_tier > 0;
  }

  std::string stackSourceColumnId() const
  {
    return stack_support_id_ + "_source_column";
  }

  // During execution the placement target follows whichever equivalent grasp
  // the planner selected, which is only known after previewAll().
  geometry_msgs::msg::Pose activePlacePose() const
  {
    return task_stack_enabled_ ? stackPlacePose() : placePose();
  }

  geometry_msgs::msg::Pose activePlaceApproachPose() const
  {
    return task_stack_enabled_ ? stackPlaceApproachPose() : placeApproachPose();
  }

  moveit_msgs::msg::CollisionObject placedCubeObject() const
  {
    if (task_stack_enabled_) {
      return stackedCubeObject();
    }
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

  moveit_msgs::msg::CollisionObject groundPlaneObject() const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.z = ground_plane_z_m_ - ground_plane_contact_epsilon_m_ -
      ground_plane_thickness_m_ * 0.5;
    pose.orientation.w = 1.0;
    return makeBox(
      ground_plane_id_, base_frame_,
      {ground_plane_xy_size_[0], ground_plane_xy_size_[1], ground_plane_thickness_m_},
      pose);
  }

  bool applyWorldScene(std::string & reason)
  {
    // validateStackPair() has already checked the tiers and initial separation.
    for (const auto & pose : {approachPose(), graspPose(), liftPose()}) {
      if (pose.position.z < ground_plane_z_m_) {
        reason = "a task TCP target is below the configured RViz ground plane";
        return false;
      }
    }
    if (task_sort_enabled_ && (activePlacePose().position.z < ground_plane_z_m_ ||
      activePlaceApproachPose().position.z < ground_plane_z_m_))
    {
      reason = "place TCP target is below the ground plane";
      return false;
    }
    std::vector<moveit_msgs::msg::CollisionObject> objects{groundPlaneObject(), cubeObject()};
    if (task_stack_enabled_) {
      // The support cube is a permanent obstacle for the whole stack task, and
      // so is the invisible part of the tower below it. Both have to be in the
      // scene before the approach is planned, not added later.
      objects.push_back(stackSupportObject());
      if (stackColumnPresent()) {
        objects.push_back(stackColumnObject());
      }
      if (stackSourceColumnPresent()) {
        objects.push_back(stackSourceColumnObject());
      }
    }
    if (!planning_scene_.applyCollisionObjects(objects)) {
      reason = "MoveIt rejected RViz-ground/cube collision objects";
      return false;
    }
    recordEvent(
      "scene_applied", task_stack_enabled_ ?
      "RViz XY plane converted to a collision slab below Z=" +
      std::to_string(ground_plane_z_m_) +
      "; visual target cube and stack support cube added" :
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
    if (task_stack_enabled_ && !applyStackSupportObject(reason)) {
      return false;
    }
    return true;
  }

  // Keep the lower cube present in the scene for the whole stack task, whatever
  // a rejected planning candidate did to the world objects in between.
  bool applyStackSupportObject(std::string & reason)
  {
    if (!task_stack_enabled_ || !locked_stack_destination_.valid) {
      return true;
    }
    if (!planning_scene_.applyCollisionObject(stackSupportObject())) {
      reason = "MoveIt rejected the stack support cube";
      return false;
    }
    if (stackColumnPresent() &&
      !planning_scene_.applyCollisionObject(stackColumnObject()))
    {
      reason = "MoveIt rejected the virtual column below the stack support cube";
      return false;
    }
    if (stackSourceColumnPresent() &&
      !planning_scene_.applyCollisionObject(stackSourceColumnObject()))
    {
      reason = "MoveIt rejected the virtual column below the stack source cube";
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

  bool isCandidatePlan(const std::string & name) const
  {
    return name.rfind("candidate_", 0) == 0;
  }

  void configurePlannerFor(const std::string & name)
  {
    const bool fast_candidate = isCandidatePlan(name);
    move_group_->setPlanningTime(
      fast_candidate ? candidate_planning_time_s_ : planning_time_s_);
    move_group_->setNumPlanningAttempts(
      fast_candidate ? candidate_planning_attempts_ : planning_attempts_);
  }

  PlannedMotion planPoseFromSeededIk(
    const std::string & name, const moveit::core::RobotStatePtr & start,
    const geometry_msgs::msg::Pose & target, double ik_timeout_s = -1.0)
  {
    auto target_state = std::make_shared<moveit::core::RobotState>(*start);
    const auto * joint_model_group =
      target_state->getRobotModel()->getJointModelGroup(arm_group_);
    if (!joint_model_group) {
      throw std::runtime_error(name + " has no configured arm joint model group");
    }
    const double timeout_s = ik_timeout_s > 0.0 ? ik_timeout_s : candidate_ik_timeout_s_;
    const bool solved = target_state->setFromIK(
      joint_model_group, target, tip_frame_, timeout_s);
    recordEvent(
      "plan_" + name + "_ik",
      std::string("success=") + (solved ? "true" : "false") +
      ",timeout_s=" + std::to_string(timeout_s));
    if (!solved) {
      throw std::runtime_error(name + " has no seeded IK solution");
    }
    target_state->update();
    return planJointState(name, start, target_state);
  }

  PlannedMotion planHome(
    const std::string & name, const moveit::core::RobotStatePtr & start)
  {
    configurePlannerFor(name);
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

  PlannedMotion planJointState(
    const std::string & name, const moveit::core::RobotStatePtr & start,
    const moveit::core::RobotStatePtr & target)
  {
    if (!target) {
      throw std::runtime_error(name + " has no validated target joint state");
    }
    configurePlannerFor(name);
    std::vector<double> target_joints;
    target->copyJointGroupPositions(arm_group_, target_joints);
    move_group_->setStartState(*start);
    move_group_->clearPoseTargets();
    if (!move_group_->setJointValueTarget(target_joints)) {
      throw std::runtime_error(name + " rejected the validated target joint state");
    }
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

  geometry_msgs::msg::Pose robotStateTipPose(const moveit::core::RobotState & state) const
  {
    const auto & transform = state.getGlobalLinkTransform(tip_frame_);
    const Eigen::Quaterniond rotation(transform.rotation());
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
  }

  PlannedMotion planSortAreaObservation(const moveit::core::RobotStatePtr & start)
  {
    auto target = std::make_shared<moveit::core::RobotState>(*start);
    const auto * group = target->getRobotModel()->getJointModelGroup(arm_group_);
    if (!group || sort_area_observation_joints_rad_.size() != group->getVariableCount()) {
      throw std::runtime_error("sort-area observation joint count does not match arm group");
    }
    const auto & names = group->getVariableNames();
    for (std::size_t index = 0; index < names.size(); ++index) {
      if (names[index] != "joint" + std::to_string(index + 1)) {
        throw std::runtime_error("sort-area observation requires joint1..joint6 ordering");
      }
    }
    target->setJointGroupPositions(group, sort_area_observation_joints_rad_);
    target->update();
    if (!target->satisfiesBounds(group)) {
      throw std::runtime_error("sort-area observation pose violates arm joint bounds");
    }
    return planJointState("sort_area_observation", start, target);
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
    const auto & final_time = trajectory.points.back().time_from_start;
    result.duration_s =
      static_cast<double>(final_time.sec) + static_cast<double>(final_time.nanosec) * 1e-9;
    if (!isFinite(result.duration_s) || result.duration_s < 0.0) {
      throw std::runtime_error(motion.name + " has an invalid planned duration");
    }
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

  void verifyValidatedSegmentStart(
    const std::string & name, const moveit::core::RobotStatePtr & expected)
  {
    if (!expected) {
      throw std::runtime_error(name + " has no validated start state");
    }
    const auto actual = currentStartState(false);
    std::vector<double> actual_joints;
    std::vector<double> expected_joints;
    actual->copyJointGroupPositions(arm_group_, actual_joints);
    expected->copyJointGroupPositions(arm_group_, expected_joints);
    if (actual_joints.size() != expected_joints.size() || actual_joints.empty()) {
      throw std::runtime_error(name + " validated start state has inconsistent dimensions");
    }
    double maximum_difference = 0.0;
    for (std::size_t index = 0; index < actual_joints.size(); ++index) {
      maximum_difference = std::max(
        maximum_difference, std::abs(actual_joints[index] - expected_joints[index]));
    }
    std::ostringstream detail;
    detail << std::setprecision(9)
           << "segment=" << name
           << ",maximum_arm_joint_difference_rad=" << maximum_difference
           << ",limit_rad=" << validated_segment_start_tolerance_rad_;
    recordEvent("validated_segment_start", detail.str());
    if (maximum_difference > validated_segment_start_tolerance_rad_) {
      throw std::runtime_error(
              name + " actual start differs from its validated trajectory: " +
              detail.str());
    }
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
      total.duration_s += part.duration_s;
      total.path_length_rad += part.path_length_rad;
      for (std::size_t index = 0; index < part.joint_travel_rad.size(); ++index) {
        total.joint_travel_rad[index] += part.joint_travel_rad[index];
      }
    }
    return total;
  }

  void requireBoundedSortTransferWrist(const PlannedMotion & motion)
  {
    // Square cubes have four equivalent grasp yaws. A long joint6 route is
    // an IK/plan choice, not a required placement rotation. Check cumulative
    // trajectory travel so a winding path cannot pass by ending near its start.
    constexpr double kMaximumJoint6TravelRad = kPi * 0.5;
    const auto metrics = jointPathMetrics(motion);
    const auto joint = std::find(
      metrics.joint_names.begin(), metrics.joint_names.end(), "joint6");
    if (joint == metrics.joint_names.end()) {
      throw std::runtime_error(motion.name + " has no joint6 trajectory");
    }
    const double travel = metrics.joint_travel_rad[
      std::distance(metrics.joint_names.begin(), joint)];
    std::ostringstream detail;
    detail << std::setprecision(9) << "segment=" << motion.name
           << ",joint6_travel_rad=" << travel
           << ",limit_rad=" << kMaximumJoint6TravelRad;
    recordEvent("sort_transfer_wrist_check", detail.str());
    if (travel > kMaximumJoint6TravelRad + 1e-6) {
      throw std::runtime_error("sort transfer wrist travel exceeds 90 degrees: " + detail.str());
    }
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
    std::string last_candidate_failure;
    double best_duration_s = std::numeric_limits<double>::infinity();
    double best_path_length_rad = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (canceled_.load()) {
        throw std::runtime_error("candidate planning canceled");
      }
      const auto & candidate = candidates[index];
      locked_grasp_orientation_ = candidate.orientation;
      locked_applied_yaw_rad_ = candidate.yaw_rad;
      locked_grasp_tilt_rad_ = candidate.tilt_rad;
      // A square top face makes the four equivalent grasps interchangeable in
      // grasp quality but not in wrist travel. Half a symmetry step (45 degrees
      // for a square top) is the worst case the reduction can leave, so a
      // candidate is dropped only when it exceeds the configured allowance.
      // The four candidates are one whole step apart, so at least one always
      // survives any allowance of half a step or more.
      const double placement_yaw_offset = stackPlacementYawOffset(candidate);
      if (task_stack_enabled_ &&
        std::abs(placement_yaw_offset) >
        maximum_stack_placement_rotation_rad_ + 1e-9)
      {
        recordEvent(
          "grasp_candidate_rejected",
          "index=" + std::to_string(index) + ",yaw_rad=" +
          std::to_string(candidate.yaw_rad) + ",tilt_rad=" +
          std::to_string(candidate.tilt_rad) + ",reason=placement_rotation_" +
          std::to_string(placement_yaw_offset) + "_rad_exceeds_limit");
        continue;
      }
      const std::string prefix = "candidate_" + std::to_string(index) + "_";
      bool attachment_attempted = false;
      try {
        std::vector<PlannedMotion> motions;
        // Prefer the short Cartesian transfer between layers, but a blocked
        // straight line does not imply that the approach pose is unreachable.
        // Fall back to collision-checked joint planning from the same start;
        // the remaining grasp/place route must still pass the full preview.
        auto plan_approach = [&]() {
            if (task_direct_approach_) {
              try {
                return planCartesian(prefix + "approach", start, approachPose());
              } catch (const std::exception & error) {
                if (canceled_.load()) {
                  throw;
                }
                recordEvent("approach_cartesian_fallback", error.what());
              }
            }
            return planPoseFromSeededIk(prefix + "approach", start, approachPose());
          };
        motions.push_back(plan_approach());
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
          // For pick_stack the placement orientation depends on the candidate,
          // so the rotation the wrist has to add is recorded with the path.
          const double placement_yaw_offset = stackPlacementYawOffset(candidate);
          motions.push_back(
            planPoseFromSeededIk(
              prefix + "place_approach", motions.back().end_state,
              activePlaceApproachPose()));
          requireBoundedSortTransferWrist(motions.back());
          motions.push_back(
            planCartesian(
              prefix + "place_descend", motions.back().end_state,
              activePlacePose()));
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
              activePlaceApproachPose()));
          if (task_return_home_) {
            motions.push_back(
              planHome(
                prefix + "return_home", motions.back().end_state));
          }
          if (task_stack_enabled_) {
            recordEvent(
              "stack_candidate_rotation",
              "index=" + std::to_string(index) + ",source_yaw_rad=" +
              std::to_string(candidate.yaw_rad) + ",placement_yaw_offset_rad=" +
              std::to_string(placement_yaw_offset));
          }
        }
        if (!restoreWorldCube(reason)) {
          throw std::runtime_error(reason);
        }
        attachment_attempted = false;
        const auto metrics = combinedJointPathMetrics(motions);
        // Candidate order encodes the preferred fallback: all equivalent yaws
        // at vertical first, then progressively larger positive or negative
        // tilts. The wrist bound above rejects a complete but winding route;
        // choose the first complete route that also passes that bound.
        std::ostringstream detail;
        detail << std::setprecision(9) << "index=" << index << ",yaw_rad=" << candidate.yaw_rad
               << ",tilt_rad=" << candidate.tilt_rad
               << ",planned_duration_s=" << metrics.duration_s
               << ",joint_space_path_length_rad=" << metrics.path_length_rad;
        for (std::size_t joint = 0; joint < metrics.joint_names.size(); ++joint) {
          detail << "," << metrics.joint_names[joint] << "_travel_rad="
                 << metrics.joint_travel_rad[joint];
        }
        recordEvent("grasp_candidate_valid", detail.str());
        best_duration_s = metrics.duration_s;
        best_path_length_rad = metrics.path_length_rad;
        best_candidate = candidate;
        best_motions = std::move(motions);
        recordEvent(
          "grasp_first_feasible",
          "index=" + std::to_string(index) + ",yaw_rad=" +
          std::to_string(candidate.yaw_rad) + ",tilt_rad=" +
          std::to_string(candidate.tilt_rad));
        break;
      } catch (const std::exception & error) {
        last_candidate_failure = error.what();
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
          std::to_string(candidate.yaw_rad) + ",tilt_rad=" +
          std::to_string(candidate.tilt_rad) + ",reason=" + error.what());
      }
    }
    if (best_motions.empty()) {
      if (task_stack_enabled_) {
        std::ostringstream message;
        message << "no equivalent " << locked_color_ << " grasp can place the cube on the "
                << locked_stack_support_color_ << " cube within "
                << maximum_stack_placement_rotation_rad_ << " rad of wrist rotation and "
                << "with a complete collision-free trajectory; last candidate failure: "
                << last_candidate_failure;
        throw std::runtime_error(message.str());
      }
      throw std::runtime_error(
              "no equivalent grasp has a complete collision-free trajectory "
              "within the sort wrist limit; last candidate failure: " +
              last_candidate_failure);
    }
    locked_grasp_orientation_ = best_candidate.orientation;
    locked_applied_yaw_rad_ = best_candidate.yaw_rad;
    locked_grasp_tilt_rad_ = best_candidate.tilt_rad;
    best_motions[0].name = "approach";
    best_motions[1].name = "descend";
    best_motions[2].name = "lift";
    if (task_sort_enabled_) {
      best_motions[3].name = "place_approach";
      best_motions[4].name = "place_descend";
      best_motions[5].name = "place_retreat";
      if (task_return_home_) {
        best_motions[6].name = "return_home";
      }
    }
    for (const auto & motion : best_motions) {
      saveTrajectory(motion);
    }
    recordEvent(
      "grasp_candidate_selected", "yaw_rad=" +
      std::to_string(best_candidate.yaw_rad) + ",tilt_rad=" +
      std::to_string(best_candidate.tilt_rad) + ",planned_duration_s=" +
      std::to_string(best_duration_s) + ",joint_space_path_length_rad=" +
      std::to_string(best_path_length_rad));
    if (task_stack_enabled_) {
      recordEvent(
        "stack_candidate_selected",
        "source_yaw_rad=" + std::to_string(best_candidate.yaw_rad) +
        ",source_tilt_rad=" + std::to_string(best_candidate.tilt_rad) +
        ",destination_yaw_rad=" + std::to_string(stackDestinationYaw()) +
        ",placement_yaw_offset_rad=" +
        std::to_string(stackPlacementYawOffset(best_candidate)) +
        ",placement_orientation_xyzw=[" +
        std::to_string(locked_grasp_orientation_.x) + "," +
        std::to_string(locked_grasp_orientation_.y) + "," +
        std::to_string(locked_grasp_orientation_.z) + "," +
        std::to_string(locked_grasp_orientation_.w) + "]");
    }
    publishPreview(best_motions);
    recordEvent(
      "preview_complete", task_stack_enabled_ ?
      (task_return_home_ ?
      "grasp, aligned stack placement, release, retreat and return-home paths planned" :
      "grasp, aligned stack placement, release and retreat paths planned; next layer continues from retreat") :
      (task_sort_enabled_ ?
      "grasp, place, release, retreat and return-home paths planned" :
      "approach, full descend and attached full lift planned"));
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
      if (motion.name == "place_approach_execute") {
        requireBoundedSortTransferWrist(motion);
      }
      std::ostringstream detail;
      detail << std::setprecision(9) << "segment=" << motion.name
             << ",planned_duration_s=" << metrics.duration_s
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

  std::optional<PendingGripperCommand> beginGripperCommand(
    double position, const std::string & name)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::duration<double>(action_timeout_s_)))
    {
      fail("gripper action server unavailable for " + name);
      return std::nullopt;
    }
    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = 0.0;
    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::duration<double>(action_timeout_s_)) !=
      std::future_status::ready)
    {
      fail("gripper goal acknowledgement timed out for " + name);
      return std::nullopt;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      fail("gripper rejected " + name);
      return std::nullopt;
    }
    PendingGripperCommand pending;
    pending.position = position;
    pending.name = name;
    pending.goal_handle = goal_handle;
    pending.result_future = gripper_client_->async_get_result(goal_handle);
    recordEvent(
      "gripper_" + name + "_started",
      "requested_position_m=" + std::to_string(position));
    return pending;
  }

  bool finishGripperCommand(PendingGripperCommand & pending)
  {
    if (pending.result_future.wait_for(std::chrono::duration<double>(action_timeout_s_)) !=
      std::future_status::ready)
    {
      gripper_client_->async_cancel_goal(pending.goal_handle);
      fail("gripper result timed out for " + pending.name);
      return false;
    }
    const auto wrapped = pending.result_future.get();
    const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result && wrapped.result->reached_goal;
    std::ostringstream detail;
    detail << "result_code=" << static_cast<int>(wrapped.code)
           << ",reached_goal=" << (wrapped.result && wrapped.result->reached_goal)
           << ",requested_position_m=" << pending.position;
    if (wrapped.result) {
      detail << ",reported_position_m=" << wrapped.result->position
             << ",stalled=" << wrapped.result->stalled;
    }
    recordEvent("gripper_" + pending.name, detail.str());
    if (!success) {
      fail("gripper " + pending.name + " failed: " + detail.str());
      return false;
    }
    return true;
  }

  bool commandGripper(double position, const std::string & name)
  {
    auto pending = beginGripperCommand(position, name);
    return pending && finishGripperCommand(*pending);
  }

  void start(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response,
    bool continuous)
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    if (auto_stack_active_.load()) {
      response->success = false;
      response->message = "Start refused: pick_stack_auto owns the arm";
      return;
    }
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
    task_stack_enabled_ = false;
    task_return_home_ = true;
    task_pre_open_ = true;
    task_direct_approach_ = false;
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
    // A cancel with no running task is idempotent. In particular, preserve
    // SORT_COMPLETE (needed to archive the placed cube on the next start) and
    // COMPLETE (the held-object lock). Do not turn either into CANCELED.
    // Do not wait on start_mutex_: active cancellation must remain immediate.
    // An automatic stack is between two steps often enough that its ownership
    // flag has to count as an active task here too.
    if (!worker_running_.load() && !action_goal_claimed_.load() &&
      !auto_stack_active_.load())
    {
      response->success = true;
      response->message = "No active task to cancel; task and holding state unchanged.";
      return;
    }
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

  void resetTabletopObjects(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    response->success = false;
    if (worker_running_.load() || action_goal_claimed_.load() || auto_stack_active_.load()) {
      response->message = "任务仍在运行，请停止任务并等待结束后重置。";
      return;
    }
    if (worker_.joinable()) {worker_.join();}
    if (action_thread_.joinable()) {action_thread_.join();}
    if (auto_action_thread_.joinable()) {auto_action_thread_.join();}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == COMPLETE) {
        response->message = "仍处于持物状态，请先释放物体并解除持物状态。";
        return;
      }
    }
    // A desktop reset must not silently turn a held object into an empty hand.
    if (!planning_scene_.getAttachedObjects({cube_id_}).empty()) {
      response->message = "夹爪仍有持物记录，请先释放物体并解除持物状态。";
      return;
    }
    const std::string prefix = cube_id_ + "_sorted_";
    std::vector<moveit_msgs::msg::CollisionObject> removals;
    std::vector<std::string> ids;
    for (const auto & entry : planning_scene_.getObjects()) {
      if (entry.first != cube_id_ && entry.first != stack_support_id_ &&
        entry.first != stackColumnId() && entry.first != stackSourceColumnId() &&
        entry.first.rfind(prefix, 0) != 0)
      {
        continue;
      }
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = base_frame_;
      object.id = entry.first;
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      removals.push_back(object);
      ids.push_back(entry.first);
    }
    if (!removals.empty() &&
      (!planning_scene_.applyCollisionObjects(removals) ||
      !planning_scene_.getObjects(ids).empty()))
    {
      response->message = "方块模型清理未完成，请重试；未重置任务状态。";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      locked_target_ = geometry_msgs::msg::PoseStamped();
      locked_color_.clear();
      locked_stack_destination_ = StackDestination();
      locked_stack_source_ = DetectedCube();
      locked_zone_ = ZoneRequest();
      locked_target_status_.clear();
      locked_place_center_.clear();
      locked_stack_support_color_.clear();
      auto_stack_order_.clear();
      auto_stack_inventory_.clear();
      auto_stack_home_state_.reset();
      place_slot_use_count_.clear();
      ignored_batch_targets_.clear();
      recent_target_poses_.clear();
      recent_target_sizes_.clear();
      have_target_pose_ = false;
      have_target_size_ = false;
      target_valid_ = false;
    }
    automatic_task_ = false;
    task_sort_enabled_ = false;
    task_stack_enabled_ = false;
    batch_mode_ = false;
    continuous_requested_ = false;
    setState(WAITING_TARGET, "桌面物体状态已重置，等待重新识别。");
    recordEvent("tabletop_objects_reset", "removed=" + std::to_string(ids.size()));
    response->success = true;
    response->message = "已清除 " + std::to_string(ids.size()) +
      " 个方块模型及放置位置记录，下一任务将重新识别桌面。";
  }

  void acknowledgeObjectReleased(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    if (worker_running_.load() || action_goal_claimed_.load() ||
      auto_stack_active_.load())
    {
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
    if (task_stack_enabled_ || task_sort_area_source_) {
      // pick_stack promises that one request always ends with the arm back at
      // its start joints and the task slot free, whatever happened in between.
      const bool completed = runCycleBody();
      if (execute_ && !completed && arm_motion_started_.load() &&
        !canceled_.load())
      {
        recoverToHomeAfterFailedStack();
      }
      return completed;
    }
    return runCycleBody();
  }

  // A failed stack must not leave the arm parked over the tower. Returning home
  // with the cube still attached is safe and lets the operator inspect the
  // scene; nothing is released automatically.
  void recoverToHomeAfterFailedStack()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_detail_ += "; returning to the start joints before reporting the failure";
    }
    publishState();
    recordEvent("stack_recovery_start", "arm motion had started; returning home");
    try {
      auto home = planHome("stack_failure_return_home", currentStartState(false));
      if (executeMotion(home, locked_home_pose_)) {
        recordEvent("stack_recovery_done", "arm returned to the task start joints");
        std::lock_guard<std::mutex> lock(mutex_);
        state_detail_ += "; arm returned to the start joints";
        state_ = RETRYABLE_FAILED;
        return;
      }
      recordEvent("stack_recovery_failed", "return-home motion did not complete");
    } catch (const std::exception & error) {
      recordEvent("stack_recovery_failed", error.what());
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_detail_ += "; automatic return home also failed, inspect the cell before the next task";
      state_ = FAILED;
    }
  }

  bool runCycleBody()
  {
    try {
      if (!observation_log_started_) {
        initializeRunLog();
      }
      observation_log_started_ = false;
      recordEvent("target_snapshot", "stable target copied; equivalent yaws will be planned");
      // The gripper is open-loop: one successful Action result proves that a
      // finite PWM burst was transmitted, not that the fingers moved. Send the
      // configured bounded opening command before any arm motion. Production
      // uses one command; extra repetitions remain an explicit diagnostic
      // option because they add latency and can make the servo hunt.
      for (int repetition = 1; execute_ && task_pre_open_ &&
        repetition <= pre_open_command_repetitions_;
        ++repetition)
      {
        const std::string name = repetition == 1 ?
          "pre_open" : "pre_open_confirm_" + std::to_string(repetition);
        if (!commandGripper(pre_open_position_m_, name) ||
          !waitForGripperState(pre_open_position_m_, name))
        {
          return false;
        }
      }
      std::vector<PlannedMotion> preview;
      try {
        preview = previewAll();
      } catch (const std::exception & error) {
        cycle_failure_skippable_ = true;
        fail(error.what());
        return false;
      }
      recordLatency("planning_done");
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
          PLAN_ONLY_COMPLETE, task_stack_enabled_ ?
          "full stack route planned; no command was sent" :
          (task_sort_enabled_ ?
          "full color-sort route planned; no command was sent" :
          "three motions planned; no command was sent"));
        return true;
      }
      std::string p6_reason;
      if (!p6Gate(p6_reason)) {
        fail("before approach: " + p6_reason);
        return false;
      }
      recordEvent("p6_gate", "before approach passed");
      recordEvent("target_lock", "before approach uses immutable start snapshot");
      recordLatency("approach_command");
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
        "27 mm close was commanded open-loop; continuing without a grip sensor claim" :
        "27 mm was commanded open-loop; physically confirm grip before lift");
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
        if (task_stack_enabled_) {
          recordEvent(
            "stack_destination_snapshot_reused",
            "placing from the immutable initial same-frame support pose; no re-observation");
          setState(
            PLACING, "moving the grasped " + locked_color_ + " cube above the " +
            locked_stack_support_color_ + " cube with the top faces aligned");
        } else {
          setState(PLACING, "moving the grasped cube to its color region");
        }
        // previewAll() proved that motion 3's exact IK branch can continue with
        // a complete collision-free Cartesian descent. A pose-only replan may
        // reach the same TCP pose through another IK branch that cannot descend.
        constexpr std::size_t kPlaceApproachIndex = 3;
        constexpr std::size_t kPlaceDescendIndex = 4;
        constexpr std::size_t kPlaceRetreatIndex = 5;
        if (preview.size() <= kPlaceRetreatIndex) {
          fail("validated placement trajectory is incomplete");
          return false;
        }
        auto place_approach = planJointState(
          "place_approach_execute", currentStartState(false),
          preview[kPlaceApproachIndex].end_state);
        if (!executeMotion(place_approach, activePlaceApproachPose())) {
          return false;
        }
        // The Cartesian segment was already checked from this exact joint
        // branch before any arm motion. Recomputing it from a sub-millimetre
        // hardware arrival error is singularity-sensitive for rotated cubes.
        verifyValidatedSegmentStart(
          "place_descend_execute", preview[kPlaceApproachIndex].end_state);
        auto place_descend = preview[kPlaceDescendIndex];
        place_descend.name = "place_descend_execute";
        if (!executeMotion(place_descend, activePlacePose())) {
          return false;
        }
        auto retreat = preview[kPlaceRetreatIndex];
        retreat.name = "place_retreat_execute";
        // This gripper has no aperture feedback: a successful action only
        // proves that the bounded PWM burst was sent.  Repeat the independent
        // open command while the arm is stationary at the placement pose so
        // one intermittent/stiction event cannot carry the cube into retreat.
        // Never close between attempts because that could re-grip or displace
        // an already placed cube.
        for (int repetition = 1; repetition <= release_command_repetitions_; ++repetition) {
          const auto name = repetition == 1 ?
            std::string("release") : "release_confirm_" + std::to_string(repetition);
          if (!commandGripper(pre_open_position_m_, name)) {
            return false;
          }
          if (!waitForGripperState(pre_open_position_m_, name)) {
            return false;
          }
        }
        // Plan with the cube attached so its retreat swept volume is checked,
        // then detach it only after every fixed 0.8 s release command completes.
        if (!releaseCubeScene(reason)) {
          fail(reason);
          return false;
        }
        verifyValidatedSegmentStart(
          "place_retreat_execute", preview[kPlaceDescendIndex].end_state);
        setState(
          RETURNING_HOME,
          "release command sequence completed; retreating from the placed cube");
        if (!executeMotion(retreat, activePlaceApproachPose())) {
          return false;
        }
        if (task_return_home_) {
          auto home = planHome("return_home_execute", currentStartState(false));
          if (!executeMotion(home, locked_home_pose_)) {
            return false;
          }
        }
        if (task_stack_enabled_) {
          setState(
            STACK_COMPLETE, "the " + locked_color_ + " cube was released on top of the " +
            locked_stack_support_color_ + " cube" +
            (task_return_home_ ? " and the arm returned to the automatic-stack start joints" :
            "; the arm remains at the safe retreat pose for the next layer"));
          return true;
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

  // Latency accounting for "how long until the arm moves". The path from user
  // input to the first motion has several stages that are easy to blame on the
  // wrong component, so each one records its own elapsed time into the event
  // log. `action_started_at_` is set when the Action is accepted, which is the
  // point the GUI's dispatch returns.
  void markActionStart()
  {
    action_started_at_ = node_->now();
    acquisition_ms_ = -1.0;
    planning_ms_ = -1.0;
    approach_ms_ = -1.0;
  }

  double sinceActionStartMs() const
  {
    if (action_started_at_.nanoseconds() == 0) {
      return -1.0;
    }
    return (node_->now() - action_started_at_).seconds() * 1000.0;
  }

  void recordLatency(const std::string & stage)
  {
    const double elapsed = sinceActionStartMs();
    std::ostringstream detail;
    detail << std::fixed << std::setprecision(1) << "\"stage\":\"" << jsonEscape(stage)
           << "\",\"elapsed_ms\":" << elapsed;
    if (stage == "target_acquired") {
      acquisition_ms_ = elapsed;
    } else if (stage == "planning_done") {
      planning_ms_ = elapsed;
    } else if (stage == "approach_command") {
      approach_ms_ = elapsed;
    }
    recordEvent("latency", detail.str());
    RCLCPP_INFO(
      node_->get_logger(), "[latency] %s at %.1f ms after action accept",
      stage.c_str(), elapsed);
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
    std::string locked_zone;
    std::string stack_support_color;
    std::string auto_stack_order;
    double stack_rotation_rad = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = state_;
      detail = state_detail_;
      batch_succeeded = batch_succeeded_;
      batch_skipped = batch_skipped_;
      locked_color = locked_color_;
      locked_zone = locked_zone_.region;
      if (locked_zone_.explicit_slot) {
        locked_zone += std::to_string(locked_zone_.slot_index + 1);
      }
      stack_support_color = locked_stack_support_color_;
      auto_stack_order = joinAutoStackOrder(auto_stack_order_);
      if (task_stack_enabled_) {
        stack_rotation_rad = stackPlacementYawOffset(
          {locked_applied_yaw_rad_, locked_grasp_orientation_});
      }
    }
    const bool auto_stack_active = auto_stack_active_.load();
    const bool retryable =
      state == WAITING_TARGET || state == RETRYABLE_FAILED ||
      state == PLAN_ONLY_COMPLETE || state == COMPLETE || state == SORT_COMPLETE ||
      state == STACK_COMPLETE ||
      state == BATCH_COMPLETE || state == BATCH_COMPLETE_WITH_SKIPS;
    std_msgs::msg::String message;
    message.data = "{\"state\":\"" + stateName(state) + "\",\"detail\":\"" +
      jsonEscape(detail) + "\",\"execute\":" + (execute_ ? "true" : "false") +
      ",\"batch_mode\":" + (batch_mode_ ? "true" : "false") +
      ",\"batch_succeeded\":" + std::to_string(batch_succeeded) +
      ",\"batch_skipped\":" + std::to_string(batch_skipped) +
      ",\"task_mode\":\"" +
      (auto_stack_active ? "pick_stack_auto" :
      (task_stack_enabled_ ? "pick_stack" :
      (locked_zone.empty() ? (task_sort_enabled_ ? "pick_sort" : "pick_hold") : "pick_place"))) +
      "\"" +
      ",\"auto_stack_active\":" + (auto_stack_active ? "true" : "false") +
      ",\"auto_stack_order_top_to_bottom\":\"" + jsonEscape(auto_stack_order) + "\"" +
      ",\"target_color\":\"" + jsonEscape(locked_color) + "\"" +
      ",\"target_zone\":\"" + jsonEscape(locked_zone) + "\"" +
      ",\"support_color\":\"" + jsonEscape(stack_support_color) + "\"" +
      ",\"stack_placement_yaw_rad\":" + std::to_string(stack_rotation_rad) +
      ",\"action_active\":" +
      ((action_goal_claimed_.load() || auto_stack_active) ? "true" : "false") +
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
           << "selected_grasp_tilt_rad: " << locked_grasp_tilt_rad_ << "\n"
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
           << "equivalent_grasp_selection: first_complete_route_in_configured_order\n"
           << "approach_clearance_m: " << approach_clearance_m_ << "\n"
           << "grasp_depth_m: " << activeGraspDepth() << "\n"
           << "candidate_planning_time_s: " << candidate_planning_time_s_ << "\n"
           << "candidate_planning_attempts: " << candidate_planning_attempts_ << "\n"
           << "candidate_ik_timeout_s: " << candidate_ik_timeout_s_ << "\n"
           << "grasp_tilt_candidates_rad: [";
    for (std::size_t index = 0; index < grasp_tilt_candidates_rad_.size(); ++index) {
      if (index > 0) {
        stream << ", ";
      }
      stream << grasp_tilt_candidates_rad_[index];
    }
    stream << "]\n"
           << "grasp_contact_offset_xyz_m: [" << grasp_contact_offset_xyz_m_[0] << ", "
           << grasp_contact_offset_xyz_m_[1] << ", "
           << grasp_contact_offset_xyz_m_[2] << "]\n"
           << "lift_distance_m: " << lift_distance_m_ << "\n"
           << "pre_open_position_m: " << pre_open_position_m_ << "\n"
           << "close_position_m: " << close_position_m_ << "\n"
           << "release_command_repetitions: " << release_command_repetitions_ << "\n"
           << "object_size_m: [" << locked_object_size_m_[0] << ", "
           << locked_object_size_m_[1] << ", " << locked_object_size_m_[2] << "]\n"
           << "sort_enabled: " << (task_sort_enabled_ ? "true" : "false") << "\n"
           << "sort_color: " << locked_color_ << "\n"
           << "sort_grid_file: " << sort_grid_file_ << "\n"
           << "stack_enabled: " << (task_stack_enabled_ ? "true" : "false") << "\n"
           << "stack_upper_color: " << (task_stack_enabled_ ? locked_color_ : "") << "\n"
           << "stack_lower_color: " << locked_stack_support_color_ << "\n"
           << "maximum_stack_cubes: " << maximum_stack_cubes_ << "\n"
           << "stack_support_tier: " << locked_stack_destination_.stack_tier << "\n"
           << "stack_support_top_xyz_m: [";
    if (task_stack_enabled_) {
      const auto & top = locked_stack_destination_.top_pose.position;
      stream << top.x << ", " << top.y << ", " << top.z;
    }
    stream << "]\n"
           << "stack_support_yaw_rad: " << stackDestinationYaw() << "\n"
           << "stack_placement_yaw_offset_rad: " << stackPlacementYawOffset(
      {locked_applied_yaw_rad_, locked_grasp_orientation_}) << "\n"
           << "stack_approach_clearance_m: " << stack_approach_clearance_m_ << "\n"
           << "stack_height_tolerance_m: " << stack_height_tolerance_m_ << "\n"
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
  rclcpp_action::Server<PickColor>::SharedPtr pick_place_server_;
  rclcpp_action::Server<PickColor>::SharedPtr pick_stack_server_;
  rclcpp_action::Server<PickColor>::SharedPtr pick_stack_internal_server_;
  rclcpp_action::Server<PickColor>::SharedPtr pick_auto_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  // Client of pick_stack_internal_server_, used only by pick_stack_auto.
  rclcpp_action::Client<PickColor>::SharedPtr pick_stack_client_;
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tabletop_service_;
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
  std::vector<double> sort_area_observation_joints_rad_;
  std::string cube_id_;
  std::string stack_support_id_;
  std::string ground_plane_id_;
  std::string log_directory_;
  std::string p6_gate_file_;
  std::string sort_grid_file_;
  std::unordered_map<std::string, std::vector<double>> place_centers_;
  std::unordered_map<std::string, std::vector<double>> place_slots_;
  std::unordered_map<std::string, std::size_t> place_slot_use_count_;
  std::unordered_map<std::string, std::vector<double>> zone_centers_;
  std::unordered_map<std::string, std::vector<double>> zone_slots_;
  double zone_slot_clearance_m_{0.025};
  bool sort_enabled_{false};
  bool stack_enabled_{false};
  // pick_stack only. The wrist must never need more than half a symmetry step of
  // extra rotation to align the two cubes; a larger requirement means the pair
  // cannot be stacked with the requested orientation.
  double maximum_stack_placement_rotation_rad_{kPi * 0.25};
  // pick_stack only. Extra height above the lower cube for the approach and
  // retreat poses, so the grasped cube clears the tower while the wrist turns.
  double stack_approach_clearance_m_{0.030};
  // pick_stack only. Allowed difference between two tier-0 bottom estimates,
  // and between the implied stacked cube height and its nominal size.
  double stack_height_tolerance_m_{kStackTabletopToleranceDefault};
  // pick_stack only. Horizontal radius that counts as "sitting on top of".
  double stack_footprint_tolerance_m_{0.020};
  // pick_stack only. Cubes of one tower, counting the cube that stays on the
  // table. A request may top up an existing tower until this many cubes are in
  // it; the perception offsets have to cover one tier less than this.
  int maximum_stack_cubes_{kMaximumStackCubesDefault};
  // Highest tier a visible cube may be reported at, i.e. the top of a tower
  // that has reached maximum_stack_cubes_.
  int maximum_stack_tier_{kMaximumStackCubesDefault - 1};
  // pick_stack_auto only. The tower it is working on right now: the planned
  // order from the top downwards, while the goal is running.
  std::vector<std::string> auto_stack_order_;
  // One RGB-D inventory is frozen before motion.  Each successful layer moves
  // the corresponding source entry to the computed tower top in memory.
  std::vector<DetectedCube> auto_stack_inventory_;
  moveit::core::RobotStatePtr auto_stack_home_state_;
  geometry_msgs::msg::Pose auto_stack_home_pose_;
  std::atomic<int> auto_stack_step_number_{0};
  std::atomic<int> auto_stack_step_count_{0};
  // pick_stack_auto only. Guards the single owner of the cell while a tower is
  // being built: `running` is the flag the goal callback checks before it
  // spawns the orchestrator, `active` is the copy the other pick servers read.
  std::mutex auto_stack_mutex_;
  bool auto_stack_running_{false};
  std::atomic<bool> auto_stack_active_{false};
  ClientGoalHandlePickColor::SharedPtr auto_stack_step_goal_;
  double release_bottom_gap_m_{0.005};
  double batch_empty_timeout_s_{10.0};
  double batch_target_match_distance_m_{0.025};
  bool execute_{false};
  double approach_clearance_m_{0.010};
  double grasp_depth_m_{0.010};
  std::vector<double> grasp_tilt_candidates_rad_;
  double candidate_planning_time_s_{0.25};
  int candidate_planning_attempts_{1};
  double candidate_ik_timeout_s_{0.05};
  std::vector<double> grasp_contact_offset_xyz_m_{0.0, 0.0, 0.0};
  double lift_distance_m_{0.050};
  double pre_open_position_m_{0.060};
  double close_position_m_{0.027};
  int pre_open_command_repetitions_{1};
  int release_command_repetitions_{2};
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
  std::vector<double> manual_pose_x_bounds_;
  std::vector<double> manual_pose_y_bounds_;
  std::vector<double> manual_pose_z_bounds_;
  double manual_pose_ik_timeout_s_{0.25};
  std::vector<double> ground_plane_xy_size_;
  std::vector<std::string> touch_links_;
  double velocity_scale_{0.065};
  double acceleration_scale_{0.065};
  double planning_time_s_{10.0};
  int planning_attempts_{5};
  double cartesian_step_m_{0.001};
  double cartesian_jump_threshold_{0.0};
  double maximum_cartesian_joint_step_rad_{0.10};
  double validated_segment_start_tolerance_rad_{0.010};
  double minimum_cartesian_fraction_{0.999999};
  double target_message_timeout_s_{1.0};
  double target_acquisition_timeout_s_{4.0};
  rclcpp::Time action_started_at_{0, 0, RCL_ROS_TIME};
  double acquisition_ms_{-1.0};
  double planning_ms_{-1.0};
  double approach_ms_{-1.0};
  double arrival_position_tolerance_m_{0.003};
  double arrival_orientation_tolerance_rad_{2.0 * kPi / 180.0};
  double action_timeout_s_{12.0};
  double state_timeout_s_{3.0};
  double gripper_state_tolerance_m_{0.003};
  double ground_plane_z_m_{0.0};
  double ground_plane_thickness_m_{0.020};
  double ground_plane_contact_epsilon_m_{0.00001};
  double maximum_cube_plane_gap_m_{0.010};
  double p6_gate_max_age_s_{360000.0};

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
  double locked_grasp_tilt_rad_{0.0};
  double locked_yaw_symmetry_rad_{kPi};
  std::string locked_target_status_;
  std::string locked_color_;
  std::string locked_stack_support_color_;
  StackDestination locked_stack_destination_;
  DetectedCube locked_stack_source_;
  std::vector<double> locked_place_center_;
  ZoneRequest locked_zone_;
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
  bool task_return_home_{true};
  bool task_pre_open_{true};
  bool task_direct_approach_{false};
  bool observation_log_started_{false};
  bool task_sort_area_source_{false};
  std::atomic<bool> task_sort_enabled_{false};
  std::atomic<bool> task_stack_enabled_{false};
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
  // pick_stack_auto runs on its own thread: its step goals are handled on
  // action_thread_, and a thread cannot join itself.
  std::thread auto_action_thread_;
  std::thread pose_action_thread_;
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
