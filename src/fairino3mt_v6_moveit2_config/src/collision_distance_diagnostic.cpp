#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
const char* bodyTypeName(collision_detection::BodyType type)
{
  switch (type)
  {
    case collision_detection::BodyTypes::ROBOT_LINK:
      return "robot_link";
    case collision_detection::BodyTypes::ROBOT_ATTACHED:
      return "robot_attached";
    case collision_detection::BodyTypes::WORLD_OBJECT:
      return "world_object";
  }
  return "unknown";
}

void printVector(const Eigen::Vector3d& vector)
{
  std::cout << '[' << vector.x() << ", " << vector.y() << ", " << vector.z() << ']';
}

void printResult(const char* name, const collision_detection::DistanceResult& result, double threshold)
{
  const auto& closest = result.minimum_distance;
  std::cout << "  \"" << name << "\": {\n";
  std::cout << "    \"collision\": " << (result.collision ? "true" : "false") << ",\n";

  // FCL leaves minimum_distance at max-double when there is no applicable
  // pair (for example, an empty planning-scene world).
  if (!std::isfinite(closest.distance) || closest.link_names[0].empty() ||
      closest.link_names[1].empty() || closest.distance == std::numeric_limits<double>::max())
  {
    std::cout << "    \"distance_m\": null,\n";
    std::cout << "    \"distance_mm\": null,\n";
    std::cout << "    \"inside_servo_slowdown_zone\": false,\n";
    std::cout << "    \"pair\": null\n";
    std::cout << "  }";
    return;
  }

  std::cout << "    \"distance_m\": " << closest.distance << ",\n";
  std::cout << "    \"distance_mm\": " << closest.distance * 1000.0 << ",\n";
  std::cout << "    \"servo_threshold_mm\": " << threshold * 1000.0 << ",\n";
  std::cout << "    \"inside_servo_slowdown_zone\": "
            << (closest.distance < threshold ? "true" : "false") << ",\n";
  std::cout << "    \"pair\": [\n";
  for (std::size_t index = 0; index < 2; ++index)
  {
    std::cout << "      {\"name\": \"" << closest.link_names[index] << "\", \"type\": \""
              << bodyTypeName(closest.body_types[index]) << "\", \"nearest_point_m\": ";
    printVector(closest.nearest_points[index]);
    std::cout << '}' << (index == 0 ? "," : "") << '\n';
  }
  std::cout << "    ]\n";
  std::cout << "  }";
}

collision_detection::DistanceResult measureDistance(
    const planning_scene::PlanningSceneConstPtr& scene, const moveit::core::RobotState& state,
    const std::string& group_name, bool self_collision)
{
  collision_detection::DistanceRequest request;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.type = collision_detection::DistanceRequestType::GLOBAL;
  request.group_name = group_name;
  request.enableGroup(scene->getRobotModel());
  request.acm = &scene->getAllowedCollisionMatrix();

  collision_detection::DistanceResult result;
  if (self_collision)
    scene->getCollisionEnv()->distanceSelf(request, result, state);
  else
    scene->getCollisionEnv()->distanceRobot(request, result, state);
  return result;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const auto node = rclcpp::Node::make_shared("collision_distance_diagnostic", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int return_code = 0;
  try
  {
    const auto monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node, "robot_description", "collision_distance_diagnostic");
    if (!monitor->getPlanningScene())
      throw std::runtime_error("failed to construct planning scene monitor");

    monitor->startStateMonitor("/joint_states");
    if (!monitor->requestPlanningSceneState("/get_planning_scene"))
      throw std::runtime_error("/get_planning_scene request failed; is move_group running?");
    if (!monitor->waitForCurrentRobotState(node->now(), 3.0))
      throw std::runtime_error("no fresh complete /joint_states sample within 3 seconds");

    planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
    const std::string group_name = "fairino3mt_v6_group";
    const auto& state = scene->getCurrentState();
    const auto self_result = measureDistance(scene, state, group_name, true);
    const auto world_result = measureDistance(scene, state, group_name, false);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "{\n";
    std::cout << "  \"group_name\": \"" << group_name << "\",\n";
    std::cout << "  \"state_colliding\": " << (scene->isStateColliding(state, group_name) ? "true" : "false")
              << ",\n";
    printResult("self", self_result, 0.010);
    std::cout << ",\n";
    printResult("scene", world_result, 0.020);
    std::cout << "\n}\n";
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    return_code = 1;
  }

  executor.cancel();
  if (spin_thread.joinable())
    spin_thread.join();
  rclcpp::shutdown();
  return return_code;
}
