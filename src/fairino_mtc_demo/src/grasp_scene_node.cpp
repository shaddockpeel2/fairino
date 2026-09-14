#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
bool validTriple(const std::vector<double> & values, bool require_positive)
{
  if (values.size() != 3) {
    return false;
  }
  for (const double value : values) {
    if (!std::isfinite(value) || (require_positive && value <= 0.0)) {
      return false;
    }
  }
  return true;
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string & id, const std::string & frame_id,
  const std::vector<double> & size, const std::vector<double> & center)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame_id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.orientation.w = 1.0;
  object.pose.position.x = center[0];
  object.pose.position.y = center[1];
  object.pose.position.z = center[2];

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(size.begin(), size.end());
  object.primitives.push_back(primitive);

  geometry_msgs::msg::Pose primitive_pose;
  primitive_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(primitive_pose);
  return object;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fr3c_first_grasp_scene");

  const auto frame_id = node->declare_parameter<std::string>("frame_id", "base_link");
  const auto table_id = node->declare_parameter<std::string>("table.id", "first_grasp_table");
  const auto table_size =
    node->declare_parameter<std::vector<double>>("table.size", {0.0, 0.0, 0.0});
  const auto table_center =
    node->declare_parameter<std::vector<double>>(
    "table.center", std::vector<double>{});
  const auto cube_id = node->declare_parameter<std::string>("cube.id", "target_cube");
  const auto cube_size =
    node->declare_parameter<std::vector<double>>("cube.size", {0.030, 0.030, 0.030});
  const auto cube_center =
    node->declare_parameter<std::vector<double>>(
    "cube.center", std::vector<double>{});
  const auto obstacle_ids =
    node->declare_parameter<std::vector<std::string>>(
    "obstacles.ids", std::vector<std::string>{});
  const auto obstacle_sizes =
    node->declare_parameter<std::vector<double>>(
    "obstacles.sizes", std::vector<double>{});
  const auto obstacle_centers =
    node->declare_parameter<std::vector<double>>(
    "obstacles.centers", std::vector<double>{});

  if (frame_id.empty() || table_id.empty() || cube_id.empty() ||
    !validTriple(table_size, true) || !validTriple(table_center, false) ||
    !validTriple(cube_size, true) || !validTriple(cube_center, false))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Refusing to publish guessed geometry. Set finite 3-element table.size, "
      "table.center, cube.size and cube.center in frame_id.");
    rclcpp::shutdown();
    return 2;
  }
  if (obstacle_sizes.size() != obstacle_ids.size() * 3 ||
    obstacle_centers.size() != obstacle_ids.size() * 3)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "obstacles.sizes and obstacles.centers must contain one flattened XYZ "
      "triple per obstacles.ids entry");
    rclcpp::shutdown();
    return 3;
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(makeBox(table_id, frame_id, table_size, table_center));
  objects.push_back(makeBox(cube_id, frame_id, cube_size, cube_center));
  for (std::size_t i = 0; i < obstacle_ids.size(); ++i) {
    const std::vector<double> size = {
      obstacle_sizes[i * 3], obstacle_sizes[i * 3 + 1], obstacle_sizes[i * 3 + 2]};
    const std::vector<double> center = {
      obstacle_centers[i * 3], obstacle_centers[i * 3 + 1],
      obstacle_centers[i * 3 + 2]};
    if (obstacle_ids[i].empty() || !validTriple(size, true) ||
      !validTriple(center, false))
    {
      RCLCPP_ERROR(node->get_logger(), "Invalid obstacle entry at index %zu", i);
      rclcpp::shutdown();
      return 4;
    }
    objects.push_back(makeBox(obstacle_ids[i], frame_id, size, center));
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene;
  if (!planning_scene.applyCollisionObjects(objects)) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt rejected the first-grasp collision scene");
    rclcpp::shutdown();
    return 5;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Applied table '%s', cube '%s' and %zu nearby obstacle(s) in frame '%s'",
    table_id.c_str(), cube_id.c_str(), obstacle_ids.size(), frame_id.c_str());
  rclcpp::shutdown();
  return 0;
}
