#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <moveit/planning_scene/planning_scene.h>

namespace p1
{
struct PathResult
{
  bool valid{false};
  std::string reason;
  std::size_t samples{0};
};

// Discrete full-robot collision checks (world + self + attached bodies + ACM).
// Small L1 joint spacing and clearance supplement, not replace, actual braking.
inline PathResult checkPath(
    const planning_scene::PlanningScene& scene, const moveit::core::RobotState& measured,
    const std::vector<double>& from, const std::vector<double>& to,
    const Eigen::Isometry3d& origin, const std::string& group_name = "fairino3mt_v6_group",
    const std::string& tip = "grasp_frame", double spacing = 0.0005,
    double clearance = 0.003)
{
  PathResult result;
  const auto* group = measured.getJointModelGroup(group_name);
  if (!group || from.size() != group->getVariableCount() || to.size() != from.size() ||
      !measured.getRobotModel()->hasLinkModel(tip))
  {
    result.reason = "invalid_group_or_dimensions";
    return result;
  }
  double length = 0;
  std::vector<double> actual;
  measured.copyJointGroupPositions(group, actual);
  for (std::size_t i = 0; i < to.size(); ++i)
  {
    if (!std::isfinite(from[i]) || !std::isfinite(to[i]) || !std::isfinite(actual[i]))
    {
      result.reason = "nonfinite_position";
      return result;
    }
    if (std::abs(to[i] - actual[i]) > 0.015 || std::abs(from[i] - actual[i]) > 0.015)
    {
      result.reason = "joint_following_error";
      return result;
    }
    length += std::abs(to[i] - from[i]);
  }
  if (!std::isfinite(spacing) || spacing <= 0 || !std::isfinite(clearance) || clearance < 0)
  {
    result.reason = "invalid_resolution";
    return result;
  }
  const auto intervals = std::max<std::size_t>(1, std::ceil(length / spacing));
  if (intervals > 128)
  {
    result.reason = "path_too_long";
    return result;
  }
  auto state = measured;
  const auto actual_tcp = measured.getGlobalLinkTransform(tip).translation().eval();
  for (std::size_t index = 0; index <= intervals; ++index)
  {
    const double fraction = static_cast<double>(index) / intervals;
    std::vector<double> q(from.size());
    for (std::size_t j = 0; j < q.size(); ++j)
      q[j] = from[j] + fraction * (to[j] - from[j]);
    state.setJointGroupPositions(group, q);
    state.update();
    ++result.samples;
    if (!state.satisfiesBounds(group, -0.08))
    {
      result.reason = "joint_limit_margin";
      return result;
    }
    const auto& tcp = state.getGlobalLinkTransform(tip);
    if ((tcp.translation() - actual_tcp).norm() > 0.002 ||
        (tcp.translation() - origin.translation()).norm() > 0.012 ||
        Eigen::AngleAxisd(origin.linear().transpose() * tcp.linear()).angle() > M_PI / 180.0)
    {
      result.reason = "tcp_lead_travel_or_orientation";
      return result;
    }
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult collision;
    // Arm's updated links include its carried camera/gripper geometry; exclude
    // fixed base/world support pairs from moving-body clearance thresholds.
    request.group_name = group_name;
    scene.checkCollision(request, collision, state);
    if (collision.collision)
    {
      result.reason = "path_collision";
      return result;
    }
    collision_detection::DistanceRequest distance_request;
    distance_request.type = collision_detection::DistanceRequestType::GLOBAL;
    distance_request.group_name = group_name;
    distance_request.enableGroup(scene.getRobotModel());
    distance_request.acm = &scene.getAllowedCollisionMatrix();
    distance_request.enable_signed_distance = true;
    collision_detection::DistanceResult self, world;
    scene.getCollisionEnv()->distanceSelf(distance_request, self, state);
    scene.getCollisionEnv()->distanceRobot(distance_request, world, state);
    if (!std::isfinite(self.minimum_distance.distance) ||
        !std::isfinite(world.minimum_distance.distance) ||
        self.minimum_distance.distance < clearance || world.minimum_distance.distance < clearance)
    {
      result.reason = "path_clearance";
      return result;
    }
  }
  result.valid = true;
  result.reason = "path_valid";
  return result;
}
}  // namespace p1
