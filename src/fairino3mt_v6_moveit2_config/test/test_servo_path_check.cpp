#include <gtest/gtest.h>
#include <limits>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include "servo_path_check.hpp"

namespace
{
planning_scene::PlanningScenePtr makeScene()
{
  const std::string xml = R"(
    <robot name="path_test">
      <link name="base_link"/>
      <link name="grasp_frame"><collision><geometry><sphere radius="0.00005"/>
      </geometry></collision></link>
      <joint name="joint1" type="prismatic"><parent link="base_link"/>
        <child link="grasp_frame"/><axis xyz="1 0 0"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    </robot>)";
  auto urdf = urdf::parseURDF(xml);
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, R"(<robot name="path_test">
      <group name="fairino3mt_v6_group"><joint name="joint1"/></group></robot>)");
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  return std::make_shared<planning_scene::PlanningScene>(model);
}

moveit::core::RobotState at(const planning_scene::PlanningScenePtr& scene, double q)
{
  auto state = scene->getCurrentState();
  state.setVariablePosition("joint1", q);
  state.update();
  return state;
}

void addObstacle(const planning_scene::PlanningScenePtr& scene, double x, double y = 0)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "thin_obstacle";
  object.header.frame_id = "base_link";
  object.operation = object.ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.0001, 0.001, 0.001};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = x;
  pose.position.y = y;
  object.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processCollisionObjectMsg(object));
}

TEST(ServoPathCheck, ClearEndpointsDoNotHideMiddleCollision)
{
  auto scene = makeScene();
  addObstacle(scene, 0);
  auto start = at(scene, -0.001);
  auto end = at(scene, 0.001);
  EXPECT_FALSE(scene->isStateColliding(start));
  EXPECT_FALSE(scene->isStateColliding(end));
  auto result = p1::checkPath(*scene, start, {-0.001}, {0.001},
                            start.getGlobalLinkTransform("grasp_frame"),
                            "fairino3mt_v6_group", "grasp_frame", 0.0005, 0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "path_collision");
  EXPECT_GT(result.samples, 1U);
}

TEST(ServoPathCheck, ClearanceRejectsNarrowGapBetweenSamples)
{
  auto scene = makeScene();
  addObstacle(scene, 0.00025);
  auto state = at(scene, -0.001);
  auto result = p1::checkPath(*scene, state, {-0.001}, {0.001},
                            state.getGlobalLinkTransform("grasp_frame"));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "path_clearance");
}

TEST(ServoPathCheck, FreeSpaceAndZeroMotionAccepted)
{
  auto scene = makeScene();
  auto state = at(scene, 0);
  auto origin = state.getGlobalLinkTransform("grasp_frame");
  EXPECT_TRUE(p1::checkPath(*scene, state, {0}, {0}, origin).valid);
  EXPECT_TRUE(p1::checkPath(*scene, state, {0}, {0.001}, origin).valid);
}

TEST(ServoPathCheck, BoundsAndMalformedTargetsFailClosed)
{
  auto scene = makeScene();
  auto state = at(scene, 0);
  auto origin = state.getGlobalLinkTransform("grasp_frame");
  EXPECT_EQ(p1::checkPath(*scene, state, {0}, {}, origin).reason, "invalid_group_or_dimensions");
  EXPECT_EQ(p1::checkPath(*scene, state, {0}, {std::numeric_limits<double>::quiet_NaN()}, origin).reason,
            "nonfinite_position");
  EXPECT_EQ(p1::checkPath(*scene, state, {0}, {0.016}, origin).reason, "joint_following_error");
  EXPECT_EQ(p1::checkPath(*scene, state, {0}, {0.003}, origin).reason, "tcp_lead_travel_or_orientation");
  state = at(scene, 0.95);
  EXPECT_EQ(p1::checkPath(*scene, state, {0.95}, {0.951},
                         state.getGlobalLinkTransform("grasp_frame")).reason, "joint_limit_margin");
}

TEST(ServoPathCheck, AttachedGeometryIsIncluded)
{
  auto scene = makeScene();
  addObstacle(scene, 0, 0.005);
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "grasp_frame";
  attached.object.header.frame_id = "grasp_frame";
  attached.object.id = "tool";
  attached.object.operation = attached.object.ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.0001, 0.001, 0.001};
  attached.object.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  pose.position.y = 0.005;
  attached.object.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(attached));
  auto state = at(scene, 0);
  EXPECT_EQ(p1::checkPath(*scene, state, {0}, {0},
                         state.getGlobalLinkTransform("grasp_frame")).reason, "path_collision");
}
}  // namespace
