#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <urdf_parser/urdf_parser.h>

#include <geometric_shapes/shapes.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
const std::vector<std::string> kFingerLinks = {
  "gripper_finger_px_py_link", "gripper_finger_px_ny_link",
  "gripper_finger_nx_py_link", "gripper_finger_nx_ny_link"};

std::string readFile(const char * path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error(path);
  }
  std::stringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

bool hasContactWith(
  const collision_detection::CollisionResult & result,
  const std::string & object, const std::string & link)
{
  for (const auto & contact : result.contacts) {
    const auto & pair = contact.first;
    if ((pair.first == object && pair.second == link) ||
      (pair.second == object && pair.first == link))
    {
      return true;
    }
  }
  return false;
}

bool hasAnyContactWith(
  const collision_detection::CollisionResult & result,
  const std::string & object)
{
  for (const auto & contact : result.contacts) {
    if (contact.first.first == object || contact.first.second == object) {
      return true;
    }
  }
  return false;
}

shapes::ShapeConstPtr box(double x, double y, double z)
{
  return shapes::ShapeConstPtr(new shapes::Box(x, y, z));
}

void printContacts(const collision_detection::CollisionResult & result)
{
  for (const auto & contact : result.contacts) {
    std::cout << contact.first.first << "/" << contact.first.second << " ";
  }
  std::cout << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    return 2;
  }

  auto urdf_model = urdf::parseURDF(readFile(argv[1]));
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!urdf_model || !srdf_model->initString(*urdf_model, readFile(argv[2]))) {
    return 3;
  }

  auto model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  const auto * arm_group = model->getJointModelGroup("fairino3mt_v6_group");
  if (!arm_group) {
    return 8;
  }
  const auto & arm_links = arm_group->getLinkModelNames();
  const bool has_grasp_frame =
    std::find(arm_links.begin(), arm_links.end(), "grasp_frame") != arm_links.end();
  std::cout << "arm variable_count=" << arm_group->getVariableCount()
            << " grasp_frame_in_group=" << has_grasp_frame << '\n';
  if (arm_group->getVariableCount() != 6 || !has_grasp_frame) {
    return 9;
  }
  for (const auto & finger : kFingerLinks) {
    const auto * link = model->getLinkModel(finger);
    if (!link || link->getShapes().empty()) {
      std::cerr << "missing finger collision link: " << finger << '\n';
      return 10;
    }
  }

  planning_scene::PlanningScene scene(model);
  auto & state = scene.getCurrentStateNonConst();
  state.setToDefaultValues();
  state.update();

  const Eigen::Isometry3d flange_to_grasp =
    state.getGlobalLinkTransform("flange_link").inverse() *
    state.getGlobalLinkTransform("grasp_frame");
  const Eigen::Vector3d expected_tcp(
    -0.000847397, 0.003027928, 0.184614459);
  const double tcp_error = (flange_to_grasp.translation() - expected_tcp).norm();
  std::cout << "flange_to_grasp_xyz=" << flange_to_grasp.translation().transpose()
            << " error=" << tcp_error << '\n';
  if (tcp_error > 1e-9) {
    return 18;
  }

  const auto fingerSpanX = [&state]() {
      const Eigen::Isometry3d gripper_inverse =
        state.getGlobalLinkTransform("gripper_link").inverse();
      const double positive_x =
        (gripper_inverse * state.getGlobalLinkTransform("gripper_finger_px_py_link"))
        .translation().x();
      const double negative_x =
        (gripper_inverse * state.getGlobalLinkTransform("gripper_finger_nx_py_link"))
        .translation().x();
      return positive_x - negative_x + 0.012;
    };
  state.setVariablePosition("gripper_command_joint", 0.027);
  state.update();
  const double closed_span = fingerSpanX();
  state.setVariablePosition("gripper_command_joint", 0.060);
  state.update();
  const double open_span = fingerSpanX();
  std::cout << "finger_span closed=" << closed_span
            << " open=" << open_span << '\n';
  if (std::abs(closed_span - 0.050) > 1e-6 ||
    std::abs(open_span - 0.084) > 1e-6)
  {
    return 19;
  }
  state.setVariablePosition("gripper_command_joint", 0.027);
  state.update();

  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 100;
  request.max_contacts_per_pair = 10;
  collision_detection::CollisionResult result;

  scene.checkSelfCollision(request, result);
  std::cout << "default self_collision=" << result.collision << '\n';
  if (result.collision) {
    printContacts(result);
    return 6;
  }

  for (const auto * name : {"pos1", "pos2"}) {
    state.setToDefaultValues(arm_group, name);
    state.update();
    result.clear();
    scene.checkSelfCollision(request, result);
    std::cout << name << " self_collision=" << result.collision << '\n';
    if (result.collision) {
      printContacts(result);
      return 6;
    }
  }

  std::mt19937 rng(42);
  bool found_self_collision = false;
  for (int i = 0; i < 500 && !found_self_collision; ++i) {
    for (const auto & name : model->getVariableNames()) {
      const auto & bounds = model->getVariableBounds(name);
      std::uniform_real_distribution<double> distribution(
        bounds.min_position_, bounds.max_position_);
      state.setVariablePosition(name, distribution(rng));
    }
    state.update();
    result.clear();
    scene.checkSelfCollision(request, result);
    if (result.collision && state.satisfiesBounds()) {
      found_self_collision = true;
      std::cout << "within joint bounds BUT self-colliding: ";
      printContacts(result);
    }
  }
  if (!found_self_collision) {
    return 7;
  }

  state.setToDefaultValues();
  state.update();
  for (const auto * link : model->getLinkModelsWithCollisionGeometry()) {
    const auto pose = state.getCollisionBodyTransform(link, 0);
    scene.getWorldNonConst()->addToObject("probe", box(0.3, 0.3, 0.3), pose);
    result.clear();
    scene.checkCollision(request, result);
    const bool detected = hasContactWith(result, "probe", link->getName());
    std::cout << link->getName() << " obstacle_detected=" << detected << '\n';
    if (!detected) {
      return 4;
    }
    scene.getWorldNonConst()->removeObject("probe");
  }

  scene.getWorldNonConst()->addToObject(
    "far", box(0.1, 0.1, 0.1),
    Eigen::Isometry3d(Eigen::Translation3d(10.0, 10.0, 10.0)));
  result.clear();
  scene.checkCollision(request, result);
  if (hasAnyContactWith(result, "far")) {
    return 5;
  }
  scene.getWorldNonConst()->removeObject("far");
  std::cout << "far obstacle: no contact\n";

  // A 30 mm cube at the grasp depth may touch only the four finger links.
  // Its centre is 15 mm behind the calibrated fingertip TCP.
  state.setVariablePosition("gripper_command_joint", 0.027);
  state.update();
  const Eigen::Isometry3d gripper_pose = state.getGlobalLinkTransform("gripper_link");
  const Eigen::Isometry3d grasp_cube_pose =
    gripper_pose * Eigen::Translation3d(0.0, 0.0, 0.160);
  scene.getWorldNonConst()->addToObject(
    "target_cube", box(0.030, 0.030, 0.030), grasp_cube_pose);
  result.clear();
  scene.checkCollision(request, result);
  bool touches_finger = false;
  for (const auto & finger : kFingerLinks) {
    touches_finger = touches_finger || hasContactWith(result, "target_cube", finger);
  }
  const bool touches_body = hasContactWith(result, "target_cube", "gripper_link");
  std::cout << "target before ACM: collision=" << result.collision
            << " finger_contact=" << touches_finger
            << " body_contact=" << touches_body << '\n';
  if (!result.collision || !touches_finger || touches_body) {
    printContacts(result);
    return 11;
  }

  auto & allowed = scene.getAllowedCollisionMatrixNonConst();
  for (const auto & finger : kFingerLinks) {
    allowed.setEntry("target_cube", finger, true);
  }
  result.clear();
  scene.checkCollision(request, result);
  std::cout << "target after finger-only ACM: collision=" << result.collision << '\n';
  if (result.collision) {
    printContacts(result);
    return 12;
  }
  scene.getWorldNonConst()->removeObject("target_cube");

  // The same target-specific allowance must not hide housing or table contact.
  const Eigen::Isometry3d body_probe_pose =
    gripper_pose * Eigen::Translation3d(0.0, 0.0, 0.040);
  scene.getWorldNonConst()->addToObject(
    "body_probe", box(0.030, 0.030, 0.030), body_probe_pose);
  result.clear();
  scene.checkCollision(request, result);
  const bool body_rejected = hasContactWith(result, "body_probe", "gripper_link");
  std::cout << "non-contact housing collision rejected=" << body_rejected << '\n';
  if (!result.collision || !body_rejected) {
    return 13;
  }
  scene.getWorldNonConst()->removeObject("body_probe");

  const Eigen::Isometry3d table_pose =
    gripper_pose * Eigen::Translation3d(0.0, 0.0, 0.180);
  scene.getWorldNonConst()->addToObject(
    "table", box(0.50, 0.50, 0.020), table_pose);
  result.clear();
  scene.checkCollision(request, result);
  std::cout << "table penetration rejected=" << result.collision << '\n';
  if (!result.collision || !hasAnyContactWith(result, "table")) {
    return 14;
  }
  scene.getWorldNonConst()->removeObject("table");

  // Attach a 30 mm cube with finger-only touch links. It must be collision-free
  // in open space and still collide with an obstacle during a carried lift.
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "grasp_frame";
  attached.touch_links = kFingerLinks;
  attached.object.id = "carried_cube";
  attached.object.header.frame_id = "grasp_frame";
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached.object.pose.orientation.w = 1.0;
  attached.object.pose.position.z = -0.015;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {0.030, 0.030, 0.030};
  attached.object.primitives.push_back(primitive);
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  attached.object.primitive_poses.push_back(identity);
  if (!scene.processAttachedCollisionObjectMsg(attached)) {
    return 15;
  }

  result.clear();
  scene.checkCollision(request, result);
  std::cout << "attached cube in free space: collision=" << result.collision << '\n';
  if (result.collision) {
    printContacts(result);
    return 16;
  }

  scene.getWorldNonConst()->addToObject(
    "lift_obstacle", box(0.040, 0.040, 0.040), grasp_cube_pose);
  result.clear();
  scene.checkCollision(request, result);
  const bool attached_rejected = hasAnyContactWith(result, "carried_cube") &&
    hasAnyContactWith(result, "lift_obstacle");
  std::cout << "attached cube obstacle collision rejected=" << attached_rejected << '\n';
  if (!result.collision || !attached_rejected) {
    printContacts(result);
    return 17;
  }

  std::cout << "P2 split gripper / selective contact / table / attached-object: PASS\n";
  return 0;
}
