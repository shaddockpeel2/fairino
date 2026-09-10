#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shapes.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <random>
std::string read(const char* p) { std::ifstream f(p); if (!f) throw std::runtime_error(p); std::stringstream s; s << f.rdbuf(); return s.str(); }
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  auto u = urdf::parseURDF(read(argv[1]));
  auto s = std::make_shared<srdf::Model>();
  if (!u || !s->initString(*u, read(argv[2]))) return 3;
  auto model = std::make_shared<moveit::core::RobotModel>(u,s);
  const auto* arm_group = model->getJointModelGroup("fairino3mt_v6_group");
  if (!arm_group) return 8;
  const auto& arm_links = arm_group->getLinkModelNames();
  const bool has_grasp_frame =
    std::find(arm_links.begin(), arm_links.end(), "grasp_frame") != arm_links.end();
  std::cout << "arm variable_count=" << arm_group->getVariableCount()
            << " grasp_frame_in_group=" << has_grasp_frame << '\n';
  if (arm_group->getVariableCount() != 6 || !has_grasp_frame) return 9;
  planning_scene::PlanningScene scene(model);
  auto& state = scene.getCurrentStateNonConst();
  state.setToDefaultValues(); state.update();
  collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=100;
  collision_detection::CollisionResult result;
  scene.checkSelfCollision(req,result);
  std::cout << "default self_collision=" << result.collision << '\n';
  if (result.collision) return 6;
  for (const auto& c: result.contacts) std::cout << c.first.first << " / " << c.first.second << '\n';
  for (auto name : {"pos1", "pos2"}) {
    state.setToDefaultValues(model->getJointModelGroup("fairino3mt_v6_group"),name); state.update();
    result.clear(); scene.checkSelfCollision(req,result);
    std::cout << name << " self_collision=" << result.collision << '\n';
    if (result.collision) return 6;
    for (const auto& c: result.contacts) std::cout << c.first.first << " / " << c.first.second << '\n';
  }
  std::mt19937 rng(42);
  bool found_self_collision=false;
  for (int i=0; i<500 && !found_self_collision; ++i) {
    for (const auto& name : model->getVariableNames()) {
      const auto& bounds=model->getVariableBounds(name);
      std::uniform_real_distribution<double> dist(bounds.min_position_,bounds.max_position_);
      state.setVariablePosition(name,dist(rng));
    }
    state.update(); result.clear(); scene.checkSelfCollision(req,result);
    if (result.collision && state.satisfiesBounds()) {
      found_self_collision=true;
      std::cout << "within joint bounds BUT self-colliding: ";
      for (const auto& c: result.contacts) std::cout << c.first.first << "/" << c.first.second << " ";
      std::cout << '\n';
    }
  }
  if (!found_self_collision) return 7;
  state.setToDefaultValues(); state.update();
  for (auto link : model->getLinkModelsWithCollisionGeometry()) {
    auto pose = state.getCollisionBodyTransform(link,0);
    scene.getWorldNonConst()->addToObject("probe", shapes::ShapeConstPtr(new shapes::Box(.3,.3,.3)), pose);
    result.clear(); scene.checkCollision(req,result);
    bool detected=false;
    for (const auto& c: result.contacts)
      if ((c.first.first=="probe" && c.first.second==link->getName()) || (c.first.second=="probe" && c.first.first==link->getName())) detected=true;
    std::cout << link->getName() << " obstacle_detected=" << detected << '\n';
    if (!detected) return 4;
    scene.getWorldNonConst()->removeObject("probe");
  }
  scene.getWorldNonConst()->addToObject("far", shapes::ShapeConstPtr(new shapes::Box(.1,.1,.1)), Eigen::Isometry3d(Eigen::Translation3d(10,10,10)));
  result.clear(); scene.checkCollision(req,result);
  for (const auto& c: result.contacts) if (c.first.first=="far" || c.first.second=="far") return 5;
  std::cout << "far obstacle: no contact; PASS\n";
}
