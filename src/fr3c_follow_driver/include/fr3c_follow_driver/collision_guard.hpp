#pragma once
#include <array>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <moveit/planning_scene/planning_scene.h>
#include "fr3c_follow_driver/position_follow.hpp"

namespace fr3c_follow_driver {
using Joints = std::array<double,6>;  // radians, joint1..joint6
struct CollisionOptions {
  double table_z=0, table_size=2, margin=.005, slow_distance=.020;
  double period=.020, timeout=.15, speed=.020, accel=.060;
};
struct CollisionInput {
  Joints joints{};
  Pose flange{};
  Eigen::Vector3d requested=Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity=Eigen::Vector3d::Zero();
  Eigen::Vector3d lead=Eigen::Vector3d::Zero();
  double stamp=0;
};
struct CollisionOutput {
  bool ready=false, colliding=false;
  double scale=0, distance=0, predicted_distance=0, stamp=0, check_ms=0;
  Eigen::Vector3d requested=Eigen::Vector3d::Zero();
  std::string reason="waiting_for_collision_check", pair;
};
// Scene and heavy FCL work belong exclusively to the worker. No SDK calls here.
class CollisionModel {
 public:
  CollisionModel(const std::string & urdf, const std::string & srdf, CollisionOptions options);
  CollisionOutput check(const CollisionInput & input);
  Pose flange(const Joints & joints) const;
  bool inverse(const Pose & target, Joints & joints) const;
  const moveit::core::RobotModelPtr & model() const { return model_; }
 private:
  bool inverse(const Eigen::Isometry3d & target, moveit::core::RobotState & state) const;
  void measure(moveit::core::RobotState & state, double & distance, std::string & pair,
               bool & collision) const;
  CollisionOptions options_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * arm_=nullptr;
  std::unique_ptr<planning_scene::PlanningScene> scene_;
};
class CollisionGuard {
 public:
  CollisionGuard(std::shared_ptr<CollisionModel> model, CollisionOptions options);
  ~CollisionGuard();
  void submit(CollisionInput input);
  CollisionOutput result(double now, const Eigen::Vector3d & requested) const;
 private:
  void work();
  std::shared_ptr<CollisionModel> model_;
  CollisionOptions options_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  bool quit_=false, have_input_=false;
  CollisionInput input_;
  CollisionOutput output_;
};
}  // namespace fr3c_follow_driver
