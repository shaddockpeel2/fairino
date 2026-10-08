#include "fr3c_follow_driver/collision_guard.hpp"
#include <chrono>
#include <Eigen/Cholesky>
#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shapes.h>

namespace fr3c_follow_driver {
namespace {
double monotonic() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
Eigen::Isometry3d transform(const Pose & pose) {
  Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
  out.linear()=rotation(pose); out.translation()=position(pose); return out;
}
void set_joints(moveit::core::RobotState & state, const Joints & joints) {
  for (unsigned i=0;i<6;++i) {
    if (!std::isfinite(joints[i])) throw std::runtime_error("invalid joint feedback");
    state.setVariablePosition("joint"+std::to_string(i+1),joints[i]);
  }
  state.update();
}
Pose pose_from(const Eigen::Isometry3d & tf) {
  const auto q=Eigen::Quaterniond(tf.linear());
  // SDK expects fixed-axis roll/pitch/yaw, not Eigen's alternate Euler branch.
  const double pitch=std::asin(std::clamp(2*(q.w()*q.y()-q.z()*q.x()),-1.,1.));
  return {tf.translation().x(),tf.translation().y(),tf.translation().z(),
    std::atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(q.x()*q.x()+q.y()*q.y())),pitch,
    std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()))};
}
}
CollisionModel::CollisionModel(const std::string & xml, const std::string & semantic, CollisionOptions options)
    : options_(options) {
  if (!std::isfinite(options.table_z+options.table_size+options.margin+options.slow_distance+
                     options.period+options.timeout+options.speed+options.accel) ||
      options.table_size<=0 || options.margin<=0 || options.slow_distance<=options.margin ||
      options.period<=0 || options.timeout<=options.period || options.speed<=0 || options.accel<=0)
    throw std::invalid_argument("invalid collision/table parameters");
  auto urdf=urdf::parseURDF(xml);
  auto srdf=std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf,semantic)) throw std::runtime_error("collision robot description is missing or invalid");
  // No gripper feedback is available in this SDK chain. Sweep each existing
  // finger box from empty fully closed to the model's 60 mm release aperture,
  // retaining its link name. This does not alter the shared grasp URDF.
  const auto command=urdf->getJoint("gripper_command_joint");
  if (!command || !command->limits) throw std::runtime_error("missing gripper collision model");
  const double aperture=command->limits->lower;
  for (const std::string name : {"gripper_finger_px_py", "gripper_finger_px_ny",
                                 "gripper_finger_nx_py", "gripper_finger_nx_ny"}) {
    auto link=urdf->links_.at(name+"_link");
    const auto joint=urdf->getJoint(name+"_joint");
    if (!joint || !joint->mimic || !link->collision || link->collision_array.size()!=1)
      throw std::runtime_error("incomplete finger collision geometry");
    auto box=std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
    if (!box) throw std::runtime_error("finger collision geometry must be a box");
    const double sweep=command->limits->upper*joint->mimic->multiplier;
    box->dim.x+=sweep;
    link->collision->origin.position.x+=joint->axis.x*
      (command->limits->upper*.5-aperture)*joint->mimic->multiplier;
  }
  model_=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  arm_=model_->getJointModelGroup("fairino3mt_v6_group");
  if (!arm_ || arm_->getVariableCount()!=6 || !model_->getLinkModel("camera_housing_link"))
    throw std::runtime_error("expected FR3C arm and camera collision model");
  for (const std::string name : {"base_link","shoulder_link","upperarm_link","forearm_link",
       "wrist1_link","wrist2_link","wrist3_link","gripper_link","camera_housing_link",
       "gripper_finger_px_py_link","gripper_finger_px_ny_link",
       "gripper_finger_nx_py_link","gripper_finger_nx_ny_link"}) {
    const auto * link=model_->getLinkModel(name);
    if (!link || link->getShapes().empty()) throw std::runtime_error("missing collision geometry: "+name);
  }
  scene_=std::make_unique<planning_scene::PlanningScene>(model_);
  auto & state=scene_->getCurrentStateNonConst(); state.setToDefaultValues();
  state.setVariablePosition("gripper_command_joint",aperture); state.update();
  // Reuse the action scene's 2 x 2 m, 20 mm tabletop, with its top at Z.
  Eigen::Isometry3d table=Eigen::Isometry3d::Identity(); table.translation().z()=options.table_z-.010;
  scene_->getWorldNonConst()->addToObject("follow_table",
    shapes::ShapeConstPtr(new shapes::Box(options.table_size,options.table_size,.020)),table);
  // The stationary robot mounting base is intentionally in contact with its support.
  scene_->getAllowedCollisionMatrixNonConst().setEntry("base_link","follow_table",true);
  // Opposing jaws can legally touch when empty/closed; their swept boxes meet
  // at that contact surface. They still collide with all other robot/world bodies.
  scene_->getAllowedCollisionMatrixNonConst().setEntry("gripper_finger_px_py_link","gripper_finger_nx_py_link",true);
  scene_->getAllowedCollisionMatrixNonConst().setEntry("gripper_finger_px_ny_link","gripper_finger_nx_ny_link",true);
}
Pose CollisionModel::flange(const Joints & joints) const {
  moveit::core::RobotState state(scene_->getCurrentState()); set_joints(state,joints);
  return pose_from(state.getGlobalLinkTransform("flange_link"));
}
bool CollisionModel::inverse(const Eigen::Isometry3d & target, moveit::core::RobotState & state) const {
  // Short, measured-state-seeded differential IK. Never search another branch.
  for (unsigned iteration=0;iteration<30;++iteration) {
    const auto & actual=state.getGlobalLinkTransform("flange_link");
    Eigen::Matrix<double,6,1> error;
    error.head<3>()=target.translation()-actual.translation();
    Eigen::AngleAxisd angle(target.linear()*actual.linear().transpose());
    error.tail<3>()=angle.axis()*angle.angle();
    if (error.head<3>().norm()<.00005 && error.tail<3>().norm()<.0002)
      return state.satisfiesBounds(arm_);
    Eigen::MatrixXd jacobian;
    if (!state.getJacobian(arm_,model_->getLinkModel("flange_link"),Eigen::Vector3d::Zero(),jacobian)) return false;
    Eigen::Matrix<double,6,6> system=jacobian*jacobian.transpose();
    system.diagonal().array()+=1e-8;
    Eigen::VectorXd step=jacobian.transpose()*system.ldlt().solve(error);
    if (!step.allFinite()) return false;
    if (step.cwiseAbs().maxCoeff()>.1) step*=.1/step.cwiseAbs().maxCoeff();
    std::vector<double> q; state.copyJointGroupPositions(arm_,q);
    for (unsigned i=0;i<6;++i) q[i]+=step[i];
    state.setJointGroupPositions(arm_,q); state.update();
  }
  return false;
}
bool CollisionModel::inverse(const Pose & target, Joints & joints) const {
  moveit::core::RobotState state(scene_->getCurrentState()); set_joints(state,joints);
  if (!inverse(transform(target),state)) return false;
  for (unsigned i=0;i<6;++i) joints[i]=state.getVariablePosition("joint"+std::to_string(i+1));
  return true;
}
void CollisionModel::measure(moveit::core::RobotState & state, double & distance,
                             std::string & pair, bool & collision) const {
  collision_detection::DistanceRequest request;
  request.type=collision_detection::DistanceRequestType::GLOBAL;
  request.acm=&scene_->getAllowedCollisionMatrix();
  request.distance_threshold=options_.slow_distance;
  // Include all attachments/fingers, not just links in the arm's kinematic chain.
  collision_detection::DistanceResult self,world;
  scene_->getCollisionEnv()->distanceSelf(request,self,state);
  scene_->getCollisionEnv()->distanceRobot(request,world,state);
  const auto & closest=self.minimum_distance.distance<world.minimum_distance.distance ?
    self.minimum_distance : world.minimum_distance;
  collision=self.collision||world.collision;
  distance=collision ? 0 : std::min(closest.distance,options_.slow_distance);
  pair=closest.link_names[0].empty() ? "outside_distance_band" : closest.link_names[0]+" / "+closest.link_names[1];
}
CollisionOutput CollisionModel::check(const CollisionInput & input) {
  const double began=monotonic();
  CollisionOutput out; out.stamp=input.stamp; out.requested=input.requested;
  try {
    if (!input.requested.allFinite() || !input.velocity.allFinite() || !input.lead.allFinite())
      throw std::runtime_error("invalid collision prediction input");
    auto state=scene_->getCurrentState(); set_joints(state,input.joints);
    if (!state.satisfiesBounds(arm_)) throw std::runtime_error("joint feedback outside model limits");
    const auto actual=state.getGlobalLinkTransform("flange_link");
    const auto observed=transform(input.flange);
    if ((actual.translation()-observed.translation()).norm()>.02 ||
        Eigen::AngleAxisd(actual.linear()*observed.linear().transpose()).angle()>.05)
      throw std::runtime_error("robot model and SDK flange feedback disagree");
    measure(state,out.distance,out.pair,out.colliding);
    out.predicted_distance=out.distance; out.ready=true;
    if (out.colliding) { out.reason="model_collision: "+out.pair; }
    else {
      // Look beyond the next command: result latency + one checker period +
      // braking distance. Cover queued reference lead and both old/new motion.
      const double latency=options_.timeout+options_.period;
      const double horizon=latency+options_.speed/(2*options_.accel);
      const Eigen::Vector3d displacement=input.lead+input.requested*horizon;
      const Eigen::Vector3d braking=input.lead+input.velocity*(latency+input.velocity.norm()/(2*options_.accel));
      bool valid=true;
      double commanded_min=out.distance;
      std::string predicted_pair=out.pair;
      std::vector<Eigen::Vector3d> paths{displacement};
      // A shorter braking segment on the same ray is already covered by the
      // longer commanded segment. Avoid repeating expensive mesh queries.
      const bool covered=braking.norm()<=displacement.norm() && braking.dot(displacement)>=0 &&
        braking.cross(displacement).norm()<1e-9;
      if (!covered) paths.push_back(braking);
      for (const Eigen::Vector3d & delta : paths) {
        if (delta.norm()<1e-9) continue;
        auto predicted=state;
        const int samples=std::max(1,static_cast<int>(std::ceil(delta.norm()/.002)));
        if (samples>32) { valid=false; break; }
        for (int i=1;i<=samples;++i) {
          auto target=actual; target.translation()+=delta*(static_cast<double>(i)/samples);
          const auto previous=predicted;
          if (!inverse(target,predicted)) { valid=false; break; }
          // Intermediate joint states cover the continuous local branch.
          const double joint_travel=previous.distance(predicted,arm_);
          if (joint_travel>.25) { valid=false; break; }
          const int steps=std::max(1,static_cast<int>(std::ceil(joint_travel/.025)));
          for (int j=1;j<=steps;++j) {
            auto sample=previous;
            previous.interpolate(predicted,static_cast<double>(j)/steps,sample); sample.update();
            double d; std::string pair; bool collision;
            measure(sample,d,pair,collision);
            if (d<out.predicted_distance) { out.predicted_distance=d; predicted_pair=pair; }
            if (delta.isApprox(displacement)) commanded_min=std::min(commanded_min,d);
          }
        }
        if (!valid) break;
      }
      if (!valid) out.reason="collision_prediction_unavailable";
      else if (out.predicted_distance<=options_.margin) {
        // Recovery away from a close surface is possible after motion has
        // stopped, provided the entire checked segment increases clearance.
        if (input.velocity.norm()<1e-6 && input.requested.norm()>1e-6 &&
            commanded_min>=out.distance-1e-6 && out.predicted_distance>=out.distance-1e-6) {
          out.scale=1; out.reason="moving_away_from_collision_margin";
        } else out.reason="collision_margin: "+predicted_pair;
      } else {
        out.scale=std::clamp((out.predicted_distance-options_.margin)/
                           (options_.slow_distance-options_.margin),0.,1.);
        // Do not trap a stopped arm merely because its original clearance is small.
        if (out.predicted_distance>=out.distance-1e-6 && input.requested.norm()>1e-6)
          out.scale=1;
        out.reason=out.scale<1 ? "collision_slowdown: "+predicted_pair : "clear";
      }
    }
  } catch (const std::exception & e) { out.ready=true; out.scale=0; out.reason=e.what(); }
  out.check_ms=(monotonic()-began)*1000; return out;
}
CollisionGuard::CollisionGuard(std::shared_ptr<CollisionModel> model, CollisionOptions options)
    : model_(std::move(model)), options_(options) { worker_=std::thread(&CollisionGuard::work,this); }
CollisionGuard::~CollisionGuard() {
  { std::lock_guard<std::mutex> lock(mutex_); quit_=true; }
  wake_.notify_one(); worker_.join();
}
void CollisionGuard::submit(CollisionInput input) {
  { std::lock_guard<std::mutex> lock(mutex_); input_=std::move(input); have_input_=true; }
  wake_.notify_one();
}
CollisionOutput CollisionGuard::result(double now, const Eigen::Vector3d & requested) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto out=output_;
  if (!out.ready || now-out.stamp>options_.timeout) {
    out.scale=0; out.reason="collision_check_pending_or_expired";
  } else if (requested.norm()>1e-6 && (out.requested.norm()<1e-6 ||
             out.requested.normalized().dot(requested.normalized())<.98 ||
             requested.norm()>out.requested.norm()+1e-6)) {
    out.scale=0; out.reason="waiting_for_new_motion_collision_check";
  }
  return out;
}
void CollisionGuard::work() {
  auto next=std::chrono::steady_clock::now();
  for (;;) {
    CollisionInput input;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_until(lock,next,[this] { return quit_; });
      if (quit_) return;
      if (!have_input_) { wake_.wait(lock,[this] { return quit_||have_input_; }); if (quit_) return; }
      input=input_;
    }
    auto output=model_->check(input);
    { std::lock_guard<std::mutex> lock(mutex_); output_=std::move(output); }
    next=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(options_.period));
  }
}
}  // namespace fr3c_follow_driver
