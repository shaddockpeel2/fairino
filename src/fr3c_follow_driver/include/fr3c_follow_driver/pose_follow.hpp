#pragma once
#include "fr3c_follow_driver/pose_geometry.hpp"

namespace fr3c_follow_driver {
struct PoseFollowOptions {
  double position_tau=.25, rotation_tau=.8, enter=6*kPi/180, exit=4*kPi/180, mapping=.5;
  double kp_position=2, kp_rotation=1, speed=.020, accel=.060;
  double angular_speed=3*kPi/180, angular_accel=10*kPi/180;
  double radius=1.00, angular_range=90*kPi/180, position_deadband=.004, angular_deadband=2*kPi/180;
  double goal_position_threshold=.008, goal_angle_threshold=4*kPi/180, goal_confirmation=.20;
  double position_lead=.015, angular_lead=5*kPi/180;
};
struct PoseTwist { Eigen::Vector3d linear=Eigen::Vector3d::Zero(), angular=Eigen::Vector3d::Zero(); };

class PoseFollow {
 public:
  PoseFollow(const Pose & flange_tcp, PoseFollowOptions options={})
      : tool_(pose_transform(flange_tcp)), options_(options) {
    for (double value : {options.position_tau,options.rotation_tau,options.enter,options.exit,
          options.mapping,options.kp_position,options.kp_rotation,options.speed,options.accel,
          options.angular_speed,options.angular_accel,options.radius,options.angular_range,
          options.position_deadband,options.angular_deadband,options.position_lead,options.angular_lead,
          options.goal_position_threshold,options.goal_angle_threshold,options.goal_confirmation})
      if (!std::isfinite(value) || value<0) throw std::invalid_argument("invalid pose follow parameters");
    if (std::min({options.mapping,options.kp_position,options.kp_rotation,options.speed,options.accel,
                  options.angular_speed,options.angular_accel,options.radius,options.angular_range,
                  options.position_lead,options.angular_lead})<=0 || options.mapping>1 ||
        options.exit>options.enter || options.enter>=kPi || options.angular_range>=kPi)
      throw std::invalid_argument("invalid pose follow limits");
  }
  Eigen::Isometry3d tcp(const Pose & flange) const { return pose_transform(flange)*tool_; }
  const Eigen::Isometry3d & tool() const { return tool_; }
  bool taught() const { return taught_; }
  const Eigen::Isometry3d & offset() const { return offset_; }
  const Eigen::Isometry3d & raw() const { return raw_; }
  const Eigen::Isometry3d & filtered() const { return filtered_; }
  const Eigen::Isometry3d & reference() const { return reference_; }
  const PoseTwist & velocity() const { return velocity_; }
  const PoseFollowOptions & options() const { return options_; }
  bool rotation_active() const { return rotation_active_; }
  bool holding() const { return holding_; }
  bool goal_pending() const { return goal_pending_; }
  void teach(const Eigen::Isometry3d & target, const Pose & actual) {
    validate(target); center_=tcp(actual); initial_rotation_=target.linear();
    raw_=filtered_=target; accepted_rotation_=target.linear(); rotation_active_=false;
    offset_=target.inverse()*center_; goal_=goal_anchor_=center_; taught_=true; reanchor(actual); holding_=true;
  }
  void clear() { taught_=false; holding_=goal_pending_=false; stop(); }
  void stop() { velocity_={}; }
  void reanchor(const Pose & actual) {
    reference_=tcp(actual); previous_command_=actual; goal_pending_=false;
    goal_pending_time_=0; stop();
  }
  bool in_range(const Eigen::Isometry3d & pose) const {
    return taught_ && pose.matrix().allFinite() &&
      (pose.translation()-center_.translation()).norm()<=options_.radius+1e-12 &&
      rotation_distance(pose.linear(),center_.linear())<=options_.angular_range+1e-12;
  }
  Eigen::Isometry3d raw_goal(const Eigen::Isometry3d & target) const {
    validate(target); return mapped(target,target.linear())*offset_;
  }
  Eigen::Isometry3d mapped_target() const { return mapped(filtered_,accepted_rotation_); }
  Eigen::Isometry3d goal() const { return goal_; }
  bool recovery_ok(const Eigen::Isometry3d & target, double translation, double angle) const {
    return (target.translation()-raw_.translation()).norm()<=translation &&
      rotation_distance(target.linear(),raw_.linear())<=angle;
  }
  void observe(const Eigen::Isometry3d & target,double dt) {
    if (!taught_ || !std::isfinite(dt) || dt<=0) throw std::invalid_argument("invalid pose observation step");
    if (!in_range(raw_goal(target))) throw std::out_of_range("target is outside the pose follow range");
    // Gaps are not evidence of continuous motion; retain the filter/deadband anchor.
    if (dt>.35) { dt=std::min(dt,.1); goal_pending_=false; goal_pending_time_=0; }
    const double ap=options_.position_tau==0?1:-std::expm1(-dt/options_.position_tau);
    const double ar=options_.rotation_tau==0?1:-std::expm1(-dt/options_.rotation_tau);
    raw_=target;
    filtered_.translation()+=ap*(target.translation()-filtered_.translation());
    filtered_.linear()=rotation_exp(ar*rotation_log(target.linear()*filtered_.linear().transpose()))*filtered_.linear();
    const auto delta=rotation_log(filtered_.linear()*accepted_rotation_.transpose());
    const double angle=delta.norm();
    if (options_.enter==0) { accepted_rotation_=filtered_.linear(); rotation_active_=false; }
    else {
      if (angle>=options_.enter) rotation_active_=true;
      else if (angle<=options_.exit+1e-12) rotation_active_=false;
      if (rotation_active_ && angle>options_.exit)
        accepted_rotation_=rotation_exp(delta*((angle-options_.exit)/angle))*accepted_rotation_;
    }
    const auto candidate=mapped_target()*offset_;
    if (!in_range(candidate)) throw std::out_of_range("filtered target is outside the pose follow range");
    // Freeze the entire TCP goal, including the position orbit from rotation.
    // Compare to a fixed anchor so genuine slow motion can accumulate through
    // the deadband. Confirmation uses new trusted image time, never servo ticks.
    const Eigen::Vector3d ep=candidate.translation()-goal_anchor_.translation();
    const Eigen::Vector3d er=rotation_log(candidate.linear()*goal_anchor_.linear().transpose());
    if (ep.norm()<=options_.goal_position_threshold+1e-12 &&
        er.norm()<=options_.goal_angle_threshold+1e-12) {
      goal_pending_=false; goal_pending_time_=0; return;
    }
    const bool consistent=!goal_pending_ ||
      ((ep.norm()<=options_.goal_position_threshold || pending_position_.norm()<=options_.goal_position_threshold ||
        ep.dot(pending_position_)>=0) &&
       (er.norm()<=options_.goal_angle_threshold || pending_rotation_.norm()<=options_.goal_angle_threshold ||
        er.dot(pending_rotation_)>=0));
    if (!goal_pending_ || !consistent) goal_pending_time_=0;
    else goal_pending_time_+=dt;
    goal_pending_=true; pending_position_=ep; pending_rotation_=er;
    if (goal_pending_time_+1e-12>=options_.goal_confirmation) {
      goal_=goal_anchor_=candidate; holding_=goal_pending_=false; goal_pending_time_=0;
    }
  }
  PoseTwist requested(const Pose & actual) const {
    const auto measured=tcp(actual), desired=goal(); PoseTwist result;
    if (holding_) return result;
    const Eigen::Vector3d ep=desired.translation()-measured.translation();
    const Eigen::Vector3d er=rotation_log(desired.linear()*measured.linear().transpose());
    if (ep.norm()>options_.position_deadband) result.linear=options_.kp_position*ep;
    if (er.norm()>options_.angular_deadband) result.angular=options_.kp_rotation*er;
    // Common scaling preserves the requested translation/rotation ratio and
    // accounts for the target-to-TCP offset swept by rotation.
    const double scale=std::max({1.,result.linear.norm()/options_.speed,
      result.angular.norm()/options_.angular_speed,
      result.angular.norm()*offset_.translation().norm()/options_.speed});
    result.linear/=scale; result.angular/=scale; return result;
  }
  Pose step(const Pose & actual,double dt,double collision_scale=1) {
    if (!taught_ || !std::isfinite(dt) || dt<=0 || dt>.016 ||
        !std::isfinite(collision_scale) || collision_scale<0 || collision_scale>1)
      throw std::invalid_argument("invalid pose follow step");
    const auto measured=tcp(actual);
    if (!in_range(measured)) throw std::out_of_range("actual TCP is outside the pose follow range");
    check_lead(measured,reference_);
    if (!holding_ && (goal_.translation()-measured.translation()).norm()<=options_.position_deadband &&
        rotation_distance(goal_.linear(),measured.linear())<=options_.angular_deadband) {
      holding_=true;
      // Take one fixed visual anchor on arrival. A filter tail just below the
      // release threshold must not keep nudging the stopped robot with noise.
      const auto candidate=mapped_target()*offset_;
      if ((candidate.translation()-goal_anchor_.translation()).norm()<=options_.goal_position_threshold &&
          rotation_distance(candidate.linear(),goal_anchor_.linear())<=options_.goal_angle_threshold) {
        goal_anchor_=candidate; goal_pending_=false; goal_pending_time_=0;
      }
    }
    auto wanted=requested(actual);
    wanted.linear*=collision_scale; wanted.angular*=collision_scale;
    velocity_.linear+=limited(wanted.linear-velocity_.linear,options_.accel*dt);
    velocity_.angular+=limited(wanted.angular-velocity_.angular,options_.angular_accel*dt);
    velocity_.linear=limited(velocity_.linear,options_.speed*collision_scale);
    velocity_.angular=limited(velocity_.angular,options_.angular_speed*collision_scale);
    auto next=reference_;
    next.translation()+=velocity_.linear*dt;
    next.linear()=rotation_exp(velocity_.angular*dt)*reference_.linear();
    if (!in_range(next)) throw std::out_of_range("reference TCP is outside the pose follow range");
    check_lead(measured,next);
    reference_=next;
    previous_command_=continuous_pose(reference_*tool_.inverse(),previous_command_);
    return previous_command_;
  }
 private:
  static void validate(const Eigen::Isometry3d & value) { continuous_pose(value,Pose{}); }
  Eigen::Isometry3d mapped(const Eigen::Isometry3d & target,const Eigen::Matrix3d & rotation_value) const {
    if (!taught_) throw std::logic_error("pose relation is not recorded");
    auto out=target;
    out.linear()=rotation_exp(options_.mapping*rotation_log(rotation_value*initial_rotation_.transpose()))*initial_rotation_;
    return out;
  }
  void check_lead(const Eigen::Isometry3d & actual,const Eigen::Isometry3d & reference) const {
    if ((reference.translation()-actual.translation()).norm()>options_.position_lead ||
        rotation_distance(reference.linear(),actual.linear())>options_.angular_lead)
      throw std::runtime_error("pose reference is too far ahead of robot feedback");
  }
  Eigen::Isometry3d tool_,offset_=Eigen::Isometry3d::Identity(),center_=Eigen::Isometry3d::Identity();
  Eigen::Isometry3d raw_=Eigen::Isometry3d::Identity(),filtered_=Eigen::Isometry3d::Identity(),reference_=Eigen::Isometry3d::Identity(),goal_=Eigen::Isometry3d::Identity(),goal_anchor_=Eigen::Isometry3d::Identity();
  Eigen::Matrix3d initial_rotation_=Eigen::Matrix3d::Identity(),accepted_rotation_=Eigen::Matrix3d::Identity();
  Pose previous_command_{};
  PoseTwist velocity_;
  PoseFollowOptions options_;
  Eigen::Vector3d pending_position_=Eigen::Vector3d::Zero(),pending_rotation_=Eigen::Vector3d::Zero();
  double goal_pending_time_=0;
  bool taught_=false,rotation_active_=false,holding_=false,goal_pending_=false;
};
}  // namespace fr3c_follow_driver
