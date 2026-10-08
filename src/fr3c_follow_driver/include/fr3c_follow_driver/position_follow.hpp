#pragma once
#include <Eigen/Geometry>
#include "fr3c_follow_driver/servo_core.hpp"

namespace fr3c_follow_driver {
inline Eigen::Matrix3d rotation(const Pose & pose) {
  return (Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
}
inline Eigen::Vector3d position(const Pose & pose) { return {pose[0], pose[1], pose[2]}; }
inline Eigen::Vector3d limited(Eigen::Vector3d v, double bound) {
  const double norm = v.norm();
  if (norm > bound) v *= bound / norm;
  return v;
}

// Keep a continuous reference, but calculate PBVS error from measured TCP.
// Vision supplies positions only; all reference integration runs at the send rate.
class PositionFollow {
 public:
  PositionFollow(Eigen::Vector3d tcp_xyz, double kp, double speed, double accel,
                 double range, double filter_s)
      : tcp_xyz_(tcp_xyz), kp_(kp), speed_(speed), accel_(accel),
        range_(range), filter_s_(filter_s) {
    if (!tcp_xyz.allFinite() || !std::isfinite(filter_s) || filter_s < 0 ||
        !std::isfinite(kp+speed+accel+range) || std::min({kp,speed,accel,range}) <= 0)
      throw std::invalid_argument("invalid follow geometry or limits");
  }
  Eigen::Vector3d tcp(const Pose & flange) const {
    return position(flange) + rotation(flange)*tcp_xyz_;
  }
  void teach(const Eigen::Vector3d & target, const Pose & flange) {
    if (!target.allFinite()) throw std::invalid_argument("invalid target");
    to_sdk(flange);
    initial_ = flange;
    center_ = tcp(flange);
    offset_ = center_ - target;
    filtered_ = target;
    taught_ = true;
    reanchor(flange);
  }
  void observe(const Eigen::Vector3d & target, double dt, bool reset=false) {
    if (!target.allFinite() || !std::isfinite(dt) || dt <= 0)
      throw std::invalid_argument("invalid target observation");
    if (!taught_) return;
    if (!in_range(target)) throw std::out_of_range("target is outside the follow range");
    const double alpha = filter_s_ == 0 ? 1 : -std::expm1(-dt/filter_s_);
    filtered_ = reset ? target : filtered_ + alpha*(target-filtered_);
  }
  bool in_range(const Eigen::Vector3d & target) const {
    return taught_ && target.allFinite() && (target+offset_-center_).norm() <= range_;
  }
  void reanchor(const Pose & flange) { reference_ = tcp(flange); velocity_.setZero(); }
  void clear() { taught_ = false; velocity_.setZero(); }
  bool taught() const { return taught_; }
  const Eigen::Vector3d & offset() const { return offset_; }
  const Eigen::Vector3d & velocity() const { return velocity_; }
  const Eigen::Vector3d & reference() const { return reference_; }
  Eigen::Vector3d requested(const Pose & actual) const { return limited(kp_*(goal()-tcp(actual)),speed_); }
  Eigen::Vector3d goal() const { return filtered_ + offset_; }
  const Pose & initial() const { return initial_; }
  Pose step(const Pose & actual, double dt, double collision_scale=1) {
    if (!taught_ || !std::isfinite(dt) || dt <= 0 || dt > .016)
      throw std::invalid_argument("invalid follow step");
    const auto actual_tcp = tcp(actual);
    if ((actual_tcp-center_).norm() > range_)
      throw std::out_of_range("actual TCP is outside the follow range");
    if ((reference_-actual_tcp).norm() > .015)
      throw std::runtime_error("servo reference is over 15 mm ahead of feedback");
    if (!std::isfinite(collision_scale) || collision_scale<0 || collision_scale>1)
      throw std::invalid_argument("invalid collision speed scale");
    const auto wanted = requested(actual)*collision_scale;
    velocity_ += limited(wanted-velocity_, accel_*dt);
    // Collision reductions take effect now; only speed increases are ramped.
    velocity_=limited(velocity_,speed_*collision_scale);
    reference_ += velocity_*dt;
    // Tool 0 controls flange. Its recorded rotation locks the calibrated TCP rotation.
    const Eigen::Vector3d flange_xyz = reference_ - rotation(initial_)*tcp_xyz_;
    Pose command = initial_;
    for (int i=0; i<3; ++i) command[i] = flange_xyz[i];
    return command;
  }
 private:
  Eigen::Vector3d tcp_xyz_, center_=Eigen::Vector3d::Zero(), offset_=Eigen::Vector3d::Zero();
  Eigen::Vector3d filtered_=Eigen::Vector3d::Zero(), reference_=Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_=Eigen::Vector3d::Zero();
  Pose initial_{};
  double kp_, speed_, accel_, range_, filter_s_;
  bool taught_=false;
};
}  // namespace fr3c_follow_driver
