#pragma once
#include "fr3c_follow_driver/pose_geometry.hpp"

namespace fr3c_follow_driver {
// A base-frame SO(3) trajectory around the measured TCP, with full tool
// compensation. Phase time advances by one send period, never by a backlog.
class TcpRotation {
 public:
  TcpRotation(const Pose & flange_tcp, Eigen::Vector3d axis, double angle,
              double speed, double accel, double hold_s=.5, bool round_trip=true)
      : flange_tcp_(pose_transform(flange_tcp)), axis_(axis), angle_(angle),
        hold_(hold_s), round_trip_(round_trip) {
    if (!axis.allFinite() || axis.norm()<1e-9 || !std::isfinite(angle+speed+accel+hold_s) ||
        std::abs(angle)<.1*kPi/180 || std::abs(angle)>5*kPi/180 ||
        speed<=0 || speed>5*kPi/180 || accel<=0 || accel>20*kPi/180 || hold_s<0 || hold_s>2)
      throw std::invalid_argument("rotation: 0.1..5 deg, speed <=5 deg/s, accel <=20 deg/s2, hold 0..2 s");
    axis_.normalize();
    leg_ = std::max(1.875*std::abs(angle_)/speed,
                   std::sqrt((10/std::sqrt(3.0))*std::abs(angle_)/accel));
  }
  void anchor(const Pose & actual_flange) { initial_tcp_=pose_transform(actual_flange)*flange_tcp_; anchored_=true; }
  double leg_duration() const { return leg_; }
  double duration() const { return (leg_+hold_)*(round_trip_?2:1); }
  const Eigen::Vector3d & axis() const { return axis_; }
  const Eigen::Isometry3d & initial_tcp() const { return initial_tcp_; }
  const Eigen::Isometry3d & flange_tcp() const { return flange_tcp_; }
  Eigen::Isometry3d tcp(const Pose & flange) const { return pose_transform(flange)*flange_tcp_; }
  double angle_at(double time_s) const {
    if (!std::isfinite(time_s)) throw std::invalid_argument("nonfinite trajectory time");
    const double t=std::max(0.0,time_s);
    if (t<=leg_) return angle_*smooth(t/leg_);
    if (t<=leg_+hold_ || !round_trip_) return angle_;
    if (t<=2*leg_+hold_) return angle_*(1-smooth((t-leg_-hold_)/leg_));
    return 0;
  }
  std::string phase(double t) const {
    if (t<leg_) return "rotating_out";
    if (t<leg_+hold_ || !round_trip_) return "holding_out";
    if (t<2*leg_+hold_) return "rotating_back";
    return "holding_back";
  }
  Eigen::Isometry3d reference_tcp(double time_s) const {
    if (!anchored_) throw std::logic_error("TCP rotation not anchored");
    auto out=initial_tcp_;
    out.linear()=Eigen::AngleAxisd(angle_at(time_s),axis_).toRotationMatrix()*initial_tcp_.linear();
    return out;
  }
  Pose command(double time_s, const Pose & previous) const {
    return continuous_pose(reference_tcp(time_s)*flange_tcp_.inverse(), previous);
  }
 private:
  static double smooth(double u) { u=std::clamp(u,0.0,1.0); return u*u*u*(10+u*(-15+6*u)); }
  Eigen::Isometry3d flange_tcp_, initial_tcp_=Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis_;
  double angle_, hold_, leg_;
  bool round_trip_, anchored_=false;
};
}  // namespace fr3c_follow_driver
