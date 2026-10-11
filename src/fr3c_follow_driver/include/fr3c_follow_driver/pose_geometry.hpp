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
inline Eigen::Isometry3d pose_transform(const Pose & pose) {
  to_sdk(pose);
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = rotation(pose); out.translation() = position(pose);
  return out;
}
inline double unwrap(double angle, double previous) {
  return previous + std::remainder(angle-previous, 2*kPi);
}
// R = Rz(yaw) Ry(pitch) Rx(roll). Choose the equivalent branch nearest
// the last SDK command, including the coupled roll/yaw at exact gimbal lock.
inline Pose continuous_pose(const Eigen::Isometry3d & value, const Pose & previous) {
  const auto & r = value.linear();
  if (!value.matrix().allFinite() || !(r.transpose()*r).isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
      !value.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-8) ||
      std::abs(r.determinant()-1) > 1e-8)
    throw std::invalid_argument("invalid rigid pose");
  to_sdk(previous);
  const double cp = std::hypot(r(0,0), r(1,0));
  const double pitch = std::atan2(-r(2,0), cp);
  Eigen::Vector3d angles;
  if (cp < 1e-10) {
    if (pitch > 0) {
      const double difference = unwrap(std::atan2(r(0,1), r(1,1)), previous[3]-previous[5]);
      angles = {(previous[3]+previous[5]+difference)/2, unwrap(pitch,previous[4]),
                (previous[3]+previous[5]-difference)/2};
    } else {
      const double sum = unwrap(std::atan2(-r(0,1), r(1,1)), previous[3]+previous[5]);
      angles = {(previous[3]-previous[5]+sum)/2, unwrap(pitch,previous[4]),
                (-previous[3]+previous[5]+sum)/2};
    }
  } else {
    Eigen::Vector3d first(std::atan2(r(2,1),r(2,2)), pitch, std::atan2(r(1,0),r(0,0)));
    Eigen::Vector3d second(first[0]+kPi, kPi-first[1], first[2]+kPi);
    double cost_first=0, cost_second=0;
    for (int i=0; i<3; ++i) {
      first[i]=unwrap(first[i],previous[i+3]); second[i]=unwrap(second[i],previous[i+3]);
      cost_first+=std::pow(first[i]-previous[i+3],2);
      cost_second+=std::pow(second[i]-previous[i+3],2);
    }
    angles = cost_first <= cost_second ? first : second;
  }
  return {value.translation().x(),value.translation().y(),value.translation().z(),
          angles.x(),angles.y(),angles.z()};
}
inline double rotation_distance(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b) {
  return Eigen::Quaterniond(a).angularDistance(Eigen::Quaterniond(b));
}
inline Eigen::Vector3d rotation_log(const Eigen::Matrix3d & value) {
  Eigen::Quaterniond q(value); q.normalize();
  if (q.w()<0) q.coeffs()*=-1;
  if (std::abs(q.w())<1e-12) {
    for (int i=0;i<3;++i) if (std::abs(q.vec()[i])>1e-12) {
      if (q.vec()[i]<0) q.coeffs()*=-1;
      break;
    }
  }
  const double norm=q.vec().norm();
  return norm<1e-12 ? Eigen::Vector3d::Zero() :
    Eigen::Vector3d(q.vec()*(2*std::atan2(norm,std::abs(q.w()))/norm));
}
inline Eigen::Matrix3d rotation_exp(const Eigen::Vector3d & value) {
  const double angle=value.norm();
  return angle<1e-12 ? Eigen::Matrix3d::Identity() :
    Eigen::AngleAxisd(angle,value/angle).toRotationMatrix();
}
}  // namespace fr3c_follow_driver
