#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <Eigen/Geometry>

namespace fr3c_follow_feedback {
inline Eigen::Isometry3d pose_mm_deg(const double values[6]) {
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(values[i])) throw std::invalid_argument("nonfinite robot pose");
  }
  constexpr double radians = 3.14159265358979323846 / 180.0;
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(values[0], values[1], values[2]) / 1000.0;
  result.linear() = (Eigen::AngleAxisd(values[5] * radians, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(values[4] * radians, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(values[3] * radians, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}

class PacketFreshness {
 public:
  using Key = std::array<unsigned, 8>;  // sequence and controller clock fields
  bool observe(const Key & key, double now) {
    if (!seen_ || key != key_) {
      seen_ = true; key_ = key; last_change_ = now;
      return true;
    }
    return false;
  }
  double age(double now) const { return seen_ ? now - last_change_ : -1.0; }
 private:
  bool seen_ = false;
  Key key_{};
  double last_change_ = 0.0;
};
}  // namespace fr3c_follow_feedback
