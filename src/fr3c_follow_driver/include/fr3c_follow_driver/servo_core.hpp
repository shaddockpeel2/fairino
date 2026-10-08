#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include "robot_types.h"

namespace fr3c_follow_driver {
using Pose = std::array<double, 6>;  // metres, fixed-axis radians; SDK boundary uses mm/deg.
constexpr double kPi = 3.14159265358979323846;

inline Pose from_sdk(const double values[6]) {
  Pose pose{};
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(values[i])) throw std::invalid_argument("nonfinite feedback pose");
    pose[i] = values[i] * (i < 3 ? 0.001 : kPi / 180.0);
  }
  return pose;
}
inline DescPose to_sdk(const Pose & p) {
  for (double v : p) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite servo pose");
  DescPose out;
  out.tran = DescTran(p[0]*1000, p[1]*1000, p[2]*1000);
  out.rpy = Rpy(p[3]*180/kPi, p[4]*180/kPi, p[5]*180/kPi);
  return out;
}

class Translation {
 public:
  Translation(int axis, double distance_m, double speed_m_s, double accel_m_s2)
      : axis_(axis), distance_(distance_m) {
    if (axis < 0 || axis > 2 || !std::isfinite(distance_m) ||
        std::abs(distance_m) < 0.0001 || std::abs(distance_m) > 0.010 ||
        !std::isfinite(speed_m_s) || speed_m_s <= 0 || speed_m_s > 0.010 ||
        !std::isfinite(accel_m_s2) || accel_m_s2 <= 0 || accel_m_s2 > 0.050)
      throw std::invalid_argument("use one axis, 0.1..10 mm, speed <=10 mm/s, acceleration <=50 mm/s2");
    // Quintic smoothstep: max normalized velocity 1.875, acceleration 10/sqrt(3).
    duration_ = std::max(1.875*std::abs(distance_)/speed_m_s,
                        std::sqrt((10/std::sqrt(3.0))*std::abs(distance_)/accel_m_s2));
  }
  double duration() const { return duration_; }
  Pose at(const Pose & initial, double time_s) const {
    if (!std::isfinite(time_s)) throw std::invalid_argument("nonfinite trajectory time");
    const double u = std::clamp(time_s/duration_, 0.0, 1.0);
    const double s = u*u*u*(10 + u*(-15 + 6*u));
    Pose pose = initial;
    pose[axis_] += distance_*s;
    return pose;
  }
 private:
  int axis_;
  double distance_, duration_;
};

inline std::string state_problem(const ROBOT_STATE_PKG & p, bool starting) {
  if (p.frame_head != 0x5A5A) return "no valid feedback";
  if (p.EmergencyStop || p.safety_stop0_state || p.safety_stop1_state || p.collisionState)
    return "emergency/safety/collision stop is active";
  if (p.main_code || p.sub_code) return "controller reports a fault";
  if (!p.rbtEnableState) return "robot is not enabled";
  if (p.robot_mode != 0) return "select automatic mode on the pendant before motion";
  if (starting && p.program_state != 1) return "stop the other controller program before motion";
  // ServoMoveStart itself changes program_state to running on controller V3.9.2.
  if (!starting && p.program_state != 1 && p.program_state != 2)
    return "controller program is paused or in an unknown state";
  if (p.robot_state != 1 && p.robot_state != 2) return "robot state is not idle/running";
  if (starting && p.robot_state != 1) return "robot must be stationary before the small move";
  if (p.tool != 0 || p.user != 0) return "this probe uses tool 0 and work object 0";
  for (int i=0; i<6; ++i) {
    if (!std::isfinite(p.toolCoord[i]) || !std::isfinite(p.wobjCoord[i]) ||
        std::abs(p.toolCoord[i]) > 1e-6 || std::abs(p.wobjCoord[i]) > 1e-6)
      return "tool 0 and work object 0 must have zero transforms";
    if (!std::isfinite(p.flange_cur_pos[i])) return "nonfinite flange feedback";
  }
  return "";
}

class Client {
 public:
  virtual ~Client() = default;
  virtual int connect(const char *) = 0;
  virtual int read(ROBOT_STATE_PKG *) = 0;
  virtual int start() = 0;
  virtual int send(const Pose &, double period_s) = 0;
  virtual int stop() = 0;
  virtual int end() = 0;
  virtual int close() = 0;
};

struct StopResult { int stop_code = 0, end_code = 0; };
class ServoSession {
 public:
  explicit ServoSession(Client & client) : client_(client) {}
  ~ServoSession() { if (active_) finish(); }
  int start() {
    if (active_) throw std::logic_error("servo already active");
    active_ = true;  // Even a failed start gets best-effort StopMotion/ServoMoveEnd.
    return client_.start();
  }
  int send(const Pose & pose, double period_s) {
    if (!active_) throw std::logic_error("servo is not active");
    if (!std::isfinite(period_s) || period_s < .001 || period_s > .016)
      throw std::invalid_argument("period must be 1..16 ms");
    to_sdk(pose);  // Reject invalid targets before any SDK call.
    return client_.send(pose, period_s);
  }
  StopResult finish() {
    if (!active_) return result_;
    result_.stop_code = client_.stop();
    result_.end_code = client_.end();  // Attempt end even when StopMotion fails.
    active_ = false;
    return result_;
  }
  bool active() const { return active_; }
 private:
  Client & client_;
  bool active_ = false;
  StopResult result_;
};
}  // namespace fr3c_follow_driver
