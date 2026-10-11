#pragma once
#include "robot_types.h"
#include <cmath>
// Read-only test stand-in for the vendor's absolute-pose RPC contract.
class FRRobot {
 public:
  inline static DescPose last_servo{}, last_inverse{};
  inline static int sends=0, inverses=0;
  static bool bounded(const DescPose & p) {
    return std::isfinite(p.rpy.rx) && std::isfinite(p.rpy.ry) && std::isfinite(p.rpy.rz) &&
      std::abs(p.rpy.rx)<=180 && std::abs(p.rpy.ry)<=180 && std::abs(p.rpy.rz)<=180;
  }
  int RPC(const char *) { return 0; }
  int GetRobotRealTimeState(ROBOT_STATE_PKG *) { return 0; }
  int ServoMoveStart() { return 0; }
  int ServoCart(int,DescPose * p,float[6],float,float,float,float,float) {
    ++sends; last_servo=*p; return bounded(*p)?0:112;
  }
  int StopMotion() { return 0; }
  int ServoMoveEnd() { return 0; }
  int CloseRPC() { return 0; }
  int GetInverseKinRef(int,DescPose * p,JointPos * seed,JointPos * solved) {
    ++inverses; last_inverse=*p; *solved=*seed; return bounded(*p)?0:112;
  }
};
