#pragma once
#include <algorithm>
#include <functional>
#include "robot.h"
#include "fr3c_follow_driver/servo_core.hpp"

namespace fr3c_follow_driver {
class NativeClient final : public Client {
 public:
  int connect(const char * ip) override { return robot_.RPC(ip); }
  int read(ROBOT_STATE_PKG * p) override { return robot_.GetRobotRealTimeState(p); }
  int start() override { return robot_.ServoMoveStart(); }
  int send(const Pose & pose, double period_s) override {
    DescPose target=to_sdk(pose); float gains[6]{1,1,1,1,1,1};
    return robot_.ServoCart(0, &target, gains, 0, 0, static_cast<float>(period_s), 0, 0);
  }
  int stop() override { return robot_.StopMotion(); }
  int end() override { return robot_.ServoMoveEnd(); }
  int close() override { return robot_.CloseRPC(); }
 private:
  FRRobot robot_;
};
class MockClient final : public Client {
 public:
  MockClient() {
    packet_.frame_head=0x5A5A; packet_.program_state=1; packet_.robot_state=1;
    packet_.rbtEnableState=1; packet_.flange_cur_pos[1]=-300;
    packet_.flange_cur_pos[2]=400; packet_.flange_cur_pos[3]=180;
  }
  void use_model(const Pose & initial, const std::array<double,6> & joints,
                 std::function<bool(const Pose &,std::array<double,6> &)> solve) {
    solve_=std::move(solve); joints_=joints; set_pose(initial); set_joints();
  }
  int connect(const char *) override { return 0; }
  int read(ROBOT_STATE_PKG * p) override {
    ++packet_.frame_cnt; packet_.robotTime.millisecond=(++count_)%1000;
    std::copy(packet_.flange_cur_pos, packet_.flange_cur_pos+6, packet_.tl_cur_pos);
    *p=packet_; return 0;
  }
  int start() override { active_=true; packet_.program_state=2; packet_.robot_state=2; return 0; }
  int send(const Pose & pose, double) override {
    if (!active_) return 1;
    if (solve_ && !solve_(pose,joints_)) return 2;
    set_pose(pose); set_joints(); return 0;
  }
  int stop() override { packet_.program_state=1; packet_.robot_state=1; return 0; }
  int end() override { active_=false; return 0; }
  int close() override { return 0; }
 private:
  void set_pose(const Pose & pose) {
    const auto p=to_sdk(pose);
    double raw[6]{p.tran.x,p.tran.y,p.tran.z,p.rpy.rx,p.rpy.ry,p.rpy.rz};
    std::copy(raw, raw+6, packet_.flange_cur_pos);
  }
  void set_joints() { for (unsigned i=0;i<6;++i) packet_.jt_cur_pos[i]=joints_[i]*180/3.14159265358979323846; }
  std::array<double,6> joints_{};
  std::function<bool(const Pose &,std::array<double,6> &)> solve_;
  ROBOT_STATE_PKG packet_{};
  unsigned count_=0;
  bool active_=false;
};

}  // namespace fr3c_follow_driver
