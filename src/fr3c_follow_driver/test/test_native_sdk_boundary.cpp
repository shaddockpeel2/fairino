#include <gtest/gtest.h>
#include "fr3c_follow_driver/clients.hpp"
#include "fr3c_follow_driver/rotation_probe.hpp"
using namespace fr3c_follow_driver;
static constexpr double deg=kPi/180;
static Pose sdk_pose(const DescPose & p) {
  const double raw[]{p.tran.x,p.tran.y,p.tran.z,p.rpy.rx,p.rpy.ry,p.rpy.rz};
  return from_sdk(raw);
}

TEST(NativeSdkBoundary, RecordedZFailureAndReturnCross180WithoutChangingTcpTrajectory) {
  const Pose tool{-.000847397,.003027928,.184614459,0,0,0};
  const Pose initial{-.06620733642578125,-.4156016235351563,.34644281005859373,
    3.072192007162873,-.029342744260601294,3.1372562283868173};
  TcpRotation plan(tool,Eigen::Vector3d::UnitZ(),2.5*deg,3*deg,10*deg);
  plan.anchor(initial);
  NativeClient client;
  ServoSession session(client); ASSERT_EQ(session.start(),0);
  std::array<double,6> seed{-1.4784650644340673,-1.8505503471891969,-1.1050812909104428,
    -1.8286835309527194,1.5483524035501994,.09848992225017829},solved{};
  Pose previous=initial;
  bool crossed=false;
  for (double t=0;t<=plan.duration()+.008;t+=.008) {
    const Pose target=plan.command(std::min(t,plan.duration()),previous);
    if (target[5]>kPi) {
      crossed=true; EXPECT_FALSE(FRRobot::bounded(to_sdk(target)));
    }
    ASSERT_EQ(client.inverse(target,seed,solved),0);
    ASSERT_EQ(session.send(target,.008),0);
    EXPECT_TRUE(FRRobot::bounded(FRRobot::last_servo));
    EXPECT_TRUE(FRRobot::bounded(FRRobot::last_inverse));
    const auto sent_tcp=pose_transform(sdk_pose(FRRobot::last_servo))*pose_transform(tool);
    const auto expected=plan.reference_tcp(std::min(t,plan.duration()));
    EXPECT_TRUE(sent_tcp.isApprox(expected,1e-10));
    EXPECT_TRUE(rotation(sdk_pose(FRRobot::last_inverse)).isApprox(rotation(target),1e-10));
    for (int i=0;i<6;++i) EXPECT_NEAR(solved[i],seed[i],1e-12);
    previous=target;
  }
  EXPECT_TRUE(crossed);
  const auto stop=session.finish(); EXPECT_EQ(stop.stop_code,0); EXPECT_EQ(stop.end_code,0);
}
TEST(NativeSdkBoundary, EveryEulerAxisAndDirectionStaysEquivalentAndInvalidInputNeverCallsSdk) {
  NativeClient client;
  std::array<double,6> seed{},solved{};
  for (int axis=3;axis<6;++axis) for (double sign : {-1.,1.}) {
    Pose p{.1,-.2,.3,.2,-.3,.4}; p[axis]=sign*181*deg;
    ASSERT_EQ(client.inverse(p,seed,solved),0);
    ASSERT_EQ(client.send(p,.008),0);
    EXPECT_TRUE(FRRobot::bounded(FRRobot::last_servo));
    EXPECT_TRUE(rotation(sdk_pose(FRRobot::last_servo)).isApprox(rotation(p),1e-10));
    EXPECT_TRUE(position(sdk_pose(FRRobot::last_servo)).isApprox(position(p),1e-10));
  }
  const int sends=FRRobot::sends,inverses=FRRobot::inverses;
  Pose p{}; p[5]=NAN;
  EXPECT_THROW(client.inverse(p,seed,solved),std::invalid_argument);
  EXPECT_THROW(client.send(p,.008),std::invalid_argument);
  EXPECT_EQ(sends,FRRobot::sends); EXPECT_EQ(inverses,FRRobot::inverses);
}
