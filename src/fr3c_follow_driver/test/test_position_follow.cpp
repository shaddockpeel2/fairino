#include <gtest/gtest.h>
#include "fr3c_follow_driver/position_follow.hpp"
using namespace fr3c_follow_driver;

TEST(PositionFollow, CollisionReductionImmediatelyConstrainsExistingVelocity) {
  PositionFollow core(Eigen::Vector3d::Zero(),2.,.02,.06,1.,0);
  Pose actual{}; core.teach(Eigen::Vector3d::Zero(),actual);
  core.observe(Eigen::Vector3d(.05,0,0),.05);
  for (int i=0;i<50;++i) actual=core.step(actual,.008);
  ASSERT_GT(core.velocity().norm(),.01);
  auto constrained=core.step(actual,.008,.1);
  EXPECT_LE((position(constrained)-position(actual)).norm(),.002*.008+1e-12);
  EXPECT_LE(core.velocity().norm(),.002+1e-12);
}

TEST(PositionFollow, TcpConversionAndFixedRotationForEveryAxis) {
  const Eigen::Vector3d tcp_xyz(-.0008,.003,.1846), target(.01,-.2,.03);
  Pose flange{.1,-.3,.4,-3.,.1,2.};
  for (int axis=0;axis<3;++axis) {
    PositionFollow core(tcp_xyz,1.5,.01,.03,.10,0);
    core.teach(target,flange);
    EXPECT_LT((core.goal()-core.tcp(flange)).norm(),1e-12);
    Eigen::Vector3d moved=target; moved[axis]+=.01;
    core.observe(moved,.07);
    const Eigen::Vector3d goal=core.goal();
    Pose actual=flange;
    Eigen::Vector3d last_velocity=Eigen::Vector3d::Zero();
    for (int i=0;i<1000;++i) {
      const auto command=core.step(actual,.008);
      EXPECT_LE(core.velocity().norm(),.01+1e-12);
      EXPECT_LE((core.velocity()-last_velocity).norm(),.03*.008+1e-12);
      for (int j=3;j<6;++j) EXPECT_DOUBLE_EQ(command[j],flange[j]);
      actual=command; last_velocity=core.velocity();
    }
    EXPECT_LT((core.tcp(actual)-goal).norm(),1e-6);
    for (int j=0;j<3;++j) {
      if (j!=axis) { EXPECT_NEAR(actual[j],flange[j],1e-12); }
    }
  }
}
TEST(PositionFollow, MeasuredFeedbackWithLagConvergesAfterTargetStops) {
  PositionFollow core(Eigen::Vector3d(0,0,.1846),1.5,.01,.03,.10,.1);
  Pose actual{0,-.3,.4,kPi,0,0};
  Eigen::Vector3d target(0,-.3,.03); core.teach(target,actual);
  for (int i=0;i<2000;++i) {
    if (i<400 && i%8==0) { target.x()+=.00032; target.y()+=.00016; core.observe(target,.064); }
    Pose command=core.step(actual,.008);
    // Simulate actuator latency; reference must never replace actual feedback.
    for (int j=0;j<3;++j) actual[j]+=.15*(command[j]-actual[j]);
  }
  EXPECT_LT((core.goal()-core.tcp(actual)).norm(),1e-6);
  EXPECT_LT(core.velocity().norm(),1e-6);
}
TEST(PositionFollow, PausedReferenceReanchorsWithoutJumpAndRestartReteaches) {
  PositionFollow core(Eigen::Vector3d(0,0,.1846),1.5,.01,.03,.10,.1);
  Pose actual{0,-.3,.4,kPi,0,0}; Eigen::Vector3d target(0,-.3,.03);
  core.teach(target,actual); target.x()+=.01; core.observe(target,.07);
  for (int i=0;i<40;++i) actual=core.step(actual,.008);
  // Target moves during occlusion. Resume keeps offset, resets filter and velocity.
  target.x()+=.02; core.observe(target,1.,true); core.reanchor(actual);
  auto resumed=core.step(actual,.008);
  EXPECT_LE((position(resumed)-position(actual)).norm(),.03*.008*.008+1e-12);
  core.clear(); actual[1]+=.004; core.teach(target,actual);
  auto restarted=core.step(actual,.008);
  EXPECT_LT((position(restarted)-position(actual)).norm(),1e-12);
}
TEST(PositionFollow, RangeCheckedBeforeFilteringAndNoRunawayWithFrozenFeedback) {
  PositionFollow core(Eigen::Vector3d::Zero(),1.5,.01,.03,.10,.1);
  Pose actual{}; core.teach(Eigen::Vector3d::Zero(),actual);
  EXPECT_THROW(core.observe(Eigen::Vector3d(.101,0,0),.07),std::out_of_range);
  core.observe(Eigen::Vector3d(.08,0,0),.07);
  bool stopped=false;
  for (int i=0;i<500;++i) {
    try { core.step(actual,.008); }
    catch (const std::runtime_error &) { stopped=true; break; }
  }
  EXPECT_TRUE(stopped);
}
TEST(PositionFollow, NonfiniteGeometryAndObservationsNeverProduceCommands) {
  EXPECT_THROW(PositionFollow(Eigen::Vector3d(NAN,0,0),1.5,.01,.03,.1,.1),std::invalid_argument);
  PositionFollow core(Eigen::Vector3d::Zero(),1.5,.01,.03,.1,.1);
  EXPECT_THROW(core.step(Pose{},.008),std::invalid_argument);
  EXPECT_THROW(core.teach(Eigen::Vector3d(NAN,0,0),Pose{}),std::invalid_argument);
  core.teach(Eigen::Vector3d::Zero(),Pose{});
  EXPECT_THROW(core.observe(Eigen::Vector3d(NAN,0,0),.07),std::invalid_argument);
  EXPECT_THROW(core.step(Pose{},.2),std::invalid_argument);
}
