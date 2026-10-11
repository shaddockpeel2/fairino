#include <gtest/gtest.h>
#include "fr3c_follow_driver/rotation_probe.hpp"
using namespace fr3c_follow_driver;
static constexpr double deg=kPi/180;

TEST(TcpRotation, AllBaseAxesCompensateFullToolAndReturnFromDifferentStarts) {
  const Pose tool{.002,-.003,.1846,.1,-.2,.3};
  for (Eigen::Vector3d axis : {Eigen::Vector3d(1,0,0),Eigen::Vector3d(0,1,0),
                              Eigen::Vector3d(0,0,1),Eigen::Vector3d(1,1,1)}) {
    for (const Pose initial : {Pose{.2,-.3,.4,3.0,.2,-.7}, Pose{-.1,-.2,.5,-2.8,-.3,.6}}) {
      TcpRotation plan(tool,axis,-2.5*deg,3*deg,10*deg);
      plan.anchor(initial);
      Pose previous=initial;
      const auto tcp0=pose_transform(initial)*pose_transform(tool);
      EXPECT_TRUE(pose_transform(plan.command(0,previous)).isApprox(pose_transform(initial),1e-10));
      bool flange_translated=false;
      for (double t=0;t<=plan.duration();t+=.008) {
        Pose command=plan.command(t,previous);
        const auto reconstructed=pose_transform(command)*pose_transform(tool);
        EXPECT_TRUE(reconstructed.translation().isApprox(tcp0.translation(),1e-10));
        EXPECT_TRUE(reconstructed.linear().isApprox(plan.reference_tcp(t).linear(),1e-10));
        if ((position(command)-position(initial)).norm()>.001) flange_translated=true;
        previous=command;
      }
      EXPECT_TRUE(flange_translated);
      const auto end=pose_transform(plan.command(plan.duration(),previous))*pose_transform(tool);
      EXPECT_TRUE(end.isApprox(tcp0,1e-10));
      const auto peak=plan.reference_tcp(plan.leg_duration());
      EXPECT_TRUE(peak.linear().isApprox(
        Eigen::AngleAxisd(-2.5*deg,axis.normalized()).toRotationMatrix()*tcp0.linear(),1e-10));
    }
  }
}
TEST(TcpRotation, NumericalAngularVelocityAndAccelerationRespectLimitsIncludingTurnaround) {
  TcpRotation plan(Pose{},Eigen::Vector3d::UnitY(),3*deg,3*deg,10*deg);
  const double dt=.0005;
  double last_angle=plan.angle_at(0),last_velocity=0;
  for (double t=dt;t<plan.duration()+dt;t+=dt) {
    const double angle=plan.angle_at(t),velocity=(angle-last_angle)/dt;
    EXPECT_LE(std::abs(velocity),3*deg+1e-8);
    EXPECT_LE(std::abs((velocity-last_velocity)/dt),10*deg+1e-7);
    last_angle=angle; last_velocity=velocity;
  }
  EXPECT_DOUBLE_EQ(plan.angle_at(plan.duration()+1),0);
}
TEST(PoseGeometry, ContinuousSdkEulerAcross180AndBothPitchSingularities) {
  for (double sign : {-1.,1.}) {
    Pose previous{.1,.2,.3,.4,sign*89*deg,.6};
    for (int i=0;i<=200;++i) {
      Pose raw=previous; raw[3]=.4; raw[4]=sign*(89+i*.01)*deg; raw[5]=.6;
      Pose next=continuous_pose(pose_transform(raw),previous);
      EXPECT_TRUE(rotation(next).isApprox(rotation(raw),1e-9));
      for (int j=3;j<6;++j) EXPECT_LT(std::abs(next[j]-previous[j]),.02);
      previous=next;
    }
  }
  Pose previous{0,0,0,179*deg,0,179*deg};
  for (double angle : {179.,180.,181.,182.}) {
    Pose raw{0,0,0,angle*deg,0,angle*deg};
    auto next=continuous_pose(pose_transform(raw),previous);
    EXPECT_NEAR(next[3],angle*deg,1e-10); EXPECT_NEAR(next[5],angle*deg,1e-10);
    EXPECT_TRUE(rotation(next).isApprox(rotation(raw),1e-10)); previous=next;
  }
}
TEST(TcpRotation, OneWayHoldsEndpointAndRejectsInvalidInput) {
  TcpRotation plan(Pose{},Eigen::Vector3d::UnitZ(),2*deg,3*deg,10*deg,.5,false);
  EXPECT_NEAR(plan.angle_at(plan.duration()),2*deg,1e-12);
  EXPECT_THROW(plan.command(0,Pose{}),std::logic_error);
  EXPECT_THROW(TcpRotation(Pose{},Eigen::Vector3d::Zero(),2*deg,3*deg,10*deg),std::invalid_argument);
  EXPECT_THROW(TcpRotation(Pose{},Eigen::Vector3d::UnitZ(),6*deg,3*deg,10*deg),std::invalid_argument);
  auto invalid=Eigen::Isometry3d::Identity(); invalid.linear()(0,0)=2;
  EXPECT_THROW(continuous_pose(invalid,Pose{}),std::invalid_argument);
}
