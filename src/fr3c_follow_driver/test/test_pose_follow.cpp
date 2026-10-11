#include <gtest/gtest.h>
#include "fr3c_follow_driver/pose_follow.hpp"
#include "fr3c_follow_driver/pose_observation.hpp"
using namespace fr3c_follow_driver;
static constexpr double deg=kPi/180;
static PoseFollowOptions exact_options() {
  PoseFollowOptions o; o.position_tau=o.rotation_tau=o.enter=o.exit=0; o.mapping=1;
  o.goal_position_threshold=o.goal_angle_threshold=o.goal_confirmation=0;
  o.position_deadband=.0015; o.angular_deadband=.75*deg; return o;
}
static Eigen::Isometry3d tag(double angle=0, Eigen::Vector3d axis=Eigen::Vector3d::UnitZ()) {
  auto out=Eigen::Isometry3d::Identity(); out.translation()=Eigen::Vector3d(.1,-.3,.03);
  out.linear()=Eigen::AngleAxisd(angle*deg,axis).toRotationMatrix(); return out;
}
TEST(PoseFollow, RecordsWithoutJumpAndPreservesFullRelationForEachAxisAndRotatedTool) {
  const Pose tool{.002,-.003,.185,.1,-.2,.3},initial{.2,-.3,.4,3.0,.1,179*deg};
  for (const Eigen::Vector3d & axis:std::array<Eigen::Vector3d,4>{Eigen::Vector3d(1,0,0),Eigen::Vector3d(0,1,0),Eigen::Vector3d(0,0,1),Eigen::Vector3d(1,1,1).normalized()}) {
    PoseFollow follow(tool,exact_options()); const auto start=tag(); follow.teach(start,initial);
    EXPECT_TRUE(follow.goal().isApprox(follow.tcp(initial),1e-10));
    EXPECT_TRUE(pose_transform(follow.step(initial,.008)).isApprox(pose_transform(initial),1e-10));
    auto moved=tag(10,axis); moved.translation()+=Eigen::Vector3d(.002,-.003,.004); follow.observe(moved,.1);
    EXPECT_TRUE(follow.goal().isApprox(moved*start.inverse()*follow.tcp(initial),1e-10));
  }
}
TEST(PoseFollow, HalfMappingReducesBothOrbitAndAttitudeAndIgnoresSeveralDegreeJitter) {
  const Pose initial{.25,-.3,.20,3.0,0,.2};
  auto options=PoseFollowOptions{}; options.goal_position_threshold=options.goal_angle_threshold=options.goal_confirmation=0;
  PoseFollow follow(Pose{},options); follow.teach(tag(),initial);
  for (int i=0;i<80;++i) {
    follow.observe(tag((i%2?1:-1)*4.5),.1);
    EXPECT_TRUE(follow.goal().isApprox(follow.tcp(initial),1e-10));
  }
  for (int i=0;i<60;++i) follow.observe(tag(20),.1);
  const double angle=rotation_distance(follow.goal().linear(),rotation(initial))/deg;
  EXPECT_GT(angle,7.5); EXPECT_LT(angle,8.1);
  auto mapped=tag(angle); EXPECT_TRUE(follow.goal().isApprox(mapped*tag().inverse()*follow.tcp(initial),1e-8));
  EXPECT_GT((follow.goal().translation()-position(initial)).norm(),.01);
}
TEST(PoseFollow, ClosedLoopBoundsConvergeAndSettlesWithFeedbackAndCommonTwistScaling) {
  PoseFollow follow(Pose{},exact_options()); Pose actual{.15,-.3,.20,3.0,0,179*deg};
  follow.teach(tag(),actual); auto moved=tag(12); moved.translation().x()+=.01; follow.observe(moved,.1);
  PoseTwist last;
  for (int i=0;i<1300;++i) {
    const Pose next=follow.step(actual,.008);
    const auto v=follow.velocity();
    EXPECT_LE(v.linear.norm(),.020+1e-10); EXPECT_LE(v.angular.norm(),3*deg+1e-10);
    EXPECT_LE((v.linear-last.linear).norm(),.060*.008+1e-10);
    EXPECT_LE((v.angular-last.angular).norm(),10*deg*.008+1e-10);
    EXPECT_TRUE(pose_transform(next).isApprox(follow.reference(),1e-10));
    actual=next; last=v;
  }
  EXPECT_LT((follow.goal().translation()-position(actual)).norm(),.002);
  EXPECT_LT(rotation_distance(follow.goal().linear(),rotation(actual)),1*deg);
  EXPECT_LT(follow.velocity().linear.norm(),1e-9); EXPECT_LT(follow.velocity().angular.norm(),1e-9);
}
TEST(PoseFollow, RawRangeIsCheckedBeforeFilteringAndFrozenFeedbackCannotAccumulateReferences) {
  auto limits=PoseFollowOptions{}; limits.radius=.20; limits.angular_range=20*deg;
  PoseFollow follow(Pose{},limits); const Pose actual{.1,-.3,.03,0,0,0};
  follow.teach(tag(),actual); auto within=tag(); within.translation().x()+=.101;
  EXPECT_NO_THROW(follow.observe(within,.01));
  follow.teach(tag(),actual); auto far=tag(); far.translation().x()+=.201;
  EXPECT_THROW(follow.observe(far,.01),std::out_of_range);
  EXPECT_TRUE(follow.goal().isApprox(pose_transform(actual),1e-12));
  EXPECT_THROW(follow.observe(tag(41),.01),std::out_of_range);
  auto options=exact_options(); PoseFollow stalled(Pose{},options); stalled.teach(tag(),actual);
  stalled.observe(tag(15),.1); bool failed=false;
  for (int i=0;i<1000 && !failed;++i) {
    try { stalled.step(actual,.008); } catch (const std::runtime_error &) { failed=true; }
  }
  EXPECT_TRUE(failed); EXPECT_LE(rotation_distance(stalled.reference().linear(),rotation(actual)),5*deg+1e-8);
}
TEST(PoseFollow, PauseReanchorsZeroVelocitiesKeepsFiltersAndReteachAdoptsNewActualPose) {
  const Pose actual{.2,-.3,.20,3.0,0,.1}; PoseFollow follow(Pose{}); follow.teach(tag(),actual);
  follow.observe(tag(20),.1); follow.step(actual,.008);
  const auto offset=follow.offset(),filtered=follow.filtered();
  Pose shifted=actual; shifted[0]+=.004; follow.reanchor(shifted);
  EXPECT_TRUE(follow.offset().isApprox(offset)); EXPECT_TRUE(follow.filtered().isApprox(filtered));
  EXPECT_TRUE(follow.reference().isApprox(pose_transform(shifted))); EXPECT_DOUBLE_EQ(follow.velocity().angular.norm(),0);
  EXPECT_TRUE(follow.recovery_ok(tag(22),.04,20*deg)); EXPECT_FALSE(follow.recovery_ok(tag(60),.04,20*deg));
  follow.teach(tag(60),shifted); EXPECT_TRUE(follow.goal().isApprox(pose_transform(shifted),1e-12));
}
TEST(PoseFollow, DemoAllowsLargeMappedTiltWithoutRemovingReferenceLeadCheck) {
  PoseFollow follow(Pose{.002,-.003,.185,.1,-.2,.3});
  Pose actual{.2,-.3,.4,3.0,.1,.2}; follow.teach(tag(),actual);
  for (int i=0;i<1600;++i) {
    if (i%10==0) follow.observe(tag(std::min(120.,i*.1),Eigen::Vector3d::UnitY()),.08);
    actual=follow.step(actual,.008);
  }
  EXPECT_GT(rotation_distance(follow.tcp(actual).linear(),follow.tcp(Pose{.2,-.3,.4,3.0,.1,.2}).linear()),25*deg);
  EXPECT_TRUE(follow.in_range(follow.reference()));
}
TEST(PoseFollow, StaticVisualAndFeedbackNoiseFreezeWholeGoalAfterArrivalAndRealMotionReleases) {
  PoseFollow follow(Pose{.002,-.003,.185,.1,-.2,.3});
  Pose actual{.2,-.3,.4,3.0,.1,.2}; follow.teach(tag(),actual);
  auto moved=tag(35,Eigen::Vector3d::UnitY()); moved.translation().x()+=.025;
  for (int i=0;i<2000;++i) {
    if (i%10==0) follow.observe(moved,.08);
    actual=follow.step(actual,.008);
  }
  ASSERT_TRUE(follow.holding()); ASSERT_LT(follow.velocity().linear.norm(),1e-12);
  const auto goal=follow.goal(),reference=follow.reference(); const Pose stopped=actual;
  for (int i=0;i<800;++i) {
    if (i%10==0) {
      const double sign=i%20?1:-1;
      auto noisy=moved; noisy.translation()+=sign*Eigen::Vector3d(.002,-.002,.001);
      noisy.linear()=rotation_exp(sign*3*deg*Eigen::Vector3d::UnitY())*moved.linear();
      follow.observe(noisy,.08);
    }
    Pose feedback=stopped; feedback[0]+=(i%2?1:-1)*.0005; feedback[3]+=(i%2?1:-1)*.2*deg;
    follow.step(feedback,.008);
    EXPECT_TRUE(follow.holding()); EXPECT_TRUE(follow.goal().isApprox(goal,1e-12));
    EXPECT_TRUE(follow.reference().isApprox(reference,1e-12));
  }
  auto real=moved; real.translation().x()+=.025;
  for (int i=0;i<60 && follow.holding();++i) follow.observe(real,.05);
  EXPECT_FALSE(follow.holding()); EXPECT_GT(follow.requested(stopped).linear.norm(),.001);
  follow.reanchor(stopped); EXPECT_FALSE(follow.goal_pending());
  follow.teach(real,stopped); EXPECT_TRUE(follow.holding());
  EXPECT_TRUE(follow.goal().isApprox(follow.tcp(stopped),1e-12));
}
TEST(PoseFollow, SlowMovementAccumulatesAndSingleSpikesOrAlternatingNoiseDoNotRelease) {
  auto options=PoseFollowOptions{}; options.position_tau=options.rotation_tau=0;
  PoseFollow follow(Pose{},options); const Pose actual{.1,-.3,.03,0,0,0}; follow.teach(tag(),actual);
  auto spike=tag(); spike.translation().x()+=.015;
  follow.observe(spike,.1); EXPECT_TRUE(follow.goal_pending()); EXPECT_TRUE(follow.holding());
  follow.observe(tag(),.1); EXPECT_FALSE(follow.goal_pending());
  for (int i=0;i<30;++i) {
    auto noise=tag(); noise.translation().x()+=(i%2?1:-1)*.015;
    follow.observe(noise,.05); EXPECT_TRUE(follow.holding());
  }
  follow.teach(tag(),actual);
  for (int i=1;i<=25;++i) { auto slow=tag(); slow.translation().x()+=i*.0005; follow.observe(slow,.05); }
  EXPECT_FALSE(follow.holding()); EXPECT_GT(follow.goal().translation().x()-actual[0],.008);
  follow.teach(tag(),actual); follow.observe(spike,.1);
  follow.observe(spike,1.0); EXPECT_TRUE(follow.holding());  // missing frames cannot confirm movement
  follow.observe(spike,.1); EXPECT_TRUE(follow.holding());
  follow.observe(spike,.1); EXPECT_FALSE(follow.holding());
}
TEST(PoseFollow, ResumeKeepsStaticAnchorAndDoesNotAbsorbRealTargetMovementIntoHold) {
  auto options=PoseFollowOptions{}; options.position_tau=options.rotation_tau=0;
  PoseFollow follow(Pose{},options); const Pose actual{.1,-.3,.03,0,0,0}; follow.teach(tag(),actual);
  auto moved=tag(); moved.translation().x()+=.025;
  follow.observe(moved,.5); EXPECT_TRUE(follow.goal_pending());
  follow.reanchor(actual); follow.step(actual,.008);
  EXPECT_TRUE(follow.holding()); EXPECT_TRUE(follow.goal().isApprox(pose_transform(actual),1e-12));
  for (int i=0;i<6;++i) follow.observe(moved,.05);
  EXPECT_FALSE(follow.holding()); EXPECT_TRUE(follow.goal().isApprox(moved,1e-12));
  follow.reanchor(actual); follow.step(actual,.008);
  EXPECT_FALSE(follow.holding()); EXPECT_GT(follow.velocity().linear.norm(),0);
}
TEST(PoseGeometry, ShortestRotationAcross180AndDeterministicPiLog) {
  const auto a=rotation_exp(Eigen::Vector3d(0,0,179*deg)),b=rotation_exp(Eigen::Vector3d(0,0,-179*deg));
  EXPECT_NEAR(rotation_log(b*a.transpose()).z(),2*deg,1e-10);
  for (const Eigen::Vector3d & axis:{Eigen::Vector3d(-1,0,0),Eigen::Vector3d(0,-1,0),Eigen::Vector3d(0,0,-1)}) {
    const auto value=rotation_exp(kPi*axis); const auto vector=rotation_log(value);
    EXPECT_TRUE(rotation_exp(vector).isApprox(value,1e-10)); EXPECT_GE(vector.sum(),0);
  }
}

static ObservationJson observation() {
  ObservationJson stamp={{"sec",1800000000},{"nanosec",123456789}};
  return {{"schema_version",1},{"frame_id","base_link"},{"source_frame_id","camera_color_optical_frame"},
    {"motion_commands_enabled",false},{"orientation_control_enabled",false},{"status","valid"},
    {"valid_orientation",true},{"valid_position",true},{"orientation_measurement_valid",true},{"target_id",1},
    {"continuity_frame","base_link"},{"confirmation_count",2},{"stamp",stamp},
    {"position_m",{.1,-.3,.03}},{"quaternion_xyzw",{0.,0.,0.,-1.}},
    {"quality",{{"stamp",stamp},{"frame_id","camera_color_optical_frame"},{"target_id",1},{"status","valid"},
      {"valid_position",true},{"orientation_ambiguous",false},{"orientation_measurement_valid",true},
      {"solver","IPPE_SQUARE"},{"candidate_count",2},{"selected_candidate_index",0},
      {"candidate_errors_px",{.01,.4}},{"candidate_rotation_distance_deg",30.}}}};
}
TEST(PoseObservation, ExactStampQuaternionSignAndSoftQualityAreDistinctFromTrustedPose) {
  auto row=observation(); auto parsed=parse_observation(row,1);
  EXPECT_TRUE(parsed.trusted); EXPECT_EQ(parsed.stamp,1800000000123456789LL);
  EXPECT_TRUE(parsed.pose.isApprox(tag()));
  row["valid_orientation"]=false; row["status"]="orientation_ambiguous";
  parsed=parse_observation(row,1); EXPECT_FALSE(parsed.trusted); EXPECT_TRUE(parsed.soft);
  row["status"]="orientation_jump"; parsed=parse_observation(row,1); EXPECT_FALSE(parsed.soft);
}

TEST(PoseObservation, ApproximateCandidatePolicyIsExplicitBoundedAndCannotPromoteInvalidData) {
  auto row=observation(); auto & quality=row["quality"];
  quality["selected_candidate_index"]=1;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  quality["orientation_candidate_tolerance_deg"]=35.; quality["orientation_approximate"]=true;
  EXPECT_TRUE(parse_observation(row,1).trusted);
  quality["candidate_rotation_distance_deg"]=36.;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  quality["candidate_rotation_distance_deg"]=30.; quality["orientation_approximate"]=false;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  quality["orientation_approximate"]=true; quality["orientation_candidate_tolerance_deg"]=46.;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  quality["orientation_candidate_tolerance_deg"]=35.; quality["orientation_ambiguous"]=true;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
}
TEST(PoseObservation, RejectsNanosecondMismatchMissingQualityAlternativePoseAndBadGeometry) {
  auto row=observation(); row["quality"]["stamp"]["nanosec"]=123456788;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  row=observation(); row["quality"]["selected_candidate_index"]=1;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  row=observation(); row["quaternion_xyzw"]={0.,0.,0.,0.};
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  row=observation(); row["confirmation_count"]=1;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
  row=observation(); row["stamp"]["sec"]=1800000000.0;
  EXPECT_THROW(parse_observation(row,1),std::invalid_argument);
}
