#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <random>
#include <geometric_shapes/shapes.h>
#include <nlohmann/json.hpp>
#include "fr3c_follow_driver/collision_guard.hpp"
using namespace fr3c_follow_driver;
namespace {
std::string read(const std::string & path) { std::ifstream f(path); std::stringstream s; s<<f.rdbuf(); return s.str(); }
Joints pose1{2.1766,-1.6214,1.5477,-1.5877,-1.355685307179586,0};
std::shared_ptr<CollisionModel> create(CollisionOptions options={}) {
  return std::make_shared<CollisionModel>(read(COLLISION_URDF),read(COLLISION_SRDF),options);
}
CollisionInput input(const CollisionModel & model, Joints q=pose1) {
  CollisionInput in; in.joints=q; in.flange=model.flange(q); in.stamp=1; return in;
}
}
TEST(CollisionGuard, ModelAndLocalInverseAgree) {
  auto model=create(); auto in=input(*model);
  auto result=model->check(in);
  std::cout<<"flange="<<in.flange[0]<<","<<in.flange[1]<<","<<in.flange[2]
           <<" distance="<<result.distance<<" pair="<<result.pair<<" ms="<<result.check_ms<<"\n";
  ASSERT_TRUE(result.ready); ASSERT_FALSE(result.colliding); ASSERT_GT(result.scale,0) << result.reason;
  auto target=in.flange; target[0]+=.005; auto q=in.joints;
  ASSERT_TRUE(model->inverse(target,q));
  EXPECT_LT((position(model->flange(q))-position(target)).norm(),.00005);
  EXPECT_LT(Eigen::AngleAxisd(rotation(model->flange(q))*rotation(target).transpose()).angle(),.0002);
}
TEST(CollisionGuard, TableBlocksAttachmentsBeforePenetrationAndAllowsRetreat) {
  auto baseline=create(); auto in=input(*baseline);
  // Lower the end effector near the base plane rather than raising a whole
  // tabletop into the shoulder. No trajectory is executed on hardware.
  const Eigen::Vector3d tcp_xyz(-.000847397,.003027928,.184614459);
  while ((position(in.flange)+rotation(in.flange)*tcp_xyz).z()>.020) {
    auto target=in.flange; target[2]-=.002;
    ASSERT_TRUE(baseline->inverse(target,in.joints)); in.flange=baseline->flange(in.joints);
  }
  std::ofstream(std::filesystem::path(COLLISION_URDF).parent_path()/"collision_low_pose.json")
    <<nlohmann::json{{"joints_rad",in.joints},{"flange",in.flange}}.dump(2);
  // Put the fixed tabletop just below the lowest moving link at this pose.
  auto state=moveit::core::RobotState(baseline->model()); state.setToDefaultValues();
  state.setVariablePosition("gripper_command_joint",.027);
  for (unsigned i=0;i<6;++i) state.setVariablePosition("joint"+std::to_string(i+1),in.joints[i]);
  state.update();
  double bottom=1e9;
  for (const auto & name : {"gripper_link","camera_housing_link","gripper_finger_px_py_link", "gripper_finger_nx_ny_link"}) {
    auto link=baseline->model()->getLinkModel(name);
    for (std::size_t k=0;k<link->getShapes().size();++k) {
      const auto * box=dynamic_cast<const shapes::Box *>(link->getShapes()[k].get());
      if (!box) continue;
      const auto tf=state.getCollisionBodyTransform(link,k);
      const double extent=tf.linear().row(2).cwiseAbs().dot(Eigen::Vector3d(box->size));
      bottom=std::min(bottom,tf.translation().z()-extent*.5);
    }
  }
  CollisionOptions options; options.table_z=bottom-.0055;
  auto model=create(options); in=input(*model,in.joints); in.requested={0,0,-options.speed};
  auto down=model->check(in);
  EXPECT_FALSE(down.colliding); EXPECT_EQ(down.scale,0) << down.reason;
  EXPECT_LT(down.predicted_distance,options.margin);
  in.requested={0,0,options.speed};
  auto up=model->check(in); EXPECT_GT(up.scale,0) << up.reason;
  options.table_z=bottom+.002; auto penetrating=create(options);
  auto hit=penetrating->check(input(*penetrating,in.joints)); EXPECT_TRUE(hit.colliding); EXPECT_EQ(hit.scale,0);
}
TEST(CollisionGuard, InvalidFeedbackFailsClosed) {
  auto model=create(); auto in=input(*model); in.flange[2]+=.05;
  auto result=model->check(in); EXPECT_EQ(result.scale,0);
  EXPECT_NE(result.reason.find("disagree"),std::string::npos);
  in=input(*model); in.joints[0]=NAN;
  EXPECT_EQ(model->check(in).scale,0);
}
TEST(CollisionGuard, ChecksSelfCollision) {
  CollisionOptions options; options.table_z=-3;
  auto model=create(options); std::mt19937 random(42);
  std::uniform_real_distribution<double> angle(-2.5,2.5);
  bool found=false;
  for (unsigned k=0;k<100;++k) {
    Joints q; for (auto & joint:q) joint=angle(random);
    auto check=model->check(input(*model,q));
    if (check.colliding && check.pair.find("follow_table")==std::string::npos) {
      found=true; EXPECT_EQ(check.scale,0); break;
    }
  }
  EXPECT_TRUE(found);
}
TEST(CollisionGuard, CachedResultExpiresAndDirectionChangesWait) {
  CollisionOptions options; auto model=create(options); CollisionGuard guard(model,options);
  auto in=input(*model); in.stamp=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  in.requested={options.speed,0,0}; guard.submit(in);
  CollisionOutput out;
  for (int i=0;i<100 && !out.ready;++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); out=guard.result(in.stamp,in.requested);
  }
  ASSERT_TRUE(out.ready); EXPECT_GT(out.scale,0) << out.reason;
  EXPECT_EQ(guard.result(in.stamp+options.timeout+.01,in.requested).scale,0);
  EXPECT_EQ(guard.result(in.stamp,-in.requested).scale,0);
}
TEST(CollisionGuard, RecordPerformanceWithoutHardware) {
  auto model=create(); auto in=input(*model); std::vector<double> times;
  in.requested={.02,0,0}; in.velocity={.01,0,0};
  for (unsigned k=0;k<120;++k) {
    auto result=model->check(in); ASSERT_GT(result.scale,0) << result.reason; times.push_back(result.check_ms);
  }
  std::sort(times.begin(),times.end());
  nlohmann::json report={{"hardware_commands_sent",0},{"samples",times.size()},
    {"check_ms_p50",times[times.size()/2]},{"check_ms_p95",times[times.size()*95/100]},
    {"check_ms_max",times.back()}};
  std::ofstream(std::filesystem::path(COLLISION_URDF).parent_path()/"collision_benchmark.json")<<report.dump(2)<<'\n';
  std::cout<<report.dump()<<'\n';
}
