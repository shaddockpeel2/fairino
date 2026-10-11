#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <geometric_shapes/shapes.h>
#include "fr3c_follow_driver/collision_guard.hpp"
using namespace fr3c_follow_driver;
static std::string read_file(const std::string & path) { std::ifstream f(path); std::stringstream s; s<<f.rdbuf(); return s.str(); }
static std::shared_ptr<CollisionModel> create_model(CollisionOptions options={}) {
  return std::make_shared<CollisionModel>(read_file(COLLISION_URDF),read_file(COLLISION_SRDF),options);
}
static CollisionInput input(const CollisionModel & model) {
  CollisionInput in; in.pose_motion=true; in.stamp=1;
  in.joints={2.1766,-1.6214,1.5477,-1.5877,-1.355685307179586,0}; in.flange=model.flange(in.joints);
  in.flange_tcp=pose_transform(Pose{-.000847397,.003027928,.184614459,.1,-.2,.3}); return in;
}
TEST(PoseCollision, PureRotationWithNonzeroToolAndRotationalReferenceLeadIsPredicted) {
  auto model=create_model(); auto in=input(*model);
  in.requested_angular={0,0,3*kPi/180}; in.rotation_lead=rotation_exp(Eigen::Vector3d(0,0,2*kPi/180));
  const auto out=model->check(in);
  EXPECT_TRUE(out.ready); EXPECT_TRUE(out.pose_motion); EXPECT_FALSE(out.colliding);
  EXPECT_GT(out.scale,0) << out.reason; EXPECT_TRUE(out.requested_angular.isApprox(in.requested_angular));
}
TEST(PoseCollision, TableSweepAndAngularBrakingAreCheckedEvenWithZeroTranslation) {
  auto baseline=create_model(); auto in=input(*baseline);
  for (int i=0;i<180 && (pose_transform(in.flange)*in.flange_tcp).translation().z()>.020;++i) {
    auto target=in.flange; target[2]-=.002; ASSERT_TRUE(baseline->inverse(target,in.joints)); in.flange=baseline->flange(in.joints);
  }
  auto state=moveit::core::RobotState(baseline->model()); state.setToDefaultValues();
  state.setVariablePosition("gripper_command_joint",.027);
  for (unsigned i=0;i<6;++i) state.setVariablePosition("joint"+std::to_string(i+1),in.joints[i]);
  state.update();
  double bottom=1e9;
  for (const auto & name:{"gripper_link","camera_housing_link","gripper_finger_px_py_link","gripper_finger_px_ny_link",
                         "gripper_finger_nx_py_link","gripper_finger_nx_ny_link"}) {
    const auto * link=baseline->model()->getLinkModel(name);
    for (std::size_t i=0;i<link->getShapes().size();++i) {
      const auto * box=dynamic_cast<const shapes::Box *>(link->getShapes()[i].get()); if (!box) continue;
      const auto tf=state.getCollisionBodyTransform(link,i);
      const double height=tf.linear().row(2).cwiseAbs().dot(Eigen::Vector3d(box->size));
      bottom=std::min(bottom,tf.translation().z()-height*.5);
    }
  }
  CollisionOptions options; options.table_z=bottom-.0052; auto model=create_model(options);
  const auto current=model->check(in); ASSERT_FALSE(current.colliding) << current.reason;
  ASSERT_GT(current.distance,options.margin) << current.reason;
  bool found=false; Eigen::Vector3d unsafe=Eigen::Vector3d::Zero();
  for (int axis=0;axis<3 && !found;++axis) for (double sign:{-1.,1.}) {
    in.requested_angular.setZero(); in.requested_angular[axis]=sign*options.angular_speed;
    const auto out=model->check(in);
    if (out.predicted_distance<options.margin && out.reason.find("collision_margin")!=std::string::npos) {
      found=true; unsafe=in.requested_angular; EXPECT_EQ(out.scale,0); break;
    }
  }
  ASSERT_TRUE(found) << "A pure TCP rotation should sweep an attachment into the table margin";
  in.requested_angular=-unsafe; in.angular_velocity=unsafe;
  const auto braking=model->check(in);
  EXPECT_EQ(braking.scale,0); EXPECT_LT(braking.predicted_distance,options.margin);
}
TEST(PoseCollision, CacheRejectsRotationReversalExpiryAndPositionOnlyResults) {
  CollisionOptions options; auto model=create_model(options); CollisionGuard guard(model,options);
  auto in=input(*model); in.requested_angular={0,0,options.angular_speed};
  in.stamp=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  guard.submit(in); CollisionOutput out;
  for (int i=0;i<100 && !out.ready;++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); out=guard.result_pose(in.stamp,in.requested,in.requested_angular);
  }
  ASSERT_TRUE(out.ready); ASSERT_GT(out.scale,0) << out.reason;
  EXPECT_EQ(guard.result_pose(in.stamp,in.requested,-in.requested_angular).scale,0);
  EXPECT_EQ(guard.result_pose(in.stamp+options.timeout+.01,in.requested,in.requested_angular).scale,0);
  CollisionGuard position_guard(model,options); in.pose_motion=false; position_guard.submit(in);
  CollisionOutput positional;
  for (int i=0;i<100 && !positional.ready;++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); positional=position_guard.result(in.stamp,in.requested);
  }
  EXPECT_EQ(position_guard.result_pose(in.stamp,in.requested,in.requested_angular).scale,0);
}
