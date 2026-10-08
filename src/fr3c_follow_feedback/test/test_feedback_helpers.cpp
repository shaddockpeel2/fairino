#include <gtest/gtest.h>
#include <limits>
#include "fr3c_follow_feedback/helpers.hpp"
using namespace fr3c_follow_feedback;

TEST(FeedbackGeometry, MillimetresAndFixedAxisDegrees) {
  double values[6]{100, -200, 300, 90, 0, 90};
  auto pose = pose_mm_deg(values);
  EXPECT_NEAR(pose.translation().x(), .1, 1e-12);
  EXPECT_NEAR(pose.translation().y(), -.2, 1e-12);
  auto rotated = pose.rotation() * Eigen::Vector3d::UnitY();
  EXPECT_NEAR(rotated.x(), 0, 1e-12);
  EXPECT_NEAR(rotated.y(), 0, 1e-12);
  EXPECT_NEAR(rotated.z(), 1, 1e-12);
}
TEST(FeedbackGeometry, RejectsNonfiniteState) {
  double values[6]{0, 0, std::numeric_limits<double>::quiet_NaN(), 0, 0, 0};
  EXPECT_THROW(pose_mm_deg(values), std::invalid_argument);
}
TEST(FeedbackFreshness, CachedPacketCannotBecomeFreshByPolling) {
  PacketFreshness tracker;
  PacketFreshness::Key key{255, 2026, 10, 8, 13, 0, 0, 0};
  EXPECT_TRUE(tracker.observe(key, 1.0));
  EXPECT_FALSE(tracker.observe(key, 1.1));
  EXPECT_NEAR(tracker.age(1.3), .3, 1e-12);
  key[0] = 0; key[7] = 8;
  EXPECT_TRUE(tracker.observe(key, 1.4));
  EXPECT_NEAR(tracker.age(1.4), 0, 1e-12);
}
