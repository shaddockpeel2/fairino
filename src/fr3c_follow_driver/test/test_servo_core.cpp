#include <gtest/gtest.h>
#include <limits>
#include "fr3c_follow_driver/servo_core.hpp"
using namespace fr3c_follow_driver;

TEST(Translation, SignedAxesAndFixedOrientation) {
  Pose initial{.1, -.3, .4, -3.0, .1, 2.0};
  for (int axis=0; axis<3; ++axis) for (double distance : {-.005, .005}) {
    Translation move(axis, distance, .005, .03);
    EXPECT_EQ(move.at(initial, 0), initial);
    const auto end = move.at(initial, move.duration()+1);
    for (int i=0; i<6; ++i) EXPECT_NEAR(end[i], initial[i]+(i==axis ? distance : 0), 1e-12);
    double last=0, last_v=0;
    const double dt=move.duration()/10000;
    for (int j=1; j<=10000; ++j) {
      const double current = move.at(initial, j*dt)[axis]-initial[axis];
      const double v=(current-last)/dt;
      EXPECT_LE(std::abs(v), .005+1e-8);
      EXPECT_LE(std::abs((v-last_v)/dt), .03+1e-5);
      EXPECT_GE(current*distance, -1e-12);
      EXPECT_LE(std::abs(current), std::abs(distance)+1e-12);
      last=current; last_v=v;
    }
  }
}
TEST(Translation, RejectsUnboundedOrInvalidMove) {
  EXPECT_THROW(Translation(3, .005, .005, .03), std::invalid_argument);
  EXPECT_THROW(Translation(0, .011, .005, .03), std::invalid_argument);
  EXPECT_THROW(Translation(0, .005, 0, .03), std::invalid_argument);
  EXPECT_THROW(Translation(0, NAN, .005, .03), std::invalid_argument);
}
TEST(SdkUnits, MetresRadiansRoundTrip) {
  double input[6]{-56, -319, 412, -173.8, .7, -177.6};
  const auto sdk = to_sdk(from_sdk(input));
  EXPECT_NEAR(sdk.tran.x, input[0], 1e-10);
  EXPECT_NEAR(sdk.tran.y, input[1], 1e-10);
  EXPECT_NEAR(sdk.tran.z, input[2], 1e-10);
  EXPECT_NEAR(sdk.rpy.rx, input[3], 1e-10);
  EXPECT_NEAR(sdk.rpy.ry, input[4], 1e-10);
  EXPECT_NEAR(sdk.rpy.rz, input[5], 1e-10);
}
TEST(ControllerState, RejectsModeFaultAndCoordinateMismatch) {
  ROBOT_STATE_PKG packet{};
  packet.frame_head=0x5A5A; packet.rbtEnableState=1; packet.program_state=1; packet.robot_state=1;
  EXPECT_TRUE(state_problem(packet, true).empty());
  packet.program_state=2; packet.robot_state=2;
  EXPECT_TRUE(state_problem(packet, false).empty());
  EXPECT_FALSE(state_problem(packet, true).empty());
  packet.program_state=3; EXPECT_FALSE(state_problem(packet, false).empty());
  packet.program_state=1; packet.robot_state=1;
  packet.robot_mode=1; EXPECT_FALSE(state_problem(packet, true).empty());
  packet.robot_mode=0; packet.wobjCoord[0]=1; EXPECT_FALSE(state_problem(packet, true).empty());
  packet.wobjCoord[0]=0; packet.safety_stop0_state=1; EXPECT_FALSE(state_problem(packet, false).empty());
  packet.safety_stop0_state=0; packet.main_code=1; EXPECT_FALSE(state_problem(packet, false).empty());
}
struct FakeClient : Client {
  int starts=0, sends=0, stops=0, ends=0, start_code=0, stop_code=0;
  int connect(const char *) override { return 0; }
  int read(ROBOT_STATE_PKG *) override { return 0; }
  int start() override { ++starts; return start_code; }
  int send(const Pose &, double) override { ++sends; return 9; }
  int stop() override { ++stops; return stop_code; }
  int end() override { ++ends; return 0; }
  int close() override { return 0; }
};
TEST(ServoCleanup, FailedStartAndSendStillStopAndEndExactlyOnce) {
  for (int start_code : {0, 7}) {
    FakeClient client; client.start_code=start_code; client.stop_code=8;
    {
      ServoSession session(client); EXPECT_EQ(session.start(), start_code);
      if (!start_code) { EXPECT_EQ(session.send(Pose{}, .008), 9); }
      auto stop=session.finish(); EXPECT_EQ(stop.stop_code, 8); EXPECT_EQ(stop.end_code, 0);
      session.finish();
    }
    EXPECT_EQ(client.stops, 1); EXPECT_EQ(client.ends, 1);
  }
}
TEST(ServoCleanup, ExceptionBeforeSendingCleansUpWithoutInvalidCommand) {
  FakeClient client;
  {
    ServoSession session(client); session.start();
    Pose invalid{}; invalid[0]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(session.send(invalid, .008), std::invalid_argument);
    EXPECT_THROW(session.send(Pose{}, .020), std::invalid_argument);
  }
  EXPECT_EQ(client.sends, 0); EXPECT_EQ(client.stops, 1); EXPECT_EQ(client.ends, 1);
}
