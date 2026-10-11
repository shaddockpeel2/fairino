#include <gtest/gtest.h>
#include <atomic>
#define main pose_follow_entry_for_test
#include "../src/pose_follow.cpp"
#undef main

class FaultClient final : public Client {
 public:
  std::atomic<int> mode{0};  // 1 repeated packet, 2 read error, 3 invalid pose.
  int connect(const char * ip) override { return mock.connect(ip); }
  int read(ROBOT_STATE_PKG * out) override {
    const int value=mode.load();
    if (value==1) { *out=last; return 0; }
    if (value==2) return 99;
    const int code=mock.read(out); last=*out;
    if (value==3) out->flange_cur_pos[0]=NAN;
    return code;
  }
  int start() override { return mock.start(); }
  int send(const Pose & pose,double period) override { return mock.send(pose,period); }
  int stop() override { return mock.stop(); }
  int end() override { return mock.end(); }
  int close() override { return mock.close(); }
 private:
  MockClient mock; ROBOT_STATE_PKG last{};
};

TEST(PoseFeedback, RepeatedErrorAndInvalidPacketsPauseThenResumeFromActualWithSameRelation) {
  const auto saved_signal=std::signal(SIGINT,interrupt_handler);
  for (int mode:{1,2,3}) {
    rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None); interrupted=0;
    auto injected=std::make_unique<FaultClient>(); auto * client=injected.get();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("mock",true),
      rclcpp::Parameter("tcp_xyz_m",std::vector<double>{0,0,.185}),
      rclcpp::Parameter("tcp_rpy_rad",std::vector<double>{0,0,0})});
    auto driver=std::make_shared<PoseFollowDriver>(std::move(injected),options);
    auto probe=std::make_shared<rclcpp::Node>("pose_feedback_probe_"+std::to_string(mode));
    Json state; double target_x=.05;
    auto observation=probe->create_publisher<std_msgs::msg::String>("/target_follow/pose_observation",10);
    auto subscription=probe->create_subscription<std_msgs::msg::String>("/target_follow/follow_state",20,
      [&](std_msgs::msg::String::ConstSharedPtr message) { state=Json::parse(message->data); });
    auto timer=probe->create_wall_timer(std::chrono::milliseconds(30),[&] {
      const auto ns=probe->now().nanoseconds()-20000000;
      Json stamp={{"sec",ns/1000000000LL},{"nanosec",ns%1000000000LL}};
      Json row={{"schema_version",1},{"status","valid"},{"stamp",stamp},{"target_id",1},
        {"frame_id","base_link"},{"source_frame_id","camera_color_optical_frame"},{"continuity_frame","base_link"},
        {"motion_commands_enabled",false},{"orientation_control_enabled",false},
        {"valid_position",true},{"valid_orientation",true},{"orientation_measurement_valid",true},{"confirmation_count",2},
        {"position_m",{target_x,-.3,.03}},{"quaternion_xyzw",{0.,0.,0.,1.}},
        {"quality",{{"stamp",stamp},{"status","valid"},{"frame_id","camera_color_optical_frame"},{"target_id",1},
          {"solver","IPPE_SQUARE"},{"candidate_count",2},{"selected_candidate_index",0},
          {"candidate_errors_px",{.01,.4}},{"candidate_rotation_distance_deg",30.},
          {"valid_position",true},{"orientation_measurement_valid",true},{"orientation_ambiguous",false}}}};
      std_msgs::msg::String message; message.data=row.dump(); observation->publish(message);
    });
    auto start=probe->create_client<std_srvs::srv::Trigger>("/target_follow/follow_start");
    std::thread worker([&] { driver->run(); });
    auto wait=[&](const std::function<bool()> & predicate,double timeout=5.) {
      const double end=steady()+timeout;
      while (steady()<end) { rclcpp::spin_some(probe); if (predicate()) return true; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
      return false;
    };
    // Avoid ASSERT while the worker is joinable; cleanup always runs.
    const bool ready=wait([&] { return !state.empty() && state.value("quality_reason",std::string("x")).empty() &&
      start->service_is_ready() && probe->count_publishers("/target_follow/flange_pose")==1; });
    EXPECT_TRUE(ready) << state.dump();
    if (ready) {
      auto future=start->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      const bool started=wait([&] { return future.wait_for(std::chrono::milliseconds(0))==std::future_status::ready; });
      EXPECT_TRUE(started);
      if (started) { EXPECT_TRUE(future.get()->success); }
      target_x=.065;
      EXPECT_TRUE(wait([&] { return state.value("servo_commands_sent",0)>30; }));
      client->mode.store(mode);
      const bool paused=wait([&] { return state.value("status",std::string())=="paused_feedback"; });
      EXPECT_TRUE(paused) << state.dump();
      if (paused) {
        const Json offset=state["offset_matrix"];
        const int sent=state["servo_commands_sent"];
        EXPECT_EQ(state["velocity_m_s"],Json::array({0.,0.,0.}));
        EXPECT_EQ(state["angular_velocity_deg_s"],Json::array({0.,0.,0.}));
        client->mode.store(0);
        EXPECT_TRUE(wait([&] { return state.value("status",std::string())=="following" && state.value("servo_commands_sent",0)>sent; })) << state.dump();
        EXPECT_EQ(offset,state["offset_matrix"]);
        EXPECT_GE(state.value("target_resumes",0),1);
      }
    }
    client->mode.store(0); std::raise(SIGINT); worker.join();
    timer->cancel(); timer.reset(); start.reset(); observation.reset(); subscription.reset();
    driver.reset(); probe.reset(); rclcpp::shutdown();
  }
  std::signal(SIGINT,saved_signal);
}
