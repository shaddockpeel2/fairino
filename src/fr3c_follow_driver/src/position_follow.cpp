#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "fr3c_follow_driver/clients.hpp"
#include "fr3c_follow_driver/position_follow.hpp"
#include "fr3c_follow_driver/collision_guard.hpp"
#include "fr3c_follow_feedback/helpers.hpp"

using namespace fr3c_follow_driver;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void interrupt_handler(int) { interrupted = 1; }
static double steady() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
static Json xyz(const Eigen::Vector3d & v) { return Json::array({v.x(),v.y(),v.z()}); }

class FollowDriver final : public rclcpp::Node {
 public:
  FollowDriver() : Node("position_follow") {
    mock_ = declare_parameter("mock",false);
    ip_ = declare_parameter("robot_ip",std::string("192.168.58.2"));
    output_ = declare_parameter("output_dir",std::string(""));
    period_ = declare_parameter("period_ms",8.0)*.001;
    target_timeout_ = declare_parameter("target_timeout_s",.35);
    feedback_timeout_ = declare_parameter("feedback_timeout_s",.20);
    double kp = declare_parameter("kp",1.5);
    double speed = declare_parameter("max_speed_m_s",.01);
    speed_limit_=speed;
    double accel = declare_parameter("max_accel_m_s2",.03);
    double range = declare_parameter("follow_radius_m",.10);
    double filter = declare_parameter("target_filter_s",.10);
    const auto tcp_xyz = declare_parameter("tcp_xyz_m",std::vector<double>{});
    const auto tcp_rpy = declare_parameter("tcp_rpy_rad",std::vector<double>{});
    if (tcp_xyz.size()!=3 || tcp_rpy.size()!=3)
      throw std::invalid_argument("provide calibrated tcp_xyz_m and tcp_rpy_rad through position_follow.launch.py");
    Pose tcp_pose{tcp_xyz[0],tcp_xyz[1],tcp_xyz[2],tcp_rpy[0],tcp_rpy[1],tcp_rpy[2]};
    to_sdk(tcp_pose);
    tcp_rotation_ = rotation(tcp_pose);
    if (!std::isfinite(period_) || period_<.001 || period_>.016 ||
        !std::isfinite(target_timeout_) || target_timeout_<=0 ||
        !std::isfinite(feedback_timeout_) || feedback_timeout_<=0)
      throw std::invalid_argument("invalid period or data timeout");
    core_ = std::make_unique<PositionFollow>(Eigen::Vector3d(tcp_xyz.data()),kp,speed,accel,range,filter);
    rcl_interfaces::msg::ParameterDescriptor collision_setting;
    collision_setting.read_only=true;
    collision_setting.description="Optional model/table check; select before launch and restart to change.";
    const bool collision_enabled=declare_parameter("collision_enabled",false,collision_setting);
    if (collision_enabled) {
      CollisionOptions options;
      options.speed=speed; options.accel=accel;
      options.table_z=declare_parameter("table_z_m",0.0);
      options.table_size=declare_parameter("table_size_m",2.0);
      options.margin=declare_parameter("collision_margin_m",.005);
      options.slow_distance=declare_parameter("collision_slow_distance_m",.020);
      options.period=declare_parameter("collision_check_period_ms",20.0)*.001;
      options.timeout=declare_parameter("collision_result_timeout_s",.15);
      collision_model_=std::make_shared<CollisionModel>(declare_parameter("robot_description",std::string("")),
        declare_parameter("robot_description_semantic",std::string("")),options);
      collision_guard_=std::make_unique<CollisionGuard>(collision_model_,options);
    }
    client_ = mock_ ? std::unique_ptr<Client>(new MockClient) : std::unique_ptr<Client>(new NativeClient);
    if (mock_ && collision_model_) {
      // Consistent mock joint/flange feedback exercises the real model and IK.
      const auto values=declare_parameter("mock_joints_rad",std::vector<double>{2.1766,-1.6214,1.5477,-1.5877,-1.355685307179586,0});
      if (values.size()!=6) throw std::invalid_argument("mock_joints_rad needs six values");
      Joints initial; std::copy(values.begin(),values.end(),initial.begin());
      static_cast<MockClient *>(client_.get())->use_model(collision_model_->flange(initial),initial,
        [model=collision_model_](const Pose & pose,Joints & q) { return model->inverse(pose,q); });
    }
    session_ = std::make_unique<ServoSession>(*client_);
    flange_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_follow/flange_pose",20);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_follow/desired_tcp_pose",10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/target_follow/follow_state",10);
    servo_pub_ = create_publisher<std_msgs::msg::String>("/target_follow/servo_state",10);
    feedback_pub_ = create_publisher<std_msgs::msg::String>("/target_follow/robot_feedback",20);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/target_follow/tag_pose_base",1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { accept_target(*message); });
    using Request = std::shared_ptr<std_srvs::srv::Trigger::Request>;
    using Response = std::shared_ptr<std_srvs::srv::Trigger::Response>;
    teach_service_ = create_service<std_srvs::srv::Trigger>("/target_follow/teach_offset",
      [this](Request, Response response) {
        try {
          if (following_) throw std::runtime_error("stop following before recording an offset");
          require_data(); core_->teach(target_,actual_);
          response->success=true; response->message="Current offset recorded; motion remains stopped.";
        } catch (const std::exception & e) { response->success=false; response->message=e.what(); }
      });
    start_service_ = create_service<std_srvs::srv::Trigger>("/target_follow/follow_start",
      [this](Request, Response response) {
        try {
          if (following_) throw std::runtime_error("following is already started");
          if (fault_ || stop_requested_) throw std::runtime_error("driver is stopping or faulted; inspect follow_state");
          require_ready();
          if (collision_guard_) {
            const auto check=collision_guard_->result(steady(),Eigen::Vector3d::Zero());
            if (!check.ready || check.colliding || check.scale<=0)
              throw std::runtime_error("cannot start: "+check.reason);
          }
          // Always teach again, including after an explicit stop. No old session offset is reused.
          core_->teach(target_,actual_);
          const int code=session_->start();
          log({{"kind","start"},{"sdk_return_code",code},{"offset_m",xyz(core_->offset())}});
          if (code) { fail("ServoMoveStart failed: "+std::to_string(code)); throw std::runtime_error(reason_); }
          following_=true; status_="following"; reason_.clear(); ++starts_;
          response->success=true; response->message="Following started with a newly recorded offset and fixed rotation.";
        } catch (const std::exception & e) { response->success=false; response->message=e.what(); }
      });
    stop_service_ = create_service<std_srvs::srv::Trigger>("/target_follow/servo_stop",
      [this](Request, Response response) {
        stop_requested_=true; response->success=true;
        response->message="Stop requested; follow_state records the SDK stop result.";
      });
    report_={{"mock",mock_},{"backend","ServoCart mode 0, tool/work-object 0"},
      {"kp",kp},{"speed_limit_mm_s",speed*1000},{"accel_limit_mm_s2",accel*1000},
      {"follow_radius_m",range},{"target_filter_s",filter},{"requested_period_ms",period_*1000},
      {"hardware_servo_commands_sent",0},{"live_follow_validated",false}};
    report_["collision_enabled"]=collision_enabled;
    report_["tracking_mode"]=collision_enabled?"collision_guarded":"legacy";
    if (collision_enabled) report_["collision_scene"]={{"table_z_m",get_parameter("table_z_m").as_double()},
      {"table_size_m",get_parameter("table_size_m").as_double()},
      {"collision_margin_m",get_parameter("collision_margin_m").as_double()},
      {"finger_aperture_envelope_m",Json::array({0.,.060})}};
    if (!output_.empty()) {
      std::filesystem::create_directories(output_); samples_.open(output_+"/samples.jsonl");
      if (!samples_) throw std::runtime_error("cannot open samples.jsonl");
    }
  }

  int run() {
    int result=0;
    try {
      int code=client_->connect(ip_.c_str());
      if (code) throw std::runtime_error("SDK connection failed: "+std::to_string(code));
      connected_=true;
      RCLCPP_INFO(get_logger(),"Position follow ready to receive data; motion starts only through follow_start. mock=%s",mock_?"true":"false");
      RCLCPP_INFO(get_logger(),"Tracking mode: %s; model collision check: %s. Change through launch/config and restart.",
        collision_guard_?"collision_guarded":"legacy",collision_guard_?"ON":"OFF");
      double last_send=0, last_state=0;
      while (rclcpp::ok() && !interrupted) {
        const double began=steady();
        read_feedback();
        rclcpp::spin_some(shared_from_this());  // Callbacks and SDK use the same sole owner thread.
        update_collision();
        if (stop_requested_) {
          following_=false; finish(); core_->clear();
          if (!fault_) { status_="stopped"; reason_="user_stop"; }
          stop_requested_=false;
        }
        if (following_) {
          if (!fresh_feedback()) fail("robot feedback expired");
          else {
            const auto problem=state_problem(packet_,false);
            if (!problem.empty()) fail(problem);
            else if (!fresh_target()) {
              if (session_->active()) { finish(); ++pauses_; core_->reanchor(actual_); }
              if (!fault_) { status_="paused_target"; reason_="stale_or_missing_target"; }
            } else if (!fault_ && collision_guard_ && collision_.scale<=0) {
              if (session_->active()) { finish(); ++collision_pauses_; }
              core_->reanchor(actual_); last_send=0;
              if (!fault_) { status_="paused_collision"; reason_=collision_.reason; }
            } else if (!fault_) {
              if (!session_->active()) {
                const auto resume_problem=state_problem(packet_,true);
                // StopMotion can return before the next packet reports idle.
                // The outer controller checks already reject real faults.
                if (!resume_problem.empty()) reason_="waiting_for_robot_to_stop";
                else if (!core_->in_range(target_)) fail("target is outside the follow range");
                else {
                  core_->observe(target_,period_,true); core_->reanchor(actual_);
                  const int start_code=session_->start();
                  if (start_code) fail("ServoMoveStart on resume failed: "+std::to_string(start_code));
                  else {
                    if (status_=="paused_collision") ++collision_resumes_; else ++resumes_;
                    status_="following"; reason_.clear(); last_send=0;
                  }
                }
              }
              if (session_->active() && !fault_) {
                // Slow SDK calls slow reference progress, without catching up missed periods.
                const double dt=last_send==0 ? period_ : std::min(period_,began-last_send);
                Pose command=core_->step(actual_,dt,collision_guard_?collision_.scale:1.);
                status_="following";
                reason_=collision_guard_ && collision_.scale<1 ? collision_.reason : "";
                const double send_time=steady();
                const int send_code=session_->send(command,period_);
                const double call_ms=(steady()-send_time)*1000;
                if (last_send>0) interval_stats_.push_back((send_time-last_send)*1000);
                last_send=send_time; call_stats_.push_back(call_ms); ++sent_;
                log({{"kind","servo"},{"stamp_s",now().seconds()},{"target_m_rad",command},
                     {"actual_m_rad",actual_},{"actual_tcp_m",xyz(core_->tcp(actual_))},
                     {"desired_tcp_m",xyz(core_->goal())},{"velocity_m_s",xyz(core_->velocity())},
                     {"target_age_ms",(now().seconds()-target_stamp_)*1000},
                     {"collision_scale",collision_guard_?collision_.scale:1.},
                     {"call_ms",call_ms},{"sdk_return_code",send_code}});
                if (send_code) fail("ServoCart failed: "+std::to_string(send_code));
                else if (call_ms>feedback_timeout_*1000) fail("ServoCart call exceeded feedback timeout");
              }
            }
          }
        } else last_send=0;
        if (steady()-last_state>=.05) { publish_state(); last_state=steady(); }
        std::this_thread::sleep_until(Clock::time_point(std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(began+period_))));
      }
      following_=false; finish(); core_->clear();
      if (!fault_) { status_="stopped"; reason_="signal"; }
    } catch (const std::exception & e) { fail(e.what()); }
    following_=false; finish();
    if (connected_) {
      const int close_code=client_->close(); report_["close_code"]=close_code; connected_=false;
      if (close_code) { fault_=true; status_="fault"; reason_="SDK close failed: "+std::to_string(close_code); }
    }
    report_.update(Json{{"result",status_},{"reason",reason_},{"servo_commands_sent",sent_},
      {"hardware_servo_commands_sent",mock_?0:sent_},{"starts",starts_},{"target_pauses",pauses_},
      {"target_resumes",resumes_},{"servo_call_ms",stats(call_stats_)},{"servo_interval_ms",stats(interval_stats_)}});
    report_["collision_pauses"]=collision_pauses_;
    report_["collision_resumes"]=collision_resumes_;
    report_["collision_check_ms"]=stats(collision_stats_);
    publish_state();
    if (!output_.empty()) {
      std::ofstream report(output_+"/report.json"); report << report_.dump(2) << '\n';
      if (!report) result=1;
    }
    std::cout << report_.dump(2) << '\n';
    return fault_?1:result;
  }
 private:
  static Json stats(std::vector<double> v) {
    if (v.empty()) return nullptr;
    std::sort(v.begin(),v.end());
    return {{"count",v.size()},{"p50",v[(v.size()-1)/2]},
            {"p95",v[static_cast<std::size_t>((v.size()-1)*.95)]},{"max",v.back()}};
  }
  void log(const Json & sample) { if (samples_) samples_ << sample.dump() << '\n'; }
  void update_collision() {
    if (!collision_guard_ || !fresh_feedback()) return;
    CollisionInput input; input.joints=joints_; input.flange=actual_; input.stamp=steady();
    if (following_ && core_->taught() && fresh_target()) {
      // Check the full configured speed in the requested direction, independent
      // of the current slowdown, so reduced speeds never hide an unsafe target.
      const auto requested=core_->requested(actual_);
      if (requested.norm()>1e-6) input.requested=requested.normalized()*speed_limit_;
      input.velocity=core_->velocity(); input.lead=core_->reference()-core_->tcp(actual_);
    }
    collision_guard_->submit(input);
    collision_=collision_guard_->result(steady(),input.requested);
    if (collision_.ready && collision_.stamp!=last_collision_stamp_) {
      collision_stats_.push_back(collision_.check_ms); last_collision_stamp_=collision_.stamp;
    }
  }
  bool fresh_feedback() const { return have_packet_ && freshness_.age(steady())<=feedback_timeout_; }
  bool fresh_target() const {
    const double age=now().seconds()-target_stamp_;
    return have_target_ && age>=-.05 && age<=target_timeout_;
  }
  void require_data() {
    if (!connected_ || !fresh_feedback()) throw std::runtime_error("fresh robot feedback is required");
    if (!fresh_target()) throw std::runtime_error("fresh base-frame target is required");
  }
  void require_ready() {
    require_data();
    if (get_publishers_info_by_topic("/target_follow/flange_pose").size()>1)
      throw std::runtime_error("another follow SDK feedback owner is running");
    const auto problem=state_problem(packet_,true);
    if (!problem.empty()) throw std::runtime_error(problem);
  }
  void accept_target(const geometry_msgs::msg::PoseStamped & message) {
    double stamp=rclcpp::Time(message.header.stamp).seconds();
    const auto & p=message.pose.position;
    Eigen::Vector3d target(p.x,p.y,p.z);
    double age=now().seconds()-stamp;
    if (message.header.frame_id!="base_link" || !target.allFinite() ||
        stamp<=target_stamp_ || age<-.05 || age>target_timeout_) return;
    const double dt=have_target_ ? stamp-target_stamp_ : period_;
    const bool reset=!fresh_target();
    target_=target; target_stamp_=stamp; have_target_=true;
    if (core_->taught()) {
      try { core_->observe(target,dt,reset); }
      catch (const std::exception & e) {
        if (following_) fail(e.what()); else core_->clear();
      }
    }
  }
  geometry_msgs::msg::PoseStamped pose_message(const Eigen::Vector3d & p, const Eigen::Matrix3d & r) {
    geometry_msgs::msg::PoseStamped out; out.header.frame_id="base_link"; out.header.stamp=now();
    out.pose.position.x=p.x(); out.pose.position.y=p.y(); out.pose.position.z=p.z();
    Eigen::Quaterniond q(r); out.pose.orientation.x=q.x(); out.pose.orientation.y=q.y();
    out.pose.orientation.z=q.z(); out.pose.orientation.w=q.w(); return out;
  }
  void read_feedback() {
    ROBOT_STATE_PKG latest{};
    const int code=client_->read(&latest);
    if (code || latest.frame_head!=0x5A5A) {
      if (following_) fail("SDK feedback read failed: "+std::to_string(code));
      return;
    }
    fr3c_follow_feedback::PacketFreshness::Key key{latest.frame_cnt,latest.robotTime.year,
      latest.robotTime.mouth,latest.robotTime.day,latest.robotTime.hour,latest.robotTime.minute,
      latest.robotTime.second,latest.robotTime.millisecond};
    if (!freshness_.observe(key,steady())) return;
    packet_=latest; actual_=from_sdk(packet_.flange_cur_pos); have_packet_=true;
    for (unsigned i=0;i<6;++i) joints_[i]=packet_.jt_cur_pos[i]*3.14159265358979323846/180;
    auto pose=pose_message(position(actual_),rotation(actual_)); flange_pub_->publish(pose);
    geometry_msgs::msg::TransformStamped tf; tf.header=pose.header; tf.child_frame_id="flange_link";
    tf.transform.translation.x=actual_[0]; tf.transform.translation.y=actual_[1]; tf.transform.translation.z=actual_[2];
    tf.transform.rotation=pose.pose.orientation; tf_->sendTransform(tf);
    std_msgs::msg::String feedback;
    feedback.data=Json{{"status","valid"},{"mock",mock_},{"sdk_return_code",code},
      {"stamp_source","host_first_observation_of_new_sdk_packet"},{"feedback_age_s",freshness_.age(steady())},
      {"robot_mode",packet_.robot_mode},{"robot_state",packet_.robot_state},
      {"program_state",packet_.program_state},{"enabled",packet_.rbtEnableState},
      {"main_error_code",packet_.main_code},{"sub_error_code",packet_.sub_code},
      {"tool_id",packet_.tool},{"work_object_id",packet_.user},
      {"motion_commands_enabled",session_->active()&&!mock_}}.dump();
    feedback_pub_->publish(feedback);
    log({{"kind","feedback"},{"stamp_s",rclcpp::Time(pose.header.stamp).seconds()},
      {"flange_m_rad",actual_},{"joints_rad",joints_}});
  }
  void finish() {
    if (!session_->active()) return;
    const auto result=session_->finish();
    report_["stop_motion_code"]=result.stop_code; report_["servo_end_code"]=result.end_code;
    log({{"kind","stop"},{"stop_motion_code",result.stop_code},{"servo_end_code",result.end_code}});
    if (result.stop_code || result.end_code) {
      fault_=true; following_=false; status_="fault"; reason_="SDK stop/end failed; inspect the controller";
    }
  }
  void fail(const std::string & reason) {
    following_=false; fault_=true; status_="fault"; reason_=reason;
    finish(); core_->clear(); RCLCPP_ERROR(get_logger(),"%s",reason_.c_str());
  }
  void publish_state() {
    Json state{{"status",status_},{"reason",reason_},{"mock",mock_},
      {"stamp_s",now().seconds()},{"motion_commands_enabled",session_->active()&&!mock_},
      {"orientation_control_enabled",false},{"following_requested",following_},
      {"offset_recorded",core_->taught()},{"servo_commands_sent",sent_},
      {"target_pauses",pauses_},{"target_resumes",resumes_},{"starts",starts_},
      {"collision_pauses",collision_pauses_},{"collision_resumes",collision_resumes_},
      {"velocity_m_s",session_->active()?xyz(core_->velocity()):Json::array({0.,0.,0.})}};
    state["tracking_mode"]=collision_guard_?"collision_guarded":"legacy";
    state["collision"]={{"enabled",false},{"ready",false},{"speed_scale",1.0},{"reason","disabled"}};
    if (collision_guard_) state["collision"]={{"enabled",true},{"ready",collision_.ready},
      {"speed_scale",collision_.scale},{"distance_mm",collision_.distance*1000},
      {"predicted_distance_mm",collision_.predicted_distance*1000},{"pair",collision_.pair},
      {"reason",collision_.reason},{"check_ms",collision_.check_ms},
      {"age_ms",collision_.ready?(steady()-collision_.stamp)*1000:0.}};
    if (have_target_) { state["target_base_m"]=xyz(target_); state["target_age_ms"]=(now().seconds()-target_stamp_)*1000; }
    if (have_packet_) { state["actual_tcp_m"]=xyz(core_->tcp(actual_)); state["feedback_age_ms"]=freshness_.age(steady())*1000; }
    if (core_->taught()) {
      state["offset_m"]=xyz(core_->offset()); state["desired_tcp_m"]=xyz(core_->goal());
      if (have_packet_) {
        Eigen::Vector3d error=core_->goal()-core_->tcp(actual_);
        state["position_error_m"]=xyz(error); state["error_norm_mm"]=error.norm()*1000;
      }
      goal_pub_->publish(pose_message(core_->goal(),rotation(core_->initial())*tcp_rotation_));
    }
    for (const char * key : {"stop_motion_code","servo_end_code"}) if (report_.contains(key)) state[key]=report_[key];
    std_msgs::msg::String message; message.data=state.dump(); state_pub_->publish(message); servo_pub_->publish(message);
    log({{"kind","state"},{"state",state}});
  }
  bool mock_=false, connected_=false, have_packet_=false, have_target_=false;
  bool following_=false, stop_requested_=false, fault_=false;
  unsigned sent_=0, starts_=0, pauses_=0, resumes_=0;
  unsigned collision_pauses_=0, collision_resumes_=0;
  double last_collision_stamp_=0;
  double period_, target_timeout_, feedback_timeout_, speed_limit_, target_stamp_=0;
  std::string ip_,output_,status_="idle",reason_;
  Eigen::Matrix3d tcp_rotation_;
  Eigen::Vector3d target_=Eigen::Vector3d::Zero();
  Pose actual_{};
  Joints joints_{};
  CollisionOutput collision_;
  std::shared_ptr<CollisionModel> collision_model_;
  std::unique_ptr<CollisionGuard> collision_guard_;
  ROBOT_STATE_PKG packet_{};
  fr3c_follow_feedback::PacketFreshness freshness_;
  std::unique_ptr<Client> client_;
  std::unique_ptr<ServoSession> session_;
  std::unique_ptr<PositionFollow> core_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  Json report_;
  std::ofstream samples_;
  std::vector<double> call_stats_,interval_stats_;
  std::vector<double> collision_stats_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr flange_pub_,goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_,servo_pub_,feedback_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr teach_service_,start_service_,stop_service_;
};

int main(int argc, char ** argv) {
  try {
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,interrupt_handler); std::signal(SIGTERM,interrupt_handler);
    int result=std::make_shared<FollowDriver>()->run(); rclcpp::shutdown(); return result;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; if (rclcpp::ok()) rclcpp::shutdown(); return 1;
  }
}
