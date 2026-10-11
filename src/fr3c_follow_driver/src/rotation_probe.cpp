#include <algorithm>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "fr3c_follow_driver/clients.hpp"
#include "fr3c_follow_feedback/helpers.hpp"
#include "fr3c_follow_driver/rotation_probe.hpp"

using namespace fr3c_follow_driver;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using fr3c_follow_feedback::PacketFreshness;
static volatile std::sig_atomic_t interrupted = 0;
static void interrupt_handler(int) { interrupted = 1; }
static double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

struct Options {
  bool move=false, mock=false, round_trip=true;
  std::string ip="192.168.58.2", axis="x", output;
  Pose flange_tcp{};
  bool calibrated=false;
  double angle_deg=2.5, speed_deg_s=3, accel_deg_s2=10, period_ms=8, hold_s=.5, stop_after_s=0;
};
static Eigen::Vector3d axis_vector(const std::string & axis) {
  if (axis=="x") return Eigen::Vector3d::UnitX();
  if (axis=="y") return Eigen::Vector3d::UnitY();
  if (axis=="z") return Eigen::Vector3d::UnitZ();
  if (axis=="xyz") return Eigen::Vector3d::Ones().normalized();
  throw std::invalid_argument("axis must be x, y, z or xyz (diagonal base-frame axis)");
}
static Options parse(int argc, char ** argv) {
  Options o;
  for (int i=1; i<argc; ++i) {
    std::string arg=argv[i];
    if (arg=="--move") o.move=true;
    else if (arg=="--inspect") o.move=false;
    else if (arg=="--mock") o.mock=true;
    else if (arg=="--one-way") o.round_trip=false;
    else if (arg=="--round-trip") o.round_trip=true;
    else if (arg=="--tcp-m-rad") {
      if (i+6>=argc) throw std::invalid_argument("--tcp-m-rad needs x y z roll pitch yaw");
      for (double & value : o.flange_tcp) {
        std::string token=argv[++i]; std::size_t used=0; value=std::stod(token,&used);
        if (used!=token.size() || !std::isfinite(value)) throw std::invalid_argument("invalid TCP component");
      }
      o.calibrated=true;
    } else if (arg=="--help") {
      std::cout << "rotation_probe [--inspect | --move] [--mock] [--axis x|y|z|xyz] "
                   "[--angle-deg 2.5] [--speed-deg-s 3] [--accel-deg-s2 10] "
                   "[--round-trip | --one-way] [--period-ms 8] [--hold-s .5] "
                   "[--stop-after-s .5] [--ip 192.168.58.2] [--output-dir PATH] "
                   "--tcp-m-rad X Y Z R P Y\n"
                   "Use rotation_demo.sh to load the calibrated TCP. Default: inspect only.\n";
      std::exit(0);
    } else {
      if (++i>=argc) throw std::invalid_argument("missing value for "+arg);
      std::string value=argv[i];
      if (arg=="--axis") o.axis=value;
      else if (arg=="--ip") o.ip=value;
      else if (arg=="--output-dir") o.output=value;
      else {
        std::size_t used=0; double v=std::stod(value,&used);
        if (used!=value.size() || !std::isfinite(v)) throw std::invalid_argument("invalid value for "+arg);
        if (arg=="--angle-deg") o.angle_deg=v;
        else if (arg=="--speed-deg-s") o.speed_deg_s=v;
        else if (arg=="--accel-deg-s2") o.accel_deg_s2=v;
        else if (arg=="--period-ms") o.period_ms=v;
        else if (arg=="--hold-s") o.hold_s=v;
        else if (arg=="--stop-after-s") o.stop_after_s=v;
        else throw std::invalid_argument("unknown option: "+arg);
      }
    }
  }
  if (!o.calibrated) throw std::invalid_argument("calibrated --tcp-m-rad is required; use rotation_demo.sh");
  TcpRotation(o.flange_tcp,axis_vector(o.axis),o.angle_deg*kPi/180,o.speed_deg_s*kPi/180,
              o.accel_deg_s2*kPi/180,o.hold_s,o.round_trip);
  if (o.period_ms<1 || o.period_ms>16 || o.stop_after_s<0)
    throw std::invalid_argument("period: 1..16 ms; stop-after: >=0 s");
  return o;
}
static Json rigid_json(const Eigen::Isometry3d & t) {
  Eigen::Quaterniond q(t.linear());
  return Json{{"position_m",{t.translation().x(),t.translation().y(),t.translation().z()}},
              {"quaternion_xyzw",{q.x(),q.y(),q.z(),q.w()}}};
}
static Json sdk_absolute_json(const Pose & pose) {
  const auto sdk=to_sdk_absolute(pose);
  return Json::array({sdk.tran.x,sdk.tran.y,sdk.tran.z,sdk.rpy.rx,sdk.rpy.ry,sdk.rpy.rz});
}
static std::array<double,6> joints(const ROBOT_STATE_PKG & packet) {
  std::array<double,6> out{};
  for (int i=0;i<6;++i) {
    if (!std::isfinite(packet.jt_cur_pos[i])) throw std::runtime_error("nonfinite joint feedback");
    out[i]=packet.jt_cur_pos[i]*kPi/180;
  }
  return out;
}

static Json statistics(std::vector<double> values) {
  if (values.empty()) return nullptr;
  std::sort(values.begin(), values.end());
  return Json{{"count",values.size()}, {"p50",values[(values.size()-1)/2]},
              {"p95",values[static_cast<std::size_t>((values.size()-1)*.95)]}, {"max",values.back()}};
}

class RotationProbe final : public rclcpp::Node {
 public:
  explicit RotationProbe(Options options) : Node("follow_rotation_probe"), options_(std::move(options)) {
    client_=options_.mock ? std::unique_ptr<Client>(new MockClient) : std::unique_ptr<Client>(new NativeClient);
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/target_follow/flange_pose",20);
    feedback_pub_=create_publisher<std_msgs::msg::String>("/target_follow/robot_feedback",20);
    state_pub_=create_publisher<std_msgs::msg::String>("/target_follow/servo_state",10);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    stop_service_=create_service<std_srvs::srv::Trigger>("/target_follow/servo_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        stop_requested_=true; response->success=true;
        response->message="Stop requested; see servo_state/report for SDK stop result.";
      });
    report_={{"stage",8},{"mock",options_.mock},{"action",options_.move?"rotation":"inspect"},
      {"backend","ServoCart mode 0, tool/work-object 0"},{"axis_frame","base_link"},{"axis",options_.axis},
      {"angle_deg",options_.angle_deg},{"round_trip",options_.round_trip},
      {"speed_limit_deg_s",options_.speed_deg_s},{"accel_limit_deg_s2",options_.accel_deg_s2},
      {"requested_period_ms",options_.period_ms},{"flange_tcp_m_rad",options_.flange_tcp},
      {"sdk_angle_encoding","absolute RPY wrapped to [-180,180] deg; internal neighboring RPY preserved"},
      {"hardware_servo_commands_sent",0},{"live_rotation_validated",false},
      {"mock_ik_is_physical",false},{"model_collision_enabled",false}};
    if (!options_.output.empty()) {
      std::filesystem::create_directories(options_.output);
      samples_.open(options_.output+"/samples.jsonl");
      if (!samples_) throw std::runtime_error("cannot create samples.jsonl");
    }
  }
  int run() {
    int result=0;
    ServoSession session(*client_);
    try {
      connected_=true;  // RPC may start SDK threads even on a failed connection.
      const int code=client_->connect(options_.ip.c_str());
      if (code) throw std::runtime_error("SDK connection failed: "+std::to_string(code));
      const double wait_end=seconds()+3;
      while (!read_feedback() && seconds()<wait_end && !interrupted)
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
      if (!have_packet_ || interrupted) throw std::runtime_error("no fresh initial feedback, or interrupted");
      Pose initial=from_sdk(packet_.flange_cur_pos);
      report_["initial_flange_m_rad"]=initial;
      report_["initial_tcp"]=rigid_json(pose_transform(initial)*pose_transform(options_.flange_tcp));
      report_["controller_state"]=controller_state();
      report_["motion_readiness_problem"]=state_problem(packet_,true);
      if (!options_.move) {
        report_["result"]="inspected"; report_["motion_commands_enabled"]=false;
      } else {
        // DDS catches another local follow feedback owner; hardware state catches an active program.
        const double discovery_end=seconds()+.6;
        while (seconds()<discovery_end && !interrupted) {
          rclcpp::spin_some(shared_from_this()); read_feedback();
          std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }
        if (get_publishers_info_by_topic("/target_follow/flange_pose").size()>1)
          throw std::runtime_error("another follow feedback owner is running; close feedback/preview/follow before this probe");
        if (interrupted || stop_requested_) throw std::runtime_error("stopped before starting servo");
        read_feedback();
        if (freshness_.age(seconds())>.20) throw std::runtime_error("initial feedback expired");
        initial=from_sdk(packet_.flange_cur_pos);
        report_["initial_flange_m_rad"]=initial;
        report_["controller_state"]=controller_state();
        const std::string problem=state_problem(packet_,true);
        if (!problem.empty()) throw std::runtime_error(problem);
        TcpRotation plan(options_.flange_tcp,axis_vector(options_.axis),options_.angle_deg*kPi/180,
                         options_.speed_deg_s*kPi/180,options_.accel_deg_s2*kPi/180,
                         options_.hold_s,options_.round_trip);
        plan.anchor(initial);
        report_["initial_tcp"]=rigid_json(plan.initial_tcp());
        report_["initial_joints_rad"]=joints(packet_);
        preflight(plan,initial);
        read_feedback();
        if (freshness_.age(seconds())>.20) throw std::runtime_error("feedback expired during IK preflight");
        const auto current_tcp=plan.tcp(from_sdk(packet_.flange_cur_pos));
        if ((current_tcp.translation()-plan.initial_tcp().translation()).norm()>.003 ||
            rotation_distance(current_tcp.linear(),plan.initial_tcp().linear())>kPi/180)
          throw std::runtime_error("robot moved during preflight; run again from the new stationary pose");
        if (interrupted || stop_requested_) throw std::runtime_error("stopped before starting servo");
        const auto start_problem=state_problem(packet_,true);
        if (!start_problem.empty()) throw std::runtime_error(start_problem);
        initial=from_sdk(packet_.flange_cur_pos); plan.anchor(initial);
        report_["initial_flange_m_rad"]=initial;
        report_["initial_tcp"]=rigid_json(plan.initial_tcp());
        const int start_code=session.start(); report_["servo_start_code"]=start_code;
        if (start_code) throw std::runtime_error("ServoMoveStart failed: "+std::to_string(start_code));
        active_=true;
        report_["trajectory_duration_s"]=plan.duration();
        report_["leg_duration_s"]=plan.leg_duration();
        Pose previous_command=initial;
        auto previous_joints=joints(packet_);
        double max_tcp_drift=0, max_rotation=0, max_joint_step=0;
        const double period=options_.period_ms*.001, began=seconds();
        double progress=0, previous_send=0, due=began;
        std::string reason="completed";
        while (progress < plan.duration()) {
          rclcpp::spin_some(shared_from_this());
          if (interrupted || stop_requested_ || (options_.stop_after_s>0 && seconds()-began>=options_.stop_after_s)) {
            reason="stopped";
            report_["stop_reason"]=interrupted?"signal":stop_requested_?"service":"scheduled";
            break;
          }
          read_feedback();
          if (freshness_.age(seconds())>.20) throw std::runtime_error("robot feedback expired");
          const std::string problem_now=state_problem(packet_,false);
          if (!problem_now.empty()) throw std::runtime_error(problem_now);
          const double next_time=std::min(progress+period,plan.duration());
          const Pose target=plan.command(next_time,previous_command);
          const auto actual=from_sdk(packet_.flange_cur_pos);
          const auto actual_tcp=plan.tcp(actual), target_tcp=plan.reference_tcp(next_time);
          const double tcp_drift=(actual_tcp.translation()-plan.initial_tcp().translation()).norm();
          const double angular_lead=rotation_distance(actual_tcp.linear(),target_tcp.linear());
          max_tcp_drift=std::max(max_tcp_drift,tcp_drift);
          max_rotation=std::max(max_rotation,rotation_distance(actual_tcp.linear(),plan.initial_tcp().linear()));
          if (tcp_drift>.015) throw std::runtime_error("TCP deviated over 15 mm from its rotation center");
          if (angular_lead>5*kPi/180) throw std::runtime_error("rotation reference is over 5 deg ahead of feedback");
          const auto current_joints=joints(packet_);
          for (int i=0;i<6;++i) {
            const double step=std::abs(current_joints[i]-previous_joints[i]);
            max_joint_step=std::max(max_joint_step,step);
            if (step>15*kPi/180) throw std::runtime_error("joint feedback jumped over 15 deg; inspect local configuration");
          }
          previous_joints=current_joints;
          const double send_start=seconds();
          const int send_code=session.send(target,period);
          const double call_ms=(seconds()-send_start)*1000;
          call_times_.push_back(call_ms);
          if (previous_send>0) intervals_.push_back((send_start-previous_send)*1000);
          previous_send=send_start;
          ++sent_;
          report_["hardware_servo_commands_sent"]=options_.mock ? 0 : sent_;
          log_sample({{"kind","servo"},{"command_time_s",next_time},{"send_steady_s",send_start},
            {"phase",plan.phase(next_time)},{"planned_angle_deg",plan.angle_at(next_time)*180/kPi},
            {"reference_tcp",rigid_json(target_tcp)},{"actual_tcp",rigid_json(actual_tcp)},
            {"target_flange_m_rad",target},{"actual_flange_m_rad",actual},
            {"target_sdk_mm_deg",sdk_absolute_json(target)},
            {"joints_rad",current_joints},{"tcp_drift_mm",tcp_drift*1000},
            {"angular_lead_deg",angular_lead*180/kPi},{"call_ms",call_ms},{"sdk_return_code",send_code}});
          previous_command=target;
          if (send_code) throw std::runtime_error("ServoCart failed: "+std::to_string(send_code));
          if (call_ms>200) throw std::runtime_error("ServoCart call exceeded 200 ms; stopping the probe");
          progress+=period;  // No catch-up burst or jump when the SDK call takes longer.
          due=std::max(due+period,send_start+period);
          publish_state(plan.phase(progress));
          std::this_thread::sleep_until(Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(due))));
        }
        report_["result"]=reason;
        report_["elapsed_s"]=seconds()-began;
        finish(session);
        const double settle_end=seconds()+.4;
        while (seconds()<settle_end) {
          read_feedback(); rclcpp::spin_some(shared_from_this());
          std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }
        const Pose final=from_sdk(packet_.flange_cur_pos);
        report_["final_flange_m_rad"]=final;
        const auto final_tcp=plan.tcp(final), goal_tcp=plan.reference_tcp(plan.duration());
        const double final_drift=(final_tcp.translation()-plan.initial_tcp().translation()).norm();
        report_["final_tcp"]=rigid_json(final_tcp);
        report_["final_joints_rad"]=joints(packet_);
        report_["max_observed_tcp_drift_mm"]=std::max(max_tcp_drift,final_drift)*1000;
        report_["max_observed_rotation_deg"]=std::max(max_rotation,
          rotation_distance(final_tcp.linear(),plan.initial_tcp().linear()))*180/kPi;
        report_["max_joint_feedback_step_deg"]=max_joint_step*180/kPi;
        report_["final_tcp_drift_mm"]=final_drift*1000;
        report_["final_tcp_angle_to_start_deg"]=rotation_distance(final_tcp.linear(),plan.initial_tcp().linear())*180/kPi;
        report_["final_tcp_angle_to_goal_deg"]=rotation_distance(final_tcp.linear(),goal_tcp.linear())*180/kPi;
        report_["final_robot_state"]=packet_.robot_state;
        report_["final_feedback_age_s"]=freshness_.age(seconds());
        report_["stop_response_observed"]=!options_.mock && packet_.robot_state==1 && freshness_.age(seconds())<=.20;
        // Diagnostics support a Demo judgement; no precision score certifies live success.

      }
    } catch (const std::exception & error) {
      report_["result"]="fault"; report_["error"]=error.what();
      std::cerr << error.what() << '\n'; result=1;
    }
    finish(session);
    if (connected_) { report_["close_code"]=client_->close(); connected_=false; }
    if (report_.contains("stop_motion_code") &&
        (report_["stop_motion_code"]!=0 || report_["servo_end_code"]!=0)) {
      result=1; report_["result"]="fault";
      if (!report_.contains("error")) report_["error"]="StopMotion/ServoMoveEnd failed";
    }
    report_["servo_commands_sent"]=sent_;
    report_["servo_call_ms"]=statistics(call_times_);
    report_["servo_interval_ms"]=statistics(intervals_);
    publish_state(report_.value("result","fault"));
    std::cout << report_.dump(2) << '\n';
    if (!options_.output.empty()) {
      std::ofstream out(options_.output+"/report.json"); out << report_.dump(2) << '\n';
      if (!out) { std::cerr << "cannot write report.json\n"; result=1; }
    }
    return result;
  }
 private:
  void preflight(const TcpRotation & plan, const Pose & initial) {
    auto seed=joints(packet_); Pose previous=initial;
    for (int i=0;i<=10;++i) {
      rclcpp::spin_some(shared_from_this());
      if (interrupted || stop_requested_) throw std::runtime_error("stopped during IK preflight");
      const Pose target=plan.command(plan.leg_duration()*i/10,previous);
      std::array<double,6> solved{};
      const int code=client_->inverse(target,seed,solved);
      log_sample({{"kind","ik_preflight"},{"index",i},{"target_flange_m_rad",target},
                  {"target_sdk_mm_deg",sdk_absolute_json(target)},
                  {"reference_joints_rad",seed},{"solved_joints_rad",solved},{"sdk_return_code",code},
                  {"mock",options_.mock}});
      if (code) {
        report_["ik_preflight_failure"]={{"index",i},{"sdk_return_code",code},
          {"target_flange_m_rad",target},{"target_sdk_mm_deg",sdk_absolute_json(target)}};
        throw std::runtime_error("local IK preflight failed at sample "+std::to_string(i)+
          ": "+std::to_string(code)+(code==112?" (target pose cannot be reached)":"")+
          "; no servo motion was started");
      }
      for (int j=0;j<6;++j) {
        if (!std::isfinite(solved[j]) || std::abs(solved[j]-seed[j])>20*kPi/180)
          throw std::runtime_error("local IK has a nonfinite or >20 deg joint step");
      }
      seed=solved; previous=target;
    }
    report_["ik_preflight_passed"]=true;
  }
  void log_sample(const Json & value) { if (samples_) samples_ << value.dump() << '\n'; }
  Json controller_state() const {
    return Json{{"robot_mode",packet_.robot_mode},{"robot_state",packet_.robot_state},
      {"program_state",packet_.program_state},{"enabled",packet_.rbtEnableState},
      {"main_error_code",packet_.main_code},{"sub_error_code",packet_.sub_code},
      {"emergency_stop",packet_.EmergencyStop},{"tool_id",packet_.tool},{"work_object_id",packet_.user},
      {"tool_coord_mm_deg",std::vector<double>(packet_.toolCoord,packet_.toolCoord+6)},
      {"work_object_coord_mm_deg",std::vector<double>(packet_.wobjCoord,packet_.wobjCoord+6)}};
  }
  bool read_feedback() {
    ROBOT_STATE_PKG latest{};
    const double began=seconds();
    const int code=client_->read(&latest);
    if (seconds()-began>.20 && active_)
      throw std::runtime_error("feedback read exceeded 200 ms");
    if (code || latest.frame_head!=0x5A5A) {
      if (active_) throw std::runtime_error("SDK feedback read failed: "+std::to_string(code));
      return false;
    }
    PacketFreshness::Key key{latest.frame_cnt, latest.robotTime.year,latest.robotTime.mouth,
      latest.robotTime.day,latest.robotTime.hour,latest.robotTime.minute,latest.robotTime.second,latest.robotTime.millisecond};
    if (!freshness_.observe(key,seconds())) return false;
    packet_=latest; have_packet_=true;
    const auto transform=fr3c_follow_feedback::pose_mm_deg(packet_.flange_cur_pos);
    const Eigen::Quaterniond q(transform.rotation());
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp=now(); pose.header.frame_id="base_link";
    pose.pose.position.x=transform.translation().x(); pose.pose.position.y=transform.translation().y();
    pose.pose.position.z=transform.translation().z();
    pose.pose.orientation.x=q.x(); pose.pose.orientation.y=q.y(); pose.pose.orientation.z=q.z(); pose.pose.orientation.w=q.w();
    pose_pub_->publish(pose);
    geometry_msgs::msg::TransformStamped tf; tf.header=pose.header; tf.child_frame_id="flange_link";
    tf.transform.translation.x=pose.pose.position.x; tf.transform.translation.y=pose.pose.position.y;
    tf.transform.translation.z=pose.pose.position.z; tf.transform.rotation=pose.pose.orientation; tf_->sendTransform(tf);
    Json status=controller_state();
    status.update(Json{{"status","valid"},{"sdk_return_code",0},{"mock",options_.mock},
      {"feedback_age_s",freshness_.age(seconds())},{"call_ms",(seconds()-began)*1000},
      {"stamp_source","host_first_observation_of_new_sdk_packet"},
      {"motion_commands_enabled",active_&&!options_.mock},
      {"controller_tcp_mm_deg",std::vector<double>(packet_.tl_cur_pos,packet_.tl_cur_pos+6)}});
    std_msgs::msg::String message; message.data=status.dump(); feedback_pub_->publish(message);
    log_sample({{"kind","feedback"},{"stamp_s",rclcpp::Time(pose.header.stamp).seconds()},
                {"flange_m_rad",from_sdk(packet_.flange_cur_pos)},{"joints_rad",joints(packet_)},{"state",status}});
    return true;
  }
  void finish(ServoSession & session) {
    if (!session.active()) return;
    const double began=seconds(); const StopResult result=session.finish(); active_=false;
    report_["stop_motion_code"]=result.stop_code; report_["servo_end_code"]=result.end_code;
    report_["stop_calls_elapsed_ms"]=(seconds()-began)*1000;
    log_sample({{"kind","stop"},{"stop_motion_code",result.stop_code},{"servo_end_code",result.end_code}});
  }
  void publish_state(const std::string & state) {
    std_msgs::msg::String message;
    message.data=Json{{"status",state},{"mock",options_.mock},{"motion_commands_enabled",active_&&!options_.mock},
                     {"servo_commands_sent",sent_},{"stage",8},{"axis",options_.axis},{"angle_deg",options_.angle_deg},
                     {"round_trip",options_.round_trip},{"stop_motion_code",report_.value("stop_motion_code",-1)},
                     {"servo_end_code",report_.value("servo_end_code",-1)}}.dump();
    state_pub_->publish(message);
  }
  Options options_;
  std::unique_ptr<Client> client_;
  ROBOT_STATE_PKG packet_{};
  PacketFreshness freshness_;
  bool connected_=false, have_packet_=false, active_=false, stop_requested_=false;
  unsigned sent_=0;
  Json report_;
  std::ofstream samples_;
  std::vector<double> call_times_, intervals_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_,state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char ** argv) {
  try {
    Options options=parse(argc,argv);
    rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,interrupt_handler); std::signal(SIGTERM,interrupt_handler);
    int result=std::make_shared<RotationProbe>(options)->run();
    rclcpp::shutdown(); return result;
  } catch(const std::exception & error) {
    std::cerr << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
