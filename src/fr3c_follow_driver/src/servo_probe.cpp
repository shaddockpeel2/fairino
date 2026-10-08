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
#include "fr3c_follow_driver/servo_core.hpp"

using namespace fr3c_follow_driver;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using fr3c_follow_feedback::PacketFreshness;
static volatile std::sig_atomic_t interrupted = 0;
static void interrupt_handler(int) { interrupted = 1; }
static double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

struct Options {
  bool move=false, mock=false;
  std::string ip="192.168.58.2", axis="x", output;
  double distance_mm=5, speed_mm_s=5, accel_mm_s2=30, period_ms=8, hold_s=.5, stop_after_s=0;
};
static Options parse(int argc, char ** argv) {
  Options o;
  for (int i=1; i<argc; ++i) {
    std::string arg=argv[i];
    if (arg=="--move") o.move=true;
    else if (arg=="--inspect") o.move=false;
    else if (arg=="--mock") o.mock=true;
    else if (arg=="--help") {
      std::cout << "servo_probe [--inspect | --move] [--mock] [--axis x|y|z] "
                   "[--distance-mm 5] [--speed-mm-s 5] [--period-ms 8] "
                   "[--accel-mm-s2 30] [--hold-s .5] [--stop-after-s .5] "
                   "[--ip 192.168.58.2] [--output-dir PATH]\n"
                   "Default: read-only inspection. --move sends one fixed-orientation translation.\n";
      std::exit(0);
    } else {
      if (++i>=argc) throw std::invalid_argument("missing value for "+arg);
      std::string value=argv[i];
      if (arg=="--axis") o.axis=value;
      else if (arg=="--ip") o.ip=value;
      else if (arg=="--output-dir") o.output=value;
      else {
        std::size_t used=0; double v=std::stod(value, &used);
        if (used!=value.size() || !std::isfinite(v)) throw std::invalid_argument("invalid value for "+arg);
        if (arg=="--distance-mm") o.distance_mm=v;
        else if (arg=="--speed-mm-s") o.speed_mm_s=v;
        else if (arg=="--accel-mm-s2") o.accel_mm_s2=v;
        else if (arg=="--period-ms") o.period_ms=v;
        else if (arg=="--hold-s") o.hold_s=v;
        else if (arg=="--stop-after-s") o.stop_after_s=v;
        else throw std::invalid_argument("unknown option: "+arg);
      }
    }
  }
  if (o.axis!="x" && o.axis!="y" && o.axis!="z") throw std::invalid_argument("axis must be x, y or z");
  Translation(o.axis=="x"?0:o.axis=="y"?1:2, o.distance_mm*.001, o.speed_mm_s*.001, o.accel_mm_s2*.001);
  if (o.period_ms<1 || o.period_ms>16 || o.hold_s<0 || o.hold_s>2 || o.stop_after_s<0)
    throw std::invalid_argument("period: 1..16 ms; hold: 0..2 s; stop-after: >=0 s");
  return o;
}

static Json statistics(std::vector<double> values) {
  if (values.empty()) return nullptr;
  std::sort(values.begin(), values.end());
  return Json{{"count",values.size()}, {"p50",values[(values.size()-1)/2]},
              {"p95",values[static_cast<std::size_t>((values.size()-1)*.95)]}, {"max",values.back()}};
}

class Probe final : public rclcpp::Node {
 public:
  explicit Probe(Options options) : Node("follow_servo_probe"), options_(std::move(options)) {
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
    report_={{"mock",options_.mock}, {"action",options_.move?"move":"inspect"},
      {"backend","ServoCart mode 0, tool/work-object 0"}, {"axis",options_.axis},
      {"distance_mm",options_.distance_mm}, {"speed_limit_mm_s",options_.speed_mm_s},
      {"accel_limit_mm_s2",options_.accel_mm_s2}, {"requested_period_ms",options_.period_ms},
      {"hardware_servo_commands_sent",0}, {"motion_response_validated",false}};
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
      const int code=client_->connect(options_.ip.c_str());
      if (code) throw std::runtime_error("SDK connection failed: "+std::to_string(code));
      connected_=true;
      const double wait_end=seconds()+3;
      while (!read_feedback() && seconds()<wait_end && !interrupted)
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
      if (!have_packet_ || interrupted) throw std::runtime_error("no fresh initial feedback, or interrupted");
      Pose initial=from_sdk(packet_.flange_cur_pos);
      report_["initial_flange_m_rad"]=initial;
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
          throw std::runtime_error("another follow feedback owner is running; close the read-only feedback before this probe");
        if (interrupted || stop_requested_) throw std::runtime_error("stopped before starting servo");
        read_feedback();
        if (freshness_.age(seconds())>.20) throw std::runtime_error("initial feedback expired");
        initial=from_sdk(packet_.flange_cur_pos);
        report_["initial_flange_m_rad"]=initial;
        report_["controller_state"]=controller_state();
        const std::string problem=state_problem(packet_,true);
        if (!problem.empty()) throw std::runtime_error(problem);
        const int start_code=session.start(); report_["servo_start_code"]=start_code;
        if (start_code) throw std::runtime_error("ServoMoveStart failed: "+std::to_string(start_code));
        active_=true;
        Translation plan(axis_index(),options_.distance_mm*.001,options_.speed_mm_s*.001,options_.accel_mm_s2*.001);
        report_["trajectory_duration_s"]=plan.duration();
        const double period=options_.period_ms*.001, began=seconds();
        double progress=0, previous_send=0, due=began;
        std::string reason="completed";
        while (progress < plan.duration()+options_.hold_s) {
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
          const Pose target=plan.at(initial,std::min(progress+period,plan.duration()));
          const auto actual=from_sdk(packet_.flange_cur_pos);
          double tracking_error=0;
          for (int i=0;i<3;++i) tracking_error+=std::pow(target[i]-actual[i],2);
          if (std::sqrt(tracking_error)>.015) throw std::runtime_error("servo reference is over 15 mm ahead of feedback");
          const double send_start=seconds();
          const int send_code=session.send(target,period);
          const double call_ms=(seconds()-send_start)*1000;
          call_times_.push_back(call_ms);
          if (previous_send>0) intervals_.push_back((send_start-previous_send)*1000);
          previous_send=send_start;
          ++sent_;
          report_["hardware_servo_commands_sent"]=options_.mock ? 0 : sent_;
          log_sample({{"kind","servo"},{"command_time_s",progress+period}, {"send_steady_s",send_start},
                      {"target_m_rad",target},{"actual_m_rad",actual},{"call_ms",call_ms},{"sdk_return_code",send_code}});
          if (send_code) throw std::runtime_error("ServoCart failed: "+std::to_string(send_code));
          if (call_ms>200) throw std::runtime_error("ServoCart call exceeded 200 ms; stopping the probe");
          progress+=period;  // No catch-up burst or jump when the SDK call takes longer.
          due=std::max(due+period,send_start+period);
          publish_state(progress<plan.duration()?"moving":"holding");
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
        std::array<double,3> delta{};
        for(int i=0;i<3;++i) delta[i]=(final[i]-initial[i])*1000;
        report_["measured_delta_mm"]=delta;
        double goal_error=0;
        const auto goal=plan.at(initial,plan.duration());
        for(int i=0;i<3;++i) goal_error+=std::pow(final[i]-goal[i],2);
        report_["measured_goal_error_mm"]=std::sqrt(goal_error)*1000;
        auto rotation=[](const Pose & p) {
          return Eigen::Quaterniond(Eigen::AngleAxisd(p[5],Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p[4],Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(p[3],Eigen::Vector3d::UnitX()));
        };
        const double rotation_deg=rotation(initial).angularDistance(rotation(final))*180/kPi;
        report_["measured_rotation_delta_deg"]=rotation_deg;
        report_["final_robot_state"]=packet_.robot_state;
        report_["final_feedback_age_s"]=freshness_.age(seconds());
        report_["stop_response_observed"]=!options_.mock && packet_.robot_state==1 && freshness_.age(seconds())<=.20;
        report_["motion_response_validated"]=!options_.mock && reason=="completed" &&
          std::sqrt(goal_error)*1000<2 && rotation_deg<1 && packet_.robot_state==1 && freshness_.age(seconds())<=.20;
      }
    } catch (const std::exception & error) {
      report_["result"]="fault"; report_["error"]=error.what();
      std::cerr << error.what() << '\n'; result=1;
    }
    finish(session);
    if (connected_) { report_["close_code"]=client_->close(); connected_=false; }
    if (report_.contains("stop_motion_code") &&
        (report_["stop_motion_code"]!=0 || report_["servo_end_code"]!=0)) result=1;
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
  int axis_index() const { return options_.axis=="x"?0:options_.axis=="y"?1:2; }
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
                {"flange_m_rad",from_sdk(packet_.flange_cur_pos)},{"state",status}});
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
                     {"servo_commands_sent",sent_},{"axis",options_.axis},{"distance_mm",options_.distance_mm}}.dump();
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
    int result=std::make_shared<Probe>(options)->run();
    rclcpp::shutdown(); return result;
  } catch(const std::exception & error) {
    std::cerr << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
