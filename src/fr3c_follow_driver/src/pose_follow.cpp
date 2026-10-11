#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "fr3c_follow_driver/clients.hpp"
#include "fr3c_follow_driver/pose_follow.hpp"
#include "fr3c_follow_driver/pose_observation.hpp"
#include "fr3c_follow_driver/collision_guard.hpp"
#include "fr3c_follow_feedback/helpers.hpp"

using namespace fr3c_follow_driver;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted=0;
static void interrupt_handler(int) { interrupted=1; }
static double steady() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
static Json xyz(const Eigen::Vector3d & value) { return Json::array({value.x(),value.y(),value.z()}); }

class PoseFollowDriver final : public rclcpp::Node {
 public:
  explicit PoseFollowDriver(std::unique_ptr<Client> injected_client=nullptr,
                            const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions())
      : Node("pose_follow",node_options) {
    mock_=setting("mock",false); ip_=setting("robot_ip",std::string("192.168.58.2"));
    output_=setting("output_dir",std::string("")); target_id_=setting("target_id",1);
    period_=setting("period_ms",8.0)*.001;
    target_timeout_=setting("target_timeout_s",.35); feedback_timeout_=setting("feedback_timeout_s",.20);
    recovery_position_=setting("recovery_position_gap_m",.15);
    recovery_angle_=setting("recovery_angle_gap_deg",120.0)*kPi/180;
    if (!std::isfinite(period_+target_timeout_+feedback_timeout_+recovery_position_+recovery_angle_) ||
        period_<.001 || period_>.016 || std::min({target_timeout_,feedback_timeout_,recovery_position_,recovery_angle_})<=0)
      throw std::invalid_argument("invalid pose timeouts or recovery limits");
    PoseFollowOptions options;
    options.position_tau=setting("position_tau_s",.25); options.rotation_tau=setting("rotation_tau_s",.8);
    options.enter=setting("deadband_enter_deg",6.0)*kPi/180; options.exit=setting("deadband_exit_deg",4.0)*kPi/180;
    options.mapping=setting("rotation_mapping_gain",.5);
    options.kp_position=setting("kp_position",2.0); options.kp_rotation=setting("kp_rotation",1.0);
    options.speed=setting("max_speed_m_s",.020); options.accel=setting("max_accel_m_s2",.060);
    options.angular_speed=setting("max_angular_speed_deg_s",3.0)*kPi/180;
    options.angular_accel=setting("max_angular_accel_deg_s2",10.0)*kPi/180;
    options.radius=setting("follow_radius_m",1.00); options.angular_range=setting("follow_angle_deg",90.0)*kPi/180;
    options.position_deadband=setting("control_position_deadband_m",.004);
    options.angular_deadband=setting("control_angle_deadband_deg",2.0)*kPi/180;
    options.goal_position_threshold=setting("goal_position_threshold_m",.008);
    options.goal_angle_threshold=setting("goal_angle_threshold_deg",4.0)*kPi/180;
    options.goal_confirmation=setting("goal_confirmation_s",.20);
    options.position_lead=setting("reference_position_lead_m",.015);
    options.angular_lead=setting("reference_angle_lead_deg",5.0)*kPi/180;
    const auto p=setting("tcp_xyz_m",std::vector<double>{}),r=setting("tcp_rpy_rad",std::vector<double>{});
    if (p.size()!=3 || r.size()!=3) throw std::invalid_argument("provide the complete calibrated TCP through pose_follow.launch.py");
    core_=std::make_unique<PoseFollow>(Pose{p[0],p[1],p[2],r[0],r[1],r[2]},options);
    const bool collision_enabled=setting("collision_enabled",false);
    if (collision_enabled) {
      CollisionOptions collision_options;
      collision_options.speed=options.speed; collision_options.accel=options.accel;
      collision_options.angular_speed=options.angular_speed; collision_options.angular_accel=options.angular_accel;
      collision_options.table_z=setting("table_z_m",0.0); collision_options.table_size=setting("table_size_m",2.0);
      collision_options.margin=setting("collision_margin_m",.005);
      collision_options.slow_distance=setting("collision_slow_distance_m",.020);
      collision_options.period=setting("collision_check_period_ms",20.0)*.001;
      collision_options.timeout=setting("collision_result_timeout_s",.15);
      collision_model_=std::make_shared<CollisionModel>(setting("robot_description",std::string("")),
        setting("robot_description_semantic",std::string("")),collision_options);
      collision_guard_=std::make_unique<CollisionGuard>(collision_model_,collision_options);
    }
    client_=injected_client?std::move(injected_client):
      mock_?std::unique_ptr<Client>(new MockClient):std::unique_ptr<Client>(new NativeClient);
    if (mock_ && collision_model_) {
      const auto values=setting("mock_joints_rad",std::vector<double>{2.1766,-1.6214,1.5477,-1.5877,-1.355685307179586,0});
      if (values.size()!=6) throw std::invalid_argument("mock_joints_rad needs six values");
      Joints initial; std::copy(values.begin(),values.end(),initial.begin());
      auto * mock_client=dynamic_cast<MockClient *>(client_.get());
      if (!mock_client) throw std::invalid_argument("collision mock requires a model-consistent client");
      mock_client->use_model(collision_model_->flange(initial),initial,
        [model=collision_model_](const Pose & pose,Joints & joints) { return model->inverse(pose,joints); });
    }
    session_=std::make_unique<ServoSession>(*client_);
    flange_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/target_follow/flange_pose",20);
    goal_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/target_follow/desired_tcp_pose",10);
    state_pub_=create_publisher<std_msgs::msg::String>("/target_follow/follow_state",10);
    servo_pub_=create_publisher<std_msgs::msg::String>("/target_follow/servo_state",10);
    feedback_pub_=create_publisher<std_msgs::msg::String>("/target_follow/robot_feedback",20);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    observation_sub_=create_subscription<std_msgs::msg::String>("/target_follow/pose_observation",10,
      [this](std_msgs::msg::String::ConstSharedPtr message) { accept_observation(message->data); });
    using Request=std::shared_ptr<std_srvs::srv::Trigger::Request>;
    using Response=std::shared_ptr<std_srvs::srv::Trigger::Response>;
    teach_service_=create_service<std_srvs::srv::Trigger>("/target_follow/teach_offset",[this](Request,Response response) {
      try {
        if (following_ || fault_) throw std::runtime_error("stop pose following before recording a relation");
        require_data(); core_->teach(target_,actual_); ++teaches_;
        response->success=true; response->message="Full target-to-TCP relation recorded; motion remains stopped.";
      } catch (const std::exception & e) { response->success=false; response->message=e.what(); }
    });
    start_service_=create_service<std_srvs::srv::Trigger>("/target_follow/follow_start",[this](Request,Response response) {
      try {
        if (session_->active()) throw std::runtime_error("pose following is already active; stop before recording again");
        if (fault_ || stop_requested_) throw std::runtime_error("driver is stopping or faulted; inspect follow_state");
        require_ready();
        if (collision_guard_) {
          const auto check=collision_guard_->result_pose(steady(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
          if (!check.ready || check.colliding || check.scale<=0) throw std::runtime_error("cannot start: "+check.reason);
        }
        core_->teach(target_,actual_); ++teaches_; recapture_=false; pause_good_ns_=0;
        const int code=session_->start(); log({{"kind","start"},{"sdk_return_code",code},{"offset_matrix",matrix(core_->offset())}});
        if (code) { fail("ServoMoveStart failed: "+std::to_string(code)); throw std::runtime_error(reason_); }
        following_=true; status_="following"; reason_.clear(); ++starts_;
        response->success=true; response->message="Pose following started from the current TCP with a newly recorded relation.";
      } catch (const std::exception & e) { response->success=false; response->message=e.what(); }
    });
    stop_service_=create_service<std_srvs::srv::Trigger>("/target_follow/servo_stop",[this](Request,Response response) {
      stop_requested_=true; response->success=true; response->message="Stop requested; follow_state records the SDK stop result.";
    });
    report_={{"stage",9},{"follow_mode","pose"},{"mock",mock_},{"backend","ServoCart mode 0, tool/work-object 0"},
      {"hardware_servo_commands_sent",0},{"live_pose_follow_validated",false},{"collision_enabled",collision_enabled},
      {"requested_period_ms",period_*1000},{"rotation_mapping_gain",options.mapping},
      {"follow_radius_m",options.radius},{"follow_angle_deg",options.angular_range*180/kPi},
      {"goal_position_threshold_m",options.goal_position_threshold},{"goal_angle_threshold_deg",options.goal_angle_threshold*180/kPi},
      {"goal_confirmation_s",options.goal_confirmation},{"control_position_deadband_m",options.position_deadband},
      {"control_angle_deadband_deg",options.angular_deadband*180/kPi},
      {"sdk_angle_encoding","absolute RPY wrapped to [-180,180] deg"},
      {"speed_limit_mm_s",options.speed*1000},{"angular_speed_limit_deg_s",options.angular_speed*180/kPi}};
    if (!output_.empty()) {
      std::filesystem::create_directories(output_); samples_.open(output_+"/samples.jsonl");
      if (!samples_) throw std::runtime_error("cannot open samples.jsonl");
    }
  }
  int run() {
    int result=0;
    try {
      connected_=true; const int code=client_->connect(ip_.c_str());
      if (code) throw std::runtime_error("SDK connection failed: "+std::to_string(code));
      RCLCPP_INFO(get_logger(),"Stage 9 POSE FOLLOW starts idle. T records; S/start records again and starts; X/stop stops. mock=%s",mock_?"true":"false");
      double last_send=0,last_state=0;
      while (rclcpp::ok() && !interrupted) {
        const double began=steady(); read_feedback(); rclcpp::spin_some(shared_from_this()); update_collision();
        if (stop_requested_) {
          following_=false; finish(); core_->clear(); recapture_=false;
          if (!fault_) { status_="stopped"; reason_="user_stop"; }
          stop_requested_=false;
        }
        if (following_) {
          if (!fresh_feedback()) { pause("paused_feedback","robot feedback unavailable; waiting for fresh feedback and trusted pose"); last_send=0; }
          else {
            const auto problem=state_problem(packet_,false);
            if (!problem.empty()) fail(problem);
            else if (recapture_) { pause("paused_reteach",reason_); last_send=0; }
            else if (!usable_target() || (!session_->active() && (!quality_reason_.empty() || good_ns_<=pause_good_ns_))) {
              const std::string why=!fresh_target()?"stale_trusted_pose":
                !quality_reason_.empty()?quality_reason_:"waiting_for_new_trusted_pose";
              pause("paused_target",why); last_send=0;
            } else if (collision_guard_ && collision_.scale<=0) {
              pause("paused_collision",collision_.reason); last_send=0;
            } else if (!fault_) {
              if (!session_->active()) {
                const auto resume_problem=state_problem(packet_,true);
                if (!resume_problem.empty()) reason_="waiting_for_robot_to_stop";
                else {
                  core_->reanchor(actual_);
                  const int start_code=session_->start();
                  if (start_code) fail("ServoMoveStart on resume failed: "+std::to_string(start_code));
                  else {
                    if (status_=="paused_collision") ++collision_resumes_; else ++resumes_;
                    status_="following"; reason_.clear(); last_send=0;
                  }
                }
              }
              if (session_->active() && !fault_) {
                const double dt=last_send==0?period_:std::min(period_,std::max(.0001,began-last_send));
                try {
                  const Pose command=core_->step(actual_,dt,collision_guard_?collision_.scale:1.);
                  status_=quality_soft_?"holding_quality":"following";
                  reason_=quality_soft_?quality_reason_:collision_guard_&&collision_.scale<1?collision_.reason:"";
                  const double sent_at=steady(); const int send_code=session_->send(command,period_);
                  const double call_ms=(steady()-sent_at)*1000;
                  if (last_send>0) intervals_.push_back((sent_at-last_send)*1000);
                  last_send=sent_at; calls_.push_back(call_ms); ++sent_;
                  Json sample{{"kind","servo"},{"stamp_s",now().seconds()},{"target_flange_m_rad",command},
                    {"actual_flange_m_rad",actual_},{"target_sdk_mm_deg",sdk_pose(command)},
                    {"velocity_m_s",xyz(core_->velocity().linear)},{"angular_velocity_deg_s",xyz(core_->velocity().angular*180/kPi)},
                    {"target_age_ms",(now().nanoseconds()-good_ns_)/1e6},{"trusted_image_stamp",stamp(good_ns_)},
                    {"collision_scale",collision_guard_?collision_.scale:1.},{"call_ms",call_ms},{"sdk_return_code",send_code}};
                  sample["settled_hold"]=core_->holding(); sample["goal_pending"]=core_->goal_pending();
                  add_pose(sample,"actual_tcp",core_->tcp(actual_)); add_pose(sample,"desired_tcp",core_->goal());
                  add_pose(sample,"reference_tcp",core_->reference()); log(sample);
                  if (send_code) fail("ServoCart failed: "+std::to_string(send_code));
                  else if (call_ms>feedback_timeout_*1000) fail("ServoCart call exceeded feedback timeout");
                } catch (const std::exception & e) { fail(e.what()); }
              }
            }
          }
        } else last_send=0;
        if (steady()-last_state>=.05) { publish_state(); last_state=steady(); }
        std::this_thread::sleep_until(Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(began+period_))));
      }
      following_=false; finish(); core_->clear(); if (!fault_) { status_="stopped"; reason_="signal"; }
    } catch (const std::exception & e) { fail(e.what()); }
    following_=false; finish();
    if (connected_) { const int code=client_->close(); connected_=false; report_["close_code"]=code;
      if (code) { fault_=true; status_="fault"; reason_="SDK close failed: "+std::to_string(code); } }
    report_.update(Json{{"result",status_},{"reason",reason_},{"servo_commands_sent",sent_},{"hardware_servo_commands_sent",mock_?0:sent_},
      {"starts",starts_},{"teaches",teaches_},{"target_pauses",pauses_},{"target_resumes",resumes_},
      {"collision_pauses",collision_pauses_},{"collision_resumes",collision_resumes_},
      {"servo_call_ms",stats(calls_)},{"servo_interval_ms",stats(intervals_)},{"collision_check_ms",stats(collision_times_)}});
    publish_state();
    if (!output_.empty()) { std::ofstream file(output_+"/report.json"); file<<report_.dump(2)<<'\n'; if (!file) result=1; }
    std::cout<<report_.dump(2)<<'\n'; return fault_?1:result;
  }
 private:
  template<class T> T setting(const std::string & name,const T & value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  static Json matrix(const Eigen::Isometry3d & value) {
    Json result=Json::array(); for (int i=0;i<4;++i) result.push_back(Json::array({value.matrix()(i,0),value.matrix()(i,1),value.matrix()(i,2),value.matrix()(i,3)})); return result;
  }
  static Json stamp(std::int64_t ns) { return {{"sec",ns/1000000000LL},{"nanosec",ns%1000000000LL}}; }
  static Json sdk_pose(const Pose & value) {
    const auto pose=to_sdk_absolute(value); return Json::array({pose.tran.x,pose.tran.y,pose.tran.z,pose.rpy.rx,pose.rpy.ry,pose.rpy.rz});
  }
  static void add_pose(Json & row,const std::string & name,const Eigen::Isometry3d & value) {
    const Eigen::Quaterniond q(value.linear()); const auto & r=value.linear();
    row[name+"_m"]=xyz(value.translation()); row[name+"_quaternion_xyzw"]=Json::array({q.x(),q.y(),q.z(),q.w()});
    row[name+"_rpy_deg"]=Json::array({std::atan2(r(2,1),r(2,2))*180/kPi,
      std::atan2(-r(2,0),std::hypot(r(0,0),r(1,0)))*180/kPi,std::atan2(r(1,0),r(0,0))*180/kPi});
  }
  static Json stats(std::vector<double> values) {
    if (values.empty()) return nullptr;
    std::sort(values.begin(),values.end());
    return {{"count",values.size()},{"p50",values[(values.size()-1)/2]},
      {"p95",values[static_cast<std::size_t>((values.size()-1)*.95)]},{"max",values.back()}};
  }
  void log(const Json & sample) { if (samples_) samples_<<sample.dump()<<'\n'; }
  bool fresh_feedback() const { return have_packet_ && feedback_valid_ && freshness_.age(steady())<=feedback_timeout_; }
  bool fresh_target() const {
    const double age=(now().nanoseconds()-good_ns_)/1e9;
    return good_ns_>0 && age>=-.05 && age<=target_timeout_ && steady()-good_wall_<=target_timeout_;
  }
  bool usable_target() const { return fresh_target() && (quality_reason_.empty() || quality_soft_); }
  void require_data() {
    if (!connected_ || !fresh_feedback()) throw std::runtime_error("fresh robot feedback is required");
    if (!usable_target()) throw std::runtime_error("fresh trusted full pose is required: "+quality_reason_);
  }
  void require_ready() {
    require_data();
    if (get_publishers_info_by_topic("/target_follow/flange_pose").size()>1) throw std::runtime_error("another follow SDK owner is running");
    const auto problem=state_problem(packet_,true); if (!problem.empty()) throw std::runtime_error(problem);
  }
  void accept_observation(const std::string & data) {
    try {
      const Json row=Json::parse(data); const auto observation=parse_observation(row,target_id_);
      if (observation.stamp && (observation.stamp<event_ns_ || observation.stamp<=good_ns_)) return;
      if (observation.stamp) {
        const double age=(now().nanoseconds()-observation.stamp)/1e9;
        if (age<-.05 || age>target_timeout_) throw std::invalid_argument("stale_pose_observation");
        event_ns_=observation.stamp;
      }
      log({{"kind","observation"},{"observation",row}});
      quality_soft_=observation.soft; quality_reason_=observation.reason;
      if (!observation.trusted) return;
      orientation_approximate_=row["quality"].value("orientation_approximate",false);
      candidate_tolerance_=row["quality"].value("orientation_candidate_tolerance_deg",5.);
      const double dt=good_ns_?(observation.stamp-good_ns_)/1e9:period_;
      target_=observation.pose;
      if (core_->taught() && !recapture_) {
        if (following_ && !session_->active() && !core_->recovery_ok(target_,recovery_position_,recovery_angle_)) {
          recapture_=true; reason_="target relocated during pause; S/start records the current relation";
        } else {
          try { core_->observe(target_,dt); }
          catch (const std::out_of_range &) {
            if (following_) { recapture_=true; reason_="target is outside the pose follow range; S/start records a new relation"; }
            else core_->clear();
          }
        }
      }
      good_ns_=observation.stamp; good_wall_=steady(); quality_reason_.clear(); quality_soft_=false;
    } catch (const std::exception & e) { quality_reason_=e.what(); quality_soft_=false; }
  }
  void update_collision() {
    if (!collision_guard_ || !fresh_feedback()) return;
    CollisionInput input; input.pose_motion=true; input.flange_tcp=core_->tool();
    input.joints=joints_; input.flange=actual_; input.stamp=steady();
    if (following_ && core_->taught() && !recapture_) {
      const auto actual_tcp=core_->tcp(actual_);
      input.velocity=core_->velocity().linear; input.angular_velocity=core_->velocity().angular;
      input.lead=core_->reference().translation()-actual_tcp.translation();
      input.rotation_lead=core_->reference().linear()*actual_tcp.linear().transpose();
      if (usable_target()) {
        const auto request=core_->requested(actual_);
        if (request.linear.norm()>1e-6) input.requested=request.linear.normalized()*core_->options().speed;
        if (request.angular.norm()>1e-6) input.requested_angular=request.angular.normalized()*core_->options().angular_speed;
      }
    }
    collision_guard_->submit(input);
    collision_=collision_guard_->result_pose(steady(),input.requested,input.requested_angular);
    if (collision_.ready && collision_.stamp!=last_collision_stamp_) {
      collision_times_.push_back(collision_.check_ms); last_collision_stamp_=collision_.stamp;
    }
  }
  geometry_msgs::msg::PoseStamped pose_message(const Eigen::Isometry3d & value) {
    geometry_msgs::msg::PoseStamped message; message.header.frame_id="base_link"; message.header.stamp=now();
    message.pose.position.x=value.translation().x(); message.pose.position.y=value.translation().y(); message.pose.position.z=value.translation().z();
    const Eigen::Quaterniond q(value.linear()); message.pose.orientation.x=q.x(); message.pose.orientation.y=q.y();
    message.pose.orientation.z=q.z(); message.pose.orientation.w=q.w(); return message;
  }
  void read_feedback() {
    ROBOT_STATE_PKG latest{}; const double began=steady(); const int code=client_->read(&latest);
    if (steady()-began>feedback_timeout_ && following_) { feedback_valid_=false; pause("paused_feedback","SDK feedback read exceeded timeout"); return; }
    if (code || latest.frame_head!=0x5A5A) { feedback_valid_=false; if (following_) pause("paused_feedback","SDK feedback read failed: "+std::to_string(code)); return; }
    fr3c_follow_feedback::PacketFreshness::Key key{latest.frame_cnt,latest.robotTime.year,latest.robotTime.mouth,
      latest.robotTime.day,latest.robotTime.hour,latest.robotTime.minute,latest.robotTime.second,latest.robotTime.millisecond};
    if (!freshness_.observe(key,steady())) return;
    Pose actual{}; Joints next_joints{};
    try {
      actual=from_sdk(latest.flange_cur_pos);
      for (unsigned i=0;i<6;++i) {
        next_joints[i]=latest.jt_cur_pos[i]*kPi/180;
        if (!std::isfinite(next_joints[i]) || (following_ && have_packet_ && std::abs(next_joints[i]-joints_[i])>15*kPi/180))
          throw std::runtime_error("invalid or discontinuous joint feedback");
      }
    } catch (const std::exception & e) {
      feedback_valid_=false; if (following_) pause("paused_feedback",e.what()); return;
    }
    packet_=latest; actual_=actual; joints_=next_joints; have_packet_=true; feedback_valid_=true;
    auto pose=pose_message(pose_transform(actual_)); flange_pub_->publish(pose);
    geometry_msgs::msg::TransformStamped tf; tf.header=pose.header; tf.child_frame_id="flange_link";
    tf.transform.translation.x=actual_[0]; tf.transform.translation.y=actual_[1]; tf.transform.translation.z=actual_[2];
    tf.transform.rotation=pose.pose.orientation; tf_->sendTransform(tf);
    Json state{{"status","valid"},{"mock",mock_},{"sdk_return_code",0},
      {"stamp_source","host_first_observation_of_new_sdk_packet"},{"feedback_age_s",freshness_.age(steady())},
      {"robot_mode",packet_.robot_mode},{"robot_state",packet_.robot_state},{"program_state",packet_.program_state},
      {"enabled",packet_.rbtEnableState},{"main_error_code",packet_.main_code},{"sub_error_code",packet_.sub_code},
      {"emergency_stop",packet_.EmergencyStop},{"tool_id",packet_.tool},{"work_object_id",packet_.user},
      {"tool_coord_mm_deg",std::vector<double>(packet_.toolCoord,packet_.toolCoord+6)},
      {"work_object_coord_mm_deg",std::vector<double>(packet_.wobjCoord,packet_.wobjCoord+6)},
      {"motion_commands_enabled",session_->active()&&!mock_}};
    std_msgs::msg::String message; message.data=state.dump(); feedback_pub_->publish(message);
    log({{"kind","feedback"},{"stamp_s",rclcpp::Time(pose.header.stamp).seconds()},{"flange_m_rad",actual_},{"joints_rad",joints_}});
  }
  void finish() {
    if (!session_->active()) return;
    const auto result=session_->finish();
    report_["stop_motion_code"]=result.stop_code; report_["servo_end_code"]=result.end_code;
    log({{"kind","stop"},{"stop_motion_code",result.stop_code},{"servo_end_code",result.end_code}});
    if (result.stop_code || result.end_code) { fault_=true; following_=false; status_="fault"; reason_="SDK stop/end failed"; }
  }
  void pause(const std::string & state,const std::string & reason) {
    if (session_->active()) { pause_good_ns_=good_ns_; finish(); if (state=="paused_collision") ++collision_pauses_; else ++pauses_; }
    core_->reanchor(actual_);
    if (!fault_) { status_=state; reason_=reason; }
  }
  void fail(const std::string & reason) {
    following_=false; fault_=true; status_="fault"; reason_=reason; finish(); core_->clear();
    RCLCPP_ERROR(get_logger(),"%s",reason_.c_str());
  }
  void publish_state() {
    Json state{{"stage",9},{"follow_mode","pose"},{"status",status_},{"reason",reason_},{"mock",mock_},
      {"stamp_s",now().seconds()},{"motion_commands_enabled",session_->active()&&!mock_},
      {"orientation_control_enabled",session_->active()&&!mock_},{"following_requested",following_},
      {"offset_recorded",core_->taught()},{"servo_commands_sent",sent_},{"starts",starts_},{"teach_count",teaches_},
      {"target_pauses",pauses_},{"target_resumes",resumes_},{"requires_reteach",recapture_},
      {"quality_reason",quality_reason_},{"trusted_image_stamp",good_ns_?stamp(good_ns_):Json(nullptr)},
      {"orientation_approximate",orientation_approximate_},{"orientation_candidate_tolerance_deg",candidate_tolerance_},
      {"rotation_mapping_gain",core_->options().mapping},{"deadband_enter_deg",core_->options().enter*180/kPi},
      {"settled_hold",core_->taught()&&core_->holding()},{"goal_pending",core_->taught()&&core_->goal_pending()},
      {"velocity_m_s",session_->active()?xyz(core_->velocity().linear):Json::array({0.,0.,0.})},
      {"angular_velocity_deg_s",session_->active()?xyz(core_->velocity().angular*180/kPi):Json::array({0.,0.,0.})}};
    state["tracking_mode"]=collision_guard_?"pose_collision_guarded":"pose";
    state["collision"]={{"enabled",false},{"ready",false},{"speed_scale",1.0},{"reason","disabled"}};
    if (collision_guard_) state["collision"]={{"enabled",true},{"ready",collision_.ready},{"pose_prediction",true},
      {"speed_scale",collision_.scale},{"distance_mm",collision_.distance*1000},{"predicted_distance_mm",collision_.predicted_distance*1000},
      {"reason",collision_.reason},{"pair",collision_.pair},{"check_ms",collision_.check_ms},{"age_ms",(steady()-collision_.stamp)*1000}};
    if (good_ns_) { add_pose(state,"target_base",target_); state["target_age_ms"]=(now().nanoseconds()-good_ns_)/1e6; }
    if (have_packet_) { add_pose(state,"actual_tcp",core_->tcp(actual_)); state["feedback_age_ms"]=freshness_.age(steady())*1000; }
    if (core_->taught()) {
      const auto goal=core_->goal(); add_pose(state,"desired_tcp",goal); add_pose(state,"filtered_target",core_->filtered());
      add_pose(state,"reference_tcp",core_->reference()); state["offset_matrix"]=matrix(core_->offset());
      state["rotation_active"]=core_->rotation_active();
      if (have_packet_) {
        const auto actual=core_->tcp(actual_); const Eigen::Vector3d ep=goal.translation()-actual.translation();
        state["position_error_m"]=xyz(ep); state["error_norm_mm"]=ep.norm()*1000;
        state["rotation_error_deg"]=rotation_distance(goal.linear(),actual.linear())*180/kPi;
      }
      goal_pub_->publish(pose_message(goal));
    }
    for (const char * key:{"stop_motion_code","servo_end_code"}) if (report_.contains(key)) state[key]=report_[key];
    std_msgs::msg::String message; message.data=state.dump(); state_pub_->publish(message); servo_pub_->publish(message);
    log({{"kind","state"},{"state",state}});
  }
  bool mock_=false,connected_=false,have_packet_=false,feedback_valid_=false,following_=false,stop_requested_=false,fault_=false;
  bool quality_soft_=false,recapture_=false,orientation_approximate_=false;
  int target_id_=1;
  std::int64_t good_ns_=0,event_ns_=0,pause_good_ns_=0;
  double period_,target_timeout_,feedback_timeout_,recovery_position_,recovery_angle_,good_wall_=0,last_collision_stamp_=0,candidate_tolerance_=5.;
  unsigned sent_=0,starts_=0,teaches_=0,pauses_=0,resumes_=0,collision_pauses_=0,collision_resumes_=0;
  std::string ip_,output_,status_="idle",reason_,quality_reason_="waiting_for_trusted_pose";
  Eigen::Isometry3d target_=Eigen::Isometry3d::Identity(); Pose actual_{}; Joints joints_{};
  ROBOT_STATE_PKG packet_{}; fr3c_follow_feedback::PacketFreshness freshness_;
  std::unique_ptr<Client> client_; std::unique_ptr<ServoSession> session_; std::unique_ptr<PoseFollow> core_;
  std::shared_ptr<CollisionModel> collision_model_; std::unique_ptr<CollisionGuard> collision_guard_; CollisionOutput collision_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  Json report_; std::ofstream samples_; std::vector<double> calls_,intervals_,collision_times_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr flange_pub_,goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_,servo_pub_,feedback_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr observation_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr teach_service_,start_service_,stop_service_;
};

int main(int argc,char ** argv) {
  try {
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,interrupt_handler); std::signal(SIGTERM,interrupt_handler);
    const int result=std::make_shared<PoseFollowDriver>()->run(); rclcpp::shutdown(); return result;
  } catch (const std::exception & e) { std::cerr<<e.what()<<'\n'; if (rclcpp::ok()) rclcpp::shutdown(); return 1; }
}
