// Read-only SDK bridge: RPC, GetRobotRealTimeState, CloseRPC are the only SDK calls.
#include <chrono>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "robot.h"
#include "fr3c_follow_feedback/helpers.hpp"

using fr3c_follow_feedback::PacketFreshness;
using Clock = std::chrono::steady_clock;

class RobotFeedback final : public rclcpp::Node {
 public:
  RobotFeedback() : Node("follow_robot_feedback") {
    ip_ = declare_parameter<std::string>("controller_ip", "192.168.58.2");
    const double hz = declare_parameter<double>("poll_hz", 125.0);
    stale_s_ = declare_parameter<double>("stale_timeout_s", 0.20);
    if (!std::isfinite(hz) || hz <= 0 || hz > 1000 || !std::isfinite(stale_s_) || stale_s_ <= 0)
      throw std::invalid_argument("invalid polling/freshness parameters");
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_follow/flange_pose", 20);
    status_pub_ = create_publisher<std_msgs::msg::String>("/target_follow/robot_feedback", 20);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const int code = robot_.RPC(ip_.c_str());
    if (code != 0) throw std::runtime_error("read-only SDK RPC failed: " + std::to_string(code));
    connected_ = true;
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this] { poll(); });
    RCLCPP_INFO(get_logger(), "Read-only feedback connected to %s; no motion API", ip_.c_str());
  }
  ~RobotFeedback() override {
    if (timer_) timer_->cancel();
    if (connected_) robot_.CloseRPC();
  }
 private:
  double steady_seconds() const {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
  }
  void status(const std::string & state, int code, double call_ms,
              const ROBOT_STATE_PKG * packet = nullptr) {
    const double age = freshness_.age(steady_seconds());
    nlohmann::json value = {{"status", state}, {"sdk_return_code", code},
      {"call_ms", call_ms}, {"feedback_age_s", age}, {"new_packets", count_},
      {"motion_commands_enabled", false},
      {"stamp_source", "host_first_observation_of_new_sdk_packet"}};
    if (packet) {
      value["frame_counter"] = packet->frame_cnt;
      value["controller_time"] = {packet->robotTime.year, packet->robotTime.mouth,
        packet->robotTime.day, packet->robotTime.hour, packet->robotTime.minute,
        packet->robotTime.second, packet->robotTime.millisecond};
      value["robot_mode"] = packet->robot_mode;
      value["robot_state"] = packet->robot_state;
      value["program_state"] = packet->program_state;
      value["main_error_code"] = packet->main_code;
      value["sub_error_code"] = packet->sub_code;
      value["emergency_stop"] = packet->EmergencyStop;
      value["enabled"] = packet->rbtEnableState;
      value["tool_id"] = packet->tool;
      value["work_object_id"] = packet->user;
      value["controller_tcp_mm_deg"] = std::vector<double>(packet->tl_cur_pos, packet->tl_cur_pos + 6);
      value["tool_coord_mm_deg"] = std::vector<double>(packet->toolCoord, packet->toolCoord + 6);
      value["work_object_coord_mm_deg"] = std::vector<double>(packet->wobjCoord, packet->wobjCoord + 6);
    }
    std_msgs::msg::String message; message.data = value.dump(); status_pub_->publish(message);
    last_status_ = steady_seconds();
  }
  void poll() {
    ROBOT_STATE_PKG packet{};
    const double start = steady_seconds();
    const int code = robot_.GetRobotRealTimeState(&packet);
    const double call_ms = (steady_seconds() - start) * 1000;
    if (code != 0 || packet.frame_head != 0x5A5A) {
      if (steady_seconds() - last_status_ > 0.2) status("no_valid_feedback", code, call_ms);
      return;
    }
    PacketFreshness::Key key{packet.frame_cnt, packet.robotTime.year, packet.robotTime.mouth,
      packet.robotTime.day, packet.robotTime.hour, packet.robotTime.minute,
      packet.robotTime.second, packet.robotTime.millisecond};
    if (!freshness_.observe(key, steady_seconds())) {
      if (freshness_.age(steady_seconds()) > stale_s_ && steady_seconds() - last_status_ > 0.2)
        status("stale_feedback", code, call_ms, &packet);
      return;  // Never give an unchanged cached packet a new ROS timestamp.
    }
    try {
      const auto transform = fr3c_follow_feedback::pose_mm_deg(packet.flange_cur_pos);
      const Eigen::Quaterniond q(transform.rotation());
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now(); pose.header.frame_id = "base_link";
      pose.pose.position.x = transform.translation().x();
      pose.pose.position.y = transform.translation().y();
      pose.pose.position.z = transform.translation().z();
      pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
      pose_pub_->publish(pose);
      geometry_msgs::msg::TransformStamped tf;
      tf.header = pose.header; tf.child_frame_id = "flange_link";
      tf.transform.translation.x = pose.pose.position.x;
      tf.transform.translation.y = pose.pose.position.y;
      tf.transform.translation.z = pose.pose.position.z;
      tf.transform.rotation = pose.pose.orientation; tf_->sendTransform(tf);
      ++count_;
      status("valid", code, call_ms, &packet);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
      status("invalid_pose", code, call_ms, &packet);
    }
  }
  FRRobot robot_;
  std::string ip_;
  bool connected_ = false;
  double stale_s_ = 0.2, last_status_ = 0.0;
  unsigned count_ = 0;
  PacketFreshness freshness_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<RobotFeedback>()); }
  catch (const std::exception & error) { RCLCPP_ERROR(rclcpp::get_logger("follow_feedback"), "%s", error.what()); result = 1; }
  rclcpp::shutdown();
  return result;
}
