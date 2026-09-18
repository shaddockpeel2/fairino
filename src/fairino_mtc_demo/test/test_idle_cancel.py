"""Compile the production cancel handler with recording stand-ins, without hardware."""
from pathlib import Path
import subprocess


def test_cancel_preserves_idle_terminal_states_and_stops_active_tasks(tmp_path):
    source = (Path(__file__).parents[1] / 'src/first_grasp_node.cpp').read_text()
    start = source.index('  void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)')
    end = source.index('\n  void acknowledgeObjectReleased(', start)
    handler = source[start:end]
    harness = r'''
#include <atomic>
#include <cassert>
#include <memory>
#include <string>
namespace std_srvs { namespace srv {
struct Trigger { struct Response { bool success = false; std::string message; }; };
}}
struct Arm { int stops = 0; void stop() { ++stops; } };
struct Gripper { int stops = 0; void async_cancel_all_goals() { ++stops; } };
struct Condition { int notices = 0; void notify_all() { ++notices; } };
struct Task {
  std::atomic<bool> worker_running_{false}, action_goal_claimed_{false}, canceled_{false};
  std::unique_ptr<Arm> move_group_ = std::make_unique<Arm>();
  std::unique_ptr<Gripper> gripper_client_ = std::make_unique<Gripper>();
  Condition condition_;
  static constexpr int CANCELED = 9;
  int state = 0;
  void setState(int value, const std::string &) { state = value; }
''' + handler + r'''
};
int main() {
  // Waiting, plan complete, holding, failed, canceled, retryable and sorted.
  for (int state : {0, 6, 7, 8, 9, 10, 13, 15, 16}) {
    Task task;
    task.state = state;
    for (int repeat = 0; repeat < 3; ++repeat) {
      auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
      task.cancel(response);
      assert(response->success);
      assert(task.state == state);
      assert(!task.canceled_);
      assert(task.move_group_->stops == 0);
      assert(task.gripper_client_->stops == 0);
      assert(task.condition_.notices == 0);
    }
  }
  // A task awaiting target/authorization and a claimed Action must still stop.
  for (int flags : {1, 2, 3}) {
    Task task;
    task.state = 0;
    task.worker_running_ = flags & 1;
    task.action_goal_claimed_ = flags & 2;
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    task.cancel(response);
    assert(response->success && task.canceled_);
    assert(task.state == Task::CANCELED);
    assert(task.move_group_->stops == 1 && task.gripper_client_->stops == 1);
    assert(task.condition_.notices == 1);
  }
}
'''
    cpp = tmp_path / 'cancel.cpp'
    binary = tmp_path / 'cancel'
    cpp.write_text(harness)
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Werror', str(cpp), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    subprocess.run([str(binary)], check=True, timeout=5)
