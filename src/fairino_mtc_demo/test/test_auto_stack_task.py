"""
Contract of the automatic stack: one command, one whole tower.

pick_stack_auto stacks the cubes that are on the tabletop in a fixed order, with
red on top and green on the table, skipping every colour that is not there. The
step order is the one thing an operator cannot see from the outside, so it is
pinned down here by running the production pairing arithmetic, and the rest of
the contract (fixed order, ownership of the cell, per-step reuse of pick_stack)
is pinned down by reading the production unit.
"""

import shutil
import subprocess
from pathlib import Path

import pytest


HEADER = r'''
#include <cstdio>
#include <string>
#include <vector>
'''

HARNESS = r'''
int failures = 0;

void check(bool ok, const char * what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

// Steps are executed in order; each step puts `upper` on `lower`, and the first
// one puts a cube straight onto the table.
std::vector<std::string> buildTower(const std::vector<std::string> & order)
{
  std::vector<std::string> tower;
  for (std::size_t step = 0; step + 1 < order.size(); ++step) {
    std::string upper;
    std::string lower;
    autoStackStepColors(order, step, upper, lower);
    if (tower.empty()) {
      tower.push_back(lower);
    }
    if (tower.back() != lower) {
      std::printf("FAIL: step %zu does not place on the current tower top\n", step);
      ++failures;
    }
    tower.push_back(upper);
  }
  return tower;
}

void checkOrder(const std::vector<std::string> & order,
                const std::vector<std::vector<std::string>> & expected_steps)
{
  check(expected_steps.size() + 1 == order.size(),
        "the expected step list does not match the order length");
  for (std::size_t step = 0; step < expected_steps.size(); ++step) {
    std::string upper;
    std::string lower;
    autoStackStepColors(order, step, upper, lower);
    check(upper == expected_steps[step][0] && lower == expected_steps[step][1],
          "the step picked the wrong pair of colours");
  }
  // Running the steps in order must reproduce the planned order exactly, read
  // from the table upwards.
  const std::vector<std::string> tower = buildTower(order);
  check(tower.size() == order.size(), "the tower does not have one cube per colour");
  for (std::size_t index = 0; index < order.size() && index < tower.size(); ++index) {
    if (tower[index] != order[order.size() - 1 - index]) {
      std::printf("FAIL: tower level %zu is not the planned colour\n", index);
      ++failures;
    }
  }
}

int main() {
  // All four colours: green on the table, red on top.
  checkOrder(
    {"red", "yellow", "blue", "green"},
    {{"blue", "green"}, {"yellow", "blue"}, {"red", "yellow"}});

  // Yellow missing: red on top, then blue, green on the table.
  checkOrder(
    {"red", "blue", "green"},
    {{"blue", "green"}, {"red", "blue"}});

  // Blue missing as well.
  checkOrder({"red", "green"}, {{"red", "green"}});

  // Only the three lower colours present: the tower is one cube shorter.
  checkOrder(
    {"yellow", "blue", "green"},
    {{"blue", "green"}, {"yellow", "blue"}});

  if (failures == 0) {
    std::printf("all automatic stack order cases passed\n");
  }
  return failures == 0 ? 0 : 1;
}
'''


def _source() -> str:
    return (Path(__file__).parents[1] / 'src' / 'first_grasp_node.cpp').read_text()


def _unit(source: str, start_marker: str, end_marker: str) -> str:
    start = source.index(start_marker)
    return source[start:source.index(end_marker, start)]


def test_automatic_order_skips_missing_colours(tmp_path):
    """Red on top, then yellow, blue, green towards the table."""
    if shutil.which('c++') is None:
        pytest.skip('no C++ compiler is available')
    pairing = _unit(
        _source(), '  static void autoStackStepColors(', '  static void setPromiseValue(')
    unit = tmp_path / 'auto_stack_order.cpp'
    unit.write_text(HEADER + pairing + HARNESS, encoding='utf-8')
    binary = tmp_path / 'auto_stack_order'
    compiled = subprocess.run(
        ['c++', '-std=c++17', '-Wall', '-Wextra', '-o', str(binary), str(unit)],
        capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    executed = subprocess.run([str(binary)], capture_output=True, text=True)
    assert executed.returncode == 0, executed.stdout + executed.stderr


def test_automatic_order_is_fixed_in_the_unit():
    """The order is a constant, never a caller-supplied list."""
    source = _source()
    assert 'kAutoStackOrder{"red", "yellow", "blue", "green"}' in source
    order = _unit(source, 'constexpr std::array<const char *, 4> kAutoStackOrder',
                  'class FirstGraspTask')
    assert 'red' in order and 'green' in order


def test_automatic_stack_is_a_separate_action_with_no_parameters():
    """The caller supplies no colour; the tower is the unit's decision."""
    source = _source()
    assert 'kAutoStackOrder' in source
    assert 'createAutoStackServer("~/pick_stack_auto")' in source
    assert 'rclcpp_action::Server<PickColor>::SharedPtr createAutoStackServer(' in source
    auto_server = _unit(
        source, '  rclcpp_action::Server<PickColor>::SharedPtr createAutoStackServer(',
        '  // -----------------------------------------------------------------------\n'
        '  // pick_stack_auto: discovery, ordering and the step loop')
    assert '!goal->color.empty()' in auto_server
    assert '!goal->zone.empty()' in auto_server
    assert '!goal->support_color.empty()' in auto_server
    assert 'rclcpp_action::GoalResponse::REJECT' in auto_server
    # Zone and support colour are part of the result, not of the request: the
    # finish helper reports the ends of the tower that was actually planned.
    finish = _unit(source, '  void finishAutoStackAction(',
                   '  void cancelInternalStackGoal()')
    assert 'result->color = order.empty() ? std::string{} : order.front();' in finish
    assert 'result->support_color = order.empty() ? std::string{} : order.back();' in finish
    assert 'result->zone.clear();' in finish
    # Nothing in the orchestrator reads a requested colour out of the goal.
    execute = _unit(source, '  void executeAutoStackAction(',
                    '  void readParameters()')
    assert 'get_goal()->color' not in execute
    assert 'get_goal()->support_color' not in execute


def test_automatic_stack_discovers_only_tabletop_cubes():
    """A cube that is already part of a tower is not an automatic source."""
    source = _source()
    discovery = _unit(source, '  bool discoverAutoStackOrder(',
                      '  bool runAutoStackStep(')
    assert 'cube.stack_tier == 0' in discovery
    # Every colour must be unambiguous on the table.
    assert 'count > 1' in discovery
    assert 'PickColor::Result::TARGET_NOT_UNIQUE' in discovery
    # A tower needs at least two colours ...
    assert 'available.size() < 2' in discovery
    assert 'PickColor::Result::TARGET_NOT_FOUND' in discovery
    # ... and must fit into the configured tower height.
    assert 'available.size() > static_cast<std::size_t>(maximum_stack_cubes_)' in discovery
    assert 'PickColor::Result::STACK_GEOMETRY_REJECTED' in discovery
    # The colour list has to come from one fresh four-colour status.
    inventory = _unit(source, '  bool waitForColourInventory(',
                      '  bool discoverAutoStackOrder(')
    assert 'data.value("detection_mode", "") == "colors"' in inventory
    assert 'data.value("filter_generation", uint64_t{0}) == filter_generation' in inventory
    assert 'detectedCubesFromStatus(' in inventory


def test_automatic_stack_reuses_the_first_inventory_and_advances_only_the_tower_top():
    """Later layers must not wait for another camera/filter generation."""
    source = _source()
    execute = _unit(source, '  void executeAutoStackAction(',
                    '  void readParameters()')
    lock = _unit(source, '  bool lockAutoStackPairFromInventory(',
                 '  bool advanceAutoStackInventory(')
    advance = _unit(source, '  bool advanceAutoStackInventory(',
                    '  // Run one single-cube stack')
    action = _unit(source, '  void executePickAction(',
                   '  // -----------------------------------------------------------------------\n'
                   '  // pick_stack_auto: discovery, ordering and the step loop')

    assert 'auto_stack_inventory_ = inventory;' in execute
    assert 'advanceAutoStackInventory(upper, lower, reason)' in execute
    assert 'lockAutoStackPairFromInventory(' in action
    assert 'if (internal_stack_step) {' in action
    assert 'setPerceptionTargetColor(color, filter_generation, reason)' in action
    assert action.index('if (internal_stack_step) {') < action.index(
        'setPerceptionTargetColor(color, filter_generation, reason)')
    assert 'auto_stack_inventory_' in lock
    assert 'lockTargetSnapshot(' in lock
    assert 'lockStackDestinationSnapshot(cubes, *support, reason)' in lock
    assert 'source->center.x = locked_stack_destination_.top_pose.position.x;' in advance
    assert 'source->stack_tier = locked_stack_destination_.stack_tier + 1;' in advance


def test_intermediate_layers_continue_from_retreat_and_only_the_last_returns_home():
    source = _source()
    step = _unit(source, '  bool runAutoStackStep(',
                 '  void executeAutoStackAction(')
    action = _unit(source, '  void executePickAction(',
                   '  // -----------------------------------------------------------------------\n'
                   '  // pick_stack_auto: discovery, ordering and the step loop')
    preview = _unit(source, '  std::vector<PlannedMotion> previewAll()',
                    '  void publishPreview(')
    cycle = _unit(source, '  bool runCycleBody()', '  void runBatch()')

    assert 'auto_stack_step_number_.store(step);' in step
    assert 'auto_stack_step_count_.store(total_steps);' in step
    assert 'task_return_home_ = !internal_stack_step ||' in action
    assert 'task_pre_open_ = !internal_stack_step ||' in action
    assert 'task_direct_approach_ = internal_stack_step &&' in action
    assert 'prior_state == STACK_COMPLETE && !internal_stack_step' in action
    assert 'if (task_return_home_) {' in preview
    assert 'planHome(' in preview
    assert 'if (task_direct_approach_) {' in preview
    assert 'approach_cartesian_fallback' in preview
    assert 'if (canceled_.load()) {' in preview
    assert 'last candidate failure: ' in preview
    assert 'planCartesian(prefix + "approach", start, approachPose())' in preview
    assert 'planPoseFromSeededIk(prefix + "approach", start, approachPose())' in preview
    assert 'execute_ && task_pre_open_' in cycle
    assert 'if (task_return_home_) {' in cycle
    assert 'planHome("return_home_execute"' in cycle
    assert 'the arm remains at the safe retreat pose for the next layer' in cycle


def test_stack_lift_clears_the_current_tower_before_crossing_to_it():
    """The fourth cube must not start its transfer below the three-cube top."""
    source = _source()
    lift = _unit(source, '  geometry_msgs::msg::Pose liftPose() const',
                 '  geometry_msgs::msg::Pose placePose() const')
    assert 'task_stack_enabled_ && locked_stack_destination_.valid' in lift
    assert 'locked_stack_destination_.top_pose.position.z' in lift
    assert 'stack_approach_clearance_m_' in lift
    assert 'pose.position.z = std::max(pose.position.z, tower_clearance_tcp_z);' in lift


def test_each_automatic_step_is_a_real_pick_stack_goal():
    """Both paths must share one implementation and one set of checks."""
    source = _source()
    assert 'bool runAutoStackStep(' in source
    step = _unit(source, '  bool runAutoStackStep(', '  void executeAutoStackAction(')
    assert 'pick_stack_client_->async_send_goal(goal, options);' in step
    assert 'goal.color = upper_color;' in step
    assert 'goal.zone = kAutoStackInternalZone;' in step
    assert 'goal.support_color = lower_color;' in step
    assert '"~/internal/pick_stack"' in source
    # The step may not re-implement the cycle, and it must wait for the result
    # before the next layer is attempted.
    assert 'runCycle' not in step
    assert 'future.wait_for(100ms)' in step


def test_automatic_stack_owns_the_cell_until_the_tower_is_done():
    """No other grasp may slip in between two layers."""
    source = _source()
    # The pick servers refuse everything that is not the internal step.
    guard = _unit(source, '  rclcpp_action::Server<PickColor>::SharedPtr createPickServer(',
                  '  rclcpp_action::Server<PickColor>::SharedPtr createAutoStackServer(')
    assert 'auto_stack_active_.load() && !internal_only' in guard
    assert 'PickColor::Result::BUSY' in guard
    assert 'goal->zone != kAutoStackInternalZone' in guard
    assert 'internal_stack_call_' not in source
    assert guard.index('std::lock_guard<std::mutex> start_lock(start_mutex_);') < \
        guard.index('auto_stack_active_.load() && !internal_only')

    # A second automatic stack is refused as well.
    auto_server = _unit(
        source, '  rclcpp_action::Server<PickColor>::SharedPtr createAutoStackServer(',
        '  // -----------------------------------------------------------------------\n'
        '  // pick_stack_auto: discovery, ordering and the step loop')
    assert 'if (auto_stack_running_) {' in auto_server
    assert 'another automatic stack is already running' in auto_server
    assert auto_server.index(
        'std::lock_guard<std::mutex> start_lock(start_mutex_);') < \
        auto_server.index('if (action_goal_claimed_.load() || worker_running_.load())')

    # Internal steps use a separate endpoint with a marker instead of opening a
    # process-wide authorization window on the public endpoint.
    step = _unit(source, '  bool runAutoStackStep(', '  void executeAutoStackAction(')
    assert 'goal.zone = kAutoStackInternalZone;' in step
    assert 'internal_stack_call_' not in step
    # The flag is cleared on every exit path of the orchestrator.
    execute = _unit(source, '  void executeAutoStackAction(', '  void readParameters()')
    assert '~RunningGuard()' in execute
    assert 'active.store(false);' in execute

    # The task-slot status the cancel and release services read must know about
    # the automatic stack, which is idle between two layers.
    assert '!action_goal_claimed_.load() &&\n      !auto_stack_active_.load()' in source
    assert 'action_goal_claimed_.load() ||\n      auto_stack_active_.load()' in source
    start = _unit(source, '  void start(', '  void authorizeDescend(')
    assert 'if (auto_stack_active_.load()) {' in start
    assert 'pick_stack_auto owns the arm' in start


def test_automatic_stack_thread_is_joined_and_shutdown_breaks_step_wait():
    source = _source()
    destructor = _unit(source, '  ~FirstGraspTask()', '\nprivate:')
    assert 'cancelInternalStackGoal();' in destructor
    assert 'if (auto_action_thread_.joinable()) {' in destructor
    assert 'auto_action_thread_.join();' in destructor

    step = _unit(source, '  bool runAutoStackStep(', '  void executeAutoStackAction(')
    assert 'if (!rclcpp::ok()) {' in step
    assert 'ROS shutdown interrupted the internal pick_stack step' in step


def test_automatic_stack_forwards_cancellation_and_reports_progress():
    source = _source()
    auto_server = _unit(
        source, '  rclcpp_action::Server<PickColor>::SharedPtr createAutoStackServer(',
        '  // -----------------------------------------------------------------------\n'
        '  // pick_stack_auto: discovery, ordering and the step loop')
    assert 'cancelInternalStackGoal();' in auto_server

    cancel = _unit(source, '  void cancelInternalStackGoal()',
                   '  bool waitForColourInventory(')
    assert 'pick_stack_client_->async_cancel_goal(handle);' in cancel

    # A cancel request on the goal and the operator cancel service both stop the
    # loop, and it stops between layers rather than in the middle of one.
    execute = _unit(source, '  void executeAutoStackAction(', '  void readParameters()')
    assert 'if (goal_handle->is_canceling() || canceled_.load()) {' in execute
    assert 'PickColor::Result::CANCELED' in execute
    assert 'publishAutoStackFeedback(' in execute
    assert 'auto_stack_discovery' in execute
    assert 'auto_stack_step' in execute

    # The published state names the automatic mode and the planned order.
    state = _unit(source, '  void publishState()', '  void initializeRunLog()')
    assert '"pick_stack_auto"' in state
    assert 'auto_stack_active' in state
    assert 'auto_stack_order_top_to_bottom' in state


def test_shipped_config_allows_the_full_four_colour_tower():
    """
    The fixed order has four colours, so the shipped limit must reach them.

    A lower maximum_stack_cubes would make the automatic stack refuse a table
    that holds more colours than the tower may contain, so the command would be
    unusable for exactly the layout it is meant for.
    """
    yaml = pytest.importorskip('yaml')
    config = next(iter(yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'first_grasp.yaml')
        .read_text(encoding='utf-8')).values()))['ros__parameters']
    assert int(config['maximum_stack_cubes']) >= 4
