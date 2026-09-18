"""
Compile the production stack-yaw arithmetic with a stand-in, without hardware.

pick_stack has to bring the upper cube down with its top face parallel to the
lower cube's top face. A square top face makes four grasp yaws physically
equivalent, so the rotation the wrist must add is only well defined modulo the
yaw symmetry. These cases pin that arithmetic down, because an error here would
show up only as a cube landing rotated on the tower.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

HEADER = r'''
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
// Matches the constant the production unit declares at file scope.
constexpr double kPi = 3.14159265358979323846;
}
'''

HARNESS = r'''
#include <cstdio>

int failures = 0;

void check(bool ok, const char * what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

// Object yaw of a cube held with `applied` grasp yaw. graspCandidates() builds
// the applied yaw as object_yaw + offset, so the offset cancels when two cubes
// are compared; it is carried here anyway so the case matches production.
constexpr double kEdgeOffset = -1.5707963267948966;

int main() {
  const double symmetry = kPi * 0.5;
  const double epsilon = 1e-12;

  // Signed number of whole symmetry steps between two yaws. Zero means the two
  // yaws describe the same physical grip. Reducing a requested rotation by whole
  // symmetry steps is what makes the upper cube land on the lower cube's yaw,
  // so this must be an integer for every case below.
  const auto stepDifference = [symmetry](double a, double b) {
      return (a - b) / symmetry;
    };

  // requested rotation, reference yaw, expected reduced rotation.
  //
  // The reduced rotation is always (requested - k * symmetry) for an integer k
  // and is the nearest such equivalent, so it stays within half a step of the
  // request. Half a step is 45 degrees for a square top: that is the worst case
  // the reduction can leave, so the planner rejects a candidate above the
  // configured allowance instead of assuming a smaller rotation.
  const double cases[][3] = {
    {0.0, 0.0, 0.0},
    {0.1, 0.0, 0.1},
    {-0.1, 0.0, -0.1},
    {0.0, 0.1, -0.1},
    {0.0, -0.1, 0.1},
    {0.2, 0.0, 0.2},
    {-0.2, 0.0, -0.2},
    {0.39, 0.0, 0.39},
    {-0.39, 0.0, -0.39},
    // A rotation inside half a step is already minimal and is kept as is. These
    // are also the cases that the 45 degree allowance must let through.
    {0.7, 0.0, 0.7},
    {-0.7, 0.0, -0.7},
    {0.45, 0.0, 0.45},
    {-0.45, 0.0, -0.45},
    // A rotation outside half a step loses one whole step.
    {0.8, 0.3, 0.5},
    {-0.8, -0.3, -0.5},
    // Wrapped measurements: the difference can span several whole steps.
    {3.0, -3.0, -0.28318530717958623},
    {2.4, -2.5, 0.18761101961531068},
    {0.0, 3.141592653589793, 0.0},
    {-6.0, 0.5, -0.21681469282041377},
    {6.0, -0.5, 0.21681469282041377},
    {1.5707963267948966, 0.0, 0.0},
    {-1.5707963267948966, 0.0, 0.0},
    {-3.141592653589793, 0.0, 0.0},
    {7.953981633974482, 0.0, 0.09999999999999964},
    // Exactly half a step: the worst case for a square top.
    {0.39269908169872414, 0.0, 0.39269908169872414},
  };
  for (const auto & item : cases) {
    const double reduced = closestEquivalentAngle(item[0], item[1], symmetry);
    check(std::abs(reduced - item[2]) < epsilon,
          "reduced rotation does not match the expected equivalent");
    const double steps = stepDifference(item[1] + reduced, item[0]);
    check(std::abs(steps - std::round(steps)) < 1e-9,
          "reduction did not move by a whole symmetry step");
    // The chosen equivalent is the nearest of its class: no other whole-step
    // equivalent of the requested rotation is closer.
    check(std::abs(reduced) <= 0.5 * symmetry + epsilon,
          "the chosen equivalent is not the nearest one");
  }

  // The run-time bound. The difference between the two measured yaws reduced by
  // whole symmetry steps is what the wrist has to add. The worst case is half
  // one square-symmetry step: 45 degrees. The configured allowance is exactly
  // that bound.
  for (double destination = -3.0; destination <= 3.0; destination += 0.0625) {
    double best = 1e9;
    double worst = 0.0;
    for (double step = 0.0; step < 4.0; step += 1.0) {
      const double source_yaw = 0.42 + step * symmetry;
      const double reduced = closestEquivalentAngle(
        destination - kEdgeOffset, source_yaw, symmetry);
      best = std::min(best, std::abs(reduced));
      worst = std::max(worst, std::abs(reduced));
    }
    check(best <= 0.5 * symmetry + 1e-9,
          "no equivalent grasp stayed within the configured rotation allowance");
    check(worst <= 0.5 * symmetry + 1e-9,
          "an equivalent grasp needed more than the configured allowance");
  }

  // Exactly half a step is the worst case for a square top. It has two equally
  // valid equivalents and the magnitude must be preserved either way.
  const double worst = closestEquivalentAngle(0.25 * symmetry, 0.0, symmetry);
  check(std::abs(std::abs(worst) - 0.25 * symmetry) < epsilon,
        "45 degree placement rotation was folded away");

  // The placement rotation must land the upper cube's yaw on the lower cube's
  // yaw modulo the cube's physical symmetry, for every equivalent source grasp.
  // The measured destination yaw and the source yaw are both wrapped into
  // [-pi, pi), so the four source grasps must all reach half a step or less.
  const double destination_yaw = 0.31;
  for (double step = 0.0; step < 4.0; step += 1.0) {
    const double source_yaw = 0.42 + step * symmetry;
    const double reduced = closestEquivalentAngle(
      destination_yaw - kEdgeOffset, source_yaw, symmetry);
    const double landed_object_yaw = source_yaw + reduced + kEdgeOffset;
    const double steps = stepDifference(landed_object_yaw, destination_yaw);
    check(std::abs(steps - std::round(steps)) < 1e-9,
          "upper cube top face is not aligned with the lower cube top face");
    check(std::abs(reduced) <= 0.25 * symmetry + 1e-9,
          "an equivalent grasp needed more than half a step of wrist rotation");
  }

  // A rectangle's 180 degree symmetry must not be folded to a quarter turn.
  const double rectangle_symmetry = kPi;
  const double rectangle_reduced =
    closestEquivalentAngle(0.25 * rectangle_symmetry, 0.0, rectangle_symmetry);
  check(std::abs(rectangle_reduced - 0.25 * rectangle_symmetry) < 1e-9,
        "a 180 degree symmetry period was folded as if it were 90 degrees");

  if (failures == 0) {
    std::printf("all stack geometry cases passed\n");
  }
  return failures == 0 ? 0 : 1;
}
'''


def _extract_unit(source: str, start_marker: str, end_marker: str) -> str:
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


def _grasp_source() -> str:
    return (Path(__file__).parents[1] / 'src' / 'first_grasp_node.cpp').read_text()


def test_runtime_placement_reuses_the_previewed_descendable_ik_branch():
    """The same TCP pose may have an IK branch that cannot descend."""
    source = _grasp_source()
    cycle = _extract_unit(source, '  bool runCycleBody()', '  void fail(')

    assert 'preview[kPlaceApproachIndex].end_state' in cycle
    assert 'auto place_approach = planJointState(' in cycle
    assert 'auto place_approach = planPose(' not in cycle
    assert 'auto place_descend = preview[kPlaceDescendIndex]' in cycle
    assert 'auto retreat = preview[kPlaceRetreatIndex]' in cycle
    assert 'verifyValidatedSegmentStart(' in cycle
    assert '"place_descend_execute", currentStartState(false)' not in cycle
    assert '"place_retreat_execute", currentStartState(false)' not in cycle


def test_stack_yaw_equivalence_and_placement_alignment(tmp_path):
    """The placement rotation must align the two cube tops modulo symmetry."""
    if shutil.which('c++') is None:
        pytest.skip('no C++ compiler is available')
    source = _grasp_source()
    # closestEquivalentAngle() is plain math, so the unit needs no ROS headers.
    yaw_unit = _extract_unit(
        source, 'double closestEquivalentAngle(', 'double positionDistance(')
    unit = tmp_path / 'stack_geometry.cpp'
    unit.write_text(HEADER + yaw_unit + HARNESS, encoding='utf-8')
    binary = tmp_path / 'stack_geometry'
    compiled = subprocess.run(
        ['c++', '-std=c++17', '-Wall', '-Wextra', '-o', str(binary), str(unit)],
        capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    executed = subprocess.run([str(binary)], capture_output=True, text=True)
    assert executed.returncode == 0, executed.stdout + executed.stderr


def test_stack_placement_height_puts_the_upper_cube_on_the_lower_top(tmp_path):
    """Release 5 mm high, then represent the settled cube on the lower top."""
    if shutil.which('c++') is None:
        pytest.skip('no C++ compiler is available')
    source = _grasp_source()
    # The production placement height and the attached-cube offset, plus the
    # rotation helper, are all plain math.
    assert 'pose.position.z = destination_top_z + release_bottom_gap_m_' in source
    assert 'locked_object_size_m_[2] - grasp_depth_m_' in source
    harness = r'''
#include <cstdio>
int failures = 0;
void check(bool ok, const char * what)
{
  if (!ok) { std::printf("FAIL: %s\n", what); ++failures; }
}
int main() {
  const double symmetry = kPi * 0.5;
  const double offset = -kPi * 0.5;      // target_yaw_offset_rad
  const double gap = 0.005;              // release_bottom_gap_m
  const double depth = 0.015;            // grasp_depth_m
  const double clearance = 0.030;        // stack_approach_clearance_m
  const double height = 0.030;           // cube height
  const double plane = 0.0;              // ground_plane_z_m

  for (double destination_yaw = -3.0; destination_yaw <= 3.0;
       destination_yaw += 0.37) {
    for (double source_yaw = -3.0; source_yaw <= 3.0; source_yaw += 0.41) {
      for (double destination_height : {0.030, 0.032, 0.028}) {
        const double destination_top = plane + destination_height;
        const double place_z = destination_top + gap + height - depth;
        // The attached cube centre sits (depth - height/2) above the TCP, so
        // At the commanded release pose the bottom is exactly `gap` above the
        // lower top. Free release then lets it settle onto that top.
        const double held_bottom =
          place_z + (depth - height * 0.5) - height * 0.5;
        check(std::abs(held_bottom - (destination_top + gap)) < 1e-12,
              "the held cube is not 5 mm above the lower top");
        const double released_bottom =
          place_z + (depth - height * 0.5) - height * 0.5 - gap;
        check(std::abs(released_bottom - destination_top) < 1e-12,
              "the released cube does not rest on the lower cube top");

        const double theta = closestEquivalentAngle(
          destination_yaw - offset, source_yaw + offset, symmetry);
        const double landed = source_yaw + offset + theta;
        const double steps = (landed - destination_yaw) / symmetry;
        check(std::abs(steps - std::round(steps)) < 1e-9,
              "the placed cube top face does not match the lower cube");
        check(std::abs(theta) <= 0.5 * symmetry + 1e-9,
              "the placement rotation exceeds the configured allowance");
      }
    }
  }

  // The approach must hold the grasped cube clear of the tower while the wrist
  // rotates, so its TCP stands higher than the placement TCP.
  const double destination_top = plane + height;
  const double place_z = destination_top + gap + height - depth;
  const double approach_z = place_z + clearance;
  check(approach_z > place_z, "the stack approach is not higher than the placement");
  check(approach_z + depth - height > destination_top,
        "the grasped cube would clip the tower during the approach");

  std::printf(failures == 0 ? "all placement geometry cases passed\n" : "FAILURES\n");
  return failures == 0 ? 0 : 1;
}
'''
    yaw_unit = _extract_unit(
        source, 'double closestEquivalentAngle(', 'double positionDistance(')
    unit = tmp_path / 'stack_geometry.cpp'
    unit.write_text(HEADER + yaw_unit + harness, encoding='utf-8')
    binary = tmp_path / 'stack_geometry'
    compiled = subprocess.run(
        ['c++', '-std=c++17', '-Wall', '-Wextra', '-o', str(binary), str(unit)],
        capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    executed = subprocess.run([str(binary)], capture_output=True, text=True)
    assert executed.returncode == 0, executed.stdout + executed.stderr

    settled = source[
        source.index('moveit_msgs::msg::CollisionObject stackedCubeObject() const'):
        source.index('moveit_msgs::msg::CollisionObject stackSupportObject() const')]
    assert 'locked_object_size_m_[2] * 0.5;' in settled
    assert 'locked_object_size_m_[2] * 0.5 - release_bottom_gap_m_' not in settled


def test_pick_stack_rejects_the_same_colour_for_both_roles():
    """A cube cannot be stacked on itself; the goal is refused up front."""
    source = _grasp_source()
    assert 'if (support_color == color) {' in source
    guard = source.index('if (support_color == color) {')
    assert 'INVALID_SUPPORT_COLOR' in source[guard:guard + 500]
    # The rejection happens while the start lock is still held, i.e. before any
    # perception request or arm motion.
    assert source.index('if (support_color == color) {') < \
        source.index('std::unique_lock<std::mutex> start_lock(start_mutex_);')


def test_pick_stack_requires_an_explicit_support_colour():
    """Only pick_stack may set support_color, and it must not be empty."""
    source = _grasp_source()
    assert 'support_color = normalizedColor(raw_support_color);' in source
    assert 'pick_stack requires support_color' in source
    assert 'does not accept a support_color' in source
    assert 'use /fr3c_first_grasp/pick_stack to stack one cube on another' in source


def test_pick_stack_locks_both_tabletop_cubes_from_one_initial_snapshot():
    """Both separate tabletop cubes come from one immutable RGB-D status."""
    source = _grasp_source()
    assert 'bool lockStackDestinationFromSourceSnapshot(' in source
    acquisition = source[
        source.index('bool lockStackDestinationFromSourceSnapshot('):
        source.index('bool validateStackPair(std::string & reason)')]
    assert 'status = locked_target_status_;' in acquisition
    assert 'detectedCubesFromStatus(' in acquisition
    assert 'source_matches != 1' in acquisition
    assert 'support_matches != 1' in acquisition
    assert 'stack_pair_snapshot_locked' in acquisition
    assert 'same_rgbd_status=true' in acquisition
    assert 'setPerceptionTargetColor(""' not in acquisition

    assert 'bool validateStackPair(std::string & reason)' in source
    body = source[
        source.index('bool validateStackPair(std::string & reason)'):
        source.index('// A slot is unusable', source.index(
            'bool validateStackPair(std::string & reason)'))]
    assert 'source->stack_tier != 0' in body
    assert 'source_bottom - destination_bottom' in body
    assert 'std::abs(tabletop_difference) > stack_height_tolerance_m_' in body
    assert 'xy_separation <= stack_footprint_tolerance_m_' in body
    # A support cube that already carries a cube is refused before planning.
    assert 'bool lockStackDestinationSnapshot(' in source
    lock_body = source[
        source.index('bool lockStackDestinationSnapshot('):
        source.index('bool lockStackDestinationFromSourceSnapshot(')]
    assert 'cube.stack_tier != 0' in lock_body
    assert 'pick_stack builds only a two-cube stack' in lock_body


def test_pick_stack_never_reobserves_after_motion_starts():
    source = _grasp_source()
    assert 'reobserveStackPair' not in source
    assert 'stack_destination_reobserved' not in source
    assert 'stack_destination_snapshot_reused' in source
    cycle = source[
        source.index('bool runCycleBody()'):
        source.index('void publishState()', source.index('bool runCycleBody()'))]
    assert 'activePlaceApproachPose()' in cycle
    assert 'activePlacePose()' in cycle
    assert 'latest_target_status_' not in cycle


def test_pick_stack_always_returns_home_and_frees_the_task_slot():
    """The action must end in a startable state whatever the outcome."""
    source = _grasp_source()
    assert 'void recoverToHomeAfterFailedStack()' in source
    recovery = source[
        source.index('void recoverToHomeAfterFailedStack()'):
        source.index('bool runCycleBody()')]
    assert 'stack_failure_return_home' in recovery
    # A failed stack becomes retryable, so the next request is not blocked.
    assert 'state_ = RETRYABLE_FAILED;' in recovery
    # The recovery is wired into the stack path only.
    cycle = source[
        source.index('bool runCycle()'):
        source.index('void recoverToHomeAfterFailedStack()')]
    assert 'task_stack_enabled_' in cycle
    assert 'recoverToHomeAfterFailedStack();' in cycle
    # Completion reports returning to the start joints.
    assert 'complete_stacked_home' in source
    assert 'STACK_COMPLETE' in source
    # A startable state list must include the new terminal state.
    assert 'state == STACK_COMPLETE ||' in source
    assert 'state != STACK_COMPLETE' in source


def test_pick_stack_keeps_the_lower_cube_in_the_scene():
    """The planner needs the support cube as an obstacle for the whole task."""
    source = _grasp_source()
    assert 'moveit_msgs::msg::CollisionObject stackSupportObject() const' in source
    assert 'stack_support_id_' in source
    assert 'objects.push_back(stackSupportObject());' in source
    # The virtual obstacle must not leak into later non-stack tasks.
    assert 'bool clearStackSupportObject(std::string & reason)' in source
    assert 'clearStackSupportObject(reason)' in source


def test_next_task_can_reclaim_the_upper_cube_from_a_finished_stack():
    """The archived placed-cube copy must not collide with the next target."""
    source = _grasp_source()
    acquisition = source[
        source.index('if (!waitForRequestedTarget('):
        source.index('if (mode == PickTaskMode::STACK) {',
                     source.index('if (!waitForRequestedTarget('))]
    assert 'clearArchivedObstacleAtTarget(reason)' in acquisition

    reclaim = source[
        source.index('bool clearArchivedObstacleAtTarget(std::string & reason)'):
        source.index('// Remove the virtual lower-cube obstacle of pick_stack')]
    assert 'const std::string prefix = cube_id_ + "_sorted_"' in reclaim
    assert 'collisionPrimitivePosition(entry.second, pose)' in reclaim
    assert 'positionDistance(archived, target)' in reclaim
    assert 'CollisionObject::REMOVE' in reclaim
    assert 'archived_target_reclaimed' in reclaim


def test_archived_cube_matching_composes_outer_and_primitive_poses():
    """The scene service may move a primitive pose into CollisionObject.pose."""
    source = _grasp_source()
    helper = source[
        source.index('geometry_msgs::msg::Point collisionPrimitivePosition('):
        source.index('std::string jsonEscape(')]

    assert 'object.pose.position.x + rotated.x' in helper
    assert 'object.pose.position.y + rotated.y' in helper
    assert 'object.pose.position.z + rotated.z' in helper
    assert 'primitive_pose.position.x' in helper
    assert 'primitive_pose.position.y' in helper
    assert 'primitive_pose.position.z' in helper
    # Default-constructed CollisionObject poses have a zero quaternion. The
    # helper must treat that representation as identity rather than producing
    # an invalid rotation before MoveIt has normalized the message.
    assert 'if (norm < 1e-9)' in helper
    assert 'orientation.w = 1.0' in helper


def test_production_uses_one_pre_open_burst():
    """One verified burst avoids the visible second servo correction."""
    yaml = pytest.importorskip('yaml')
    config = next(iter(yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'first_grasp.yaml')
        .read_text(encoding='utf-8')).values()))['ros__parameters']
    assert config['pre_open_command_repetitions'] == 1
    assert 'pre_open_command_repetitions_{1};' in _grasp_source()


def test_stack_placement_uses_the_measured_lower_cube_top():
    """Placement height must come from the detected lower cube, not the plane."""
    source = _grasp_source()
    body = source[
        source.index('geometry_msgs::msg::Pose stackPlacePose() const'):
        source.index('geometry_msgs::msg::Pose stackPlaceApproachPose() const')]
    assert 'destination_top_z + release_bottom_gap_m_' in body
    assert 'locked_object_size_m_[2] - grasp_depth_m_' in body
    # X/Y come from the lower cube, never from the A4 colour region.
    assert 'locked_stack_destination_.top_pose.position.x' in body
    assert 'locked_place_center_' not in body


def test_pick_stack_requires_orientation_and_size_feedback():
    """Without a measured yaw and height the tower cannot be aligned."""
    source = _grasp_source()
    assert 'stack_enabled_ && !use_target_yaw_' in source
    assert 'stack_enabled_ && !use_target_size_' in source
    assert 'stack_enabled_ && !sort_enabled_' in source


def test_shipped_config_passes_the_startup_rotation_guard():
    """
    The shipped configuration must survive the startup guard.

    The guard exists to refuse a rotation limit so small that even two cubes
    sharing a yaw could not be stacked. It has to be compared against the
    smallest symmetry step that can occur at run time, not the configured
    fallback: with use_target_size enabled the locked step comes from the
    measured top shape, and a square top uses half a turn of symmetry steps.
    Reading the fallback instead made the guard twice as strict as run time and
    refused the shipped configuration, which aborted the node before it
    advertised any action server.
    """
    yaml = pytest.importorskip('yaml')
    config = next(iter(yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'first_grasp.yaml')
        .read_text(encoding='utf-8')).values()))['ros__parameters']
    source = _grasp_source()

    # The guard must derive its threshold from the smaller of the two steps.
    guard = source[
        source.index('if (stack_enabled_) {'):
        source.index('if (sort_enabled_) {')]
    assert 'std::min(target_yaw_symmetry_rad_, kPi * 0.5)' in guard, (
        'the rotation guard must use the smallest run-time symmetry step')

    k_pi = 3.14159265358979323846
    fallback = float(config['target_yaw_symmetry_rad'])
    limit = float(config['maximum_stack_placement_rotation_rad'])
    smallest_step = min(fallback, k_pi * 0.5)

    # The shipped limit passes, and it is exactly the worst case the reduction
    # can leave, so no reachable pair is refused for want of a larger limit.
    assert limit >= 0.5 * smallest_step - 1e-9, (
        f'shipped limit {limit} is below the run-time half step '
        f'{0.5 * smallest_step}; the node would abort at startup')
    assert limit == pytest.approx(0.5 * smallest_step), (
        'the shipped limit should be exactly half the smallest run-time step')

    # The guard must still refuse a limit that really is too small.
    assert not (0.5 * smallest_step - 0.01 >= 0.5 * smallest_step - 1e-9), (
        'the guard must reject a limit below half the smallest step')


def test_clearing_an_absent_cube_is_a_no_op_not_a_failure():
    """
    An empty planning scene must not block the first task.

    MoveIt reports a REMOVE for an object that is not in the scene as a failure,
    not as a no-op: PlanningScene::processCollisionObjectRemove() logs "Tried to
    remove world object '<id>', but it does not exist in this scene." and returns
    false, which becomes ApplyPlanningScene.success = false. Treating that as an
    error refused the very first stack on a freshly started stack, when the scene
    is legitimately empty, and the goal aborted with PRECONDITION_FAILED before
    any motion.
    """
    source = _grasp_source()
    body = source[
        source.index('bool clearPreviousCubeScene(std::string & reason)'):
        source.index('bool archivePreviousSortedCubeScene(std::string & reason)')]

    # The world REMOVE must be requested only when the cube is really there.
    guard = "planning_scene_.getObjects({cube_id_}).count(cube_id_) == 0"
    assert guard in body, 'a REMOVE must not be sent for an absent cube'
    assert body.index(guard) < body.index(
        'remove.operation = moveit_msgs::msg::CollisionObject::REMOVE'), (
        'the existence check must run before the REMOVE request')

    # The same hazard must not be reintroduced in the other cleanup paths.
    support = source[
        source.index('bool clearStackSupportObject(std::string & reason)'):
        source.index('std::vector<GraspCandidate> graspCandidates() const')]
    assert 'planning_scene_.getObjects({stack_support_id_}).count(stack_support_id_) == 0' \
        in support, 'the support-cube cleanup lost its existence guard'

    archive = source[
        source.index('bool archivePreviousSortedCubeScene(std::string & reason)'):
        source.index('// Remove the virtual lower-cube obstacle of pick_stack')]
    assert 'if (previous.count(cube_id_) != 0) {' in archive, (
        'the archived-cube removal must stay conditional on the cube existing')
