from pathlib import Path


SOURCE = (Path(__file__).parents[1] / 'src/first_grasp_node.cpp').read_text()


def _unit(start, end):
    return SOURCE[SOURCE.index(start):SOURCE.index(end, SOURCE.index(start))]


def test_manual_xyz_action_is_bounded_and_collision_checked_before_execution():
    validator = _unit(
        '  bool manualPoseTargetValid(',
        '  void executeMoveToPoseAction(')
    execute = _unit(
        '  void executeMoveToPoseAction(',
        '  rclcpp_action::Server<PickColor>::SharedPtr createPickServer(')
    for bounds in ('manual_pose_x_bounds_', 'manual_pose_y_bounds_',
                   'manual_pose_z_bounds_'):
        assert bounds in validator
    assert execute.index('manualPoseTargetValid(') < execute.index(
        'planPoseFromSeededIk(')
    assert execute.index('groundPlaneObject()') < execute.index(
        'planPoseFromSeededIk(')
    assert execute.index('planPoseFromSeededIk(') < execute.index(
        'executeMotion(')
    assert 'publishPreview({motion})' in execute
    assert 'manual_xyz_collision_check_passed' in execute
    assert 'cubeIsStillAttached()' in execute
    assert 'getCurrentPose(tip_frame_).pose' in execute
    assert 'target.position.x =' in execute
    assert 'target.orientation' not in execute


def test_manual_xyz_action_shares_the_arm_owner_and_supports_cancellation():
    server = _unit(
        '  rclcpp_action::Server<MoveToPose>::SharedPtr createMoveToPoseServer(',
        '  void publishMoveToPoseFeedback(')
    assert 'action_goal_claimed_.exchange(true)' in server
    assert 'auto_stack_active_.load()' in server
    assert 'canceled_.store(true)' in server
    assert 'move_group_->stop()' in server
    finish = _unit('  void finishMoveToPoseAction(', '  bool manualPoseTargetValid(')
    assert 'action_goal_claimed_.store(false)' in finish
    assert 'previous task state preserved' in finish
