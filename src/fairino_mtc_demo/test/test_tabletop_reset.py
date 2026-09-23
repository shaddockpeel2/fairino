from pathlib import Path


def test_reset_is_serialized_and_preserves_unrelated_obstacles_and_held_objects():
    source = (Path(__file__).parents[1] / 'src/first_grasp_node.cpp').read_text()
    reset = source.split('  void resetTabletopObjects(', 1)[1].split(
        '  void acknowledgeObjectReleased(', 1)[0]
    assert 'start_lock(start_mutex_)' in reset
    for gate in ('worker_running_.load()', 'action_goal_claimed_.load()',
                 'auto_stack_active_.load()', 'state_ == COMPLETE',
                 'getAttachedObjects({cube_id_})'):
        assert reset.index(gate) < reset.index('applyCollisionObjects(removals)')
    for object_filter in ('entry.first != cube_id_', 'entry.first != stack_support_id_',
                          'entry.first != stackColumnId()', 'entry.first.rfind(prefix, 0) != 0'):
        assert object_filter in reset
    assert 'getObjects(ids).empty()' in reset
    for cache in ('place_slot_use_count_', 'auto_stack_inventory_', 'ignored_batch_targets_'):
        assert f'{cache}.clear()' in reset
    assert 'setState(WAITING_TARGET' in reset
    assert 'executeMotion(' not in reset
    assert 'applyAttachedCollisionObject(' not in reset
