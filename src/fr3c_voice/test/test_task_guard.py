import pytest

from fr3c_voice.task_guard import TaskDispatchGuard


def test_guard_rejects_duplicate_until_terminal_result():
    guard = TaskDispatchGuard()
    first = guard.begin('one', 'pick_hold', 'red')
    assert first.active
    with pytest.raises(RuntimeError):
        guard.begin('two', 'pick_sort', 'blue')
    guard.update('one', 'accepted', 'server accepted')
    assert guard.snapshot().active
    guard.finish('one', True, False, 'actual result received')
    assert not guard.snapshot().active
    second = guard.begin('two', 'pick_sort', 'blue')
    assert second.request_id == 'two'


def test_stale_callbacks_cannot_overwrite_current_task():
    guard = TaskDispatchGuard()
    guard.begin('current', 'pick_hold', 'green')
    guard.update('old', 'done', 'stale callback')
    assert guard.snapshot().request_id == 'current'
    assert guard.snapshot().phase == 'submitting'
