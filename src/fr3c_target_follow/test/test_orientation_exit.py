"""ROS teardown errors are ignored only when the context is shutting down."""
import pytest

from fr3c_target_follow import orientation_node


@pytest.mark.parametrize('shutting_down', [True, False])
def test_runtime_error_handling_preserves_errors_in_running_context(monkeypatch, shutting_down):
    cleanup = []
    class FakeNode:
        def destroy_node(self):
            cleanup.append('node')
    monkeypatch.setattr(orientation_node, 'OrientationCheck', FakeNode)
    monkeypatch.setattr(orientation_node.rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(orientation_node.rclpy, 'ok', lambda: not shutting_down)
    monkeypatch.setattr(orientation_node.rclpy, 'shutdown', lambda: cleanup.append('context'))
    def spin(node):
        raise RuntimeError('subscription conversion failed')
    monkeypatch.setattr(orientation_node.rclpy, 'spin', spin)
    if shutting_down:
        orientation_node.main([])
        assert cleanup == ['node']
    else:
        with pytest.raises(RuntimeError, match='subscription conversion failed'):
            orientation_node.main([])
        assert cleanup == ['node', 'context']
