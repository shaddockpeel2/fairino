"""Discover sources once before the Demo takes ownership of its own processes."""
import os
import time

TOPICS = {
    'camera': '/camera/color/image_raw',
    'detector': '/target_follow/tag_pose',
    'flange': '/target_follow/flange_pose',
    'feedback': '/target_follow/robot_feedback',
    'preview': '/target_follow/preview_state',
    'base_target': '/target_follow/tag_pose_base',
    'follow': '/target_follow/follow_state',
    'servo': '/target_follow/servo_state',
}


def discover():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    context = Context()
    rclpy.init(args=[], context=context)
    node = Node('follow_demo_probe_' + str(os.getpid()), context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 2.0
        while context.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        return {
            'publishers': {key: node.count_publishers(topic) for key, topic in TOPICS.items()},
            'nodes': [name for name, namespace in node.get_node_names_and_namespaces()
                      if namespace == '/'],
        }
    finally:
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


def source_plan(graph, mock=False, viewer=True):
    counts = graph['publishers']
    occupied = [TOPICS[key] for key in ('flange', 'feedback', 'preview', 'base_target', 'follow', 'servo')
                if counts.get(key, 0)]
    if occupied:
        raise RuntimeError('Close the existing feedback/preview/follow/servo launch first; '
                           'the Demo supplies its own SDK owner and base conversion. Found: '
                           + ', '.join(occupied))
    for key in ('camera', 'detector'):
        if counts.get(key, 0) > 1:
            raise RuntimeError('Multiple publishers on ' + TOPICS[key] + '; keep one source before starting.')
    return {
        'start_camera': not mock and counts.get('camera', 0) == 0,
        'start_detector': counts.get('detector', 0) == 0,
        'start_viewer': viewer and 'target_pose_viewer' not in graph.get('nodes', []),
    }
