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
    'quality': '/target_follow/quality',
    'orientation': '/target_follow/orientation_state',
    'pose_preview': '/target_follow/pose_preview/state',
    'pose_observation': '/target_follow/pose_observation',
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


def orientation_source_plan(graph, start_sources=True):
    """Stage 6 owns no actuator and reuses one existing read-only SDK owner."""
    counts = graph['publishers']
    for key, topic in TOPICS.items():
        if counts.get(key, 0) > 1:
            raise RuntimeError('Multiple publishers on '+topic+'; keep one source.')
    if counts.get('follow', 0) or counts.get('servo', 0):
        raise RuntimeError('Close the motion-capable follow/servo launch before this read-only check.')
    if counts.get('orientation', 0):
        raise RuntimeError('An orientation check is already running in this ROS domain.')
    if counts.get('pose_preview', 0):
        raise RuntimeError('A pose preview is already running in this ROS domain.')
    if bool(counts.get('flange', 0)) != bool(counts.get('feedback', 0)):
        raise RuntimeError('Incomplete robot feedback source; flange and feedback must have one owner.')
    if counts.get('base_target', 0) and not counts.get('flange', 0):
        raise RuntimeError('Base conversion has no robot feedback source.')
    if not start_sources:
        return dict(start_camera=False, start_detector=False, start_feedback=False, start_base=False)
    return dict(start_camera=not counts.get('camera', 0), start_detector=not counts.get('detector', 0),
                start_feedback=not counts.get('flange', 0), start_base=not counts.get('base_target', 0))


def pose_follow_source_plan(graph, mock=False, viewer=True):
    plan = source_plan(graph, mock=mock, viewer=viewer)
    counts = graph['publishers']
    for key in ('quality', 'pose_observation', 'orientation', 'pose_preview'):
        if counts.get(key, 0) > (1 if key == 'quality' else 0):
            raise RuntimeError('Close the existing pose preview/orientation pipeline or duplicate quality source: '+TOPICS[key])
    return plan
