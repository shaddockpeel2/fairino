"""Return publisher counts; topic names can also come from subscribers alone."""
import os
import sys
import time
# Fast DDS writes native diagnostics to stdout too. Keep the four-count protocol clean.
counts_output = os.fdopen(os.dup(sys.stdout.fileno()), 'w')
os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('follow_graph_probe_' + str(os.getpid()))
try:
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    topics = ['/camera/color/image_raw', '/target_follow/tag_pose',
              '/target_follow/flange_pose', '/target_follow/preview_state']
    print(' '.join(str(node.count_publishers(topic)) for topic in topics), file=counts_output, flush=True)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    counts_output.close()
