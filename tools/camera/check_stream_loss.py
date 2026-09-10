#!/usr/bin/env python3
"""Explicit camera-process stop test. Never accepts a robot process."""
import argparse,json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from diagnostic_msgs.msg import DiagnosticArray
p=argparse.ArgumentParser();p.add_argument('--camera-pid',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args()
cmd=Path(f'/proc/{a.camera_pid}/cmdline').read_bytes().replace(b'\0',b' ')
if b'/rclcpp_components/component_container' not in cmd or b'/camera' not in cmd:
    raise SystemExit('Refusing to stop a process other than the camera component container')
rclpy.init();n=Node('fr3c_camera_stream_loss_test');points=[];diagnostics=[]
n.create_subscription(PointStamped,'/fr3c_camera/probe_point',lambda m:points.append(time.monotonic()),10)
def diagnostic_level(value):
    return value[0] if isinstance(value, bytes) else int(value)
n.create_subscription(DiagnosticArray,'/fr3c_camera/diagnostics',lambda m:diagnostics.append((time.monotonic(),diagnostic_level(m.status[0].level),m.status[0].message)),10)
end=time.monotonic()+15
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
before=len(points);stopped=time.monotonic();os.kill(a.camera_pid,signal.SIGSTOP)
end=stopped+5
try:
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
finally:
    os.kill(a.camera_pid,signal.SIGCONT)
late=[t for t in points if t>stopped+1.]
warn=[d for d in diagnostics if d[0]>stopped+1. and d[1]>0 and 'no_fresh_valid_point' in d[2]]
result={'pass':before>0 and not late and bool(warn),'points_before_stop':before,'points_after_1s':len(late),'stale_diagnostics':len(warn),'last_status':diagnostics[-1][2] if diagnostics else None}
Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown();raise SystemExit(0 if result['pass'] else 1)
