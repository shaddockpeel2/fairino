"""Bounded audit of read-only robot feedback and base-frame following preview."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from .audit import percentiles


class PreviewAudit(Node):
    def __init__(self, output):
        super().__init__('follow_preview_audit')
        self.output=output
        self.robot=[];self.states=[];self.flange=[];self.targets=[]
        self.stream=(output/'samples.jsonl').open('w')
        self.create_subscription(String,'/target_follow/robot_feedback',self.robot_state,50)
        self.create_subscription(String,'/target_follow/preview_state',self.preview_state,50)
        self.create_subscription(PoseStamped,'/target_follow/flange_pose',lambda m:self.pose('flange',m,self.flange),50)
        self.create_subscription(PoseStamped,'/target_follow/tag_pose_base',lambda m:self.pose('target',m,self.targets),10)

    def pose(self,kind,message,collection):
        collection.append(message)
        p,q=message.pose.position,message.pose.orientation
        self.save(kind,dict(stamp_s=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                            frame_id=message.header.frame_id,position_m=[p.x,p.y,p.z],
                            quaternion_xyzw=[q.x,q.y,q.z,q.w]))

    def save(self,kind,row):
        row['audit_receive_s']=time.monotonic()
        self.stream.write(json.dumps(dict(kind=kind,**row),allow_nan=False)+'\n')

    def robot_state(self,message):
        row=json.loads(message.data);self.save('robot',row);self.robot.append(row)

    def preview_state(self,message):
        row=json.loads(message.data);self.save('preview',row);self.states.append(row)

    def report(self,duration):
        robot=[r for r in self.robot if r['status']=='valid']
        preview=[r for r in self.states if r['status']=='preview']
        def hz(rows):
            return (len(rows)-1)/(rows[-1]['audit_receive_s']-rows[0]['audit_receive_s']) if len(rows)>1 else 0
        def positions(messages):
            return np.array([[m.pose.position.x,m.pose.position.y,m.pose.position.z] for m in messages])
        report=dict(duration_s=duration,robot_packets=len(robot),robot_feedback_fps=hz(robot),
                    flange_samples=len(self.flange),base_target_samples=len(self.targets),
                    preview_samples=len(preview),preview_fps=hz(preview),
                    robot_status_counts=dict(Counter(r['status'] for r in self.robot)),
                    preview_status_counts=dict(Counter(r['status'] for r in self.states)),
                    sdk_call_ms=percentiles([r['call_ms'] for r in robot]),
                    motion_commands_enabled=any(r['motion_commands_enabled'] for r in self.states+self.robot),
                    orientation_control_enabled=any(r['orientation_control_enabled'] for r in self.states))
        stamps=[m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in self.flange]
        report['feedback_intervals_ms']=percentiles((np.diff(stamps)*1000).tolist())
        report['feedback_timestamp_regressions']=int(np.sum(np.diff(stamps)<=0))
        if robot:report['latest_robot']=robot[-1]
        if self.targets:
            xyz=positions(self.targets);centre=np.median(xyz,axis=0)
            report['median_target_base_mm']=(centre*1000).tolist()
            report['target_base_deviation_mm']=percentiles((np.linalg.norm(xyz-centre,axis=1)*1000).tolist())
            report['target_axis_span_mm']=((xyz.max(axis=0)-xyz.min(axis=0))*1000).tolist()
        if self.flange:
            xyz=positions(self.flange)
            report['robot_flange_axis_span_mm']=((xyz.max(axis=0)-xyz.min(axis=0))*1000).tolist()
        for key in ('target_age_ms','interpolation_gap_ms','error_norm_mm'):
            report[key]=percentiles([r[key] for r in preview])
        report['preview_speed_mm_s']=percentiles([float(np.linalg.norm(r['velocity_m_s'])*1000) for r in preview])
        report['nonpreview_nonzero_velocity_samples']=int(sum(r['status']!='preview' and np.linalg.norm(r.get('velocity_m_s',[0,0,0]))>1e-12 for r in self.states))
        if preview:report['latest_preview']=preview[-1]
        report['spread_note']='Base target spread includes actual target movement; robot was not commanded by this pipeline.'
        self.stream.close()
        (self.output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        return report


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s',type=float,default=30)
    parser.add_argument('--output',type=Path,required=True)
    options,ros_args=parser.parse_known_args(args)
    if not np.isfinite(options.duration_s) or options.duration_s<=0:parser.error('duration must be positive')
    options.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=ros_args);node=PreviewAudit(options.output);start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<options.duration_s:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        report=node.report(time.monotonic()-start);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(report,indent=2,ensure_ascii=False))
    if not report['base_target_samples']:raise SystemExit(1)
