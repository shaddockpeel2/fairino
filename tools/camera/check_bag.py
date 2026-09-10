#!/usr/bin/env python3
"""Offline readback of recorded messages, preserving original timestamps."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from fr3c_camera.rgbd import depth_metres, stamp_seconds

p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--output',required=True);a=p.parse_args()
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
types={t.name:get_message(t.type) for t in r.get_all_topics_and_types()}
counts=Counter();depth_samples=[];point_samples=[];diagnostics=Counter();stamps={}
while r.has_next():
    topic,data,_=r.read_next();m=deserialize_message(data,types[topic]);counts[topic]+=1
    if topic=='/camera/depth/image_raw':
        d=depth_metres(m);v=d[np.isfinite(d)&(d>0)]
        depth_samples.append({'valid_fraction':float(len(v)/d.size),'median_m':float(np.median(v)) if len(v) else None})
    if topic=='/fr3c_camera/probe_point':point_samples.append([m.point.x,m.point.y,m.point.z])
    if topic=='/fr3c_camera/diagnostics':
        for status in m.status:diagnostics[status.message]+=1
    if hasattr(m,'header'):stamps.setdefault(topic,[]).append(stamp_seconds(m))
required=['/camera/color/image_raw','/camera/depth/image_raw','/camera/color/camera_info','/camera/depth/camera_info','/tf_static']
result={'pass':all(counts[t]>0 for t in required),'messages':dict(counts),'depth_frames':len(depth_samples),
        'depth_median_m':float(np.median([x['median_m'] for x in depth_samples if x['median_m'] is not None])) if depth_samples else None,
        'probe_frames':len(point_samples),'probe_median_xyz_m':np.median(point_samples,axis=0).tolist() if point_samples else None,
        'probe_z_p05_p95_m':np.percentile(np.array(point_samples)[:,2],[5,95]).tolist() if point_samples else None,
        'diagnostics':dict(diagnostics),'note':'Scene-dependent readback, not absolute accuracy or object detection validation.'}
Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));raise SystemExit(0 if result['pass'] else 1)
