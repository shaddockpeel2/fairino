"""Bounded live RGB-D audit; exits nonzero for missing/misaligned/stale streams."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from message_filters import Subscriber, ApproximateTimeSynchronizer
from .rgbd import stamp_seconds, validate_pair, project_color_pixel, image_array


class Audit(Node):
    def __init__(self, args):
        super().__init__('fr3c_rgbd_audit')
        self.args = args
        self.start_time = time.monotonic() + args.warmup
        self.arrivals = {'color': [], 'depth': []}
        self.stamps = {'color': [], 'depth': []}
        self.ages = []
        self.skews = []
        self.valid = []
        self.patches = []
        self.failures = Counter()
        self.infos = {}
        self.transforms = {}
        self.snapshot = False
        self.subs = []
        ns = args.namespace.rstrip('/')
        for label, typ, topic in [('color',Image,'/color/image_raw'), ('depth',Image,'/depth/image_raw'),
                                 ('ci',CameraInfo,'/color/camera_info'), ('di',CameraInfo,'/depth/camera_info')]:
            sub = Subscriber(self, typ, ns+topic, qos_profile=qos_profile_sensor_data)
            sub.registerCallback(lambda msg, key=label: self.observe(key,msg))
            self.subs.append(sub)
        self.sync = ApproximateTimeSynchronizer(self.subs, 20, args.maximum_rgb_depth_skew)
        self.sync.registerCallback(self.pair)
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/tf_static', self.tf, qos)

    def tf(self, msg):
        for t in msg.transforms:
            self.transforms[t.child_frame_id] = {'parent':t.header.frame_id,
                'translation':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                'rotation_xyzw':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]}

    def observe(self, label, msg):
        if time.monotonic() < self.start_time: return
        if label in self.arrivals:
            self.arrivals[label].append(time.monotonic())
            self.stamps[label].append(stamp_seconds(msg))
            self.ages.append(self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg))
        else:
            self.infos[label] = {'width':msg.width,'height':msg.height,'frame_id':msg.header.frame_id,
                'k':list(msg.k),'d':list(msg.d),'p':list(msg.p),'distortion_model':msg.distortion_model}

    def pair(self, color, depth, ci, di):
        if time.monotonic() < self.start_time: return
        try:
            metres = validate_pair(
                color, depth, ci, di,
                self.get_clock().now().nanoseconds * 1e-9,
                max_skew=self.args.maximum_rgb_depth_skew)
        except (ValueError, cv2.error) as e:
            self.failures[str(e)] += 1
            return
        self.skews.append(abs(stamp_seconds(color)-stamp_seconds(depth)))
        self.valid.append(float((np.isfinite(metres) & (metres>0)).mean()))
        samples = {}
        for name, x, y in [('center',.5,.5),('left',.25,.5),('right',.75,.5),('top',.5,.25),('bottom',.5,.75)]:
            try:
                xyz, fraction, spread = project_color_pixel(metres,ci,di,x*(ci.width-1),y*(ci.height-1))
                samples[name] = {'xyz_m':xyz,'valid_fraction':fraction,'spread_m':spread}
            except (ValueError, cv2.error) as e: samples[name] = {'rejected':str(e)}
        self.patches.append(samples)
        if not self.snapshot:
            folder = Path(self.args.output); folder.mkdir(parents=True,exist_ok=True)
            rgb = image_array(color)
            cv2.imwrite(str(folder/'color.jpg'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR) if color.encoding=='rgb8' else rgb)
            cv2.imwrite(str(folder/'depth_mm.png'),np.clip(np.nan_to_num(metres)*1000,0,65535).astype(np.uint16))
            preview = cv2.applyColorMap(np.clip(metres*255/2,0,255).astype(np.uint8),cv2.COLORMAP_TURBO)
            preview[~np.isfinite(metres)|(metres<=0)] = 0
            cv2.imwrite(str(folder/'depth_preview.jpg'),preview)
            self.snapshot = True

    def report(self):
        streams = {}
        for name, a in self.arrivals.items():
            gaps = np.diff(a)
            source_gaps = np.diff(self.stamps[name])
            streams[name] = {'frames':len(a),'fps':(len(a)-1)/(a[-1]-a[0]) if len(a)>1 else 0,
                'max_gap_s':float(max(gaps)) if len(gaps) else None,
                'timestamp_regressions':int(sum(source_gaps<=0))}
        checks = {
            'streams_meet_minimum_fps': all(
                s['fps'] >= self.args.minimum_fps and
                s['frames'] >= self.args.duration * self.args.minimum_fps
                for s in streams.values()),
            'no_long_receive_gap': all(
                s['max_gap_s'] is not None and
                s['max_gap_s'] < self.args.maximum_receive_gap
                for s in streams.values()),
            'monotonic_timestamps':all(s['timestamp_regressions']==0 for s in streams.values()),
            'fresh_timestamps': bool(self.ages) and min(self.ages)>=-0.05 and max(self.ages)<=0.5,
            'enough_aligned_pairs': len(self.skews) >= (
                self.args.duration * self.args.minimum_paired_fps),
            'rgb_depth_skew_bounded': bool(self.skews) and (
                float(np.percentile(self.skews, 95)) <= self.args.maximum_rgb_depth_skew),
            'valid_pair_contract':not self.failures,
            'some_valid_depth':bool(self.valid) and float(np.median(self.valid))>0.1,
            'camera_info_received':len(self.infos)==2,
            'optical_tf_received': 'camera_color_optical_frame' in self.transforms,
        }
        return {'pass':all(checks.values()),'checks':checks,'duration_s':self.args.duration,
            'streams':streams,'aligned_pairs':len(self.skews),'pair_failures':dict(self.failures),
            'skew_p50_s':float(np.median(self.skews)) if self.skews else None,
            'skew_p95_s':float(np.percentile(self.skews,95)) if self.skews else None,
            'max_age_s':max(self.ages) if self.ages else None,
            'median_valid_depth_fraction':float(np.median(self.valid)) if self.valid else None,
            'camera_info':self.infos,'tf_static':self.transforms,
            'last_five_patch_samples':self.patches[-5:],
            'limitations':['Bench only, no hand-eye or base_link transform.',
                'No measured distance ground truth; valid fraction depends on scene.',
                'System timestamps do not prove hardware exposure synchronisation.',
                'Pixel probe is not an object detector or grasp command.']}


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration',type=float,default=60)
    parser.add_argument('--warmup',type=float,default=10)
    parser.add_argument('--namespace',default='/camera')
    parser.add_argument('--output',required=True)
    parser.add_argument('--minimum-fps', type=float, default=8.0)
    parser.add_argument('--minimum-paired-fps', type=float, default=4.0)
    parser.add_argument('--maximum-receive-gap', type=float, default=1.5)
    parser.add_argument('--maximum-rgb-depth-skew', type=float, default=0.06)
    options = parser.parse_args(args)
    if (options.duration <= 0 or options.warmup < 0 or options.minimum_fps <= 0 or
            options.minimum_paired_fps <= 0 or
            options.maximum_receive_gap <= 0 or options.maximum_rgb_depth_skew <= 0):
        parser.error('duration and validation thresholds must be positive')
    rclpy.init()
    node = Audit(options)
    end = node.start_time + options.duration
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic()<end: rclpy.spin_once(node,timeout_sec=0.2)
    except KeyboardInterrupt: interrupted=True
    result = node.report()
    if interrupted: result['pass']=False; result['interrupted']=True
    folder=Path(options.output); folder.mkdir(parents=True,exist_ok=True)
    (folder/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('pass','checks','streams','aligned_pairs','median_valid_depth_fraction')},indent=2))
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if result['pass'] else 1)
