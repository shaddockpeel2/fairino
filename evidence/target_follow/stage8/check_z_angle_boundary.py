"""Read-only GetInverseKinRef comparison on the recorded Z path; no motion API."""
import http.client
import json
from pathlib import Path
import xmlrpc.client

import numpy as np

from fr3c_target_follow.demo_graph import discover
from fr3c_target_follow.follow_geometry import transform_rpy
from fr3c_target_follow.marker_pose import rpy_degrees
from fr3c_target_follow.pose_preview import rotation_exp

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT/'20261010_105736_71024'
OUTPUT = ROOT/'z_angle_boundary'


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=3)


def main():
    graph = discover()
    if any(graph['publishers'].get(k, 0) for k in ('flange', 'feedback', 'follow', 'servo')):
        raise SystemExit('Close the existing robot SDK owner before this read-only diagnostic.')
    original = json.loads((SOURCE/'report.json').read_text())
    rows = [json.loads(line) for line in (SOURCE/'samples.jsonl').read_text().splitlines()]
    anchor = next(row for row in rows if row['kind'] == 'ik_preflight' and row['index'] == 0)
    flange = transform_rpy(anchor['target_flange_m_rad'][:3], anchor['target_flange_m_rad'][3:])
    tool = transform_rpy(original['flange_tcp_m_rad'][:3], original['flange_tcp_m_rad'][3:])
    tcp0 = flange @ tool
    seed = np.degrees(anchor['reference_joints_rad']).tolist()
    previous = np.degrees(anchor['target_flange_m_rad'][3:])
    result = dict(source_session=SOURCE.name, graph=graph, robot_ip='192.168.58.2',
                  rpc_methods=['GetInverseKinRef'], hardware_servo_commands_sent=0,
                  motion_commands_enabled=False, points=[])
    with xmlrpc.client.ServerProxy('http://192.168.58.2:20003', transport=BoundedTransport()) as rpc:
        for i in range(11):
            u = i/10
            angle = np.radians(original['angle_deg'])*u**3*(10-15*u+6*u*u)
            tcp = tcp0.copy()
            tcp[:3, :3] = rotation_exp([0., 0., angle]) @ tcp0[:3, :3]
            target = tcp @ np.linalg.inv(tool)
            canonical = np.asarray(rpy_degrees(target[:3, :3]))
            unwrapped = previous + np.degrees(np.arctan2(
                np.sin(np.radians(canonical-previous)), np.cos(np.radians(canonical-previous))))
            position_mm = (target[:3, 3]*1000).tolist()
            raw_pose = position_mm + unwrapped.tolist()
            wrapped_pose = position_mm + canonical.tolist()
            raw = rpc.GetInverseKinRef(0, raw_pose, seed)
            wrapped = rpc.GetInverseKinRef(0, wrapped_pose, seed)
            row = dict(index=i, raw_pose_mm_deg=raw_pose, canonical_pose_mm_deg=wrapped_pose,
                       reference_joints_deg=seed, raw_code=raw[0], canonical_code=wrapped[0],
                       canonical_joints_deg=wrapped[1:] if wrapped[0] == 0 else None)
            result['points'].append(row)
            if wrapped[0] == 0:
                seed = wrapped[1:]
            previous = unwrapped
    OUTPUT.mkdir(exist_ok=True)
    (OUTPUT/'readonly_comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
