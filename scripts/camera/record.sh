#!/usr/bin/env bash
# Bounded, camera-only recording. No robot topics or command interfaces.
set -eo pipefail
camera_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$camera_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_CAMERA_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_CAMERA_LOCALHOST_ONLY:-1}"
cd "$camera_ws"
camera_output="${1:-$camera_ws/evidence/camera/$(date +%Y%m%d_%H%M%S)/rgbd_bag}"
mkdir -p "$(dirname -- "$camera_output")"
set +e
timeout --signal=INT --kill-after=10 "${2:-10}" ros2 bag record -o "$camera_output" \
 /camera/color/image_raw /camera/depth/image_raw \
 /camera/color/camera_info /camera/depth/camera_info /tf_static \
 /fr3c_camera/probe_point /fr3c_camera/diagnostics
camera_status=$?
set -e
if [[ "$camera_status" != 0 && "$camera_status" != 124 ]]; then exit "$camera_status"; fi
ros2 bag info "$camera_output"
/usr/bin/python3 - "$camera_output/metadata.yaml" <<'PY_CHECK'
import sys, yaml
from pathlib import Path
m=yaml.safe_load(Path(sys.argv[1]).read_text())['rosbag2_bagfile_information']
counts={t['topic_metadata']['name']:t['message_count'] for t in m['topics_with_message_count']}
required=['/camera/color/image_raw','/camera/depth/image_raw','/camera/color/camera_info','/camera/depth/camera_info','/tf_static']
missing=[t for t in required if counts.get(t,0)==0]
if missing: raise SystemExit('Invalid recording, missing messages: '+', '.join(missing))
print('Required RGB-D topics recorded successfully.')
PY_CHECK
