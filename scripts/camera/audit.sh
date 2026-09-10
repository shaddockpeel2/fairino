#!/usr/bin/env bash
# Run while run_bench.sh is active in the same ROS domain.
set -eo pipefail
camera_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$camera_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_CAMERA_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_CAMERA_LOCALHOST_ONLY:-1}"
cd "$camera_ws"
camera_output="${1:-$camera_ws/evidence/camera/$(date +%Y%m%d_%H%M%S)}"
ros2 run fr3c_camera rgbd_audit --duration "${2:-60}" --warmup 10 \
  --minimum-fps "${FR3C_CAMERA_MINIMUM_FPS:-8.0}" \
  --minimum-paired-fps "${FR3C_CAMERA_MINIMUM_PAIRED_FPS:-4.0}" \
  --maximum-receive-gap "${FR3C_CAMERA_MAXIMUM_RECEIVE_GAP:-1.5}" \
  --maximum-rgb-depth-skew "${FR3C_CAMERA_MAXIMUM_RGB_DEPTH_SKEW:-0.06}" \
  --output "$camera_output"
