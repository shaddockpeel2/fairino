#!/usr/bin/env bash
# Run while run_bench.sh is active. Read-only viewer; click an RGB target ROI.
set -eo pipefail
camera_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$camera_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_CAMERA_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_CAMERA_LOCALHOST_ONLY:-1}"
camera_output="${FR3C_LIVE_OUTPUT:-$camera_ws/evidence/camera/$(date +%Y%m%d_%H%M%S)_live}"
exec ros2 run fr3c_camera rgbd_live_view --output "$camera_output" \
  --display-rotation "${FR3C_LIVE_DISPLAY_ROTATION:-180}" "$@"
