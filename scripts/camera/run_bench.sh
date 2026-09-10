#!/usr/bin/env bash
set -eo pipefail
camera_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$camera_ws/scripts/camera/activate.bash"
# Override explicitly when integrating into a robot ROS domain.
export ROS_DOMAIN_ID="${FR3C_CAMERA_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_CAMERA_LOCALHOST_ONLY:-1}"
cd "$camera_ws"
if [[ "${FR3C_CAMERA_SKIP_PREFLIGHT:-0}" != "1" ]]; then
  ros2 run fr3c_camera dabai_camera_preflight \
    --serial "${FR3C_CAMERA_SERIAL:-CH82C5201C9}"
fi
exec ros2 launch fr3c_camera bench.launch.py "$@"
