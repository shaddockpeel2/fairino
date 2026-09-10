#!/usr/bin/env bash
set -eo pipefail

# Rebuild a clean ROS library environment. This board's interactive shell may
# contain Qt/OGRE paths that make RViz fail while creating its render window.
if [[ "${FR3C_CAMERA_CLEAN_RVIZ_ENV:-0}" != "1" ]]; then
  exec env -u LD_LIBRARY_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH \
    FR3C_CAMERA_CLEAN_RVIZ_ENV=1 \
    bash --noprofile --norc "$0" "$@"
fi

camera_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$camera_ws/scripts/camera/activate.bash"

# Keep the camera-only tool isolated by default. Override these variables when
# it must join the robot's ROS graph.
export ROS_DOMAIN_ID="${FR3C_CAMERA_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_CAMERA_LOCALHOST_ONLY:-1}"

cd "$camera_ws"
if [[ "${FR3C_CAMERA_SKIP_PREFLIGHT:-0}" != "1" ]]; then
  ros2 run fr3c_camera dabai_camera_preflight \
    --serial "${FR3C_CAMERA_SERIAL:-CH82C5201C9}"
fi

exec ros2 launch fr3c_camera camera_rviz.launch.py "$@"
