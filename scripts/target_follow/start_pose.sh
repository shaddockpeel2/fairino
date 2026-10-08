#!/usr/bin/env bash
set -eo pipefail
follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$follow_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$follow_ws/evidence/target_follow/runtime/ros_logs}"

# Query owners, not topic names: a surviving detector can subscribe to a dead camera.
read -r follow_color_count follow_pose_count follow_feedback_count follow_preview_count \
  < <(python3 "$follow_ws/scripts/target_follow/publisher_counts.py")
if (( follow_color_count > 0 )); then
  follow_start_camera=false
else
  ros2 run fr3c_camera dabai_camera_preflight --serial CH82C5201C9
  follow_start_camera=true
fi
if (( follow_pose_count > 0 )); then
  follow_start_detector=false
else
  follow_start_detector=true
fi
if [[ "$follow_start_camera" == false && "$follow_start_detector" == false ]]; then
  echo 'Camera and tag detector publishers are already running in this ROS domain.'
  exit 0
fi
exec ros2 launch fr3c_target_follow pose_only.launch.py \
  start_camera:="$follow_start_camera" start_detector:="$follow_start_detector" "$@"
