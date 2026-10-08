#!/usr/bin/env bash
set -eo pipefail
follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$follow_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$follow_ws/evidence/target_follow/stage2/ros_logs}"
read -r follow_color_count follow_pose_count follow_feedback_count follow_preview_count \
  < <(python3 "$follow_ws/scripts/target_follow/publisher_counts.py")
if (( follow_color_count == 0 || follow_pose_count == 0 )); then
  echo 'Start scripts/target_follow/start_pose.sh in this ROS domain first.' >&2
  exit 1
fi
if (( follow_feedback_count > 0 )); then
  follow_start_feedback=false
else
  follow_start_feedback=true
fi
if (( follow_preview_count > 0 )); then
  follow_start_preview=false
else
  follow_start_preview=true
fi
if [[ "$follow_start_feedback" == false && "$follow_start_preview" == false ]]; then
  echo 'Robot feedback and following preview publishers are already running in this ROS domain.'
  exit 0
fi
exec ros2 launch fr3c_target_follow preview.launch.py \
  start_feedback:="$follow_start_feedback" start_preview:="$follow_start_preview" "$@"
