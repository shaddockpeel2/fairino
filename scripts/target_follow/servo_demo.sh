#!/usr/bin/env bash
set -eo pipefail
follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$follow_ws/scripts/camera/activate.bash"
follow_mock=false
follow_move=false
follow_output=''
follow_previous=''
for follow_arg in "$@"; do
  if [[ "$follow_previous" == --output-dir ]]; then follow_output="$follow_arg"; fi
  if [[ "$follow_arg" == --mock ]]; then follow_mock=true; fi
  if [[ "$follow_arg" == --move ]]; then follow_move=true; fi
  if [[ "$follow_arg" == --inspect ]]; then follow_move=false; fi
  follow_previous="$follow_arg"
done
if [[ "$follow_mock" == true ]]; then
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-189}"
else
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
fi
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
if [[ "$follow_move" == true && "$follow_mock" == false ]]; then
  read -r follow_color_count follow_pose_count follow_feedback_count follow_preview_count \
    < <(python3 "$follow_ws/scripts/target_follow/publisher_counts.py")
  if (( follow_feedback_count > 0 )); then
    echo 'Close the existing read-only feedback/preview launch before the ServoCart probe; this probe supplies feedback itself.' >&2
    exit 1
  fi
fi
follow_stamp="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
follow_output="${follow_output:-$follow_ws/evidence/target_follow/stage3/$follow_stamp}"
export ROS_LOG_DIR="$follow_output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
echo "Report directory: $follow_output"
exec ros2 run fr3c_follow_driver servo_probe --output-dir "$follow_output" "$@"
