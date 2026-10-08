#!/usr/bin/env bash
set -eo pipefail
follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$follow_ws/scripts/camera/activate.bash"
follow_mock=false
follow_output=''
for follow_arg in "$@"; do
  if [[ "$follow_arg" == --help || "$follow_arg" == -h ]]; then
    echo 'Usage: start_follow.sh [collision_enabled:=false|true] [name:=value ...]'
    echo 'Default uses position_follow.yaml (legacy mode: collision_enabled=false).'
    echo 'Switch after stopping and exiting the current driver. Launch starts idle.'
    exit 0
  fi
  if [[ "$follow_arg" == mock:=true ]]; then follow_mock=true; fi
  if [[ "$follow_arg" == output_dir:=* ]]; then follow_output="${follow_arg#output_dir:=}"; fi
done
if [[ "$follow_mock" == true ]]; then
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-189}"
else
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
fi
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
follow_stamp="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
follow_output="${follow_output:-$follow_ws/evidence/target_follow/stage4/$follow_stamp}"
export ROS_LOG_DIR="$follow_output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
read -r follow_color_count follow_pose_count follow_feedback_count follow_preview_count \
  < <(python3 "$follow_ws/scripts/target_follow/publisher_counts.py")
if (( follow_feedback_count > 0 || follow_preview_count > 0 )); then
  echo 'Close the existing feedback/preview/servo launch first; this launch supplies feedback and base conversion.' >&2
  exit 1
fi
if [[ "$follow_mock" == false ]] && (( follow_color_count == 0 || follow_pose_count == 0 )); then
  echo 'Start scripts/target_follow/start_pose.sh in this ROS domain first.' >&2
  exit 1
fi
echo "Report directory: $follow_output"
echo 'Driver starts idle. Use follow_control.sh start to record the current offset and start following.'
echo 'collision_enabled:=false selects the previous following loop; true enables the model/table check.'
exec ros2 launch fr3c_target_follow position_follow.launch.py output_dir:="$follow_output" "$@"
