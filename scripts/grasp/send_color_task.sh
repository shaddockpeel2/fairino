#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <hold|sort> <red|yellow|blue|green>" >&2
  exit 2
fi

task_mode="$1"
target_color="$2"
case "$task_mode" in
  hold) action_name=/fr3c_first_grasp/pick_hold ;;
  sort) action_name=/fr3c_first_grasp/pick_sort ;;
  *) echo "Unsupported task mode: $task_mode" >&2; exit 2 ;;
esac
case "$target_color" in
  red|yellow|blue|green) ;;
  *) echo "Unsupported color: $target_color" >&2; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
exec ros2 action send_goal --feedback \
  "$action_name" fairino_msgs/action/PickColor "{color: '$target_color'}"
