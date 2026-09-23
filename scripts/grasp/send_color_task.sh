#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 <hold|sort> <red|yellow|blue|green> [zone]" >&2
  echo "       $0 place <red|yellow|blue|green> <zone>" >&2
  echo "       $0 stack <red|yellow|blue|green> <lower red|yellow|blue|green>" >&2
  echo "       $0 auto" >&2
  echo "  zone:  calibrated A4 region (A|B|C|D) or one 1-based slot (for example A3)" >&2
  echo "  stack: puts the first colour on top of the second colour" >&2
  echo "  auto:  stacks every cube currently on the table (red on top); no parameters" >&2
}

if [[ $# -eq 0 ]]; then
  usage
  exit 2
fi

task_mode="$1"
if [[ "$task_mode" == auto ]]; then
  if [[ $# -ne 1 ]]; then
    echo "auto takes no parameters, for example: $0 auto" >&2
    usage
    exit 2
  fi
  target_color=""
  target_support=""
elif [[ $# -ge 2 && $# -le 3 ]]; then
  target_color="$2"
  target_support="${3:-}"
else
  usage
  exit 2
fi

case "$task_mode" in
  hold) action_name=/fr3c_first_grasp/pick_hold ;;
  sort) action_name=/fr3c_first_grasp/pick_sort ;;
  place) action_name=/fr3c_first_grasp/pick_place ;;
  stack) action_name=/fr3c_first_grasp/pick_stack ;;
  auto) action_name=/fr3c_first_grasp/pick_stack_auto ;;
  *) echo "Unsupported task mode: $task_mode" >&2; usage; exit 2 ;;
esac
if [[ "$task_mode" != auto ]]; then
  case "$target_color" in
    red|yellow|blue|green) ;;
    *) echo "Unsupported color: $target_color" >&2; exit 2 ;;
  esac
fi

target_zone=""
target_support_color=""
if [[ "$task_mode" == stack ]]; then
  if [[ -z "$target_support" ]]; then
    echo "stack requires the lower colour, for example: $0 stack red blue" >&2
    exit 2
  fi
  case "$target_support" in
    red|yellow|blue|green) ;;
    *) echo "Unsupported lower colour: $target_support" >&2; exit 2 ;;
  esac
  if [[ "$target_support" == "$target_color" ]]; then
    echo "stack needs two different colours; both are $target_color" >&2
    exit 2
  fi
  target_support_color="$target_support"
elif [[ "$task_mode" != auto ]]; then
  target_zone="$target_support"
fi

case "$task_mode:$target_zone" in
  place:) echo "place requires a zone, for example: $0 place red C" >&2; exit 2 ;;
  hold:*|sort:*|stack:*|auto:*)
    if [[ -n "$target_zone" ]]; then
      echo "$task_mode does not accept a zone; use 'place' instead" >&2
      exit 2
    fi
    ;;
  place:*)
    if [[ ! "$target_zone" =~ ^[A-Za-z][0-9]{0,2}$ ]]; then
      echo "Unsupported zone: $target_zone" >&2; exit 2
    fi
    ;;
esac

# ROS/ament setup scripts are not nounset-safe: even a shell that has already
# sourced ROS can hit an unset AMENT_TRACE_SETUP_FILES here. Keep strict mode for
# this script, but suspend only `set -u` while importing generated environments.
set +u
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
set -u
exec ros2 action send_goal --feedback \
  "$action_name" fairino_msgs/action/PickColor \
  "{color: '$target_color', zone: '$target_zone', support_color: '$target_support_color'}"
