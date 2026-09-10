#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "用法: $0 open|close|POSITION_METRES" >&2
  exit 2
fi

case "$1" in
  open) position='0.130' ;;
  close) position='0.0' ;;
  *) position="$1" ;;
esac

source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
set -u

exec ros2 action send_goal --feedback \
  /ydi_j22_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: ${position}, max_effort: 0.0}}"
