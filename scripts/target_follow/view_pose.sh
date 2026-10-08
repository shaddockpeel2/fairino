#!/usr/bin/env bash
set -eo pipefail
follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$follow_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$follow_ws/evidence/target_follow/runtime/ros_logs}"
exec ros2 run fr3c_target_follow pose_view "$@"
