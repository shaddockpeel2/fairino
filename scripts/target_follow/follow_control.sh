#!/usr/bin/env bash
set -eo pipefail
follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
follow_action="${1:-}"
case "$follow_action" in
  start|stop|teach|status) ;;
  *) echo 'Usage: follow_control.sh start|stop|teach|status [--mock] (start records the current offset again)' >&2; exit 1 ;;
esac
shift
follow_domain=187
if [[ "${1:-}" == --mock ]]; then follow_domain=189; shift; fi
source "$follow_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-$follow_domain}"
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$follow_ws/evidence/target_follow/runtime/control_ros_logs}"
exec ros2 run fr3c_target_follow follow_control "$follow_action" "$@"
