#!/usr/bin/env bash
set -eo pipefail
orientation_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
orientation_viewer=true
orientation_args=()
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then orientation_viewer=false; fi
for orientation_arg in "$@"; do
  case "$orientation_arg" in
    --help|-h)
      echo 'Usage: start_orientation_check.sh [--no-view] [quality_profile:=conservative|demo] [scenario:=static|target_rotation|robot_rotation] [duration_s:=30] [name:=value ...]'
      echo 'Stage 6 read-only: starts missing vision/feedback/conversion, records paired orientation.'
      echo 'No motion client. View keys only Q/Esc. Audit completion or Ctrl-C closes owned sources.'
      echo 'Stop and exit the motion-capable Demo first. Existing read-only sources are reused.'
      exit 0 ;;
    --no-view) orientation_viewer=false ;;
    *:=*) orientation_args+=("$orientation_arg") ;;
    *) echo "Unknown argument: $orientation_arg" >&2; exit 1 ;;
  esac
done
source "$orientation_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
orientation_output="$orientation_ws/evidence/target_follow/stage6/$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
for orientation_arg in "${orientation_args[@]}"; do
  if [[ "$orientation_arg" == output_dir:=* ]]; then orientation_output="${orientation_arg#output_dir:=}"; fi
done
export ROS_LOG_DIR="$orientation_output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch fr3c_target_follow orientation_check.launch.py \
  output_dir:="$orientation_output" viewer:="$orientation_viewer" "${orientation_args[@]}"
