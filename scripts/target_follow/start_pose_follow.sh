#!/usr/bin/env bash
set -eo pipefail
pose_follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
pose_follow_mock=false
pose_follow_viewer=true
pose_follow_output=''
pose_follow_args=()
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then pose_follow_viewer=false; fi
for pose_follow_arg in "$@"; do
  case "$pose_follow_arg" in
    --help|-h)
      echo 'Usage: start_pose_follow.sh [--mock] [--no-view] [collision_enabled:=false|true] [rotation_mapping_gain:=0.5] [name:=value ...]'
      echo 'Stage 9 full pose following, starts IDLE. T records, S records again and starts, X stops.'
      echo 'Close previous feedback/preview/follow/orientation/probe processes first; one SDK owner.'
      echo 'Ctrl-C or closing the owned viewer ends the driver and owned sources.'
      exit 0 ;;
    --mock) pose_follow_mock=true; pose_follow_args+=(mock:=true) ;;
    --no-view) pose_follow_viewer=false ;;
    mock:=*) pose_follow_mock="${pose_follow_arg#mock:=}"; pose_follow_args+=("$pose_follow_arg") ;;
    output_dir:=*) pose_follow_output="${pose_follow_arg#output_dir:=}"; pose_follow_args+=("$pose_follow_arg") ;;
    *:=*) pose_follow_args+=("$pose_follow_arg") ;;
    *) echo "Unknown argument: $pose_follow_arg" >&2; exit 1 ;;
  esac
done
source "$pose_follow_ws/scripts/camera/activate.bash"
if [[ "$pose_follow_mock" == true ]]; then
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-189}"
else
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
fi
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
pose_follow_output="${pose_follow_output:-$pose_follow_ws/evidence/target_follow/stage9/$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$}"
export ROS_LOG_DIR="$pose_follow_output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
echo "Stage 9 pose-follow logs: $pose_follow_output"
exec ros2 launch fr3c_target_follow pose_follow.launch.py \
  output_dir:="$pose_follow_output" viewer:="$pose_follow_viewer" "${pose_follow_args[@]}"
