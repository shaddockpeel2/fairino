#!/usr/bin/env bash
set -eo pipefail
follow_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
follow_mock=false
follow_viewer=true
follow_output=''
follow_args=()
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then follow_viewer=false; fi
for follow_arg in "$@"; do
  case "$follow_arg" in
    --help|-h)
      echo 'Usage: start_demo.sh [--mock] [--no-view] [name:=value ...]'
      echo 'Starts missing vision sources and the idle driver. T teach / S start / X stop.'
      echo 'Ctrl-C or closing the Demo-owned viewer stops the driver and owned nodes.'
      echo 'Reuse camera/detector in FR3C_FOLLOW_DOMAIN_ID (default 187, mock 189).'
      echo 'collision_enabled:=false selects legacy following; true enables model/table checks (restart to switch).'
      exit 0 ;;
    --mock) follow_mock=true; follow_args+=(mock:=true) ;;
    --no-view) follow_viewer=false; follow_args+=(viewer:=false) ;;
    mock:=*) follow_mock="${follow_arg#mock:=}"; follow_args+=("$follow_arg") ;;
    output_dir:=*) follow_output="${follow_arg#output_dir:=}"; follow_args+=("$follow_arg") ;;
    *:=*) follow_args+=("$follow_arg") ;;
    *) echo "Unknown argument: $follow_arg (use --help)" >&2; exit 1 ;;
  esac
done
source "$follow_ws/scripts/camera/activate.bash"
if [[ "$follow_mock" == true ]]; then
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-189}"
else
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
fi
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
follow_stamp="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
follow_output="${follow_output:-$follow_ws/evidence/target_follow/stage5/$follow_stamp}"
export ROS_LOG_DIR="$follow_output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
echo "Demo logs: $follow_output"
exec ros2 launch fr3c_target_follow demo.launch.py \
  output_dir:="$follow_output" viewer:="$follow_viewer" "${follow_args[@]}"
