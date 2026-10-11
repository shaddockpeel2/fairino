#!/usr/bin/env bash
set -eo pipefail
pose_preview_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
pose_preview_viewer=true
pose_preview_args=()
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then pose_preview_viewer=false; fi
for pose_preview_arg in "$@"; do
  case "$pose_preview_arg" in
    --help|-h)
      echo 'Usage: start_pose_preview.sh [--no-view] [auto_teach:=true] [rotation_mapping_gain:=0.5] [duration_s:=90] [name:=value ...]'
      echo 'Stage 7 read-only pose preview. T records the relation; no S/X motion clients.'
      echo 'Demo quality is the default. For SSH, --no-view auto_teach:=true records the first trusted pose.'
      echo 'Stop and exit the motion-capable Demo and previous orientation check first.'
      exit 0 ;;
    --no-view) pose_preview_viewer=false ;;
    *:=*) pose_preview_args+=("$pose_preview_arg") ;;
    *) echo "Unknown argument: $pose_preview_arg" >&2; exit 1 ;;
  esac
done
source "$pose_preview_ws/scripts/camera/activate.bash"
export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
pose_preview_output="$pose_preview_ws/evidence/target_follow/stage7/$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
for pose_preview_arg in "${pose_preview_args[@]}"; do
  if [[ "$pose_preview_arg" == output_dir:=* ]]; then pose_preview_output="${pose_preview_arg#output_dir:=}"; fi
done
export ROS_LOG_DIR="$pose_preview_output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch fr3c_target_follow pose_preview.launch.py \
  output_dir:="$pose_preview_output" viewer:="$pose_preview_viewer" "${pose_preview_args[@]}"
