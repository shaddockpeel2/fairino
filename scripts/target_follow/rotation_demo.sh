#!/usr/bin/env bash
set -eo pipefail
rotation_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$rotation_ws/scripts/camera/activate.bash"
rotation_mock=false
rotation_output=''
rotation_tcp_file="$rotation_ws/src/fairino3mt_v6_moveit2_config/config/ydi_j22_tcp.yaml"
rotation_args=()
while (( $# )); do
  case "$1" in
    --help|-h)
      echo 'Usage: rotation_demo.sh [--inspect | --move] [--mock] [--axis x|y|z|xyz] [--angle-deg 2.5]'
      echo '  [--speed-deg-s 3] [--accel-deg-s2 10] [--round-trip | --one-way]'
      echo '  [--period-ms 8] [--hold-s .5] [--stop-after-s .5] [--tcp-file PATH] [--output-dir PATH] [--ip IP]'
      echo 'Stage 8: default read-only inspection; --move rotates around the current measured TCP and returns.'
      echo 'Close all other feedback/servo clients first. No vision source or saved starting pose is used.'
      exit 0 ;;
    --tcp-file|--output-dir)
      if (( $# < 2 )); then echo "Missing value for $1" >&2; exit 1; fi
      if [[ "$1" == --tcp-file ]]; then rotation_tcp_file="$2"; else rotation_output="$2"; fi
      shift 2 ;;
    --tcp-m-rad) echo 'Use --tcp-file to load a calibrated TCP.' >&2; exit 1 ;;
    --mock) rotation_mock=true; rotation_args+=("$1"); shift ;;
    *) rotation_args+=("$1"); shift ;;
  esac
done
if [[ "$rotation_mock" == true ]]; then
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-189}"
else
  export ROS_DOMAIN_ID="${FR3C_FOLLOW_DOMAIN_ID:-187}"
fi
export ROS_LOCALHOST_ONLY="${FR3C_FOLLOW_LOCALHOST_ONLY:-1}"
rotation_tcp_values="$(python3 - "$rotation_tcp_file" <<'PY'
import math
from pathlib import Path
import sys
import yaml
config = yaml.safe_load(Path(sys.argv[1]).read_text())['ydi_j22_tcp']
if (config.get('calibrated') is not True or config.get('parent_frame') != 'flange_link'
        or config.get('child_frame') != 'grasp_frame'):
    raise SystemExit('A calibrated flange_link -> grasp_frame TCP is required.')
xyz = [float(value) for value in config['xyz_m']]
rpy = [float(value) for value in config['rpy_rad']]
if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz+rpy):
    raise SystemExit('Invalid full TCP transform.')
print(' '.join(str(float(v)) for v in xyz+rpy))
PY
)"
read -r -a rotation_tcp <<< "$rotation_tcp_values"
if [[ "$rotation_mock" == false ]]; then
  read -r rotation_color_count rotation_pose_count rotation_feedback_count rotation_preview_count \
    < <(python3 "$rotation_ws/scripts/target_follow/publisher_counts.py")
  if (( rotation_feedback_count > 0 )); then
    echo 'Close the existing feedback, preview or follow process first; this probe owns SDK feedback and motion.' >&2
    exit 1
  fi
fi
rotation_stamp="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
rotation_output="${rotation_output:-$rotation_ws/evidence/target_follow/stage8/$rotation_stamp}"
export ROS_LOG_DIR="$rotation_output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cp -- "$rotation_tcp_file" "$rotation_output/tcp_used.yaml"
echo "Stage 8 report directory: $rotation_output"
rotation_binary="$rotation_ws/install/fr3c_follow_driver/lib/fr3c_follow_driver/rotation_probe"
if [[ ! -x "$rotation_binary" ]]; then
  echo 'Build fr3c_follow_driver first to install rotation_probe.' >&2
  exit 1
fi
exec "$rotation_binary" --tcp-m-rad "${rotation_tcp[@]}" \
  --output-dir "$rotation_output" "${rotation_args[@]}"
