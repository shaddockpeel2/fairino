#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 /absolute/path/to/session [additional handeye_capture options]" >&2
  exit 64
fi

session_path=$1
shift
workspace=/home/industio/fairino_ws

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

if ! ros2 run fr3c_camera dabai_camera_preflight; then
  echo "Hand-eye capture refused: DaBai USB preflight failed." >&2
  echo "Both Orbbec functions must enumerate at 480 Mbps before capture." >&2
  exit 3
fi

exec ros2 run fr3c_calibration handeye_capture \
  --session "${session_path}" "$@"
