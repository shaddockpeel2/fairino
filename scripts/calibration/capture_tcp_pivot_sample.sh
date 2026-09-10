#!/usr/bin/env bash
set -eo pipefail

workspace=/home/industio/fairino_ws
session_default="${workspace}/evidence/P10/tcp_pivot_20260910"
session_path=${1:-${session_default}}
if [[ $# -gt 0 ]]; then
  shift
fi

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

exec ros2 run fr3c_calibration tcp_pivot_capture \
  --session "${session_path}" "$@"
