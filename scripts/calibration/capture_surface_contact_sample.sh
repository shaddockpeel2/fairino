#!/usr/bin/env bash
set -eo pipefail

workspace=/home/industio/fairino_ws

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

exec ros2 run fr3c_calibration surface_contact_capture "$@"
