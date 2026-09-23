#!/usr/bin/env bash
set -eo pipefail
# Match the established RViz launcher: this board's interactive Qt library
# paths conflict with the ROS/OGRE render window.
if [[ "${FR3C_TILTED_TEST_CLEAN_ENV:-0}" != "1" ]]; then
  exec env -u LD_LIBRARY_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH \
    FR3C_TILTED_TEST_CLEAN_ENV=1 bash --noprofile --norc "$0" "$@"
fi
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$task_root/install/setup.bash"
exec /usr/bin/python3 "$task_root/tools/tilted_red_grasp_test.py" "$@"
