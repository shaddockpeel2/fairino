#!/usr/bin/env bash
set -eo pipefail

# Rebuild a clean ROS library environment for RViz on this board.
if [[ "${FR3C_MOVEIT_CAMERA_CLEAN_ENV:-0}" != "1" ]]; then
  exec env -u LD_LIBRARY_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH \
    FR3C_MOVEIT_CAMERA_CLEAN_ENV=1 \
    bash --noprofile --norc "$0" "$@"
fi

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

if ros2 node list 2>/dev/null | grep -qx '/ydi_j22_gripper'; then
  echo 'ERROR: 检测到已有 /ydi_j22_gripper；请先关闭夹爪驱动再启动 MoveIt。' >&2
  exit 3
fi

cd "$workspace"
ros2 run fr3c_camera dabai_camera_preflight \
  --serial "${FR3C_CAMERA_SERIAL:-CH82C5201C9}"

echo "WARNING: this starts the current ros2_control configuration, which uses the real FAIRINO hardware plugin." >&2
gripper_backend="${FR3C_GRIPPER_BACKEND:-ft232h}"
if [[ "$gripper_backend" == "ft232h" ]]; then
  gripper_hardware_output_enabled='true'
else
  gripper_hardware_output_enabled='false'
fi

exec ros2 launch fairino3mt_v6_moveit2_config camera_moveit_demo.launch.py \
  start_gripper_driver:=true \
  gripper_backend:="$gripper_backend" \
  gripper_hardware_output_enabled:="$gripper_hardware_output_enabled" \
  "$@"
