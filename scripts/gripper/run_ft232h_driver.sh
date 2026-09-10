#!/usr/bin/env bash
set -eo pipefail

confirmation_value='I_UNDERSTAND_OPEN_LOOP_PWM'
if [[ "${FR3C_ENABLE_FT232H_PWM:-}" != "${confirmation_value}" ]]; then
  echo 'FT232H PWM 未解锁；不会启动真实输出后端。' >&2
  echo "请先执行: export FR3C_ENABLE_FT232H_PWM=${confirmation_value}" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
set -u

if ros2 node list 2>/dev/null | grep -qx '/ydi_j22_gripper'; then
  echo '检测到已有 /ydi_j22_gripper；请先关闭 mock/旧驱动，避免两个 action server。' >&2
  exit 3
fi

exec ros2 launch fr3c_gripper_driver gripper.launch.py \
  backend:=ft232h hardware_output_enabled:=true "$@"
