#!/usr/bin/env bash
set -eo pipefail

confirmation='I_AM_MONITORING_GRIPPER'
if [[ "${1:-}" != "$confirmation" ]]; then
  echo "用法: $0 $confirmation" >&2
  exit 2
fi

command_script='/home/industio/fairino_ws/scripts/gripper/command_gripper.sh'

echo '阶段 1/2：只发送一次抓取位 26 mm（台架 31°）。'
"$command_script" close

echo '阶段 2/2：只发送一次张开释放位 60 mm（台架 70°）。'
"$command_script" open

echo '验证序列完成；末态为 60 mm。脚本不做循环、扫角或额外保持动作。'
