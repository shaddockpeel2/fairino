#!/usr/bin/env bash
set -eo pipefail

confirmation='I_AM_MONITORING_GRIPPER'
if [[ "${1:-}" != "$confirmation" ]]; then
  echo "用法: $0 $confirmation" >&2
  exit 2
fi

command_script='/home/industio/fairino_ws/scripts/gripper/command_gripper.sh'

echo '阶段 1/3：命令并保持 60 mm。'
"$command_script" 0.060

echo '阶段 2/3：执行 5 次 60↔100 mm 目标切换。'
for cycle in 1 2 3 4 5; do
  echo "cycle $cycle: 100 mm"
  "$command_script" 0.100
  echo "cycle $cycle: 60 mm"
  "$command_script" 0.060
done

echo '阶段 3/3：末态保持 60 mm，观察 30 秒是否存在延迟旧动作。'
sleep 30
echo '验证序列完成；预期最终开度仍为 60 mm。'
