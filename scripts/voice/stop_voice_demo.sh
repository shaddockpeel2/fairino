#!/usr/bin/env bash
# Stop the phase-6 speech pipeline and the phase-3 Qwen service.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

echo "== 停止语音管线服务"
if pgrep -f "fr3c_voice.tts_service" >/dev/null 2>&1; then
  for pid in $(pgrep -f "fr3c_voice.tts_service"); do kill -TERM "${pid}" 2>/dev/null; done
  sleep 2
  echo "   已发送停止信号"
else
  echo "   未在运行"
fi

echo "== 停止 Qwen 服务"
if [ -f scripts/voice/stop_qwen_service.sh ]; then
  scripts/voice/stop_qwen_service.sh 2>&1 | tail -1
else
  echo "   未找到停止脚本"
fi

sleep 2
echo
echo "剩余相关进程:"
ps -eo pid,args 2>/dev/null | grep -E "fr3c_voice.tts_service|fr3c_voice.service|qwen3_vl_worker" \
  | grep -v grep | grep -v "ps -eo" || echo "   无"
