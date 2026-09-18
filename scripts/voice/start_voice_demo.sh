#!/usr/bin/env bash
# Bring up the phase-6 speech pipeline and the phase-3 Qwen service, then report
# readiness. Safe to re-run: it refuses to start a second copy of either service.
#
# Usage:
#   scripts/voice/start_voice_demo.sh                 # start both
#   scripts/voice/start_voice_demo.sh --speaker 45    # also raise speaker volume
#   scripts/voice/start_voice_demo.sh --no-qwen       # TTS only (fixed text)
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

SPEAKER=""
WITH_QWEN=1
while [ $# -gt 0 ]; do
  case "$1" in
    --speaker) SPEAKER="${2:-}"; shift 2 ;;
    --speaker=*) SPEAKER="${1#*=}"; shift ;;
    --no-qwen) WITH_QWEN=0; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

P6_RUNTIME="${ROOT}/evidence/voice/P6/runtime"
P3_RUNTIME="${ROOT}/evidence/voice/P3/runtime"
P6_SOCKET="${P6_RUNTIME}/fr3c-tts-pipeline.sock"
P3_SOCKET="${P3_RUNTIME}/fr3c-qwen-service.sock"
PY_TTS="${ROOT}/.venv-tts/bin/python"

mkdir -p "${P6_RUNTIME}" "${P3_RUNTIME}"

echo "== 检查音频设备"
if [ ! -d /dev/snd ]; then
  echo "错误：/dev/snd 不存在。请在真实终端里运行（不要在受限沙箱里），并确认 USB 声卡已插好。" >&2
  exit 1
fi
if ! aplay -l 2>/dev/null | grep -q "USB Audio Device"; then
  echo "错误：没有找到 'USB Audio Device' 声卡。检查 USB 连接后重试。" >&2
  aplay -l 2>&1 | head -5 >&2
  exit 1
fi
echo "   USB 声卡已就绪"

if [ -n "${SPEAKER}" ]; then
  amixer -c 2 sset Speaker "${SPEAKER}%" >/dev/null 2>&1 && echo "   扬声器音量设为 ${SPEAKER}%"
fi
# The amixer listing has a "Playback channels: Front Left - Front Right" header
# before the real value line, so match the value line explicitly.
volume_value="$(amixer -c 2 sget Speaker 2>/dev/null \
  | awk '/Front Left: Playback/ { match($0, /\[[0-9]+%\]/); print substr($0, RSTART, RLENGTH); exit }')"
echo "   当前音量: ${volume_value:-未知}"

echo "== 启动语音管线服务"
if [ -S "${P6_SOCKET}" ] && pgrep -f "fr3c_voice.tts_service" >/dev/null 2>&1; then
  echo "   已在运行，跳过"
else
  nohup setsid env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="${ROOT}/src/fr3c_voice" \
    "${PY_TTS}" -m fr3c_voice.tts_service \
    --trace "${P6_RUNTIME}/service-trace.jsonl" \
    </dev/null > "${P6_RUNTIME}/service.log" 2>&1 &
  echo "   启动中（模型加载约 4 秒 + 预热）..."
fi

echo "== 等待语音服务就绪"
ready=0
for _ in $(seq 1 60); do
  if [ -S "${P6_SOCKET}" ] && grep -q '"event": "ready"' "${P6_RUNTIME}/service.log" 2>/dev/null; then
    ready=1
    break
  fi
  if grep -q '"event": "error"' "${P6_RUNTIME}/service.log" 2>/dev/null; then
    echo "   启动失败：" >&2
    tail -3 "${P6_RUNTIME}/service.log" >&2
    exit 1
  fi
  sleep 1
done
[ "${ready}" = "1" ] || { echo "   超时未就绪，见 ${P6_RUNTIME}/service.log" >&2; exit 1; }
echo "   就绪"

if [ "${WITH_QWEN}" = "1" ]; then
  echo "== 启动 Qwen 服务（NPU）"
  if [ -S "${P3_SOCKET}" ] && pgrep -f "fr3c_voice.service" >/dev/null 2>&1; then
    echo "   已在运行，跳过"
  else
    nohup setsid env PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 \
      bash -c "source /opt/ros/humble/setup.bash && source '${ROOT}/install/setup.bash' && \
               export PYTHONPATH='${ROOT}/src/fr3c_voice':\"\${PYTHONPATH:-}\" && \
               /usr/bin/python3 -m fr3c_voice.service --workspace '${ROOT}' --no-ros" \
      </dev/null > "${P6_RUNTIME}/qwen.log" 2>&1 &
    echo "   启动中（Worker 加载约 5 秒）..."
  fi
  for _ in $(seq 1 90); do
    [ -S "${P3_SOCKET}" ] && break
    sleep 1
  done
  if [ -S "${P3_SOCKET}" ]; then
    echo "   就绪"
  else
    echo "   Qwen 未就绪，见 ${P6_RUNTIME}/qwen.log（固定文本模式仍可用）" >&2
  fi
fi

echo
echo "================ 就绪 ================"
echo "语音管线 socket : ${P6_SOCKET}"
[ "${WITH_QWEN}" = "1" ] && echo "Qwen 服务 socket: ${P3_SOCKET}"
echo
echo "试听（新终端）:"
echo "  cd ${ROOT}"
echo "  ./.venv-tts/bin/python scripts/voice/p6_speak.py --text '你好，我是抓取助手。'"
echo "  ./.venv-tts/bin/python scripts/voice/p6_speak.py --prompt '请用一百个字介绍你自己。'"
echo
echo "交互试听台:"
echo "  ./.venv-tts/bin/python scripts/voice/p6_demo.py"
echo
echo "停止服务:"
echo "  scripts/voice/stop_voice_demo.sh"
