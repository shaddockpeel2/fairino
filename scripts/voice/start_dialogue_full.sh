#!/usr/bin/env bash
# Start the whole offline dialogue page: Qwen service + streaming speech
# pipeline + GUI.
#
# Order matters. The speech service takes the speaker exclusively, and the GUI
# greys out its speech controls when the service is missing, so both services
# must be up before the window opens.
#
# Usage:
#   scripts/voice/start_dialogue_full.sh              # speech on, speaker 40%
#   scripts/voice/start_dialogue_full.sh --speaker 55
#   scripts/voice/start_dialogue_full.sh --no-speech  # page without playback
#   scripts/voice/start_dialogue_full.sh --check-only # report readiness, no GUI
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

SPEAKER="40"
WITH_SPEECH=1
CHECK_ONLY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --speaker) SPEAKER="${2:-40}"; shift 2 ;;
    --speaker=*) SPEAKER="${1#*=}"; shift ;;
    --no-speech) WITH_SPEECH=0; shift ;;
    --check-only) CHECK_ONLY=1; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# A second window would compete with the first for the exclusive microphone.
# Check before installing the shutdown trap so this invocation cannot stop the
# services owned by the already running window.
if [ "${CHECK_ONLY}" != "1" ] &&
   pgrep -f "^/usr/bin/python3 -m fr3c_voice.dialogue_gui --workspace ${ROOT}( |$)" >/dev/null 2>&1; then
  echo "对话窗口已在运行，请使用现有窗口；如需重启，请先关闭它。"
  exit 0
fi

GUI_PID=""
SHUTTING_DOWN=0

shutdown_stack() {
  local status="${1:-0}"
  if [ "${SHUTTING_DOWN}" = "1" ]; then
    return
  fi
  SHUTTING_DOWN=1
  trap - INT TERM EXIT

  echo
  echo "== 正在平滑关闭整套对话服务"
  if [ -n "${GUI_PID}" ] && kill -0 "${GUI_PID}" 2>/dev/null; then
    echo "   关闭 GUI、唤醒与 ASR..."
    kill -TERM "${GUI_PID}" 2>/dev/null || true
    local gui_exited=0 gui_state=""
    for _ in $(seq 1 150); do
      if ! kill -0 "${GUI_PID}" 2>/dev/null; then
        gui_exited=1
        break
      fi
      gui_state="$(ps -o stat= -p "${GUI_PID}" 2>/dev/null || true)"
      case "${gui_state}" in
        Z*) gui_exited=1; break ;;
      esac
      sleep 0.1
    done
    if [ "${gui_exited}" = "0" ]; then
      echo "   GUI 未及时退出，终止其剩余进程..." >&2
      kill -TERM -- "-${GUI_PID}" 2>/dev/null || true
      sleep 1
      kill -KILL -- "-${GUI_PID}" 2>/dev/null || true
    fi
    wait "${GUI_PID}" 2>/dev/null || true
  fi
  "${ROOT}/scripts/voice/stop_voice_demo.sh" || true
  echo "== 全部关闭完成"
  return "${status}"
}

handle_signal() {
  local status="$1"
  shutdown_stack "${status}" || true
  exit "${status}"
}

# In interactive mode this script owns the whole stack, including services
# which were already healthy when it started.  Keep --check-only non-owning so
# it remains useful as a readiness probe.
if [ "${CHECK_ONLY}" != "1" ]; then
  trap 'handle_signal 130' INT
  trap 'handle_signal 143' TERM
  trap 'status=$?; shutdown_stack "${status}" || true; exit "${status}"' EXIT
fi

start_if_needed() {
  # $1 = description, $2 = pgrep pattern, $3 = socket path, $4 = command
  local description="$1" pattern="$2" socket="$3" command="$4"
  if socket_healthy "${socket}" && pgrep -f "${pattern}" >/dev/null 2>&1; then
    echo "   ${description} 已在运行"
    return 0
  fi
  echo "   ${description} 启动中..."
  # A task runner may tear down its process group after this script exits;
  # detach the long-lived service into a fresh session as well as using nohup.
  nohup setsid bash -c "${command}" </dev/null \
    > "${LOG_BASE}/${description}.log" 2>&1 &
  for _ in $(seq 1 90); do
    socket_healthy "${socket}" && { echo "   ${description} 就绪"; return 0; }
    sleep 1
  done
  echo "   !! ${description} 未就绪，见 ${LOG_BASE}/${description}.log" >&2
  return 1
}

socket_healthy() {
  local socket="$1"
  [ -S "${socket}" ] || return 1
  /usr/bin/python3 - "${socket}" <<'PY' >/dev/null 2>&1
import json
import socket
import sys

client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
client.settimeout(1.0)
try:
    client.connect(sys.argv[1])
    client.sendall(b'{"op":"health"}\n')
    line = client.makefile('r', encoding='utf-8').readline()
    reply = json.loads(line)
    if not reply.get('healthy'):
        raise SystemExit(1)
finally:
    client.close()
PY
}

LOG_BASE="${ROOT}/evidence/voice/P6/runtime"
mkdir -p "${LOG_BASE}"
KNOWLEDGE_PYTHON="${FR3C_KNOWLEDGE_PYTHON:-/home/DofbotPro_JetsonNano_0106/.venv-rk3588/bin/python}"
if [ ! -x "${KNOWLEDGE_PYTHON}" ]; then
  echo "   !! 知识库隔离环境缺失，将保留普通聊天并禁用知识检索" >&2
  KNOWLEDGE_PYTHON=/usr/bin/python3
fi

echo "== 1/3 Qwen 模型服务"
start_if_needed "qwen-service" "fr3c_voice.service" \
  "${ROOT}/evidence/voice/P3/runtime/fr3c-qwen-service.sock" \
  "cd '${ROOT}' && source /opt/ros/humble/setup.bash && source '${ROOT}/install/setup.bash' && \
   PYTHONUNBUFFERED=1 PYTHONPATH='${ROOT}/src/fr3c_voice':\"\${PYTHONPATH:-}\" \
   '${KNOWLEDGE_PYTHON}' -m fr3c_voice.service --workspace '${ROOT}'"

if [ "${WITH_SPEECH}" = "1" ]; then
  echo "== 2/3 流式语音管线（独占扬声器）"
  start_if_needed "tts-pipeline" "fr3c_voice.tts_service" \
    "${LOG_BASE}/fr3c-tts-pipeline.sock" \
    "cd '${ROOT}' && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH='${ROOT}/src/fr3c_voice' \
     '${ROOT}/.venv-tts/bin/python' -m fr3c_voice.tts_service \
     --trace '${LOG_BASE}/service-trace.jsonl'"
  if [ -n "${SPEAKER}" ] && [ -d /dev/snd ]; then
    amixer -c 2 sset Speaker "${SPEAKER}%" >/dev/null 2>&1 \
      && echo "   扬声器音量设为 ${SPEAKER}%"
  fi
else
  echo "== 2/3 跳过语音管线（--no-speech）"
fi

echo "== 3/3 对话页状态"
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="${ROOT}/src/fr3c_voice" \
  /usr/bin/python3 -m fr3c_voice.dialogue_gui --workspace "${ROOT}" --validate-only

if [ "${CHECK_ONLY}" = "1" ]; then
  echo "（--check-only：未打开窗口）"
  exit 0
fi

echo
echo "打开对话页。关闭窗口或按 Ctrl+C 将平滑关闭全部相关进程。"
echo
# Give the GUI its own process group.  Ctrl+C is handled here and converted to
# SIGTERM so Tk can run its normal close path before the services are stopped.
setsid "${ROOT}/scripts/voice/start_dialogue_gui.sh" "$@" &
GUI_PID=$!
set +e
wait "${GUI_PID}"
gui_status=$?
set -e
GUI_PID=""
exit "${gui_status}"
