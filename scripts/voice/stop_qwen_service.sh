#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="${workspace}/src/fr3c_voice:${PYTHONPATH:-}"
runtime="${workspace}/evidence/voice/P3/runtime"
service_socket="${runtime}/fr3c-qwen-service.sock"
worker_socket="${runtime}/qwen3vl-worker.sock"

is_qwen_pid() {
  local pid="$1" command
  [ -r "/proc/${pid}/cmdline" ] || return 1
  command="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null)" || return 1
  case "${command}" in
    *fr3c_voice.service*"--workspace ${workspace}"*) return 0 ;;
    *qwen3_vl_worker*"${worker_socket}"*) return 0 ;;
    *) return 1 ;;
  esac
}

find_qwen_pids() {
  local proc_dir pid
  for proc_dir in /proc/[0-9]*; do
    pid="${proc_dir##*/}"
    is_qwen_pid "${pid}" && printf '%s\n' "${pid}"
  done
  return 0
}

# Prefer the service protocol so the controller can reap its worker cleanly.
if [ -S "${service_socket}" ]; then
  /usr/bin/python3 -m fr3c_voice.client shutdown \
    --workspace "${workspace}" "$@" >/dev/null 2>&1 || true
fi

for _ in 1 2 3 4 5; do
  [ -z "$(find_qwen_pids)" ] && break
  sleep 1
done

# If the service already died, its worker can outlive it in a separate session.
# Signal only processes whose full command line identifies this workspace.
remaining="$(find_qwen_pids)"
if [ -n "${remaining}" ]; then
  while IFS= read -r pid; do
    [ -n "${pid}" ] || continue
    is_qwen_pid "${pid}" || continue
    kill -TERM "${pid}" 2>/dev/null || true
    kill -CONT "${pid}" 2>/dev/null || true
  done <<< "${remaining}"
fi

for _ in 1 2 3 4 5; do
  [ -z "$(find_qwen_pids)" ] && break
  sleep 1
done

remaining="$(find_qwen_pids)"
if [ -n "${remaining}" ]; then
  while IFS= read -r pid; do
    [ -n "${pid}" ] || continue
    is_qwen_pid "${pid}" && kill -KILL "${pid}" 2>/dev/null || true
  done <<< "${remaining}"
fi

if [ -z "$(find_qwen_pids)" ]; then
  [ ! -e "${service_socket}" ] || unlink "${service_socket}"
  [ ! -e "${worker_socket}" ] || unlink "${worker_socket}"
  [ ! -e "${worker_socket}.pid" ] || unlink "${worker_socket}.pid"
  echo "Qwen 服务与 worker 已停止"
else
  echo "Qwen 进程未能完全停止" >&2
  exit 1
fi
