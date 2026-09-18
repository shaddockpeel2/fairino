#!/usr/bin/env bash
# Foreground streaming speech service: owns the speaker for the whole session.
#
# This is a separate terminal on purpose. Two processes must never open the
# playback device at once, and a foreground service makes it obvious when the
# speaker is taken. Ctrl+C stops it and releases the device.
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${workspace}"

if [ ! -d /dev/snd ]; then
  echo "错误：/dev/snd 不存在，请在真实终端中运行。" >&2
  exit 1
fi

python="${workspace}/.venv-tts/bin/python"
if [ ! -x "${python}" ]; then
  echo "错误：找不到 ${python}" >&2
  exit 1
fi

runtime="${workspace}/evidence/voice/P6/runtime"
mkdir -p "${runtime}"

echo "候选: matcha-icefall-zh-baker | 设备: plughw:CARD=Device,DEV=0"
echo "socket: ${runtime}/fr3c-tts-pipeline.sock"
echo "模型加载与预热约 8 秒..."
exec env PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="${workspace}/src/fr3c_voice" \
  "${python}" -m fr3c_voice.tts_service \
  --trace "${runtime}/service-trace.jsonl" "$@"
