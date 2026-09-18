#!/usr/bin/env bash
# P5 end-to-end driver: PCM sweep -> pronunciation render -> summary.
#
# The acoustic calibration is a separate, explicit step because it makes noise
# and must respect whatever the bench is doing:
#
#   scripts/voice/p5_acoustic.py calibrate --output <run>
#   scripts/voice/p5_acoustic.py report --workdir <run>/bench --output <run> \
#       --calibration <run>/calibration.json
#
# The load levels are separate too, because `qwen` needs the P3 service running
# and `full` needs the human-operated stack:
#
#   scripts/voice/p5_load.py --level tts  --candidate <name> --output <run>/load_tts
#   scripts/voice/start_qwen_service.sh          # then:
#   scripts/voice/p5_load.py --level qwen --candidate <name> --output <run>/load_qwen
#
# Usage: scripts/voice/run_p5_all.sh [run-dir]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

RUN="${1:-evidence/voice/P5/$(date +%Y%m%d)}"
PY="${ROOT}/.venv-tts/bin/python"
mkdir -p "${RUN}"

echo "== 1/3 PCM sweep -> ${RUN}/bench"
scripts/voice/run_p5_bench.sh "${RUN}/bench"

echo "== 2/3 pronunciation corpus -> ${RUN}/pronunciation"
PYTHONDONTWRITEBYTECODE=1 "${PY}" scripts/voice/p5_pronunciation.py --output "${RUN}/pronunciation"

echo "== 3/3 summary"
PYTHONDONTWRITEBYTECODE=1 python3 scripts/voice/p5_summarize.py "${RUN}"

echo
echo "run complete: ${RUN}"
echo "next (noisy, run deliberately): scripts/voice/p5_acoustic.py calibrate --output ${RUN}"
echo "next (needs P3 service):        scripts/voice/start_qwen_service.sh && scripts/voice/p5_load.py --level qwen ..."
