#!/usr/bin/env bash
# P5 PCM baseline sweep: every main candidate, same script, same corpus, same
# affinity, so the comparison is apples-to-apples.  Audio is never played.
#
# Usage: scripts/voice/run_p5_bench.sh <evidence-run-dir> [extra p5_bench args]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

OUT="${1:?usage: run_p5_bench.sh <evidence-run-dir> [extra args]}"
shift || true
PY="${ROOT}/.venv-tts/bin/python"
REPEATS="${P5_REPEATS:-5}"
COLD="${P5_COLD_REPEATS:-2}"
AFFINITY="${P5_AFFINITY:-big}"
THREADS="${P5_THREADS:-3}"   # 3 threads measured fastest on the A76 cluster; see evidence/voice/P5/<run>/threads

CANDIDATES=(
  matcha-icefall-zh-baker
  vits-zh-aishell3
  vits-melo-tts-zh_en
  vits-zh-hf-fanchen-C
  kokoro-multi-lang-v1_1
)

mkdir -p "${OUT}"
echo "run dir: ${OUT}"
echo "repeats=${REPEATS} cold=${COLD} affinity=${AFFINITY} threads=${THREADS}"

for candidate in "${CANDIDATES[@]}"; do
  target="${OUT}/${candidate}"
  if [ -f "${target}/summary.json" ]; then
    echo "== ${candidate}: summary already present, skipping"
    continue
  fi
  echo "== ${candidate}"
  PYTHONDONTWRITEBYTECODE=1 "${PY}" scripts/voice/p5_bench.py \
    --candidate "${candidate}" \
    --output "${target}" \
    --repeats "${REPEATS}" \
    --cold-repeats "${COLD}" \
    --affinity "${AFFINITY}" \
    --threads "${THREADS}" \
    "$@" 2>&1 | tee "${target}.log"
done

echo "== sweep complete"
