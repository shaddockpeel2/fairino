#!/usr/bin/env bash
# Build the isolated P5 TTS environment.
#
# The reference project's sherpa-onnx 1.13.6 runtime is reused by symbolic link
# so the two projects cannot drift apart and 48 MiB of native libraries are not
# duplicated.  The reference tree itself stays read-only: nothing is written
# there.  Unrelated reference dependencies (rknnlite, serial, pyftdi, rkllm) are
# deliberately NOT linked, so this environment cannot collide with the YOLO
# environment that pins rknnlite 2.3.2.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REFERENCE="${FR3C_QWEN_ASSET_ROOT:-/home/industio/DofbotPro_JetsonNano_0106}"
REFERENCE_SP="${REFERENCE}/dofbot/.venv/lib/python3.10/site-packages"
VENV="${ROOT}/.venv-tts"
SP="${VENV}/lib/python3.10/site-packages"

if [ ! -d "${REFERENCE_SP}/sherpa_onnx" ]; then
  echo "reference sherpa_onnx not found under ${REFERENCE_SP}" >&2
  exit 1
fi

if [ ! -x "${VENV}/bin/python" ]; then
  python3 -m venv --system-site-packages "${VENV}"
fi

# numpy must come from the system site-packages of the interpreter, exactly as
# the reference venv resolves it.
python3 - <<'PY'
import numpy, sys
print(f"system numpy {numpy.__version__} from {numpy.__file__}")
PY

linked=0
skipped=0
for name in sherpa_onnx sherpa_onnx.libs sherpa_onnx_bin-1.13.6.dist-info \
            sherpa_onnx_core-1.13.6.dist-info sherpa_onnx-1.13.6.dist-info \
            _soundfile.py soundfile.py _soundfile_data soundfile-0.14.0.dist-info; do
  source="${REFERENCE_SP}/${name}"
  target="${SP}/${name}"
  if [ ! -e "${source}" ]; then
    echo "  skip (absent in reference): ${name}"
    skipped=$((skipped + 1))
    continue
  fi
  if [ -L "${target}" ]; then
    rm -f "${target}"
  elif [ -e "${target}" ]; then
    echo "  keep (already present): ${name}"
    continue
  fi
  ln -s "${source}" "${target}"
  echo "  linked: ${name}"
  linked=$((linked + 1))
done

echo "linked ${linked}, skipped ${skipped}"

"${VENV}/bin/python" - <<'PY'
import numpy
import sherpa_onnx

print("python", __import__("sys").version.split()[0])
print("sherpa_onnx", sherpa_onnx.__version__)
print("numpy", numpy.__version__)
required = ("OfflineTtsVitsModelConfig", "OfflineTtsMatchaModelConfig",
            "OfflineTtsKokoroModelConfig", "OfflineTtsModelConfig", "OfflineTtsConfig")
missing = [name for name in required if not hasattr(sherpa_onnx, name)]
if missing:
    raise SystemExit(f"sherpa_onnx is missing required config classes: {missing}")
print("tts config classes: ok")
PY
