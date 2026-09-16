#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
wheel="$workspace/third_party/rknn/rknn_toolkit_lite2-2.3.2-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl"
expected_sha256='821e80c95e6838308c133915660b1a6ae78bb8d079b2cbbd46a02dae61192d33'
ruamel_archive="$workspace/third_party/rknn/ruamel_runtime_py310_aarch64.tar.gz"
ruamel_sha256='08132e52d754392d97dc3c8ce5beb54c9256c7ba4df1f9ea2a1033fd8de24da6'

python3 - <<'PY'
import platform
import sys
if sys.version_info[:2] != (3, 10) or platform.machine() != 'aarch64':
    raise SystemExit('YOLO runtime requires AArch64 Python 3.10 on this RK3588 board')
PY

actual_sha256="$(sha256sum "$wheel" | cut -d ' ' -f 1)"
if [[ "$actual_sha256" != "$expected_sha256" ]]; then
  echo "ERROR: bundled RKNNLite wheel SHA-256 mismatch" >&2
  exit 1
fi
if [[ "$(sha256sum "$ruamel_archive" | cut -d ' ' -f 1)" != "$ruamel_sha256" ]]; then
  echo "ERROR: bundled ruamel runtime SHA-256 mismatch" >&2
  exit 1
fi

python3 -m venv --system-site-packages "$workspace/.venv-yolo"
tar -xzf "$ruamel_archive" -C "$workspace/.venv-yolo/lib/python3.10/site-packages"
"$workspace/.venv-yolo/bin/python" -m pip install \
  --no-index --no-deps --no-cache-dir "$wheel"
"$workspace/.venv-yolo/bin/python" - <<'PY'
import ruamel.yaml
from rknnlite.api import RKNNLite
print('Workspace-local RKNNLite runtime is ready.')
PY
