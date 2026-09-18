#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
set -u
export PYTHONPATH="${workspace}/src/fr3c_voice:${PYTHONPATH:-}"
knowledge_python="${FR3C_KNOWLEDGE_PYTHON:-/home/DofbotPro_JetsonNano_0106/.venv-rk3588/bin/python}"
if [[ ! -x "${knowledge_python}" ]]; then
  echo "warning: pinned knowledge Python is missing; RAG will degrade to ordinary chat" >&2
  knowledge_python=/usr/bin/python3
fi
exec "${knowledge_python}" -m fr3c_voice.service --workspace "${workspace}" "$@"
