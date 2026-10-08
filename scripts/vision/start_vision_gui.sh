#!/usr/bin/env bash
set -eo pipefail
vision_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# No camera or robot startup: inherit the domain of the camera being reused.
export PYTHONPATH="${vision_workspace}/src/fr3c_voice:${PYTHONPATH:-}"
exec /usr/bin/python3 -m fr3c_voice.edge_vision_page --workspace "${vision_workspace}" "$@"
