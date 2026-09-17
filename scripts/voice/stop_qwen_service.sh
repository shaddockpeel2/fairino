#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="${workspace}/src/fr3c_voice:${PYTHONPATH:-}"
exec python3 -m fr3c_voice.client shutdown --workspace "${workspace}" "$@"
