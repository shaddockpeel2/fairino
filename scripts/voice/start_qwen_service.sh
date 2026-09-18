#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
set -u
export PYTHONPATH="${workspace}/src/fr3c_voice:${PYTHONPATH:-}"
exec /usr/bin/python3 -m fr3c_voice.service --workspace "${workspace}" "$@"
