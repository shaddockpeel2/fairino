#!/usr/bin/env bash
set -eo pipefail
vision_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${vision_workspace}/install/setup.bash" ]]; then
  source "${vision_workspace}/install/setup.bash"
fi
set -u
export PYTHONPATH="${vision_workspace}/src/fr3c_camera:${PYTHONPATH:-}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
vision_python="${FR3C_EDGE_VISION_PYTHON:-${vision_workspace}/.venv-yolo/bin/python}"
if [[ ! -x "${vision_python}" ]]; then
  vision_python=/usr/bin/python3
fi
# Reuse the active camera and ROS domain; never start robot or camera here.
exec nice -n 10 "${vision_python}" -m fr3c_camera.edge_vision_node \
  --workspace "${vision_workspace}" "$@"
