# Source this file in a clean bash terminal.
# Avoid user-site NumPy/OpenCV shadowing Ubuntu's ROS-compatible packages.
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
_fr3c_camera_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$_fr3c_camera_ws/install/setup.bash"
unset _fr3c_camera_ws
