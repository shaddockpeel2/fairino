#!/usr/bin/env bash
set -eo pipefail

# Rebuild a clean ROS library environment for RViz on this board.
if [[ "${FR3C_MOVEIT_CAMERA_CLEAN_ENV:-0}" != "1" ]]; then
  exec env -u LD_LIBRARY_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH \
    FR3C_MOVEIT_CAMERA_CLEAN_ENV=1 \
    bash --noprofile --norc "$0" "$@"
fi

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

# Select exactly one perception scene before starting the hardware stack.
detection_mode="${FR3C_DETECTION_MODE:-blue}"
launch_args=()
for argument in "$@"; do
  case "$argument" in
    detection_mode:=*) detection_mode="${argument#detection_mode:=}" ;;
    *) launch_args+=("$argument") ;;
  esac
done
case "$detection_mode" in
  blue) ;;
  colors) ;;
  yolo)
    yolo_site_packages="$workspace/.venv-yolo/lib/python3.10/site-packages"
    if [[ ! -f "$yolo_site_packages/rknnlite/api/rknn_lite.py" ||
          ! -f "$yolo_site_packages/ruamel/yaml/__init__.py" ]]; then
      echo "ERROR: run scripts/setup_yolo_runtime.sh before starting YOLO mode." >&2
      exit 2
    fi
    export PYTHONPATH="$yolo_site_packages${PYTHONPATH:+:$PYTHONPATH}"
    ;;
  *)
    echo "ERROR: detection_mode must be blue, colors or yolo, got '$detection_mode'." >&2
    exit 2
    ;;
esac

existing_nodes="$(ros2 node list 2>/dev/null || true)"
for node in \
  /controller_manager \
  /move_group \
  /ydi_j22_gripper \
  /perception/blue_cube_localizer \
  /fr3c_first_grasp; do
  if grep -qx "$node" <<<"$existing_nodes"; then
    echo "ERROR: 检测到已有 $node；请先关闭旧抓取栈，避免同名节点和重复控制源。" >&2
    exit 3
  fi
done

cd "$workspace"
ros2 run fr3c_camera dabai_camera_preflight \
  --serial "${FR3C_CAMERA_SERIAL:-CH82C5201C9}"

echo "WARNING: this starts the current ros2_control configuration, which uses the real FAIRINO hardware plugin." >&2
gripper_backend="${FR3C_GRIPPER_BACKEND:-ft232h}"
if [[ "$gripper_backend" == "ft232h" ]]; then
  # This operator-facing combined launcher intentionally arms the real
  # gripper. The lower-level gripper launcher remains double-gated so that
  # starting it directly cannot emit hardware PWM by accident.
  export FR3C_ENABLE_FT232H_PWM=I_UNDERSTAND_OPEN_LOOP_PWM
  gripper_hardware_output_enabled='true'
  echo 'WARNING: FT232H gripper PWM is ARMED; the first accepted gripper goal will move it.' >&2
else
  gripper_hardware_output_enabled='false'
fi

exec ros2 launch fairino_mtc_demo full_first_grasp.launch.py \
  detection_mode:="$detection_mode" \
  sort_enabled:="${FR3C_SORT_ENABLED:-$([[ "$detection_mode" == colors ]] && echo true || echo false)}" \
  stack_enabled:="${FR3C_STACK_ENABLED:-$([[ "$detection_mode" == colors ]] && echo true || echo false)}" \
  gripper_backend:="$gripper_backend" \
  gripper_hardware_output_enabled:="$gripper_hardware_output_enabled" \
  start_blue_cube_localizer:="${FR3C_START_BLUE_CUBE_LOCALIZER:-true}" \
  start_first_grasp:="${FR3C_START_FIRST_GRASP:-true}" \
  first_grasp_execute:="${FR3C_FIRST_GRASP_EXECUTE:-true}" \
  run_p6_preflight:="${FR3C_RUN_P6_PREFLIGHT:-true}" \
  "${launch_args[@]}"
