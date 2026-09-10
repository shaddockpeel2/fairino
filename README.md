# FAIRINO ROS 2 workspace (RK3588 / ARM64)

This is an isolated ROS 2 Humble colcon workspace for a FAIRINO controller at
`192.168.58.2`. It does not modify or overlay the other ROS 2 workspaces on
this board.

## Contents

- `src/fairino_msgs`: FAIRINO ROS interfaces.
- `src/fairino_hardware`: FAIRINO hardware/API package matching controller
  software V3.9.2.
- `src/fairino_hardware/libfairino/lib/libfairino.so.2.3.2`: SDK built natively
  for AArch64 from FAIRINO C++ SDK tag `v2.3.2-robot3.9.2`.
- `src/orbbec_vendor`: ARM64 Orbbec ROS 2 wrapper imported from the vendor archive.
- `src/fr3c_camera`: DaBai DCW2 bench bringup, RGB-D validation, and
  camera-frame projection interfaces for later hand-eye integration.

## Build

Open a clean shell and build only this workspace:

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

The official V3.9.2 source is not clean under the ROS lint suites, so normal
builds disable style-only tests. This does not disable compilation checks.

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 run fairino_hardware ros2_cmd_server
```

The camera bench is deliberately isolated from the robot ROS graph by default:

```bash
cd /home/industio/fairino_ws
scripts/camera/run_bench.sh
```

The default camera is the DaBai DCW2 (`CH82C5201C9`) using a USB2-safe
640x480 RGB + aligned-depth profile. See
`docs/hardware/DaBai_DCW2_接入说明.md` for its verified configuration,
validation results, and remaining calibration work. The older
`P03_camera_bench.md` is retained as the discarded Gemini Pro test record.

The board interface connected to the controller must have an address in
`192.168.58.0/24` (currently `192.168.58.10/24`). The controller address is
defined as `192.168.58.2` in the FAIRINO source.

Do not source another project workspace in the same shell before building or
running this workspace unless an overlay is intentional. Do not upgrade this
board to Ubuntu 24.04 for this setup; ROS 2 Humble targets Ubuntu 22.04.

## Safety note

Connection and read-only SDK calls have been validated. The upstream V3.9.2
command server still emits compiler warnings in some motion-command paths.
Audit those paths and test at reduced speed in a safeguarded area before using
movement services.
