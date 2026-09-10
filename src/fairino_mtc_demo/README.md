# FAIRINO FR3-C / FR3MT MTC demo

This package was migrated from:

`/home/industio/ros2_robotarm/src/frcobot_ros2-3.0.0_RobotV3.7.7/fairino_mtc_demo`

FAIRINO support confirmed that the ROS model name `fairino3mt` corresponds to
the FR3-C used by this project. The migrated demo therefore uses:

- robot: `fairino3mt_v6_robot`
- MoveIt package: `fairino3mt_v6_moveit2_config`
- planning group: `fairino3mt_v6_group`
- controller: `fairino3mt_controller`
- tip frame: `grasp_frame` (YDI-J22 flexible-finger closing tip; practical v2 is
  `[-0.847397,3.027928,178.0] mm` from `flange_link`)

## Build

To avoid high memory consumption on the RK3588, build this package with one
worker:

```bash
source /opt/ros/humble/setup.bash
cd /home/industio/fairino_ws
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --packages-select fairino_mtc_demo \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
```

## Mock planning

Start the fake-hardware environment:

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fairino_mtc_demo mtc_demo_env.launch.py
```

For a headless test, append `use_rviz:=false`.

In a second terminal, plan one forward task without executing it:

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fairino_mtc_demo mtc_demo_app.launch.py
```

Execution is deliberately disabled by default. Mock execution must be enabled
explicitly:

```bash
ros2 launch fairino_mtc_demo mtc_demo_app.launch.py \
  execute:=true cycle_count:=1
```

## Current limitations

- `fairino3_mt_v6.urdf` has no robot-link collision geometry. The current plan
  result is not collision-safe and must never be used as a true-robot gate.
- The demo only attaches and detaches a planning-scene object. It does not
  command a physical gripper.
- The collision object pose and the 2--4 mm Cartesian smoke-test distances are
  placeholders, not calibrated FR3-C grasp parameters.
- The current MoveIt configuration uses `mock_components/GenericSystem`.
- Robot V3.9.2 hardware execution remains disabled until the ros2_control
  lifecycle, joint-name mapping, collision model, TCP/load and safety gates are
  independently verified.
