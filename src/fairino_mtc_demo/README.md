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
  `[-0.847397,3.027928,184.614459] mm` from `flange_link`)

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

- Legacy `mtc_node` only attaches/detaches a placeholder cylinder and does not
  command a physical gripper. Do not use it for the first RGB grasp.
- `first_grasp_node` converts the RViz `base_link` XY grid at Z=0 into a
  MoveIt collision slab; target and TCP goals below that plane are rejected.
- The YDI-J22 has no position, force, or grasp feedback. Its action result and
  joint state are open-loop estimates, so lift remains manually gated.
- Real execution still requires the P6 load/clearance/single-command-source
  field checks even after a plan-only preview succeeds.

## P6 execution preflight

The P6 tool is read-only and consumes status exported by the existing
`fairino_hardware` connection. It does not start the general FAIRINO command
server or a second SDK client:

```bash
ros2 run fairino_mtc_demo p6_preflight \
  --config /home/industio/fairino_ws/src/fairino_mtc_demo/config/p6_preflight.yaml \
  --evidence-root /home/industio/fairino_ws/evidence/P6 \
  --confirm-workcell-clear \
  --confirm-lift-clearance-50mm \
  --confirm-cable-slack \
  --confirm-stop-method-ready \
  --confirm-no-external-motion-source
```

The supplied config records an operator-directed prototype waiver for payload
calibration. The controller's live payload is still captured in each report,
but does not block this first 5%-speed proof-of-concept cycle. Set
`payload.required: true`, measure/apply the complete installed end effector,
and enter its mass, flange-relative CoG and basis before accuracy/dynamics work
or higher-speed operation. Any failed check overwrites the execution gate with
`passed=false`. A P5 process initially accepts only a fresh passing gate, then
remembers that exact gate for the lifetime of the running stack. Every
operator-authorized segment still checks that the file remains present,
unchanged and passing. Restarting the full stack invalidates the old gate and
runs a new P6 snapshot; an arbitrary ten-minute session timeout is not used.

## P5 one-shot RGB cube task

For the physical first-grasp workflow, the workspace launcher starts the real
FAIRINO/MoveIt stack, camera, calibrated TF, FT232H gripper, P3 localizer, P5
service node and delayed P6 preflight together. P3, P5 and P6 are enabled by
default; no arm motion occurs until `/fr3c_first_grasp/start` is called:

```bash
cd /home/industio/fairino_ws
scripts/run_moveit_camera_rviz.sh
```

Optional environment switches are `FR3C_START_BLUE_CUBE_LOCALIZER`,
`FR3C_START_FIRST_GRASP`, `FR3C_FIRST_GRASP_EXECUTE` and
`FR3C_RUN_P6_PREFLIGHT`; set one to `false` only when intentionally disabling
that part of the stack.

`first_grasp_node` is the real-task replacement for the fixed-cylinder cyclic
demo. It copies the current stable P3 top pose and uses the P4 parameters directly:
15 mm top clearance, 15 mm grasp depth, 50 mm lift, 60 mm pre-open and 26 mm
close. The old `mtc_node` remains available only as an upstream example.

Before starting, review `config/first_grasp.yaml`, especially the target bounds
and explicit downward tool orientation. Keep `execute:=false` for the first review:

```bash
ros2 launch fairino_mtc_demo first_grasp.launch.py \
  task_config:=/absolute/path/to/measured_first_grasp.yaml
ros2 service call /fr3c_first_grasp/start std_srvs/srv/Trigger '{}'
```

The plan-only run adds the RViz-ground collision slab and visual cube, plans the routed approach,
requires complete vertical descend/lift Cartesian paths, previews the attached
cube lift in RViz, and writes target/trajectory/return evidence under
`evidence/P5/runs`. It never commands the gripper or arm.

P3 continuously tracks the scene. Eight accepted observations establish a
stable target, and later stable observations update that saved pose. A brief
rejected frame does not erase the rolling window; a long observation gap or a
20 mm target jump starts a new one. `valid` describes the current observation,
not merely whether an older pose has ever been saved.

`/fr3c_first_grasp/start` does not reset P3 and does not wait for perception to
become valid later. At the instant of the call it requires a current, fresh,
stable target and copies that pose into the task. Later camera updates cannot
move the accepted task target. If the gate is not ready, the call returns
`success=false` immediately and may be called again after the operator fixes
the scene. A planning or gripper pre-open failure before arm motion is also
retryable; failures after arm motion begins remain terminal for safety.

After a successful lift, `complete_holding` is ready for another explicit
`/start`. First physically remove/place the cube, finish any manual flange-button
motion, and wait for P3 to report a fresh stable table target. The next `/start`
then removes the previous trial's attached/world collision object, verifies the
new target is on the configured table plane, and locks the new snapshot. No P3
reset or grasp-stack restart is required between successful trials.

After reviewing the preview and completing the P6 field gates, enable execution.
There is no loop or automatic retry:

```bash
ros2 launch fairino_mtc_demo first_grasp.launch.py \
  task_config:=/absolute/path/to/measured_first_grasp.yaml execute:=true

# Explicitly pre-opens, previews all three paths, and moves to the approach pose.
ros2 service call /fr3c_first_grasp/start std_srvs/srv/Trigger '{}'

# Call only after inspecting the 15 mm-above-top alignment.
ros2 service call /fr3c_first_grasp/descend_and_close std_srvs/srv/Trigger '{}'

# The 26 mm result is open-loop. Call only after physically confirming the cube is held.
ros2 service call /fr3c_first_grasp/confirm_grip_and_lift std_srvs/srv/Trigger '{}'
```

`/fr3c_first_grasp/cancel` stops active motion and cancels the gripper action.
Failure or cancellation deliberately does not open the fingers, detach a held
cube, retry, or start another stage.

After the full path, alignment, and open-loop grip have been physically
validated, the same state machine can run without the two manual holds:

```bash
ros2 service call /fr3c_first_grasp/start_continuous std_srvs/srv/Trigger '{}'
```

This service uses no fixed delay. It pre-authorizes the descend and lift
transitions, while every operation still waits for the preceding operation to
report success. Any planning, execution, gripper, or P6 failure stops the
remaining sequence.
