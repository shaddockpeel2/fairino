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
`passed=false`. A P5 process initially accepts only a passing gate generated
within the previous hour, then remembers that exact gate for the lifetime of
the running stack. Every operator-authorized segment still checks that the
file remains present, unchanged and passing. Restarting the full stack
invalidates the old gate and runs a new P6 snapshot; the one-hour limit applies
only to first acceptance, not to an already accepted running session.

## P5 one-shot RGB box task

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
demo. It copies the current stable P3 top pose and matching nominal size. P3
classifies either a 30 x 30 x 30 mm cube or a 60 x 30 x 30 mm rectangular box,
estimates a top-edge/long-axis yaw, and publishes both with the same timestamp.
The task uses the P4 parameters directly:
10 mm top clearance, 15 mm grasp depth, 50 mm lift, 60 mm pre-open and 26 mm
close. The old `mtc_node` remains available only as an upstream example.

Before starting, review `config/first_grasp.yaml`, especially the target bounds,
dynamic-size input, downward nominal tool orientation, and target-yaw alignment
offset. Cubes use 90-degree yaw symmetry; rectangular boxes use 180-degree yaw
symmetry. Each equivalent orientation is fully preview-planned from the same
start state, then the complete feasible route with the shortest unweighted
six-axis joint-space path is selected. Joint 6 has no additional travel or
limit-margin preference; MoveIt's URDF limits, collision checks, and full-path
requirements still apply. Keep `execute:=false` for the first review:

```bash
ros2 launch fairino_mtc_demo first_grasp.launch.py \
  task_config:=/absolute/path/to/measured_first_grasp.yaml
ros2 service call /fr3c_first_grasp/start std_srvs/srv/Trigger '{}'
```

The plan-only run adds the RViz-ground collision slab and correctly sized,
oriented visual box, plans the routed approach,
requires complete vertical descend/lift Cartesian paths, previews the attached
cube lift in RViz, and writes target/trajectory/return evidence under
`evidence/P5/runs`. It never commands the gripper or arm.

P3 continuously tracks the scene. Its `object_size` topic is consumed together
with `top_pose`; mismatched timestamps are rejected. A top-point axis ratio at
or below 1.25 is a cube, at or above 1.55 is a rectangular box, and the
ambiguous band is rejected. Eight accepted observations establish a
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
The manual `/start` flow remains one target at a time:

```bash
ros2 launch fairino_mtc_demo first_grasp.launch.py \
  task_config:=/absolute/path/to/measured_first_grasp.yaml execute:=true

# Explicitly pre-opens, previews all three paths, and moves to the approach pose.
ros2 service call /fr3c_first_grasp/start std_srvs/srv/Trigger '{}'

# Call only after inspecting the 10 mm-above-top alignment.
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

This service plans to the pose 10 mm above the object, descends vertically to
the final grasp pose, closes the gripper, and performs the existing vertical
lift. It uses no fixed delay, and every operation still waits for the
preceding operation to report success. Any planning, execution, gripper, or
P6 failure stops the remaining sequence. In executable four-color sorting,
this service runs a batch in red-yellow-blue-green order: a target whose full
route cannot be planned before arm motion is skipped, while any failure after
arm motion starts aborts the batch. After every return home it reacquires the
scene, and completes only after `actionable_cube_count` remains zero for 10
seconds. See `docs/four_color_sorting.md` for placement-slot behavior.

## Stage-2 color task actions

In `detection_mode:=colors` with `sort_enabled:=true`, the node also exposes
four single-request actions using `fairino_msgs/action/PickColor`:

- `/fr3c_first_grasp/pick_hold`: approach, descend, open-loop close, and lift;
  success leaves the cube held and does not claim sensor-verified grasping.
- `/fr3c_first_grasp/pick_sort`: performs the same grasp, places only that
  selected color in its configured A4 region, releases, and returns home.
- `/fr3c_first_grasp/pick_place`: performs the same grasp but places into a
  zone named by the goal instead of the color-mapped region. `zone` is `A` for
  the first free slot of region A, or `A3` for one explicit 1-based slot.
- `/fr3c_first_grasp/pick_stack`: grasps the cube of `color` and stacks it on
  the cube of `support_color`. Requires `stack_enabled:=true`. The placement
  target is the lower cube's measured top face, not a calibrated zone, and the
  upper cube is rotated so both top faces share one yaw. Two-cube stacks only.

Each goal first calls the perception `set_target_color` service. The returned
filter generation invalidates every older pose, and the task accepts only a
fresh stable pose whose color and generation match the goal. Zero matching
physical targets, two or more matching targets, stale data, task busy, a held
object, planning failure, execution failure, and cancellation have distinct
result codes. A task is never silently retried.

`pick_hold` and `pick_sort` must leave `zone` empty; a non-empty `zone` is
rejected with `INVALID_ZONE` so each action keeps exactly one meaning. Only
`pick_place` accepts a zone, and it never falls back to the color-mapped region:
an unknown region or an out-of-range slot returns `INVALID_ZONE`, and an
occupied slot returns `ZONE_UNAVAILABLE`. Both are decided before any
perception wait or arm motion, so they cannot move the robot. A zone is a
calibrated A4 region name, so `pick_place(red, zone:=D)` is a valid request and
is not checked against the color.

Only `pick_stack` accepts `support_color`, and it must differ from `color`.
Every other action rejects a non-empty `support_color` with
`INVALID_SUPPORT_COLOR`. `pick_stack` first stabilizes the requested source
colour, then locks both source and support from that status's same-frame
`detected_targets` list. It never re-observes either coordinate after motion
starts. Before any motion it also refuses a pair that is not two cubes lying side by side
(`STACK_GEOMETRY_REJECTED`), including a lower cube that already carries a cube,
so it cannot silently build a three-high tower. The lower cube stays a planning
obstacle for the whole task and is cleared afterwards.

`pick_stack` is the one action that promises the task slot back whatever
happens: a failure after motion started triggers a return-home trajectory, the
final state is startable, and the next goal can be sent immediately. If the
failure happened after closing the gripper, the next goal returns
`HOLDING_OBJECT` until the operator removes the cube, exactly like `pick_hold`.

The placement rotation is bounded by `maximum_stack_placement_rotation_rad`
(default 45 degrees). A square top face gives four equivalent grasps whose yaws
differ by 90 degrees, so the nearest aligned placement is at most half of that
symmetry step away. A candidate above the allowance is dropped before
planning; if all four are dropped the pair is reported unstackable while the arm
is still at home.

Slot occupancy is read from the live MoveIt planning scene: a slot counts as
occupied when a known obstacle lies within `zone_slot_clearance_m` of it. No
call counter is kept, so clearing the table and restarting the stack restores
the correct occupancy automatically. The active target cube and the ground slab
are never treated as slot obstacles.

Start with planning only, then send one goal:

```bash
cd /home/industio/fairino_ws
FR3C_FIRST_GRASP_EXECUTE=false scripts/run_moveit_camera_rviz.sh detection_mode:=colors
scripts/grasp/send_color_task.sh hold red
scripts/grasp/send_color_task.sh sort yellow
scripts/grasp/send_color_task.sh place blue A
scripts/grasp/send_color_task.sh place red D3
scripts/grasp/send_color_task.sh stack red blue
```

The calibrated zones and their slot coordinates are launch parameters, so they
can be inspected without guessing:

```bash
ros2 param get /fr3c_first_grasp place_zone_names
ros2 param get /fr3c_first_grasp place_zone_a_slots_m
```

Set `FR3C_FIRST_GRASP_EXECUTE=true` only after inspecting the full planned
route and completing the existing P6/site safety checks. Action cancellation
stops active MoveIt/gripper goals but deliberately does not open or detach a
possibly held object.

After a successful `pick_hold`, physically place/remove the cube and make sure
the gripper no longer holds it. Then explicitly synchronize the software-held
state before sending another Action:

```bash
ros2 service call /fr3c_first_grasp/acknowledge_object_released \
  std_srvs/srv/Trigger '{}'
```

This acknowledgement sends no arm or gripper command. It removes the previous
attached/world cube from the MoveIt planning scene, clears the previous Action's
perception color filter back to all colors, and returns the task to
`waiting_for_target`. Calling it before the object is physically clear is
unsafe because the next task begins by opening the gripper.
