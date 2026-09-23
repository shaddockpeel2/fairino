# FR3-C 2-D visual servo

This package implements stages two through five of
`docs/2D方块居中跟踪五步开发规划.md`. Dry-run remains the default. In stage
three, `execute=true` requests a heartbeat lease from the trajectory control
bridge, starts MoveIt Servo, transforms the camera-plane command into
`base_link`, and publishes only base XY translation.

Build and launch against an already running color camera:

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fairino_msgs fr3c_camera fr3c_visual_servo
source install/setup.bash
ros2 launch fr3c_visual_servo track_2d_dry_run.launch.py target_color:=red
```

Start a bounded 30-second dry-run goal in another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 action send_goal --feedback \
  /fr3c_visual_servo/track_2d fairino_msgs/action/TrackTarget2D \
  "{color: red, max_duration_s: 30.0}"
```

`Ctrl-C` asks the action server to cancel. Inspect only the isolated dry-run
outputs below:

```bash
ros2 topic echo /fr3c_visual_servo/dry_run/expected_twist
ros2 topic echo /fr3c_visual_servo/dry_run/status
ros2 topic echo /perception/color_target_2d/debug_image
```

The configured perception color and goal color must match. The status topic
contains a unique run ID, parameter snapshot, phase, expected command, and
dead-reckoned expected XY displacement for evidence correlation.

## Stage-three mock/fake wiring

First start the existing MoveIt/ros2_control stack. Its controller-spawn launch
also starts the unified trajectory/Servo control bridge. Then start Servo and
tracking:

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fr3c_visual_servo track_2d_servo.launch.py \
  target_color:=red execute:=true
```

`track_2d_servo.launch.py` defaults to `execute:=false`. Use `true` with fake/mock hardware for stage three, or with real hardware
after completing stage-four direction and stop verification. It never starts motion by itself; the
explicit Action goal shown above is still required.

The bridge enforces one owner at a time:

- `TRAJECTORY`: an existing MoveIt/RViz trajectory goal;
- `SERVO_TRACKING`: the tracking Action's renewable lease.

If heartbeats stop for 0.5 seconds or Servo reports a collision halt, the
bridge stops Servo, deactivates the dedicated Servo position controller, and
releases control.
Servo halts at singularities and joint bounds while the Action retains its
lease; tracking can resume when the visible target moves back into reach.
A failed stop/release is
latched and blocks subsequent automatic motion.

## P1: forward-position execution backend

P1 keeps the V1 `fr3c_servo.yaml`/JTC launch as a comparison baseline.  The
isolated `p1_servo_forward.launch.py` instead publishes six joint positions as
`std_msgs/Float64MultiArray` to
`fairino3mt_servo_position_controller`.  It starts no perception or tracking
Action and sends no motion by itself.

After the base MoveIt/ros2_control stack is running, start the P1 backend:

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fr3c_visual_servo p1_servo_forward.launch.py \
  publish_period:=0.020 low_latency_mode:=false
```

The supported period matrix is `0.020`, `0.010`, and `0.008` seconds.  The
launch also exposes `low_latency_mode`, `smoothing_filter_plugin`, and
`butterworth_filter_coeff`.  Run only one matrix point at a time and fully
restart this launch between profiles.

The guarded diagnostic performs a two-second zero-command phase and records
the forward command stream, joint/TCP feedback, ServoJ timing, post-zero
travel, release state, and automated P1 thresholds:

```bash
python3 tools/stage4_constant_twist_diagnostic.py x+ \
  --backend forward_position --duration-s 0.5 --operator-ready \
  --output evidence/P1_execution/forward_020_xplus_0p5s.json

python3 tools/stage4_constant_twist_diagnostic.py x+ \
  --backend forward_position --duration-s 2.0 --operator-ready \
  --output evidence/P1_execution/forward_020_xplus_2s.json

# Run only after the normal direction gates pass.
python3 tools/stage4_constant_twist_diagnostic.py x+ \
  --backend forward_position --duration-s 1.5 --cancel-after-s 0.25 \
  --operator-ready --output evidence/P1_execution/forward_cancel.json
python3 tools/stage4_constant_twist_diagnostic.py x+ \
  --backend forward_position --duration-s 1.5 \
  --heartbeat-timeout-after-s 0.25 --operator-ready \
  --output evidence/P1_execution/forward_heartbeat_timeout.json
python3 tools/stage4_constant_twist_diagnostic.py x+ \
  --backend forward_position --duration-s 1.5 \
  --publisher-exit-after-s 0.25 --operator-ready \
  --output evidence/P1_execution/forward_publisher_exit.json
```

These commands can move the real robot.  Do not run them without explicit
operator clearance, a working physical stop, synchronized ground collision
geometry, and the staged order in the V2 handoff document.  P1 is not passed
until the real 5 mm/s × 2 s displacement, speed realization, stopping travel,
ServoJ timing, error-free release, and operator smoothness gates all pass.

The stage-four/stage-five sections below document the retained V1 JTC baseline.
Do not resume those visual closed-loop procedures with the V2 bridge until the
P1 real-hardware gate passes and the launch is explicitly migrated to the
forward-position profile.

## Stage four: one short pulse at a time

The stage-four runner is `tools/stage4_direction_pulse.py`. It does not send an
Action goal or run the tracking loop. Each invocation requests one tracking
lease, sends one base XY pulse, publishes zero commands, and releases the lease.
The default pulse is 0.005 m/s for at most 0.5 s (2.5 mm nominal); an explicit
speed up to 0.015 m/s has been physically validated. Higher speeds up to
0.075 m/s require staged validation as described below. The red target must
be unique and fresh. The runner also checks
joint state, TCP and camera TF, an idle control bridge, and a table/ground
collision object in `/get_planning_scene` before acquiring control.

Start the existing real robot, MoveIt, camera, red perception and dry-run
Action stack first. Start only Servo for short pulses, avoiding duplicate
Action and perception nodes:

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fr3c_visual_servo track_2d_servo.launch.py \
  target_color:=red execute:=false \
  start_perception:=false start_action_server:=false \
  servo_publish_period:=0.10
```

After Servo starts, load the project's measured ground slab into both MoveIt
and Servo scenes:

```bash
python3 tools/stage4_ground_scene.py --require-servo-sync
```

Before any motion, confirm clear space for the arm, gripper, camera and cables,
check a working physical stop, confirm the calibrated camera TF and table
collision object in RViz, and confirm no other motion source is running. With
the operator at the stop, run zero speed first, then one direction per command:

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/stage4_zero_speed_test.py | tee evidence/stage4/zero_speed.json
python3 tools/stage4_zero_speed_test.py --watchdog-test \
  | tee evidence/stage4/zero_watchdog.json
python3 tools/stage4_direction_pulse.py x+ --operator-ready \
  --output evidence/stage4/base_plus_x.json
```

The scene tool reads the plane geometry from
`src/fairino_mtc_demo/config/first_grasp.yaml`; it does not use placeholder
table values. `--remove` removes that plane after tests if it was absent before.

Repeat with `x-`, `y+`, and `y-`. For image-direction checks, place the red
cube to the left, right, above, and below the center and run `left`, `right`,
`up`, or `down` separately. Those four modes apply the current proportional
camera-plane law through the camera-to-base TF, command only the selected image
axis, and report whether the corresponding pixel error decreased. A tiny
single-pulse shift can be inconclusive; inspect the before/after pixels and
repeat a direction only after confirming the previous lease released. Stop
direction tests if `motion_verdict` is `undertravel`: image direction cannot be
confirmed from motion near pixel noise. If the error grows, correct the camera
axis sign or TF before any further direction test.

The real ServoJ path responded poorly to 20 ms single-point trajectories. The
100 ms Servo period yielded about 0.6–0.7 mm TCP motion for a 0.5 s pulse,
while keeping the Twist input at or below 0.005 m/s. The controller still lags:
some motion occurs during the zero-command stop sequence. Review the measured
post-zero and post-release displacement rather than assuming an instant stop.

The JSON includes the command, target pixels, TCP and joint endpoints, a
100 ms-window TCP speed estimate, Servo/controller message counts, Servo status history, scene
object IDs, lease times, and final control state. `Ctrl-C` or a lost target
causes zero Twist output and lease cleanup. The zero-command
`--watchdog-test` stops lease heartbeats after a pulse. The following modes
exercise bounded in-motion stop paths, each with at most 0.5 s of commanded
motion:

```bash
python3 tools/stage4_direction_pulse.py x+ --operator-ready \
  --watchdog-during-pulse --output evidence/stage4/moving_watchdog.json
python3 tools/stage4_direction_pulse.py x- --operator-ready \
  --cancel-after-s 0.25 --output evidence/stage4/moving_cancel.json
python3 tools/stage4_direction_pulse.py x+ --operator-ready \
  --target-loss-after-s 0.25 --output evidence/stage4/moving_target_loss.json
```

The target-loss option injects invalid data inside the test runner; it does
not physically occlude the camera. The real camera was separately confirmed
to publish `target_not_found` when the red block left view. The [stage-four
record](/home/industio/fairino_ws/evidence/stage4/README.md) contains the
completed demo results and remaining live Action/occlusion checks. Remove the
temporary scene object after testing:

```bash
python3 tools/stage4_ground_scene.py --remove --require-servo-sync
```


## Stage five: real closed-loop tracking

The completed 0.015 m/s live test, measured results, and full cold-start
commands are recorded in
`docs/2d优化/2D方块居中跟踪收尾技术报告-20260922.md`.

With the robot, MoveIt and camera stack running, stop any old dry-run tracking
launch first to avoid duplicate Action/perception nodes. Launching alone sends
no goal:

```bash
source /opt/ros/humble/setup.bash
source /home/industio/fairino_ws/install/setup.bash
ros2 launch fr3c_visual_servo track_2d_servo.launch.py execute:=true
```

From the workspace in a sourced terminal, synchronize the measured ground
collision object, then start a bounded run with an operator at the robot:

```bash
python3 tools/stage4_ground_scene.py --require-servo-sync
python3 tools/stage5_track_acceptance.py --duration 30 --output evidence/stage5/hold.json
python3 tools/stage5_track_acceptance.py --duration 120 --output evidence/stage5/tracking.json
```

The client records Action feedback/results, actual published Twist commands,
Servo states, control states and parameter snapshots. Ctrl-C cancels the goal
and waits for its result. To reproducibly exercise the actual cancellation path:

```bash
python3 tools/stage5_track_acceptance.py --duration 10 --cancel-after 3 --output evidence/stage5/cancel.json
```

For the first run, put one red block **in view but outside the center region**
before sending the Action goal. Feedback should advance through `ACQUIRING`,
`INITIAL_CENTERING`, and `CENTER_HOLD`; the arm moves during
`INITIAL_CENTERING`. Startup centering must reach a 12 x 9 px half-width and
half-height region for three camera frames. Keep the block still while it
centers, then move it in four directions, diagonally and back. The action stays active in
`CENTER_HOLD` and resumes `TRACKING` when the block leaves the exit region.
Test short occlusion, sustained loss and a second red block
in separate runs. These physical cases need an operator; software injection
must be labelled separately. A duration result of `OK` only proves the timer
completed, not that every acceptance scenario passed.

The initial centering uses the same configured speed, acceleration, collision
checks, stale-image stop, and control lease as
normal tracking. If the block is not uniquely visible, the arm stays still in
`ACQUIRING` and the Action times out after 5 seconds. A block outside the
camera view provides no image direction for this 2-D controller.

Real output uses fresh TCP TF to enforce 5 mm height drift and 0.05 rad
orientation drift. There is no Action-relative XY travel limit. Dry-run retains
command-integrated displacement for diagnostics; real execution reports
measured displacement. Camera TF
and TCP TF older than 0.5 seconds stop output. The observation timeout defaults to 0.40 seconds. For the measured camera
jitter, pass `observation_stale_s:=0.65` to the Servo launch; this sets both
perception and Action timeouts together. This demo setting does not meet the
original 0.25-second freshness requirement. Explicit invalid detections still
produce zero commands immediately; sustained loss terminates after 1 second.
The default speed is 0.005 m/s, acceleration is 0.03 m/s², and the Servo
period is 0.10 seconds. No Z or angular command is generated.

### Faster tracking profile tested on 2026-09-22

The explicit 0.015 m/s profile was tested with the gripper joint state set to
the operator-confirmed 60 mm open aperture. At `servo_publish_period:=0.20`,
four base-axis pulses passed at 0.010 m/s and four more at 0.015 m/s.
Software cancel, injected target loss, and post-pulse heartbeat release also passed at
0.015 m/s. Each 0.015 m/s direction pulse lasted 0.25 s, requested 3.75 mm,
and moved 1.13-1.18 mm along the command. Servo status stayed zero and the
controller returned to manual-ready state after every pulse.

In a 90 s Action, initial image-error closing speed was 5.59 px/s versus
2.06 px/s in the preceding 0.005 m/s run: a measured 2.7x gain for these
different starting positions. The initial target reached `CENTER_HOLD` after
10.0 s of active centering versus 36.9 s previously. Slow manual cube motion
then caused repeated `TRACKING` and `CENTER_HOLD` transitions. The Action
ended normally with Servo status zero, 51.5 mm maximum measured relative XY
travel, and `monitoring_manual_ready`. See
`evidence/stage5/fast15_initial_then_move_20260922.json`.

The launch default remains 0.005 m/s. For a smoother 0.015 m/s tracking trial,
keep the target within the camera view and use the previously verified 0.10 s
Servo period with gentler 0.030 m/s² acceleration:

```bash
ros2 launch fr3c_visual_servo track_2d_servo.launch.py \
  target_color:=red execute:=true observation_stale_s:=0.40 \
  servo_publish_period:=0.10 max_speed_mps:=0.015 gain_mps:=0.240 \
  max_acceleration_mps2:=0.030
```

The earlier 0.015 m/s / 0.20 s profile remains the measured speed reference.
The 0.10 s smooth profile has not yet been tested as a continuous real Action.
MoveIt Servo emits one joint waypoint per period; with position-only joint
state and position-only trajectory points, the trajectory controller interpolates
linearly, so halving the period reduces the size and duration of each slope
change. It also reduces realized travel on this ServoJ path. Changing only
`publish_joint_velocities` would not enable cubic interpolation here because
the hardware exposes no joint velocity state for the start of each segment.

The controller accepts an explicit limit up to 0.075 m/s and rejects values
above that or acceleration above 0.090 m/s². The default remains 0.005 m/s.
The 0.075 m/s limit is five times the previously tested input limit, not a
measured fivefold TCP speed. At the 0.10 s Servo period, the ServoJ path may
deliver a smaller fraction of requested speed. Keep the collision scene and
joint states complete.

Before a 0.075 m/s tracking Action, validate 0.030, 0.045, 0.060, then
0.075 m/s with the bounded stage-four pulse tool, checking x+/x-/y+/y-,
stop behavior, TCP distance, and control release at each step. Speeds above
0.015 m/s require `--high-speed-validation` and have a 15 mm nominal pulse
distance limit. At 0.075 m/s, use a 0.20 s pulse. The continuous fast profile
is `max_speed_mps:=0.075 gain_mps:=1.200
max_acceleration_mps2:=0.090 servo_publish_period:=0.10`, after those physical
checks pass. Start with a short bounded Action and compare measured TCP speed,
stopping distance, and image error; the fast profile is not yet physically
validated.
`tools/stage5_track_acceptance.py` now records timestamped TCP positions in
`tcp_positions`; `tools/stage5_summarize.py` reports 100 ms window XY speed
statistics when those samples are present. Use those measurements to judge
whether the arm actually follows faster. The old radial displacement field
alone cannot measure true TCP path speed.

The 0.04 m/s field retest showed 22 nonzero-to-zero command transitions in
about 28 s, mostly from brief red-target detection failures, while the live
Servo period was still 0.20 s. The controller now bridges only a recent
single-frame detection glitch for at most 0.16 s and at most 5 mm nominal
travel. Repeated or stale-image failures command zero. Default
`lost_timeout_s=0` keeps the Action in zero-speed `LOST_HOLD` until recovery,
cancel, or its maximum duration; set a positive timeout to restore abort-on-loss
behavior. The perception node retains continuity across one bad frame after
target lock, and the stage-five recorder logs `perception_status` for review.
For the next short 0.04 m/s physical trial, restart the tracking launch with
`servo_publish_period:=0.10 gain_mps:=0.480 max_acceleration_mps2:=0.090`.
This combination has not yet been physically verified.

After a run verify `monitoring_manual_ready`. For a latched release fault, stop
other goals, inspect bridge/controller logs, correct the underlying controller
or communication issue and restart the control stack; do not clear the fault
while automatic motion is active. When ending the session, stop the tracking
launch after the Action result; remove the temporary scene object only if it
was absent before testing.
