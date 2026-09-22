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

If heartbeats stop for 0.5 seconds, or Servo reports collision halt,
singularity halt, or a joint-bound halt, the bridge stops Servo, deactivates
the execution controller, and releases control. A failed stop/release is
latched and blocks subsequent automatic motion.

## Stage four: one short pulse at a time

The stage-four runner is `tools/stage4_direction_pulse.py`. It does not send an
Action goal or run the tracking loop. Each invocation requests one tracking
lease, sends one base XY pulse, publishes zero commands, and releases the lease.
Speed is at most 0.005 m/s and the requested pulse is at most 0.5 s (2.5 mm
nominal). The red target must be unique and fresh. The runner also checks
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

Keep the block still for 30 seconds, then move it slowly in four directions,
diagonally and back. Test short occlusion, sustained loss and a second red block
in separate runs. These physical cases need an operator; software injection
must be labelled separately. A duration result of `OK` only proves the timer
completed, not that every acceptance scenario passed.

Real output uses fresh TCP TF to enforce the 80 mm radial XY boundary relative
to acquisition, 5 mm height drift and 0.05 rad orientation drift. Dry-run retains
command-integrated displacement; real execution uses measured displacement to
avoid prematurely reaching the boundary when ServoJ follows slowly. Camera TF
and TCP TF older than 0.5 seconds stop output. The observation timeout defaults to 0.40 seconds. For the measured camera
jitter, pass `observation_stale_s:=0.65` to the Servo launch; this sets both
perception and Action timeouts together. This demo setting does not meet the
original 0.25-second freshness requirement. Explicit invalid detections still
produce zero commands immediately; sustained loss terminates after 1 second. Speed remains 0.005 m/s, acceleration 0.03 m/s² and Servo
period 0.10 seconds. No Z or angular command is generated.

After a run verify `monitoring_manual_ready`. For a latched release fault, stop
other goals, inspect bridge/controller logs, correct the underlying controller
or communication issue and restart the control stack; do not clear the fault
while automatic motion is active. When ending the session, stop the tracking
launch after the Action result; remove the temporary scene object only if it
was absent before testing.
