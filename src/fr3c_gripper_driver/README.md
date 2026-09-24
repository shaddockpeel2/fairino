# FR3-C / YDI-J22 gripper driver

This package exposes the YDI-J22 through the ROS 2
`control_msgs/action/GripperCommand` action:

```text
/ydi_j22_gripper_controller/gripper_cmd
```

The project enables two command states:

| Project command | Requested pulse | Effective 50 kHz pulse | Meaning |
| ---: | ---: | ---: | --- |
| `0.027 m` | `820 us` | `820 us` | grasp target |
| `0.060 m` | `1300 us` | `1300 us` | pre-open/release |

The former 26 mm/31-degree bench command produced an 800 us pulse at 10 kHz.
At 50 kHz, one sample is 20 us, so the new close command produces a pulse one
sample wider. The 27 mm aperture is an open-loop estimate based on the former
26 mm/800 us and 60 mm/1300 us calibration points; measure the physical grip
before relying on it. Any other ROS position is rejected; there is no sweep
or intermediate-position command in the production interface.

## Output behavior

The FT232H backend is encapsulated in `ft232h_servo.py`. Every accepted goal:

1. opens FT232H asynchronous bit-bang on AD7;
2. writes one complete 0.8 s, 50 Hz waveform (40 frames / 40000 bytes) for
   every pre-open, grasp, and release command;
3. waits for the bounded waveform to drain;
4. writes AD7 low and closes the controller.

There is no background writer, queued refill loop, or unbounded PWM stream.
This mirrors the better-performing standalone `gripper_ft232h.py` behavior and
prevents old waveform chunks from accumulating and executing later.

The gripper has no position, force, current, temperature, or stall feedback.
Action position is therefore an open-loop estimate, effort is unknown, and a
positive `max_effort` is rejected. A successful result remains a commanded
estimate, not proof that the fingers physically held the requested aperture.
Startup publishes the configured 60 mm
release estimate for MoveIt but emits no PWM.

## Build and mock test

```bash
cd /home/industio/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3c_gripper_driver \
  fairino_description fairino3mt_v6_moveit2_config --symlink-install
source install/setup.bash
ros2 launch fr3c_gripper_driver gripper.launch.py
```

In another sourced terminal:

```bash
/home/industio/fairino_ws/scripts/gripper/command_gripper.sh close
/home/industio/fairino_ws/scripts/gripper/command_gripper.sh open
```

`close` sends the 27 mm target and `open` sends the 60 mm target. A numeric
value is accepted by the shell helper, but the action server rejects anything
other than exactly `0.027` or `0.060` m.

## Real FT232H operation

Real output requires the parameter and environment-variable safety gates. Merely
starting the node opens no FTDI device and emits no PWM.

```bash
export FR3C_ENABLE_FT232H_PWM=I_UNDERSTAND_OPEN_LOOP_PWM
/home/industio/fairino_ws/scripts/gripper/run_ft232h_driver.sh
```

For a single monitored two-state check:

```bash
/home/industio/fairino_ws/scripts/gripper/verify_paced_sequence.sh \
  I_AM_MONITORING_GRIPPER
```

That script sends one close followed by one open command only. It performs no
cycling, sweep, or delayed hold. The combined MoveIt/camera wrapper uses the
same driver and can be forced to mock mode with `FR3C_GRIPPER_BACKEND=mock`.

Diagnostics remain at `/diagnostics`. `frames_written` and `bytes_written` are
cumulative completed finite bursts, not evidence of a background output loop.
