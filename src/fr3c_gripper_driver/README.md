# FR3-C / YDI-J22 gripper driver

This ROS 2 package exposes the YDI-J22 as
`control_msgs/action/GripperCommand` at:

```text
/ydi_j22_gripper_controller/gripper_cmd
```

The current hardware has no position, force, current, temperature, or stall
feedback. The action result therefore reports an **estimated commanded
position**, never measured effort. Any positive `max_effort` is rejected.

## Current mapping

- ROS position: measured opposed-fingertip aperture, `0.000 m` closed to
  `0.130 m` open.
- FT232H pulse: `550 us` closed to `1650 us` open.
- Electrical limits: `500..2500 us`.

The driver uses the measured piecewise-linear aperture calibration:

| Aperture | Effective pulse |
| ---: | ---: |
| 0 mm | 550 us |
| 24 mm | 820 us |
| 60 mm | 1100 us |
| 100 mm | 1380 us |
| 110 mm | 1480 us |
| 130 mm | 1650 us |

Effective pulses are stated at the 10 us resolution of the 100 kHz bit-bang
stream. The drawing's 166 mm value is the maximum expanded outer envelope, not
the clear fingertip aperture and not evidence that it corresponds to 2500 us.
Do not change the operating endpoint to 2500 us on that basis.

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
/home/industio/fairino_ws/scripts/gripper/command_gripper.sh open
/home/industio/fairino_ws/scripts/gripper/command_gripper.sh close
/home/industio/fairino_ws/scripts/gripper/command_gripper.sh 0.060
```

## FT232H operation

The real backend has a double gate. Export the exact token, then start the
driver. Merely starting it does not open the FTDI device and emits no PWM. The
first accepted action goal starts continuous output and may move immediately.

```bash
export FR3C_ENABLE_FT232H_PWM=I_UNDERSTAND_OPEN_LOOP_PWM
/home/industio/fairino_ws/scripts/gripper/run_ft232h_driver.sh
```

Then send `open`, `close`, or a numeric command with `command_gripper.sh`.
Cancel freezes the current estimated PWM command; node shutdown closes the
FT232H GPIO controller, so it does not provide a powered holding guarantee.

After changing the backend, run the paced multi-goal regression while watching
the physical gripper:

```bash
/home/industio/fairino_ws/scripts/gripper/verify_paced_sequence.sh \
  I_AM_MONITORING_GRIPPER
```

It performs five 60↔100 mm cycles and then holds 60 mm for 30 seconds to expose
any delayed stale PWM output.

The combined MoveIt/camera wrapper starts the FT232H gripper node by default.
With the confirmation token it is armed; without the token the action server
remains online but rejects hardware goals. Starting the node still emits no PWM.
The wrapper refuses to start if another `/ydi_j22_gripper` already exists.
Every accepted goal and effective pulse is written to the ROS log.

```bash
export FR3C_ENABLE_FT232H_PWM=I_UNDERSTAND_OPEN_LOOP_PWM
/home/industio/fairino_ws/scripts/run_moveit_camera_rviz.sh
```

Inspect `/diagnostics`; `feedback_mode` is always
`open_loop_estimated` for this hardware. `frames_written` and `bytes_written`
show whether the continuous output loop is progressing.
