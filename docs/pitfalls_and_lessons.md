# Pitfalls and Lessons Learned

This file records problems that took real debugging time.

## 1. GoPro Loopback Is Not Proof of Video

Symptom:

```text
/dev/video42 exists
v4l2-ctl shows a GoPro loopback
OpenCV read() or ffmpeg capture blocks/times out
```

Cause:

```text
The GoPro restarted or reconnected outside webcam mode.
The v4l2loopback device persists even when no UDP video is arriving.
```

Fix:

```bash
/home/dell/bin/heal_gopro_webcam.sh
```

or use the Desktop launcher:

```text
/home/dell/Desktop/Heal GoPro Webcam.desktop
```

The script must verify a real frame, not just device existence.

## 2. LeRobot Gripper Units Are Not ROS Gripper Units

Symptom:

```text
Leader trigger appears to move.
Follower gripper does not close, moves in the wrong direction, or clamps open.
```

Cause:

```text
LeRobot gripper: normalized RANGE_0_100.
ROS Feetech driver: radians around raw midpoint 2048.
```

Fix:

```text
Calibrate gripper-only raw ranges.
Convert raw ticks to radians.
Remap leader gripper radians to follower gripper radians in teleop.cpp.
```

Current conversion:

```text
ros_rad = (raw_tick - 2048) * 2*pi / 4096
```

## 3. A Live Preview Can Own the Camera Device

Symptom:

```text
solve_camera_extrinsics_from_board.py fails:
  Failed to open V4L2 device: /dev/video2
```

Cause:

```text
ffplay preview was still holding the UVC camera.
```

Fix:

```bash
ps -eo pid,args | awk '/[f]fplay .*\\/dev\\/video2/ {print $1}' | xargs -r kill -TERM
```

## 4. Touch Board Corners Are Pattern Corners, Not Paper Corners

The A3 paper is larger than the printed checkerboard pattern. For touch
calibration, use the outer checkerboard-pattern corners.

Current board pattern:

```text
width  = 11 * 0.034 = 0.374 m
height =  8 * 0.034 = 0.272 m
```

## 5. Top-Right Board Corner Was Unreachable

The touch recorder supports `--corner-layout tl_bl_br`.

This solves:

```text
origin: top_left
y axis: top_left -> bottom_left
x axis: bottom_left -> bottom_right
```

It skips an independent fourth-corner check. Repeat with all four corners if
the board can be repositioned.

## 6. TF Warmup Errors Can Be Misleading

`tf2_echo` often prints an initial "frame does not exist" while the graph warms
up and then prints valid transforms. Look at the whole output, not only the
first warning.

## 7. Installed Launch Files Can Lag Source

When operating directly on the Dell, source files and installed files can
diverge:

```text
source:
  /home/dell/Documents/so101-ros-physical-ai/so101_bringup/launch/...

installed:
  /home/dell/Documents/so101-ros-physical-ai/install/so101_bringup/share/...
```

If a launch file change does not appear at runtime, rebuild or copy the
installed launch file.

## 8. Direct Gripper Commands Require Stopping Teleop Relay

If `so101_teleop` is running, it republishes follower commands at 50 Hz. Manual
topic publishes are immediately overwritten.

Stop the relay before direct tests.

## 9. Servo Torque Can Stay Enabled After Failed Attempts

If the arm is locked:

```text
do not force joints by hand
stop ROS control processes
run a direct LeRobot torque-off pass
```

## 10. solvePnP Gives Optical Camera Frame

OpenCV `solvePnP` returns `T_camera_object` in optical camera convention. The
current stack publishes the solved transforms directly as optical frames:

```text
follower/static_camera_optical_frame
follower/wrist_camera_optical_frame
```

Do not additionally apply the URDF optical-frame rotation unless you are
explicitly converting to a non-optical camera link.

