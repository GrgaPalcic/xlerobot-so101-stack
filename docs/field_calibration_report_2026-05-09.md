# Field Calibration Report: 2026-05-09

This report captures the current Dell calibration state. It is intentionally
specific and should be replaced by a newer dated report after the next full
calibration.

## Hardware Used

```text
Robot host:
  Dell laptop at 192.168.1.73
  repo: /home/dell/Documents/so101-ros-physical-ai

Arms:
  SO-101 leader
  SO-101 follower
  Feetech STS3215 servos

Cameras:
  wrist: Arducam OV9782 / UC-852, /dev/video2, 1280x800 MJPG
  overhead: GoPro HERO11 Black, /dev/video42 loopback, 1280x720 YUYV

Calibration board:
  caib.io ChArUco marker board
  A3 landscape
  11 x 8 squares
  square size: 34 mm
  marker size: 25 mm
  dictionary: DICT_5X5_100
  start id: 2
  pattern size: 374 mm x 272 mm
```

## Gripper Calibration

Problem:

```text
LeRobot exposes SO101 grippers as RANGE_0_100.
ROS exposes the same motors as radians around midpoint 2048.
Direct relay of leader gripper values caused wrong direction and incomplete
follower closing.
```

Fix:

```text
scripts/calibrate_so101_grippers_only.py
```

Captured ranges:

```text
leader:
  homing_offset: 782
  raw range: 1565..2811
  ROS rad: -0.740913..1.170427

follower:
  homing_offset: -1581
  raw range: 1523..3026
  ROS rad: -0.805340..1.500233
```

Current teleop remap:

```yaml
gripper_map_enabled: true
gripper_joint_name: "gripper"
gripper_leader_open: -0.740913
gripper_leader_close: 1.170427
gripper_follower_open: -0.805340
gripper_follower_close: 1.500233
```

Files updated:

```text
so101_bringup/config/hardware/leader_joints.yaml
so101_bringup/config/hardware/follower_joints.yaml
so101_bringup/config/hardware/lerobot_leader_arm.json
so101_bringup/config/hardware/lerobot_follower_arm.json
so101_teleop/config/teleop.yaml
```

## Board in Base

Script:

```text
scripts/record_board_touch_points.py
```

Point layout:

```text
top_left
bottom_left
bottom_right
```

Reason: top-right checkerboard corner was not reachable by the arm.

Touch solve:

```text
T_base_board translation:
  [0.418791984, 0.071924063, 0.025419617]

T_base_board quaternion xyzw:
  [0.714691380, -0.698616081, 0.022046383, -0.025802306]

quality:
  measured width:  0.3777 m vs expected 0.3740 m
  measured height: 0.2642 m vs expected 0.2720 m
  raw xy angle:    89.24 deg
```

Interpretation:

```text
Width is close. Height is about 8 mm short, likely touch-point placement or
tool-tip ambiguity. Usable for stack smoke tests. Repeat with a measured tool
tip and fourth corner when precision matters.
```

Output:

```text
so101_bringup/config/cameras/extrinsics/board_in_base_touch.yaml
```

## Overhead Camera Extrinsic

Camera:

```text
GoPro HERO11 Black
mode: HyperView / wide webcam feed
device: /dev/video42
```

Input intrinsics:

```text
so101_bringup/config/cameras/calibrations/cam_overhead_gopro_hyperview.yaml
```

Script:

```text
scripts/solve_camera_extrinsics_from_board.py
```

Result:

```text
parent: follower/base_link
child: follower/static_camera_optical_frame

translation:
  [0.004521045, -0.115300473, 0.277194991]

quaternion xyzw:
  [-0.626906101, 0.646509608, -0.308160835, 0.306677303]

quality:
  detected markers: 42
  inlier points: 74 / 168
  mean reprojection error: 1.44 px
  max reprojection error: 4.62 px
```

Outputs:

```text
so101_bringup/config/cameras/extrinsics/cam_overhead_in_base_from_board.yaml
so101_bringup/config/cameras/extrinsics/cam_overhead_extrinsic_overlay.jpg
```

## Wrist Camera Extrinsic

Camera:

```text
Arducam OV9782 / UC-852
device: /dev/video2
```

Input intrinsics:

```text
so101_bringup/config/cameras/calibrations/cam_wrist_arducam_uc852.yaml
```

Arm pose at wrist shot:

```text
follower joint state:
  elbow_flex:    0.378893
  gripper:      -0.776194
  shoulder_lift:-1.810097
  shoulder_pan:  0.375825
  wrist_flex:    1.790156
  wrist_roll:   -1.474156

T_base_gripper translation:
  [0.113, -0.021, 0.195]

T_base_gripper RPY:
  [110.588, 2.573, 69.433] deg
```

Result:

```text
mount parent: follower/gripper_frame_link
child: follower/wrist_camera_optical_frame

translation:
  [0.002344943, 0.072594056, -0.119362094]

quaternion xyzw:
  [0.001371523, -0.196624502, 0.979365428, 0.046693506]

quality:
  detected markers: 38
  inlier points: 144 / 152
  mean reprojection error: 1.04 px
  max reprojection error: 2.92 px
```

Outputs:

```text
so101_bringup/config/cameras/extrinsics/cam_wrist_eye_in_hand_from_board.yaml
so101_bringup/config/cameras/extrinsics/cam_wrist_extrinsic_overlay.jpg
```

## Active TF Launch

The calibrated values are installed in:

```text
so101_bringup/launch/camera_tf.launch.py
```

Live TF verification:

```bash
ros2 run tf2_ros tf2_echo follower/base_link follower/static_camera_optical_frame
ros2 run tf2_ros tf2_echo follower/gripper_frame_link follower/wrist_camera_optical_frame
```

## Known Weaknesses

```text
Board touch solve:
  missing independent top-right check point
  zero tool offset assumption
  height 8 mm short

GoPro:
  webcam loopback can be stale
  use Heal GoPro Webcam launcher before capture

Wrist:
  ffplay preview can hold /dev/video2 exclusively
  close-focus lens makes marker sharpness distance-sensitive
```

