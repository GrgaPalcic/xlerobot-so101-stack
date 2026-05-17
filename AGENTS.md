# Agent Orientation

This repository is a real-hardware ROS 2 stack for the SO-101 arm. Treat it as
an active lab notebook plus runnable codebase. Some files are upstream/adapted
from earlier SO-101 ROS work, and some files were added during the local
dual-camera grasping integration on the Dell host.

Do not assume simulation safety. Many launch files can command real Feetech
STS3215 servos.

## Current Field Setup

Primary robot host:

```text
Dell laptop
host: dell@192.168.1.73
repo: /home/dell/Documents/so101-ros-physical-ai
calibration source: /home/dell/Documents/lerobot-calib
LeRobot checkout: /home/dell/Documents/lerobot
ROS: Jazzy
display: usually :1
```

Do not commit or document private passwords. SSH may already be configured in
the local environment.

GPU host:

```text
local/GPU side
Depth Anything 3 models: /home/grga/Documents/Depth-Anything-3/models
grasp server package: grasp_server/
future heavy planner: DGX/Qwen service, not ROS control
```

## System Story

The intended runtime split is:

```text
                natural-language task / object prompt
                              |
                              v
                     Qwen / planner service
                              |
                              v
 Dell ROS host         grasp_request_node          GPU/DGX services
---------------       -------------------        -------------------------
 SO101 follower  <---  ROS service/client   <-->  Grounding / SAM / DA3
 leader teleop         image + camera_info        point cloud isolation
 cameras + TF          TF base <-> cameras        GraspNet / M2T2
 RViz markers          grasp candidates           later Qwen grounding
       |
       v
 later ROS2 IK + collision-aware execution
```

The stack currently has these layers:

```text
hardware and TF
  so101_description/
  feetech_ros2_driver/    (submodule, third-party driver)
  so101_bringup/
  so101_teleop/

vision and calibration
  so101_camera_calibration/   (older Viser calibration tools)
  scripts/*calibrat*.py      (field calibration scripts used on Dell)
  so101_bringup/config/cameras/

grasp perception
  so101_grasp_msgs/
  so101_grasping/
  grasp_server/              (DA3 camera-conditioned depth + masks + grasps)

motion / planning
  so101_kinematics/
  so101_kinematics_msgs/
  so101_moveit_config/

data and learned policies
  episode_recorder/
  rosbag_to_lerobot/
  so101_inference/
  policy_server/
```

## Current Calibrated Facts

The current Dell calibration run produced:

```text
Board touch output:
  so101_bringup/config/cameras/extrinsics/board_in_base_touch.yaml

Overhead GoPro:
  intrinsics: so101_bringup/config/cameras/calibrations/cam_overhead_gopro_hyperview.yaml
  extrinsics: follower/base_link -> follower/static_camera_optical_frame
  quality: 42 markers, mean reprojection error about 1.44 px

Wrist Arducam:
  intrinsics: so101_bringup/config/cameras/calibrations/cam_wrist_arducam_uc852.yaml
  extrinsics: follower/gripper_frame_link -> follower/wrist_camera_optical_frame
  quality: 38 markers, mean reprojection error about 1.04 px

Camera TF launch:
  so101_bringup/launch/camera_tf.launch.py
```

The camera extrinsics are optical-frame transforms. Keep camera node
`frame_id` values aligned with these TF child frames:

```text
/static_camera/... image header -> follower/static_camera_optical_frame
/follower/... image header      -> follower/wrist_camera_optical_frame
```

## Hurdles That Must Not Be Rediscovered

Gripper calibration mismatch:

```text
LeRobot SO101 gripper:
  MotorNormMode.RANGE_0_100
  leader action and follower command are normalized percentages

ROS driver:
  exposes every joint, including gripper, as radians around raw midpoint 2048
  writes raw Goal_Position = from_radians(command) + 2048

Consequence:
  copying LeRobot gripper numbers into ROS teleop does not work.
  The teleop relay needs an explicit leader-gripper-radians ->
  follower-gripper-radians remap.
```

The current gripper-only calibration captured raw ranges:

```text
leader gripper raw:   1565..2811
follower gripper raw: 1523..3026

ROS radians:
leader open/close:   -0.740913 .. 1.170427
follower open/close: -0.805340 .. 1.500233
```

These values are used in `so101_teleop/config/teleop.yaml`.

GoPro webcam mode:

```text
When the GoPro is unplugged/restarted, /dev/video42 may still exist while no
frames arrive. The loopback is not proof of a live stream.

Fix:
  /home/dell/Desktop/Heal GoPro Webcam.desktop
  /home/dell/bin/heal_gopro_webcam.sh

The script calls:
  http://172.x.x.51:8080/gp/gpWebcam/START?res=720
then verifies a real frame from /dev/video42.
```

Camera calibration target:

```text
Printed caib.io ChArUco board:
  A3 landscape
  11 x 8 squares
  square size: 34 mm
  marker size: 25 mm
  DICT_5X5_100
  start id: 2
  physical pattern: 374 mm x 272 mm

Touch the outer checkerboard-pattern corners, not paper corners.
Top-right was unreachable, so the accepted board touch layout is:
  top_left, bottom_left, bottom_right
```

ROS control lockups:

```text
If arms are stiff after a failed launch, ROS control likely left torque on.
Stop controller/launch processes and use LeRobot Feetech bus torque-off if
needed. Do not force joints by hand against enabled torque.
```

Camera frame gotcha:

```text
OpenCV solvePnP returns the optical camera frame convention. Do not publish the
result under a non-optical frame name unless you also add the fixed optical
rotation. In this repo the calibrated static TFs are published directly to:
  follower/static_camera_optical_frame
  follower/wrist_camera_optical_frame
```

## Safe Hardware Rules

1. Never run a real-arm command unless the user confirms the workspace is clear.
2. Prefer state-only bringup for calibration and inspection.
3. Stop teleop relays before direct `/follower/forward_controller/commands`
   tests, or the relay will immediately overwrite your command.
4. Do not assume `/dev/ttyUSB0` and `/dev/ttyUSB1` identity after reconnect.
   Enumerate and verify.
5. Do not edit or reset servo EEPROM parameters casually. `joint_config_file`
   writes supported parameters to the motors at launch.
6. Do not bake credentials into docs, scripts, launch files, or desktop files.

## Build and Test Norms

Use ROS 2 Jazzy and colcon for ROS packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Python utility scripts under `scripts/` can usually be syntax-checked with:

```bash
python3 -m py_compile scripts/name.py
```

Python package tests:

```bash
pytest grasp_server/tests
```

Avoid building the whole workspace if the task only changes docs or helper
scripts. If changing message definitions, package setup files, or launch files
that are installed into `install/`, rebuild or copy to the Dell install tree
when operating in the field.

## Directory Instructions

Before editing a subsystem, read its local `AGENTS.md` if present:

```text
so101_bringup/AGENTS.md
so101_description/AGENTS.md
so101_teleop/AGENTS.md
scripts/AGENTS.md
so101_grasping/AGENTS.md
grasp_server/AGENTS.md
so101_camera_calibration/AGENTS.md
so101_kinematics/AGENTS.md
so101_moveit_config/AGENTS.md
episode_recorder/AGENTS.md
rosbag_to_lerobot/AGENTS.md
so101_inference/AGENTS.md
policy_server/AGENTS.md
```

Do not add files inside `feetech_ros2_driver/` unless the task is explicitly to
modify the third-party submodule.
