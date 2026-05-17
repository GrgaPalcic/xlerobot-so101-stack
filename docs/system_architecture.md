# SO-101 System Architecture

This repository is a ROS 2 stack for a real SO-101 leader/follower arm system
with dual cameras and a remote perception/grasping pipeline.

## One-Screen System Map

```text
                                 future DGX / GPU host
                            +-----------------------------+
                            | Qwen VLM / task planner     |
                            | Grounding DINO fallback     |
                            | SAM / SAM2 masks            |
                            | DA3 calibrated depth        |
                            | GraspNet or M2T2            |
                            +--------------+--------------+
                                           ^
                                           |
                                      network API
                                           |
+------------------------------------------+----------------------------------+
| Dell ROS host                                                               |
|                                                                             |
|  cameras                  perception client                 robot control    |
|  +------------------+     +---------------------+          +--------------+ |
|  | GoPro overhead   | --> | so101_grasping      | -------> | future IK /  | |
|  | Arducam wrist    | --> | grasp_request_node  |          | MoveIt exec  | |
|  +------------------+     +---------------------+          +------+-------+ |
|           |                         ^                              |         |
|           v                         |                              v         |
|  camera_info + TF            calibrated TF tree             SO101 follower   |
|                                                                 ^           |
|                                                                 |           |
|                                                            SO101 leader      |
+-----------------------------------------------------------------------------+
```

## Repository Layer Map

```text
real hardware layer
  feetech_ros2_driver/       third-party ros2_control Feetech driver
  so101_description/         URDF/xacro, meshes, ros2_control macros
  so101_bringup/             launch, hardware YAML, camera YAML, TF
  so101_teleop/              leader -> follower command relay

calibration and camera layer
  so101_camera_calibration/  older Viser-based calibration tools
  scripts/                   field calibration scripts and camera utilities
  so101_bringup/config/cameras/

grasping/perception layer
  so101_grasp_msgs/          ROS service/message definitions
  so101_grasping/            ROS client, compressed camera node, markers
  grasp_server/              GPU-side model server

motion layer
  so101_kinematics/          robokin/Placo/Viser IK helpers
  so101_moveit_config/       MoveIt config for collision-aware planning

data/policy layer
  episode_recorder/          MCAP episode writer
  rosbag_to_lerobot/         LeRobot dataset conversion
  so101_inference/           ROS policy inference clients
  policy_server/             GPU policy inference server
```

## Runtime Data Flow

```text
camera frames
  /static_camera/image_raw/compressed
  /follower/image_raw/compressed
        |
        v
camera_info + calibrated optical TF
        |
        v
grasp_request_node
        |
        v
remote grasp_server request
        |
        v
mask -> metric depth -> object cloud -> grasp candidates
        |
        v
RViz markers and later IK/collision execution
```

## Current TF Design

The calibrated camera transforms are published directly to optical frames:

```text
world
  |
  +-- follower/base_link
        |
        +-- follower/... arm chain ...
        |     |
        |     +-- follower/gripper_frame_link
        |           |
        |           +-- follower/wrist_camera_optical_frame
        |
        +-- follower/static_camera_optical_frame
```

Why optical frames directly:

```text
OpenCV solvePnP returns the camera optical frame.
Camera messages use optical-frame IDs.
Perception code expects optical camera frames.
Therefore camera_tf.launch.py publishes optical-frame transforms directly.
```

## Control Boundaries

Dell owns real hardware:

```text
serial arms
cameras
TF
RViz
ROS services
final command publication
```

GPU/DGX owns inference:

```text
VLM/task planning
grounding
segmentation
depth
grasp generation
policy inference
```

Keep these boundaries clean. Do not move real-time servo control to the GPU
host.

## Current Missing Production Pieces

```text
collision-aware grasp execution
  Need validated MoveIt/IK path from grasp pose to pregrasp/approach/close/lift.

camera calibration refinement
  Current board touch height was about 8 mm short; usable for tests but should
  be repeated with a measured tool point and an independent fourth corner.

Qwen/DGX integration
  Planned as a service boundary. Dell should stay unchanged once endpoint is
  available.

object identity consistency
  Dual camera segmentation must be tied to the same object instance. Prefer
  task-grounded labels/boxes and cross-view consistency checks before point
  cloud fusion.
```
