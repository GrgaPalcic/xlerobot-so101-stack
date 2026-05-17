# Housekeeping Report

This repo has three kinds of content:

```text
1. Base SO-101 ROS stack adapted from the original repository.
2. Current local integration work for dual-camera grasping.
3. Generated runtime/build artifacts that should not be kept.
```

## Keep: Core Stack

These are core ROS packages and should stay:

```text
so101_bringup/
so101_description/
so101_teleop/
so101_moveit_config/
feetech_ros2_driver/       third-party submodule
episode_recorder/
rosbag_to_lerobot/
so101_inference/
policy_server/
so101_kinematics/
so101_kinematics_msgs/
so101_camera_calibration/
```

## Keep: New Grasping Work

These are part of the current perception/grasp stack and should stay:

```text
so101_grasp_msgs/
so101_grasping/
grasp_server/
so101_bringup/config/grasping/
so101_bringup/launch/grasping.launch.py
so101_bringup/rviz/grasping_stack.rviz
scripts/run_grasping_stack.sh
scripts/run_grasp_server.sh
scripts/run_grasp_tunnel.sh
scripts/capture_stack_layers.py
scripts/publish_stack_snapshot.py
```

## Keep: Current Field Calibration Assets

These encode the current working hardware state:

```text
so101_bringup/config/cameras/calibrations/cam_overhead_gopro_hyperview.yaml
so101_bringup/config/cameras/calibrations/cam_wrist_arducam_uc852.yaml
so101_bringup/config/cameras/extrinsics/
scripts/calibrate_so101_grippers_only.py
scripts/record_board_touch_points.py
scripts/solve_camera_extrinsics_from_board.py
scripts/heal_gopro_webcam.sh
scripts/heal_gopro_webcam.desktop
```

## Delete Immediately: Generated Caches

These are safe to delete and should not be committed:

```text
**/__pycache__/
**/.pytest_cache/
*.egg-info/
```

Specific generated artifacts seen in this checkout:

```text
grasp_server/.pytest_cache/
grasp_server/grasp_server.egg-info/
grasp_server/grasp_server/__pycache__/
grasp_server/tests/__pycache__/
scripts/__pycache__/
so101_bringup/launch/__pycache__/
so101_grasping/__pycache__/
so101_grasping/so101_grasping/__pycache__/
```

## Candidate Cleanup: Duplicate Camera Configs

The camera config directory contains historical alternatives. Do not delete
until a stable launch path is chosen, but mark them as candidates:

```text
so101_bringup/config/cameras/so101_usb_cam.yaml
so101_bringup/config/cameras/so101_usb_cam_dell_tryout.yaml
so101_bringup/config/cameras/so101_v4l2_cam.yaml
so101_bringup/config/cameras/so101_gs_cam.yaml
so101_bringup/config/cameras/so101_libcam_cam.yaml
so101_bringup/config/cameras/so101_realsense2.yaml
```

Current Dell path uses:

```text
so101_cameras_dell_tryout.yaml
so101_opencv_cam_dell_tryout.yaml
```

Recommendation:

```text
Keep generic configs as examples if this repo remains public/general.
If this repo becomes a single-lab deployment repo, move unused camera configs
to docs/reference/ or delete after one successful full-stack run.
```

## Candidate Cleanup: Old Calibration Framework vs Field Scripts

`so101_camera_calibration/` is useful but not the path used for the latest
field calibration. Keep it as a reusable tool, but document that current values
come from `scripts/`.

Do not delete yet.

## Candidate Cleanup: Adapted Original Repo Marketing Content

The root `README.md` still reads like a broad upstream project README. That is
fine for public distribution, but it does not clearly separate:

```text
upstream/adapted base stack
current local Dell/GPU grasping stack
experimental policy/recording paths
```

Recommendation:

```text
Keep README.md.
Add a short "Current Local Field Stack" section near the top after the docs are
accepted, pointing to docs/README.md and docs/field_calibration_report_2026-05-09.md.
```

## Candidate Cleanup: Generated Images

Calibration overlays and frames are useful for auditability:

```text
so101_bringup/config/cameras/extrinsics/*overlay.jpg
so101_bringup/config/cameras/extrinsics/*frame.jpg
```

For a clean package, keep YAML in config and move images to:

```text
docs/assets/calibration/2026-05-09/
```

Current recommendation: keep until the first post-calibration grasp run is
validated.

## Do Not Delete

Do not delete these even if they look redundant:

```text
so101_bringup/config/hardware/lerobot_*.json
so101_bringup/config/hardware/*_joints.yaml
so101_teleop/config/teleop.yaml
scripts/heal_gopro_webcam.*
docs/field_calibration_report_2026-05-09.md
```

They encode field state and debugging knowledge.

