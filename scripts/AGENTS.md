# scripts Agent Notes

`scripts/` contains field utilities, calibration scripts, visualization tools,
and helper launch wrappers. These scripts are not all installed as ROS package
entry points; many are run directly on the Dell or GPU host.

## Current Field Calibration Scripts

```text
calibrate_so101_grippers_only.py
  Calibrates only motor ID 6 on leader and follower.
  Preserves other arm joint calibration.
  Updates LeRobot JSON and ROS hardware YAML.

record_board_touch_points.py
  Uses TF to record board corner touches in follower/base_link.
  Advanced fallback/debug path. Current accepted layout from the legacy
  one-arm workflow was top_left, bottom_left, bottom_right.

capture_wrist_handeye_dataset.py
  Live wrist camera preview plus browser/terminal jog UI. Captures fixed
  workspace board PnP and base/gripper TF samples for each wrist camera.

solve_wrist_robot_world_handeye.py
  Uses cv2.calibrateRobotWorldHandEye to solve world->base_link and
  gripper_frame_link->wrist_camera_optical_frame from captured samples.

solve_camera_extrinsics_from_board.py
  Detects caib.io board in camera image.
  Solves camera optical frame pose with solvePnP.
  Can compose eye-in-hand mount transform from live TF.

heal_gopro_webcam.sh
  Starts GoPro webcam mode through the GoPro USB HTTP API.
  Repairs/verifies /dev/video42 loopback stream.

audit_so101_calibration.py
  Read-only joint/TF/camera calibration audit.
  Compares live camera TFs against calibration YAML, records manual joint pose
  samples, and summarizes optional grasp stack snapshots.
```

## Current Board Specification

```text
caib.io ChArUco marker board
A3 landscape
11 x 8 squares
square: 34 mm
marker: 25 mm
dictionary: DICT_5X5_100
start id: 2
pattern size: 374 mm x 272 mm
```

## Script Design Rules

1. Keep scripts idempotent when possible.
2. Never hardcode private passwords.
3. Print enough live status for a person watching the Dell screen.
4. Prefer writing overlays/images/YAML results for every perception step.
5. For real-hardware scripts, clearly state whether motors will be torqued,
   EEPROM will be written, or ROS commands will be published.

## Common Failure Modes

```text
GoPro:
  /dev/video42 exists but read() blocks.
  Run heal_gopro_webcam.sh and verify a real frame.

Wrist camera:
  A live ffplay preview can hold /dev/video2 exclusively.
  Kill preview before capture/calibration.

TF lookup:
  First tf2_echo line often says frame does not exist while graph warms up.
  Later lines may still be valid.

OpenCV PnP:
  The result is an optical camera frame.
  Use optical frame IDs in downstream TF/config.
```
