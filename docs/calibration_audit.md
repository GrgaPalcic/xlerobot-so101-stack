# SO-101 Calibration Audit

Use this audit when the grasp images look plausible but the physical arm moves
to the wrong place. The goal is to separate three different questions:

```text
camera pixels/masks look right?
        |
        v
metric object cloud lands in follower/base_link?
        |
        v
ROS joint state and TF match the real arm?
        |
        v
planner/executor can safely move?
```

The grasp overlay images are diagnostic layers only. The decisive evidence is
the metric cloud and TF data in `snapshot.json`, `clouds.npz`, and the live TF
tree.

## Safe Read-Only Bringup

Launch only the follower state broadcaster. This reads hardware state and
publishes TF; it does not spawn arm or gripper command controllers.

```bash
cd /home/dell/Documents/so101-ros-physical-ai
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch so101_bringup follower_state_only.launch.py \
  hardware_type:=real \
  usb_port:=/dev/ttyUSB1 \
  joint_config_file:=/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/hardware/follower_joints.yaml \
  controller_config_file:=/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/ros2_control/follower_split_controllers.yaml \
  use_rviz:=true
```

If the follower is on a different USB device after reconnect, change only
`usb_port`.

## Run The Audit

In a second terminal:

```bash
cd /home/dell/Documents/so101-ros-physical-ai
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/audit_so101_calibration.py
```

The script will ask for manual poses:

```text
P0 neutral reference
P1 shoulder_pan only
P2 shoulder_lift only
P3 elbow_flex only
P4 wrist_flex only
P5 wrist_roll only
```

Move only the named joint for each pose, then press Enter. The script records
joint states and `follower/base_link -> follower/gripper_frame_link` TF. It
also compares live camera TFs against the calibrated YAML files:

```text
follower/base_link -> follower/static_camera_optical_frame
follower/gripper_frame_link -> follower/wrist_camera_optical_frame
```

Output is written to:

```text
/tmp/so101_calibration_audit_<timestamp>/audit.md
/tmp/so101_calibration_audit_<timestamp>/audit.json
```

## Add A Grasp Snapshot

After the joint/TF audit passes, capture the perception stack without executing
motion:

```bash
python3 scripts/capture_stack_layers.py \
  --out-dir /tmp/so101_stack_layers_live \
  --prompt "pink cube" \
  --top-k 8
```

Then summarize it together with the audit:

```bash
python3 scripts/audit_so101_calibration.py \
  --skip-manual-poses \
  --snapshot-dir /tmp/so101_stack_layers_live
```

## Pass Criteria Before Motion

```text
camera TF checks:
  translation error <= 0.02 m
  rotation error <= 5 deg

manual joint mapping:
  each P1-P5 pose has the expected dominant joint
  RViz gripper pose agrees with the physical arm pose

snapshot:
  object_cloud is non-empty
  wrist_object_cloud is non-empty when the wrist sees the cube
  object_cloud median is within about 2-3 cm of the measured cube location
```

Do not execute a grasp if any of those fail. A good-looking `ggcnn_overlay.png`
does not prove the robot-frame coordinates are correct.

## Layer Meanings

```text
input_overhead.jpg / input_wrist.jpg
  raw camera frames

overhead_mask.png / wrist_mask.png
  GroundedSAM object masks

overhead_depth.png / wrist_depth.png
  DA3 depth visualizations, not a robot-frame proof

overhead_masked_depth.png / wrist_masked_depth.png
  DA3 depth visualization inside the selected mask

clouds.npz
  metric object clouds in the configured base frame

ggcnn_quality.png / ggcnn_angle.png / ggcnn_depth_input.png
  2D GG-CNN internal maps

ggcnn_overlay.png
  visual grasp proposal overlay, not a MoveIt plan

snapshot.json
  numeric grasps, TF poses, cloud counts, and service result
```

