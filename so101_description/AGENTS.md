# so101_description Agent Notes

This package owns the SO-101 robot model, xacro composition, meshes, and
ros2_control hardware interface macros.

## Ownership

```text
urdf/
  so101_arm.urdf.xacro          top-level arm model
  so101_arm_common.xacro        shared links/joints
  so101_cameras.xacro           optional camera frame links
  ros2_control/                 Feetech ros2_control xacro
  end_effectors/                leader/follower gripper frame definitions

meshes/                         STL geometry
onshape/                        exported original model artifacts
rviz/                           display-only visualization configs
```

## Frame Conventions

The follower runtime usually prefixes frames with `follower/` through launch
namespacing. Local URDF names such as `base_link` or `gripper_frame_link`
become:

```text
follower/base_link
follower/gripper_frame_link
```

Do not confuse the physical gripper tip with `gripper_frame_link`. The touch
calibration used zero tool offset in `follower/gripper_frame_link`, which was
a pragmatic field assumption. If a better tool point is measured later, rerun
board touch calibration with `--tool-offset`.

## Camera Frames

The current calibrated field stack publishes camera optical frames via
`so101_bringup/launch/camera_tf.launch.py`, not through the default xacro
camera arguments. If you re-enable URDF camera links, avoid duplicate TF edges
for the same child frames.

## Servo Safety Values

The follower gripper has protection/tuning values in the ros2_control xacro and
can also be overridden in hardware YAML:

```text
max_torque_limit: 500
protection_current: 250
overload_torque: 25
```

LeRobot calibration does not produce those values. Preserve them unless there
is a deliberate hardware safety change.

