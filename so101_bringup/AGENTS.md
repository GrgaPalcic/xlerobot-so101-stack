# so101_bringup Agent Notes

`so101_bringup` owns launch files, hardware configs, camera configs, RViz
layouts, grasping parameters, and top-level integration. It is the first place
to check when runtime behavior differs from source code intent.

## Important Paths

```text
launch/
  teleop.launch.py              leader/follower hardware and teleop
  follower_state_only.launch.py follower state broadcaster only
  grasping.launch.py            camera + grasp perception integration
  camera_tf.launch.py           calibrated static camera TFs
  cameras.launch.py             camera node multiplexer

config/hardware/
  leader_joints.yaml
  follower_joints.yaml
  lerobot_leader_arm.json
  lerobot_follower_arm.json

config/cameras/
  so101_cameras_dell_tryout.yaml
  so101_opencv_cam_dell_tryout.yaml
  calibrations/
  extrinsics/

config/grasping/
  grasping.yaml
```

## Hardware Config Warnings

`joint_config_file` values are not passive documentation. The Feetech driver
writes supported values to servo registers during initialization. This includes
homing offsets and range limits.

Use YAML joint config only when the values are known-good for the connected
arm. Do not copy values between robots unless they are physically the same
calibration.

Current field values are from `/home/dell/Documents/lerobot-calib` and were
synced into:

```text
config/hardware/leader_joints.yaml
config/hardware/follower_joints.yaml
config/hardware/lerobot_leader_arm.json
config/hardware/lerobot_follower_arm.json
```

## Camera TF Reality

`camera_tf.launch.py` is calibrated for optical frames:

```text
follower/base_link
  -> follower/static_camera_optical_frame

follower/gripper_frame_link
  -> follower/wrist_camera_optical_frame
```

The names must match camera node `frame_id` parameters and grasping config.
Do not switch to `static_camera/cam_overhead` or `follower/cam_wrist` unless
all dependent configs are updated and the optical-frame convention is handled.

## Dell Tryout Camera Config

The Dell field configuration uses `opencv_compressed` camera nodes from
`so101_grasping` because other camera drivers were unstable on this host.

Current devices:

```text
Arducam wrist: /dev/video2, 1280x800 MJPG
GoPro overhead: /dev/video42, 1280x720 YUYV via loopback
```

GoPro `/dev/video42` can exist while no frames arrive. Run the Desktop healer
or `/home/dell/bin/heal_gopro_webcam.sh` before assuming the camera is alive.

## Launch Hygiene

When updating launch files on the Dell, remember that `colcon build` installs
launch files under:

```text
install/so101_bringup/share/so101_bringup/launch/
```

During field work we sometimes copied launch files into `install/` directly to
avoid a full rebuild. If you make a durable code change, rebuild or ensure the
installed copy is also updated.

