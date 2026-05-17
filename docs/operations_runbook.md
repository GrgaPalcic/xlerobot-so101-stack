# Operations Runbook

This is the practical "what do I do now" guide for the current Dell setup.

## Preflight

```bash
ssh dell@192.168.1.73
cd /home/dell/Documents/so101-ros-physical-ai
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Check devices:

```bash
ls -l /dev/ttyUSB* /dev/video*
v4l2-ctl --list-devices
```

Expected current field devices:

```text
/dev/ttyUSB0 or /dev/ttyUSB1   leader/follower, must verify after reconnect
/dev/video2                    wrist Arducam
/dev/video42                   GoPro loopback
```

## Heal GoPro Webcam

After every GoPro restart/reconnect:

```text
Double-click:
  /home/dell/Desktop/Heal GoPro Webcam.desktop
```

Or:

```bash
/home/dell/bin/heal_gopro_webcam.sh
```

Success means the script prints a successful frame probe from `/dev/video42`.

## Start Teleop Stack

Current field launch:

```bash
ros2 launch so101_bringup teleop.launch.py \
  hardware_type:=real \
  leader_usb_port:=/dev/ttyUSB1 \
  follower_usb_port:=/dev/ttyUSB0 \
  leader_joint_config_file:=/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/hardware/leader_joints.yaml \
  follower_joint_config_file:=/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/hardware/follower_joints.yaml \
  leader_rviz:=false \
  follower_rviz:=false \
  use_teleop_rviz:=false \
  use_cameras:=false \
  use_camera_tf:=false \
  arm_controller:=forward_controller \
  teleop_delay_s:=6.0
```

After launch:

```bash
ros2 topic echo /leader/joint_states --once
ros2 topic echo /follower/joint_states --once
ros2 topic echo /follower/forward_controller/commands --once
```

## Start Camera TF

```bash
ros2 launch so101_bringup camera_tf.launch.py
```

Verify:

```bash
ros2 run tf2_ros tf2_echo follower/base_link follower/static_camera_optical_frame
ros2 run tf2_ros tf2_echo follower/gripper_frame_link follower/wrist_camera_optical_frame
```

`follower_vision.launch.py` and `grasping.launch.py` now publish these
calibrated optical-frame TFs by default. Only use URDF camera TF overrides for
custom experiments:

```bash
ros2 launch so101_bringup follower_vision.launch.py \
  use_calibrated_camera_tf:=false \
  use_urdf_camera_tf:=true \
  cam_static_xyz:="..." \
  cam_static_rpy:="..."
```

## Read-Only Calibration Audit

Before any grasp execution after calibration or hardware reconnect:

```bash
ros2 launch so101_bringup follower_state_only.launch.py \
  hardware_type:=real \
  usb_port:=/dev/ttyUSB1 \
  joint_config_file:=/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/hardware/follower_joints.yaml \
  controller_config_file:=/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/ros2_control/follower_split_controllers.yaml \
  use_rviz:=true
```

Then in a second terminal:

```bash
python3 scripts/audit_so101_calibration.py
```

The audit is read-only. It checks camera TFs against calibration YAML, records
manual joint/TF samples, and writes `/tmp/so101_calibration_audit_*/audit.md`.

## Start Cameras

Use the Dell tryout config:

```bash
ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/dell/Documents/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_dell_tryout.yaml
```

Expected topics:

```text
/static_camera/image_raw/compressed
/static_camera/camera_info
/follower/image_raw/compressed
/follower/camera_info
```

## Start Grasping Stack

The intended stack is:

```text
camera nodes
camera TF
grasp_request_node
remote grasp_server
RViz markers
```

Use:

```bash
/home/dell/Documents/so101-ros-physical-ai/scripts/run_grasping_stack.sh
```

If the remote server is not reachable, check the reverse tunnel or direct
server address in `so101_bringup/config/grasping/grasping.yaml`.

## Layer-by-Layer Output Check

Before commanding the arm, inspect every layer:

```text
1. Raw overhead/wrist images.
2. Same target object selected in both views.
3. Masks overlay the same physical object.
4. Depth is metric and plausible.
5. Object point cloud is isolated and in base frame.
6. Grasp candidates appear on/near the object.
7. Planned approach path clears table, board, and camera/arm geometry.
8. Only then consider hardware motion.
```

## Stop / Release Hardware

Stop launch processes first. If arms remain stiff, disable torque through
LeRobot on both buses:

```bash
cd /home/dell/Documents/lerobot
PYTHONPATH=/home/dell/Documents/lerobot/src /home/dell/Documents/lerobot/.venv/bin/python - <<'PY'
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

motors = {
    name: Motor(i, "sts3215", MotorNormMode.DEGREES)
    for i, name in enumerate(
        ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
        1,
    )
}

for port in ("/dev/ttyUSB0", "/dev/ttyUSB1"):
    print(f"torque off: {port}", flush=True)
    bus = FeetechMotorsBus(port=port, motors=motors)
    try:
        bus.connect(handshake=False)
        bus.set_baudrate(1_000_000)
        bus.disable_torque(num_retry=5)
    finally:
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass
PY
```

## Camera Debug Commands

Wrist:

```bash
ffmpeg -y -f v4l2 -input_format mjpeg -video_size 1280x800 -i /dev/video2 -frames:v 1 /tmp/wrist.jpg
```

GoPro:

```bash
/home/dell/bin/heal_gopro_webcam.sh
ffmpeg -y -f v4l2 -input_format yuyv422 -video_size 1280x720 -i /dev/video42 -frames:v 1 /tmp/gopro.jpg
```

If a preview holds `/dev/video2`, stop `ffplay` before running calibration or
camera node capture.
