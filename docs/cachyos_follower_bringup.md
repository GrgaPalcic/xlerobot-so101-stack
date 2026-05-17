# CachyOS Follower Bringup

Minimal native bringup path for one real SO-101 follower arm on CachyOS/Arch.

This phase is intentionally narrow:

- one follower arm only
- no leader arm
- no cameras
- no MoveIt
- no Pixi / Rerun
- headless first, RViz later only if needed

## What This Uses

- A packaged ROS 2 Jazzy underlay at `/opt/ros/jazzy`
- A small source overlay in `~/ros2_ws/src`
- LeRobot motor setup + calibration as the source of truth

Do not reuse the sample joint override YAMLs from this repo on first hardware bringup.

## Scripts

- `scripts/install_cachyos_follower_deps.sh`
  Installs the native host packages and AUR dependencies needed for the underlay and overlay build.
- `scripts/bootstrap_cachyos_follower_overlay.sh`
  Creates `~/ros2_ws/src`, symlinks this repo into it, initializes the Feetech submodule, and clones or updates the source overlay repos.
- `scripts/build_cachyos_follower_overlay.sh`
  Builds only the minimal follower stack on top of the Jazzy underlay.
- `scripts/check_cachyos_follower.sh`
  Runs a preflight check for the underlay, overlay, serial group membership, and device symlink.

## Recommended Order

1. Install packages.

   ```bash
   ./scripts/install_cachyos_follower_deps.sh
   ```

2. Bootstrap the overlay workspace.

   ```bash
   ./scripts/bootstrap_cachyos_follower_overlay.sh
   ```

3. Build the minimal follower stack.

   ```bash
   ./scripts/build_cachyos_follower_overlay.sh
   ```

4. Run the local preflight checks.

   ```bash
   ./scripts/check_cachyos_follower.sh
   ```

5. Complete hardware prep from [hardware.md](/home/grga/Documents/so101-ros-physical-ai/docs/hardware.md):

- LeRobot motor setup and calibration for the follower arm
- add the user to `dialout` and re-login
- create the `/dev/so101_follower` udev symlink
- leave `joint_config_file` empty for first real bringup

## Mock Bringup

Use mock hardware before touching the real arm:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower.launch.py hardware_type:=mock use_rviz:=false
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers -c /follower/controller_manager
```

Expected active controllers:

- `joint_state_broadcaster`
- `forward_controller`

## Real Bringup

Only after calibration, udev, and group membership are done:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower.launch.py \
  hardware_type:=real \
  usb_port:=/dev/so101_follower \
  use_rviz:=false
```

Expected behavior:

- no serial permission errors
- no `feetech_ros2_driver` plugin load errors
- live `/follower/joint_states`

Only after that should you send a very small manual delta on `/follower/forward_controller/commands`.

## Wayland Note

Keep phase 1 headless by default. If RViz is needed later on GNOME Wayland and behaves badly, fall back to:

```bash
QT_QPA_PLATFORM=xcb rviz2
```
