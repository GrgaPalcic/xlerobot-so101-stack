# XLeRobot Dual-Arm Calibration And Test Runbook

This is the second-run procedure for the current XLeRobot-style setup:

```text
left SO-101 follower arm
right SO-101 follower arm
one center GoPro in SuperView/webcam mode
one Arducam wrist camera on each arm
both arms bolted to one common physical base
```

The intent is to forget the previous camera/base calibration and rerun the
stack as if this were the first field setup. Old calibration files are useful
only as history and as examples of file format.

Do not run motion commands from this document unless the real workspace is
clear and the current step explicitly says it commands hardware. Most steps are
read-only or hand-guided calibration.

The same procedure is also available as a resumable CLI wizard through
`xlerobot-calib`. See `docs/xlerobot_calibration_cli.md` for operator usage.

## Current Git Context

The current working branch is:

```bash
git branch --show-current
# xlerobot/calibration-cli
```

The prior dirty field integration was archived before this branch:

```text
17da8ff Archive Dell dual-camera grasping integration
966a334 Merge remote-tracking branch 'origin/main' into archive/dell-dual-camera-grasping-2026-05-17
```

Commit `17da8ff` is the big Dell dual-camera grasping integration. It added the
current field scripts, GoPro/Arducam calibration outputs, grasp server, grasp
ROS client, audits, and operations docs. Commit `966a334` merged current
`origin/main`, including the LeRobot 0.5.1 inference-side updates from
`185adfd`.

## Important Decisions For This Run

1. Do not use the old `camera_tf.launch.py` calibrated values during this run.
   Those values are for one follower arm, one wrist Arducam, and one overhead
   GoPro.

2. Treat both physical SO-101 arms as follower arms unless one is explicitly
   being used as a passive teleop leader. For this XLeRobot layout, use
   `left` and `right` namespaces, not `leader` and `follower`.

3. Define `world` from a rigid reachable fiducial plate on the workspace. Do
   not require CAD or tape-measured arm base positions. The measured outputs of
   the run are `world -> left/base_link` and `world -> right/base_link`.

4. Use the large ChArUco/caib.io board for camera intrinsics. Use a smaller
   rigid workspace fiducial plate for world/base/camera extrinsics.

5. Do not mount a tiny marker on the arm as the primary base calibration. A tag
   on the arm can be useful later as a visual sanity check, but the base solve
   should come from both arms touching the same fixed workspace fiducial.

6. ROS does not have a separate magic motor calibration step. The Feetech ROS
   driver either reads the servo state after LeRobot setup/calibration, or it
   writes values from `joint_config_file` into EEPROM at launch. Passing a
   stale YAML is worse than passing no YAML.

7. MoveIt/RViz full-pose interactive targets are not a reliable test for this
   arm. SO-101 has 5 arm DOF plus a separate gripper, so arbitrary 6D target
   poses are overconstrained. Use position-first IK and joint-space dry tests.

## Frame Names

Use these names consistently:

```text
world
  fixed workspace fiducial frame

left/base_link
right/base_link
  arm base frames from robot_state_publisher frame_prefix

left/gripper_frame_link
right/gripper_frame_link
  URDF gripper frames, not necessarily the physical touch point

left/wrist_camera_optical_frame
right/wrist_camera_optical_frame
center_gopro_optical_frame
  optical camera frames used by OpenCV/ROS CameraInfo and TF
```

Camera image headers must use the optical frame names above.

## What `T_base_board` Means

`scripts/record_board_touch_points.py` writes a transform named
`T_base_board`:

```text
parent_frame: left/base_link or right/base_link
child_frame: calibration_board
```

That is the board pose expressed in the arm base frame. In this run,
`calibration_board` is the `world` frame, so the inverse of that transform is
the static TF you need:

```text
left/base_link -> calibration_board  from touch script
world -> left/base_link              inverse used in TF
```

The arm base position is therefore measured by touching the fixed target. It is
not taken from CAD and not taken from a tape measure.

## Fiducial Target Plan

Use two physical targets:

```text
intrinsics target:
  large caib.io / ChArUco board
  A3 landscape if possible
  previous known-good dimensions:
    cols: 11
    rows: 8
    square: 0.034 m
    marker: 0.025 m
    dictionary: DICT_5X5_100
    start id: 2
    marker count: 44

workspace/world target:
  smaller rigid ChArUco board, not a single ArUco marker
  must be fixed to the table/common base and reachable by both arms
  recommended Plate A:
    cols: 7
    rows: 5
    square: 0.020 m
    marker: 0.014 m
    dictionary: DICT_5X5_100
    start id: 49
    marker count: 17
    physical pattern: 0.140 m x 0.100 m
```

The print-ready version used by this runbook is:

```text
docs/assets/fiducials/calib.io_charuco_420x297_8x11_34_25_DICT_5X5.pdf
docs/assets/fiducials/xlerobot_world_targets_7x5_20mm_aruco5x5_100_ids49-99_a4.pdf
docs/assets/fiducials/charuco_board_presets.yaml
```

The world-target PDF contains three independent plates. Use Plate A by default.
If print quality, mounting, glare, or detection is poor, use Plate B or Plate C
by applying the matching board preset. The smaller plates are 140 mm x 100 mm
patterns printed inside an A4 PDF; the physical target can be cut and mounted
on an A5-sized or smaller rigid backing.

PDF order from `docs/assets/fiducials/generate_xlerobot_world_target.py`:

```text
Page 1 top:    Plate A, ids 49-65, preset world_plate_a_7x5_20_14_id49
Page 1 bottom: Plate B, ids 66-82, preset world_plate_b_7x5_20_14_id66
Page 2 center: Plate C, ids 83-99, preset world_plate_c_7x5_20_14_id83
```

If the GoPro sees too few markers on the 7 x 5 target, print a 9 x 7 version
with the same square and marker sizes. That gives 32 markers and a 0.180 m x
0.140 m pattern.

The workspace/world target should be rigid. Tape paper to glass, acrylic, FR4,
or another flat plate. If the target is used as the support plane, keep the
printed surface coplanar with the object support surface. If the target sits
above the table, record that height offset and do not blindly use z=0 as the
table plane.

## Tool Offset / TCP

`--tool-offset` is the vector from `gripper_frame_link` to the actual physical
touch point, expressed in the gripper frame.

Last time, the gripper tip was moved to the board corners. That is a valid
touch method only if the same physical point touches every corner and the
orientation stays repeatable. With `--tool-offset 0 0 0`, the solve assumes the
URDF `gripper_frame_link` origin is the touch point. That is only an
approximation.

Recommended for this run:

1. Clamp a small rigid pointer or screw in the gripper so the contact point is
   visible and repeatable.
2. If there is no TCP solver yet, keep the wrist orientation as similar as
   possible for all board corner touches.
3. Start with `--tool-offset "0 0 0"` only as a practical fallback.
4. If the left and right base solves disagree by more than about 5 to 8 mm
   when checked against the same world target, stop and add a TCP pivot solve
   before proceeding.

A TCP pivot solve means touching one fixed divot while changing wrist
orientations, then solving the constant offset from `gripper_frame_link` to the
contact point. This repo does not currently include that solver.

## Terminal Setup

On the Dell robot host:

```bash
ssh dell@192.168.1.73

export WS=/home/dell/Documents/xlerobot-so101-stack
export RUN_ID=$(date -u +%Y%m%dT%H%M%SZ)
export OUT=$WS/field_runs/xlerobot_$RUN_ID

mkdir -p "$OUT"/{logs,config,images,intrinsics,extrinsics,touch,audit,snapshots,bags}

cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

calib() {
  xlerobot-calib --workspace "$WS" --out "$OUT" "$@"
}
```

Record the exact branch and commit:

```bash
git status --short --branch | tee "$OUT/logs/git_status.txt"
git log --oneline --decorate --max-count=20 | tee "$OUT/logs/git_log.txt"
```

Archive old calibration outputs into the run directory. Do not delete them from
the repo during field work:

```bash
mkdir -p "$OUT/old_calibration_snapshot"
cp -a so101_bringup/config/cameras/calibrations "$OUT/old_calibration_snapshot/" 2>/dev/null || true
cp -a so101_bringup/config/cameras/extrinsics "$OUT/old_calibration_snapshot/" 2>/dev/null || true
cp -a so101_bringup/launch/camera_tf.launch.py "$OUT/old_calibration_snapshot/" 2>/dev/null || true
```

## Device Inventory

Run this after every USB reconnect:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/video* /dev/v4l/by-id/* 2>/dev/null | tee "$OUT/logs/devices.txt"
v4l2-ctl --list-devices | tee "$OUT/logs/v4l2.txt"
```

Record serial identities:

```bash
ls -l /dev/serial/by-id/* /dev/serial/by-path/* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tee "$OUT/logs/serial_ports.txt"
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  name=$(basename "$dev")
  udevadm info -q property -n "$dev" 2>/dev/null | sort | tee "$OUT/logs/${name}_udev.txt" || true
done
```

Do not run `lerobot-find-port` from the calibration wizard. It is an
interactive unplug/replug helper and its prompt is hidden when command output is
captured.

Set environment variables after confirming identities:

```bash
export LEFT_PORT=/dev/ttyUSB_LEFT_CONFIRMED
export RIGHT_PORT=/dev/ttyUSB_RIGHT_CONFIRMED

export LEFT_WRIST_DEV=/dev/v4l/by-id/LEFT_ARDUCAM_CONFIRMED
export RIGHT_WRIST_DEV=/dev/v4l/by-id/RIGHT_ARDUCAM_CONFIRMED
export CENTER_GOPRO_DEV=/dev/video42
```

Do not assume `/dev/ttyUSB0` and `/dev/ttyUSB1` stayed stable after reconnect.
Many FE-URT-1/CH340 adapters expose the same USB serial string, so
`/dev/serial/by-id` may identify only one of them. Prefer
`/dev/serial/by-path` or custom udev names once the physical left/right mapping
is known.

If both arms are connected through one PCB and the PC exposes only one motor
serial adapter, there is only one Linux port to set. That topology is valid only
when the PCB exposes independently addressable buses or every servo on the
shared bus has a globally unique ID. Two SO-101 arms with duplicate IDs 1..6 on
one shared Feetech bus will collide and cannot be calibrated as separate
left/right arms by the standard LeRobot commands below.

## Heal And Probe The GoPro

After every GoPro reconnect or reboot:

```bash
/home/dell/bin/heal_gopro_webcam.sh | tee "$OUT/logs/heal_gopro.txt"

ffmpeg -y \
  -f v4l2 \
  -input_format yuyv422 \
  -video_size 1280x720 \
  -i "$CENTER_GOPRO_DEV" \
  -frames:v 1 \
  "$OUT/images/center_gopro_probe.jpg"
```

If this blocks or writes a black/stale image, fix the GoPro before continuing.
`/dev/video42` existing is not enough.

## Motor Setup And Calibration

Use the official LeRobot SO-101 command style for LeRobot 0.5.1:

```text
https://huggingface.co/docs/lerobot/v0.5.1/so101
```

Only run setup-motors if the motors need IDs/baudrate written or if the bus was
assembled from unconfigured/repurposed motors. This writes EEPROM.

Left arm:

```bash
cd /home/dell/Documents/lerobot
source .venv/bin/activate

lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port="$LEFT_PORT" \
  2>&1 | tee "$OUT/logs/left_setup_motors.txt"
```

Right arm:

```bash
cd /home/dell/Documents/lerobot
source .venv/bin/activate

lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port="$RIGHT_PORT" \
  2>&1 | tee "$OUT/logs/right_setup_motors.txt"
```

Always run a fresh calibration for this second run:

```bash
cd /home/dell/Documents/lerobot
source .venv/bin/activate

lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port="$LEFT_PORT" \
  --robot.id=xlerobot_left \
  2>&1 | tee "$OUT/logs/left_lerobot_calibrate.txt"

lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port="$RIGHT_PORT" \
  --robot.id=xlerobot_right \
  2>&1 | tee "$OUT/logs/right_lerobot_calibrate.txt"
```

If LeRobot says calibration already exists, force a real recalibration using
the prompt in that tool or move the old JSON aside before rerunning. Do not
reuse the old single-arm calibration unless the fresh audit below proves ROS
and LeRobot agree.

## Generate Per-Run Controller Configs

The checked-in controller YAML is rooted at `follower:`. Namespaced dual-arm
launches need matching `left:` and `right:` roots.

```bash
python3 - <<'PY'
import os
from pathlib import Path

ws = Path(os.environ["WS"])
out = Path(os.environ["OUT"])
src = (ws / "so101_bringup/config/ros2_control/follower_split_controllers.yaml").read_text()

for ns in ("left", "right"):
    text = src.replace("follower:", f"{ns}:", 1)
    path = out / "config" / f"{ns}_split_controllers.yaml"
    path.write_text(text)
    print(path)
PY
```

## Generate Per-Run Joint Configs From Fresh LeRobot JSON

The old problem was that LeRobot and ROS appeared to disagree. For this run,
do not copy the old `follower_joints.yaml` to both arms. Use the fresh
LeRobot calibration JSONs as the source and generate per-arm ROS YAML files.

Find the fresh JSONs:

```bash
find \
  /home/dell/Documents/lerobot-calib \
  "$HOME/.cache/huggingface/lerobot/calibration" \
  \( -name '*xlerobot_left*.json' -o -name '*xlerobot_right*.json' \) \
  2>/dev/null | sort | tee "$OUT/logs/fresh_lerobot_json_candidates.txt"
```

Set the paths explicitly:

```bash
export LEFT_LEROBOT_JSON=/path/to/xlerobot_left.json
export RIGHT_LEROBOT_JSON=/path/to/xlerobot_right.json
```

Generate ROS joint YAMLs. This makes the ROS driver write the same
`homing_offset`, `range_min`, and `range_max` values that LeRobot just
produced. It also preserves the gripper protection values used by this stack.

```bash
python3 - <<'PY'
import json
import os
from pathlib import Path

import yaml

out = Path(os.environ["OUT"])

extra_common = {
    "p_coefficient": 16,
    "i_coefficient": 0,
    "d_coefficient": 32,
    "return_delay_time": 0,
    "acceleration": 254,
}
gripper_extra = {
    "max_torque_limit": 500,
    "protection_current": 250,
    "overload_torque": 25,
}

for side, env_name in (("left", "LEFT_LEROBOT_JSON"), ("right", "RIGHT_LEROBOT_JSON")):
    source = Path(os.environ[env_name])
    data = json.loads(source.read_text())
    joints = {}
    for name, values in data.items():
        row = {
            "id": int(values["id"]),
            "homing_offset": int(values["homing_offset"]),
            "range_min": int(values["range_min"]),
            "range_max": int(values["range_max"]),
            **extra_common,
        }
        if name == "gripper":
            row.update(gripper_extra)
        joints[name] = row
    target = out / "config" / f"{side}_joints_from_lerobot.yaml"
    target.write_text(yaml.safe_dump({"joints": joints}, sort_keys=False))
    print(target)
PY

export LEFT_JOINT_CONFIG="$OUT/config/left_joints_from_lerobot.yaml"
export RIGHT_JOINT_CONFIG="$OUT/config/right_joints_from_lerobot.yaml"
```

If you want to see whether EEPROM alone is already correct, run one additional
state-only audit with `joint_config_file:=` empty and compare it with the
generated-YAML audit. The generated YAML is the safer default for this run
because it makes ROS and the fresh LeRobot calibration use the same numbers.

## ROS State-Only Audit

State-only launch reads hardware state and publishes TF. It does not spawn arm
or gripper command controllers.

Terminal L1:

```bash
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch so101_bringup follower_state_only.launch.py \
  namespace:=left \
  frame_prefix:=left/ \
  hardware_type:=real \
  usb_port:="$LEFT_PORT" \
  joint_config_file:="$LEFT_JOINT_CONFIG" \
  controller_config_file:="$OUT/config/left_split_controllers.yaml" \
  use_rviz:=false
```

Terminal R1:

```bash
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch so101_bringup follower_state_only.launch.py \
  namespace:=right \
  frame_prefix:=right/ \
  hardware_type:=real \
  usb_port:="$RIGHT_PORT" \
  joint_config_file:="$RIGHT_JOINT_CONFIG" \
  controller_config_file:="$OUT/config/right_split_controllers.yaml" \
  use_rviz:=false
```

In a third terminal:

```bash
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /left/joint_states --once | tee "$OUT/logs/left_joint_states_once.txt"
ros2 topic echo /right/joint_states --once | tee "$OUT/logs/right_joint_states_once.txt"

ros2 run tf2_ros tf2_echo left/base_link left/gripper_frame_link 2>&1 | tee "$OUT/logs/left_base_to_gripper_tf.txt"
ros2 run tf2_ros tf2_echo right/base_link right/gripper_frame_link 2>&1 | tee "$OUT/logs/right_base_to_gripper_tf.txt"
```

Manual sanity check:

```text
Move one joint at a time by hand only while torque is not fighting you.
Confirm the dominant changed ROS joint is the physical joint you moved.
Stop if joint signs or names are obviously wrong.
```

If the arms are stiff after a failed launch, stop the launch processes before
touching the arms. If they remain stiff, use the torque-off procedure from
`docs/operations_runbook.md`.

## Intrinsic Calibration

Use the large A3 caib.io board for intrinsics. Apply the preset once to write
the board dimensions into `run_state.yaml`; do not hand-export duplicate
`INTR_*` constants.

```bash
calib use-board-preset intrinsics_a3_11x8_34_25_id2
```

The manual intrinsics steps print the capture and solve commands with the
current preset values expanded from the run state:

```bash
calib run-step intrinsics_left
calib run-step intrinsics_right
calib run-step intrinsics_gopro
```

Each printed capture command starts a browser preview on port 8765. Open
`http://192.168.1.73:8765/preview.html` while the capture command is running to
see the live detection overlay and optionally force a capture of the current
pose. The script still auto-captures stable, diverse board poses. The printed
commands are intentionally conservative for final intrinsics: they collect more
samples than the minimum, wait briefly for camera warmup, and require a still
board before saving each frame.

Keep the GoPro in the exact same mode and resolution that runtime will use.
Changing SuperView/wide/linear, resolution, or cropping invalidates intrinsics
and extrinsics. For GoPro, try the V4L2 capture command printed by
`intrinsics_gopro` first; use its ffmpeg fallback only if OpenCV blocks on
`/dev/video42`.

The intrinsics solver auto-selects frames by default. No command changes are
required: the solver first writes all-frame audit YAMLs, then writes the
filtered calibration to the expected YAML names. If median or worst errors look
high, inspect `caib_marker_board_calibration_summary.json` and the per-model
`*_frame_diagnostics.csv` files before recapturing.

Pick the YAML with the better median/worst view error. "Use" a model by
recording that YAML in the run state with `calib set-config ..._info`; later
extrinsics and camera-config generation read those paths. For the wrist
Arducams, prefer `plumb_bob` unless `rational_polynomial` gives a clear
reprojection-error improvement. A tiny residual improvement from the rational
model is not worth unstable-looking high-order distortion coefficients. For the
GoPro SuperView feed, `rational_polynomial` is often the better candidate.
Record the chosen files:

```bash
calib set-config left_wrist_info "$OUT/intrinsics/left_wrist_arducam_plumb_bob.yaml"
calib set-config right_wrist_info "$OUT/intrinsics/right_wrist_arducam_plumb_bob.yaml"
calib set-config center_gopro_info "$OUT/intrinsics/center_gopro_superview_rational_polynomial.yaml"
```

Adjust those paths if the summaries show a better model.

## Workspace Fiducial Constants

Apply the preset for the physical plate you mounted. Plate A is the default.
Use Plate B or Plate C only if that is the actual plate on the table.

```bash
calib use-board-preset world_plate_a_7x5_20_14_id49
```

The preset writes `world_cols`, `world_rows`, `world_square_m`,
`world_marker_m`, `world_start_id`, `world_marker_count`, and `world_dict` into
`run_state.yaml`. The wizard action below uses those values directly.

Generate the `world -> calibration_board` identity YAML and the support-plane
YAML from the preset-backed run state:

```bash
calib run-step generate_world_files
```

## Solve Each Arm Base From Touches

Keep the fixed workspace target physically locked down. Do not move it between
left and right arm touch solves.

If both arms can reach all four corners, use `tl_tr_bl_br`. If a corner is not
reachable, use `tl_bl_br`. The smaller world target is meant to make all four
corners reachable.

Terminal with both state-only launches already running. These manual steps
print commands with the current world-board preset expanded from run state:

```bash
calib run-step touch_left_base
calib run-step touch_right_base
```

No leader arms or teleop are used for these steps. The state-only launches use
the follower URDF geometry with state interfaces only, so they read servo
positions and publish TF while leaving the arms hand-movable. Move the arm by
hand so the same physical point on the gripper touches each requested outer
checkerboard-pattern corner, hold it still, then press Enter in the recorder
terminal. The recorder samples TF; it does not publish arm commands.

Do not force a stiff arm. If an arm fights you, a non-state-only launch or a
previous failed process may still have torque enabled. Stop the launch/processes
first; if the arm remains stiff, run the torque-off snippet at the end of this
runbook using the current left/right ports.

If top-right is not reachable:

```bash
--corner-layout tl_bl_br
```

Stop if either solve reports poor geometry:

```text
measured width differs by more than 3 mm
measured height differs by more than 3 mm
raw x/y angle differs from 90 deg by more than 2 deg
fourth-corner check error is more than 5 mm, if using all four corners
```

## Invert Touch Solves And Publish `world -> base_link`

Generate static TF commands from the touch YAMLs:

```bash
python3 - <<'PY'
import math
import os
from pathlib import Path

import numpy as np
import yaml

def quat_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=float)

def matrix_to_quat(r):
    tr = float(np.trace(r))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return [(r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s, (r[1,0]-r[0,1])/s, 0.25*s]
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1 + r[0,0] - r[1,1] - r[2,2]) * 2
        return [0.25*s, (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s, (r[2,1]-r[1,2])/s]
    if i == 1:
        s = math.sqrt(1 + r[1,1] - r[0,0] - r[2,2]) * 2
        return [(r[0,1]+r[1,0])/s, 0.25*s, (r[1,2]+r[2,1])/s, (r[0,2]-r[2,0])/s]
    s = math.sqrt(1 + r[2,2] - r[0,0] - r[1,1]) * 2
    return [(r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, 0.25*s, (r[1,0]-r[0,1])/s]

def load_tf(path):
    data = yaml.safe_load(Path(path).read_text())
    tf = data["transform"]
    r = np.array(tf.get("rotation_matrix") or quat_to_matrix(tf["quaternion_xyzw"]), dtype=float)
    t = np.array(tf["translation_xyz"], dtype=float)
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return data, m

def invert(m):
    out = np.eye(4)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return out

out = Path(os.environ["OUT"])
for ns in ("left", "right"):
    src = out / "touch" / f"{ns}_base_to_world_board.yaml"
    data, base_board = load_tf(src)
    world_base = invert(base_board)
    q = matrix_to_quat(world_base[:3, :3])
    q = np.array(q, dtype=float)
    q = q / np.linalg.norm(q)
    t = world_base[:3, 3]
    child = f"{ns}/base_link"
    cmd = (
        "ros2 run tf2_ros static_transform_publisher "
        f"--x {t[0]:.9f} --y {t[1]:.9f} --z {t[2]:.9f} "
        f"--qx {q[0]:.9f} --qy {q[1]:.9f} --qz {q[2]:.9f} --qw {q[3]:.9f} "
        f"--frame-id world --child-frame-id {child}"
    )
    print(cmd)
    result = {
        "transform": {
            "name": f"T_world_{ns}_base",
            "parent_frame": "world",
            "child_frame": child,
            "translation_xyz": t.tolist(),
            "rotation_matrix": world_base[:3, :3].tolist(),
            "quaternion_xyzw": q.tolist(),
        },
        "source": str(src),
    }
    (out / "extrinsics" / f"world_to_{ns}_base.yaml").write_text(yaml.safe_dump(result, sort_keys=False))
PY
```

Run the two printed static TF commands in two persistent terminals, or put them
in a temporary launch file for this run.

Verify:

```bash
ros2 run tf2_ros tf2_echo world left/base_link 2>&1 | tee "$OUT/logs/world_to_left_base_tf.txt"
ros2 run tf2_ros tf2_echo world right/base_link 2>&1 | tee "$OUT/logs/world_to_right_base_tf.txt"
ros2 run tf2_ros tf2_echo world left/gripper_frame_link 2>&1 | tee "$OUT/logs/world_to_left_gripper_tf.txt"
ros2 run tf2_ros tf2_echo world right/gripper_frame_link 2>&1 | tee "$OUT/logs/world_to_right_gripper_tf.txt"
```

## Center GoPro Extrinsic In World

Place the fixed workspace target where the GoPro sees at least 8 markers. Use
the same board preset already recorded in `run_state.yaml`.

```bash
calib run-step camera_extrinsics
```

This manual step prints the center GoPro solve and wrist-pose templates with
the current board preset, camera device paths, and camera-info YAML paths
expanded from `run_state.yaml`.

Accept only if:

```text
detected markers >= 8
mean reprojection error <= 2.0 px
overlay axes are attached to the correct target corner and orientation
camera pose looks physically plausible in RViz/TF
```

The script prints a static TF command. Keep it running:

```bash
# Use the exact command printed by solve_camera_extrinsics_from_board.py.
```

## Wrist Camera Extrinsics

The wrist camera solve must output the mount transform:

```text
left/gripper_frame_link -> left/wrist_camera_optical_frame
right/gripper_frame_link -> right/wrist_camera_optical_frame
```

For each arm, keep `world -> arm/base_link` static TF running and keep the
state-only launch running. Move the arm by hand through several poses where the
wrist camera sees the fixed target clearly.

### Left Wrist

Use the left-wrist template printed by `calib run-step camera_extrinsics` and
repeat it for at least five distinct poses:

```text
pose01: centered target, medium range
pose02: closer target
pose03: farther target
pose04: wrist rolled left
pose05: wrist rolled right
```

Name the outputs `left_wrist_pose02.yaml`, and so on.

### Right Wrist

Repeat the same sequence with the right-wrist template printed by
`calib run-step camera_extrinsics`.

### Wrist Consistency Check

Run this after collecting several wrist poses:

```bash
python3 - <<'PY'
import glob
import math
import os
from pathlib import Path

import numpy as np
import yaml

def q_angle_deg(a, b):
    a = np.array(a, dtype=float); a /= np.linalg.norm(a)
    b = np.array(b, dtype=float); b /= np.linalg.norm(b)
    return math.degrees(2 * math.acos(min(1.0, abs(float(a @ b)))))

out = Path(os.environ["OUT"])
for side in ("left", "right"):
    rows = []
    for path in sorted(glob.glob(str(out / "extrinsics" / f"{side}_wrist_pose*.yaml"))):
        data = yaml.safe_load(Path(path).read_text())
        mt = data.get("mount_transform") or {}
        rows.append((path, mt.get("translation_xyz"), mt.get("quaternion_xyzw"), data.get("quality", {})))
    if not rows:
        continue
    t = np.array([r[1] for r in rows], dtype=float)
    q0 = rows[0][2]
    angles = [q_angle_deg(q0, r[2]) for r in rows]
    print(side)
    print("  files:", len(rows))
    print("  translation mean:", t.mean(axis=0).tolist())
    print("  translation span mm:", ((t.max(axis=0) - t.min(axis=0)) * 1000).tolist())
    print("  max rotation delta deg:", max(angles))
    for path, _t, _q, quality in rows:
        print(" ", Path(path).name, quality)
PY
```

Accept if the wrist mount transform is stable:

```text
translation span <= 5 mm per axis
max rotation delta <= 3 deg
mean reprojection error <= 2 px for each accepted pose
```

If this fails, do not average bad data. Check:

```text
wrong camera intrinsics
board moved during capture
world -> base touch solve error
wrist camera mount flex
wrong camera frame_id
wrong left/right camera device
```

Choose the best YAML per side or compute a proper average later. For a field
smoke test, use the lowest reprojection-error pose only if consistency is good.

## Build Temporary Camera Configs

Generate a per-run camera launch config. This does not modify checked-in files.

```bash
calib run-step generate_camera_config
```

Launch cameras:

```bash
ros2 launch so101_bringup cameras.launch.py \
  cameras_config:="$OUT/config/xlerobot_cameras.yaml"
```

Verify topics and frame IDs:

```bash
ros2 topic list | tee "$OUT/logs/topics_after_cameras.txt"
ros2 topic echo /left/camera_info --once | tee "$OUT/logs/left_camera_info_once.txt"
ros2 topic echo /right/camera_info --once | tee "$OUT/logs/right_camera_info_once.txt"
ros2 topic echo /center_gopro/camera_info --once | tee "$OUT/logs/center_gopro_camera_info_once.txt"
```

Expected image topics:

```text
/left/image_raw/compressed
/right/image_raw/compressed
/center_gopro/image_raw/compressed
```

Expected header frames:

```text
left/wrist_camera_optical_frame
right/wrist_camera_optical_frame
center_gopro_optical_frame
```

## Full TF Check

At this point these chains must exist:

```text
world -> left/base_link -> left/gripper_frame_link -> left/wrist_camera_optical_frame
world -> right/base_link -> right/gripper_frame_link -> right/wrist_camera_optical_frame
world -> center_gopro_optical_frame
```

Check:

```bash
ros2 run tf2_ros tf2_echo world left/wrist_camera_optical_frame 2>&1 | tee "$OUT/logs/world_to_left_wrist_camera_tf.txt"
ros2 run tf2_ros tf2_echo world right/wrist_camera_optical_frame 2>&1 | tee "$OUT/logs/world_to_right_wrist_camera_tf.txt"
ros2 run tf2_ros tf2_echo world center_gopro_optical_frame 2>&1 | tee "$OUT/logs/world_to_center_gopro_tf.txt"
```

Do not continue if any TF lookup fails.

## Read-Only Calibration Audit

The current audit script is one-arm oriented, but its arguments let you run it
per arm. Use it to check joint mapping and gripper TF samples. Camera TF
comparison expects old file shapes, so keep camera checks as supplemental here.

Left:

```bash
python3 scripts/audit_so101_calibration.py \
  --joint-states-topic /left/joint_states \
  --base-frame left/base_link \
  --gripper-frame left/gripper_frame_link \
  --out-dir "$OUT/audit/left_manual" \
  2>&1 | tee "$OUT/logs/left_manual_audit.txt"
```

Right:

```bash
python3 scripts/audit_so101_calibration.py \
  --joint-states-topic /right/joint_states \
  --base-frame right/base_link \
  --gripper-frame right/gripper_frame_link \
  --out-dir "$OUT/audit/right_manual" \
  2>&1 | tee "$OUT/logs/right_manual_audit.txt"
```

Pass criteria:

```text
each manual pose has the expected dominant joint
gripper_frame_link moves in the same direction as the physical arm
reported joint values are plausible and repeatable
left/right do not swap devices
```

## Grasp Server Startup

On the GPU/local side, run the server with `world` as the request base frame.
This avoids left/right support-plane mismatch and gives one common scene frame.

```bash
cd /home/grga/Documents/so101-ros-physical-ai

export REPO_ROOT=/home/grga/Documents/so101-ros-physical-ai
export VENV_PATH=/home/grga/Documents/Depth-Anything-3/.venv
export METRIC_MODEL=/home/grga/Documents/Depth-Anything-3/models/DA3-LARGE-1.1
export GRASP_BACKEND=ggcnn
export DA3_CONDITIONING=required
export DA3_FALLBACK_INDEPENDENT=false
export SUPPORT_PLANE_YAML=/home/dell/Documents/xlerobot-so101-stack/field_runs/xlerobot_<RUN_ID>/extrinsics/world_support_plane.yaml
export WORKSPACE_BOUNDS="-0.50 0.50 -0.35 0.35 -0.05 0.50"

scripts/run_grasp_server.sh
```

Replace `<RUN_ID>` with the actual run directory name, or copy the
`world_support_plane.yaml` to the GPU host.

If the Dell reaches the GPU through the reverse tunnel, also start the tunnel
as described in `docs/grasping_stack.md`.

## Grasp Request Nodes

Run one request node per arm side, but keep `base_frame:=world` so the server
gets both wrist and GoPro views in the same calibrated frame.

Left perception node:

```bash
ros2 run so101_grasping grasp_request_node --ros-args \
  -r __ns:=/left_grasp \
  -p server_address:=127.0.0.1:8091 \
  -p recv_timeout_ms:=120000 \
  -p send_timeout_ms:=5000 \
  -p default_top_k:=10 \
  -p max_data_age_s:=0.5 \
  -p base_frame:=world \
  -p overhead_camera_frame:=center_gopro_optical_frame \
  -p wrist_camera_frame:=left/wrist_camera_optical_frame \
  -p overhead_image_topic:=/center_gopro/image_raw/compressed \
  -p wrist_image_topic:=/left/image_raw/compressed \
  -p overhead_camera_info_topic:=/center_gopro/camera_info \
  -p wrist_camera_info_topic:=/left/camera_info
```

Right perception node:

```bash
ros2 run so101_grasping grasp_request_node --ros-args \
  -r __ns:=/right_grasp \
  -p server_address:=127.0.0.1:8091 \
  -p recv_timeout_ms:=120000 \
  -p send_timeout_ms:=5000 \
  -p default_top_k:=10 \
  -p max_data_age_s:=0.5 \
  -p base_frame:=world \
  -p overhead_camera_frame:=center_gopro_optical_frame \
  -p wrist_camera_frame:=right/wrist_camera_optical_frame \
  -p overhead_image_topic:=/center_gopro/image_raw/compressed \
  -p wrist_image_topic:=/right/image_raw/compressed \
  -p overhead_camera_info_topic:=/center_gopro/camera_info \
  -p wrist_camera_info_topic:=/right/camera_info
```

Call services without executing motion:

```bash
ros2 service call /left_grasp/detect_grasps so101_grasp_msgs/srv/DetectGrasps \
  "{prompt: 'pink cube', top_k: 8}" \
  | tee "$OUT/logs/left_detect_grasps.txt"

ros2 service call /right_grasp/detect_grasps so101_grasp_msgs/srv/DetectGrasps \
  "{prompt: 'pink cube', top_k: 8}" \
  | tee "$OUT/logs/right_detect_grasps.txt"
```

Expected debug topics:

```text
/left_grasp/so101_grasping/object_cloud
/left_grasp/so101_grasping/grasp_markers
/right_grasp/so101_grasping/object_cloud
/right_grasp/so101_grasping/grasp_markers
```

The existing `scripts/capture_stack_layers.py` is hardcoded for the old
`/follower` and `/static_camera` topics. Do not use it unchanged for the
dual-arm run. Either update it later to accept topic/service arguments, or save
artifacts with `ros2 bag record`:

```bash
ros2 bag record -o "$OUT/bags/left_grasp_probe" \
  /center_gopro/image_raw/compressed \
  /center_gopro/camera_info \
  /left/image_raw/compressed \
  /left/camera_info \
  /left_grasp/so101_grasping/object_cloud \
  /left_grasp/so101_grasping/overhead_object_cloud \
  /left_grasp/so101_grasping/wrist_object_cloud \
  /left_grasp/so101_grasping/grasp_markers \
  /left_grasp/so101_grasping/overhead_mask/compressed \
  /left_grasp/so101_grasping/wrist_mask/compressed \
  /left_grasp/so101_grasping/ggcnn_overlay/compressed
```

## Perception Acceptance Checks

Before any planner work:

```text
raw images are live and from the intended cameras
center GoPro and wrist camera see the same object
masks select the same physical object
object_cloud is non-empty in world
cloud z is near the support plane plus object height
grasp markers sit on the object in RViz with fixed frame world
left and right request nodes produce compatible object locations
```

A good-looking 2D `ggcnn_overlay` is not enough. The 3D cloud and TF must be
right.

## MoveIt / IK Test Plan

Do not use RViz's arbitrary 6D interactive marker as the primary test. The
SO-101 arm is 5 arm DOF, so exact arbitrary orientation requests can be
unsolvable and the marker may snap back.

The current MoveIt config already sets PickIK to position-first behavior:

```yaml
rotation_scale: 0.0
approximate: true
```

Current MoveIt test tooling is still single-arm biased and hardcodes
`/follower/joint_states` in `so101_moveit_test.py`. For now:

1. Run MoveIt in `hardware_type:=mock` first.
2. Test one physical arm at a time using compatibility namespace `follower`, or
   update the MoveIt test script before using `left` and `right`.
3. Keep `allow_execution:=false` until calibration and reachability pass.
4. Treat grasp orientation as a hint, not a hard 6D target.
5. Transform candidate grasp positions from `world` into the selected arm base
   before solving IK.

Mock smoke test:

```bash
ros2 launch so101_bringup moveit_py_test.launch.py \
  hardware_type:=mock \
  namespace:=follower \
  use_cameras:=false
```

For physical tests, do not run the current looping MoveIt test against the real
arms without editing it. It plans and executes in a loop.

## Stop / Torque Off

Stop launch processes first. If torque remains enabled, use the torque-off
snippet in `docs/operations_runbook.md`, with the current left/right ports:

```bash
cd /home/dell/Documents/lerobot
PYTHONPATH=/home/dell/Documents/lerobot/src /home/dell/Documents/lerobot/.venv/bin/python - <<'PY'
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

ports = ["LEFT_PORT_VALUE", "RIGHT_PORT_VALUE"]
motors = {
    name: Motor(i, "sts3215", MotorNormMode.DEGREES)
    for i, name in enumerate(
        ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
        1,
    )
}

for port in ports:
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

Replace `LEFT_PORT_VALUE` and `RIGHT_PORT_VALUE` before running.

## Final Field Report

After the run, write a dated report next to the old one:

```text
docs/field_calibration_report_<YYYY-MM-DD>_xlerobot.md
```

Include:

```text
left/right LeRobot calibration IDs and JSON paths
camera intrinsic YAML chosen for each camera
world target dimensions and start id
touch solve quality for each base
world -> left/base_link
world -> right/base_link
world -> center_gopro_optical_frame
left/gripper_frame_link -> left/wrist_camera_optical_frame
right/gripper_frame_link -> right/wrist_camera_optical_frame
perception snapshot or bag paths
known failures and stop points
```

Do not promote generated calibration files into `so101_bringup/config/` until
the acceptance checks pass.

## Stop Criteria

Stop the run and fix the layer that failed if any of these happen:

```text
LeRobot and ROS joint identities disagree
left/right arm ports or cameras are swapped
GoPro frame is stale or wrong mode
camera_info frame_id does not match TF child frame
world -> base touch solve is geometrically bad
wrist mount transform is not stable across poses
center GoPro PnP overlay is wrong or reprojection error is high
object cloud appears below the support plane or far from the real object
MoveIt requires exact 6D orientation for a 5-DOF arm
```

The correct order is:

```text
motors -> ROS state -> intrinsics -> world/base -> camera extrinsics -> TF -> perception -> IK dry run -> execution
```
