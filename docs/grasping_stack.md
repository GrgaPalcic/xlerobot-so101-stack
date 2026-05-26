# SO-101 Grasping Stack

Dual-machine grasp perception setup for the SO-101:

- Dell host `dell@192.168.1.73`
  - runs the ROS 2 stack
  - owns the arm, cameras, TF tree, and service client
- GPU host `192.168.1.54`
  - runs `grasp_server`
  - owns Depth Anything 3, Grounding DINO + SAM, GraspNet, and optional TiPToP/M2T2 inference
- Future DGX host
  - should run Qwen VLM/planning services and heavier perception backends
  - should be consumed over HTTP from the GPU-side service, keeping Dell/ROS unchanged

## Transport

Direct TCP from the Dell host into this GPU host is currently blocked on the LAN, so the deployment uses a reverse SSH tunnel from the GPU host to the Dell host:

- local `127.0.0.1:8091` on the GPU host
- forwarded to remote `127.0.0.1:8091` on the Dell host
- ROS on the Dell host keeps using `server_address:=127.0.0.1:8091`

The helper script is [scripts/run_grasp_tunnel.sh](../scripts/run_grasp_tunnel.sh).

## Calibrated Runtime

After the XLeRobot calibration run has intrinsics, hand-eye, center GoPro
extrinsics, and generated camera config, use the short host-specific wrappers.

On the GPU/local host:

```bash
cd /home/grga/Documents/so101-ros-physical-ai
./scripts/xlerobot_grasp_gpu.sh up
```

On the Dell host:

```bash
cd /home/dell/Documents/xlerobot-so101-stack
./scripts/xlerobot_grasp_stack.sh up right
./scripts/xlerobot_grasp_stack.sh detect right "pink cube"
./scripts/xlerobot_grasp_stack.sh plan right "pink cube"
./scripts/xlerobot_grasp_stack.sh execute right "pink cube"
```

Use `detect` and `plan` for either side, but real execution defaults to the
right arm only. This avoids two arms competing for an ambiguous target when
there are multiple similar objects, such as two pink cubes. `execute` first
verifies the fixed ChArUco board from the center GoPro against the accepted
calibration, then requires typing `EXECUTE` before it enables planner execution
for that one service call. This board check catches camera/world drift.

Wrist refinement is currently opt-in because the SO-101 wrist camera look pose
can move the arm into awkward configurations before the refreshed detection is
validated:

```bash
WRIST_REFINE_BEFORE_GRASP=true ./scripts/xlerobot_grasp_stack.sh execute right "pink cube"
```

The runtime also skips the MoveIt named `zero` pose by default. Plans start
from the live joint state and go directly toward the over-object/pregrasp
sequence, avoiding a large detour through an arbitrary canonical pose.

Close height is surface-relative in calibrated runtime. The planner transforms
the per-run `world_support_plane.yaml` into the active arm base frame, stages
ready/pregrasp/descent along that plane normal, and does not apply a fixed
arm-base `min_close_z_m` floor. This matters because the carriage and arm base
move relative to different tables; update the support plane when the object
surface changes.

The planner scores feasible primitive options by actual planned `wrist_roll`
movement and keeps searching until it finds a low-roll option. The default
preferred roll change is `0.35` rad, with a hard guard at `0.90` rad to avoid
unnecessary gripper barrel-roll motion. For debugging only, set
`MAX_WRIST_ROLL_DELTA_RAD=0` to disable the hard guard, or increase it before
`plan`/`execute` if the object genuinely requires a larger roll.

Wrist refinement must remain tied to the original overhead target. Refreshed
wrist detections are transformed into the active arm base frame, then rejected
if they move more than the configured same-object gate
(`wrist_refine_max_xy_shift_m`, `wrist_refine_max_z_shift_m`) from the initial
target. This prevents the wrist camera from latching onto pink arm parts or
other distractors after the view move.

The wrist view is also used as a confirmation gate before descent. A refreshed
wrist detection must remain near the original target; the runtime does not
switch to the refreshed wrist target unless `WRIST_REFINE_BEFORE_GRASP=true`.
Each `detect`, `plan`, and `execute` call writes both the latest log and a
timestamped copy under `field_runs/<run>/logs/`.

The GPU wrapper defaults the initial GG-CNN grasp crop and untrusted
cross-view fallback to the overhead view. Wrist data still participates when
the two calibrated views agree, and the wrist camera remains the pre-descent
confirmation gate. If the GPU log says `single_wrist:metric_untrusted`, the
views disagreed too much for safe fusion and the run should be treated as a
perception fault rather than a reliable object pose.

The board check uses the same GoPro intrinsics and image size as calibration.
Its `BOARD_VERIFY_PNP_REPROJECTION_ERROR_PX` setting is only the solvePnP
RANSAC inlier gate for the live verification frame. If the cube or wrist camera
mounts cover part of the 7 x 5 board, a very strict gate can fit a wrong planar
pose from too few corners. Keep the board as visible as practical and inspect
the saved `grasp/<side>/board_verify/*overlay*.jpg` when the verifier reports
low marker or inlier counts.

Stop the runtime:

```bash
./scripts/xlerobot_grasp_stack.sh down left
./scripts/xlerobot_grasp_gpu.sh down
```

## GPU Host

### Runtime scripts

- [scripts/run_grasp_server.sh](../scripts/run_grasp_server.sh)
  - starts `grasp-server` from `/home/grga/Documents/Depth-Anything-3/.venv`
  - defaults to `/home/grga/Documents/Depth-Anything-3/models/DA3-LARGE-1.1`
  - uses DA3 camera-token conditioning from calibrated intrinsics and optical-frame TF
  - defaults to `checkpoint-rs.tar` under `/home/grga/Documents/graspnet-baseline`
  - supports `GRASP_BACKEND=m2t2` with `M2T2_URL=http://<host>:8123`
- [scripts/xlerobot_grasp_gpu.sh](../scripts/xlerobot_grasp_gpu.sh)
  - syncs the current support-plane calibration from Dell
  - starts the GPU server with calibrated DA3 and GGCNN defaults
  - opens the reverse SSH tunnel to the Dell
- [scripts/run_grasp_tunnel.sh](../scripts/run_grasp_tunnel.sh)
  - keeps `127.0.0.1:8091` on the Dell host forwarded back to this machine
- [scripts/build_graspnet_extensions.sh](../scripts/build_graspnet_extensions.sh)
  - builds the `pointnet2` and `knn` CUDA extensions from `graspnet-baseline`
  - uses a small `nvcc --version` compatibility wrapper because the host toolkit is CUDA 13.2 while the PyTorch env is CUDA 12.8

### Systemd user services

Installed on this machine:

- `so101-grasp-server.service`
- `so101-grasp-tunnel.service`

Useful commands:

```bash
systemctl --user status so101-grasp-server.service
systemctl --user status so101-grasp-tunnel.service
journalctl --user -u so101-grasp-server.service -f
journalctl --user -u so101-grasp-tunnel.service -f
```

### Remaining model assets

The local Depth Anything 3 checkout contains camera-conditioned DA3 checkpoints under `/home/grga/Documents/Depth-Anything-3/models`. The grasp pipeline now prefers:

- `/home/grga/Documents/Depth-Anything-3/models/DA3-LARGE-1.1`

The server sends both views through DA3 together:

```text
camera_to_base TF  -> inverse -> DA3 world-to-camera extrinsic
camera_info K      -> DA3 intrinsics
base_link          -> DA3 world frame
```

Segmentation runs on DA3's processed image output, so mask pixels, depth pixels,
and the returned processed intrinsics are in the same coordinate space.

`DA3METRIC-LARGE` remains usable only as a fallback per-view depth model; it
does not expose the camera encoder used for extrinsic/intrinsic conditioning.

GraspNet also still needs its baseline checkpoint:

- `/home/grga/Documents/graspnet-baseline/checkpoint-rs.tar`

The upstream `graspnet-baseline` README links the official `checkpoint-rs.tar` download.

## TiPToP Reference

TiPToP is cloned at `/home/grga/Documents/tiptop` for reference. The full
project assumes DROID/Franka/Bamboo/ZED hardware, but its M2T2 grasp service is
directly useful here because it accepts a point cloud and RGB colors over HTTP.

The SO-101 stack now keeps the grasp generator behind a backend switch:

```bash
GRASP_BACKEND=graspnet scripts/run_grasp_server.sh
GRASP_BACKEND=m2t2 M2T2_URL=http://127.0.0.1:8123 scripts/run_grasp_server.sh
```

M2T2 poses are currently passed through in the service output frame. Before
execution on the real arm, validate the gripper-frame convention against SO-101
and add the needed fixed transform if the approach axis differs.

## Qwen/DGX Plan

Use the DGX as a remote inference host, not as a ROS control host. The intended
split is:

- Dell: ROS 2, cameras, TF, arm state/control, later IK execution.
- GPU/DGX services: Qwen VLM grounding, Qwen task planning, DA3/depth,
  SAM/SAM2 masks, GraspNet or M2T2 grasps.
- `grasp_server`: stable bridge from ROS requests to those inference services.

Qwen should enter in two places:

- Perception grounding: image plus prompt/task to JSON boxes and labels. The
  stack can feed the boxes to SAM/SAM2 and keep the existing point-cloud path.
- Planning: natural-language command to structured intent such as
  `pick_object`, `place_target`, and constraints. That planner output should
  drive perception prompts and later ROS2 IK/collision execution.

Expected DGX interface:

```text
POST /v1/chat/completions
OpenAI-compatible Qwen-VL/Qwen endpoint for grounding and planning JSON
```

Keep Grounding DINO + SAM as fallback until the DGX endpoint is reachable and
validated on real camera frames.

## Current Dell Camera Setup

The Dell tryout config uses
[so101_opencv_cam_dell_tryout.yaml](../so101_bringup/config/cameras/so101_opencv_cam_dell_tryout.yaml)
instead of `usb_cam`, because `usb_cam` was unstable on the current host.

Current calibrated devices:

- Wrist camera: Arducam OV9782 / UC-852 on `/dev/video2`, 1280x800 MJPG.
- Top camera: GoPro HERO11 Black loopback on `/dev/video42`, 1280x720 YUYV.

Current calibrated frames:

```text
follower/base_link -> follower/static_camera_optical_frame
follower/gripper_frame_link -> follower/wrist_camera_optical_frame
```

The calibration files live under:

```text
so101_bringup/config/cameras/calibrations/
so101_bringup/config/cameras/extrinsics/
```

### GoPro webcam bridge check

The GoPro loopback can leave `/dev/video42` present while the ffmpeg bridge is
stale. Device existence is not enough; verify a real frame.

Use the installed Dell helper after every GoPro restart or USB reconnect:

```bash
/home/dell/bin/heal_gopro_webcam.sh
```

or double-click:

```text
/home/dell/Desktop/Heal GoPro Webcam.desktop
```

The script calls the GoPro USB HTTP API to start webcam mode, restarts the
UDP-to-v4l2 bridge if needed, and saves a probe frame at
`/tmp/gopro_heal_probe.jpg`.

## Dell Host

The ROS client package stays in the repo workspace at `/home/dell/Documents/so101-ros-physical-ai`.

Launch manually on the Dell host:

```bash
~/Documents/xlerobot-so101-stack/scripts/xlerobot_grasp_stack.sh up left
```

Or from this machine:

```bash
ssh dell '~/Documents/so101-ros-physical-ai/scripts/run_grasping_stack.sh'
```

The launch file already defaults to:

- `grasp_server_address:=127.0.0.1:8091`

That is the correct address when the reverse tunnel is up.
