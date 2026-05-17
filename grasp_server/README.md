# Grasp Server

Remote GPU backend for the SO-101 grasp perception stack.

Pipeline:
- JPEG RGB images from overhead and wrist cameras
- Grounding DINO + SAM for prompt-driven segmentation
- DA3 camera-conditioned depth from calibrated `K` and base-to-camera extrinsics
- masked point-cloud backprojection into `follower/base_link`
- fused object cloud filtering
- GraspNet baseline grasp generation by default, TiPToP/M2T2 via HTTP, or
  planar GG-CNN for SO-101 5-DOF cube/tabletop picking

## External dependencies

This package expects these projects to already be installed into the Python
environment or available on `PYTHONPATH`:

- `depth_anything_3`
- `graspnet-baseline`
- `graspnetAPI`

M2T2 is optional. It follows the TiPToP service contract:

- `POST /predict`
- request JSON contains `pointcloud.points`, `pointcloud.rgb`, `num_points`,
  `num_runs`, `mask_thresh`, and `apply_bounds`
- response JSON contains `grasps`, `grasp_confidence`, and optionally
  `grasp_contacts`

GG-CNN is optional and only loaded when `GRASP_BACKEND=ggcnn`. It expects the
upstream checkout and Cornell pretrained weights:

```bash
git clone https://github.com/dougsm/ggcnn.git /home/grga/Documents/ggcnn
cd /home/grga/Documents/ggcnn
curl -L -o ggcnn_weights_cornell.zip \
  https://github.com/dougsm/ggcnn/releases/download/v0.1/ggcnn_weights_cornell.zip
unzip -o ggcnn_weights_cornell.zip
```

Recommended local setup on the GPU host:

```bash
pip install -e /home/grga/Documents/Depth-Anything-3
git clone https://github.com/graspnet/graspnet-baseline.git ~/graspnet-baseline
pip install -e ~/graspnet-baseline
git clone https://github.com/graspnet/graspnetAPI.git ~/graspnetAPI
pip install -e ~/graspnetAPI
pip install -e /home/grga/Documents/so101-ros-physical-ai/grasp_server
```

## Run

```bash
grasp-server \
  --host 0.0.0.0 \
  --port 8091 \
  --metric-model /home/grga/Documents/Depth-Anything-3/models/DA3-LARGE-1.1 \
  --da3-conditioning auto \
  --graspnet-root ~/graspnet-baseline \
  --graspnet-checkpoint ~/graspnet-baseline/checkpoint-rs.tar
```

`DA3-LARGE/BASE/SMALL/GIANT` and nested DA3 models expose the camera-token
conditioning path. `DA3METRIC-LARGE` is only a monocular metric model; if it is
used here, the server falls back to independent focal-scaled depth.

Use M2T2 instead of GraspNet when an M2T2 server is running:

```bash
GRASP_BACKEND=m2t2 \
M2T2_URL=http://127.0.0.1:8123 \
scripts/run_grasp_server.sh
```

Use GG-CNN for the planar 5-DOF SO-101 path:

```bash
GRASP_BACKEND=ggcnn \
GGCNN_PRIMARY_VIEW=overhead \
scripts/run_grasp_server.sh
```

GG-CNN runs on the selected segmented view, masks quality peaks outside the
object, backprojects the best pixel through calibrated intrinsics/extrinsics,
and emits `source=ggcnn` planar grasp candidates for the SO-101 primitive
planner. Debug topics include quality, angle, overlay, and depth input images.

## Qwen/DGX plan

The Dell/ROS side should not know which VLM is used. The GPU/DGX side should
expose HTTP services that this stack can call:

- Qwen vision-language grounding: image plus task or object prompt to bounding
  boxes/labels, followed by SAM/SAM2 for masks.
- Qwen task planning: natural-language command to structured pick/place intent,
  target object, destination object/region, and constraints.
- Depth and grasp services: DA3/Depth Anything or stereo depth, then GraspNet or
  M2T2 over the object point cloud.

Until the DGX endpoint exists, Grounding DINO + SAM remains the local grounding
fallback and GraspNet remains the default grasp backend.
