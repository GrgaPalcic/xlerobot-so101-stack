# grasp_server Agent Notes

`grasp_server` runs on the GPU/DGX side, not on the Dell ROS control host. It
is the inference boundary for depth, grounding, segmentation, point cloud
isolation, and grasp generation.

## Contract

```text
Dell ROS node  <---- msgpack/HTTP/ZMQ style request ---->  grasp_server
camera image(s)                                         DA3 / SAM / GraspNet
camera intrinsics                                       object cloud
camera/base TF                                          grasp candidates
```

Keep the Dell side agnostic to which heavy model is used. The GPU server can
switch between Grounding DINO, Qwen, SAM/SAM2, GraspNet, or M2T2 as long as the
wire output remains stable.

## Current Plan

```text
Depth:
  DA3-LARGE-1.1 camera-conditioned depth preferred.
  Use CameraInfo.K and inverse(camera_to_base) as DA3 intrinsics/extrinsics.
  DA3METRIC-LARGE is monocular fallback only, not the conditioned path.

Grounding:
  Qwen VLM on future DGX is intended.
  Grounding DINO + SAM remains fallback.

Grasps:
  GraspNet baseline default.
  TiPToP/M2T2 can be a backend behind the same service boundary.
```

## Do Not

1. Do not put ROS hardware control on the GPU server.
2. Do not require the Dell to import CUDA/model libraries.
3. Do not assume a point cloud frame without storing the camera frame and TF
   used to create it.
4. Do not project masks from original image coordinates onto DA3 depth after
   batch preprocessing; use DA3 processed images/intrinsics or replicate the
   exact preprocessing transform.
