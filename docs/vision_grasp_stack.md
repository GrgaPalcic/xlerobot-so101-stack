# Vision and Grasp Stack

This document focuses on the dual-camera object-grasping stack.

## Stack Diagram

```text
        overhead GoPro                         wrist Arducam
  follower/static_camera_optical_frame   follower/wrist_camera_optical_frame
                 |                                      |
                 +------------------+-------------------+
                                    |
                                    v
                          synchronized request
                                    |
                                    v
                         object grounding / masks
                                    |
                                    v
                    DA3 calibrated depth estimation
                                    |
                                    v
                      object point cloud in base frame
                                    |
                                    v
                         grasp candidates in base
                                    |
                                    v
                      RViz markers / future execution
```

## Frame Contract

```text
base frame:
  follower/base_link

overhead image frame:
  follower/static_camera_optical_frame

wrist image frame:
  follower/wrist_camera_optical_frame
```

Every point cloud and grasp candidate must carry a frame ID. If a frame is
missing, do not execute.

## DA3 Coordinate Contract

DA3 must receive camera geometry in the same space used later for point-cloud
fusion:

```text
ROS TF from request:
  camera_to_base = follower/base_link <- camera_optical

DA3 conditioning input:
  world frame       = follower/base_link
  extrinsic matrix  = inverse(camera_to_base)
  intrinsics matrix = CameraInfo.K for that image

Projection after DA3:
  mask pixels       = DA3 processed image space
  depth pixels      = DA3 processed image space
  K                 = DA3 returned processed intrinsics
  point cloud       = backproject(depth, K), then camera_to_base
```

Do not resize original masks onto DA3 depth by guesswork. DA3 may resize and
center-crop batched views, so segmentation must operate on the processed image
returned by DA3 or on an explicitly replicated preprocessing transform.

## Same-Object Constraint

The two camera views must segment the same physical object. A common failure is
that one camera masks one cube while the other masks a different cube.

Mitigations:

```text
1. Use a structured object prompt from Qwen/task planner.
2. Ground both views with the same label and task intent.
3. Reject if projected 3D object centroids disagree beyond a threshold.
4. For multiple similar cubes, prefer one primary view and use the second view
   only for depth/occlusion validation until cross-view association is robust.
```

## Planned Qwen/DGX Boundary

```text
input:
  task text
  overhead image
  wrist image
  optional scene metadata

output:
  target object label
  per-view boxes/masks or mask prompts
  place target if any
  constraints
```

The Dell should call a service. It should not import the Qwen model.

## What to Inspect Before Motion

```text
raw image
  |
  v
mask overlay
  |
  v
depth visualization
  |
  v
object cloud in base frame
  |
  v
candidate grasps with approach axes
  |
  v
planned pregrasp and approach path
```

If any layer looks wrong, stop. Do not compensate with hand-tuned arm motion.
