# so101_grasping Agent Notes

This package is the ROS-side grasp perception client. It consumes camera
images, camera info, TF, and the remote GPU grasp server response, then
publishes service results and RViz markers.

## Main Files

```text
so101_grasping/compressed_camera_node.py
  Lightweight OpenCV/V4L2 compressed camera node used on the Dell tryout.

so101_grasping/grasp_request_node.py
  Assembles image/camera_info/TF request for the remote grasp server.

so101_grasping/grasp_planner_node.py
  Planner-facing node for grasp candidates and visualization.
```

## Frame Contract

The grasping config currently expects:

```text
base_frame: follower/base_link
overhead_camera_frame: follower/static_camera_optical_frame
wrist_camera_frame: follower/wrist_camera_optical_frame
```

These must match:

```text
camera node frame_id
camera_tf.launch.py static TF child frames
camera extrinsics YAML labels
```

If a camera frame lookup fails, first check `camera_tf.launch.py` and
`so101_bringup/config/cameras/so101_opencv_cam_dell_tryout.yaml`.

## Perception Layer Output Discipline

Every stack run should be able to show:

```text
raw images -> masks -> metric depth -> object cloud -> grasp candidates -> RViz markers
```

If a layer fails, save the input and output artifacts before changing code.
This avoids re-debugging camera mode, masks, and TF alignment at the same time.

