# so101_inference Agent Notes

This package runs LeRobot policy inference from ROS observations.

It is separate from the grasping stack. Learned policy inference consumes
camera images and joint states; grasping consumes calibrated geometry, depth,
segmentation, and a grasp generator.

Keep camera observation names configurable. Policies may expect different keys:

```text
ACT examples: top, wrist
SmolVLA examples: camera1, camera2
```

For large models, prefer async inference through `policy_server` or future DGX
services rather than loading them on the Dell.

