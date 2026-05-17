# episode_recorder Agent Notes

This package records ROS topics into MCAP episodes for imitation learning.

It should stay independent of the grasping stack. The recorder only needs
topic names, task labels, and episode controls.

When changing topics or camera frame names in bringup, check recorder configs
and downstream conversion configs so datasets stay consistent.

Avoid adding model inference dependencies here.

