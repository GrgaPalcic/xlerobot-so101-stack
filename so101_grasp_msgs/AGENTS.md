# so101_grasp_msgs Agent Notes

This package defines ROS interfaces for grasp detection and planning.

Interface changes are expensive because they require rebuilding dependent
packages and updating both the Dell and GPU-side client/server glue.

Before changing `.msg` or `.srv` files:

1. Check `so101_grasping`.
2. Check `grasp_server`.
3. Rebuild the workspace.
4. Verify service calls from a sourced install environment.

