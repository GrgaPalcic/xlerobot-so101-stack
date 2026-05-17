# so101_moveit_config Agent Notes

This package is MoveIt-generated configuration for the SO-101 arm.

Use it as the long-term target for collision-aware IK/planning. The current
grasping stack has perception and grasp candidates, but real execution still
needs careful validation through MoveIt or another collision-aware planner.

Be cautious with generated files. If regenerating with MoveIt Setup Assistant,
diff the results carefully and verify controller names still match
`so101_bringup/config/ros2_control`.

