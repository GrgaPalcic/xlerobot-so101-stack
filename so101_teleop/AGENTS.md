# so101_teleop Agent Notes

This package owns leader-to-follower command relaying.

## Main Files

```text
src/teleop.cpp          main follower command relay
config/teleop.yaml     runtime parameters
launch/                package-level launch helpers
```

## Critical Gripper Lesson

LeRobot and ROS expose the SO-101 gripper differently:

```text
LeRobot:
  gripper = MotorNormMode.RANGE_0_100
  leader reads normalized percentage
  follower writes normalized percentage

ROS driver:
  gripper is a position interface in radians around raw midpoint 2048
```

Without an explicit remap, the follower gripper can move the wrong direction or
clamp open. The current relay supports:

```yaml
gripper_map_enabled: true
gripper_joint_name: "gripper"
gripper_leader_open: -0.740913
gripper_leader_close: 1.170427
gripper_follower_open: -0.805340
gripper_follower_close: 1.500233
```

These values came from gripper-only raw calibration:

```text
leader raw:   1565..2811
follower raw: 1523..3026
```

If either gripper is recalibrated, update `teleop.yaml` from the new raw limits:

```text
ros_rad = (raw_tick - 2048) * 2*pi / 4096
```

## Test Pattern

Before debugging arm motion, verify the relay path:

```bash
ros2 topic echo /leader/joint_states --once
ros2 topic echo /follower/forward_controller/commands --once
ros2 topic echo /follower/joint_states --once
```

If direct gripper testing is needed, stop the relay first or it will overwrite
manual `/follower/forward_controller/commands` publishes.

