# so101_kinematics Agent Notes

This package owns Python IK/control helpers built on robokin, Placo, and Viser.
It is the likely bridge from grasp pose to follower arm movement until the full
MoveIt collision-aware path is validated.

## Main Concept

```text
target pose / joint target
        |
        v
so101_kinematics node
        |
        v
trajectory / forward controller command
        |
        v
follower hardware
```

## Safety

Before connecting grasp outputs to these nodes:

1. Confirm camera TFs and point-cloud frame are correct.
2. Dry-run in RViz.
3. Use conservative velocities.
4. Keep a physical stop plan.
5. Do not execute collision-risk trajectories until MoveIt/collision checking
   is integrated.

