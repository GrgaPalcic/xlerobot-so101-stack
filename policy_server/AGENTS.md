# policy_server Agent Notes

`policy_server` is a GPU-side action inference server for LeRobot policies. It
is not part of the geometric grasp pipeline, although both may eventually run
on the same DGX.

Keep transport compatibility with `so101_inference`:

```text
ZMQ default port: 8090
gRPC optional
```

Do not put robot serial access, ros2_control, or servo commands here. The Dell
ROS host remains the only real-time hardware control boundary.

