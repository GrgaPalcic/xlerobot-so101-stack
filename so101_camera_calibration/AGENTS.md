# so101_camera_calibration Agent Notes

This package contains older Viser-based intrinsic and hand-eye tools. It is
still useful, but the latest Dell field calibration used standalone scripts in
`scripts/` instead:

```text
scripts/capture_caib_marker_board_v4l2.py
scripts/calibrate_caib_marker_board_from_frames.py
scripts/record_board_touch_points.py
scripts/solve_camera_extrinsics_from_board.py
```

Treat this package as a reusable calibration framework, not necessarily the
source of the current calibrated values.

Current field board and camera intrinsics/extrinsics live under:

```text
so101_bringup/config/cameras/calibrations/
so101_bringup/config/cameras/extrinsics/
```

When comparing methods, remember:

```text
Viser hand-eye output may be base -> static_camera_optical_frame.
Field script output is also optical frame, then written directly to TF.
Do not add a second optical-frame conversion.
```

