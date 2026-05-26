# Vision Hand-Eye Calibration Progress

## Decision

Use the fixed 7x5 workspace ChArUco plate as `world` and solve each wrist with
a many-sample robot-world/hand-eye calibration. The previous corner-touch base
workflow is now an advanced fallback because the SO-101 arms are too stiff,
geared, and compliant for repeatable exact corner contact.

## Implemented Workflow

Normal wizard path after intrinsics:

```bash
calib run-step generate_world_files
calib run-step vision_handeye_left
calib run-step vision_handeye_right
calib run-step camera_extrinsics
```

Easy side-specific wrappers:

```bash
./scripts/xlerobot_vision_handeye.sh left
./scripts/xlerobot_vision_handeye.sh right
```

Each side launches real split-arm control, the Cartesian motion service, a wrist
camera overlay stream, browser jog controls, and sample capture. The operator
collects many poses where the fixed plate is visible, then quits the recorder;
the solver runs automatically.

## Outputs

Per side:

```text
field_runs/<run>/handeye/<side>/samples.jsonl
field_runs/<run>/handeye/<side>/frames/*.jpg
field_runs/<run>/handeye/<side>/overlays/*.jpg
field_runs/<run>/handeye/<side>/samples_summary.json
field_runs/<run>/extrinsics/world_to_<side>_base.yaml
field_runs/<run>/extrinsics/<side>_wrist_camera_in_gripper.yaml
field_runs/<run>/extrinsics/<side>_vision_handeye_summary.json
```

Aggregate:

```text
field_runs/<run>/extrinsics/vision_handeye_summary.json
field_runs/<run>/logs/static_tf_world_bases.sh
field_runs/<run>/logs/static_tf_wrist_cameras.sh
```

The GoPro center camera still uses the fixed board identity and
`solve_camera_extrinsics_from_board.py` in `calib run-step camera_extrinsics`.

## Field Collection Notes

Collect at least 30 samples per wrist if possible. Prefer pose diversity over
exact position: vary range, board image location, wrist roll, wrist flex, and
view angle while keeping the board fully stationary. Reject samples whose board
overlay axes look attached to the wrong target or whose reprojection error is
high.

Default filter:

```text
min samples: 15
min markers: 8
max mean reprojection error: 2.5 px
```

The browser UI defaults to port 8780 for left and 8781 for right.

## Open Follow-Ups

- Validate the first real two-arm hand-eye run in RViz/TF before promoting
  generated extrinsics into source config.
- Add a richer residual/pose-diversity report after real data shows which
  diagnostics are most useful.
- Consider a future standalone camera-intrinsics GUI, but keep this hand-eye
  workflow inside the XLeRobot field CLI for now.
