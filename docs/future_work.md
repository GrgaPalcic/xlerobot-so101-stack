# Future Work

## Camera Intrinsics Calibration App

The current caib.io/ChArUco intrinsic calibration flow works, but it is split
across CLI commands, browser preview files, JSON summaries, CSV diagnostics, and
manual judgement. A small standalone app would make repeated field calibration
faster and less error-prone.

Useful baseline projects to compare before building:

- Caliscope: GUI-oriented ChArUco calibration with visual feedback, reprojection
  inspection, and outlier filtering. It is more multicamera/mocap oriented than
  this stack and does not directly write our ROS camera-info YAML/run-state
  outputs.
- ROS `camera_calibration`: mature ROS camera calibration tooling, but its
  workflow is image-topic/checkerboard oriented and does not match the current
  USB-device, caib.io marker-board, and robust frame-selection flow.
- Smaller ChArUco calibration repos: often useful references, but many are
  ROS 1, notebook, or one-off script workflows rather than a field-ready GUI.

Desired app shape:

- Pick a V4L2/USB camera device, resolution, FPS, and pixel format.
- Pick a board preset or enter ChArUco board specs: dictionary, square count,
  square size, marker size, start id, and marker count.
- Show a live detection overlay with marker count, motion, board center, board
  size, skew, and capture coverage by axis.
- Auto-capture stable, diverse frames while allowing manual capture from the UI.
- Highlight missing coverage regions so the operator knows where to move the
  board next.
- Run all-frame and selected-frame solves for plumb-bob and rational-polynomial
  models.
- Apply robust outlier rejection and coverage-preserving frame selection.
- Compare trials side by side by RMS, median/worst per-frame reprojection error,
  selected frame count, coverage, and intrinsic stability.
- Export ROS camera-info YAMLs, audit YAMLs, summary JSON, diagnostics CSV, and
  optional overlays using the same names expected by the XLeRobot calibration
  CLI.

The current `scripts/capture_caib_marker_board_v4l2.py` and
`scripts/calibrate_caib_marker_board_from_frames.py` are a reasonable backend
starting point. A first version could be a local web UI that wraps those scripts
instead of rewriting the calibration solver.
