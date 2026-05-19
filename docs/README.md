# SO-101 Stack Documentation Index

This directory has two kinds of documentation:

1. Stable human docs for operating and understanding the stack.
2. Field reports that capture current local calibration and integration facts.

Start here:

```text
docs/system_architecture.md
  High-level architecture, subsystem map, and data/control flows.

docs/operations_runbook.md
  What to launch, what to verify, and what to do when hardware/cameras fail.

docs/calibration_audit.md
  Read-only joint/TF/camera/grasp snapshot audit before any real grasp motion.

docs/xlerobot_dual_arm_calibration_runbook.md
  Clean second-run procedure for the two-arm XLeRobot-style calibration.

docs/xlerobot_calibration_cli.md
  Operator notes for the resumable `xlerobot-calib` CLI wizard.

docs/assets/fiducials/
  Print-ready small world fiducial targets for the XLeRobot calibration run.

docs/field_calibration_report_2026-05-09.md
  The current Dell calibration run: board, grippers, cameras, TFs, quality.

docs/pitfalls_and_lessons.md
  Problems that cost time and must not be rediscovered.

docs/housekeeping_report.md
  What is first-party, adapted/upstream, generated, stale, or safe to delete.
```

Existing focused docs:

```text
docs/hardware.md
docs/grasping_stack.md
docs/cachyos_follower_bringup.md
```

Agent-facing docs are in `AGENTS.md` files at the repository root and major
subdirectories.
