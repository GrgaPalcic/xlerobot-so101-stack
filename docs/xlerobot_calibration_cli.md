# XLeRobot Calibration CLI

`xlerobot-calib` is a resumable command-line wrapper around the dual-arm
calibration runbook. It is intentionally a CLI wizard first, not a full TUI,
so it can run reliably over SSH on the Dell robot host.

## Build

```bash
cd /home/dell/Documents/so101-ros-physical-ai
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select xlerobot_calibration
source install/setup.bash
```

## Start A Run

```bash
xlerobot-calib wizard \
  --workspace /home/dell/Documents/so101-ros-physical-ai
```

The wizard creates or resumes:

```text
field_runs/xlerobot_<RUN_ID>/run_state.yaml
```

Generated logs, images, touch solves, extrinsics, configs, bags, and reports
stay under that run directory. `field_runs/` is ignored by git.

## Useful Commands

```bash
xlerobot-calib doctor
xlerobot-calib status
xlerobot-calib show-config
xlerobot-calib set-config left_port /dev/ttyUSB0
xlerobot-calib set-config right_port /dev/ttyUSB1
xlerobot-calib run-step device_inventory
xlerobot-calib run-step generate_controller_configs
xlerobot-calib run-step generate_world_files
xlerobot-calib run-step generate_camera_config --dry-run
xlerobot-calib export-report
```

Most commands accept `--workspace` and `--out` either before or after the
subcommand.

## Safety Model

The CLI tracks each step as:

```text
pending
dry_run
complete
failed
```

Steps that can write motor EEPROM or involve real hardware are marked dangerous
and ask for confirmation unless `--yes` is passed. Long-running ROS launches
and hand-guided touch/camera captures are manual steps: the CLI prints the
exact command/instructions and asks whether to mark the step complete.

## Config Values

Set these before running the dependent steps:

```text
left_port
right_port
left_wrist_dev
right_wrist_dev
center_gopro_dev
left_lerobot_json
right_lerobot_json
left_wrist_info
right_wrist_info
center_gopro_info
```

Generated values such as `left_joint_config`, `right_joint_config`, and the
per-run camera config paths are written back into `run_state.yaml`.

## Extraction Plan

This package is designed to be extracted into a clean XLeRobot repository after
one successful field pass. Until then, it deliberately reuses the current
workspace scripts and ROS packages so the first implementation remains close to
the working lab stack.

