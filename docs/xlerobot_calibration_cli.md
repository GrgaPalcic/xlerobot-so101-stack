# XLeRobot Calibration CLI

`xlerobot-calib` is a resumable command-line wrapper around the dual-arm
calibration runbook. It is intentionally a CLI wizard first, not a full TUI,
so it can run reliably over SSH on the Dell robot host.

## Build

```bash
cd /home/dell/Documents/xlerobot-so101-stack
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select xlerobot_calibration
source install/setup.bash
```

The package executable is available through ROS:

```bash
ros2 run xlerobot_calibration xlerobot-calib status
```

On the Dell host, `/home/dell/bin/xlerobot-calib` wraps that command and sources
the current workspace install.

## Start A Run

On a colcon-built ROS 2 workspace, invoke the CLI through `ros2 run`:

```bash
export XLEROBOT_WS=/home/dell/Documents/xlerobot-so101-stack
export XLEROBOT_RUN=$XLEROBOT_WS/field_runs/xlerobot_printed_plate_20260520

ros2 run xlerobot_calibration xlerobot-calib \
  --workspace "$XLEROBOT_WS" \
  --out "$XLEROBOT_RUN" \
  wizard
```

For repeated commands, define a shell helper:

```bash
calib() {
  ros2 run xlerobot_calibration xlerobot-calib \
    --workspace "$XLEROBOT_WS" \
    --out "$XLEROBOT_RUN" \
    "$@"
}
```

On the Dell host, `/home/dell/bin/xlerobot-calib` is also available as a
shortcut after the package has been built. Still pass `--workspace` and `--out`
when resuming a specific run.

The wizard creates or resumes:

```text
<XLEROBOT_RUN>/run_state.yaml
```

Generated logs, images, touch solves, extrinsics, configs, bags, and reports
stay under that run directory. `field_runs/` is ignored by git.

If `--out` is omitted, the CLI looks for the latest
`<workspace>/field_runs/xlerobot_*/run_state.yaml`. Running from a different
checkout therefore makes it appear to start from scratch.

## Useful Commands

```bash
calib doctor
calib status
calib show-config
calib board-presets
calib use-board-preset intrinsics_a3_11x8_34_25_id2
calib use-board-preset world_plate_a_7x5_20_14_id49
calib set-config left_port /dev/serial/by-path/LEFT_ADAPTER_CONFIRMED
calib set-config right_port /dev/serial/by-path/RIGHT_ADAPTER_CONFIRMED
calib run-step device_inventory
calib run-step lerobot_find_ports
calib run-step generate_controller_configs
calib run-step generate_world_files
calib run-step generate_camera_config --dry-run
calib export-report
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

Steps that can write motor EEPROM or involve real hardware ask for confirmation
unless `--yes` is passed. Long-running ROS launches and hand-guided camera
captures either print exact manual instructions or run through a wrapper that
shows every command before starting. The normal wrist extrinsic path is
`vision_handeye_left/right`, which opens a browser preview/jog UI and writes the
hand-eye outputs automatically after the recorder quits.

`lerobot_find_ports` is also manual. The upstream `lerobot-find-port` helper is
interactive and asks the operator to unplug a motor bus. Running it as a
captured wizard command hides that prompt and looks like a hang. For a one-cable
PCB, record `/dev/serial/by-id` and `/dev/serial/by-path` and confirm whether
the PCB exposes separate motor buses or one shared bus before setting
`left_port` and `right_port`. Many FE-URT-1/CH340 adapters share the same USB
serial string, so prefer `/dev/serial/by-path` or custom udev names when two
adapters are attached.

`setup_motors_*` and `calibrate_*` are manual for the same reason: LeRobot asks
the operator to press Enter after moving joints or connecting exactly one motor.
Run the printed command in a terminal so those prompts remain visible.

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

## Fiducial Board Presets

The printed board assets are tracked under `docs/assets/fiducials/`, with the
same names exposed by the CLI:

```bash
calib board-presets
calib use-board-preset intrinsics_a3_11x8_34_25_id2
calib use-board-preset world_plate_a_7x5_20_14_id49
```

Use the large A3 caib.io board for intrinsics. Use one of the smaller rigid
workspace plates for world/base solves and camera extrinsics. Plate A is the
default. Plate B and Plate C are separate boards in the same PDF; switch by
applying the matching preset instead of hand-editing exported constants.

## Extraction Plan

This package is designed to be extracted into a clean XLeRobot repository after
one successful field pass. Until then, it deliberately reuses the current
workspace scripts and ROS packages so the first implementation remains close to
the working lab stack.
