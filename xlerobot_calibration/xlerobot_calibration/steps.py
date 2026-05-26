from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Step:
    id: str
    title: str
    description: str
    kind: str = "command"
    commands: tuple[str, ...] = ()
    action: str = ""
    prerequisites: tuple[str, ...] = ()
    required_config: tuple[str, ...] = ()
    dangerous: bool = False
    manual_complete_prompt: str = "Mark this step complete?"


STEPS: tuple[Step, ...] = (
    Step(
        id="git_snapshot",
        title="Record git state",
        description="Save branch, status, and recent commits into the run logs.",
        commands=(
            "git -C {workspace} status --short --branch > {out}/logs/git_status.txt\n"
            "git -C {workspace} log --oneline --decorate --max-count=20 > {out}/logs/git_log.txt\n"
            "git -C {workspace} config --show-origin --get user.signingkey > {out}/logs/git_signing_key.txt || true",
        ),
    ),
    Step(
        id="device_inventory",
        title="Record USB and video devices",
        description="Capture the current arm and camera device inventory.",
        commands=(
            "ls -l /dev/ttyUSB* /dev/ttyACM* /dev/video* /dev/v4l/by-id/* 2>/dev/null "
            "> {out}/logs/devices.txt || true\n"
            "v4l2-ctl --list-devices > {out}/logs/v4l2.txt",
        ),
    ),
    Step(
        id="gopro_probe",
        title="Heal and probe center GoPro",
        description="Start GoPro webcam mode and save one probe frame.",
        commands=(
            "/home/dell/bin/heal_gopro_webcam.sh > {out}/logs/heal_gopro.txt 2>&1\n"
            "ffmpeg -y -f v4l2 -input_format yuyv422 -video_size 1280x720 "
            "-i {center_gopro_dev} -frames:v 1 {out}/images/center_gopro_probe.jpg "
            ">> {out}/logs/heal_gopro.txt 2>&1",
        ),
    ),
    Step(
        id="lerobot_find_ports",
        title="Find LeRobot arm ports",
        description=(
            "Record serial devices and decide the motor-bus topology. "
            "Do not run lerobot-find-port inside this wizard; it waits for unplug/replug input."
        ),
        kind="manual",
        commands=(
            "ls -l /dev/serial/by-id/* /dev/serial/by-path/* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null "
            "| tee {out}/logs/serial_ports.txt\n"
            "for dev in /dev/ttyUSB* /dev/ttyACM*; do\n"
            "  [ -e \"$dev\" ] || continue\n"
            "  name=$(basename \"$dev\")\n"
            "  udevadm info -q property -n \"$dev\" 2>/dev/null | sort | tee \"{out}/logs/$name\"_udev.txt || true\n"
            "done\n"
            "\n"
            "# Port identity note:\n"
            "# Many FE-URT-1/CH340 adapters expose the same USB serial string. If /dev/serial/by-id points to only\n"
            "# one adapter even though two /dev/ttyUSB* devices exist, use /dev/serial/by-path or custom udev names.\n"
            "#\n"
            "# One-cable note:\n"
            "# If both arms share one USB serial adapter, this wizard cannot discover separate left/right ports.\n"
            "# That topology is usable only when the PCB exposes independently addressable buses or all servo IDs\n"
            "# on the shared bus are globally unique. If both arms use SO-101 IDs 1..6 on one bus, they collide.\n"
            "# Set left_port/right_port only after confirming the actual topology.",
        ),
    ),
    Step(
        id="setup_motors_left",
        title="Setup left arm motors",
        description=(
            "Run LeRobot setup-motors for the left follower arm. This writes EEPROM and is interactive: "
            "connect exactly one requested motor at a time."
        ),
        kind="manual",
        required_config=("left_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-setup-motors --robot.type=so101_follower --robot.port={left_port} "
            "2>&1 | tee {out}/logs/left_setup_motors.txt",
        ),
    ),
    Step(
        id="setup_motors_right",
        title="Setup right arm motors",
        description=(
            "Run LeRobot setup-motors for the right follower arm. This writes EEPROM and is interactive: "
            "connect exactly one requested motor at a time."
        ),
        kind="manual",
        required_config=("right_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-setup-motors --robot.type=so101_follower --robot.port={right_port} "
            "2>&1 | tee {out}/logs/right_setup_motors.txt",
        ),
    ),
    Step(
        id="calibrate_left",
        title="Calibrate left arm with LeRobot",
        description="Run fresh LeRobot calibration for the left follower arm. This is interactive.",
        kind="manual",
        required_config=("left_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-calibrate --robot.type=so101_follower --robot.port={left_port} --robot.id=xlerobot_left "
            "2>&1 | tee {out}/logs/left_lerobot_calibrate.txt",
        ),
    ),
    Step(
        id="calibrate_right",
        title="Calibrate right arm with LeRobot",
        description="Run fresh LeRobot calibration for the right follower arm. This is interactive.",
        kind="manual",
        required_config=("right_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-calibrate --robot.type=so101_follower --robot.port={right_port} --robot.id=xlerobot_right "
            "2>&1 | tee {out}/logs/right_lerobot_calibrate.txt",
        ),
    ),
    Step(
        id="generate_controller_configs",
        title="Generate left/right controller YAMLs",
        description="Copy follower split controller config into left and right namespace roots.",
        kind="action",
        action="generate_controller_configs",
    ),
    Step(
        id="generate_joint_configs",
        title="Generate ROS joint YAMLs from fresh LeRobot JSON",
        description="Convert fresh LeRobot JSON calibration into per-arm ROS hardware YAMLs.",
        kind="action",
        action="generate_joint_configs",
        required_config=("left_lerobot_json", "right_lerobot_json"),
    ),
    Step(
        id="state_only_left",
        title="Launch left state-only ROS audit",
        description="Start the left state-only launch in a terminal and verify /left/joint_states.",
        kind="manual",
        prerequisites=("generate_controller_configs", "generate_joint_configs"),
        required_config=("left_port", "left_joint_config"),
        commands=(
            "cd {workspace}\n"
            "source /opt/ros/jazzy/setup.bash\n"
            "source install/setup.bash\n"
            "ros2 launch so101_bringup follower_state_only.launch.py "
            "namespace:=left frame_prefix:=left/ hardware_type:=real usb_port:={left_port} "
            "joint_config_file:={left_joint_config} controller_config_file:={left_controller_config} "
            "use_rviz:=false",
        ),
    ),
    Step(
        id="state_only_right",
        title="Launch right state-only ROS audit",
        description="Start the right state-only launch in a terminal and verify /right/joint_states.",
        kind="manual",
        prerequisites=("generate_controller_configs", "generate_joint_configs"),
        required_config=("right_port", "right_joint_config"),
        commands=(
            "cd {workspace}\n"
            "source /opt/ros/jazzy/setup.bash\n"
            "source install/setup.bash\n"
            "ros2 launch so101_bringup follower_state_only.launch.py "
            "namespace:=right frame_prefix:=right/ hardware_type:=real usb_port:={right_port} "
            "joint_config_file:={right_joint_config} controller_config_file:={right_controller_config} "
            "use_rviz:=false",
        ),
    ),
    Step(
        id="intrinsics_left",
        title="Capture and solve left wrist intrinsics",
        description="Run the left wrist caib.io capture and intrinsic solve commands from the runbook.",
        kind="manual",
        required_config=("left_wrist_dev",),
        commands=(
            "python3 {workspace}/scripts/capture_caib_marker_board_v4l2.py "
            "--device {left_wrist_dev} --width 1280 --height 800 --fps 30 --fourcc MJPG "
            "--camera-name left_wrist_arducam --output-dir {out}/intrinsics/left_wrist_capture "
            "--aruco-dict {intr_dict} --start-id {intr_start_id} --marker-count {intr_marker_count} "
            "--target-samples 80 --min-markers 8 --max-motion-px 1.5 "
            "--capture-cooldown-s 1.2 --warmup-s 2.0 --preview-port 8765\n"
            "\n"
            "python3 {workspace}/scripts/calibrate_caib_marker_board_from_frames.py "
            "--frames-dir {out}/intrinsics/left_wrist_capture/frames --output-dir {out}/intrinsics "
            "--camera-name left_wrist_arducam --cols {intr_cols} --rows {intr_rows} "
            "--square-m {intr_square_m} --marker-m {intr_marker_m} --start-id {intr_start_id} "
            "--marker-count {intr_marker_count} --aruco-dict {intr_dict} --min-markers 8 --min-frames 20",
        ),
    ),
    Step(
        id="intrinsics_right",
        title="Capture and solve right wrist intrinsics",
        description="Run the right wrist caib.io capture and intrinsic solve commands from the runbook.",
        kind="manual",
        required_config=("right_wrist_dev",),
        commands=(
            "python3 {workspace}/scripts/capture_caib_marker_board_v4l2.py "
            "--device {right_wrist_dev} --width 1280 --height 800 --fps 30 --fourcc MJPG "
            "--camera-name right_wrist_arducam --output-dir {out}/intrinsics/right_wrist_capture "
            "--aruco-dict {intr_dict} --start-id {intr_start_id} --marker-count {intr_marker_count} "
            "--target-samples 80 --min-markers 8 --max-motion-px 1.5 "
            "--capture-cooldown-s 1.2 --warmup-s 2.0 --preview-port 8765\n"
            "\n"
            "python3 {workspace}/scripts/calibrate_caib_marker_board_from_frames.py "
            "--frames-dir {out}/intrinsics/right_wrist_capture/frames --output-dir {out}/intrinsics "
            "--camera-name right_wrist_arducam --cols {intr_cols} --rows {intr_rows} "
            "--square-m {intr_square_m} --marker-m {intr_marker_m} --start-id {intr_start_id} "
            "--marker-count {intr_marker_count} --aruco-dict {intr_dict} --min-markers 8 --min-frames 20",
        ),
    ),
    Step(
        id="intrinsics_gopro",
        title="Capture and solve center GoPro intrinsics",
        description="Run the GoPro caib.io capture and intrinsic solve commands from the runbook.",
        kind="manual",
        required_config=("center_gopro_dev",),
        commands=(
            "python3 {workspace}/scripts/capture_caib_marker_board_v4l2.py "
            "--device {center_gopro_dev} --width 1280 --height 720 --fps 30 --fourcc YUYV "
            "--camera-name center_gopro_superview --output-dir {out}/intrinsics/center_gopro_capture "
            "--aruco-dict {intr_dict} --start-id {intr_start_id} --marker-count {intr_marker_count} "
            "--target-samples 90 --min-markers 8 --max-motion-px 1.5 "
            "--capture-cooldown-s 1.2 --warmup-s 2.0 --preview-port 8765\n"
            "\n"
            "# If OpenCV blocks on /dev/video42, collect frames with ffmpeg instead:\n"
            "mkdir -p {out}/intrinsics/center_gopro_capture/frames\n"
            "for i in $(seq -w 1 90); do\n"
            "  read -r -p \"Place board pose $i, hold still, press Enter...\"\n"
            "  ffmpeg -y -f v4l2 -input_format yuyv422 -video_size 1280x720 "
            "-i {center_gopro_dev} -frames:v 1 {out}/intrinsics/center_gopro_capture/frames/capture_${{i}}.jpg\n"
            "done\n"
            "\n"
            "python3 {workspace}/scripts/calibrate_caib_marker_board_from_frames.py "
            "--frames-dir {out}/intrinsics/center_gopro_capture/frames --output-dir {out}/intrinsics "
            "--camera-name center_gopro_superview --cols {intr_cols} --rows {intr_rows} "
            "--square-m {intr_square_m} --marker-m {intr_marker_m} --start-id {intr_start_id} "
            "--marker-count {intr_marker_count} --aruco-dict {intr_dict} --min-markers 8 --min-frames 25",
        ),
    ),
    Step(
        id="generate_world_files",
        title="Generate world board and support-plane YAMLs",
        description="Create identity board YAML and support plane YAML for the fixed workspace target.",
        kind="action",
        action="generate_world_files",
    ),
    Step(
        id="vision_handeye_left",
        title="Solve left wrist hand-eye from vision",
        description=(
            "Launch the left arm, wrist camera preview, and browser jog UI. Capture varied board views, "
            "then solve world->left/base_link and left/gripper_frame_link->left/wrist_camera_optical_frame."
        ),
        kind="action",
        action="vision_handeye_left",
        prerequisites=("generate_controller_configs", "generate_joint_configs", "generate_world_files", "intrinsics_left"),
        required_config=("left_port", "left_joint_config", "left_wrist_dev", "left_wrist_info"),
    ),
    Step(
        id="vision_handeye_right",
        title="Solve right wrist hand-eye from vision",
        description=(
            "Launch the right arm, wrist camera preview, and browser jog UI. Capture varied board views, "
            "then solve world->right/base_link and right/gripper_frame_link->right/wrist_camera_optical_frame."
        ),
        kind="action",
        action="vision_handeye_right",
        prerequisites=("generate_controller_configs", "generate_joint_configs", "generate_world_files", "intrinsics_right"),
        required_config=("right_port", "right_joint_config", "right_wrist_dev", "right_wrist_info"),
    ),
    Step(
        id="camera_extrinsics",
        title="Solve center GoPro extrinsic",
        description="Run solve_camera_extrinsics_from_board.py for the fixed center GoPro in world.",
        kind="manual",
        prerequisites=("vision_handeye_left", "vision_handeye_right"),
        required_config=("center_gopro_dev", "center_gopro_info"),
        commands=(
            "ffmpeg -y -f v4l2 -input_format yuyv422 -video_size 1280x720 "
            "-i {center_gopro_dev} -frames:v 1 {out}/images/center_gopro_world_board.jpg\n"
            "\n"
            "python3 {workspace}/scripts/solve_camera_extrinsics_from_board.py "
            "--image {out}/images/center_gopro_world_board.jpg --camera-name center_gopro_optical_frame "
            "--camera-info {center_gopro_info} --board-in-base {out}/extrinsics/world_board_identity.yaml "
            "--output {out}/extrinsics/center_gopro_in_world.yaml "
            "--overlay-output {out}/images/center_gopro_world_board_overlay.jpg "
            "--frame-output {out}/images/center_gopro_world_board_frame.jpg --parent-frame world "
            "--cols {world_cols} --rows {world_rows} --square-m {world_square_m} --marker-m {world_marker_m} "
            "--start-id {world_start_id} --marker-count {world_marker_count} --aruco-dict {world_dict} "
            "--min-markers 8",
        ),
    ),
    Step(
        id="generate_camera_config",
        title="Generate per-run camera configs",
        description="Create xlerobot_cameras.yaml and xlerobot_opencv_cam.yaml for three cameras.",
        kind="action",
        action="generate_camera_config",
        required_config=("left_wrist_dev", "right_wrist_dev", "center_gopro_dev", "left_wrist_info", "right_wrist_info", "center_gopro_info"),
    ),
    Step(
        id="generate_grasp_runtime_config",
        title="Generate calibrated grasp runtime configs",
        description="Create side-specific grasp request, planner, and MoveIt config files for the current run.",
        kind="action",
        action="generate_grasp_runtime_config",
        prerequisites=("generate_camera_config", "vision_handeye_left", "vision_handeye_right", "camera_extrinsics"),
        required_config=("left_port", "right_port", "left_joint_config", "right_joint_config", "left_wrist_info", "right_wrist_info", "center_gopro_info"),
    ),
    Step(
        id="tf_validation",
        title="Validate full TF tree",
        description="Verify world to both bases, wrist cameras, and center GoPro optical frame.",
        kind="manual",
        prerequisites=("vision_handeye_left", "vision_handeye_right", "camera_extrinsics"),
    ),
    Step(
        id="perception_dry_run",
        title="Run perception dry run",
        description="Start grasp request nodes and call left/right detect_grasps without motion execution.",
        kind="manual",
        prerequisites=("tf_validation", "generate_camera_config", "generate_grasp_runtime_config"),
    ),
    Step(
        id="export_report",
        title="Export field report draft",
        description="Write a markdown report summarizing config, artifacts, and step status.",
        kind="action",
        action="export_report",
    ),
)


def steps_by_id() -> dict[str, Step]:
    return {step.id: step for step in STEPS}
