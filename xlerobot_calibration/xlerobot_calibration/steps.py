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
        description="Run LeRobot port discovery twice so the operator can identify left and right arms.",
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-find-port > {out}/logs/lerobot_find_port_1.txt 2>&1\n"
            "lerobot-find-port > {out}/logs/lerobot_find_port_2.txt 2>&1",
        ),
    ),
    Step(
        id="setup_motors_left",
        title="Setup left arm motors",
        description="Run LeRobot setup-motors for the left follower arm. This writes EEPROM.",
        required_config=("left_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-setup-motors --robot.type=so101_follower --robot.port={left_port} "
            "> {out}/logs/left_setup_motors.txt 2>&1",
        ),
    ),
    Step(
        id="setup_motors_right",
        title="Setup right arm motors",
        description="Run LeRobot setup-motors for the right follower arm. This writes EEPROM.",
        required_config=("right_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-setup-motors --robot.type=so101_follower --robot.port={right_port} "
            "> {out}/logs/right_setup_motors.txt 2>&1",
        ),
    ),
    Step(
        id="calibrate_left",
        title="Calibrate left arm with LeRobot",
        description="Run fresh LeRobot calibration for the left follower arm.",
        required_config=("left_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-calibrate --robot.type=so101_follower --robot.port={left_port} --robot.id=xlerobot_left "
            "> {out}/logs/left_lerobot_calibrate.txt 2>&1",
        ),
    ),
    Step(
        id="calibrate_right",
        title="Calibrate right arm with LeRobot",
        description="Run fresh LeRobot calibration for the right follower arm.",
        required_config=("right_port",),
        dangerous=True,
        commands=(
            "cd /home/dell/Documents/lerobot\n"
            "source .venv/bin/activate\n"
            "lerobot-calibrate --robot.type=so101_follower --robot.port={right_port} --robot.id=xlerobot_right "
            "> {out}/logs/right_lerobot_calibrate.txt 2>&1",
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
    ),
    Step(
        id="intrinsics_right",
        title="Capture and solve right wrist intrinsics",
        description="Run the right wrist caib.io capture and intrinsic solve commands from the runbook.",
        kind="manual",
        required_config=("right_wrist_dev",),
    ),
    Step(
        id="intrinsics_gopro",
        title="Capture and solve center GoPro intrinsics",
        description="Run the GoPro caib.io capture and intrinsic solve commands from the runbook.",
        kind="manual",
        required_config=("center_gopro_dev",),
    ),
    Step(
        id="generate_world_files",
        title="Generate world board and support-plane YAMLs",
        description="Create identity board YAML and support plane YAML for the fixed workspace target.",
        kind="action",
        action="generate_world_files",
    ),
    Step(
        id="touch_left_base",
        title="Touch fixed target with left arm",
        description="Run record_board_touch_points.py for left/base_link.",
        kind="manual",
        prerequisites=("state_only_left", "generate_world_files"),
        dangerous=True,
        commands=(
            "python3 {workspace}/scripts/record_board_touch_points.py "
            "--base-frame left/base_link --tool-frame left/gripper_frame_link --tool-offset '0 0 0' "
            "--cols {world_cols} --rows {world_rows} --square-m {world_square_m} --marker-m {world_marker_m} "
            "--start-id {world_start_id} --dictionary {world_dict} --corner-layout tl_tr_bl_br "
            "--samples 11 --output {out}/touch/left_base_to_world_board.yaml",
        ),
    ),
    Step(
        id="touch_right_base",
        title="Touch fixed target with right arm",
        description="Run record_board_touch_points.py for right/base_link.",
        kind="manual",
        prerequisites=("state_only_right", "generate_world_files"),
        dangerous=True,
        commands=(
            "python3 {workspace}/scripts/record_board_touch_points.py "
            "--base-frame right/base_link --tool-frame right/gripper_frame_link --tool-offset '0 0 0' "
            "--cols {world_cols} --rows {world_rows} --square-m {world_square_m} --marker-m {world_marker_m} "
            "--start-id {world_start_id} --dictionary {world_dict} --corner-layout tl_tr_bl_br "
            "--samples 11 --output {out}/touch/right_base_to_world_board.yaml",
        ),
    ),
    Step(
        id="invert_touch_solves",
        title="Invert touch solves into world-to-base TFs",
        description="Generate world_to_left_base.yaml, world_to_right_base.yaml, and static TF commands.",
        kind="action",
        action="invert_touch_solves",
        prerequisites=("touch_left_base", "touch_right_base"),
    ),
    Step(
        id="camera_extrinsics",
        title="Solve center and wrist camera extrinsics",
        description="Run solve_camera_extrinsics_from_board.py for center GoPro and wrist mount poses.",
        kind="manual",
        prerequisites=("invert_touch_solves",),
        required_config=("left_wrist_info", "right_wrist_info", "center_gopro_info"),
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
        id="tf_validation",
        title="Validate full TF tree",
        description="Verify world to both bases, wrist cameras, and center GoPro optical frame.",
        kind="manual",
        prerequisites=("invert_touch_solves", "camera_extrinsics"),
    ),
    Step(
        id="perception_dry_run",
        title="Run perception dry run",
        description="Start grasp request nodes and call left/right detect_grasps without motion execution.",
        kind="manual",
        prerequisites=("tf_validation", "generate_camera_config"),
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
