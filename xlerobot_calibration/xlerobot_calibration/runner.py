from __future__ import annotations

import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

import yaml

from .report import write_report
from .state import context, mark_step, save_state, step_status
from .steps import Step, steps_by_id


class StepError(RuntimeError):
    pass


ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def render(text: str, state: dict[str, Any]) -> str:
    return text.format_map(context(state))


def check_prerequisites(state: dict[str, Any], step: Step) -> None:
    missing = [item for item in step.prerequisites if step_status(state, item) != "complete"]
    if missing:
        raise StepError(f"missing prerequisites for {step.id}: {', '.join(missing)}")
    cfg = state.get("config", {})
    missing_cfg = [key for key in step.required_config if is_missing_config_value(cfg.get(key))]
    if missing_cfg:
        raise StepError(f"missing config for {step.id}: {', '.join(missing_cfg)}")


def is_missing_config_value(value: Any) -> bool:
    if value is None:
        return True
    text = str(value).strip()
    if not text:
        return True
    return "CONFIRMED" in text or text.startswith("/path/to/")


def shell_join(command: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def confirm(message: str, yes: bool) -> bool:
    if yes:
        return True
    answer = input(f"{message} [y/N] ").strip().lower()
    return answer in {"y", "yes"}


def run_step(state: dict[str, Any], step_id: str, *, dry_run: bool = False, yes: bool = False) -> None:
    step = steps_by_id().get(step_id)
    if step is None:
        raise StepError(f"unknown step: {step_id}")
    check_prerequisites(state, step)
    if step.dangerous and not dry_run:
        if not confirm(f"Step '{step.id}' may write hardware state or requires clear real hardware. Continue?", yes):
            raise StepError("operator declined dangerous step")

    if step.kind == "manual":
        run_manual_step(state, step, dry_run=dry_run, yes=yes)
    elif step.kind == "action":
        run_action_step(state, step, dry_run=dry_run, yes=yes)
    else:
        run_command_step(state, step, dry_run=dry_run)
    save_state(state)


def run_manual_step(state: dict[str, Any], step: Step, *, dry_run: bool, yes: bool) -> None:
    print(f"\n# {step.title}")
    print(step.description)
    if step.commands:
        print("\nCommands/instructions:")
        for command in step.commands:
            print(render(command, state))
            print("")
    if dry_run:
        mark_step(state, step.id, status="dry_run")
        return
    if confirm(step.manual_complete_prompt, yes):
        mark_step(state, step.id, status="complete")
    else:
        mark_step(state, step.id, status="pending")


def run_command_step(state: dict[str, Any], step: Step, *, dry_run: bool) -> None:
    command = "\n".join(render(command, state) for command in step.commands)
    log_path = Path(state["out_dir"]) / "logs" / f"{step.id}.log"
    if dry_run:
        print(command)
        mark_step(state, step.id, status="dry_run", log_path=str(log_path))
        return

    env = os.environ.copy()
    env.update(context(state))
    result = subprocess.run(
        command,
        cwd=state["workspace"],
        env=env,
        shell=True,
        executable="/bin/bash",
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    log_path.write_text(result.stdout or "", encoding="utf-8")
    if result.stdout:
        print(result.stdout)
    if result.returncode != 0:
        mark_step(
            state,
            step.id,
            status="failed",
            log_path=str(log_path),
            error=f"command exited {result.returncode}",
            returncode=result.returncode,
        )
        raise StepError(f"{step.id} failed, see {log_path}")
    mark_step(state, step.id, status="complete", log_path=str(log_path), returncode=result.returncode)


def build_touch_jog_commands(
    state: dict[str, Any],
    side: str,
    *,
    tool_offset: str = "0 0 0",
    corner_layout: str = "tl_tr_bl_br",
    samples: int = 11,
    jog_step_m: float = 0.002,
    max_jog_step_m: float = 0.10,
    jog_duration_sec: float = 1.5,
    jog_strategy: str = "cartesian",
    joint_step_rad: float = 0.035,
    command_speed: int | None = None,
    command_acceleration: int | None = None,
    arm_max_torque_limit: int | None = None,
    arm_protection_current: int | None = None,
    arm_overload_torque: int | None = None,
    web_port: int = 8780,
) -> tuple[list[str], list[str], list[str]]:
    if side not in {"left", "right"}:
        raise StepError(f"side must be left or right, got {side!r}")

    cfg = state.get("config", {})
    missing = [
        key
        for key in (
            f"{side}_port",
            f"{side}_joint_config",
            "world_cols",
            "world_rows",
            "world_square_m",
            "world_marker_m",
            "world_start_id",
            "world_dict",
        )
        if is_missing_config_value(cfg.get(key))
    ]
    if missing:
        raise StepError(f"missing config for touch-jog {side}: {', '.join(missing)}")

    workspace = str(Path(state["workspace"]))
    out = Path(state["out_dir"])
    controller_config = out / "config" / f"{side}_split_controllers.yaml"
    touch_output = out / "touch" / f"{side}_base_to_world_board.yaml"
    effective_command_speed = 2400 if command_speed is None else command_speed
    effective_command_acceleration = 50 if command_acceleration is None else command_acceleration
    joint_config = prepare_touch_jog_joint_config(
        state,
        side,
        command_speed=command_speed,
        command_acceleration=command_acceleration,
        arm_max_torque_limit=arm_max_torque_limit,
        arm_protection_current=arm_protection_current,
        arm_overload_torque=arm_overload_torque,
    )

    bringup_cmd = [
        "ros2",
        "launch",
        "so101_bringup",
        "follower_split.launch.py",
        f"namespace:={side}",
        f"frame_prefix:={side}/",
        "hardware_type:=real",
        f"usb_port:={cfg[f'{side}_port']}",
        f"joint_config_file:={joint_config}",
        f"controller_config_file:={controller_config}",
        "arm_controller:=arm_forward_controller",
        "use_rviz:=false",
    ]
    motion_cmd = [
        "ros2",
        "launch",
        "so101_kinematics",
        "cartesian_motion_split.launch.py",
        f"arm:={side}",
    ]
    recorder_cmd = [
        "python3",
        f"{workspace}/scripts/record_board_touch_points.py",
        "--base-frame",
        f"{side}/base_link",
        "--tool-frame",
        f"{side}/gripper_frame_link",
        "--tool-offset",
        tool_offset,
        "--cols",
        str(cfg["world_cols"]),
        "--rows",
        str(cfg["world_rows"]),
        "--square-m",
        str(cfg["world_square_m"]),
        "--marker-m",
        str(cfg["world_marker_m"]),
        "--start-id",
        str(cfg["world_start_id"]),
        "--dictionary",
        str(cfg["world_dict"]),
        "--corner-layout",
        corner_layout,
        "--samples",
        str(samples),
        "--jog-service",
        f"/{side}/go_to_pose",
        "--jog-step-m",
        str(jog_step_m),
        "--max-jog-step-m",
        str(max_jog_step_m),
        "--jog-duration-sec",
        str(jog_duration_sec),
        "--jog-strategy",
        jog_strategy,
        "--joint-jog-topic",
        f"/{side}/arm_forward_controller/commands",
        "--joint-states-topic",
        f"/{side}/joint_states",
        "--joint-step-rad",
        str(joint_step_rad),
        "--command-speed",
        str(effective_command_speed),
        "--command-acceleration",
        str(effective_command_acceleration),
        "--profile-command",
        f"./scripts/xlerobot_touch_jog.sh {side}",
        "--output",
        str(touch_output),
    ]
    if web_port > 0:
        recorder_cmd.extend(["--jog-web-port", str(web_port)])
    return bringup_cmd, motion_cmd, recorder_cmd


def build_vision_handeye_commands(
    state: dict[str, Any],
    side: str,
    *,
    target_samples: int = 30,
    min_samples: int = 15,
    min_markers: int = 8,
    max_reproj_px: float = 2.5,
    jog_step_m: float = 0.005,
    max_jog_step_m: float = 0.10,
    jog_duration_sec: float = 1.5,
    jog_strategy: str = "cartesian",
    joint_step_rad: float = 0.05235987755982989,
    command_speed: int | None = None,
    command_acceleration: int | None = None,
    arm_max_torque_limit: int | None = None,
    arm_protection_current: int | None = None,
    arm_overload_torque: int | None = None,
    web_port: int | None = None,
) -> tuple[list[str], list[str], list[str], list[str]]:
    if side not in {"left", "right"}:
        raise StepError(f"side must be left or right, got {side!r}")

    cfg = state.get("config", {})
    missing = [
        key
        for key in (
            f"{side}_port",
            f"{side}_joint_config",
            f"{side}_wrist_dev",
            f"{side}_wrist_info",
            "world_cols",
            "world_rows",
            "world_square_m",
            "world_marker_m",
            "world_start_id",
            "world_marker_count",
            "world_dict",
        )
        if is_missing_config_value(cfg.get(key))
    ]
    if missing:
        raise StepError(f"missing config for vision-handeye {side}: {', '.join(missing)}")

    workspace = str(Path(state["workspace"]))
    out = Path(state["out_dir"])
    controller_config = out / "config" / f"{side}_split_controllers.yaml"
    handeye_dir = out / "handeye" / side
    effective_command_speed = 2400 if command_speed is None else command_speed
    effective_command_acceleration = 50 if command_acceleration is None else command_acceleration
    effective_web_port = (8780 if side == "left" else 8781) if web_port is None else web_port
    joint_config = prepare_touch_jog_joint_config(
        state,
        side,
        command_speed=command_speed,
        command_acceleration=command_acceleration,
        arm_max_torque_limit=arm_max_torque_limit,
        arm_protection_current=arm_protection_current,
        arm_overload_torque=arm_overload_torque,
        profile_name="vision_handeye",
    )

    bringup_cmd = [
        "ros2",
        "launch",
        "so101_bringup",
        "follower_split.launch.py",
        f"namespace:={side}",
        f"frame_prefix:={side}/",
        "hardware_type:=real",
        f"usb_port:={cfg[f'{side}_port']}",
        f"joint_config_file:={joint_config}",
        f"controller_config_file:={controller_config}",
        "arm_controller:=arm_forward_controller",
        "use_rviz:=false",
    ]
    motion_cmd = [
        "ros2",
        "launch",
        "so101_kinematics",
        "cartesian_motion_split.launch.py",
        f"arm:={side}",
    ]
    capture_cmd = [
        "python3",
        f"{workspace}/scripts/capture_wrist_handeye_dataset.py",
        "--device",
        str(cfg[f"{side}_wrist_dev"]),
        "--width",
        "1280",
        "--height",
        "800",
        "--fps",
        "30",
        "--fourcc",
        "MJPG",
        "--camera-name",
        f"{side}_wrist_arducam",
        "--camera-info",
        str(cfg[f"{side}_wrist_info"]),
        "--output-dir",
        str(handeye_dir),
        "--world-frame",
        "world",
        "--base-frame",
        f"{side}/base_link",
        "--gripper-frame",
        f"{side}/gripper_frame_link",
        "--camera-frame",
        f"{side}/wrist_camera_optical_frame",
        "--cols",
        str(cfg["world_cols"]),
        "--rows",
        str(cfg["world_rows"]),
        "--square-m",
        str(cfg["world_square_m"]),
        "--marker-m",
        str(cfg["world_marker_m"]),
        "--start-id",
        str(cfg["world_start_id"]),
        "--marker-count",
        str(cfg["world_marker_count"]),
        "--aruco-dict",
        str(cfg["world_dict"]),
        "--min-markers",
        str(min_markers),
        "--max-sample-reproj-px",
        str(max_reproj_px),
        "--target-samples",
        str(target_samples),
        "--jog-service",
        f"/{side}/go_to_pose",
        "--jog-step-m",
        str(jog_step_m),
        "--max-jog-step-m",
        str(max_jog_step_m),
        "--jog-duration-sec",
        str(jog_duration_sec),
        "--jog-strategy",
        jog_strategy,
        "--joint-jog-topic",
        f"/{side}/arm_forward_controller/commands",
        "--joint-states-topic",
        f"/{side}/joint_states",
        "--joint-step-rad",
        str(joint_step_rad),
        "--command-speed",
        str(effective_command_speed),
        "--command-acceleration",
        str(effective_command_acceleration),
        "--profile-command",
        f"./scripts/xlerobot_vision_handeye.sh {side}",
    ]
    if effective_web_port > 0:
        capture_cmd.extend(["--web-port", str(effective_web_port)])
    solve_cmd = [
        "python3",
        f"{workspace}/scripts/solve_wrist_robot_world_handeye.py",
        "--samples",
        str(handeye_dir / "samples.jsonl"),
        "--output-dir",
        str(out / "extrinsics"),
        "--side",
        side,
        "--min-samples",
        str(min_samples),
        "--min-markers",
        str(min_markers),
        "--max-reproj-px",
        str(max_reproj_px),
        "--world-frame",
        "world",
        "--base-frame",
        f"{side}/base_link",
        "--gripper-frame",
        f"{side}/gripper_frame_link",
        "--camera-frame",
        f"{side}/wrist_camera_optical_frame",
    ]
    return bringup_cmd, motion_cmd, capture_cmd, solve_cmd


def prepare_touch_jog_joint_config(
    state: dict[str, Any],
    side: str,
    *,
    command_speed: int | None = None,
    command_acceleration: int | None = None,
    arm_max_torque_limit: int | None = None,
    arm_protection_current: int | None = None,
    arm_overload_torque: int | None = None,
    profile_name: str = "touch_jog",
) -> str:
    validate_optional_range("command_speed", command_speed, 0, 32767)
    validate_optional_range("command_acceleration", command_acceleration, 0, 255)
    validate_optional_range("arm_max_torque_limit", arm_max_torque_limit, 0, 4095)
    validate_optional_range("arm_protection_current", arm_protection_current, 0, 4095)
    validate_optional_range("arm_overload_torque", arm_overload_torque, 0, 255)
    base_path = Path(str(state["config"][f"{side}_joint_config"]))
    overrides = {
        "command_speed": command_speed,
        "command_acceleration": command_acceleration,
        "max_torque_limit": arm_max_torque_limit,
        "protection_current": arm_protection_current,
        "overload_torque": arm_overload_torque,
    }
    if all(value is None for value in overrides.values()):
        return str(base_path)

    data = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    joints = data.get("joints")
    if not isinstance(joints, dict):
        raise StepError(f"{base_path} has no top-level joints map")

    for name in ARM_JOINTS:
        if name not in joints:
            raise StepError(f"{base_path} is missing arm joint {name}")
        row = joints[name]
        for key, value in overrides.items():
            if value is not None:
                row[key] = int(value)

    target = Path(state["out_dir"]) / "config" / f"{side}_{profile_name}_joints.yaml"
    target.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return str(target)


def validate_optional_range(name: str, value: int | None, minimum: int, maximum: int) -> None:
    if value is None:
        return
    if value < minimum or value > maximum:
        raise StepError(f"{name} must be in [{minimum}, {maximum}], got {value}")


def run_touch_jog(
    state: dict[str, Any],
    side: str,
    *,
    tool_offset: str = "0 0 0",
    corner_layout: str = "tl_tr_bl_br",
    samples: int = 11,
    jog_step_m: float = 0.002,
    max_jog_step_m: float = 0.10,
    jog_duration_sec: float = 1.5,
    jog_strategy: str = "cartesian",
    joint_step_rad: float = 0.035,
    command_speed: int | None = None,
    command_acceleration: int | None = None,
    arm_max_torque_limit: int | None = None,
    arm_protection_current: int | None = None,
    arm_overload_torque: int | None = None,
    web_port: int = 8780,
    stop_existing: bool = True,
    dry_run: bool = False,
    yes: bool = False,
) -> None:
    if step_status(state, "generate_world_files") != "complete":
        raise StepError("missing prerequisite for touch-jog: generate_world_files")
    if step_status(state, "generate_controller_configs") != "complete":
        raise StepError("missing prerequisite for touch-jog: generate_controller_configs")
    if step_status(state, "generate_joint_configs") != "complete":
        raise StepError("missing prerequisite for touch-jog: generate_joint_configs")
    validate_optional_range("command_speed", command_speed, 0, 32767)
    validate_optional_range("command_acceleration", command_acceleration, 0, 255)
    validate_optional_range("arm_max_torque_limit", arm_max_torque_limit, 0, 4095)
    validate_optional_range("arm_protection_current", arm_protection_current, 0, 4095)
    validate_optional_range("arm_overload_torque", arm_overload_torque, 0, 255)

    bringup_cmd, motion_cmd, recorder_cmd = build_touch_jog_commands(
        state,
        side,
        tool_offset=tool_offset,
        corner_layout=corner_layout,
        samples=samples,
        jog_step_m=jog_step_m,
        max_jog_step_m=max_jog_step_m,
        jog_duration_sec=jog_duration_sec,
        jog_strategy=jog_strategy,
        joint_step_rad=joint_step_rad,
        command_speed=command_speed,
        command_acceleration=command_acceleration,
        arm_max_torque_limit=arm_max_torque_limit,
        arm_protection_current=arm_protection_current,
        arm_overload_torque=arm_overload_torque,
        web_port=web_port,
    )

    print("")
    print(f"# Touch jog: {side}")
    print("This starts a real command controller and sends small jog moves.")
    print("Any existing same-side state-only/jog launch will be stopped first.")
    tuning_values = {
        "command_speed": command_speed,
        "command_acceleration": command_acceleration,
        "arm_max_torque_limit": arm_max_torque_limit,
        "arm_protection_current": arm_protection_current,
        "arm_overload_torque": arm_overload_torque,
    }
    enabled_tuning = {key: value for key, value in tuning_values.items() if value is not None}
    if enabled_tuning:
        print("Touch-jog motor tuning overrides:")
        for key, value in enabled_tuning.items():
            print(f"  {key}: {value}")
        if any(key.startswith("arm_") for key in enabled_tuning):
            print("  Note: arm torque/current overrides are written to servo registers at launch.")
    print("")
    print("Commands that will run:")
    print(shell_join(bringup_cmd))
    print(shell_join(motion_cmd))
    print(shell_join(recorder_cmd))
    existing = find_side_control_processes(side)
    if existing:
        print("")
        print(f"Existing {side} ROS control processes detected:")
        for proc in existing:
            print(f"  pid={proc['pid']} {proc['cmd']}")
        if not stop_existing:
            raise StepError(f"existing {side} ROS control processes are running")
    print("")

    if dry_run:
        return
    if not confirm("Workspace is clear and this arm is safe to command?", yes):
        raise StepError("operator declined touch-jog")

    workspace = Path(state["workspace"])
    out = Path(state["out_dir"])
    logs = out / "logs"
    bringup_log = logs / f"{side}_touch_jog_bringup.log"
    motion_log = logs / f"{side}_touch_jog_motion.log"
    env = os.environ.copy()
    env.update(context(state))
    env["PYTHONUNBUFFERED"] = "1"

    processes: list[subprocess.Popen[str]] = []

    def start_logged(command: list[str], log_path: Path) -> subprocess.Popen[str]:
        log_file = log_path.open("w", encoding="utf-8")
        proc = subprocess.Popen(
            command,
            cwd=workspace,
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        proc._xlerobot_log_file = log_file  # type: ignore[attr-defined]
        processes.append(proc)
        return proc

    try:
        if existing and stop_existing:
            print(f"Stopping existing {side} ROS control processes...")
            stop_side_control_processes(side)
            _wait_for_controller_manager_absent(side, timeout_s=8.0)

        print(f"Starting arm bringup; log: {bringup_log}")
        bringup = start_logged(bringup_cmd, bringup_log)
        _wait_for_controller_active(side, bringup, bringup_log, timeout_s=18.0)

        print(f"Starting Cartesian motion node; log: {motion_log}")
        motion = start_logged(motion_cmd, motion_log)
        _wait_for_service(f"/{side}/go_to_pose", motion, motion_log, timeout_s=12.0)

        print("")
        print("Recorder starting. At each corner prompt, use x+/x-/y+/y-/z+/z- then Enter/sample.")
        result = subprocess.run(recorder_cmd, cwd=workspace, env=env, text=True, check=False)
        if result.returncode != 0:
            mark_step(state, f"touch_{side}_base", status="failed", error=f"touch jog exited {result.returncode}")
            raise StepError(f"touch jog exited {result.returncode}")

        artifact = out / "touch" / f"{side}_base_to_world_board.yaml"
        mark_step(state, f"touch_{side}_base", status="complete", artifacts=[str(artifact)])
        print(f"Saved: {artifact}")
    finally:
        for proc in reversed(processes):
            _terminate_process_group(proc)
            log_file = getattr(proc, "_xlerobot_log_file", None)
            if log_file is not None:
                log_file.close()


def run_vision_handeye(
    state: dict[str, Any],
    side: str,
    *,
    target_samples: int = 30,
    min_samples: int = 15,
    min_markers: int = 8,
    max_reproj_px: float = 2.5,
    jog_step_m: float = 0.005,
    max_jog_step_m: float = 0.10,
    jog_duration_sec: float = 1.5,
    jog_strategy: str = "cartesian",
    joint_step_rad: float = 0.05235987755982989,
    command_speed: int | None = None,
    command_acceleration: int | None = None,
    arm_max_torque_limit: int | None = None,
    arm_protection_current: int | None = None,
    arm_overload_torque: int | None = None,
    web_port: int | None = None,
    stop_existing: bool = True,
    dry_run: bool = False,
    yes: bool = False,
) -> None:
    if step_status(state, "generate_world_files") != "complete":
        raise StepError("missing prerequisite for vision-handeye: generate_world_files")
    if step_status(state, "generate_controller_configs") != "complete":
        raise StepError("missing prerequisite for vision-handeye: generate_controller_configs")
    if step_status(state, "generate_joint_configs") != "complete":
        raise StepError("missing prerequisite for vision-handeye: generate_joint_configs")
    validate_optional_range("command_speed", command_speed, 0, 32767)
    validate_optional_range("command_acceleration", command_acceleration, 0, 255)
    validate_optional_range("arm_max_torque_limit", arm_max_torque_limit, 0, 4095)
    validate_optional_range("arm_protection_current", arm_protection_current, 0, 4095)
    validate_optional_range("arm_overload_torque", arm_overload_torque, 0, 255)

    bringup_cmd, motion_cmd, capture_cmd, solve_cmd = build_vision_handeye_commands(
        state,
        side,
        target_samples=target_samples,
        min_samples=min_samples,
        min_markers=min_markers,
        max_reproj_px=max_reproj_px,
        jog_step_m=jog_step_m,
        max_jog_step_m=max_jog_step_m,
        jog_duration_sec=jog_duration_sec,
        jog_strategy=jog_strategy,
        joint_step_rad=joint_step_rad,
        command_speed=command_speed,
        command_acceleration=command_acceleration,
        arm_max_torque_limit=arm_max_torque_limit,
        arm_protection_current=arm_protection_current,
        arm_overload_torque=arm_overload_torque,
        web_port=web_port,
    )

    print("")
    print(f"# Vision hand-eye: {side}")
    print("This starts a real command controller, wrist camera preview, and sample recorder.")
    print("Any existing same-side state-only/jog launch will be stopped first.")
    tuning_values = {
        "command_speed": command_speed,
        "command_acceleration": command_acceleration,
        "arm_max_torque_limit": arm_max_torque_limit,
        "arm_protection_current": arm_protection_current,
        "arm_overload_torque": arm_overload_torque,
    }
    enabled_tuning = {key: value for key, value in tuning_values.items() if value is not None}
    if enabled_tuning:
        print("Vision hand-eye motor tuning overrides:")
        for key, value in enabled_tuning.items():
            print(f"  {key}: {value}")
        if any(key.startswith("arm_") for key in enabled_tuning):
            print("  Note: arm torque/current overrides are written to servo registers at launch.")
    print("")
    print("Commands that will run:")
    print(shell_join(bringup_cmd))
    print(shell_join(motion_cmd))
    print(shell_join(capture_cmd))
    print(shell_join(solve_cmd))
    existing = find_side_control_processes(side)
    if existing:
        print("")
        print(f"Existing {side} ROS control processes detected:")
        for proc in existing:
            print(f"  pid={proc['pid']} {proc['cmd']}")
        if not stop_existing:
            raise StepError(f"existing {side} ROS control processes are running")
    print("")

    step_id = f"vision_handeye_{side}"
    if dry_run:
        mark_step(state, step_id, status="dry_run")
        return
    if not confirm("Workspace is clear and this arm is safe to command?", yes):
        raise StepError("operator declined vision-handeye")

    workspace = Path(state["workspace"])
    out = Path(state["out_dir"])
    logs = out / "logs"
    bringup_log = logs / f"{side}_vision_handeye_bringup.log"
    motion_log = logs / f"{side}_vision_handeye_motion.log"
    capture_log = logs / f"{side}_vision_handeye_capture.log"
    solve_log = logs / f"{side}_vision_handeye_solve.log"
    env = os.environ.copy()
    env.update(context(state))
    env["PYTHONUNBUFFERED"] = "1"

    processes: list[subprocess.Popen[str]] = []

    def start_logged(command: list[str], log_path: Path) -> subprocess.Popen[str]:
        log_file = log_path.open("w", encoding="utf-8")
        proc = subprocess.Popen(
            command,
            cwd=workspace,
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        proc._xlerobot_log_file = log_file  # type: ignore[attr-defined]
        processes.append(proc)
        return proc

    try:
        if existing and stop_existing:
            print(f"Stopping existing {side} ROS control processes...")
            stop_side_control_processes(side)
            _wait_for_controller_manager_absent(side, timeout_s=8.0)

        print(f"Starting arm bringup; log: {bringup_log}")
        bringup = start_logged(bringup_cmd, bringup_log)
        _wait_for_controller_active(side, bringup, bringup_log, timeout_s=18.0)

        print(f"Starting Cartesian motion node; log: {motion_log}")
        motion = start_logged(motion_cmd, motion_log)
        _wait_for_service(f"/{side}/go_to_pose", motion, motion_log, timeout_s=12.0)

        print("")
        print("Recorder starting. Use the web page to jog, vary wrist poses, and collect samples.")
        with capture_log.open("w", encoding="utf-8") as log_file:
            capture = subprocess.run(
                capture_cmd,
                cwd=workspace,
                env=env,
                text=True,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                check=False,
            )
        if capture.returncode != 0:
            mark_step(state, step_id, status="failed", log_path=str(capture_log), error=f"capture exited {capture.returncode}")
            raise StepError(f"vision hand-eye capture exited {capture.returncode}, see {capture_log}")

        print(f"Solving hand-eye; log: {solve_log}")
        with solve_log.open("w", encoding="utf-8") as log_file:
            solve = subprocess.run(
                solve_cmd,
                cwd=workspace,
                env=env,
                text=True,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                check=False,
            )
        if solve.returncode != 0:
            mark_step(state, step_id, status="failed", log_path=str(solve_log), error=f"solve exited {solve.returncode}")
            raise StepError(f"vision hand-eye solve exited {solve.returncode}, see {solve_log}")

        artifacts = [
            out / "handeye" / side / "samples.jsonl",
            out / "handeye" / side / "samples_summary.json",
            out / "extrinsics" / f"world_to_{side}_base.yaml",
            out / "extrinsics" / f"{side}_wrist_camera_in_gripper.yaml",
            out / "extrinsics" / f"{side}_vision_handeye_summary.json",
            out / "extrinsics" / "vision_handeye_summary.json",
            out / "logs" / "static_tf_world_bases.sh",
            out / "logs" / "static_tf_wrist_cameras.sh",
        ]
        mark_step(state, step_id, status="complete", log_path=str(solve_log), artifacts=[str(path) for path in artifacts])
        print(f"Saved hand-eye artifacts under {out / 'handeye' / side} and {out / 'extrinsics'}")
    finally:
        for proc in reversed(processes):
            _terminate_process_group(proc)
            log_file = getattr(proc, "_xlerobot_log_file", None)
            if log_file is not None:
                log_file.close()


def find_side_control_processes(side: str) -> list[dict[str, str]]:
    result = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    side_tokens = (f"namespace:={side}", f"__ns:=/{side}")
    control_tokens = (
        "follower_split.launch.py",
        "follower_state_only.launch.py",
        "cartesian_motion_split.launch.py",
        "cartesian_motion_node",
        "ros2_control_node",
        "robot_state_publisher",
        "controller_manager/spawner",
    )
    current_pid = os.getpid()
    found: list[dict[str, str]] = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        pid_text, _, cmd = line.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == current_pid:
            continue
        if not any(token in cmd for token in side_tokens):
            continue
        if not any(token in cmd for token in control_tokens):
            continue
        found.append({"pid": str(pid), "cmd": cmd.strip()})
    return found


def stop_side_control_processes(side: str) -> None:
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        procs = find_side_control_processes(side)
        if not procs:
            return
        for proc in procs:
            try:
                os.kill(int(proc["pid"]), sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                pass
        time.sleep(2.0 if sig != signal.SIGKILL else 0.5)


def _wait_for_controller_manager_absent(side: str, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _service_exists(f"/{side}/controller_manager/list_controllers"):
            return
        time.sleep(0.3)


def _wait_for_controller_active(
    side: str,
    bringup: subprocess.Popen[str],
    log_path: Path,
    *,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    last_output = ""
    while time.monotonic() < deadline:
        if bringup.poll() is not None:
            raise StepError(_process_failure_message("arm bringup", bringup, log_path))
        result = subprocess.run(
            ["ros2", "control", "list_controllers", "-c", f"/{side}/controller_manager"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            timeout=3.0,
        )
        last_output = result.stdout or ""
        if (
            "joint_state_broadcaster" in last_output
            and "joint_state_broadcaster" in _active_controller_lines(last_output)
            and "arm_forward_controller" in _active_controller_lines(last_output)
        ):
            print(f"{side} arm_forward_controller is active.")
            return
        time.sleep(0.5)
    tail = _tail_file(log_path, 30)
    raise StepError(
        f"{side} arm_forward_controller did not become active.\n"
        f"Last controller output:\n{last_output}\n"
        f"Bringup log tail:\n{tail}"
    )


def _active_controller_lines(text: str) -> str:
    return "\n".join(line for line in text.splitlines() if " active" in line)


def _wait_for_service(
    service_name: str,
    proc: subprocess.Popen[str],
    log_path: Path,
    *,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise StepError(_process_failure_message("Cartesian motion node", proc, log_path))
        if _service_exists(service_name):
            print(f"{service_name} is available.")
            return
        time.sleep(0.3)
    raise StepError(f"{service_name} did not become available.\n{_tail_file(log_path, 30)}")


def _service_exists(service_name: str) -> bool:
    result = subprocess.run(
        ["ros2", "service", "list"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
        timeout=3.0,
    )
    return service_name in result.stdout.splitlines()


def _terminate_process_group(proc: subprocess.Popen[str]) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5.0)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass


def _process_failure_message(label: str, proc: subprocess.Popen[str], log_path: Path) -> str:
    tail = _tail_file(log_path, 20)
    return f"{label} exited {proc.returncode}; see {log_path}\n{tail}"


def _tail_file(path: Path, lines: int) -> str:
    if not path.exists():
        return ""
    return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])


def run_action_step(state: dict[str, Any], step: Step, *, dry_run: bool, yes: bool = False) -> None:
    if step.action == "vision_handeye_left":
        run_vision_handeye(state, "left", dry_run=dry_run, yes=yes)
        return
    if step.action == "vision_handeye_right":
        run_vision_handeye(state, "right", dry_run=dry_run, yes=yes)
        return
    if dry_run:
        print(f"Would run action: {step.action}")
        mark_step(state, step.id, status="dry_run")
        return
    actions = {
        "generate_controller_configs": generate_controller_configs,
        "generate_joint_configs": generate_joint_configs,
        "generate_world_files": generate_world_files,
        "invert_touch_solves": invert_touch_solves,
        "generate_camera_config": generate_camera_config,
        "generate_grasp_runtime_config": generate_grasp_runtime_config,
        "export_report": export_report,
    }
    action = actions.get(step.action)
    if action is None:
        raise StepError(f"unknown action: {step.action}")
    artifacts = action(state)
    mark_step(state, step.id, status="complete", artifacts=[str(path) for path in artifacts])


def generate_controller_configs(state: dict[str, Any]) -> list[Path]:
    workspace = Path(state["workspace"])
    source = workspace / "so101_bringup/config/ros2_control/follower_split_controllers.yaml"
    text = source.read_text(encoding="utf-8")
    out_dir = Path(state["out_dir"]) / "config"
    artifacts = []
    for namespace in ("left", "right"):
        path = out_dir / f"{namespace}_split_controllers.yaml"
        path.write_text(text.replace("follower:", f"{namespace}:", 1), encoding="utf-8")
        artifacts.append(path)
    return artifacts


def generate_joint_configs(state: dict[str, Any]) -> list[Path]:
    cfg = state.setdefault("config", {})
    out_dir = Path(state["out_dir"]) / "config"
    artifacts = []
    for side, key in (("left", "left_lerobot_json"), ("right", "right_lerobot_json")):
        source = Path(str(cfg.get(key, ""))).expanduser()
        if not source.exists():
            raise StepError(f"{key} does not exist: {source}")
        data = json.loads(source.read_text(encoding="utf-8"))
        joints = {}
        for name, values in data.items():
            row = {
                "id": int(values["id"]),
                "homing_offset": int(values["homing_offset"]),
                "range_min": int(values["range_min"]),
                "range_max": int(values["range_max"]),
                "p_coefficient": 16,
                "i_coefficient": 0,
                "d_coefficient": 32,
                "return_delay_time": 0,
                "acceleration": 254,
            }
            if name == "gripper":
                row.update({"max_torque_limit": 500, "protection_current": 250, "overload_torque": 25})
            joints[name] = row
        target = out_dir / f"{side}_joints_from_lerobot.yaml"
        target.write_text(yaml.safe_dump({"joints": joints}, sort_keys=False), encoding="utf-8")
        cfg[f"{side}_joint_config"] = str(target)
        artifacts.append(target)
    return artifacts


def generate_world_files(state: dict[str, Any]) -> list[Path]:
    cfg = state["config"]
    out_dir = Path(state["out_dir"]) / "extrinsics"
    width = int(cfg["world_cols"]) * float(cfg["world_square_m"])
    height = int(cfg["world_rows"]) * float(cfg["world_square_m"])
    identity = {
        "timestamp_utc": state.get("run_id", ""),
        "transform": {
            "name": "T_world_board",
            "parent_frame": "world",
            "child_frame": "calibration_board",
            "translation_xyz": [0.0, 0.0, 0.0],
            "rotation_matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "board": {
            "source": "fixed workspace caib.io marker board",
            "dictionary": cfg["world_dict"],
            "start_id": int(cfg["world_start_id"]),
            "marker_count": int(cfg["world_marker_count"]),
            "cols": int(cfg["world_cols"]),
            "rows": int(cfg["world_rows"]),
            "square_m": float(cfg["world_square_m"]),
            "marker_m": float(cfg["world_marker_m"]),
            "width_m": width,
            "height_m": height,
        },
    }
    support = {
        "points_xyz_in_base": {
            "top_left": [0.0, 0.0, 0.0],
            "bottom_left": [0.0, height, 0.0],
            "bottom_right": [width, height, 0.0],
        },
        "note": "World support plane. Valid only if the fixed target plane is the object support plane.",
    }
    identity_path = out_dir / "world_board_identity.yaml"
    support_path = out_dir / "world_support_plane.yaml"
    identity_path.write_text(yaml.safe_dump(identity, sort_keys=False), encoding="utf-8")
    support_path.write_text(yaml.safe_dump(support, sort_keys=False), encoding="utf-8")
    return [identity_path, support_path]


def invert_touch_solves(state: dict[str, Any]) -> list[Path]:
    out = Path(state["out_dir"])
    artifacts: list[Path] = []
    commands = []
    for side in ("left", "right"):
        source = out / "touch" / f"{side}_base_to_world_board.yaml"
        if not source.exists():
            raise StepError(f"missing touch solve: {source}")
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        base_board = _load_transform_matrix(data["transform"])
        world_base = _invert_transform(base_board)
        quat = _matrix_to_quat_xyzw(world_base[:3, :3])
        translation = world_base[:3, 3]
        target = out / "extrinsics" / f"world_to_{side}_base.yaml"
        result = {
            "transform": {
                "name": f"T_world_{side}_base",
                "parent_frame": "world",
                "child_frame": f"{side}/base_link",
                "translation_xyz": translation.tolist(),
                "rotation_matrix": world_base[:3, :3].tolist(),
                "quaternion_xyzw": quat,
            },
            "source": str(source),
        }
        target.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        artifacts.append(target)
        commands.append(
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {translation[0]:.9f} --y {translation[1]:.9f} --z {translation[2]:.9f} "
            f"--qx {quat[0]:.9f} --qy {quat[1]:.9f} --qz {quat[2]:.9f} --qw {quat[3]:.9f} "
            f"--frame-id world --child-frame-id {side}/base_link"
        )
    script = out / "logs" / "static_tf_world_bases.sh"
    script.write_text("#!/usr/bin/env bash\nset -euo pipefail\n" + "\n".join(commands) + "\n", encoding="utf-8")
    artifacts.append(script)
    return artifacts


def generate_camera_config(state: dict[str, Any]) -> list[Path]:
    cfg = state["config"]
    out = Path(state["out_dir"])
    params_path = out / "config" / "xlerobot_opencv_cam.yaml"
    cameras_path = out / "config" / "xlerobot_cameras.yaml"
    params = {
        "/**": {"ros__parameters": {"framerate": 30.0, "jpeg_quality": 85}},
        "/left/cam_wrist": {
            "ros__parameters": {
                "video_device": cfg["left_wrist_dev"],
                "fourcc": "MJPG",
                "image_width": 1280,
                "image_height": 800,
                "fallback_fov_degrees": 70.0,
                "frame_id": "left/wrist_camera_optical_frame",
                "camera_name": "left_wrist_arducam",
            }
        },
        "/right/cam_wrist": {
            "ros__parameters": {
                "video_device": cfg["right_wrist_dev"],
                "fourcc": "MJPG",
                "image_width": 1280,
                "image_height": 800,
                "fallback_fov_degrees": 70.0,
                "frame_id": "right/wrist_camera_optical_frame",
                "camera_name": "right_wrist_arducam",
            }
        },
        "/center_gopro/cam_overhead": {
            "ros__parameters": {
                "video_device": cfg["center_gopro_dev"],
                "fourcc": "YUYV",
                "capture_backend": "ffmpeg_raw",
                "ffmpeg_input_format": "yuyv422",
                "image_width": 1280,
                "image_height": 720,
                "fallback_fov_degrees": 113.0,
                "frame_id": "center_gopro_optical_frame",
                "camera_name": "center_gopro_superview",
            }
        },
    }
    cameras = {
        "cameras": [
            {
                "name": "cam_wrist",
                "camera_type": "opencv_compressed",
                "param_path": str(params_path),
                "namespace": "left",
                "camera_info_url": "file://" + str(cfg["left_wrist_info"]),
            },
            {
                "name": "cam_wrist",
                "camera_type": "opencv_compressed",
                "param_path": str(params_path),
                "namespace": "right",
                "camera_info_url": "file://" + str(cfg["right_wrist_info"]),
            },
            {
                "name": "cam_overhead",
                "camera_type": "opencv_compressed",
                "param_path": str(params_path),
                "namespace": "center_gopro",
                "camera_info_url": "file://" + str(cfg["center_gopro_info"]),
            },
        ]
    }
    params_path.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
    cameras_path.write_text(yaml.safe_dump(cameras, sort_keys=False), encoding="utf-8")
    return [params_path, cameras_path]


def _grasp_support_plane_params(out: Path) -> tuple[list[float], list[float]]:
    support_path = out / "extrinsics" / "world_support_plane.yaml"
    if not support_path.exists():
        return [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]
    data = yaml.safe_load(support_path.read_text(encoding="utf-8")) or {}
    points = data.get("points_xyz_in_base", {})
    try:
        top_left = [float(value) for value in points["top_left"]]
        bottom_left = [float(value) for value in points["bottom_left"]]
        bottom_right = [float(value) for value in points["bottom_right"]]
    except (KeyError, TypeError, ValueError) as exc:
        raise StepError(f"invalid support plane file for grasp runtime: {support_path}") from exc
    if len(top_left) != 3 or len(bottom_left) != 3 or len(bottom_right) != 3:
        raise StepError(f"invalid support plane point length in {support_path}")

    x_axis = [bottom_right[i] - bottom_left[i] for i in range(3)]
    y_axis = [bottom_left[i] - top_left[i] for i in range(3)]
    normal = [
        x_axis[1] * y_axis[2] - x_axis[2] * y_axis[1],
        x_axis[2] * y_axis[0] - x_axis[0] * y_axis[2],
        x_axis[0] * y_axis[1] - x_axis[1] * y_axis[0],
    ]
    norm = math.sqrt(sum(value * value for value in normal))
    if norm < 1e-9:
        raise StepError(f"degenerate support plane in {support_path}")
    normal = [value / norm for value in normal]
    if normal[2] < 0.0:
        normal = [-value for value in normal]
    return top_left, normal


def generate_grasp_runtime_config(state: dict[str, Any]) -> list[Path]:
    cfg = state["config"]
    out = Path(state["out_dir"])
    config_dir = out / "config"
    artifacts: list[Path] = []

    base_moveit_controllers = Path(state["workspace"]) / "so101_moveit_config/config/moveit_controllers.yaml"
    base_moveit_py = Path(state["workspace"]) / "so101_moveit_config/config/moveit_py_config.yaml"
    controllers_template = base_moveit_controllers.read_text(encoding="utf-8")
    moveit_py_template = base_moveit_py.read_text(encoding="utf-8")

    default_server = str(cfg.get("grasp_server_address", "127.0.0.1:8091"))
    default_prompt = str(cfg.get("prompt", "pink cube"))
    default_top_k = int(cfg.get("top_k", 8))
    support_plane_point, support_plane_normal = _grasp_support_plane_params(out)

    for side in ("left", "right"):
        wrist_yaml = out / "extrinsics" / f"{side}_wrist_camera_in_gripper.yaml"
        if not wrist_yaml.exists():
            raise StepError(f"missing wrist camera extrinsic for {side}: {wrist_yaml}")
        wrist_data = yaml.safe_load(wrist_yaml.read_text(encoding="utf-8")) or {}
        wrist_transform = wrist_data.get("transform", {})
        wrist_translation = [float(value) for value in wrist_transform.get("translation_xyz", [])]
        wrist_quat = [float(value) for value in wrist_transform.get("quaternion_xyzw", [])]
        if len(wrist_translation) != 3 or len(wrist_quat) != 4:
            raise StepError(f"invalid wrist camera extrinsic for {side}: {wrist_yaml}")
        other_side = "right" if side == "left" else "left"

        grasping = {
            f"/{side}_grasp/grasp_request_node": {
                "ros__parameters": {
                    "server_address": default_server,
                    "recv_timeout_ms": 120000,
                    "send_timeout_ms": 5000,
                    "default_top_k": default_top_k,
                    "max_data_age_s": 0.75,
                    "base_frame": "world",
                    "overhead_camera_frame": "center_gopro_optical_frame",
                    "wrist_camera_frame": f"{side}/wrist_camera_optical_frame",
                    "overhead_image_topic": "/center_gopro/image_raw/compressed",
                    "wrist_image_topic": f"/{side}/image_raw/compressed",
                    "overhead_camera_info_topic": "/center_gopro/camera_info",
                    "wrist_camera_info_topic": f"/{side}/camera_info",
                }
            },
            f"/{side}_grasp/grasp_planner_node": {
                "ros__parameters": {
                    "detect_service": f"/{side}_grasp/detect_grasps",
                    "plan_service": f"/{side}_grasp/plan_grasp",
                    "allow_execution": False,
                    "moveit_node_name": f"{side}_grasp_moveit_py",
                    "default_prompt": default_prompt,
                    "default_top_k": default_top_k,
                    "detect_timeout_s": 60.0,
                    "grasp_frame": "world",
                    "arm_base_frame": f"{side}/base_link",
                    "moveit_frame": "base_link",
                    "ee_frame": "gripper_frame_link",
                    "joint_states_topic": f"/{side}/joint_states",
                    "object_cloud_topic": f"/{side}_grasp/so101_grasping/object_cloud",
                    "wrist_object_cloud_topic": f"/{side}_grasp/so101_grasping/wrist_object_cloud",
                    "display_topic": f"/{side}_grasp/so101_grasping/display_planned_path",
                    "planned_markers_topic": f"/{side}_grasp/so101_grasping/planned_path_markers",
                    "gripper_action": f"/{side}/gripper_controller/gripper_cmd",
                    "ready_configuration_name": "",
                    "prefer_low_wrist_roll": True,
                    "preferred_wrist_roll_delta_rad": 0.35,
                    "max_wrist_roll_delta_rad": 0.90,
                    "use_support_plane_staging": True,
                    "support_plane_frame": "world",
                    "support_plane_point_xyz": support_plane_point,
                    "support_plane_normal_xyz": support_plane_normal,
                    "close_surface_clearance_m": 0.003,
                    "require_wrist_cloud_for_execution": True,
                    "wrist_refine_before_grasp": False,
                    "wrist_refine_max_xy_shift_m": 0.08,
                    "wrist_refine_max_z_shift_m": 0.10,
                    "wrist_camera_xyz_in_ee": wrist_translation,
                    "wrist_camera_quat_xyzw_in_ee": wrist_quat,
                }
            },
        }

        grasping_path = config_dir / f"{side}_grasp_runtime.yaml"
        grasping_path.write_text(yaml.safe_dump(grasping, sort_keys=False), encoding="utf-8")
        artifacts.append(grasping_path)

        side_cameras = {
            "cameras": [
                {
                    "name": "cam_wrist",
                    "camera_type": "opencv_compressed",
                    "param_path": str(config_dir / "xlerobot_opencv_cam.yaml"),
                    "namespace": side,
                    "camera_info_url": "file://" + str(cfg[f"{side}_wrist_info"]),
                },
                {
                    "name": "cam_overhead",
                    "camera_type": "opencv_compressed",
                    "param_path": str(config_dir / "xlerobot_opencv_cam.yaml"),
                    "namespace": "center_gopro",
                    "camera_info_url": "file://" + str(cfg["center_gopro_info"]),
                },
            ],
            "omitted_camera_namespace": other_side,
        }
        side_cameras_path = config_dir / f"{side}_grasp_cameras.yaml"
        side_cameras_path.write_text(yaml.safe_dump(side_cameras, sort_keys=False), encoding="utf-8")
        artifacts.append(side_cameras_path)

        controllers_path = config_dir / f"{side}_moveit_controllers.yaml"
        controllers_path.write_text(
            controllers_template.replace("follower/", f"{side}/"),
            encoding="utf-8",
        )
        artifacts.append(controllers_path)

        moveit_py_path = config_dir / f"{side}_moveit_py_config.yaml"
        moveit_py_path.write_text(
            moveit_py_template.replace("/follower/joint_states", f"/{side}/joint_states"),
            encoding="utf-8",
        )
        artifacts.append(moveit_py_path)

    return artifacts


def export_report(state: dict[str, Any]) -> list[Path]:
    return [write_report(state)]


def doctor(workspace: Path) -> list[str]:
    lines = [f"workspace: {workspace}"]
    lines.append(f"git: {shutil.which('git') or 'missing'}")
    lines.append(f"ros2: {shutil.which('ros2') or 'missing'}")
    lines.append(f"ffmpeg: {shutil.which('ffmpeg') or 'missing'}")
    lines.append(f"v4l2-ctl: {shutil.which('v4l2-ctl') or 'missing'}")
    for key in ("gpg.format", "user.signingkey", "commit.gpgsign", "gpg.ssh.allowedSignersFile"):
        value = subprocess.run(
            ["git", "-C", str(workspace), "config", "--show-origin", "--get", key],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        ).stdout.strip()
        lines.append(f"{key}: {value or 'unset'}")
    return lines


def _load_transform_matrix(transform: dict[str, Any]):
    import numpy as np

    matrix = np.eye(4, dtype=float)
    if "rotation_matrix" in transform:
        matrix[:3, :3] = np.asarray(transform["rotation_matrix"], dtype=float)
    else:
        matrix[:3, :3] = _quat_to_matrix(transform["quaternion_xyzw"])
    matrix[:3, 3] = np.asarray(transform["translation_xyz"], dtype=float)
    return matrix


def _invert_transform(matrix):
    import numpy as np

    out = np.eye(4, dtype=float)
    out[:3, :3] = matrix[:3, :3].T
    out[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return out


def _quat_to_matrix(quat):
    import numpy as np

    x, y, z, w = [float(value) for value in quat]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _matrix_to_quat_xyzw(rot) -> list[float]:
    import numpy as np

    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = [(rot[2, 1] - rot[1, 2]) / s, (rot[0, 2] - rot[2, 0]) / s, (rot[1, 0] - rot[0, 1]) / s, 0.25 * s]
    else:
        diag = np.diag(rot)
        if diag[0] > diag[1] and diag[0] > diag[2]:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            quat = [0.25 * s, (rot[0, 1] + rot[1, 0]) / s, (rot[0, 2] + rot[2, 0]) / s, (rot[2, 1] - rot[1, 2]) / s]
        elif diag[1] > diag[2]:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            quat = [(rot[0, 1] + rot[1, 0]) / s, 0.25 * s, (rot[1, 2] + rot[2, 1]) / s, (rot[0, 2] - rot[2, 0]) / s]
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            quat = [(rot[0, 2] + rot[2, 0]) / s, (rot[1, 2] + rot[2, 1]) / s, 0.25 * s, (rot[1, 0] - rot[0, 1]) / s]
    arr = np.asarray(quat, dtype=float)
    arr /= np.linalg.norm(arr)
    return [float(value) for value in arr]
