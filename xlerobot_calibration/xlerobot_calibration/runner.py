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
        run_action_step(state, step, dry_run=dry_run)
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
    jog_duration_sec: float = 1.5,
    jog_strategy: str = "cartesian",
    joint_step_rad: float = 0.035,
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

    bringup_cmd = [
        "ros2",
        "launch",
        "so101_bringup",
        "follower_split.launch.py",
        f"namespace:={side}",
        f"frame_prefix:={side}/",
        "hardware_type:=real",
        f"usb_port:={cfg[f'{side}_port']}",
        f"joint_config_file:={cfg[f'{side}_joint_config']}",
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
        "--output",
        str(touch_output),
    ]
    if web_port > 0:
        recorder_cmd.extend(["--jog-web-port", str(web_port)])
    return bringup_cmd, motion_cmd, recorder_cmd


def run_touch_jog(
    state: dict[str, Any],
    side: str,
    *,
    tool_offset: str = "0 0 0",
    corner_layout: str = "tl_tr_bl_br",
    samples: int = 11,
    jog_step_m: float = 0.002,
    jog_duration_sec: float = 1.5,
    jog_strategy: str = "cartesian",
    joint_step_rad: float = 0.035,
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

    bringup_cmd, motion_cmd, recorder_cmd = build_touch_jog_commands(
        state,
        side,
        tool_offset=tool_offset,
        corner_layout=corner_layout,
        samples=samples,
        jog_step_m=jog_step_m,
        jog_duration_sec=jog_duration_sec,
        jog_strategy=jog_strategy,
        joint_step_rad=joint_step_rad,
        web_port=web_port,
    )

    print("")
    print(f"# Touch jog: {side}")
    print("This starts a real command controller and sends small GoToPose moves.")
    print("Any existing same-side state-only/jog launch will be stopped first.")
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


def run_action_step(state: dict[str, Any], step: Step, *, dry_run: bool) -> None:
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
