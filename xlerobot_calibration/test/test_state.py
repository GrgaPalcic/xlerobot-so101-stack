from argparse import Namespace
import json
from pathlib import Path

import yaml

from xlerobot_calibration.board_presets import apply_board_preset, preset_names
from xlerobot_calibration.cli import resolve_existing_state
from xlerobot_calibration.report import write_report
from xlerobot_calibration.runner import (
    build_touch_jog_commands,
    generate_controller_configs,
    generate_joint_configs,
    generate_world_files,
    is_missing_config_value,
)
from xlerobot_calibration.state import create_state, load_state, mark_step


def test_state_round_trip(tmp_path: Path):
    workspace = tmp_path / "ws"
    workspace.mkdir()
    state = create_state(workspace, run_id="20260517T000000Z")
    loaded = load_state(Path(state["out_dir"]))
    assert loaded["run_id"] == "20260517T000000Z"
    assert loaded["config"]["center_gopro_dev"] == "/dev/video42"


def test_generate_world_files(tmp_path: Path):
    workspace = tmp_path / "ws"
    workspace.mkdir()
    state = create_state(workspace, run_id="20260517T000000Z")
    artifacts = generate_world_files(state)
    assert {path.name for path in artifacts} == {"world_board_identity.yaml", "world_support_plane.yaml"}
    assert all(path.exists() for path in artifacts)


def test_generate_controller_configs(tmp_path: Path):
    workspace = tmp_path / "ws"
    cfg_dir = workspace / "so101_bringup/config/ros2_control"
    cfg_dir.mkdir(parents=True)
    (cfg_dir / "follower_split_controllers.yaml").write_text("follower:\n  controller_manager: {}\n")
    state = create_state(workspace, run_id="20260517T000000Z")
    artifacts = generate_controller_configs(state)
    assert (Path(state["out_dir"]) / "config/left_split_controllers.yaml").read_text().startswith("left:")
    assert (Path(state["out_dir"]) / "config/right_split_controllers.yaml").read_text().startswith("right:")
    assert len(artifacts) == 2


def test_report_includes_step_status(tmp_path: Path):
    workspace = tmp_path / "ws"
    workspace.mkdir()
    state = create_state(workspace, run_id="20260517T000000Z")
    mark_step(state, "git_snapshot", status="complete")
    report = write_report(state)
    text = report.read_text()
    assert "XLeRobot Calibration Report" in text
    assert "complete: git_snapshot" in text


def test_placeholder_config_values_are_missing():
    assert is_missing_config_value("/dev/ttyUSB_LEFT_CONFIRMED")
    assert is_missing_config_value("/path/to/xlerobot_left.json")
    assert not is_missing_config_value("/dev/ttyUSB0")


def test_board_presets_update_intrinsics_and_world_config(tmp_path: Path):
    workspace = tmp_path / "ws"
    workspace.mkdir()
    state = create_state(workspace, run_id="20260517T000000Z")

    assert "intrinsics_a3_11x8_34_25_id2" in preset_names()
    apply_board_preset(state, "intrinsics_a3_11x8_34_25_id2")
    apply_board_preset(state, "world_plate_c_7x5_20_14_id83")

    assert state["config"]["intr_start_id"] == 2
    assert state["config"]["intr_square_m"] == 0.034
    assert state["config"]["world_start_id"] == 83
    assert state["config"]["world_marker_m"] == 0.014


def test_touch_jog_commands_use_state_config(tmp_path: Path):
    workspace = tmp_path / "ws"
    workspace.mkdir()
    cfg_dir = workspace / "so101_bringup/config/ros2_control"
    cfg_dir.mkdir(parents=True)
    (cfg_dir / "follower_split_controllers.yaml").write_text("follower:\n  controller_manager: {}\n")
    left_json = tmp_path / "left.json"
    right_json = tmp_path / "right.json"
    payload = {
        name: {
            "id": idx,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for idx, name in enumerate(
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
            start=1,
        )
    }
    left_json.write_text(json.dumps(payload))
    right_json.write_text(json.dumps(payload))
    state = create_state(workspace, run_id="20260517T000000Z")
    state["config"]["left_port"] = "/dev/serial/by-path/left"
    state["config"]["left_lerobot_json"] = str(left_json)
    state["config"]["right_lerobot_json"] = str(right_json)
    generate_controller_configs(state)
    generate_joint_configs(state)
    generate_world_files(state)

    bringup, motion, recorder = build_touch_jog_commands(state, "left")

    assert "usb_port:=/dev/serial/by-path/left" in bringup
    assert "namespace:=left" in bringup
    assert "arm:=left" in motion
    assert "--jog-service" in recorder
    assert "/left/go_to_pose" in recorder
    assert "--jog-strategy" in recorder
    assert "cartesian" in recorder
    assert "--max-jog-step-m" in recorder
    assert "0.1" in recorder
    assert "--joint-jog-topic" in recorder
    assert "/left/arm_forward_controller/commands" in recorder
    assert "--command-speed" in recorder
    assert "2400" in recorder
    assert "--command-acceleration" in recorder
    assert "50" in recorder
    assert "--jog-web-port" in recorder
    assert "8780" in recorder
    assert str(Path(state["out_dir"]) / "touch/left_base_to_world_board.yaml") in recorder


def test_touch_jog_tuning_writes_overlay_joint_config(tmp_path: Path):
    workspace = tmp_path / "ws"
    workspace.mkdir()
    cfg_dir = workspace / "so101_bringup/config/ros2_control"
    cfg_dir.mkdir(parents=True)
    (cfg_dir / "follower_split_controllers.yaml").write_text("follower:\n  controller_manager: {}\n")
    payload = {
        name: {
            "id": idx,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for idx, name in enumerate(
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
            start=1,
        )
    }
    left_json = tmp_path / "left.json"
    right_json = tmp_path / "right.json"
    left_json.write_text(json.dumps(payload))
    right_json.write_text(json.dumps(payload))
    state = create_state(workspace, run_id="20260517T000000Z")
    state["config"]["left_port"] = "/dev/serial/by-path/left"
    state["config"]["left_lerobot_json"] = str(left_json)
    state["config"]["right_lerobot_json"] = str(right_json)
    generate_controller_configs(state)
    generate_joint_configs(state)
    generate_world_files(state)

    bringup, _, _ = build_touch_jog_commands(
        state,
        "left",
        command_speed=900,
        command_acceleration=20,
        arm_protection_current=450,
    )

    overlay = Path(next(item.split(":=", 1)[1] for item in bringup if item.startswith("joint_config_file:=")))
    assert overlay.name == "left_touch_jog_joints.yaml"
    data = yaml.safe_load(overlay.read_text())
    for name in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]:
        assert data["joints"][name]["command_speed"] == 900
        assert data["joints"][name]["command_acceleration"] == 20
        assert data["joints"][name]["protection_current"] == 450
    assert "command_speed" not in data["joints"]["gripper"]


def test_workspace_argument_overrides_stale_run_state_workspace(tmp_path: Path):
    old_workspace = tmp_path / "old_ws"
    new_workspace = tmp_path / "new_ws"
    old_workspace.mkdir()
    new_workspace.mkdir()
    state = create_state(old_workspace, run_id="20260517T000000Z")

    loaded = resolve_existing_state(Namespace(out=state["out_dir"]), new_workspace)

    assert loaded["workspace"] == str(new_workspace.resolve())
