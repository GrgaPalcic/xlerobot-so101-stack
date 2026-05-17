from pathlib import Path

from xlerobot_calibration.report import write_report
from xlerobot_calibration.runner import generate_controller_configs, generate_world_files, is_missing_config_value
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
