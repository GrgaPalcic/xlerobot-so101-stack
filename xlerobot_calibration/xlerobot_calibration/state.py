from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


RUN_DIR_GLOB = "xlerobot_*"


DEFAULT_CONFIG: dict[str, Any] = {
    "left_port": "/dev/ttyUSB_LEFT_CONFIRMED",
    "right_port": "/dev/ttyUSB_RIGHT_CONFIRMED",
    "left_wrist_dev": "/dev/v4l/by-id/LEFT_ARDUCAM_CONFIRMED",
    "right_wrist_dev": "/dev/v4l/by-id/RIGHT_ARDUCAM_CONFIRMED",
    "center_gopro_dev": "/dev/video42",
    "left_lerobot_json": "",
    "right_lerobot_json": "",
    "left_wrist_info": "",
    "right_wrist_info": "",
    "center_gopro_info": "",
    "left_joint_config": "",
    "right_joint_config": "",
    "intr_cols": 11,
    "intr_rows": 8,
    "intr_square_m": 0.034,
    "intr_marker_m": 0.025,
    "intr_start_id": 2,
    "intr_marker_count": 44,
    "intr_dict": "DICT_5X5_100",
    "world_cols": 7,
    "world_rows": 5,
    "world_square_m": 0.020,
    "world_marker_m": 0.014,
    "world_start_id": 50,
    "world_marker_count": 18,
    "world_dict": "DICT_5X5_100",
    "grasp_server_address": "127.0.0.1:8091",
    "workspace_bounds": "-0.50 0.50 -0.35 0.35 -0.05 0.50",
    "prompt": "pink cube",
    "top_k": 8,
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def make_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def default_out_dir(workspace: Path, run_id: str) -> Path:
    return workspace / "field_runs" / f"xlerobot_{run_id}"


def state_path(out_dir: Path) -> Path:
    return out_dir / "run_state.yaml"


def ensure_run_dirs(out_dir: Path) -> None:
    for name in (
        "logs",
        "config",
        "images",
        "intrinsics",
        "extrinsics",
        "touch",
        "audit",
        "snapshots",
        "bags",
    ):
        (out_dir / name).mkdir(parents=True, exist_ok=True)


def create_state(workspace: Path, run_id: str | None = None, out_dir: Path | None = None) -> dict[str, Any]:
    run_id = run_id or make_run_id()
    out_dir = out_dir or default_out_dir(workspace, run_id)
    ensure_run_dirs(out_dir)
    state = {
        "schema_version": 1,
        "created_at": utc_now(),
        "updated_at": utc_now(),
        "run_id": run_id,
        "workspace": str(workspace.resolve()),
        "out_dir": str(out_dir.resolve()),
        "config": deepcopy(DEFAULT_CONFIG),
        "steps": {},
    }
    save_state(state)
    return state


def load_state(out_dir: Path) -> dict[str, Any]:
    path = state_path(out_dir)
    if not path.exists():
        raise FileNotFoundError(f"run state does not exist: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    data.setdefault("config", {})
    data.setdefault("steps", {})
    return data


def save_state(state: dict[str, Any]) -> None:
    state["updated_at"] = utc_now()
    out_dir = Path(state["out_dir"])
    ensure_run_dirs(out_dir)
    state_path(out_dir).write_text(yaml.safe_dump(state, sort_keys=False), encoding="utf-8")


def find_latest_state(workspace: Path) -> dict[str, Any] | None:
    root = workspace / "field_runs"
    if not root.exists():
        return None
    candidates = sorted(
        (path for path in root.glob(RUN_DIR_GLOB) if state_path(path).exists()),
        key=lambda path: path.name,
    )
    if not candidates:
        return None
    return load_state(candidates[-1])


def context(state: dict[str, Any]) -> dict[str, str]:
    cfg = state.get("config", {})
    out = Path(state["out_dir"])
    world_width = float(cfg.get("world_cols", 0)) * float(cfg.get("world_square_m", 0))
    world_height = float(cfg.get("world_rows", 0)) * float(cfg.get("world_square_m", 0))
    values: dict[str, Any] = {
        "workspace": state["workspace"],
        "ws": state["workspace"],
        "out": state["out_dir"],
        "run_id": state["run_id"],
        "world_width_m": world_width,
        "world_height_m": world_height,
        "left_controller_config": out / "config" / "left_split_controllers.yaml",
        "right_controller_config": out / "config" / "right_split_controllers.yaml",
        "world_board_identity": out / "extrinsics" / "world_board_identity.yaml",
        "world_support_plane": out / "extrinsics" / "world_support_plane.yaml",
        "xlerobot_cameras": out / "config" / "xlerobot_cameras.yaml",
        "xlerobot_camera_params": out / "config" / "xlerobot_opencv_cam.yaml",
    }
    values.update(cfg)
    return {key: str(value) for key, value in values.items()}


def apply_config_overrides(state: dict[str, Any], overrides: list[str]) -> None:
    cfg = state.setdefault("config", {})
    for item in overrides:
        if "=" not in item:
            raise ValueError(f"expected KEY=VALUE override, got {item!r}")
        key, value = item.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"empty config key in override {item!r}")
        cfg[key] = coerce_value(value.strip())


def coerce_value(value: str) -> Any:
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


def step_status(state: dict[str, Any], step_id: str) -> str:
    return str(state.get("steps", {}).get(step_id, {}).get("status", "pending"))


def mark_step(
    state: dict[str, Any],
    step_id: str,
    *,
    status: str,
    log_path: str = "",
    artifacts: list[str] | None = None,
    error: str = "",
    returncode: int | None = None,
) -> None:
    record = state.setdefault("steps", {}).setdefault(step_id, {})
    record.update(
        {
            "status": status,
            "updated_at": utc_now(),
            "log_path": log_path,
            "artifacts": artifacts or record.get("artifacts", []),
            "error": error,
        }
    )
    if returncode is not None:
        record["returncode"] = int(returncode)

