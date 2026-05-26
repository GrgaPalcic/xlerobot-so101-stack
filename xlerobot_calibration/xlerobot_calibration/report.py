from __future__ import annotations

from pathlib import Path
from typing import Any

from .steps import STEPS


def write_report(state: dict[str, Any]) -> Path:
    out_dir = Path(state["out_dir"])
    report_path = out_dir / "xlerobot_calibration_report.md"
    cfg = state.get("config", {})
    lines = [
        "# XLeRobot Calibration Report",
        "",
        f"run_id: {state.get('run_id', '')}",
        f"workspace: {state.get('workspace', '')}",
        f"out_dir: {state.get('out_dir', '')}",
        "",
        "## Key Config",
        "",
    ]
    for key in (
        "left_port",
        "right_port",
        "left_wrist_dev",
        "right_wrist_dev",
        "center_gopro_dev",
        "left_lerobot_json",
        "right_lerobot_json",
        "left_wrist_info",
        "right_wrist_info",
        "center_gopro_info",
        "left_joint_config",
        "right_joint_config",
    ):
        lines.append(f"- {key}: {cfg.get(key, '')}")

    lines.extend(["", "## Step Status", ""])
    step_records = state.get("steps", {})
    for step in STEPS:
        record = step_records.get(step.id, {})
        status = record.get("status", "pending")
        lines.append(f"- {status}: {step.id} - {step.title}")
        if record.get("log_path"):
            lines.append(f"  log: {record['log_path']}")
        for artifact in record.get("artifacts", []) or []:
            lines.append(f"  artifact: {artifact}")
        if record.get("error"):
            lines.append(f"  error: {record['error']}")

    lines.extend(
        [
            "",
            "## Promotion Checklist",
            "",
            "- [ ] Left/right LeRobot calibration JSONs are fresh and archived.",
            "- [ ] ROS joint state audit agrees with physical joint identity.",
            "- [ ] Vision hand-eye residuals are acceptable for both wrist cameras.",
            "- [ ] Camera intrinsics and extrinsics pass reprojection checks.",
            "- [ ] TF tree is complete in fixed frame `world`.",
            "- [ ] Perception dry run produces object clouds and grasp markers in `world`.",
            "- [ ] No generated calibration files have been promoted into source config prematurely.",
            "",
        ]
    )
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return report_path
