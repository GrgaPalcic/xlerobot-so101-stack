#!/usr/bin/env python3
"""Solve wrist robot-world/hand-eye calibration from captured board samples."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


def make_transform(rot: np.ndarray, translation: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = np.asarray(rot, dtype=np.float64).reshape(3, 3)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def invert_transform(tf: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = tf[:3, :3].T
    out[:3, 3] = -tf[:3, :3].T @ tf[:3, 3]
    return out


def quat_to_matrix(quat: list[float]) -> np.ndarray:
    x, y, z, w = [float(value) for value in quat]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat_xyzw(rot: np.ndarray) -> list[float]:
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = [
            (rot[2, 1] - rot[1, 2]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
            (rot[1, 0] - rot[0, 1]) / s,
            0.25 * s,
        ]
    else:
        diag = np.diag(rot)
        if diag[0] > diag[1] and diag[0] > diag[2]:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            quat = [
                0.25 * s,
                (rot[0, 1] + rot[1, 0]) / s,
                (rot[0, 2] + rot[2, 0]) / s,
                (rot[2, 1] - rot[1, 2]) / s,
            ]
        elif diag[1] > diag[2]:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            quat = [
                (rot[0, 1] + rot[1, 0]) / s,
                0.25 * s,
                (rot[1, 2] + rot[2, 1]) / s,
                (rot[0, 2] - rot[2, 0]) / s,
            ]
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            quat = [
                (rot[0, 2] + rot[2, 0]) / s,
                (rot[1, 2] + rot[2, 1]) / s,
                0.25 * s,
                (rot[1, 0] - rot[0, 1]) / s,
            ]
    arr = np.asarray(quat, dtype=np.float64)
    arr /= np.linalg.norm(arr)
    return [float(value) for value in arr]


def transform_from_record(data: dict[str, Any]) -> np.ndarray:
    if "matrix" in data:
        return np.asarray(data["matrix"], dtype=np.float64).reshape(4, 4)
    if "rotation_matrix" in data:
        rot = np.asarray(data["rotation_matrix"], dtype=np.float64).reshape(3, 3)
    else:
        rot = quat_to_matrix(data["quaternion_xyzw"])
    return make_transform(rot, np.asarray(data["translation_xyz"], dtype=np.float64))


def transform_to_record(name: str, parent_frame: str, child_frame: str, tf: np.ndarray) -> dict[str, Any]:
    return {
        "name": name,
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation_xyz": tf[:3, 3].tolist(),
        "rotation_matrix": tf[:3, :3].tolist(),
        "quaternion_xyzw": matrix_to_quat_xyzw(tf[:3, :3]),
    }


def to_plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return to_plain(value.tolist())
    if isinstance(value, dict):
        return {str(key): to_plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_plain(item) for item in value]
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def load_samples(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSON sample") from exc
    return rows


def filter_samples(
    samples: list[dict[str, Any]],
    *,
    min_markers: int,
    max_reproj_px: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    kept = []
    rejected = []
    for sample in samples:
        quality = sample.get("quality", {})
        markers = int(quality.get("detected_markers", 0))
        error = float(quality.get("mean_reprojection_error_px", math.inf))
        reason = ""
        if markers < min_markers:
            reason = f"markers {markers} < {min_markers}"
        elif error > max_reproj_px:
            reason = f"mean reprojection {error:.3f}px > {max_reproj_px:.3f}px"
        if reason:
            item = dict(sample)
            item["reject_reason"] = reason
            rejected.append(item)
        else:
            kept.append(sample)
    return kept, rejected


def _method_value(name: str) -> int:
    normalized = name.strip().upper()
    if normalized in {"SHAH", "CALIB_ROBOT_WORLD_HAND_EYE_SHAH"}:
        return int(cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
    if normalized in {"LI", "CALIB_ROBOT_WORLD_HAND_EYE_LI"}:
        return int(cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI)
    raise ValueError(f"unknown robot-world/hand-eye method: {name}")


def solve_handeye(
    samples: list[dict[str, Any]],
    *,
    method: str = "SHAH",
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]]]:
    if not hasattr(cv2, "calibrateRobotWorldHandEye"):
        raise RuntimeError("this OpenCV build does not provide calibrateRobotWorldHandEye")

    r_world2cam = []
    t_world2cam = []
    r_base2gripper = []
    t_base2gripper = []
    parsed = []
    for sample in samples:
        try:
            t_camera_world = transform_from_record(sample["T_camera_world"])
            t_gripper_base = transform_from_record(sample["T_gripper_base"])
        except KeyError as exc:
            raise ValueError(f"sample {sample.get('name', '<unnamed>')} is missing {exc}") from exc
        r_world2cam.append(t_camera_world[:3, :3])
        t_world2cam.append(t_camera_world[:3, 3].reshape(3, 1))
        r_base2gripper.append(t_gripper_base[:3, :3])
        t_base2gripper.append(t_gripper_base[:3, 3].reshape(3, 1))
        parsed.append(
            {
                "sample": sample,
                "T_camera_world": t_camera_world,
                "T_gripper_base": t_gripper_base,
            }
        )

    result = cv2.calibrateRobotWorldHandEye(
        r_world2cam,
        t_world2cam,
        r_base2gripper,
        t_base2gripper,
        method=_method_value(method),
    )
    r_base2world, t_base2world, r_gripper2cam, t_gripper2cam = result
    t_world_base = make_transform(r_base2world, t_base2world)
    t_camera_gripper = make_transform(r_gripper2cam, t_gripper2cam)
    return t_world_base, t_camera_gripper, parsed


def residuals_for_solution(
    parsed_samples: list[dict[str, Any]],
    t_world_base: np.ndarray,
    t_camera_gripper: np.ndarray,
) -> list[dict[str, Any]]:
    rows = []
    t_base_world = invert_transform(t_world_base)
    for item in parsed_samples:
        sample = item["sample"]
        expected_camera_world = t_camera_gripper @ item["T_gripper_base"] @ t_base_world
        measured_camera_world = item["T_camera_world"]
        delta = expected_camera_world @ invert_transform(measured_camera_world)
        rot_error = rotation_angle_deg(delta[:3, :3])
        trans_error = float(np.linalg.norm(delta[:3, 3]))
        rows.append(
            {
                "name": sample.get("name", ""),
                "translation_error_m": trans_error,
                "rotation_error_deg": rot_error,
                "mean_reprojection_error_px": sample.get("quality", {}).get("mean_reprojection_error_px"),
                "detected_markers": sample.get("quality", {}).get("detected_markers"),
            }
        )
    return rows


def rotation_angle_deg(rot: np.ndarray) -> float:
    value = float((np.trace(rot) - 1.0) * 0.5)
    value = min(1.0, max(-1.0, value))
    return math.degrees(math.acos(value))


def summarize_residuals(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {}
    trans = np.asarray([row["translation_error_m"] for row in rows], dtype=np.float64)
    rot = np.asarray([row["rotation_error_deg"] for row in rows], dtype=np.float64)
    return {
        "translation_error_mean_m": float(np.mean(trans)),
        "translation_error_max_m": float(np.max(trans)),
        "rotation_error_mean_deg": float(np.mean(rot)),
        "rotation_error_max_deg": float(np.max(rot)),
    }


def write_static_tf_scripts(output_dir: Path) -> list[Path]:
    logs_dir = output_dir.parent / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    world_base_commands = []
    wrist_camera_commands = []

    for path in sorted(output_dir.glob("world_to_*_base.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        command = static_tf_command(data["transform"])
        world_base_commands.append(command)

    for path in sorted(output_dir.glob("*_wrist_camera_in_gripper.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        command = static_tf_command(data["transform"])
        wrist_camera_commands.append(command)

    scripts = []
    for name, commands in (
        ("static_tf_world_bases.sh", world_base_commands),
        ("static_tf_wrist_cameras.sh", wrist_camera_commands),
    ):
        path = logs_dir / name
        text = "#!/usr/bin/env bash\nset -euo pipefail\n" + "\n".join(commands) + ("\n" if commands else "")
        path.write_text(text, encoding="utf-8")
        path.chmod(0o755)
        scripts.append(path)
    return scripts


def static_tf_command(transform: dict[str, Any]) -> str:
    t = [float(value) for value in transform["translation_xyz"]]
    q = [float(value) for value in transform["quaternion_xyzw"]]
    return (
        "ros2 run tf2_ros static_transform_publisher "
        f"--x {t[0]:.9f} --y {t[1]:.9f} --z {t[2]:.9f} "
        f"--qx {q[0]:.9f} --qy {q[1]:.9f} --qz {q[2]:.9f} --qw {q[3]:.9f} "
        f"--frame-id {transform['parent_frame']} --child-frame-id {transform['child_frame']}"
    )


def update_aggregate_summary(output_dir: Path) -> Path:
    aggregate = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "sides": {},
    }
    for path in sorted(output_dir.glob("*_vision_handeye_summary.json")):
        if path.name == "vision_handeye_summary.json":
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        side = str(data.get("side") or path.name.split("_", 1)[0])
        aggregate["sides"][side] = data
    target = output_dir / "vision_handeye_summary.json"
    target.write_text(json.dumps(to_plain(aggregate), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return target


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--min-samples", type=int, default=15)
    parser.add_argument("--min-markers", type=int, default=8)
    parser.add_argument("--max-reproj-px", type=float, default=2.5)
    parser.add_argument("--method", choices=("SHAH", "LI"), default="SHAH")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--base-frame", default="")
    parser.add_argument("--gripper-frame", default="")
    parser.add_argument("--camera-frame", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_frame = args.base_frame or f"{args.side}/base_link"
    gripper_frame = args.gripper_frame or f"{args.side}/gripper_frame_link"
    camera_frame = args.camera_frame or f"{args.side}/wrist_camera_optical_frame"

    all_samples = load_samples(args.samples)
    kept, rejected = filter_samples(
        all_samples,
        min_markers=args.min_markers,
        max_reproj_px=args.max_reproj_px,
    )
    if len(kept) < args.min_samples:
        raise RuntimeError(
            f"Need at least {args.min_samples} usable samples after filtering, "
            f"kept {len(kept)} of {len(all_samples)}"
        )

    t_world_base, t_camera_gripper, parsed = solve_handeye(kept, method=args.method)
    t_gripper_camera = invert_transform(t_camera_gripper)
    residuals = residuals_for_solution(parsed, t_world_base, t_camera_gripper)
    residual_summary = summarize_residuals(residuals)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    world_base_path = args.output_dir / f"world_to_{args.side}_base.yaml"
    wrist_path = args.output_dir / f"{args.side}_wrist_camera_in_gripper.yaml"
    summary_path = args.output_dir / f"{args.side}_vision_handeye_summary.json"

    world_base = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "transform": transform_to_record(
            f"T_{args.world_frame}_{args.side}_base",
            args.world_frame,
            base_frame,
            t_world_base,
        ),
        "source": str(args.samples),
        "method": f"cv2.calibrateRobotWorldHandEye/{args.method}",
    }
    wrist = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "transform": transform_to_record(
            f"T_{args.side}_gripper_wrist_camera",
            gripper_frame,
            camera_frame,
            t_gripper_camera,
        ),
        "inverse_transform": transform_to_record(
            f"T_{args.side}_wrist_camera_gripper",
            camera_frame,
            gripper_frame,
            t_camera_gripper,
        ),
        "source": str(args.samples),
        "method": f"cv2.calibrateRobotWorldHandEye/{args.method}",
    }
    summary = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "side": args.side,
        "samples_path": str(args.samples),
        "method": f"cv2.calibrateRobotWorldHandEye/{args.method}",
        "frames": {
            "world": args.world_frame,
            "base": base_frame,
            "gripper": gripper_frame,
            "camera": camera_frame,
        },
        "thresholds": {
            "min_samples": args.min_samples,
            "min_markers": args.min_markers,
            "max_reproj_px": args.max_reproj_px,
        },
        "counts": {
            "total_samples": len(all_samples),
            "used_samples": len(kept),
            "rejected_samples": len(rejected),
        },
        "used_sample_names": [sample.get("name", "") for sample in kept],
        "rejected_samples": [
            {
                "name": sample.get("name", ""),
                "reason": sample.get("reject_reason", ""),
                "quality": sample.get("quality", {}),
            }
            for sample in rejected
        ],
        "residual_summary": residual_summary,
        "residuals": residuals,
        "outputs": {
            "world_to_base": str(world_base_path),
            "wrist_camera_in_gripper": str(wrist_path),
        },
    }

    world_base_path.write_text(yaml.safe_dump(to_plain(world_base), sort_keys=False), encoding="utf-8")
    wrist_path.write_text(yaml.safe_dump(to_plain(wrist), sort_keys=False), encoding="utf-8")
    summary_path.write_text(json.dumps(to_plain(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    static_scripts = write_static_tf_scripts(args.output_dir)
    aggregate_summary = update_aggregate_summary(args.output_dir)

    print(f"{args.side}: used {len(kept)}/{len(all_samples)} samples")
    print(
        "  residual translation mean/max: "
        f"{residual_summary.get('translation_error_mean_m', 0.0) * 1000.0:.2f}/"
        f"{residual_summary.get('translation_error_max_m', 0.0) * 1000.0:.2f} mm"
    )
    print(
        "  residual rotation mean/max: "
        f"{residual_summary.get('rotation_error_mean_deg', 0.0):.3f}/"
        f"{residual_summary.get('rotation_error_max_deg', 0.0):.3f} deg"
    )
    print(f"  saved {world_base_path}")
    print(f"  saved {wrist_path}")
    print(f"  saved {summary_path}")
    for path in static_scripts:
        print(f"  updated {path}")
    print(f"  updated {aggregate_summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
