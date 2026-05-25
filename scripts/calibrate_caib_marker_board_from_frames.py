#!/usr/bin/env python3
"""Calibrate intrinsics from frames of a caib.io ChArUco board.

caib.io's ChArUco generator can place ArUco markers on the opposite checker
parity from OpenCV's default CharucoBoard_create layout. This script calibrates
from detected marker corners directly, using the physical caib.io marker layout.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

ROBUST_SIGMA_FLOOR_PX = 0.05
COVERAGE_FIELDS = ("x", "y", "size", "skew")


@dataclass(frozen=True)
class DetectionRecord:
    name: str
    corners: list[np.ndarray]
    ids: np.ndarray
    marker_count: int
    x: float
    y: float
    size: float
    skew: float


@dataclass(frozen=True)
class CalibrationSolve:
    rms: float
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    rvecs: tuple[np.ndarray, ...]
    tvecs: tuple[np.ndarray, ...]
    view_errors_px: dict[str, float]


@dataclass(frozen=True)
class SelectionResult:
    threshold_px: float
    median_px: float
    robust_sigma_px: float
    selected_names: list[str]
    selected_reasons: dict[str, str]
    rejected_reasons: dict[str, str]
    fallback_min_frames: bool
    capped_max_frames: bool
    input_frame_count: int
    residual_candidate_count: int
    effective_min_selected_frames: int
    effective_max_selected_frames: int


def _dictionary(name: str):
    attr = name if name.startswith("DICT_") else f"DICT_{name}"
    return aruco.getPredefinedDictionary(getattr(aruco, attr))


def _make_caib_board(cols: int, rows: int, square_m: float, marker_m: float, start_id: int, dictionary):
    obj_points = []
    ids = []
    marker_id = start_id
    margin = 0.5 * (square_m - marker_m)
    for row in range(rows):
        marker_cols = range(0, cols, 2) if row % 2 == 0 else range(1, cols, 2)
        for col in marker_cols:
            x = col * square_m + margin
            y = row * square_m + margin
            obj_points.append(
                np.asarray(
                    [
                        [x, y, 0.0],
                        [x + marker_m, y, 0.0],
                        [x + marker_m, y + marker_m, 0.0],
                        [x, y + marker_m, 0.0],
                    ],
                    dtype=np.float32,
                )
            )
            ids.append(marker_id)
            marker_id += 1
    board = aruco.Board_create(obj_points, dictionary, np.asarray(ids, dtype=np.int32))
    return board, {marker_id: obj for marker_id, obj in zip(ids, obj_points)}


def _coverage_params(corners: list[np.ndarray], image_size: tuple[int, int]) -> tuple[float, float, float, float]:
    points = np.concatenate([corner.reshape(-1, 2) for corner in corners], axis=0).astype(np.float64)
    image_w, image_h = image_size
    min_xy = np.min(points, axis=0)
    max_xy = np.max(points, axis=0)
    center = 0.5 * (min_xy + max_xy)
    width = max(float(max_xy[0] - min_xy[0]), 0.0) / max(float(image_w), 1.0)
    height = max(float(max_xy[1] - min_xy[1]), 0.0) / max(float(image_h), 1.0)
    size = float(np.sqrt(width * height))
    x = float(center[0] / max(float(image_w), 1.0))
    y = float(center[1] / max(float(image_h), 1.0))

    if len(points) >= 2:
        cov = np.cov(points.T)
        denom = float(np.sqrt(max(cov[0, 0], 0.0) * max(cov[1, 1], 0.0)))
        skew = float(abs(cov[0, 1]) / denom) if denom > 1e-9 else 0.0
    else:
        skew = 0.0
    return x, y, size, skew


def _detect_frames(args, dictionary):
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    valid_ids = set(range(args.start_id, args.start_id + args.marker_count))
    records = []
    image_size = None
    for frame_path in sorted(Path(args.frames_dir).glob(args.glob)):
        image = cv2.imread(str(frame_path))
        if image is None:
            continue
        image_size = (image.shape[1], image.shape[0])
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is None:
            continue
        kept_corners = []
        kept_ids = []
        for corner, marker_id in zip(corners, ids.ravel()):
            marker_id = int(marker_id)
            if marker_id in valid_ids:
                kept_corners.append(corner.astype(np.float32))
                kept_ids.append([marker_id])
        if len(kept_corners) >= args.min_markers:
            x, y, size, skew = _coverage_params(kept_corners, image_size)
            records.append(
                DetectionRecord(
                    name=frame_path.name,
                    corners=kept_corners,
                    ids=np.asarray(kept_ids, dtype=np.int32),
                    marker_count=len(kept_corners),
                    x=x,
                    y=y,
                    size=size,
                    skew=skew,
                )
            )
    if image_size is None:
        raise RuntimeError(f"No readable frames found in {args.frames_dir}")
    if len(records) < args.min_frames:
        raise RuntimeError(f"Need at least {args.min_frames} usable frames, found {len(records)}")
    return image_size, records


def _flatten(records: list[DetectionRecord]):
    all_corners = []
    all_ids = []
    counter = []
    for record in records:
        all_corners.extend(record.corners)
        all_ids.extend(record.ids.reshape(-1, 1))
        counter.append(len(record.corners))
    return all_corners, np.asarray(all_ids, dtype=np.int32), np.asarray(counter, dtype=np.int32)


def _view_errors(records, obj_by_id, camera_matrix, dist_coeffs, rvecs, tvecs):
    errors = {}
    for record, rvec, tvec in zip(records, rvecs, tvecs):
        object_points = []
        image_points = []
        for corner, marker_id in zip(record.corners, record.ids.ravel()):
            object_points.append(obj_by_id[int(marker_id)])
            image_points.append(corner.reshape(4, 2))
        object_points = np.concatenate(object_points, axis=0).astype(np.float32)
        image_points = np.concatenate(image_points, axis=0).astype(np.float32)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        diff = image_points - projected.reshape(-1, 2)
        errors[record.name] = float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))
    return errors


def _calibration_solve(image_size, records, board, obj_by_id, flags: int) -> CalibrationSolve:
    corners, ids, counter = _flatten(records)
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraAruco(
        corners, ids, counter, board, image_size, None, None, flags=flags
    )
    rvecs = tuple(np.asarray(rvec) for rvec in rvecs)
    tvecs = tuple(np.asarray(tvec) for tvec in tvecs)
    errors = _view_errors(records, obj_by_id, camera_matrix, dist_coeffs, rvecs, tvecs)
    return CalibrationSolve(
        rms=float(rms),
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        dist_coeffs=np.asarray(dist_coeffs, dtype=np.float64),
        rvecs=rvecs,
        tvecs=tvecs,
        view_errors_px=errors,
    )


def _robust_threshold(errors: list[float], max_view_error_px: float, mad_multiplier: float):
    values = np.asarray(errors, dtype=np.float64)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_sigma = max(1.4826 * mad, ROBUST_SIGMA_FLOOR_PX)
    threshold = min(float(max_view_error_px), median + float(mad_multiplier) * robust_sigma)
    return threshold, median, robust_sigma


def _normalized_coverage(records: list[DetectionRecord], indices: list[int]) -> dict[int, np.ndarray]:
    matrix = np.asarray(
        [[getattr(records[index], field) for field in COVERAGE_FIELDS] for index in indices],
        dtype=np.float64,
    )
    mins = np.min(matrix, axis=0)
    spans = np.max(matrix, axis=0) - mins
    spans[spans < 1e-9] = 1.0
    normalized = (matrix - mins) / spans
    return {index: normalized[row] for row, index in enumerate(indices)}


def _add_reason(reasons: dict[int, str], index: int, reason: str) -> None:
    if index in reasons:
        parts = reasons[index].split(",")
        if reason not in parts:
            reasons[index] = f"{reasons[index]},{reason}"
    else:
        reasons[index] = reason


def _coverage_preserving_subset(
    records: list[DetectionRecord],
    candidate_indices: list[int],
    errors_by_name: dict[str, float],
    max_count: int,
) -> tuple[set[int], dict[int, str]]:
    if len(candidate_indices) <= max_count:
        return set(candidate_indices), {index: "below_residual_threshold" for index in candidate_indices}

    selected: set[int] = set()
    selected_reasons: dict[int, str] = {}
    normalized = _normalized_coverage(records, candidate_indices)

    def add(index: int, reason: str) -> None:
        if len(selected) >= max_count and index not in selected:
            return
        selected.add(index)
        _add_reason(selected_reasons, index, reason)

    for col, field in enumerate(COVERAGE_FIELDS):
        min_index = min(
            candidate_indices,
            key=lambda index: (normalized[index][col], errors_by_name[records[index].name]),
        )
        max_index = max(
            candidate_indices,
            key=lambda index: (normalized[index][col], -errors_by_name[records[index].name]),
        )
        add(min_index, f"coverage_extreme:{field}_min")
        add(max_index, f"coverage_extreme:{field}_max")
        if len(selected) >= max_count:
            break

    if not selected:
        lowest_error = min(candidate_indices, key=lambda index: errors_by_name[records[index].name])
        add(lowest_error, "lowest_error_seed")

    while len(selected) < max_count:
        remaining = [index for index in candidate_indices if index not in selected]
        if not remaining:
            break

        def novelty(index: int) -> tuple[float, float, int]:
            min_distance = min(float(np.linalg.norm(normalized[index] - normalized[other])) for other in selected)
            return min_distance, -errors_by_name[records[index].name], records[index].marker_count

        add(max(remaining, key=novelty), "pose_novelty")

    return selected, selected_reasons


def _select_frame_subset(
    records: list[DetectionRecord],
    errors_by_name: dict[str, float],
    *,
    max_view_error_px: float,
    outlier_mad_multiplier: float,
    min_selected_frames: int,
    max_selected_frames: int,
) -> SelectionResult:
    if not records:
        raise ValueError("No frame records supplied for selection")

    errors = [errors_by_name[record.name] for record in records]
    threshold, median, robust_sigma = _robust_threshold(errors, max_view_error_px, outlier_mad_multiplier)
    effective_min = min(max(int(min_selected_frames), 1), len(records))
    effective_max = min(max(int(max_selected_frames), effective_min), len(records))

    candidate_indices = [index for index, record in enumerate(records) if errors_by_name[record.name] <= threshold]
    selected_reasons_by_index: dict[int, str] = {}
    fallback_min_frames = False
    capped_max_frames = False

    if len(candidate_indices) < effective_min:
        fallback_min_frames = True
        selected_indices = set(
            sorted(
                range(len(records)),
                key=lambda index: (errors_by_name[records[index].name], -records[index].marker_count),
            )[:effective_min]
        )
        selected_reasons_by_index = {index: "minimum_frame_fallback" for index in selected_indices}
    else:
        selected_indices = set(candidate_indices)
        selected_reasons_by_index = {index: "below_residual_threshold" for index in selected_indices}

    if len(selected_indices) > effective_max:
        capped_max_frames = True
        selected_indices, selected_reasons_by_index = _coverage_preserving_subset(
            records,
            [index for index in range(len(records)) if index in selected_indices],
            errors_by_name,
            effective_max,
        )

    selected_names = [record.name for index, record in enumerate(records) if index in selected_indices]
    selected_reasons = {
        records[index].name: selected_reasons_by_index.get(index, "selected") for index in selected_indices
    }
    rejected_reasons = {}
    for index, record in enumerate(records):
        if index in selected_indices:
            continue
        error = errors_by_name[record.name]
        if error > threshold:
            rejected_reasons[record.name] = f"view_error_px {error:.3f} > threshold_px {threshold:.3f}"
        else:
            rejected_reasons[record.name] = "over_max_selected_frames"

    return SelectionResult(
        threshold_px=float(threshold),
        median_px=float(median),
        robust_sigma_px=float(robust_sigma),
        selected_names=selected_names,
        selected_reasons=selected_reasons,
        rejected_reasons=rejected_reasons,
        fallback_min_frames=fallback_min_frames,
        capped_max_frames=capped_max_frames,
        input_frame_count=len(records),
        residual_candidate_count=len(candidate_indices),
        effective_min_selected_frames=effective_min,
        effective_max_selected_frames=effective_max,
    )


def _save_yaml(path: Path, args, image_size, camera_matrix, dist_coeffs, distortion_model: str):
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).ravel()
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, args.rectify_alpha)
    rectification = np.eye(3, dtype=np.float64)
    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:3, :3] = new_camera_matrix
    info = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_name": args.camera_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": camera_matrix.flatten().tolist()},
        "distortion_model": distortion_model,
        "distortion_coefficients": {"rows": 1, "cols": int(len(dist_coeffs)), "data": dist_coeffs.tolist()},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": rectification.flatten().tolist()},
        "projection_matrix": {"rows": 3, "cols": 4, "data": projection.flatten().tolist()},
    }
    with path.open("w") as stream:
        yaml.safe_dump(info, stream, sort_keys=False)


def _solve_summary(solve: CalibrationSolve, records: list[DetectionRecord], yaml_path: Path) -> dict:
    errors = [solve.view_errors_px[record.name] for record in records]
    return {
        "frame_count": len(records),
        "rms": float(solve.rms),
        "median_view_error_px": float(np.median(errors)),
        "worst_view_error_px": float(np.max(errors)),
        "camera_matrix": solve.camera_matrix.tolist(),
        "distortion": solve.dist_coeffs.ravel().tolist(),
        "yaml": str(yaml_path),
        "view_errors_px": {record.name: float(solve.view_errors_px[record.name]) for record in records},
    }


def _selection_summary(selection: SelectionResult, pass_index: int) -> dict:
    return {
        "pass": pass_index,
        "input_frame_count": selection.input_frame_count,
        "residual_candidate_count": selection.residual_candidate_count,
        "selected_frame_count": len(selection.selected_names),
        "threshold_px": selection.threshold_px,
        "median_px": selection.median_px,
        "robust_sigma_px": selection.robust_sigma_px,
        "fallback_min_frames": selection.fallback_min_frames,
        "capped_max_frames": selection.capped_max_frames,
        "effective_min_selected_frames": selection.effective_min_selected_frames,
        "effective_max_selected_frames": selection.effective_max_selected_frames,
        "selected_frames": selection.selected_names,
        "rejected_frames": [
            {"name": name, "reason": reason} for name, reason in sorted(selection.rejected_reasons.items())
        ],
    }


def _frame_summary(records: list[DetectionRecord]) -> list[dict]:
    return [
        {
            "name": record.name,
            "marker_count": record.marker_count,
            "x": record.x,
            "y": record.y,
            "size": record.size,
            "skew": record.skew,
        }
        for record in records
    ]


def _write_diagnostics_csv(
    path: Path,
    records: list[DetectionRecord],
    all_solve: CalibrationSolve,
    selected_solve: CalibrationSolve,
    selected_names: set[str],
    frame_reasons: dict[str, str],
) -> None:
    selected_errors = selected_solve.view_errors_px
    fieldnames = [
        "frame",
        "marker_count",
        "x",
        "y",
        "size",
        "skew",
        "all_frame_error_px",
        "final_error_px",
        "selected",
        "reason",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    "frame": record.name,
                    "marker_count": record.marker_count,
                    "x": f"{record.x:.6f}",
                    "y": f"{record.y:.6f}",
                    "size": f"{record.size:.6f}",
                    "skew": f"{record.skew:.6f}",
                    "all_frame_error_px": f"{all_solve.view_errors_px[record.name]:.6f}",
                    "final_error_px": (
                        f"{selected_errors[record.name]:.6f}" if record.name in selected_errors else ""
                    ),
                    "selected": "true" if record.name in selected_names else "false",
                    "reason": frame_reasons.get(record.name, ""),
                }
            )


def _run_model(args, image_size, records, board, obj_by_id, model_name: str, flags: int) -> dict:
    output_dir = Path(args.output_dir)
    all_solve = _calibration_solve(image_size, records, board, obj_by_id, flags)
    all_yaml_path = output_dir / f"{args.camera_name}_all_frames_{model_name}.yaml"
    _save_yaml(all_yaml_path, args, image_size, all_solve.camera_matrix, all_solve.dist_coeffs, model_name)

    frame_reasons: dict[str, str] = {record.name: "auto_select_disabled" for record in records}
    selection_passes = []
    selected_records = records
    selected_solve = all_solve

    if args.auto_select:
        frame_reasons = {}
        current_records = records
        current_solve = all_solve
        selected_name_set = {record.name for record in records}
        for pass_index in range(1, args.selection_passes + 1):
            selection = _select_frame_subset(
                current_records,
                current_solve.view_errors_px,
                max_view_error_px=args.max_view_error_px,
                outlier_mad_multiplier=args.outlier_mad_multiplier,
                min_selected_frames=args.min_selected_frames,
                max_selected_frames=args.max_selected_frames,
            )
            selection_passes.append(_selection_summary(selection, pass_index))
            for name, reason in selection.selected_reasons.items():
                frame_reasons[name] = reason
            for name, reason in selection.rejected_reasons.items():
                frame_reasons[name] = reason

            selected_name_set = set(selection.selected_names)
            next_records = [record for record in current_records if record.name in selected_name_set]
            changed = len(next_records) != len(current_records) or any(
                left.name != right.name for left, right in zip(next_records, current_records)
            )
            selected_records = next_records
            if changed:
                selected_solve = _calibration_solve(image_size, selected_records, board, obj_by_id, flags)
                current_records = selected_records
                current_solve = selected_solve
            else:
                selected_solve = current_solve
                break

        for record in records:
            if record.name not in frame_reasons:
                frame_reasons[record.name] = "selected"

    selected_yaml_path = output_dir / f"{args.camera_name}_{model_name}.yaml"
    _save_yaml(
        selected_yaml_path,
        args,
        image_size,
        selected_solve.camera_matrix,
        selected_solve.dist_coeffs,
        model_name,
    )

    selected_names = {record.name for record in selected_records}
    diagnostics_csv_path = output_dir / f"{args.camera_name}_{model_name}_frame_diagnostics.csv"
    _write_diagnostics_csv(
        diagnostics_csv_path,
        records,
        all_solve,
        selected_solve,
        selected_names,
        frame_reasons,
    )

    all_summary = _solve_summary(all_solve, records, all_yaml_path)
    selected_summary = _solve_summary(selected_solve, selected_records, selected_yaml_path)
    rejected = [
        {
            "name": record.name,
            "reason": frame_reasons.get(record.name, ""),
            "all_view_error_px": float(all_solve.view_errors_px[record.name]),
        }
        for record in records
        if record.name not in selected_names
    ]
    selected_summary.update(
        {
            "frames": [record.name for record in selected_records],
            "rejected_frames": rejected,
            "selection_passes": selection_passes,
        }
    )

    return {
        "model": model_name,
        "rms": selected_summary["rms"],
        "median_view_error_px": selected_summary["median_view_error_px"],
        "worst_view_error_px": selected_summary["worst_view_error_px"],
        "camera_matrix": selected_summary["camera_matrix"],
        "distortion": selected_summary["distortion"],
        "yaml": str(selected_yaml_path),
        "all_frames": all_summary,
        "selected": selected_summary,
        "diagnostics_csv": str(diagnostics_csv_path),
    }


def _add_selection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--auto-select", dest="auto_select", action="store_true", default=True)
    parser.add_argument("--no-auto-select", dest="auto_select", action="store_false")
    parser.add_argument("--max-view-error-px", type=float, default=2.5)
    parser.add_argument("--outlier-mad-multiplier", type=float, default=3.5)
    parser.add_argument("--max-selected-frames", type=int, default=50)
    parser.add_argument("--selection-passes", type=int, default=2)
    parser.add_argument("--min-selected-frames", type=int, default=None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-dir", required=True)
    parser.add_argument("--glob", default="capture_*.jpg")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--camera-name", default="cam_overhead_gopro_hyperview")
    parser.add_argument("--cols", type=int, default=11)
    parser.add_argument("--rows", type=int, default=8)
    parser.add_argument("--square-m", type=float, default=0.034)
    parser.add_argument("--marker-m", type=float, default=0.025)
    parser.add_argument("--start-id", type=int, default=2)
    parser.add_argument("--marker-count", type=int, default=44)
    parser.add_argument("--aruco-dict", default="DICT_5X5_100")
    parser.add_argument("--min-markers", type=int, default=6)
    parser.add_argument("--min-frames", type=int, default=20)
    parser.add_argument("--rectify-alpha", type=float, default=0.0)
    _add_selection_args(parser)
    args = parser.parse_args()

    if args.min_selected_frames is None:
        args.min_selected_frames = args.min_frames
    if args.max_view_error_px <= 0:
        parser.error("--max-view-error-px must be positive")
    if args.outlier_mad_multiplier <= 0:
        parser.error("--outlier-mad-multiplier must be positive")
    if args.max_selected_frames <= 0:
        parser.error("--max-selected-frames must be positive")
    if args.selection_passes <= 0:
        parser.error("--selection-passes must be positive")
    if args.min_selected_frames <= 0:
        parser.error("--min-selected-frames must be positive")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    dictionary = _dictionary(args.aruco_dict)
    board, obj_by_id = _make_caib_board(args.cols, args.rows, args.square_m, args.marker_m, args.start_id, dictionary)
    image_size, records = _detect_frames(args, dictionary)

    results = [
        _run_model(args, image_size, records, board, obj_by_id, "plumb_bob", 0),
        _run_model(args, image_size, records, board, obj_by_id, "rational_polynomial", cv2.CALIB_RATIONAL_MODEL),
    ]
    summary = {
        "image_size": list(image_size),
        "frames": _frame_summary(records),
        "board": {
            "source": "caib.io",
            "cols": args.cols,
            "rows": args.rows,
            "square_m": args.square_m,
            "marker_m": args.marker_m,
            "start_id": args.start_id,
            "aruco_dict": args.aruco_dict,
        },
        "selection": {
            "enabled": bool(args.auto_select),
            "max_view_error_px": args.max_view_error_px,
            "outlier_mad_multiplier": args.outlier_mad_multiplier,
            "min_selected_frames": args.min_selected_frames,
            "max_selected_frames": args.max_selected_frames,
            "selection_passes": args.selection_passes,
        },
        "results": results,
    }
    with (output_dir / "caib_marker_board_calibration_summary.json").open("w") as stream:
        json.dump(summary, stream, indent=2)
    for result in results:
        selected = result["selected"]
        all_frames = result["all_frames"]
        print(
            f"{result['model']}: selected={selected['frame_count']}/{all_frames['frame_count']} "
            f"rms={result['rms']:.4f}px median={result['median_view_error_px']:.4f}px "
            f"worst={result['worst_view_error_px']:.4f}px -> {result['yaml']}"
        )
        print(f"  all-frame audit: {all_frames['yaml']}")
        print(f"  diagnostics: {result['diagnostics_csv']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
