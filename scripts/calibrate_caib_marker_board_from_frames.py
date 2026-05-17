#!/usr/bin/env python3
"""Calibrate intrinsics from frames of a caib.io ChArUco board.

caib.io's ChArUco generator can place ArUco markers on the opposite checker
parity from OpenCV's default CharucoBoard_create layout. This script calibrates
from detected marker corners directly, using the physical caib.io marker layout.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml


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


def _detect_frames(args, dictionary):
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    valid_ids = set(range(args.start_id, args.start_id + args.marker_count))
    per_frame = []
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
            per_frame.append((frame_path.name, kept_corners, np.asarray(kept_ids, dtype=np.int32)))
    if image_size is None:
        raise RuntimeError(f"No readable frames found in {args.frames_dir}")
    if len(per_frame) < args.min_frames:
        raise RuntimeError(f"Need at least {args.min_frames} usable frames, found {len(per_frame)}")
    return image_size, per_frame


def _flatten(per_frame):
    all_corners = []
    all_ids = []
    counter = []
    for _name, corners, ids in per_frame:
        all_corners.extend(corners)
        all_ids.extend(ids.reshape(-1, 1))
        counter.append(len(corners))
    return all_corners, np.asarray(all_ids, dtype=np.int32), np.asarray(counter, dtype=np.int32)


def _view_errors(per_frame, obj_by_id, camera_matrix, dist_coeffs, rvecs, tvecs):
    errors = []
    for (_name, corners, ids), rvec, tvec in zip(per_frame, rvecs, tvecs):
        object_points = []
        image_points = []
        for corner, marker_id in zip(corners, ids.ravel()):
            object_points.append(obj_by_id[int(marker_id)])
            image_points.append(corner.reshape(4, 2))
        object_points = np.concatenate(object_points, axis=0).astype(np.float32)
        image_points = np.concatenate(image_points, axis=0).astype(np.float32)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        diff = image_points - projected.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
    return errors


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


def _calibrate(args, image_size, per_frame, board, obj_by_id, model_name: str, flags: int):
    corners, ids, counter = _flatten(per_frame)
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraAruco(
        corners, ids, counter, board, image_size, None, None, flags=flags
    )
    errors = _view_errors(per_frame, obj_by_id, camera_matrix, dist_coeffs, rvecs, tvecs)
    yaml_path = Path(args.output_dir) / f"{args.camera_name}_{model_name}.yaml"
    _save_yaml(yaml_path, args, image_size, camera_matrix, dist_coeffs, model_name)
    return {
        "model": model_name,
        "rms": float(rms),
        "median_view_error_px": float(np.median(errors)),
        "worst_view_error_px": float(np.max(errors)),
        "camera_matrix": camera_matrix.tolist(),
        "distortion": np.asarray(dist_coeffs).ravel().tolist(),
        "yaml": str(yaml_path),
    }


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
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    dictionary = _dictionary(args.aruco_dict)
    board, obj_by_id = _make_caib_board(args.cols, args.rows, args.square_m, args.marker_m, args.start_id, dictionary)
    image_size, per_frame = _detect_frames(args, dictionary)

    results = [
        _calibrate(args, image_size, per_frame, board, obj_by_id, "plumb_bob", 0),
        _calibrate(args, image_size, per_frame, board, obj_by_id, "rational_polynomial", cv2.CALIB_RATIONAL_MODEL),
    ]
    summary = {
        "image_size": list(image_size),
        "frames": [name for name, _corners, _ids in per_frame],
        "board": {
            "source": "caib.io",
            "cols": args.cols,
            "rows": args.rows,
            "square_m": args.square_m,
            "marker_m": args.marker_m,
            "start_id": args.start_id,
            "aruco_dict": args.aruco_dict,
        },
        "results": results,
    }
    with (output_dir / "caib_marker_board_calibration_summary.json").open("w") as stream:
        json.dump(summary, stream, indent=2)
    for result in results:
        print(
            f"{result['model']}: rms={result['rms']:.4f}px "
            f"median={result['median_view_error_px']:.4f}px "
            f"worst={result['worst_view_error_px']:.4f}px -> {result['yaml']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
