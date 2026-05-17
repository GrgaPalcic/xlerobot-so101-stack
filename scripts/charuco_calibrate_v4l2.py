#!/usr/bin/env python3
"""Capture and calibrate ChArUco intrinsics directly from a V4L2 camera."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml


PARAM_RANGES = (0.7, 0.7, 0.4, 0.5)


def resolve_dictionary(name: str):
    attr = name if name.startswith("DICT_") else f"DICT_{name}"
    if not hasattr(aruco, attr):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    dict_id = getattr(aruco, attr)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)


def make_board(squares_x: int, squares_y: int, square_m: float, marker_m: float, dictionary):
    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(squares_x, squares_y, square_m, marker_m, dictionary)
    return aruco.CharucoBoard((squares_x, squares_y), square_m, marker_m, dictionary)


def board_corners(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(), dtype=np.float32)
    return np.asarray(board.chessboardCorners, dtype=np.float32)


def make_detector_params():
    if hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
    return params


def detect_charuco(gray, board, dictionary, detector_params, min_markers: int):
    marker_corners, marker_ids, _ = aruco.detectMarkers(gray, dictionary, parameters=detector_params)
    if marker_ids is None or len(marker_ids) == 0:
        return None, None
    try:
        _, char_corners, char_ids = aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray,
            board,
            minMarkers=min_markers,
        )
    except TypeError:
        _, char_corners, char_ids = aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
    if char_corners is None or char_ids is None or len(char_corners) < 4:
        return None, None
    return char_corners, char_ids


def largest_rectangle_corners(char_corners, char_ids, xdim: int, ydim: int):
    ids_flat = char_ids.ravel().astype(int)
    id_to_pixel = {int(corner_id): char_corners[idx, 0] for idx, corner_id in enumerate(ids_flat)}
    visible = set(id_to_pixel)
    best_area = 0
    best = None
    for y1 in range(ydim):
        for y2 in range(y1, ydim):
            for x1 in range(xdim):
                for x2 in range(x1, xdim):
                    area = (x2 - x1 + 1) * (y2 - y1 + 1)
                    if area <= best_area:
                        continue
                    ids = (
                        y1 * xdim + x1,
                        y1 * xdim + x2,
                        y2 * xdim + x1,
                        y2 * xdim + x2,
                    )
                    if all(corner_id in visible for corner_id in ids):
                        best_area = area
                        best = (x1, x2, y1, y2)
    if best is None or best_area < 4:
        return None
    x1, x2, y1, y2 = best
    return (
        id_to_pixel[y1 * xdim + x1],
        id_to_pixel[y1 * xdim + x2],
        id_to_pixel[y2 * xdim + x2],
        id_to_pixel[y2 * xdim + x1],
    )


def quad_area(tl, tr, br, bl) -> float:
    p = np.asarray([tl, tr, br, bl], dtype=np.float64)
    x = p[:, 0]
    y = p[:, 1]
    return 0.5 * abs(
        x[0] * y[1] - x[1] * y[0]
        + x[1] * y[2] - x[2] * y[1]
        + x[2] * y[3] - x[3] * y[2]
        + x[3] * y[0] - x[0] * y[3]
    )


def quad_skew(tl, tr, br) -> float:
    ab = tl - tr
    cb = br - tr
    cos_angle = np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb) + 1e-12)
    angle = math.acos(max(-1.0, min(1.0, float(cos_angle))))
    return min(1.0, 2.0 * abs((math.pi / 2.0) - angle))


def board_params(char_corners, char_ids, image_size, squares_x: int, squares_y: int):
    xdim = squares_x - 1
    ydim = squares_y - 1
    rect = largest_rectangle_corners(char_corners, char_ids, xdim, ydim)
    if rect is None:
        return None
    tl, tr, br, bl = rect
    width, height = image_size
    area = quad_area(tl, tr, br, bl)
    border = math.sqrt(max(area, 1.0))
    cx = float(np.mean([tl[0], tr[0], br[0], bl[0]]))
    cy = float(np.mean([tl[1], tr[1], br[1], bl[1]]))
    x = (cx - border / 2) / max(width - border, 1.0)
    y = (cy - border / 2) / max(height - border, 1.0)
    size = math.sqrt(area / max(width * height, 1.0))
    skew = quad_skew(tl, tr, br)
    return [
        float(np.clip(x, 0.0, 1.0)),
        float(np.clip(y, 0.0, 1.0)),
        float(np.clip(size, 0.0, 1.0)),
        float(skew),
    ]


def param_distance(a, b) -> float:
    return float(sum(abs(x - y) for x, y in zip(a, b)))


def progress(params_list: list[list[float]]) -> tuple[list[float], float]:
    if not params_list:
        return [0.0, 0.0, 0.0, 0.0], 0.0
    arr = np.asarray(params_list, dtype=np.float64)
    lo = arr.min(axis=0)
    hi = arr.max(axis=0)
    spans = [hi[0] - lo[0], hi[1] - lo[1], hi[2], hi[3]]
    per_axis = [min(float(span) / expected, 1.0) for span, expected in zip(spans, PARAM_RANGES)]
    return per_axis, float(np.mean(per_axis))


def compute_motion(prev_corners, prev_ids, corners, ids) -> float | None:
    if prev_corners is None or prev_ids is None:
        return None
    prev = {int(corner_id): prev_corners[idx, 0] for idx, corner_id in enumerate(prev_ids.ravel())}
    cur = {int(corner_id): corners[idx, 0] for idx, corner_id in enumerate(ids.ravel())}
    shared = sorted(set(prev) & set(cur))
    if not shared:
        return None
    deltas = np.asarray([cur[corner_id] - prev[corner_id] for corner_id in shared], dtype=np.float64)
    return float(np.mean(np.linalg.norm(deltas, axis=1)))


def draw_overlay(frame, corners, ids, message: str):
    overlay = frame.copy()
    if corners is not None and ids is not None:
        aruco.drawDetectedCornersCharuco(overlay, corners, ids)
    cv2.putText(overlay, message, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA)
    return overlay


def save_ros_yaml(path: Path, camera_name: str, image_size, k, d, distortion_model: str, alpha: float):
    width, height = image_size
    if distortion_model == "equidistant":
        r = np.eye(3, dtype=np.float64)
        p = np.zeros((3, 4), dtype=np.float64)
        p[:3, :3] = k
    else:
        k_new, _ = cv2.getOptimalNewCameraMatrix(k, d, image_size, alpha)
        r = np.eye(3, dtype=np.float64)
        p = np.zeros((3, 4), dtype=np.float64)
        p[:3, :3] = k_new
    info = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": camera_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": np.asarray(k).reshape(3, 3).flatten().tolist()},
        "distortion_model": distortion_model,
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(np.asarray(d).size),
            "data": np.asarray(d, dtype=np.float64).ravel().tolist(),
        },
        "rectification_matrix": {"rows": 3, "cols": 3, "data": r.flatten().tolist()},
        "projection_matrix": {"rows": 3, "cols": 4, "data": p.flatten().tolist()},
    }
    with path.open("w") as stream:
        yaml.safe_dump(info, stream, sort_keys=False)


def pinhole_view_errors(corners_list, ids_list, board, k, d, rvecs, tvecs) -> list[float]:
    corners_3d = board_corners(board)
    errors = []
    for corners, ids, rvec, tvec in zip(corners_list, ids_list, rvecs, tvecs):
        obj = corners_3d[ids.ravel()]
        projected, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
        diff = corners.reshape(-1, 2) - projected.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
    return errors


def fisheye_view_errors(object_points, image_points, k, d, rvecs, tvecs) -> list[float]:
    errors = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.fisheye.projectPoints(obj, rvec, tvec, k, d)
        diff = img.reshape(-1, 2) - projected.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
    return errors


def calibrate_all(output_dir: Path, args, image_size, board, corners_list, ids_list, params_list):
    if len(corners_list) < args.min_samples_calibrate:
        print(f"Need at least {args.min_samples_calibrate} captures, have {len(corners_list)}", flush=True)
        return 2

    results = []
    flags_by_name = {
        "plumb_bob": 0,
        "rational_polynomial": cv2.CALIB_RATIONAL_MODEL,
    }
    for model_name, flags in flags_by_name.items():
        try:
            err, k, d, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=corners_list,
                charucoIds=ids_list,
                board=board,
                imageSize=image_size,
                cameraMatrix=None,
                distCoeffs=None,
                flags=flags,
            )
            view_errors = pinhole_view_errors(corners_list, ids_list, board, k, d, rvecs, tvecs)
            yaml_path = output_dir / f"{args.camera_name}_{model_name}.yaml"
            save_ros_yaml(yaml_path, args.camera_name, image_size, k, d, model_name, args.rectify_alpha)
            results.append({
                "model": model_name,
                "rms": float(err),
                "median_view_error_px": float(np.median(view_errors)),
                "worst_view_error_px": float(np.max(view_errors)),
                "camera_matrix": k.tolist(),
                "distortion": np.asarray(d).ravel().tolist(),
                "yaml": str(yaml_path),
            })
        except cv2.error as exc:
            results.append({"model": model_name, "error": str(exc)})

    try:
        corners_3d = board_corners(board)
        object_points = [corners_3d[ids.ravel()].reshape(-1, 1, 3).astype(np.float64) for ids in ids_list]
        image_points = [corners.reshape(-1, 1, 2).astype(np.float64) for corners in corners_list]
        width, height = image_size
        k = np.array([[0.5 * width, 0.0, 0.5 * width], [0.0, 0.5 * width, 0.5 * height], [0.0, 0.0, 1.0]])
        d = np.zeros((4, 1), dtype=np.float64)
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
        err, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
            object_points,
            image_points,
            image_size,
            k,
            d,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7),
        )
        view_errors = fisheye_view_errors(object_points, image_points, k, d, rvecs, tvecs)
        yaml_path = output_dir / f"{args.camera_name}_equidistant.yaml"
        save_ros_yaml(yaml_path, args.camera_name, image_size, k, d, "equidistant", args.rectify_alpha)
        results.append({
            "model": "equidistant",
            "rms": float(err),
            "median_view_error_px": float(np.median(view_errors)),
            "worst_view_error_px": float(np.max(view_errors)),
            "camera_matrix": k.tolist(),
            "distortion": np.asarray(d).ravel().tolist(),
            "yaml": str(yaml_path),
        })
    except cv2.error as exc:
        results.append({"model": "equidistant", "error": str(exc)})

    per_axis, overall = progress(params_list)
    summary = {
        "capture_count": len(corners_list),
        "image_size": [int(image_size[0]), int(image_size[1])],
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_m": args.square_m,
            "marker_m": args.marker_m,
            "aruco_dict": args.aruco_dict,
        },
        "coverage": {
            "x": per_axis[0],
            "y": per_axis[1],
            "size": per_axis[2],
            "skew": per_axis[3],
            "overall": overall,
        },
        "results": results,
    }
    with (output_dir / "calibration_summary.json").open("w") as stream:
        json.dump(summary, stream, indent=2)

    print("\nCalibration results:", flush=True)
    for result in results:
        if "error" in result:
            print(f"  {result['model']}: failed: {result['error'].splitlines()[0]}", flush=True)
        else:
            print(
                f"  {result['model']}: rms={result['rms']:.4f}px "
                f"median={result['median_view_error_px']:.4f}px "
                f"worst={result['worst_view_error_px']:.4f}px -> {result['yaml']}",
                flush=True,
            )
    return 0


def load_capture_npz(path: Path):
    data = np.load(path, allow_pickle=True)
    corners = [np.asarray(item, dtype=np.float32) for item in data["corners"]]
    ids = [np.asarray(item, dtype=np.int32) for item in data["ids"]]
    params = [list(map(float, item)) for item in data["params"]]
    image_size = tuple(int(v) for v in data["image_size"])
    return image_size, corners, ids, params


def save_capture_npz(path: Path, image_size, corners_list, ids_list, params_list):
    np.savez_compressed(
        path,
        image_size=np.asarray(image_size, dtype=np.int32),
        corners=np.asarray(corners_list, dtype=object),
        ids=np.asarray(ids_list, dtype=object),
        params=np.asarray(params_list, dtype=object),
    )


def run_capture(args, output_dir: Path, board, dictionary, detector_params):
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open {args.device}")

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
    image_size = (actual_width, actual_height)
    print(f"Capturing {args.device} at {actual_width}x{actual_height} {args.fps:g}fps fourcc={args.fourcc}", flush=True)
    print(f"Output: {output_dir}", flush=True)
    print("Move the board slowly. Cover center, all four corners, near/far sizes, and strong tilts.", flush=True)

    corners_list = []
    ids_list = []
    params_list = []
    prev_corners = None
    prev_ids = None
    last_status = 0.0
    last_capture = 0.0
    stop_requested = False
    manual_capture_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    while not stop_requested:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Camera read failed", flush=True)
            time.sleep(0.25)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = detect_charuco(gray, board, dictionary, detector_params, args.min_markers)
        message = "no board"
        capture_now = False
        params = None
        motion = None
        if corners is not None and ids is not None and len(corners) >= args.min_corners:
            params = board_params(corners, ids, image_size, args.squares_x, args.squares_y)
            motion = compute_motion(prev_corners, prev_ids, corners, ids)
            if params is not None and motion is not None:
                nearest = min((param_distance(params, old) for old in params_list), default=999.0)
                still = motion <= args.max_motion_px
                novel = nearest >= args.min_param_dist or len(params_list) == 0
                capture_now = (
                    manual_capture_requested
                    or (args.auto_capture and still and novel and time.monotonic() - last_capture > args.capture_cooldown_s)
                )
                message = (
                    f"{len(corners)} corners motion={motion:.1f}px nearest={nearest:.2f} "
                    f"x={params[0]:.2f} y={params[1]:.2f} size={params[2]:.2f} skew={params[3]:.2f}"
                )
            else:
                message = f"{len(corners)} corners; hold still"
            prev_corners = corners.copy()
            prev_ids = ids.copy()
        else:
            have = 0 if corners is None else len(corners)
            message = f"{have} corners; need {args.min_corners}"
            prev_corners = None
            prev_ids = None

        overlay = draw_overlay(frame, corners, ids, f"{len(corners_list)}/{args.target_samples} {message}")
        if capture_now and params is not None:
            idx = len(corners_list) + 1
            corners_list.append(corners.copy())
            ids_list.append(ids.copy())
            params_list.append(params)
            cv2.imwrite(str(output_dir / "frames" / f"capture_{idx:03d}.jpg"), frame)
            cv2.imwrite(str(output_dir / "overlays" / f"capture_{idx:03d}.jpg"), overlay)
            save_capture_npz(output_dir / "captures.npz", image_size, corners_list, ids_list, params_list)
            last_capture = time.monotonic()
            manual_capture_requested = False
            per_axis, overall = progress(params_list)
            print(
                f"CAPTURE {idx:03d}: corners={len(corners)} coverage="
                f"x={per_axis[0]:.0%} y={per_axis[1]:.0%} size={per_axis[2]:.0%} "
                f"skew={per_axis[3]:.0%} overall={overall:.0%}",
                flush=True,
            )

        now = time.monotonic()
        if now - last_status > 1.0:
            cv2.imwrite(str(output_dir / "latest_detection.jpg"), overlay)
            per_axis, overall = progress(params_list)
            print(
                f"status captures={len(corners_list)}/{args.target_samples} "
                f"coverage={overall:.0%} {message}",
                flush=True,
            )
            last_status = now

        if args.window:
            cv2.imshow("charuco calibration", overlay)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                stop_requested = True
            elif key == ord("c") and corners is not None and ids is not None and params is not None:
                manual_capture_requested = True

        if len(corners_list) >= args.target_samples:
            print(f"Target samples reached: {args.target_samples}", flush=True)
            break

    cap.release()
    if args.window:
        cv2.destroyAllWindows()
    save_capture_npz(output_dir / "captures.npz", image_size, corners_list, ids_list, params_list)
    return image_size, corners_list, ids_list, params_list


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video42")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="YUYV")
    parser.add_argument("--camera-name", default="cam_overhead")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--calibrate-existing", default="", help="Path to an existing captures.npz")
    parser.add_argument("--squares-x", type=int, default=11)
    parser.add_argument("--squares-y", type=int, default=8)
    parser.add_argument("--square-m", type=float, default=0.034)
    parser.add_argument("--marker-m", type=float, default=0.024)
    parser.add_argument("--aruco-dict", default="DICT_5X5_100")
    parser.add_argument("--target-samples", type=int, default=55)
    parser.add_argument("--min-samples-calibrate", type=int, default=18)
    parser.add_argument("--min-corners", type=int, default=18)
    parser.add_argument("--min-markers", type=int, default=1)
    parser.add_argument("--max-motion-px", type=float, default=3.0)
    parser.add_argument("--min-param-dist", type=float, default=0.12)
    parser.add_argument("--capture-cooldown-s", type=float, default=0.7)
    parser.add_argument("--rectify-alpha", type=float, default=0.0)
    parser.add_argument("--auto-capture", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--window", action="store_true", help="Show an OpenCV preview window when running locally")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output_dir:
        output_dir = Path(args.output_dir)
    elif args.calibrate_existing:
        output_dir = Path(args.calibrate_existing).resolve().parent
    else:
        output_dir = Path.cwd() / "calibration_runs" / f"{args.camera_name}_charuco_{time.strftime('%Y%m%d_%H%M%S')}"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "frames").mkdir(exist_ok=True)
    (output_dir / "overlays").mkdir(exist_ok=True)

    dictionary = resolve_dictionary(args.aruco_dict)
    board = make_board(args.squares_x, args.squares_y, args.square_m, args.marker_m, dictionary)
    detector_params = make_detector_params()

    if args.calibrate_existing:
        image_size, corners_list, ids_list, params_list = load_capture_npz(Path(args.calibrate_existing))
    else:
        image_size, corners_list, ids_list, params_list = run_capture(args, output_dir, board, dictionary, detector_params)

    return calibrate_all(output_dir, args, image_size, board, corners_list, ids_list, params_list)


if __name__ == "__main__":
    sys.exit(main())
