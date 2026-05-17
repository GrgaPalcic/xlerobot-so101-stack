#!/usr/bin/env python3
"""Capture calibration frames from a V4L2 camera using a caib.io marker board."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np


PARAM_RANGES = (0.7, 0.7, 0.45, 0.5)


def _dictionary(name: str):
    attr = name if name.startswith("DICT_") else f"DICT_{name}"
    return aruco.getPredefinedDictionary(getattr(aruco, attr))


def _detector_params():
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return params


def _marker_motion(prev, cur):
    if prev is None:
        return None
    shared = sorted(set(prev) & set(cur))
    if not shared:
        return None
    deltas = np.asarray([cur[i] - prev[i] for i in shared], dtype=np.float64)
    return float(np.mean(np.linalg.norm(deltas, axis=1)))


def _quad_skew(points):
    rect = cv2.minAreaRect(points.astype(np.float32))
    angle = abs(float(rect[2]))
    angle = min(angle, abs(90.0 - angle))
    return min(angle / 45.0, 1.0)


def _params_from_points(points, image_size):
    width, height = image_size
    hull = cv2.convexHull(points.astype(np.float32))
    area = float(cv2.contourArea(hull))
    cx = float(points[:, 0].mean())
    cy = float(points[:, 1].mean())
    size = (area / max(width * height, 1.0)) ** 0.5
    return [
        float(np.clip(cx / width, 0.0, 1.0)),
        float(np.clip(cy / height, 0.0, 1.0)),
        float(np.clip(size, 0.0, 1.0)),
        _quad_skew(points),
    ]


def _distance(a, b):
    return float(sum(abs(x - y) for x, y in zip(a, b)))


def _progress(params_list):
    if not params_list:
        return [0.0, 0.0, 0.0, 0.0], 0.0
    arr = np.asarray(params_list, dtype=np.float64)
    lo = arr.min(axis=0)
    hi = arr.max(axis=0)
    spans = [hi[0] - lo[0], hi[1] - lo[1], hi[2], hi[3]]
    axes = [min(float(span) / expected, 1.0) for span, expected in zip(spans, PARAM_RANGES)]
    return axes, float(np.mean(axes))


def _detect(gray, dictionary, params, start_id, marker_count):
    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return [], None
    valid_ids = set(range(start_id, start_id + marker_count))
    kept = []
    centers = {}
    for corner, marker_id in zip(corners, ids.ravel()):
        marker_id = int(marker_id)
        if marker_id not in valid_ids:
            continue
        pts = corner.reshape(4, 2).astype(np.float32)
        kept.append((marker_id, pts))
        centers[marker_id] = pts.mean(axis=0)
    return kept, centers


def _draw(frame, kept, text):
    display = frame.copy()
    if kept:
        corners = [pts.reshape(1, 4, 2).astype(np.float32) for _marker_id, pts in kept]
        ids = np.asarray([[marker_id] for marker_id, _pts in kept], dtype=np.int32)
        aruco.drawDetectedMarkers(display, corners, ids)
    cv2.putText(display, text, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA)
    return display


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--camera-name", default="cam_wrist_arducam_uc852")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--aruco-dict", default="DICT_5X5_100")
    parser.add_argument("--start-id", type=int, default=2)
    parser.add_argument("--marker-count", type=int, default=44)
    parser.add_argument("--target-samples", type=int, default=50)
    parser.add_argument("--min-markers", type=int, default=8)
    parser.add_argument("--max-motion-px", type=float, default=3.0)
    parser.add_argument("--min-param-dist", type=float, default=0.11)
    parser.add_argument("--capture-cooldown-s", type=float, default=0.7)
    parser.add_argument("--auto-capture", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    frames_dir = output_dir / "frames"
    overlays_dir = output_dir / "overlays"
    frames_dir.mkdir(parents=True, exist_ok=True)
    overlays_dir.mkdir(parents=True, exist_ok=True)

    dictionary = _dictionary(args.aruco_dict)
    detector_params = _detector_params()
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open {args.device}")

    image_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width, int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height)
    print(f"Capturing {args.device} at {image_size[0]}x{image_size[1]} {args.fps:g}fps fourcc={args.fourcc}", flush=True)
    print(f"Output: {output_dir}", flush=True)
    print("Move pose-by-pose and pause. For close-focus lenses, stay in sharp focus; partial board views are OK.", flush=True)

    stop = False

    def _stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    prev_centers = None
    captured_params = []
    last_capture = 0.0
    last_status = 0.0
    count = 0
    while not stop and count < args.target_samples:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Camera read failed", flush=True)
            time.sleep(0.2)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kept, centers = _detect(gray, dictionary, detector_params, args.start_id, args.marker_count)
        now = time.monotonic()
        message = f"{len(kept)} markers"
        capture = False
        params = None
        if len(kept) >= args.min_markers and centers is not None:
            points = np.concatenate([pts for _marker_id, pts in kept], axis=0)
            params = _params_from_points(points, image_size)
            motion = _marker_motion(prev_centers, centers)
            nearest = min((_distance(params, old) for old in captured_params), default=999.0)
            if motion is None:
                message += "; hold still"
            else:
                message += f" motion={motion:.1f}px nearest={nearest:.2f} x={params[0]:.2f} y={params[1]:.2f} size={params[2]:.2f} skew={params[3]:.2f}"
                capture = (
                    args.auto_capture
                    and motion <= args.max_motion_px
                    and nearest >= args.min_param_dist
                    and now - last_capture >= args.capture_cooldown_s
                )
            prev_centers = {marker_id: center.copy() for marker_id, center in centers.items()}
        else:
            prev_centers = None
            message += f"; need {args.min_markers}"

        overlay = _draw(frame, kept, f"{count}/{args.target_samples} {message}")
        if capture and params is not None:
            count += 1
            captured_params.append(params)
            cv2.imwrite(str(frames_dir / f"capture_{count:03d}.jpg"), frame)
            cv2.imwrite(str(overlays_dir / f"capture_{count:03d}.jpg"), overlay)
            last_capture = now
            axes, overall = _progress(captured_params)
            print(
                f"CAPTURE {count:03d}: markers={len(kept)} "
                f"coverage=x={axes[0]:.0%} y={axes[1]:.0%} size={axes[2]:.0%} skew={axes[3]:.0%} overall={overall:.0%}",
                flush=True,
            )

        if now - last_status > 1.0:
            cv2.imwrite(str(output_dir / "latest_detection.jpg"), overlay)
            axes, overall = _progress(captured_params)
            print(f"status captures={count}/{args.target_samples} coverage={overall:.0%} {message}", flush=True)
            last_status = now

    cap.release()
    print(f"Finished with {count} captures in {frames_dir}", flush=True)
    return 0 if count >= args.target_samples else 2


if __name__ == "__main__":
    sys.exit(main())
