#!/usr/bin/env python3
"""Capture wrist camera board observations and robot poses for hand-eye solve."""

from __future__ import annotations

import argparse
import json
import math
import queue
import select
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from record_board_touch_points import (
    ARM_JOINTS,
    JogSession,
    JointJogSession,
    matrix_to_quat_xyzw,
    parse_jog_command,
    wait_for_tool_pose,
)
from solve_camera_extrinsics_from_board import (
    _detect_board,
    _dictionary,
    _invert_transform,
    _load_camera_info,
    _make_caib_marker_object_points,
    _make_transform,
    _reprojection_error,
    _solve_pnp,
    _to_plain,
)


WEB_UI_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SO101 Vision Hand-Eye</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #111418; color: #eef2f7; }
    body { margin: 0; padding: 14px; }
    main { display: grid; grid-template-columns: minmax(320px, 1.5fr) minmax(300px, 0.9fr); gap: 14px; max-width: 1220px; margin: 0 auto; }
    h1 { font-size: 22px; margin: 0 0 8px; }
    h2 { font-size: 15px; margin: 14px 0 8px; color: #d7dee8; }
    img { width: 100%; background: #05070a; border: 1px solid #303844; border-radius: 8px; }
    .panel { border: 1px solid #303844; background: #171c22; border-radius: 8px; padding: 12px; }
    .status { white-space: pre-wrap; min-height: 120px; color: #d7dee8; }
    .grid { display: grid; grid-template-columns: repeat(3, minmax(78px, 1fr)); gap: 8px; }
    .joint-grid { display: grid; grid-template-columns: repeat(2, minmax(112px, 1fr)); gap: 8px; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    button { min-height: 46px; border: 1px solid #394455; border-radius: 8px; background: #222a34; color: #eef2f7; font-size: 16px; font-weight: 640; }
    button:active { transform: translateY(1px); background: #2f3a49; }
    button.primary { background: #1f6feb; border-color: #2d7cf0; }
    button.warn { background: #7a2323; border-color: #a13a3a; }
    label { display: block; color: #a9b4c2; font-size: 13px; margin-bottom: 4px; }
    input { width: 100%; box-sizing: border-box; min-height: 40px; padding: 8px 10px; border-radius: 8px; border: 1px solid #394455; background: #151a20; color: #eef2f7; font-size: 15px; }
    .inputs { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .note { color: #a9b4c2; font-size: 13px; line-height: 1.35; margin-top: 8px; }
    @media (max-width: 850px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
<main>
  <section>
    <h1>SO101 Vision Hand-Eye</h1>
    <img id="preview" src="/latest.jpg" alt="latest wrist camera overlay">
  </section>
  <section class="panel">
    <div id="status" class="status">Waiting for camera...</div>
    <div class="inputs">
      <div><label for="cartStep">Cartesian step, mm</label><input id="cartStep" type="number" min="0.1" max="100" step="0.5" value="5"></div>
      <div><label for="jointStep">Joint step, deg</label><input id="jointStep" type="number" min="0.2" max="45" step="0.5" value="3"></div>
      <div><label for="durationSec">Duration, sec</label><input id="durationSec" type="number" min="0.1" max="10" step="0.1" value="1.5" onchange="sendDuration()"></div>
      <div><label for="sampleNote">Sample target</label><input id="sampleNote" value="varied pose"></div>
    </div>
    <div class="note">Keep the fixed 7x5 board still. Collect many distinct wrist poses with the board visible; roll/tilt/range diversity matters more than exact positioning.</div>

    <h2>Cartesian</h2>
    <div class="grid">
      <button onclick="sendAxis('x-')">X-</button>
      <button onclick="sendAxis('z+')">Z+</button>
      <button onclick="sendAxis('x+')">X+</button>
      <button onclick="sendAxis('y-')">Y-</button>
      <button onclick="sendAxis('z-')">Z-</button>
      <button onclick="sendAxis('y+')">Y+</button>
    </div>

    <h2>Joints</h2>
    <div class="joint-grid">
      <button onclick="sendJoint('pan-')">Pan -</button><button onclick="sendJoint('pan+')">Pan +</button>
      <button onclick="sendJoint('lift-')">Lift -</button><button onclick="sendJoint('lift+')">Lift +</button>
      <button onclick="sendJoint('elbow-')">Elbow -</button><button onclick="sendJoint('elbow+')">Elbow +</button>
      <button onclick="sendJoint('wrist-')">Wrist -</button><button onclick="sendJoint('wrist+')">Wrist +</button>
      <button onclick="sendJoint('roll-')">Roll -</button><button onclick="sendJoint('roll+')">Roll +</button>
    </div>

    <h2>Record</h2>
    <div class="row">
      <button onclick="send('pose')">Pose</button>
      <button onclick="send('sync')">Sync Joints</button>
      <button class="primary" onclick="sendSample()">Sample</button>
      <button class="warn" onclick="send('q')">Quit</button>
    </div>
  </section>
</main>
<script>
let lastSeq = -1;
let stepInputsInitialized = false;
async function send(command) {
  await fetch('/api/command', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({command})
  });
  await refresh();
}
function sendAxis(axis) {
  const mm = Number(document.getElementById('cartStep').value || 5);
  send(`${axis} ${mm / 1000}`);
}
function sendJoint(joint) {
  const deg = Number(document.getElementById('jointStep').value || 3);
  send(`${joint} ${deg * Math.PI / 180}`);
}
function sendDuration() {
  const seconds = Number(document.getElementById('durationSec').value || 1.5);
  send(`dur ${seconds}`);
}
function sendSample() {
  const note = document.getElementById('sampleNote').value || '';
  send(`sample ${note}`);
}
async function refresh() {
  const res = await fetch('/api/state');
  const data = await res.json();
  document.getElementById('status').textContent = data.status || '';
  if (!stepInputsInitialized) {
    if (data.cart_step_m) document.getElementById('cartStep').value = (data.cart_step_m * 1000).toFixed(1);
    if (data.joint_step_rad) document.getElementById('jointStep').value = (data.joint_step_rad * 180 / Math.PI).toFixed(1);
    if (data.duration_s) document.getElementById('durationSec').value = Number(data.duration_s).toFixed(1);
    stepInputsInitialized = true;
  }
  if (data.latest_seq !== lastSeq) {
    lastSeq = data.latest_seq;
    document.getElementById('preview').src = `/latest.jpg?seq=${lastSeq}`;
  }
}
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
"""


class HandEyeWebInterface:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.command_queue: queue.Queue[str] = queue.Queue()
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "status": "Waiting for camera...",
            "latest_seq": 0,
            "cart_step_m": 0.005,
            "joint_step_rad": math.radians(3.0),
            "duration_s": 1.5,
        }
        self._jpeg: bytes | None = None
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        interface = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: Any) -> None:
                return

            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    body = WEB_UI_HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if parsed.path == "/api/state":
                    interface._send_json(self, interface.state())
                    return
                if parsed.path == "/latest.jpg":
                    jpeg = interface.latest_jpeg()
                    if jpeg is None:
                        self.send_error(503)
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    return
                self.send_error(404)

            def do_POST(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path != "/api/command":
                    self.send_error(404)
                    return
                length = int(self.headers.get("Content-Length", "0"))
                try:
                    payload = json.loads(self.rfile.read(length).decode("utf-8"))
                    command = str(payload.get("command", "")).strip()
                except Exception:
                    self.send_error(400)
                    return
                if command:
                    interface.command_queue.put(command)
                    interface.set_status(f"Queued: {command}")
                interface._send_json(self, {"ok": True})

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        actual_port = int(self._server.server_address[1])
        print(f"Vision hand-eye UI: http://127.0.0.1:{actual_port}/")
        if self.host in ("", "0.0.0.0"):
            print(f"Vision hand-eye UI from another machine: http://192.168.1.73:{actual_port}/")

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def state(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def set_status(
        self,
        status: str,
        *,
        cart_step_m: float | None = None,
        joint_step_rad: float | None = None,
        duration_s: float | None = None,
    ) -> None:
        with self._lock:
            self._state["status"] = status
            if cart_step_m is not None:
                self._state["cart_step_m"] = cart_step_m
            if joint_step_rad is not None:
                self._state["joint_step_rad"] = joint_step_rad
            if duration_s is not None:
                self._state["duration_s"] = duration_s

    def update_image(self, image: np.ndarray) -> None:
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if not ok:
            return
        with self._lock:
            self._jpeg = encoded.tobytes()
            self._state["latest_seq"] = int(self._state["latest_seq"]) + 1

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def get_command_nowait(self) -> str | None:
        try:
            return self.command_queue.get_nowait()
        except queue.Empty:
            return None

    @staticmethod
    def _send_json(handler: BaseHTTPRequestHandler, data: dict[str, Any]) -> None:
        body = json.dumps(data).encode("utf-8")
        handler.send_response(200)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)


class LatestObservation:
    def __init__(self) -> None:
        self.frame: np.ndarray | None = None
        self.overlay: np.ndarray | None = None
        self.t_camera_world: np.ndarray | None = None
        self.quality: dict[str, Any] = {}
        self.status: str = "No valid board pose yet."


def transform_record(name: str, parent_frame: str, child_frame: str, tf: np.ndarray) -> dict[str, Any]:
    return {
        "name": name,
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation_xyz": tf[:3, 3].tolist(),
        "rotation_matrix": tf[:3, :3].tolist(),
        "quaternion_xyzw": matrix_to_quat_xyzw(tf[:3, :3]),
    }


def draw_overlay(
    image: np.ndarray,
    corners: list[np.ndarray],
    ids: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    label: str,
) -> np.ndarray:
    overlay = image.copy()
    aruco.drawDetectedMarkers(overlay, corners, ids)
    axis = np.asarray([[0.0, 0.0, 0.0], [0.06, 0.0, 0.0], [0.0, 0.06, 0.0], [0.0, 0.0, -0.06]], dtype=np.float32)
    image_axis, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    pts = image_axis.reshape(-1, 2).astype(int)
    cv2.line(overlay, tuple(pts[0]), tuple(pts[1]), (0, 0, 255), 3)
    cv2.line(overlay, tuple(pts[0]), tuple(pts[2]), (0, 255, 0), 3)
    cv2.line(overlay, tuple(pts[0]), tuple(pts[3]), (255, 0, 0), 3)
    cv2.putText(overlay, label, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.82, (0, 255, 255), 2, cv2.LINE_AA)
    return overlay


def update_observation(
    obs: LatestObservation,
    frame: np.ndarray,
    *,
    dictionary,
    obj_by_id: dict[int, np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    min_markers: int,
    max_sample_reproj_px: float,
    camera_frame: str,
) -> None:
    obs.frame = frame.copy()
    try:
        object_points, image_points, corners, ids = _detect_board(frame, dictionary, obj_by_id, min_markers)
        rvec, tvec, inlier_idx = _solve_pnp(object_points, image_points, camera_matrix, dist_coeffs)
        mean_err, max_err = _reprojection_error(
            object_points[inlier_idx],
            image_points[inlier_idx],
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
        )
        rot, _ = cv2.Rodrigues(rvec)
        obs.t_camera_world = _make_transform(rot, tvec.reshape(3))
        obs.quality = {
            "detected_markers": int(len(corners)),
            "inlier_points": int(len(inlier_idx)),
            "total_points": int(len(object_points)),
            "mean_reprojection_error_px": mean_err,
            "max_reprojection_error_px": max_err,
            "image_width": int(frame.shape[1]),
            "image_height": int(frame.shape[0]),
        }
        sample_ok = mean_err <= max_sample_reproj_px
        label = f"{camera_frame}: {len(corners)} markers, err {mean_err:.2f}px"
        if not sample_ok:
            label += " HIGH"
        obs.overlay = draw_overlay(frame, corners, ids, camera_matrix, dist_coeffs, rvec, tvec, label)
        obs.status = label
    except Exception as exc:
        obs.t_camera_world = None
        obs.quality = {}
        obs.overlay = frame.copy()
        cv2.putText(obs.overlay, str(exc)[:100], (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 180, 255), 2, cv2.LINE_AA)
        obs.status = f"Board pose unavailable: {exc}"


def read_terminal_command() -> str | None:
    if not sys.stdin.isatty():
        return None
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if readable:
        return sys.stdin.readline().strip()
    return None


def save_sample(
    *,
    args: argparse.Namespace,
    obs: LatestObservation,
    tf_buffer: Buffer,
    joint_jog: JointJogSession | None,
    sample_index: int,
    note: str,
) -> dict[str, Any]:
    if obs.frame is None or obs.overlay is None or obs.t_camera_world is None:
        raise RuntimeError("no valid board pose to sample")
    mean_error = float(obs.quality.get("mean_reprojection_error_px", math.inf))
    if mean_error > args.max_sample_reproj_px:
        raise RuntimeError(f"board reprojection error {mean_error:.3f}px exceeds {args.max_sample_reproj_px:.3f}px")

    t_base_gripper = wait_for_tool_pose(tf_buffer, args.base_frame, args.gripper_frame, args.tf_timeout_s)
    t_gripper_base = _invert_transform(t_base_gripper)
    name = f"sample_{sample_index:03d}"
    frame_rel = Path("frames") / f"{name}.jpg"
    overlay_rel = Path("overlays") / f"{name}.jpg"
    frame_path = args.output_dir / frame_rel
    overlay_path = args.output_dir / overlay_rel
    frame_path.parent.mkdir(parents=True, exist_ok=True)
    overlay_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(frame_path), obs.frame)
    cv2.imwrite(str(overlay_path), obs.overlay)

    joint_state = {}
    if joint_jog is not None:
        measured = joint_jog._measured_positions()  # noqa: SLF001 - field utility sharing live cache.
        joint_state = {
            "names": list(ARM_JOINTS),
            "positions": [float(value) for value in measured],
        }

    row = {
        "name": name,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "note": note,
        "frame": str(frame_rel),
        "overlay": str(overlay_rel),
        "world_frame": args.world_frame,
        "base_frame": args.base_frame,
        "gripper_frame": args.gripper_frame,
        "camera_frame": args.camera_frame,
        "T_camera_world": transform_record("T_camera_world", args.camera_frame, args.world_frame, obs.t_camera_world),
        "T_base_gripper": transform_record("T_base_gripper", args.base_frame, args.gripper_frame, t_base_gripper),
        "T_gripper_base": transform_record("T_gripper_base", args.gripper_frame, args.base_frame, t_gripper_base),
        "joint_state": joint_state,
        "quality": obs.quality,
    }
    with (args.output_dir / "samples.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(_to_plain(row), sort_keys=True) + "\n")
    return row


def write_summary(args: argparse.Namespace, sample_count: int) -> None:
    summary = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "camera_frame": args.camera_frame,
        "base_frame": args.base_frame,
        "gripper_frame": args.gripper_frame,
        "world_frame": args.world_frame,
        "camera_info": str(args.camera_info),
        "device": args.device,
        "samples": sample_count,
        "target_samples": args.target_samples,
        "board": {
            "cols": args.cols,
            "rows": args.rows,
            "square_m": args.square_m,
            "marker_m": args.marker_m,
            "start_id": args.start_id,
            "marker_count": args.marker_count,
            "dictionary": args.aruco_dict,
        },
    }
    (args.output_dir / "samples_summary.json").write_text(
        json.dumps(_to_plain(summary), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def existing_sample_counts(samples_path: Path) -> tuple[int, int]:
    saved = 0
    max_index = 0
    if not samples_path.exists():
        return saved, max_index
    for line in samples_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        saved += 1
        try:
            name = str(json.loads(line).get("name", ""))
        except json.JSONDecodeError:
            continue
        if name.startswith("sample_"):
            try:
                max_index = max(max_index, int(name.rsplit("_", 1)[1]))
            except ValueError:
                pass
    return saved, max_index


def status_text(args: argparse.Namespace, obs: LatestObservation, sample_count: int, last_action: str) -> str:
    return "\n".join(
        [
            f"samples: {sample_count}/{args.target_samples}",
            obs.status,
            f"frames: {args.world_frame}, {args.base_frame}, {args.gripper_frame}, {args.camera_frame}",
            f"sample threshold: mean reprojection <= {args.max_sample_reproj_px:.2f}px",
            last_action,
            "terminal commands: x+/x-/y+/y-/z+/z-, pan+/-, lift+/-, elbow+/-, wrist+/-, roll+/-, sample, pose, sync, q",
        ]
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--camera-name", default="")
    parser.add_argument("--camera-info", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--base-frame", required=True)
    parser.add_argument("--gripper-frame", required=True)
    parser.add_argument("--camera-frame", required=True)
    parser.add_argument("--cols", type=int, default=7)
    parser.add_argument("--rows", type=int, default=5)
    parser.add_argument("--square-m", type=float, default=0.020)
    parser.add_argument("--marker-m", type=float, default=0.014)
    parser.add_argument("--start-id", type=int, default=49)
    parser.add_argument("--marker-count", type=int, default=17)
    parser.add_argument("--aruco-dict", default="DICT_5X5_100")
    parser.add_argument("--min-markers", type=int, default=8)
    parser.add_argument("--max-sample-reproj-px", type=float, default=2.5)
    parser.add_argument("--target-samples", type=int, default=30)
    parser.add_argument("--tf-timeout-s", type=float, default=5.0)
    parser.add_argument("--jog-service", default="")
    parser.add_argument("--jog-step-m", type=float, default=0.005)
    parser.add_argument("--max-jog-step-m", type=float, default=0.10)
    parser.add_argument("--jog-duration-sec", type=float, default=1.5)
    parser.add_argument("--jog-timeout-s", type=float, default=8.0)
    parser.add_argument("--jog-strategy", choices=("cartesian", "joint_quintic"), default="cartesian")
    parser.add_argument("--joint-jog-topic", default="")
    parser.add_argument("--joint-states-topic", default="")
    parser.add_argument("--joint-step-rad", type=float, default=math.radians(3.0))
    parser.add_argument("--web-port", type=int, default=8780)
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--command-speed", type=int, default=2400)
    parser.add_argument("--command-acceleration", type=int, default=50)
    parser.add_argument("--profile-command", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if bool(args.joint_jog_topic) != bool(args.joint_states_topic):
        print("ERROR: --joint-jog-topic and --joint-states-topic must be used together", file=sys.stderr)
        return 2
    if args.jog_step_m <= 0.0 or args.max_jog_step_m <= 0.0 or args.jog_step_m > args.max_jog_step_m:
        print("ERROR: invalid jog step settings", file=sys.stderr)
        return 2
    if args.jog_duration_sec <= 0.0:
        print("ERROR: --jog-duration-sec must be positive", file=sys.stderr)
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "frames").mkdir(exist_ok=True)
    (args.output_dir / "overlays").mkdir(exist_ok=True)
    camera_matrix, dist_coeffs, expected_size = _load_camera_info(args.camera_info)
    obj_by_id = _make_caib_marker_object_points(args.cols, args.rows, args.square_m, args.marker_m, args.start_id)
    dictionary = _dictionary(args.aruco_dict)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if not cap.isOpened():
        print(f"ERROR: failed to open V4L2 device {args.device}", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("so101_wrist_handeye_capture")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    joint_jog: JointJogSession | None = None
    if args.joint_jog_topic:
        joint_jog = JointJogSession(
            node=node,
            joint_states_topic=args.joint_states_topic,
            command_topic=args.joint_jog_topic,
            joint_step_rad=args.joint_step_rad,
            duration_s=args.jog_duration_sec,
            timeout_s=args.jog_timeout_s,
        )

    jog_session: JogSession | None = None
    if args.jog_service:
        jog_session = JogSession(
            node=node,
            tf_buffer=tf_buffer,
            base_frame=args.base_frame,
            tool_frame=args.gripper_frame,
            tool_offset=np.zeros(3, dtype=float),
            service_name=args.jog_service,
            step_m=args.jog_step_m,
            duration_s=args.jog_duration_sec,
            max_step_m=args.max_jog_step_m,
            timeout_s=args.jog_timeout_s,
            strategy=args.jog_strategy,
            joint_jog=joint_jog,
        )

    web: HandEyeWebInterface | None = None
    if args.web_port:
        web = HandEyeWebInterface(args.web_host, args.web_port)
        web.start()

    obs = LatestObservation()
    sample_count, max_sample_index = existing_sample_counts(args.output_dir / "samples.jsonl")
    next_sample_index = max_sample_index + 1
    last_action = "Ready. Move to a new visible-board pose and press Sample."

    print("SO101 wrist vision hand-eye capture")
    print(f"camera: {args.device} -> {args.camera_frame}")
    print(f"TF sample: {args.base_frame} <- {args.gripper_frame}")
    print(f"output: {args.output_dir / 'samples.jsonl'}")
    if expected_size != (args.width, args.height):
        print(f"WARN camera_info is {expected_size[0]}x{expected_size[1]}, capture requested {args.width}x{args.height}")
    print("Use the browser buttons or terminal commands. Press q/quit when done.")

    try:
        while True:
            ok, frame = cap.read()
            if ok and frame is not None:
                update_observation(
                    obs,
                    frame,
                    dictionary=dictionary,
                    obj_by_id=obj_by_id,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    min_markers=args.min_markers,
                    max_sample_reproj_px=args.max_sample_reproj_px,
                    camera_frame=args.camera_frame,
                )
                if web is not None and obs.overlay is not None:
                    web.update_image(obs.overlay)

            command_text = web.get_command_nowait() if web is not None else None
            if command_text is None:
                command_text = read_terminal_command()
            if command_text:
                command_text = command_text.strip()
                try:
                    parse_text = "sample" if command_text.lower().startswith("sample ") else command_text
                    command = parse_jog_command(parse_text)
                    if command.kind == "quit":
                        break
                    if command.kind == "sample":
                        note = ""
                        parts = command_text.split(maxsplit=1)
                        if len(parts) == 2:
                            note = parts[1]
                        row = save_sample(
                            args=args,
                            obs=obs,
                            tf_buffer=tf_buffer,
                            joint_jog=joint_jog,
                            sample_index=next_sample_index,
                            note=note,
                        )
                        sample_count += 1
                        next_sample_index += 1
                        last_action = (
                            f"sampled {row['name']}: markers={row['quality']['detected_markers']} "
                            f"err={row['quality']['mean_reprojection_error_px']:.2f}px"
                        )
                        print(last_action)
                    elif command.kind == "pose":
                        pose = wait_for_tool_pose(tf_buffer, args.base_frame, args.gripper_frame, args.tf_timeout_s)
                        quat = matrix_to_quat_xyzw(pose[:3, :3])
                        last_action = f"pose xyz={pose[:3, 3].round(4).tolist()} qxyzw={[round(v, 5) for v in quat]}"
                        print(last_action)
                    elif command.kind == "sync":
                        if joint_jog is None:
                            last_action = "joint jog is not enabled"
                        else:
                            last_action = joint_jog.sync_to_measured()
                        print(last_action)
                    elif command.kind == "step":
                        assert command.value is not None
                        if jog_session is None:
                            last_action = "Cartesian jog is not enabled"
                        elif command.value > args.max_jog_step_m:
                            last_action = f"max jog step is {args.max_jog_step_m * 1000.0:.1f} mm"
                        else:
                            jog_session.step_m = command.value
                            last_action = f"cartesian step set to {command.value * 1000.0:.1f} mm"
                    elif command.kind == "duration":
                        assert command.value is not None
                        if jog_session is not None:
                            jog_session.duration_s = command.value
                        if joint_jog is not None:
                            joint_jog.duration_s = command.value
                        last_action = f"duration set to {command.value:.2f} s"
                    elif command.kind == "joint_step":
                        assert command.value is not None
                        if joint_jog is None:
                            last_action = "joint jog is not enabled"
                        else:
                            joint_jog.joint_step_rad = command.value
                            last_action = f"joint step set to {math.degrees(command.value):.1f} deg"
                    elif command.kind == "joint_move":
                        if joint_jog is None:
                            last_action = "joint jog is not enabled"
                        else:
                            assert command.joint_name is not None
                            assert command.joint_sign is not None
                            joint_jog.move(command.joint_name, command.joint_sign, step_rad=command.value)
                            last_action = f"moved {command.joint_name}"
                    elif command.kind == "move":
                        if jog_session is None:
                            last_action = "Cartesian jog is not enabled"
                        else:
                            assert command.delta_axis is not None
                            step_m = jog_session.step_m if command.value is None else command.value
                            if step_m > args.max_jog_step_m:
                                last_action = f"max jog step is {args.max_jog_step_m * 1000.0:.1f} mm"
                            else:
                                jog_session.move(command.delta_axis * step_m)
                                last_action = "moved Cartesian jog"
                    elif command.kind == "help":
                        last_action = "Use browser buttons or terminal jog/sample commands."
                    else:
                        last_action = f"unsupported command: {command_text}"
                except Exception as exc:
                    last_action = f"ERROR: {exc}"
                    print(last_action)

            cart_step = jog_session.step_m if jog_session is not None else args.jog_step_m
            joint_step = joint_jog.joint_step_rad if joint_jog is not None else args.joint_step_rad
            duration = jog_session.duration_s if jog_session is not None else args.jog_duration_sec
            status = status_text(args, obs, sample_count, last_action)
            if web is not None:
                web.set_status(status, cart_step_m=cart_step, joint_step_rad=joint_step, duration_s=duration)
            time.sleep(0.02)
    finally:
        write_summary(args, sample_count)
        if web is not None:
            web.stop()
        cap.release()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    print(f"captured {sample_count} samples")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
