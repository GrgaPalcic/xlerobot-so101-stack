"""Small OpenCV camera publisher for grasping tryouts.

This avoids depending on camera drivers that are currently unstable on the Dell
host. It publishes only the compressed image and CameraInfo topics consumed by
grasp_request_node.
"""

from __future__ import annotations

import math
import os
import subprocess
import threading
import time
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
import yaml


def np_eye3() -> list[float]:
    return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def camera_matrix_to_p(k: list[float]) -> list[float]:
    return [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]


class FfmpegRawCapture:
    """Small ffmpeg-backed V4L2 reader for loopback devices OpenCV cannot read."""

    def __init__(self, device: str, width: int, height: int, fps: float, input_format: str) -> None:
        self._width = width
        self._height = height
        self._frame_size = width * height * 3
        self._lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._closed = False
        self._process = subprocess.Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "v4l2",
                "-framerate",
                str(max(1.0, fps)),
                "-video_size",
                f"{width}x{height}",
                "-input_format",
                input_format,
                "-i",
                device,
                "-an",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "pipe:1",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
        self._thread = threading.Thread(target=self._read_loop, name=f"ffmpeg-{device}", daemon=True)
        self._thread.start()

    def isOpened(self) -> bool:
        return self._process.poll() is None

    def read(self):
        if not self.isOpened():
            return False, None
        with self._lock:
            if self._latest is None:
                return False, None
            return True, self._latest.copy()

    def release(self) -> None:
        self._closed = True
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()

    def _read_loop(self) -> None:
        assert self._process.stdout is not None
        while not self._closed:
            chunks = bytearray()
            while len(chunks) < self._frame_size and not self._closed:
                chunk = self._process.stdout.read(self._frame_size - len(chunks))
                if not chunk:
                    break
                chunks.extend(chunk)
            if len(chunks) != self._frame_size:
                break
            frame = np.frombuffer(chunks, dtype=np.uint8).reshape((self._height, self._width, 3)).copy()
            with self._lock:
                self._latest = frame


class CompressedCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("compressed_camera_node")
        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("framerate", 30.0)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("capture_backend", "opencv")
        self.declare_parameter("ffmpeg_input_format", "yuyv422")
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("camera_name", self.get_name())
        self.declare_parameter("fallback_fov_degrees", 60.0)
        self.declare_parameter("fallback_video_device", "")
        self.declare_parameter("fallback_fourcc", "")
        self.declare_parameter("fallback_image_width", 0)
        self.declare_parameter("fallback_image_height", 0)
        self.declare_parameter("fallback_device_fov_degrees", 0.0)
        self.declare_parameter("camera_info_url", "")

        self._device = str(self.get_parameter("video_device").value)
        self._width = int(self.get_parameter("image_width").value)
        self._height = int(self.get_parameter("image_height").value)
        self._fps = float(self.get_parameter("framerate").value)
        self._fourcc = str(self.get_parameter("fourcc").value)
        self._capture_backend = str(self.get_parameter("capture_backend").value)
        self._ffmpeg_input_format = str(self.get_parameter("ffmpeg_input_format").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._camera_name = str(self.get_parameter("camera_name").value)
        self._fov_degrees = float(self.get_parameter("fallback_fov_degrees").value)
        self._fallback_device = str(self.get_parameter("fallback_video_device").value)
        self._fallback_fourcc = str(self.get_parameter("fallback_fourcc").value)
        self._fallback_width = int(self.get_parameter("fallback_image_width").value)
        self._fallback_height = int(self.get_parameter("fallback_image_height").value)
        self._fallback_fov_degrees = float(self.get_parameter("fallback_device_fov_degrees").value)
        self._camera_info_url = str(self.get_parameter("camera_info_url").value)
        self._calibration_info = self._load_camera_info(self._camera_info_url)
        self._last_warn_s = 0.0
        self._read_failures = 0
        self._using_fallback = False

        self._image_pub = self.create_publisher(CompressedImage, "image_raw/compressed", 2)
        self._info_pub = self.create_publisher(CameraInfo, "camera_info", 2)

        self._cap = self._open_capture(self._device, self._fourcc, self._width, self._height)

        period = 1.0 / max(1.0, self._fps)
        self._timer = self.create_timer(period, self._publish_frame)
        self.get_logger().info(
            f"Publishing {self._camera_name} from {self._device} at {self._width}x{self._height} "
            f"fourcc={self._fourcc or 'default'} fps={self._fps:.1f} backend={self._capture_backend}"
        )

    def _open_capture(self, device: str, fourcc_name: str, width: int, height: int):
        if self._capture_backend == "ffmpeg_raw":
            cap = FfmpegRawCapture(device, width, height, self._fps, self._ffmpeg_input_format)
            # Give ffmpeg a short window to open the V4L2 reader and deliver the first frame.
            deadline = time.monotonic() + 5.0
            while cap.isOpened() and time.monotonic() < deadline:
                ok, _ = cap.read()
                if ok:
                    break
                time.sleep(0.05)
            if not cap.isOpened():
                raise RuntimeError(f"Failed to open ffmpeg camera device {device}")
            self._device = device
            self._fourcc = fourcc_name
            self._width = width
            self._height = height
            self._camera_info = self._build_camera_info()
            return cap

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if fourcc_name:
            fourcc = cv2.VideoWriter_fourcc(*fourcc_name[:4])
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, self._fps)

        if not cap.isOpened():
            raise RuntimeError(f"Failed to open camera device {device}")

        self._device = device
        self._fourcc = fourcc_name
        self._width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or width
        self._height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or height
        self._camera_info = self._build_camera_info()
        return cap

    def _switch_to_fallback(self) -> bool:
        if self._using_fallback or not self._fallback_device:
            return False

        self.get_logger().warning(
            f"Switching {self._camera_name} from {self._device} to fallback camera {self._fallback_device}"
        )
        self._cap.release()
        if self._fallback_fov_degrees > 0.0:
            self._fov_degrees = self._fallback_fov_degrees
        width = self._fallback_width if self._fallback_width > 0 else self._width
        height = self._fallback_height if self._fallback_height > 0 else self._height
        fourcc = self._fallback_fourcc or self._fourcc
        self._cap = self._open_capture(self._fallback_device, fourcc, width, height)
        self._using_fallback = True
        self._read_failures = 0
        self.get_logger().info(
            f"Publishing {self._camera_name} from fallback {self._device} at {self._width}x{self._height} "
            f"fourcc={self._fourcc or 'default'} fps={self._fps:.1f} backend={self._capture_backend}"
        )
        return True

    def _build_camera_info(self) -> CameraInfo:
        if self._calibration_info is not None:
            return self._camera_info_from_yaml(self._calibration_info)

        info = CameraInfo()
        info.width = self._width
        info.height = self._height
        info.distortion_model = "plumb_bob"
        fov_rad = math.radians(self._fov_degrees)
        focal = 0.5 * float(self._width) / math.tan(0.5 * fov_rad)
        cx = 0.5 * float(self._width - 1)
        cy = 0.5 * float(self._height - 1)
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return info

    def _resolve_camera_info_path(self, url: str) -> str:
        if not url:
            return ""
        parsed = urlparse(url)
        if parsed.scheme == "file":
            return parsed.path
        if parsed.scheme == "package":
            package, _, relative = parsed.path.lstrip("/").partition("/")
            if not package or not relative:
                raise ValueError(f"Invalid package camera_info_url: {url}")
            return os.path.join(get_package_share_directory(package), relative)
        if parsed.scheme:
            raise ValueError(f"Unsupported camera_info_url scheme: {parsed.scheme}")
        return url

    def _load_camera_info(self, url: str) -> dict | None:
        if not url:
            return None
        path = self._resolve_camera_info_path(url)
        with open(path, "r") as stream:
            data = yaml.safe_load(stream) or {}
        self.get_logger().info(f"Loaded camera calibration for {self._camera_name} from {path}")
        return data

    def _camera_info_from_yaml(self, data: dict) -> CameraInfo:
        info = CameraInfo()
        info.width = int(data.get("image_width", self._width))
        info.height = int(data.get("image_height", self._height))
        if info.width != self._width or info.height != self._height:
            self.get_logger().warning(
                f"Calibration image size {info.width}x{info.height} does not match "
                f"capture size {self._width}x{self._height}"
            )

        info.distortion_model = str(data.get("distortion_model", "plumb_bob"))
        info.k = [float(v) for v in data["camera_matrix"]["data"]]
        info.d = [float(v) for v in data.get("distortion_coefficients", {}).get("data", [])]
        info.r = [float(v) for v in data.get("rectification_matrix", {}).get("data", np_eye3())]
        info.p = [float(v) for v in data.get("projection_matrix", {}).get("data", camera_matrix_to_p(info.k))]
        return info

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warn_s > 5.0:
            self.get_logger().warning(message)
            self._last_warn_s = now

    def _publish_frame(self) -> None:
        ok, frame_bgr = self._cap.read()
        if not ok or frame_bgr is None:
            self._read_failures += 1
            self._warn_throttled(f"Failed to read frame from {self._device}")
            if self._read_failures >= 1:
                self._switch_to_fallback()
            return
        self._read_failures = 0

        ok, encoded = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            self._warn_throttled("Failed to JPEG-encode camera frame")
            return

        stamp = self.get_clock().now().to_msg()
        image_msg = CompressedImage()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self._frame_id
        image_msg.format = "jpeg"
        image_msg.data = encoded.tobytes()

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self._frame_id
        info_msg.width = self._camera_info.width
        info_msg.height = self._camera_info.height
        info_msg.distortion_model = self._camera_info.distortion_model
        info_msg.d = list(self._camera_info.d)
        info_msg.k = list(self._camera_info.k)
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = list(self._camera_info.p)

        self._image_pub.publish(image_msg)
        self._info_pub.publish(info_msg)

    def destroy_node(self) -> bool:
        if hasattr(self, "_cap"):
            self._cap.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CompressedCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
