# Copyright 2026 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 service node for prompt-driven grasp proposal generation."""

from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any

import msgpack
import numpy as np
import rclpy
import zmq
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from so101_grasp_msgs.msg import GraspCandidate
from so101_grasp_msgs.srv import DetectGrasps


def _transform_to_matrix(transform_msg) -> np.ndarray:
    transform = transform_msg.transform
    quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    matrix = quaternion_matrix(quat).astype(np.float32)
    matrix[0, 3] = float(transform.translation.x)
    matrix[1, 3] = float(transform.translation.y)
    matrix[2, 3] = float(transform.translation.z)
    return matrix


def _camera_info_to_matrix(camera_info: CameraInfo) -> np.ndarray:
    return np.asarray(camera_info.k, dtype=np.float32).reshape(3, 3)


def _intrinsics_valid(matrix: np.ndarray) -> bool:
    return bool(
        matrix.shape == (3, 3)
        and np.isfinite(matrix).all()
        and matrix[0, 0] > 0.0
        and matrix[1, 1] > 0.0
        and matrix[2, 2] > 0.0
    )


def _make_fallback_intrinsics(width: int, height: int, fov_degrees: float) -> np.ndarray:
    fov_rad = math.radians(float(fov_degrees))
    focal = 0.5 * float(max(width, height)) / math.tan(max(1e-3, 0.5 * fov_rad))
    return np.asarray(
        [
            [focal, 0.0, 0.5 * (width - 1)],
            [0.0, focal, 0.5 * (height - 1)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )


def _compressed_image_size(msg: CompressedImage) -> tuple[int, int]:
    data = memoryview(msg.data)
    if len(data) >= 24 and data[:8].tobytes() == b"\x89PNG\r\n\x1a\n":
        width = struct.unpack(">I", data[16:20])[0]
        height = struct.unpack(">I", data[20:24])[0]
        return int(width), int(height)

    if len(data) >= 4 and data[0] == 0xFF and data[1] == 0xD8:
        offset = 2
        while offset + 9 < len(data):
            if data[offset] != 0xFF:
                offset += 1
                continue
            marker = data[offset + 1]
            offset += 2
            while marker == 0xFF and offset < len(data):
                marker = data[offset]
                offset += 1
            if marker in {0xD8, 0xD9}:
                continue
            if offset + 2 > len(data):
                break
            segment_len = struct.unpack(">H", data[offset : offset + 2])[0]
            if segment_len < 2 or offset + segment_len > len(data):
                break
            if marker in {
                0xC0,
                0xC1,
                0xC2,
                0xC3,
                0xC5,
                0xC6,
                0xC7,
                0xC9,
                0xCA,
                0xCB,
                0xCD,
                0xCE,
                0xCF,
            }:
                if offset + 7 > len(data):
                    break
                height = struct.unpack(">H", data[offset + 3 : offset + 5])[0]
                width = struct.unpack(">H", data[offset + 5 : offset + 7])[0]
                return int(width), int(height)
            offset += segment_len

    raise RuntimeError("Unsupported compressed image format for fallback intrinsics")


def _encode_packet(
    message_type: str,
    scalars: dict[str, Any] | None = None,
    arrays: dict[str, np.ndarray] | None = None,
    blobs: dict[str, bytes] | None = None,
) -> bytes:
    header: dict[str, Any] = {
        "type": message_type,
        "scalars": scalars or {},
        "arrays": [],
        "blobs": [],
    }
    body_parts: list[bytes] = []

    for name, value in (arrays or {}).items():
        array = np.ascontiguousarray(np.asarray(value))
        body = array.tobytes()
        header["arrays"].append(
            {
                "name": name,
                "shape": list(array.shape),
                "dtype": str(array.dtype),
                "nbytes": len(body),
            }
        )
        body_parts.append(body)

    for name, blob in (blobs or {}).items():
        blob_bytes = bytes(blob)
        header["blobs"].append({"name": name, "nbytes": len(blob_bytes)})
        body_parts.append(blob_bytes)

    header_bytes = msgpack.packb(header, use_bin_type=True)
    return struct.pack("<I", len(header_bytes)) + header_bytes + b"".join(body_parts)


def _decode_packet(data: bytes) -> dict[str, Any]:
    header_len = struct.unpack("<I", data[:4])[0]
    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = memoryview(data)[4 + header_len :]
    offset = 0

    arrays: dict[str, np.ndarray] = {}
    for meta in header.get("arrays", []):
        dtype = np.dtype(meta["dtype"])
        nbytes = int(meta["nbytes"])
        shape = tuple(meta["shape"])
        array = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape).copy()
        arrays[meta["name"]] = array
        offset += nbytes

    blobs: dict[str, bytes] = {}
    for meta in header.get("blobs", []):
        nbytes = int(meta["nbytes"])
        blobs[meta["name"]] = bytes(body[offset : offset + nbytes])
        offset += nbytes

    return {
        "type": header["type"],
        "scalars": dict(header.get("scalars", {})),
        "arrays": arrays,
        "blobs": blobs,
    }


@dataclass
class ZmqClientConfig:
    host: str
    port: int
    recv_timeout_ms: int
    send_timeout_ms: int


class ZmqGraspClient:
    """Simple request-response client for the remote grasp server."""

    def __init__(self, config: ZmqClientConfig, logger) -> None:
        self._config = config
        self._logger = logger
        self._ctx: zmq.Context | None = None
        self._req: zmq.Socket | None = None
        self._lock = threading.Lock()
        self._connect()

    def _connect(self) -> None:
        if self._req is not None:
            self._req.close(linger=0)
        if self._ctx is None:
            self._ctx = zmq.Context()
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.RCVTIMEO, self._config.recv_timeout_ms)
        self._req.setsockopt(zmq.SNDTIMEO, self._config.send_timeout_ms)
        self._req.connect(f"tcp://{self._config.host}:{self._config.port}")
        self._logger.info(f"Connected grasp client to {self._config.host}:{self._config.port}")

    def close(self) -> None:
        if self._req is not None:
            self._req.close(linger=0)
            self._req = None
        if self._ctx is not None:
            self._ctx.term()
            self._ctx = None

    def detect_grasps(
        self,
        *,
        prompt: str,
        top_k: int,
        overhead_jpeg: bytes,
        wrist_jpeg: bytes,
        overhead_intrinsics: np.ndarray,
        wrist_intrinsics: np.ndarray,
        overhead_camera_to_base: np.ndarray,
        wrist_camera_to_base: np.ndarray,
    ) -> dict[str, Any]:
        payload = _encode_packet(
            "detect_grasps",
            scalars={"prompt": prompt, "top_k": int(top_k)},
            arrays={
                "overhead_intrinsics": overhead_intrinsics.astype(np.float32, copy=False),
                "wrist_intrinsics": wrist_intrinsics.astype(np.float32, copy=False),
                "overhead_camera_to_base": overhead_camera_to_base.astype(np.float32, copy=False),
                "wrist_camera_to_base": wrist_camera_to_base.astype(np.float32, copy=False),
            },
            blobs={"overhead_jpeg": overhead_jpeg, "wrist_jpeg": wrist_jpeg},
        )

        with self._lock:
            try:
                assert self._req is not None
                self._req.send(payload)
                reply = self._req.recv()
            except zmq.ZMQError as exc:
                self._logger.error(f"Grasp server transport error: {exc}")
                self._connect()
                raise RuntimeError(f"Grasp server transport error: {exc}") from exc

        message = _decode_packet(reply)
        if message["type"] != "detect_grasps_response":
            raise RuntimeError(f"Unexpected reply type: {message['type']}")
        return message


class GraspRequestNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_request_node")

        self.declare_parameter("server_address", "127.0.0.1:8091")
        self.declare_parameter("recv_timeout_ms", 20000)
        self.declare_parameter("send_timeout_ms", 2000)
        self.declare_parameter("default_top_k", 10)
        self.declare_parameter("max_data_age_s", 0.5)
        self.declare_parameter("base_frame", "follower/base_link")
        self.declare_parameter("overhead_camera_frame", "follower/static_camera_optical_frame")
        self.declare_parameter("wrist_camera_frame", "follower/wrist_camera_optical_frame")
        self.declare_parameter("overhead_image_topic", "/static_camera/image_raw/compressed")
        self.declare_parameter("wrist_image_topic", "/follower/image_raw/compressed")
        self.declare_parameter("overhead_camera_info_topic", "/static_camera/camera_info")
        self.declare_parameter("wrist_camera_info_topic", "/follower/camera_info")
        self.declare_parameter("fallback_fov_degrees", 60.0)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._overhead_camera_frame = str(self.get_parameter("overhead_camera_frame").value)
        self._wrist_camera_frame = str(self.get_parameter("wrist_camera_frame").value)
        self._default_top_k = int(self.get_parameter("default_top_k").value)
        self._max_data_age_s = float(self.get_parameter("max_data_age_s").value)
        self._fallback_fov_degrees = float(self.get_parameter("fallback_fov_degrees").value)

        server_address = str(self.get_parameter("server_address").value)
        host, port = server_address.rsplit(":", 1)
        self._client = ZmqGraspClient(
            ZmqClientConfig(
                host=host,
                port=int(port),
                recv_timeout_ms=int(self.get_parameter("recv_timeout_ms").value),
                send_timeout_ms=int(self.get_parameter("send_timeout_ms").value),
            ),
            self.get_logger(),
        )

        self._latest_overhead_image: CompressedImage | None = None
        self._latest_wrist_image: CompressedImage | None = None
        self._latest_overhead_info: CameraInfo | None = None
        self._latest_wrist_info: CameraInfo | None = None
        self._latest_overhead_receipt: float | None = None
        self._latest_wrist_receipt: float | None = None
        self._warned_overhead_fallback_intrinsics = False
        self._warned_wrist_fallback_intrinsics = False

        self._state_lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("overhead_image_topic").value),
            self._on_overhead_image,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("wrist_image_topic").value),
            self._on_wrist_image,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("overhead_camera_info_topic").value),
            self._on_overhead_info,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("wrist_camera_info_topic").value),
            self._on_wrist_info,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )

        self._cloud_pub = self.create_publisher(PointCloud2, "so101_grasping/object_cloud", 1)
        self._overhead_cloud_pub = self.create_publisher(PointCloud2, "so101_grasping/overhead_object_cloud", 1)
        self._wrist_cloud_pub = self.create_publisher(PointCloud2, "so101_grasping/wrist_object_cloud", 1)
        self._marker_pub = self.create_publisher(MarkerArray, "so101_grasping/grasp_markers", 1)
        self._overhead_mask_pub = self.create_publisher(CompressedImage, "so101_grasping/overhead_mask/compressed", 1)
        self._wrist_mask_pub = self.create_publisher(CompressedImage, "so101_grasping/wrist_mask/compressed", 1)
        self._overhead_depth_pub = self.create_publisher(CompressedImage, "so101_grasping/overhead_depth/compressed", 1)
        self._wrist_depth_pub = self.create_publisher(CompressedImage, "so101_grasping/wrist_depth/compressed", 1)
        self._overhead_masked_depth_pub = self.create_publisher(
            CompressedImage, "so101_grasping/overhead_masked_depth/compressed", 1
        )
        self._wrist_masked_depth_pub = self.create_publisher(
            CompressedImage, "so101_grasping/wrist_masked_depth/compressed", 1
        )
        self._ggcnn_quality_pub = self.create_publisher(CompressedImage, "so101_grasping/ggcnn_quality/compressed", 1)
        self._ggcnn_angle_pub = self.create_publisher(CompressedImage, "so101_grasping/ggcnn_angle/compressed", 1)
        self._ggcnn_overlay_pub = self.create_publisher(CompressedImage, "so101_grasping/ggcnn_overlay/compressed", 1)
        self._ggcnn_depth_input_pub = self.create_publisher(
            CompressedImage, "so101_grasping/ggcnn_depth_input/compressed", 1
        )
        self.create_service(
            DetectGrasps,
            "detect_grasps",
            self._on_detect_grasps,
            callback_group=self._cb_group,
        )

        self.get_logger().info("grasp_request_node ready")

    def destroy_node(self) -> bool:
        self._client.close()
        return super().destroy_node()

    def _on_overhead_image(self, msg: CompressedImage) -> None:
        with self._state_lock:
            self._latest_overhead_image = msg
            self._latest_overhead_receipt = time.monotonic()

    def _on_wrist_image(self, msg: CompressedImage) -> None:
        with self._state_lock:
            self._latest_wrist_image = msg
            self._latest_wrist_receipt = time.monotonic()

    def _on_overhead_info(self, msg: CameraInfo) -> None:
        with self._state_lock:
            self._latest_overhead_info = msg

    def _on_wrist_info(self, msg: CameraInfo) -> None:
        with self._state_lock:
            self._latest_wrist_info = msg

    def _snapshot_inputs(self) -> tuple[CompressedImage, CompressedImage, CameraInfo | None, CameraInfo | None]:
        with self._state_lock:
            overhead_image = self._latest_overhead_image
            wrist_image = self._latest_wrist_image
            overhead_info = self._latest_overhead_info
            wrist_info = self._latest_wrist_info
            overhead_age = None if self._latest_overhead_receipt is None else time.monotonic() - self._latest_overhead_receipt
            wrist_age = None if self._latest_wrist_receipt is None else time.monotonic() - self._latest_wrist_receipt

        if overhead_image is None or wrist_image is None:
            raise RuntimeError("Missing camera image data")
        if overhead_age is None or overhead_age > self._max_data_age_s:
            raise RuntimeError(f"Overhead image is stale ({overhead_age})")
        if wrist_age is None or wrist_age > self._max_data_age_s:
            raise RuntimeError(f"Wrist image is stale ({wrist_age})")

        return overhead_image, wrist_image, overhead_info, wrist_info

    def _resolve_intrinsics(
        self,
        *,
        camera_name: str,
        image_msg: CompressedImage,
        camera_info: CameraInfo | None,
    ) -> np.ndarray:
        if camera_info is not None:
            intrinsics = _camera_info_to_matrix(camera_info)
            if _intrinsics_valid(intrinsics):
                return intrinsics

        width = int(camera_info.width) if camera_info is not None and camera_info.width > 0 else 0
        height = int(camera_info.height) if camera_info is not None and camera_info.height > 0 else 0
        if width <= 0 or height <= 0:
            width, height = _compressed_image_size(image_msg)

        intrinsics = _make_fallback_intrinsics(width, height, self._fallback_fov_degrees)
        warn_attr = f"_warned_{camera_name}_fallback_intrinsics"
        if not getattr(self, warn_attr):
            self.get_logger().warning(
                f"Using fallback intrinsics for {camera_name} camera: "
                f"{width}x{height}, assumed FOV {self._fallback_fov_degrees:.1f} deg"
            )
            setattr(self, warn_attr, True)
        return intrinsics

    def _lookup_camera_to_base(self, camera_frame: str, stamp_msg) -> np.ndarray:
        stamp = Time.from_msg(stamp_msg)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                camera_frame,
                stamp,
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                camera_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        return _transform_to_matrix(transform)

    def _publish_cloud(self, publisher, points: np.ndarray, stamp_msg) -> None:
        header = Header(frame_id=self._base_frame, stamp=stamp_msg)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(header, fields, np.asarray(points, dtype=np.float32))
        publisher.publish(cloud)

    def _publish_markers(self, grasps: list[GraspCandidate]) -> None:
        markers = MarkerArray()
        for idx, grasp in enumerate(grasps):
            marker = Marker()
            marker.header = grasp.header
            marker.ns = "so101_grasping"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = grasp.pose
            marker.scale.x = 0.08
            marker.scale.y = max(0.004, float(grasp.width) * 0.5)
            marker.scale.z = 0.008
            score = max(0.0, min(1.0, float(grasp.score)))
            marker.color.r = 1.0 - score
            marker.color.g = score
            marker.color.b = 0.2
            marker.color.a = 0.95
            markers.markers.append(marker)
        self._marker_pub.publish(markers)

    def _publish_mask(self, publisher, data: bytes, stamp_msg) -> None:
        if not data:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self._base_frame
        msg.format = "png"
        msg.data = data
        publisher.publish(msg)

    def _build_grasp_candidates(
        self,
        positions: np.ndarray,
        quaternions: np.ndarray,
        scores: np.ndarray,
        widths: np.ndarray,
        sources: list[str],
        stamp_msg,
    ) -> list[GraspCandidate]:
        grasps: list[GraspCandidate] = []
        for idx in range(positions.shape[0]):
            candidate = GraspCandidate()
            candidate.header.frame_id = self._base_frame
            candidate.header.stamp = stamp_msg
            candidate.pose = Pose()
            candidate.pose.position.x = float(positions[idx, 0])
            candidate.pose.position.y = float(positions[idx, 1])
            candidate.pose.position.z = float(positions[idx, 2])
            candidate.pose.orientation.x = float(quaternions[idx, 0])
            candidate.pose.orientation.y = float(quaternions[idx, 1])
            candidate.pose.orientation.z = float(quaternions[idx, 2])
            candidate.pose.orientation.w = float(quaternions[idx, 3])
            candidate.score = float(scores[idx])
            candidate.width = float(widths[idx])
            candidate.source = sources[idx] if idx < len(sources) else "graspnet"
            grasps.append(candidate)
        return grasps

    def _on_detect_grasps(self, request: DetectGrasps.Request, response: DetectGrasps.Response):
        prompt = request.prompt.strip()
        if not prompt:
            response.success = False
            response.message = "Prompt must not be empty"
            return response

        top_k = int(request.top_k) if int(request.top_k) > 0 else self._default_top_k

        try:
            overhead_image, wrist_image, overhead_info, wrist_info = self._snapshot_inputs()
            overhead_camera_to_base = self._lookup_camera_to_base(
                self._overhead_camera_frame, overhead_image.header.stamp
            )
            wrist_camera_to_base = self._lookup_camera_to_base(
                self._wrist_camera_frame, wrist_image.header.stamp
            )
            result = self._client.detect_grasps(
                prompt=prompt,
                top_k=top_k,
                overhead_jpeg=bytes(overhead_image.data),
                wrist_jpeg=bytes(wrist_image.data),
                overhead_intrinsics=self._resolve_intrinsics(
                    camera_name="overhead",
                    image_msg=overhead_image,
                    camera_info=overhead_info,
                ),
                wrist_intrinsics=self._resolve_intrinsics(
                    camera_name="wrist",
                    image_msg=wrist_image,
                    camera_info=wrist_info,
                ),
                overhead_camera_to_base=overhead_camera_to_base,
                wrist_camera_to_base=wrist_camera_to_base,
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"detect_grasps failed: {exc}")
            return response

        scalars = result["scalars"]
        success = bool(scalars.get("success", False))
        response.success = success
        response.message = str(scalars.get("message", ""))
        depth_modes = scalars.get("depth_modes", {})
        if depth_modes:
            depth_summary = ", ".join(f"{name}={mode}" for name, mode in dict(depth_modes).items())
            response.message = f"{response.message}; depth_modes: {depth_summary}"
            self.get_logger().info(f"Depth modes from grasp server: {depth_summary}")

        arrays = result["arrays"]
        positions = arrays.get("grasp_positions", np.zeros((0, 3), dtype=np.float32))
        quaternions = arrays.get("grasp_quaternions", np.zeros((0, 4), dtype=np.float32))
        scores = arrays.get("grasp_scores", np.zeros((0,), dtype=np.float32))
        widths = arrays.get("grasp_widths", np.zeros((0,), dtype=np.float32))
        sources = list(scalars.get("grasp_sources", ["graspnet"] * len(scores)))

        grasps: list[GraspCandidate] = []
        if success:
            grasps = self._build_grasp_candidates(
                positions=positions,
                quaternions=quaternions,
                scores=scores,
                widths=widths,
                sources=sources,
                stamp_msg=self.get_clock().now().to_msg(),
            )
        response.grasps = grasps

        stamp_msg = self.get_clock().now().to_msg()
        overhead_object_cloud = arrays.get("overhead_object_cloud", np.zeros((0, 3), dtype=np.float32))
        wrist_object_cloud = arrays.get("wrist_object_cloud", np.zeros((0, 3), dtype=np.float32))
        object_cloud = arrays.get("object_cloud", np.zeros((0, 3), dtype=np.float32))
        self._publish_cloud(self._overhead_cloud_pub, overhead_object_cloud, stamp_msg)
        self._publish_cloud(self._wrist_cloud_pub, wrist_object_cloud, stamp_msg)
        self._publish_cloud(self._cloud_pub, object_cloud, stamp_msg)
        self._publish_markers(grasps)
        self._publish_mask(self._overhead_mask_pub, result["blobs"].get("overhead_mask_png", b""), stamp_msg)
        self._publish_mask(self._wrist_mask_pub, result["blobs"].get("wrist_mask_png", b""), stamp_msg)
        self._publish_mask(self._overhead_depth_pub, result["blobs"].get("overhead_depth_png", b""), stamp_msg)
        self._publish_mask(self._wrist_depth_pub, result["blobs"].get("wrist_depth_png", b""), stamp_msg)
        self._publish_mask(
            self._overhead_masked_depth_pub,
            result["blobs"].get("overhead_masked_depth_png", b""),
            stamp_msg,
        )
        self._publish_mask(
            self._wrist_masked_depth_pub,
            result["blobs"].get("wrist_masked_depth_png", b""),
            stamp_msg,
        )
        self._publish_mask(self._ggcnn_quality_pub, result["blobs"].get("ggcnn_quality_png", b""), stamp_msg)
        self._publish_mask(self._ggcnn_angle_pub, result["blobs"].get("ggcnn_angle_png", b""), stamp_msg)
        self._publish_mask(self._ggcnn_overlay_pub, result["blobs"].get("ggcnn_overlay_png", b""), stamp_msg)
        self._publish_mask(
            self._ggcnn_depth_input_pub,
            result["blobs"].get("ggcnn_depth_input_png", b""),
            stamp_msg,
        )
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspRequestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
