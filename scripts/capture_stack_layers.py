#!/usr/bin/env python3
"""Capture one SO-101 grasp stack pass from live ROS topics."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2
from so101_grasp_msgs.srv import DetectGrasps
from tf2_ros import Buffer, TransformListener


IMAGE_TOPICS = {
    "input_overhead.jpg": "/static_camera/image_raw/compressed",
    "input_wrist.jpg": "/follower/image_raw/compressed",
    "overhead_mask.png": "/so101_grasping/overhead_mask/compressed",
    "wrist_mask.png": "/so101_grasping/wrist_mask/compressed",
    "overhead_depth.png": "/so101_grasping/overhead_depth/compressed",
    "wrist_depth.png": "/so101_grasping/wrist_depth/compressed",
    "overhead_masked_depth.png": "/so101_grasping/overhead_masked_depth/compressed",
    "wrist_masked_depth.png": "/so101_grasping/wrist_masked_depth/compressed",
    "ggcnn_quality.png": "/so101_grasping/ggcnn_quality/compressed",
    "ggcnn_angle.png": "/so101_grasping/ggcnn_angle/compressed",
    "ggcnn_overlay.png": "/so101_grasping/ggcnn_overlay/compressed",
    "ggcnn_depth_input.png": "/so101_grasping/ggcnn_depth_input/compressed",
}

CLOUD_TOPICS = {
    "object_cloud": "/so101_grasping/object_cloud",
    "overhead_object_cloud": "/so101_grasping/overhead_object_cloud",
    "wrist_object_cloud": "/so101_grasping/wrist_object_cloud",
}


class StackLayerCapture(Node):
    def __init__(self) -> None:
        super().__init__("so101_stack_layers_capture")
        self.images: dict[str, bytes] = {}
        self.clouds: dict[str, np.ndarray] = {}
        self.client = self.create_client(DetectGrasps, "/detect_grasps")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        for name, topic in IMAGE_TOPICS.items():
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, name=name: self._on_image(name, msg),
                qos_profile_sensor_data,
            )

        for name, topic in CLOUD_TOPICS.items():
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, name=name: self._on_cloud(name, msg),
                qos_profile_sensor_data,
            )

    def _on_image(self, name: str, msg: CompressedImage) -> None:
        self.images[name] = bytes(msg.data)

    def _on_cloud(self, name: str, msg: PointCloud2) -> None:
        points = [
            [float(point[0]), float(point[1]), float(point[2])]
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        self.clouds[name] = np.asarray(points, dtype=np.float32)

    def lookup_pose(self, target_frame: str) -> dict[str, object]:
        transform = self.tf_buffer.lookup_transform(
            "follower/base_link",
            target_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "frame_id": "follower/base_link",
            "child_frame_id": target_frame,
            "position": [float(translation.x), float(translation.y), float(translation.z)],
            "orientation_xyzw": [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)],
        }


def _wait_for_live_inputs(node: StackLayerCapture, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "input_overhead.jpg" in node.images and "input_wrist.jpg" in node.images:
            return
    raise RuntimeError("Did not receive both live camera inputs")


def _call_grasp_service(node: StackLayerCapture, prompt: str, top_k: int, timeout_s: float):
    if not node.client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("/detect_grasps service is not available")

    request = DetectGrasps.Request()
    request.prompt = prompt
    request.top_k = top_k
    future = node.client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise TimeoutError("/detect_grasps timed out")
    return future.result()


def _collect_debug_outputs(node: StackLayerCapture, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def _write_outputs(node: StackLayerCapture, response, out_dir: Path, prompt: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, data in node.images.items():
        (out_dir / name).write_bytes(data)

    np.savez_compressed(out_dir / "clouds.npz", **node.clouds)

    grasps = []
    for index, grasp in enumerate(response.grasps):
        grasps.append(
            {
                "index": index,
                "score": float(grasp.score),
                "width": float(grasp.width),
                "source": str(grasp.source),
                "position": [
                    float(grasp.pose.position.x),
                    float(grasp.pose.position.y),
                    float(grasp.pose.position.z),
                ],
                "orientation_xyzw": [
                    float(grasp.pose.orientation.x),
                    float(grasp.pose.orientation.y),
                    float(grasp.pose.orientation.z),
                    float(grasp.pose.orientation.w),
                ],
            }
        )

    poses: dict[str, object] = {}
    for frame in (
        "follower/gripper_frame_link",
        "follower/wrist_camera_optical_frame",
        "follower/static_camera_optical_frame",
    ):
        try:
            poses[frame] = node.lookup_pose(frame)
        except Exception as exc:  # noqa: BLE001 - saved as debug metadata
            poses[frame] = {"error": str(exc)}

    metadata = {
        "prompt": prompt,
        "success": bool(response.success),
        "message": str(response.message),
        "num_grasps": len(response.grasps),
        "grasps": grasps,
        "poses": poses,
        "cloud_counts": {name: int(points.shape[0]) for name, points in node.clouds.items()},
        "images": sorted(node.images.keys()),
    }
    (out_dir / "snapshot.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    lines = [
        f"prompt: {metadata['prompt']}",
        f"success: {metadata['success']}",
        f"message: {metadata['message']}",
        f"num_grasps: {metadata['num_grasps']}",
    ]
    for name, count in metadata["cloud_counts"].items():
        lines.append(f"cloud[{name}]: {count}")
    for grasp in grasps:
        position = grasp["position"]
        quat = grasp["orientation_xyzw"]
        lines.append(
            f"grasp[{grasp['index']}]: score={grasp['score']:.4f} width={grasp['width']:.4f} "
            f"source={grasp['source']} pos=({position[0]:.4f},{position[1]:.4f},{position[2]:.4f}) "
            f"quat=({quat[0]:.4f},{quat[1]:.4f},{quat[2]:.4f},{quat[3]:.4f})"
        )
    lines.append("images: " + ", ".join(metadata["images"]))
    (out_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="/tmp/so101_stack_layers_live")
    parser.add_argument("--prompt", default="pink cube")
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--service-timeout-s", type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = StackLayerCapture()
    try:
        _wait_for_live_inputs(node, timeout_s=20.0)
        response = _call_grasp_service(node, args.prompt, args.top_k, args.service_timeout_s)
        _collect_debug_outputs(node, timeout_s=8.0)
        _write_outputs(node, response, Path(args.out_dir), args.prompt)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
