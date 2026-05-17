#!/usr/bin/env python3
"""Replay a captured SO-101 grasp stack snapshot into RViz-friendly topics."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


BASE_FRAME = "follower/base_link"


def _quat_to_matrix(quat_xyzw: list[float]) -> np.ndarray:
    x, y, z, w = quat_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-8:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _point(values: np.ndarray | list[float]) -> Point:
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2])
    return point


class StackSnapshotPublisher(Node):
    def __init__(self, snapshot_dir: Path) -> None:
        super().__init__("so101_stack_snapshot_publisher")
        self.snapshot_dir = snapshot_dir
        self.metadata = json.loads((snapshot_dir / "snapshot.json").read_text(encoding="utf-8"))
        clouds = np.load(snapshot_dir / "clouds.npz")
        self.clouds = {name: clouds[name].astype(np.float32, copy=False) for name in clouds.files}

        self.object_cloud_pub = self.create_publisher(PointCloud2, "/so101_grasping/snapshot/object_cloud", 1)
        self.overhead_cloud_pub = self.create_publisher(PointCloud2, "/so101_grasping/snapshot/overhead_object_cloud", 1)
        self.wrist_cloud_pub = self.create_publisher(PointCloud2, "/so101_grasping/snapshot/wrist_object_cloud", 1)
        self.grasp_markers_pub = self.create_publisher(MarkerArray, "/so101_grasping/snapshot/grasp_markers", 1)
        self.path_markers_pub = self.create_publisher(MarkerArray, "/so101_grasping/snapshot/path_preview", 1)
        self.default_markers_pub = self.create_publisher(MarkerArray, "/visualization_marker_array", 1)
        self.timer = self.create_timer(0.5, self.publish_snapshot)

    def _header(self) -> Header:
        header = Header()
        header.frame_id = BASE_FRAME
        header.stamp = self.get_clock().now().to_msg()
        return header

    def _cloud_msg(self, points: np.ndarray) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(self._header(), fields, np.asarray(points, dtype=np.float32))

    def publish_snapshot(self) -> None:
        if "object_cloud" in self.clouds:
            self.object_cloud_pub.publish(self._cloud_msg(self.clouds["object_cloud"]))
        if "overhead_object_cloud" in self.clouds:
            self.overhead_cloud_pub.publish(self._cloud_msg(self.clouds["overhead_object_cloud"]))
        if "wrist_object_cloud" in self.clouds:
            self.wrist_cloud_pub.publish(self._cloud_msg(self.clouds["wrist_object_cloud"]))

        grasp_markers = self._make_grasp_markers()
        path_markers = self._make_path_markers()
        self.grasp_markers_pub.publish(grasp_markers)
        self.path_markers_pub.publish(path_markers)

        combined = MarkerArray()
        combined.markers.extend(grasp_markers.markers)
        combined.markers.extend(path_markers.markers)
        self.default_markers_pub.publish(combined)

    def _make_grasp_markers(self) -> MarkerArray:
        markers = MarkerArray()
        header = self._header()
        for grasp in self.metadata.get("grasps", []):
            marker = Marker()
            marker.header = header
            marker.ns = "snapshot_grasps"
            marker.id = int(grasp["index"])
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position = _point(grasp["position"])
            quat = grasp["orientation_xyzw"]
            marker.pose.orientation.x = float(quat[0])
            marker.pose.orientation.y = float(quat[1])
            marker.pose.orientation.z = float(quat[2])
            marker.pose.orientation.w = float(quat[3])
            marker.scale.x = 0.08
            marker.scale.y = max(0.004, float(grasp["width"]) * 0.5)
            marker.scale.z = 0.008
            score = max(0.0, min(1.0, float(grasp["score"])))
            marker.color.r = 1.0 - score
            marker.color.g = score
            marker.color.b = 0.2
            marker.color.a = 0.95
            markers.markers.append(marker)
        return markers

    def _make_path_markers(self) -> MarkerArray:
        markers = MarkerArray()
        header = self._header()
        grasps = self.metadata.get("grasps", [])
        ee_pose = self.metadata.get("poses", {}).get("follower/gripper_frame_link", {})
        if not grasps or "position" not in ee_pose:
            return markers

        best = grasps[0]
        start = np.asarray(ee_pose["position"], dtype=np.float32)
        goal = np.asarray(best["position"], dtype=np.float32)
        approach_axis = _quat_to_matrix(best["orientation_xyzw"])[:, 0]
        pregrasp = goal - approach_axis * 0.10

        line = Marker()
        line.header = header
        line.ns = "snapshot_path_preview"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.012
        line.color.r = 0.0
        line.color.g = 0.85
        line.color.b = 1.0
        line.color.a = 0.95
        line.points = [_point(start), _point(pregrasp), _point(goal)]
        markers.markers.append(line)

        for marker_id, name, position, color in (
            (1, "current_ee", start, (0.1, 0.4, 1.0)),
            (2, "pregrasp", pregrasp, (1.0, 0.85, 0.1)),
            (3, "goal_grasp", goal, (0.1, 1.0, 0.25)),
        ):
            sphere = Marker()
            sphere.header = header
            sphere.ns = f"snapshot_path_{name}"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = _point(position)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.035
            sphere.scale.y = 0.035
            sphere.scale.z = 0.035
            sphere.color.r, sphere.color.g, sphere.color.b = color
            sphere.color.a = 0.9
            markers.markers.append(sphere)

        text = Marker()
        text.header = header
        text.ns = "snapshot_path_label"
        text.id = 4
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = _point(goal + np.asarray([0.0, 0.0, 0.08], dtype=np.float32))
        text.pose.orientation.w = 1.0
        text.scale.z = 0.035
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "path preview only: no IK/collision yet"
        markers.markers.append(text)
        return markers


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot-dir", default="/tmp/so101_stack_layers_live")
    parser.add_argument("--duration-s", type=float, default=3600.0)
    args = parser.parse_args()

    rclpy.init()
    node = StackSnapshotPublisher(Path(args.snapshot_dir))
    deadline = time.monotonic() + args.duration_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
