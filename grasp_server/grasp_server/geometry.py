"""Lightweight geometry helpers used by the grasp perception backend."""

from __future__ import annotations

import numpy as np


def resize_intrinsics(intrinsics: np.ndarray, src_hw: tuple[int, int], dst_hw: tuple[int, int]) -> np.ndarray:
    src_h, src_w = src_hw
    dst_h, dst_w = dst_hw
    scale_x = float(dst_w) / float(src_w)
    scale_y = float(dst_h) / float(src_h)
    scaled = np.asarray(intrinsics, dtype=np.float32).copy()
    scaled[0, 0] *= scale_x
    scaled[1, 1] *= scale_y
    scaled[0, 2] *= scale_x
    scaled[1, 2] *= scale_y
    return scaled


def backproject_masked_depth(
    depth_m: np.ndarray,
    intrinsics: np.ndarray,
    mask: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = (
        np.asarray(mask, dtype=bool)
        & np.isfinite(depth)
        & (depth >= min_depth_m)
        & (depth <= max_depth_m)
    )
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)

    rows, cols = np.nonzero(valid)
    z = depth[rows, cols]
    fx = float(intrinsics[0, 0])
    fy = float(intrinsics[1, 1])
    cx = float(intrinsics[0, 2])
    cy = float(intrinsics[1, 2])
    x = (cols.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    points_h = np.concatenate(
        [np.asarray(points, dtype=np.float32), np.ones((points.shape[0], 1), dtype=np.float32)],
        axis=1,
    )
    return (np.asarray(transform, dtype=np.float32) @ points_h.T).T[:, :3]


def invert_transform(transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, dtype=np.float32)
    if matrix.shape == (3, 4):
        padded = np.eye(4, dtype=np.float32)
        padded[:3, :4] = matrix
        matrix = padded
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected 4x4 or 3x4 transform, got {matrix.shape}")
    return np.linalg.inv(matrix).astype(np.float32)


def workspace_mask(points: np.ndarray, bounds: tuple[float, float, float, float, float, float]) -> np.ndarray:
    if points.size == 0:
        return np.zeros((0,), dtype=bool)
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    return (
        np.isfinite(points).all(axis=1)
        & (points[:, 0] >= xmin)
        & (points[:, 0] <= xmax)
        & (points[:, 1] >= ymin)
        & (points[:, 1] <= ymax)
        & (points[:, 2] >= zmin)
        & (points[:, 2] <= zmax)
    )


def crop_workspace(points: np.ndarray, bounds: tuple[float, float, float, float, float, float]) -> np.ndarray:
    if points.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(points[workspace_mask(points, bounds)], dtype=np.float32)
