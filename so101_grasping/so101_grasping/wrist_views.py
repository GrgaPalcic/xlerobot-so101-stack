"""Wrist camera view generation for supervised grasp refinement."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_matrix


@dataclass(frozen=True)
class WristViewCandidate:
    label: str
    standoff_m: float
    lateral_offset_m: float
    lateral_axis: str
    camera_position: np.ndarray
    ee_position: np.ndarray
    ee_quat_xyzw: np.ndarray
    target_in_camera: np.ndarray


def _normalize(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if norm < 1e-9 or not np.isfinite(norm):
        return np.asarray(fallback, dtype=np.float64)
    return values / norm


def upward_view_axis(plane_normal: np.ndarray | None = None) -> np.ndarray:
    """Return an arm-base view axis that points away from the support surface."""

    fallback = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    if plane_normal is None:
        return fallback
    normal = _normalize(np.asarray(plane_normal, dtype=np.float64), fallback)
    if float(normal @ fallback) < 0.0:
        normal = -normal
    if abs(float(normal @ fallback)) < 0.20:
        return fallback
    return normal


def tangent_basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    axis = _normalize(axis, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
    first = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    if abs(float(first @ axis)) > 0.90:
        first = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    first = _normalize(first - axis * float(first @ axis), np.asarray([1.0, 0.0, 0.0], dtype=np.float64))
    second = _normalize(np.cross(axis, first), np.asarray([1.0, 0.0, 0.0], dtype=np.float64))
    return first, second


def _offset_options(lateral_offsets_m: list[float]) -> list[tuple[str, float, np.ndarray]]:
    offsets: list[tuple[str, float, np.ndarray]] = [("center", 0.0, np.zeros(2, dtype=np.float64))]
    seen = {0.0}
    for raw_offset in lateral_offsets_m:
        offset = abs(float(raw_offset))
        if offset < 1e-6 or offset in seen:
            continue
        seen.add(offset)
        mm = int(round(offset * 1000.0))
        offsets.extend(
            [
                (f"u+{mm}mm", offset, np.asarray([offset, 0.0], dtype=np.float64)),
                (f"u-{mm}mm", offset, np.asarray([-offset, 0.0], dtype=np.float64)),
                (f"v+{mm}mm", offset, np.asarray([0.0, offset], dtype=np.float64)),
                (f"v-{mm}mm", offset, np.asarray([0.0, -offset], dtype=np.float64)),
            ]
        )
    return offsets


def wrist_view_candidates(
    *,
    target: np.ndarray,
    wrist_camera_xyz_in_ee: np.ndarray,
    wrist_camera_quat_xyzw_in_ee: np.ndarray,
    standoffs_m: list[float],
    lateral_offsets_m: list[float],
    min_ee_z_m: float,
    plane_normal: np.ndarray | None = None,
) -> list[WristViewCandidate]:
    target = np.asarray(target, dtype=np.float64)
    axis = upward_view_axis(plane_normal)
    tangent_u, tangent_v = tangent_basis(axis)
    ee_to_camera = quaternion_matrix(wrist_camera_quat_xyzw_in_ee)[:3, :3]
    camera_xyz_in_ee = np.asarray(wrist_camera_xyz_in_ee, dtype=np.float64)

    candidates: list[WristViewCandidate] = []
    for standoff in standoffs_m:
        standoff = max(0.04, float(standoff))
        for offset_label, offset_abs, offset_uv in _offset_options(lateral_offsets_m):
            lateral = tangent_u * float(offset_uv[0]) + tangent_v * float(offset_uv[1])
            camera_position = target + axis * standoff + lateral

            optical_z = _normalize(target - camera_position, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
            optical_x = _normalize(
                tangent_u - optical_z * float(tangent_u @ optical_z),
                np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
            )
            optical_y = _normalize(np.cross(optical_z, optical_x), np.asarray([0.0, 1.0, 0.0], dtype=np.float64))
            base_to_camera = np.column_stack([optical_x, optical_y, optical_z])

            base_to_ee = base_to_camera @ ee_to_camera.T
            ee_position = camera_position - base_to_ee @ camera_xyz_in_ee
            if float(ee_position[2]) < float(min_ee_z_m):
                ee_position = ee_position.copy()
                ee_position[2] = float(min_ee_z_m)
            actual_camera_position = ee_position + base_to_ee @ camera_xyz_in_ee
            target_in_camera = base_to_camera.T @ (target - actual_camera_position)

            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = base_to_ee
            quat = np.asarray(quaternion_from_matrix(matrix), dtype=np.float64)
            quat /= max(float(np.linalg.norm(quat)), 1e-9)
            candidates.append(
                WristViewCandidate(
                    label=f"s{int(round(standoff * 1000.0))}mm_{offset_label}",
                    standoff_m=standoff,
                    lateral_offset_m=offset_abs,
                    lateral_axis=offset_label,
                    camera_position=actual_camera_position.astype(np.float64),
                    ee_position=ee_position.astype(np.float64),
                    ee_quat_xyzw=quat.astype(np.float64),
                    target_in_camera=target_in_camera.astype(np.float64),
                )
            )
    return candidates
