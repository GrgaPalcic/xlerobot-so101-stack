"""Geometry helpers for surface-relative grasp staging."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SurfaceRelativeStages:
    ready: np.ndarray
    pregrasp: np.ndarray
    close: np.ndarray
    close_surface_clearance: float


def normalize_vector(vector: np.ndarray, fallback: tuple[float, float, float] = (0.0, 0.0, 1.0)) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if norm < 1e-9:
        values = np.asarray(fallback, dtype=np.float64)
        norm = float(np.linalg.norm(values))
    return values / max(norm, 1e-9)


def signed_plane_distance(point: np.ndarray, plane_point: np.ndarray, plane_normal: np.ndarray) -> float:
    normal = normalize_vector(plane_normal)
    return float((np.asarray(point, dtype=np.float64) - np.asarray(plane_point, dtype=np.float64)) @ normal)


def point_with_min_surface_clearance(
    point: np.ndarray,
    plane_point: np.ndarray,
    plane_normal: np.ndarray,
    *,
    offset_m: float = 0.0,
    min_clearance_m: float = 0.0,
) -> tuple[np.ndarray, float]:
    normal = normalize_vector(plane_normal)
    adjusted = np.asarray(point, dtype=np.float64) + normal * float(offset_m)
    signed = signed_plane_distance(adjusted, plane_point, normal)
    min_clearance = max(0.0, float(min_clearance_m))
    if signed < min_clearance:
        adjusted = adjusted + normal * (min_clearance - signed)
        signed = min_clearance
    return adjusted, float(signed)


def surface_relative_stages(
    target: np.ndarray,
    plane_point: np.ndarray,
    plane_normal: np.ndarray,
    *,
    ready_clearance_m: float,
    pregrasp_clearance_m: float,
    close_clearance_m: float,
    close_surface_clearance_m: float,
) -> SurfaceRelativeStages:
    normal = normalize_vector(plane_normal)
    close, close_signed = point_with_min_surface_clearance(
        target,
        plane_point,
        normal,
        offset_m=close_clearance_m,
        min_clearance_m=close_surface_clearance_m,
    )
    return SurfaceRelativeStages(
        ready=close + normal * max(0.0, float(ready_clearance_m)),
        pregrasp=close + normal * max(0.0, float(pregrasp_clearance_m)),
        close=close,
        close_surface_clearance=float(close_signed),
    )
