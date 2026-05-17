"""Prompt-driven grasp perception pipeline for the GPU host."""

from __future__ import annotations

import io
import json
import logging
import os
import sys
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import requests
import torch
from PIL import Image
from safetensors.torch import load_file
from scipy.spatial.transform import Rotation
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor, SamModel, SamProcessor

from grasp_server.geometry import backproject_masked_depth, invert_transform, resize_intrinsics, transform_points, workspace_mask

LOG = logging.getLogger(__name__)


class PipelineError(RuntimeError):
    """Base class for recoverable pipeline errors."""


class DependencyError(PipelineError):
    """Raised when an optional third-party dependency is unavailable."""


class DetectionError(PipelineError):
    """Raised when no usable object detection is available in a view."""


@dataclass(frozen=True)
class DetectionBox:
    index: int
    box_xyxy: tuple[float, float, float, float]
    score: float

    @property
    def center_x(self) -> float:
        return 0.5 * (self.box_xyxy[0] + self.box_xyxy[2])

    @property
    def center_y(self) -> float:
        return 0.5 * (self.box_xyxy[1] + self.box_xyxy[3])


@dataclass
class ViewCandidate:
    view_name: str
    detection: DetectionBox
    rank: int
    points_base: np.ndarray
    colors: np.ndarray
    mask: np.ndarray
    depth_m: np.ndarray
    centroid_base: np.ndarray


@dataclass
class DepthEstimate:
    depth_m: np.ndarray
    intrinsics: np.ndarray
    image_rgb: np.ndarray
    confidence: np.ndarray | None
    mode: str


@dataclass(frozen=True)
class SupportPlane:
    point: np.ndarray
    normal: np.ndarray


@dataclass
class PipelineConfig:
    metric_model: str
    grounding_model: str
    sam_model: str
    graspnet_root: str
    graspnet_checkpoint: str
    device: str
    process_res: int
    da3_conditioning: str
    da3_ref_view_strategy: str
    da3_use_ray_pose: bool
    da3_fallback_independent: bool
    box_threshold: float
    text_threshold: float
    min_depth_m: float
    max_depth_m: float
    voxel_size: float
    grasp_backend: str
    graspnet_num_points: int
    graspnet_num_view: int
    graspnet_collision_thresh: float
    graspnet_voxel_size: float
    m2t2_url: str
    m2t2_grasp_threshold: float
    m2t2_num_points: int
    m2t2_num_runs: int
    m2t2_apply_bounds: bool
    m2t2_timeout_s: float
    m2t2_default_width: float
    ggcnn_root: str
    ggcnn_weights: str
    ggcnn_primary_view: str
    ggcnn_input_size: int
    ggcnn_quality_threshold: float
    ggcnn_peak_min_distance_px: int
    ggcnn_crop_padding: float
    ggcnn_min_width_m: float
    ggcnn_max_width_m: float
    ggcnn_local_z_radius_m: float
    ggcnn_local_z_percentile: float
    workspace_bounds: tuple[float, float, float, float, float, float]
    support_plane_yaml: str
    support_plane_snap: bool
    support_plane_clearance_m: float
    support_plane_snap_min_correction_m: float
    support_plane_snap_max_correction_m: float
    support_plane_min_grasp_clearance_m: float
    max_detections_per_view: int
    association_max_distance_m: float
    association_primary_view: str
    association_axis: str


def _decode_jpeg(data: bytes) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise DetectionError("Failed to decode JPEG input")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _encode_mask_png(mask: np.ndarray) -> bytes:
    mask_u8 = (np.asarray(mask, dtype=np.uint8) * 255)
    ok, encoded = cv2.imencode(".png", mask_u8)
    if not ok:
        return b""
    return encoded.tobytes()


def _encode_depth_png(depth_m: np.ndarray, valid_mask: np.ndarray | None = None, label: str | None = None) -> bytes:
    depth = np.asarray(depth_m, dtype=np.float32)
    finite_mask = np.isfinite(depth) & (depth > 0.0)
    if valid_mask is not None:
        finite_mask &= np.asarray(valid_mask, dtype=bool)
    if not np.any(finite_mask):
        return b""

    lo = float(np.percentile(depth[finite_mask], 5.0))
    hi = float(np.percentile(depth[finite_mask], 95.0))
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        lo = float(np.min(depth[finite_mask]))
        hi = float(np.max(depth[finite_mask])) + 1e-6

    normalized = np.clip((depth - lo) / max(1e-6, hi - lo), 0.0, 1.0)
    vis_u8 = (255.0 * (1.0 - normalized)).astype(np.uint8)
    vis_bgr = cv2.applyColorMap(vis_u8, cv2.COLORMAP_TURBO)
    vis_bgr[~finite_mask] = 0
    if label:
        text = str(label)[:96]
        cv2.putText(vis_bgr, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis_bgr, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".png", vis_bgr)
    if not ok:
        return b""
    return encoded.tobytes()


def _encode_heatmap_png(values: np.ndarray, label: str | None = None) -> bytes:
    image = np.asarray(values, dtype=np.float32)
    finite = np.isfinite(image)
    if not np.any(finite):
        return b""
    lo = float(np.nanmin(image[finite]))
    hi = float(np.nanmax(image[finite]))
    normalized = np.zeros(image.shape, dtype=np.float32)
    if hi > lo:
        normalized[finite] = np.clip((image[finite] - lo) / (hi - lo), 0.0, 1.0)
    vis_bgr = cv2.applyColorMap((255.0 * normalized).astype(np.uint8), cv2.COLORMAP_TURBO)
    vis_bgr[~finite] = 0
    if label:
        text = str(label)[:96]
        cv2.putText(vis_bgr, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis_bgr, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".png", vis_bgr)
    if not ok:
        return b""
    return encoded.tobytes()


def _encode_rgb_png(image_rgb: np.ndarray, label: str | None = None) -> bytes:
    vis_bgr = cv2.cvtColor(np.asarray(image_rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    if label:
        text = str(label)[:96]
        cv2.putText(vis_bgr, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis_bgr, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".png", vis_bgr)
    if not ok:
        return b""
    return encoded.tobytes()


def _square_crop_for_mask(mask: np.ndarray, padding: float) -> tuple[int, int, int, int]:
    rows, cols = np.nonzero(np.asarray(mask, dtype=bool))
    if len(rows) == 0:
        raise DetectionError("GG-CNN cannot crop an empty object mask")
    height, width = mask.shape[:2]
    min_row, max_row = int(rows.min()), int(rows.max()) + 1
    min_col, max_col = int(cols.min()), int(cols.max()) + 1
    center_row = 0.5 * (min_row + max_row)
    center_col = 0.5 * (min_col + max_col)
    side = int(np.ceil(max(max_row - min_row, max_col - min_col) * max(1.0, float(padding))))
    side = max(8, min(side, height, width))

    top = int(round(center_row - 0.5 * side))
    left = int(round(center_col - 0.5 * side))
    top = min(max(0, top), height - side)
    left = min(max(0, left), width - side)
    return top, left, top + side, left + side


def _crop_pixel_to_image_pixel(
    row: float,
    col: float,
    crop: tuple[int, int, int, int],
    input_size: int,
) -> tuple[float, float]:
    top, left, bottom, right = crop
    scale_row = float(bottom - top) / float(input_size)
    scale_col = float(right - left) / float(input_size)
    image_row = float(top) + (float(row) + 0.5) * scale_row - 0.5
    image_col = float(left) + (float(col) + 0.5) * scale_col - 0.5
    return image_row, image_col


def _backproject_pixel_to_base(
    row: float,
    col: float,
    depth_m: float,
    intrinsics: np.ndarray,
    camera_to_base: np.ndarray,
) -> np.ndarray:
    k = np.asarray(intrinsics, dtype=np.float32)
    z = float(depth_m)
    point_camera = np.asarray(
        [
            (float(col) - float(k[0, 2])) * z / float(k[0, 0]),
            (float(row) - float(k[1, 2])) * z / float(k[1, 1]),
            z,
            1.0,
        ],
        dtype=np.float32,
    )
    return (np.asarray(camera_to_base, dtype=np.float32) @ point_camera)[:3].astype(np.float32)


def _image_angle_to_base_yaw(
    row: float,
    col: float,
    depth_m: float,
    angle_rad: float,
    intrinsics: np.ndarray,
    camera_to_base: np.ndarray,
    delta_px: float = 12.0,
) -> float:
    center = _backproject_pixel_to_base(row, col, depth_m, intrinsics, camera_to_base)
    # GG-CNN angles are image-plane anti-clockwise from horizontal; image rows grow down.
    end_row = float(row) - float(np.sin(angle_rad)) * delta_px
    end_col = float(col) + float(np.cos(angle_rad)) * delta_px
    end = _backproject_pixel_to_base(end_row, end_col, depth_m, intrinsics, camera_to_base)
    axis_xy = end[:2] - center[:2]
    if float(np.linalg.norm(axis_xy)) < 1e-6:
        return 0.0
    return float(np.arctan2(axis_xy[1], axis_xy[0]))


def _grasp_width_pixels_to_m(
    row: float,
    col: float,
    depth_m: float,
    angle_rad: float,
    width_px: float,
    intrinsics: np.ndarray,
    camera_to_base: np.ndarray,
) -> float:
    half = max(1.0, float(width_px) * 0.5)
    row_delta = -float(np.sin(angle_rad)) * half
    col_delta = float(np.cos(angle_rad)) * half
    left = _backproject_pixel_to_base(row - row_delta, col - col_delta, depth_m, intrinsics, camera_to_base)
    right = _backproject_pixel_to_base(row + row_delta, col + col_delta, depth_m, intrinsics, camera_to_base)
    return float(np.linalg.norm(right - left))


def _load_da3_model(model_name_or_path: str, device: torch.device):
    try:
        from depth_anything_3.api import DepthAnything3
    except ImportError as exc:
        raise DependencyError("depth_anything_3 is not installed in the grasp_server environment") from exc

    model_path = Path(model_name_or_path)
    if model_path.exists():
        config_path = model_path / "config.json"
        weight_path = model_path / "model.safetensors"
        if not config_path.exists() or not weight_path.exists():
            raise DependencyError(f"Invalid DA3 model directory: {model_path}")
        with open(config_path, "r", encoding="utf-8") as handle:
            config = json.load(handle)
        model = DepthAnything3(**config)
        weights = load_file(str(weight_path))
        model.load_state_dict(weights, strict=False)
    else:
        model = DepthAnything3.from_pretrained(model_name_or_path)

    return model.to(device).eval()


def _autocast_context(device: torch.device):
    if device.type == "cuda":
        return torch.autocast(device_type="cuda", dtype=torch.float16)
    return nullcontext()


def _candidate_sort_key(candidate: ViewCandidate, axis: str) -> float:
    if axis == "y":
        return candidate.detection.center_y
    return candidate.detection.center_x


def _rank_candidates(candidates: list[ViewCandidate], axis: str) -> list[ViewCandidate]:
    ranked = sorted(candidates, key=lambda candidate: _candidate_sort_key(candidate, axis))
    for rank, candidate in enumerate(ranked):
        candidate.rank = rank
    return ranked


def _load_support_plane(path: str) -> SupportPlane | None:
    if not path:
        return None
    plane_path = Path(path).expanduser()
    if not plane_path.exists():
        LOG.warning("Support plane calibration file does not exist: %s", plane_path)
        return None
    try:
        import yaml
    except ImportError as exc:
        raise DependencyError("PyYAML is required to load support-plane calibration") from exc

    with plane_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    points = data.get("points_xyz_in_base", {})
    try:
        top_left = np.asarray(points["top_left"], dtype=np.float32)
        bottom_left = np.asarray(points["bottom_left"], dtype=np.float32)
        bottom_right = np.asarray(points["bottom_right"], dtype=np.float32)
    except KeyError as exc:
        raise DependencyError(f"Invalid support-plane calibration file: missing {exc}") from exc

    normal = np.cross(bottom_left - top_left, bottom_right - bottom_left).astype(np.float32)
    norm = float(np.linalg.norm(normal))
    if not np.isfinite(norm) or norm < 1e-6:
        raise DependencyError(f"Invalid support-plane calibration file: degenerate plane in {plane_path}")
    normal /= norm
    if normal[2] < 0.0:
        normal *= -1.0
    LOG.info(
        "Loaded support plane from %s: point=(%.3f, %.3f, %.3f), normal=(%.3f, %.3f, %.3f)",
        plane_path,
        top_left[0],
        top_left[1],
        top_left[2],
        normal[0],
        normal[1],
        normal[2],
    )
    return SupportPlane(point=top_left, normal=normal)


def _best_rank_pair(
    primary: list[ViewCandidate],
    secondary: list[ViewCandidate],
) -> tuple[ViewCandidate, ViewCandidate]:
    secondary_by_rank = {candidate.rank: candidate for candidate in secondary}
    pairs = [
        (candidate, secondary_by_rank[candidate.rank])
        for candidate in primary
        if candidate.rank in secondary_by_rank
    ]
    if not pairs:
        count = min(len(primary), len(secondary))
        pairs = list(zip(primary[:count], secondary[:count]))
    if not pairs:
        raise DetectionError("No cross-view candidate pair is available")
    return max(pairs, key=lambda pair: pair[0].detection.score + pair[1].detection.score)


def select_cross_view_candidates(
    candidates_by_view: dict[str, list[ViewCandidate]],
    *,
    primary_view: str,
    axis: str,
    max_metric_distance_m: float,
) -> tuple[dict[str, ViewCandidate], str]:
    """Select one associated object instance per view.

    With calibrated cameras, metric centroid agreement is the only safe way to
    fuse views. If the views disagree, return a single primary-view candidate
    instead of rank-fusing clouds that may come from different objects.
    """

    available = {view: _rank_candidates(list(candidates), axis) for view, candidates in candidates_by_view.items() if candidates}
    if not available:
        raise DetectionError("No valid object candidates in any view")
    if len(available) == 1:
        view, candidates = next(iter(available.items()))
        return {view: max(candidates, key=lambda candidate: candidate.detection.score)}, f"single_view:{view}"

    primary_name = primary_view if primary_view in available else sorted(available)[0]
    secondary_name = next(view for view in sorted(available) if view != primary_name)
    primary = available[primary_name]
    secondary = available[secondary_name]

    best_metric_pair: tuple[ViewCandidate, ViewCandidate] | None = None
    best_metric_cost = float("inf")
    for left in primary:
        for right in secondary:
            distance = float(np.linalg.norm(left.centroid_base - right.centroid_base))
            score_bonus = 0.02 * (left.detection.score + right.detection.score)
            cost = distance - score_bonus
            if cost < best_metric_cost:
                best_metric_cost = cost
                best_metric_pair = (left, right)

    if best_metric_pair is not None:
        metric_distance = float(np.linalg.norm(best_metric_pair[0].centroid_base - best_metric_pair[1].centroid_base))
        if metric_distance <= max_metric_distance_m:
            return (
                {primary_name: best_metric_pair[0], secondary_name: best_metric_pair[1]},
                f"metric_centroid:{metric_distance:.3f}m",
            )

    if best_metric_pair is not None:
        primary_candidate, secondary_candidate = best_metric_pair
        metric_distance = float(np.linalg.norm(primary_candidate.centroid_base - secondary_candidate.centroid_base))
    else:
        metric_distance = float("inf")
    best_primary = max(primary, key=lambda candidate: candidate.detection.score)
    return (
        {primary_name: best_primary},
        f"single_{primary_name}:metric_untrusted:{metric_distance:.3f}m",
    )


class MetricDepthEstimator:
    def __init__(self, config: PipelineConfig) -> None:
        self._device = torch.device(config.device if config.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu"))
        self._model = _load_da3_model(config.metric_model, self._device)
        self._process_res = config.process_res
        self._conditioning = config.da3_conditioning
        self._ref_view_strategy = config.da3_ref_view_strategy
        self._use_ray_pose = config.da3_use_ray_pose
        self._fallback_independent = config.da3_fallback_independent

    def _supports_camera_conditioning(self) -> bool:
        da3_net = getattr(self._model, "model", None)
        return getattr(da3_net, "cam_enc", None) is not None

    def _prediction_depth(self, prediction, index: int) -> np.ndarray:
        depth = np.asarray(prediction.depth, dtype=np.float32)
        if depth.ndim == 2:
            depth = depth[np.newaxis]
        return np.asarray(depth[index], dtype=np.float32)

    def _prediction_confidence(self, prediction, index: int) -> np.ndarray | None:
        conf = getattr(prediction, "conf", None)
        if conf is None:
            return None
        conf = np.asarray(conf, dtype=np.float32)
        if conf.ndim == 2:
            conf = conf[np.newaxis]
        return np.asarray(conf[index], dtype=np.float32)

    def _prediction_image(self, prediction, index: int, fallback_rgb: np.ndarray, depth_hw: tuple[int, int]) -> np.ndarray:
        processed_images = getattr(prediction, "processed_images", None)
        if processed_images is not None:
            processed = np.asarray(processed_images)
            if processed.ndim == 3:
                processed = processed[np.newaxis]
            return np.asarray(processed[index], dtype=np.uint8)
        return cv2.resize(fallback_rgb, (depth_hw[1], depth_hw[0]), interpolation=cv2.INTER_LINEAR)

    def _estimate_independent(self, image_rgb: np.ndarray, intrinsics: np.ndarray) -> DepthEstimate:
        with _autocast_context(self._device):
            prediction = self._model.inference(
                [image_rgb],
                process_res=self._process_res,
                process_res_method="upper_bound_resize",
            )

        raw_depth = self._prediction_depth(prediction, 0)
        scaled_intrinsics = resize_intrinsics(
            intrinsics,
            src_hw=image_rgb.shape[:2],
            dst_hw=raw_depth.shape[:2],
        )
        focal_px = float(0.5 * (scaled_intrinsics[0, 0] + scaled_intrinsics[1, 1]))
        metric_depth = focal_px * raw_depth / 300.0
        processed_rgb = self._prediction_image(prediction, 0, image_rgb, raw_depth.shape[:2])
        return DepthEstimate(
            depth_m=metric_depth.astype(np.float32),
            intrinsics=scaled_intrinsics.astype(np.float32),
            image_rgb=processed_rgb,
            confidence=self._prediction_confidence(prediction, 0),
            mode="independent_focal_scaled",
        )

    def _camera_centers(self, extrinsics: np.ndarray) -> np.ndarray:
        centers = []
        for extrinsic in np.asarray(extrinsics, dtype=np.float32):
            centers.append(invert_transform(extrinsic)[:3, 3])
        return np.asarray(centers, dtype=np.float32)

    def _two_view_depth_scale(self, input_extrinsics: np.ndarray, predicted_extrinsics: np.ndarray | None) -> float:
        if predicted_extrinsics is None:
            return 1.0
        input_centers = self._camera_centers(input_extrinsics)
        predicted_centers = self._camera_centers(predicted_extrinsics)
        if len(input_centers) < 2 or len(predicted_centers) < 2:
            return 1.0
        input_baseline_m = float(np.linalg.norm(input_centers[1] - input_centers[0]))
        predicted_baseline = float(np.linalg.norm(predicted_centers[1] - predicted_centers[0]))
        if not np.isfinite(input_baseline_m) or not np.isfinite(predicted_baseline) or predicted_baseline < 1e-6:
            return 1.0
        return input_baseline_m / predicted_baseline

    def _estimate_conditioned(
        self,
        *,
        names: list[str],
        images: list[np.ndarray],
        intrinsics: np.ndarray,
        extrinsics: np.ndarray,
    ) -> dict[str, DepthEstimate]:
        """Run DA3 with calibrated camera tokens and avoid public two-view Umeyama alignment.

        DA3's public API uses Umeyama Sim(3) alignment after inference when
        calibrated extrinsics are provided. With exactly two camera centers that
        alignment is rank-degenerate. We still want the camera-token-conditioned
        forward pass, so we call the same private preprocessing/forward helpers
        and restore metric scale from the calibrated baseline.
        """

        imgs_cpu, processed_extrinsics_t, processed_intrinsics_t = self._model._preprocess_inputs(
            images,
            extrinsics=extrinsics,
            intrinsics=intrinsics,
            process_res=self._process_res,
            process_res_method="upper_bound_resize",
        )
        imgs, ex_t, in_t = self._model._prepare_model_inputs(
            imgs_cpu,
            processed_extrinsics_t,
            processed_intrinsics_t,
        )
        ex_t_norm = self._model._normalize_extrinsics(ex_t.clone() if ex_t is not None else None)
        raw_output = self._model._run_model_forward(
            imgs,
            ex_t_norm,
            in_t,
            export_feat_layers=[],
            infer_gs=False,
            use_ray_pose=self._use_ray_pose,
            ref_view_strategy=self._ref_view_strategy,
        )
        prediction = self._model._convert_to_prediction(raw_output)
        prediction = self._model._add_processed_images(prediction, imgs_cpu)

        processed_extrinsics = processed_extrinsics_t.cpu().numpy().astype(np.float32)
        processed_intrinsics = processed_intrinsics_t.cpu().numpy().astype(np.float32)
        depth_scale = self._two_view_depth_scale(processed_extrinsics, prediction.extrinsics)
        LOG.info("DA3 conditioned two-view metric scale: %.6f", depth_scale)

        estimates: dict[str, DepthEstimate] = {}
        for index, name in enumerate(names):
            depth_m = self._prediction_depth(prediction, index) * depth_scale
            processed_rgb = self._prediction_image(prediction, index, images[index], depth_m.shape[:2])
            estimates[name] = DepthEstimate(
                depth_m=depth_m.astype(np.float32),
                intrinsics=processed_intrinsics[index].astype(np.float32),
                image_rgb=processed_rgb,
                confidence=self._prediction_confidence(prediction, index),
                mode="camera_conditioned_two_view",
            )
        return estimates

    def estimate(self, image_rgb: np.ndarray, intrinsics: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        estimate = self._estimate_independent(image_rgb, intrinsics)
        return estimate.depth_m, estimate.intrinsics

    def estimate_views(
        self,
        views: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]],
    ) -> dict[str, DepthEstimate]:
        if not views:
            return {}
        if self._conditioning == "disabled" or len(views) < 2:
            LOG.info("Running DA3 independent focal-scaled depth for %d view(s)", len(views))
            return {
                name: self._estimate_independent(image_rgb, intrinsics)
                for name, (image_rgb, intrinsics, _camera_to_base) in views.items()
            }

        if not self._supports_camera_conditioning():
            message = (
                f"DA3 model '{type(self._model).__name__}' has no camera encoder; "
                "use DA3-LARGE/BASE/SMALL/GIANT or NESTED for camera-conditioned depth"
            )
            if self._conditioning == "required":
                raise DependencyError(message)
            LOG.warning("%s; falling back to independent focal-scaled depth", message)
            return {
                name: self._estimate_independent(image_rgb, intrinsics)
                for name, (image_rgb, intrinsics, _camera_to_base) in views.items()
            }

        names = list(views)
        images = [views[name][0] for name in names]
        intrinsics = np.stack([np.asarray(views[name][1], dtype=np.float32) for name in names], axis=0)
        # DA3 expects world-to-camera extrinsics. Our ROS TF is camera optical -> follower/base_link,
        # so the base frame is the DA3 world and the inverse TF is the conditioned camera extrinsic.
        extrinsics = np.stack([invert_transform(views[name][2]) for name in names], axis=0)
        LOG.info("Running DA3 camera-conditioned depth for views=%s", ",".join(names))

        try:
            return self._estimate_conditioned(
                names=names,
                images=images,
                intrinsics=intrinsics,
                extrinsics=extrinsics,
            )
        except Exception as exc:
            if self._conditioning == "required" or not self._fallback_independent:
                raise DetectionError(f"DA3 camera-conditioned depth failed: {exc}") from exc
            LOG.warning("DA3 camera-conditioned depth failed: %s; falling back to independent depth", exc)
            return {
                name: self._estimate_independent(image_rgb, intrinsics_matrix)
                for name, (image_rgb, intrinsics_matrix, _camera_to_base) in views.items()
            }


class GroundedSamSegmenter:
    def __init__(self, config: PipelineConfig) -> None:
        self._device = torch.device(config.device if config.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu"))
        self._box_threshold = config.box_threshold
        self._text_threshold = config.text_threshold
        model_kwargs = {"torch_dtype": torch.float16} if self._device.type == "cuda" else {}
        self._grounding_processor = AutoProcessor.from_pretrained(config.grounding_model)
        self._grounding_model = AutoModelForZeroShotObjectDetection.from_pretrained(
            config.grounding_model,
            **model_kwargs,
        ).to(self._device)
        self._sam_processor = SamProcessor.from_pretrained(config.sam_model)
        self._sam_model = SamModel.from_pretrained(config.sam_model, **model_kwargs).to(self._device)
        self._grounding_model.eval()
        self._sam_model.eval()

    def _to_device(self, batch: dict) -> dict:
        output = {}
        for key, value in batch.items():
            if torch.is_tensor(value):
                value = value.to(self._device)
                if self._device.type == "cuda" and value.is_floating_point():
                    value = value.half()
            output[key] = value.to(self._device) if hasattr(value, "to") else value
        return output

    def detect_boxes(self, image_rgb: np.ndarray, prompt: str) -> list[DetectionBox]:
        pil_image = Image.fromarray(image_rgb)
        text = prompt.strip()
        if not text.endswith("."):
            text = f"{text}."

        grounding_inputs = self._to_device(
            self._grounding_processor(images=pil_image, text=text, return_tensors="pt")
        )
        with torch.inference_mode(), _autocast_context(self._device):
            grounding_outputs = self._grounding_model(**grounding_inputs)
        detections = self._grounding_processor.post_process_grounded_object_detection(
            grounding_outputs,
            grounding_inputs["input_ids"],
            threshold=self._box_threshold,
            text_threshold=self._text_threshold,
            target_sizes=[image_rgb.shape[:2]],
        )[0]

        scores = detections.get("scores")
        if scores is None or len(scores) == 0:
            raise DetectionError(f"No detection for prompt '{prompt}'")

        boxes = detections["boxes"]
        score_values = scores.detach().cpu().numpy() if torch.is_tensor(scores) else np.asarray(scores)
        box_values = boxes.detach().cpu().numpy() if torch.is_tensor(boxes) else np.asarray(boxes)
        order = np.argsort(score_values)[::-1]
        results: list[DetectionBox] = []
        for rank_idx, detection_idx in enumerate(order):
            box = tuple(float(value) for value in box_values[detection_idx].tolist())
            results.append(
                DetectionBox(
                    index=int(detection_idx),
                    box_xyxy=box,
                    score=float(score_values[detection_idx]),
                )
            )
        return results

    def segment_box(self, image_rgb: np.ndarray, box_xyxy: tuple[float, float, float, float]) -> np.ndarray:
        pil_image = Image.fromarray(image_rgb)
        box = [float(value) for value in box_xyxy]
        sam_inputs = self._to_device(
            self._sam_processor(images=pil_image, input_boxes=[[box]], return_tensors="pt")
        )
        with torch.inference_mode(), _autocast_context(self._device):
            sam_outputs = self._sam_model(**sam_inputs, multimask_output=False)
        masks = self._sam_processor.image_processor.post_process_masks(
            sam_outputs.pred_masks.cpu(),
            sam_inputs["original_sizes"].cpu(),
            sam_inputs["reshaped_input_sizes"].cpu(),
        )
        mask = np.asarray(masks[0])
        while mask.ndim > 2:
            mask = mask[0]
        return (mask > 0).astype(bool)

    def segment(self, image_rgb: np.ndarray, prompt: str) -> tuple[np.ndarray, float]:
        detections = self.detect_boxes(image_rgb, prompt)
        best = detections[0]
        return self.segment_box(image_rgb, best.box_xyxy), best.score


class GraspNetAdapter:
    def __init__(self, config: PipelineConfig) -> None:
        if not config.graspnet_checkpoint:
            raise DependencyError("graspnet checkpoint path is required")

        self._cfg = config
        self._device = torch.device(config.device if config.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu"))

        if config.graspnet_root:
            root = Path(config.graspnet_root).expanduser()
            sys.path.insert(0, str(root))
            sys.path.insert(0, str(root / "models"))
            sys.path.insert(0, str(root / "utils"))
            sys.path.insert(0, str(root / "dataset"))

        try:
            from graspnetAPI import GraspGroup
            from collision_detector import ModelFreeCollisionDetector
            from graspnet import GraspNet, pred_decode
        except ImportError as exc:
            raise DependencyError(
                "graspnet-baseline and graspnetAPI must be installed for grasp generation"
            ) from exc

        self._GraspGroup = GraspGroup
        self._ModelFreeCollisionDetector = ModelFreeCollisionDetector
        self._pred_decode = pred_decode

        self._net = GraspNet(
            input_feature_dim=0,
            num_view=config.graspnet_num_view,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        ).to(self._device)

        checkpoint = torch.load(os.path.expanduser(config.graspnet_checkpoint), map_location=self._device)
        state_dict = checkpoint["model_state_dict"] if "model_state_dict" in checkpoint else checkpoint
        self._net.load_state_dict(state_dict)
        self._net.eval()

    def _sample_points(self, points: np.ndarray) -> np.ndarray:
        target = self._cfg.graspnet_num_points
        if len(points) == 0:
            raise DetectionError("Object cloud is empty")
        if len(points) >= target:
            idxs = np.random.choice(len(points), target, replace=False)
        else:
            idxs1 = np.arange(len(points))
            idxs2 = np.random.choice(len(points), target - len(points), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        return np.asarray(points[idxs], dtype=np.float32)

    def propose_grasps(self, points: np.ndarray, top_k: int) -> dict[str, np.ndarray | list[str]]:
        sampled = self._sample_points(points)
        end_points = {
            "point_clouds": torch.from_numpy(sampled[np.newaxis].astype(np.float32)).to(self._device),
            "cloud_colors": np.zeros((sampled.shape[0], 3), dtype=np.float32),
        }

        with torch.inference_mode():
            network_outputs = self._net(end_points)
            grasp_preds = self._pred_decode(network_outputs)

        grasp_group = self._GraspGroup(grasp_preds[0].detach().cpu().numpy())
        if self._cfg.graspnet_collision_thresh > 0:
            detector = self._ModelFreeCollisionDetector(points, voxel_size=self._cfg.graspnet_voxel_size)
            collision_mask = detector.detect(
                grasp_group,
                approach_dist=0.05,
                collision_thresh=self._cfg.graspnet_collision_thresh,
            )
            grasp_group = grasp_group[~collision_mask]

        grasp_group.nms()
        grasp_group.sort_by_score()

        translations = np.asarray(grasp_group.translations, dtype=np.float32)
        if len(translations) == 0:
            raise DetectionError("GraspNet returned no grasps after filtering")

        valid_mask = np.isfinite(translations).all(axis=1) & (translations[:, 2] > self._cfg.workspace_bounds[4])
        grasp_group = grasp_group[valid_mask]
        count = min(top_k, len(grasp_group))
        if count <= 0:
            raise DetectionError("No valid grasps after workspace filtering")

        positions = np.asarray(grasp_group.translations[:count], dtype=np.float32)
        rotations = np.asarray(grasp_group.rotation_matrices[:count], dtype=np.float32)
        quaternions = Rotation.from_matrix(rotations).as_quat().astype(np.float32)
        scores = np.asarray(grasp_group.scores[:count], dtype=np.float32)
        widths = np.asarray(grasp_group.widths[:count], dtype=np.float32)

        return {
            "grasp_positions": positions,
            "grasp_quaternions": quaternions,
            "grasp_scores": scores,
            "grasp_widths": widths,
            "grasp_sources": ["graspnet"] * count,
        }


class M2T2Adapter:
    """HTTP client for the TiPToP/M2T2 grasp service."""

    def __init__(self, config: PipelineConfig) -> None:
        if not config.m2t2_url:
            raise DependencyError("m2t2 url is required when grasp_backend=m2t2")
        self._cfg = config

    def _build_payload(self, points: np.ndarray, colors: np.ndarray) -> dict[str, object]:
        return {
            "pointcloud": {
                "points": np.asarray(points, dtype=np.float32).tolist(),
                "rgb": np.clip(np.asarray(colors, dtype=np.float32), 0.0, 1.0).tolist(),
            },
            "num_points": self._cfg.m2t2_num_points,
            "num_runs": self._cfg.m2t2_num_runs,
            "mask_thresh": self._cfg.m2t2_grasp_threshold,
            "apply_bounds": self._cfg.m2t2_apply_bounds,
        }

    def propose_grasps(self, points: np.ndarray, colors: np.ndarray, top_k: int) -> dict[str, np.ndarray | list[str]]:
        if len(points) == 0:
            raise DetectionError("Object cloud is empty")
        if len(colors) != len(points):
            colors = np.zeros_like(points, dtype=np.float32)

        endpoint = f"{self._cfg.m2t2_url.rstrip('/')}/predict"
        response = requests.post(
            endpoint,
            json=self._build_payload(points, colors),
            timeout=self._cfg.m2t2_timeout_s,
        )
        response.raise_for_status()
        result = response.json()

        poses_chunks: list[np.ndarray] = []
        score_chunks: list[np.ndarray] = []
        for poses, confidences in zip(result.get("grasps", []), result.get("grasp_confidence", [])):
            if not poses:
                continue
            poses_arr = np.asarray(poses, dtype=np.float32).reshape(-1, 4, 4)
            conf_arr = np.asarray(confidences, dtype=np.float32).reshape(-1)
            if len(conf_arr) != len(poses_arr):
                conf_arr = np.ones((len(poses_arr),), dtype=np.float32)
            poses_chunks.append(poses_arr)
            score_chunks.append(conf_arr)

        if not poses_chunks:
            raise DetectionError("M2T2 returned no grasps")

        poses_all = np.concatenate(poses_chunks, axis=0)
        scores_all = np.concatenate(score_chunks, axis=0)
        valid_mask = np.isfinite(poses_all).all(axis=(1, 2)) & np.isfinite(scores_all)
        poses_all = poses_all[valid_mask]
        scores_all = scores_all[valid_mask]
        if len(poses_all) == 0:
            raise DetectionError("M2T2 returned no finite grasps")

        order = np.argsort(scores_all)[::-1][:top_k]
        poses_top = poses_all[order]
        scores_top = scores_all[order]
        positions = poses_top[:, :3, 3].astype(np.float32)
        rotations = poses_top[:, :3, :3].astype(np.float32)
        quaternions = Rotation.from_matrix(rotations).as_quat().astype(np.float32)
        widths = np.full((len(positions),), self._cfg.m2t2_default_width, dtype=np.float32)

        return {
            "grasp_positions": positions,
            "grasp_quaternions": quaternions,
            "grasp_scores": scores_top.astype(np.float32),
            "grasp_widths": widths,
            "grasp_sources": ["m2t2"] * len(positions),
        }


@dataclass(frozen=True)
class GGCNNCrop:
    view_name: str
    crop: tuple[int, int, int, int]
    depth_input: np.ndarray
    mask_input: np.ndarray


class GGCNNAdapter:
    """Planar GG-CNN grasp proposal from a segmented depth crop.

    GG-CNN predicts image-plane antipodal grasps, which is a better match for
    the SO-101's practical 5-DOF pick primitive than unconstrained 6-DOF grasps.
    """

    def __init__(self, config: PipelineConfig) -> None:
        self._cfg = config
        self._device = torch.device(config.device if config.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu"))
        self._input_size = int(config.ggcnn_input_size)
        if self._input_size <= 0:
            raise DependencyError("ggcnn input size must be positive")

        root = Path(config.ggcnn_root).expanduser()
        if not root.exists():
            raise DependencyError(f"ggcnn root does not exist: {root}")
        weights = Path(config.ggcnn_weights).expanduser()
        if not weights.exists():
            raise DependencyError(f"ggcnn weights do not exist: {weights}")

        sys.path.insert(0, str(root))
        try:
            from models.ggcnn import GGCNN
        except ImportError as exc:
            raise DependencyError(f"Could not import GG-CNN from {root}") from exc

        self._net = GGCNN().to(self._device)
        state_dict = torch.load(str(weights), map_location=self._device)
        if isinstance(state_dict, dict) and "state_dict" in state_dict:
            state_dict = state_dict["state_dict"]
        self._net.load_state_dict(state_dict)
        self._net.eval()

    def _select_view(self, selected_by_view: dict[str, ViewCandidate]) -> tuple[str, ViewCandidate]:
        primary = self._cfg.ggcnn_primary_view.strip().lower()
        if primary in selected_by_view:
            return primary, selected_by_view[primary]
        if selected_by_view:
            view_name = sorted(selected_by_view)[0]
            return view_name, selected_by_view[view_name]
        raise DetectionError("GG-CNN needs a selected segmented view")

    def _prepare_crop(self, view_name: str, candidate: ViewCandidate) -> GGCNNCrop:
        depth = np.asarray(candidate.depth_m, dtype=np.float32)
        mask = np.asarray(candidate.mask, dtype=bool)
        if depth.shape != mask.shape:
            raise DetectionError(f"GG-CNN depth/mask shape mismatch for {view_name}: {depth.shape} != {mask.shape}")

        crop = _square_crop_for_mask(mask, self._cfg.ggcnn_crop_padding)
        top, left, bottom, right = crop
        depth_crop = depth[top:bottom, left:right].astype(np.float32, copy=True)
        mask_crop = mask[top:bottom, left:right]
        finite = np.isfinite(depth_crop) & (depth_crop >= self._cfg.min_depth_m) & (depth_crop <= self._cfg.max_depth_m)
        object_valid = finite & mask_crop
        if np.count_nonzero(object_valid) < 32:
            raise DetectionError(f"GG-CNN {view_name} crop has too few valid object-depth pixels")

        background_valid = finite & ~mask_crop
        if np.any(background_valid):
            fill_depth = float(np.percentile(depth_crop[background_valid], 70.0))
        else:
            fill_depth = float(np.percentile(depth_crop[object_valid], 95.0) + 0.05)

        prepared = np.full(depth_crop.shape, fill_depth, dtype=np.float32)
        prepared[object_valid] = depth_crop[object_valid]
        resized_depth = cv2.resize(
            prepared,
            (self._input_size, self._input_size),
            interpolation=cv2.INTER_AREA,
        ).astype(np.float32)
        resized_mask = cv2.resize(
            mask_crop.astype(np.uint8),
            (self._input_size, self._input_size),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
        mean_depth = float(np.mean(resized_depth[resized_mask])) if np.any(resized_mask) else float(np.mean(resized_depth))
        normalized_depth = np.clip(resized_depth - mean_depth, -1.0, 1.0).astype(np.float32)
        return GGCNNCrop(
            view_name=view_name,
            crop=crop,
            depth_input=normalized_depth,
            mask_input=resized_mask,
        )

    def _post_process(self, outputs: tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        q_img, cos_img, sin_img, width_img = outputs
        q = q_img.detach().cpu().numpy().squeeze().astype(np.float32)
        angle = (torch.atan2(sin_img, cos_img) / 2.0).detach().cpu().numpy().squeeze().astype(np.float32)
        width = (width_img.detach().cpu().numpy().squeeze().astype(np.float32) * 150.0)
        q = cv2.GaussianBlur(q, (0, 0), 2.0)
        angle = cv2.GaussianBlur(angle, (0, 0), 2.0)
        width = cv2.GaussianBlur(width, (0, 0), 1.0)
        if q.shape != (self._input_size, self._input_size):
            q = cv2.resize(q, (self._input_size, self._input_size), interpolation=cv2.INTER_LINEAR)
            angle = cv2.resize(angle, (self._input_size, self._input_size), interpolation=cv2.INTER_LINEAR)
            width = cv2.resize(width, (self._input_size, self._input_size), interpolation=cv2.INTER_LINEAR)
        return q.astype(np.float32), angle.astype(np.float32), width.astype(np.float32)

    def _find_peaks(self, quality: np.ndarray, top_k: int) -> list[tuple[int, int]]:
        finite = np.isfinite(quality)
        if not np.any(finite):
            return []
        min_distance = max(1, int(self._cfg.ggcnn_peak_min_distance_px))
        threshold = float(self._cfg.ggcnn_quality_threshold)
        work = np.where(finite, quality, -np.inf).astype(np.float32)
        peaks: list[tuple[int, int]] = []
        for _ in range(max(1, int(top_k))):
            flat_index = int(np.argmax(work))
            row, col = np.unravel_index(flat_index, work.shape)
            score = float(work[row, col])
            if not np.isfinite(score) or score < threshold:
                break
            peaks.append((int(row), int(col)))
            r0 = max(0, row - min_distance)
            r1 = min(work.shape[0], row + min_distance + 1)
            c0 = max(0, col - min_distance)
            c1 = min(work.shape[1], col + min_distance + 1)
            work[r0:r1, c0:c1] = -np.inf
        return peaks

    def _depth_at_image_pixel(self, candidate: ViewCandidate, row: float, col: float) -> float:
        depth = np.asarray(candidate.depth_m, dtype=np.float32)
        mask = np.asarray(candidate.mask, dtype=bool)
        r = int(np.clip(round(row), 0, depth.shape[0] - 1))
        c = int(np.clip(round(col), 0, depth.shape[1] - 1))
        value = float(depth[r, c])
        if np.isfinite(value) and self._cfg.min_depth_m <= value <= self._cfg.max_depth_m:
            return value
        valid = mask & np.isfinite(depth) & (depth >= self._cfg.min_depth_m) & (depth <= self._cfg.max_depth_m)
        if not np.any(valid):
            raise DetectionError("GG-CNN selected pixel has no valid depth fallback")
        return float(np.median(depth[valid]))

    def _local_cloud_z(self, points: np.ndarray, xy: np.ndarray, fallback_z: float) -> float:
        finite = np.asarray(points, dtype=np.float32)
        valid = np.isfinite(finite).all(axis=1)
        finite = finite[valid]
        if len(finite) == 0:
            return float(fallback_z)
        distances = np.linalg.norm(finite[:, :2] - xy[np.newaxis, :], axis=1)
        local = finite[distances <= max(0.005, float(self._cfg.ggcnn_local_z_radius_m))]
        if len(local) < 8:
            local = finite
        percentile = float(np.clip(self._cfg.ggcnn_local_z_percentile, 0.0, 100.0))
        z = float(np.percentile(local[:, 2], percentile))
        return z if np.isfinite(z) else float(fallback_z)

    def _snap_position_to_plane(self, position: np.ndarray, support_plane: SupportPlane | None) -> np.ndarray:
        if support_plane is None:
            return position
        adjusted = np.asarray(position, dtype=np.float32).copy()
        signed = float((adjusted - support_plane.point) @ support_plane.normal)
        min_clearance = max(0.0, float(self._cfg.support_plane_min_grasp_clearance_m))
        if signed < min_clearance:
            adjusted += (min_clearance - signed) * support_plane.normal.astype(np.float32)
        return adjusted

    def _make_debug_overlay(
        self,
        candidate: ViewCandidate,
        crop: GGCNNCrop,
        peaks: list[tuple[int, int]],
        angle: np.ndarray,
        width: np.ndarray,
    ) -> bytes:
        top, left, bottom, right = crop.crop
        rgb_crop = candidate.depth_m[top:bottom, left:right]
        depth_vis = cv2.imdecode(np.frombuffer(_encode_depth_png(rgb_crop), dtype=np.uint8), cv2.IMREAD_COLOR)
        if depth_vis is None:
            depth_vis = np.zeros((self._input_size, self._input_size, 3), dtype=np.uint8)
        depth_vis = cv2.resize(depth_vis, (self._input_size, self._input_size), interpolation=cv2.INTER_LINEAR)
        if peaks:
            row, col = peaks[0]
            theta = float(angle[row, col])
            length = max(20.0, float(width[row, col]))
            jaw = max(8.0, 0.5 * length)
            axis = np.asarray([np.cos(theta), -np.sin(theta)], dtype=np.float32)
            normal = np.asarray([-axis[1], axis[0]], dtype=np.float32)
            center = np.asarray([float(col), float(row)], dtype=np.float32)
            corners = np.asarray(
                [
                    center - axis * length * 0.5 - normal * jaw * 0.5,
                    center + axis * length * 0.5 - normal * jaw * 0.5,
                    center + axis * length * 0.5 + normal * jaw * 0.5,
                    center - axis * length * 0.5 + normal * jaw * 0.5,
                ],
                dtype=np.int32,
            )
            cv2.polylines(depth_vis, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(depth_vis, (int(col), int(row)), 4, (0, 0, 255), -1, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".png", depth_vis)
        if not ok:
            return b""
        return encoded.tobytes()

    def propose_grasps(
        self,
        *,
        selected_by_view: dict[str, ViewCandidate],
        intrinsics_by_view: dict[str, np.ndarray],
        camera_to_base_by_view: dict[str, np.ndarray],
        support_plane: SupportPlane | None,
        top_k: int,
    ) -> dict[str, object]:
        view_name, candidate = self._select_view(selected_by_view)
        if view_name not in camera_to_base_by_view:
            raise DetectionError(f"GG-CNN missing camera transform for {view_name}")
        if view_name not in intrinsics_by_view:
            raise DetectionError(f"GG-CNN missing intrinsics for {view_name}")
        intrinsics = np.asarray(intrinsics_by_view[view_name], dtype=np.float32)
        crop = self._prepare_crop(view_name, candidate)

        tensor = torch.from_numpy(crop.depth_input[np.newaxis, np.newaxis].astype(np.float32)).to(self._device)
        with torch.inference_mode():
            outputs = self._net(tensor)
        quality, angle, width_px = self._post_process(outputs)

        masked_quality = np.where(crop.mask_input, quality, -np.inf).astype(np.float32)
        peaks = self._find_peaks(masked_quality, top_k)
        if not peaks:
            max_quality = float(np.nanmax(masked_quality)) if np.any(np.isfinite(masked_quality)) else float("nan")
            raise DetectionError(f"GG-CNN returned no masked grasp peaks above threshold (max={max_quality:.3f})")

        camera_to_base = np.asarray(camera_to_base_by_view[view_name], dtype=np.float32)
        positions: list[np.ndarray] = []
        quaternions: list[np.ndarray] = []
        scores: list[float] = []
        widths: list[float] = []

        for row, col in peaks:
            image_row, image_col = _crop_pixel_to_image_pixel(row, col, crop.crop, self._input_size)
            depth = self._depth_at_image_pixel(candidate, image_row, image_col)
            position = _backproject_pixel_to_base(
                image_row,
                image_col,
                depth,
                intrinsics,
                camera_to_base,
            )
            position[2] = self._local_cloud_z(candidate.points_base, position[:2], position[2])
            position = self._snap_position_to_plane(position, support_plane)

            theta = float(angle[row, col])
            yaw = _image_angle_to_base_yaw(
                image_row,
                image_col,
                depth,
                theta,
                intrinsics,
                camera_to_base,
            )
            width_orig_px = float(width_px[row, col]) * float(crop.crop[3] - crop.crop[1]) / float(self._input_size)
            width_m = _grasp_width_pixels_to_m(
                image_row,
                image_col,
                depth,
                theta,
                width_orig_px,
                intrinsics,
                camera_to_base,
            )
            width_m = float(np.clip(width_m, self._cfg.ggcnn_min_width_m, self._cfg.ggcnn_max_width_m))
            quat = Rotation.from_euler("xyz", [0.0, np.pi, yaw]).as_quat().astype(np.float32)

            positions.append(position.astype(np.float32))
            quaternions.append(quat)
            scores.append(float(quality[row, col]))
            widths.append(width_m)

        return {
            "grasp_positions": np.asarray(positions, dtype=np.float32),
            "grasp_quaternions": np.asarray(quaternions, dtype=np.float32),
            "grasp_scores": np.asarray(scores, dtype=np.float32),
            "grasp_widths": np.asarray(widths, dtype=np.float32),
            "grasp_sources": ["ggcnn"] * len(positions),
            "debug_blobs": {
                "ggcnn_quality_png": _encode_heatmap_png(masked_quality, label=f"ggcnn quality: {view_name}"),
                "ggcnn_angle_png": _encode_heatmap_png(angle, label=f"ggcnn angle: {view_name}"),
                "ggcnn_overlay_png": self._make_debug_overlay(candidate, crop, peaks, angle, width_px),
                "ggcnn_depth_input_png": _encode_heatmap_png(
                    np.where(crop.mask_input, crop.depth_input, np.nan),
                    label=f"ggcnn depth input: {view_name}",
                ),
            },
        }


class GraspPerceptionPipeline:
    def __init__(self, config: PipelineConfig) -> None:
        self._cfg = config
        self._segmenter: GroundedSamSegmenter | None = None
        self._depth_estimator: MetricDepthEstimator | None = None
        self._graspnet: GraspNetAdapter | None = None
        self._m2t2: M2T2Adapter | None = None
        self._ggcnn: GGCNNAdapter | None = None
        self._support_plane = _load_support_plane(config.support_plane_yaml)

    def _segmenter_instance(self) -> GroundedSamSegmenter:
        if self._segmenter is None:
            self._segmenter = GroundedSamSegmenter(self._cfg)
        return self._segmenter

    def _depth_estimator_instance(self) -> MetricDepthEstimator:
        if self._depth_estimator is None:
            self._depth_estimator = MetricDepthEstimator(self._cfg)
        return self._depth_estimator

    def _graspnet_instance(self) -> GraspNetAdapter:
        if self._graspnet is None:
            self._graspnet = GraspNetAdapter(self._cfg)
        return self._graspnet

    def _m2t2_instance(self) -> M2T2Adapter:
        if self._m2t2 is None:
            self._m2t2 = M2T2Adapter(self._cfg)
        return self._m2t2

    def _ggcnn_instance(self) -> GGCNNAdapter:
        if self._ggcnn is None:
            self._ggcnn = GGCNNAdapter(self._cfg)
        return self._ggcnn

    def _snap_points_to_support_plane(self, points: np.ndarray, label: str) -> np.ndarray:
        if not self._cfg.support_plane_snap or self._support_plane is None or len(points) == 0:
            return points
        finite = np.asarray(points, dtype=np.float32)
        valid = np.isfinite(finite).all(axis=1)
        if not np.any(valid):
            return points

        signed = (finite[valid] - self._support_plane.point) @ self._support_plane.normal
        low = float(np.percentile(signed, 5.0))
        correction = float(self._cfg.support_plane_clearance_m - low)
        if correction < self._cfg.support_plane_snap_min_correction_m:
            return points
        if correction > self._cfg.support_plane_snap_max_correction_m:
            LOG.warning(
                "Skipping support-plane snap for %s: correction %.3fm exceeds max %.3fm",
                label,
                correction,
                self._cfg.support_plane_snap_max_correction_m,
            )
            return points

        adjusted = np.asarray(points, dtype=np.float32).copy()
        adjusted += correction * self._support_plane.normal.astype(np.float32)
        LOG.info(
            "Applied support-plane snap to %s: lower signed distance %.3fm -> clearance %.3fm "
            "(correction %.3fm)",
            label,
            low,
            self._cfg.support_plane_clearance_m,
            correction,
        )
        return adjusted

    def _filter_cloud(self, points: np.ndarray, colors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        mask = workspace_mask(points, self._cfg.workspace_bounds)
        cropped = np.asarray(points[mask], dtype=np.float32)
        cropped_colors = np.asarray(colors[mask], dtype=np.float32) if len(colors) == len(points) else np.zeros_like(cropped)
        if cropped.size == 0:
            return cropped, np.zeros((0, 3), dtype=np.float32)

        try:
            import open3d as o3d
        except ImportError as exc:
            raise DependencyError("open3d is required for object cloud filtering") from exc

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(cropped.astype(np.float64))
        point_cloud.colors = o3d.utility.Vector3dVector(np.clip(cropped_colors.astype(np.float64), 0.0, 1.0))
        point_cloud = point_cloud.voxel_down_sample(self._cfg.voxel_size)
        if len(point_cloud.points) >= 32:
            point_cloud, _ = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        return np.asarray(point_cloud.points, dtype=np.float32), np.asarray(point_cloud.colors, dtype=np.float32)

    def _propose_grasps(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        top_k: int,
        *,
        selected_by_view: dict[str, ViewCandidate],
        intrinsics_by_view: dict[str, np.ndarray],
        camera_to_base_by_view: dict[str, np.ndarray],
    ) -> dict[str, object]:
        backend = self._cfg.grasp_backend.strip().lower()
        if backend == "graspnet":
            return self._graspnet_instance().propose_grasps(points, top_k)
        if backend == "m2t2":
            return self._m2t2_instance().propose_grasps(points, colors, top_k)
        if backend == "ggcnn":
            return self._ggcnn_instance().propose_grasps(
                selected_by_view=selected_by_view,
                intrinsics_by_view=intrinsics_by_view,
                camera_to_base_by_view=camera_to_base_by_view,
                support_plane=self._support_plane,
                top_k=top_k,
            )
        raise DependencyError(f"Unknown grasp backend: {self._cfg.grasp_backend}")

    def _filter_grasps_against_support_plane(
        self,
        grasp_results: dict[str, object],
    ) -> dict[str, object]:
        if self._support_plane is None:
            return grasp_results
        positions = np.asarray(grasp_results["grasp_positions"], dtype=np.float32)
        if len(positions) == 0:
            return grasp_results

        signed = (positions - self._support_plane.point) @ self._support_plane.normal
        valid = np.isfinite(signed) & (signed >= self._cfg.support_plane_min_grasp_clearance_m)
        rejected = int(len(valid) - np.count_nonzero(valid))
        if rejected:
            LOG.info(
                "Rejected %d grasp(s) below support plane: min signed distance %.3fm, required %.3fm",
                rejected,
                float(np.nanmin(signed)),
                self._cfg.support_plane_min_grasp_clearance_m,
            )
        if not np.any(valid):
            raise DetectionError(
                "All grasp candidates are below the calibrated support plane "
                f"(min signed distance {float(np.nanmin(signed)):.3f}m)"
            )

        filtered: dict[str, object] = {}
        for key, value in grasp_results.items():
            if isinstance(value, np.ndarray) and len(value) == len(valid):
                filtered[key] = value[valid]
            elif isinstance(value, list) and len(value) == len(valid):
                filtered[key] = [item for item, keep in zip(value, valid) if bool(keep)]
            else:
                filtered[key] = value
        return filtered

    def _process_view(
        self,
        *,
        depth_estimate: DepthEstimate,
        prompt: str,
        camera_to_base: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, np.ndarray]:
        image_rgb = depth_estimate.image_rgb
        depth_m = depth_estimate.depth_m
        mask, score = self._segmenter_instance().segment(image_rgb, prompt)
        if mask.shape != depth_m.shape:
            mask = cv2.resize(mask.astype(np.uint8), (depth_m.shape[1], depth_m.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
        points_camera = backproject_masked_depth(
            depth_m,
            depth_estimate.intrinsics,
            mask,
            min_depth_m=self._cfg.min_depth_m,
            max_depth_m=self._cfg.max_depth_m,
        )
        if len(points_camera) == 0:
            raise DetectionError("Segmentation succeeded but produced no valid depth points")
        valid = mask & np.isfinite(depth_m) & (depth_m >= self._cfg.min_depth_m) & (depth_m <= self._cfg.max_depth_m)
        rows, cols = np.nonzero(valid)
        colors = image_rgb[rows, cols].astype(np.float32) / 255.0
        points_base = transform_points(points_camera, camera_to_base)
        return points_base, colors, mask, score, depth_m

    def _process_view_candidates(
        self,
        *,
        view_name: str,
        depth_estimate: DepthEstimate,
        prompt: str,
        camera_to_base: np.ndarray,
    ) -> tuple[list[ViewCandidate], np.ndarray]:
        image_rgb = depth_estimate.image_rgb
        depth_m = depth_estimate.depth_m
        scaled_intrinsics = depth_estimate.intrinsics
        detections = self._segmenter_instance().detect_boxes(image_rgb, prompt)
        detections = detections[: max(1, self._cfg.max_detections_per_view)]

        candidates: list[ViewCandidate] = []
        for detection in detections:
            try:
                mask = self._segmenter_instance().segment_box(image_rgb, detection.box_xyxy)
            except DetectionError:
                continue
            if mask.shape != depth_m.shape:
                mask = cv2.resize(mask.astype(np.uint8), (depth_m.shape[1], depth_m.shape[0]), interpolation=cv2.INTER_NEAREST) > 0

            points_camera = backproject_masked_depth(
                depth_m,
                scaled_intrinsics,
                mask,
                min_depth_m=self._cfg.min_depth_m,
                max_depth_m=self._cfg.max_depth_m,
            )
            if len(points_camera) == 0:
                continue

            valid = mask & np.isfinite(depth_m) & (depth_m >= self._cfg.min_depth_m) & (depth_m <= self._cfg.max_depth_m)
            rows, cols = np.nonzero(valid)
            colors = image_rgb[rows, cols].astype(np.float32) / 255.0
            points_base = transform_points(points_camera, camera_to_base)
            points_base = self._snap_points_to_support_plane(points_base, f"{view_name}:candidate:{detection.index}")
            if len(points_base) == 0:
                continue
            centroid_base = np.median(points_base, axis=0).astype(np.float32)
            candidates.append(
                ViewCandidate(
                    view_name=view_name,
                    detection=detection,
                    rank=-1,
                    points_base=points_base,
                    colors=colors,
                    mask=mask,
                    depth_m=depth_m,
                    centroid_base=centroid_base,
                )
            )

        if not candidates:
            raise DetectionError(f"No valid segmented candidates for prompt '{prompt}'")
        return candidates, depth_m

    def detect_grasps(self, request: dict[str, object]) -> dict[str, object]:
        prompt = str(request["prompt"])
        top_k = int(request["top_k"])
        overhead_intrinsics = np.asarray(request["overhead_intrinsics"], dtype=np.float32)
        wrist_intrinsics = np.asarray(request["wrist_intrinsics"], dtype=np.float32)
        overhead_camera_to_base = np.asarray(request["overhead_camera_to_base"], dtype=np.float32)
        wrist_camera_to_base = np.asarray(request["wrist_camera_to_base"], dtype=np.float32)

        overhead_rgb = _decode_jpeg(request["overhead_jpeg"])
        wrist_rgb = _decode_jpeg(request["wrist_jpeg"])
        depth_estimates = self._depth_estimator_instance().estimate_views(
            {
                "overhead": (overhead_rgb, overhead_intrinsics, overhead_camera_to_base),
                "wrist": (wrist_rgb, wrist_intrinsics, wrist_camera_to_base),
            }
        )

        candidates_by_view: dict[str, list[ViewCandidate]] = {}
        view_clouds: dict[str, np.ndarray] = {
            "overhead": np.zeros((0, 3), dtype=np.float32),
            "wrist": np.zeros((0, 3), dtype=np.float32),
        }
        depth_pngs: dict[str, bytes] = {"overhead": b"", "wrist": b""}
        masked_depth_pngs: dict[str, bytes] = {"overhead": b"", "wrist": b""}
        overhead_mask = np.zeros(overhead_rgb.shape[:2], dtype=bool)
        wrist_mask = np.zeros(wrist_rgb.shape[:2], dtype=bool)
        depth_modes = {name: estimate.mode for name, estimate in depth_estimates.items()}

        for name in ("overhead", "wrist"):
            depth_pngs[name] = _encode_depth_png(
                depth_estimates[name].depth_m,
                label=f"{name}: {depth_estimates[name].mode}",
            )

        for name, transform in (
            ("overhead", overhead_camera_to_base),
            ("wrist", wrist_camera_to_base),
        ):
            try:
                candidates, depth_m = self._process_view_candidates(
                    view_name=name,
                    depth_estimate=depth_estimates[name],
                    prompt=prompt,
                    camera_to_base=transform,
                )
            except DetectionError as exc:
                LOG.warning("Skipping %s view: %s", name, exc)
                continue
            candidates_by_view[name] = candidates
            LOG.info("Found %d %s candidates for prompt '%s'", len(candidates), name, prompt)

        def make_response(
            *,
            success: bool,
            message: str,
            object_cloud: np.ndarray | None = None,
            grasp_results: dict[str, object] | None = None,
        ) -> dict[str, object]:
            if object_cloud is None:
                object_cloud = np.zeros((0, 3), dtype=np.float32)
            if grasp_results is None:
                grasp_results = {
                    "grasp_positions": np.zeros((0, 3), dtype=np.float32),
                    "grasp_quaternions": np.zeros((0, 4), dtype=np.float32),
                    "grasp_scores": np.zeros((0,), dtype=np.float32),
                    "grasp_widths": np.zeros((0,), dtype=np.float32),
                    "grasp_sources": [],
                    "debug_blobs": {},
                }
            debug_blobs = dict(grasp_results.get("debug_blobs", {}))
            return {
                "success": bool(success),
                "message": message,
                "overhead_object_cloud": view_clouds["overhead"].astype(np.float32),
                "wrist_object_cloud": view_clouds["wrist"].astype(np.float32),
                "object_cloud": np.asarray(object_cloud, dtype=np.float32),
                "grasp_positions": np.asarray(grasp_results["grasp_positions"], dtype=np.float32),
                "grasp_quaternions": np.asarray(grasp_results["grasp_quaternions"], dtype=np.float32),
                "grasp_scores": np.asarray(grasp_results["grasp_scores"], dtype=np.float32),
                "grasp_widths": np.asarray(grasp_results["grasp_widths"], dtype=np.float32),
                "grasp_sources": list(grasp_results["grasp_sources"]),
                "depth_modes": depth_modes,
                "overhead_mask_png": _encode_mask_png(overhead_mask),
                "wrist_mask_png": _encode_mask_png(wrist_mask),
                "overhead_depth_png": depth_pngs["overhead"],
                "wrist_depth_png": depth_pngs["wrist"],
                "overhead_masked_depth_png": masked_depth_pngs["overhead"],
                "wrist_masked_depth_png": masked_depth_pngs["wrist"],
                "ggcnn_quality_png": debug_blobs.get("ggcnn_quality_png", b""),
                "ggcnn_angle_png": debug_blobs.get("ggcnn_angle_png", b""),
                "ggcnn_overlay_png": debug_blobs.get("ggcnn_overlay_png", b""),
                "ggcnn_depth_input_png": debug_blobs.get("ggcnn_depth_input_png", b""),
            }

        if not candidates_by_view:
            return make_response(success=False, message=f"No usable segmented point cloud for prompt '{prompt}'")

        selected_by_view, association_strategy = select_cross_view_candidates(
            candidates_by_view,
            primary_view=self._cfg.association_primary_view,
            axis=self._cfg.association_axis,
            max_metric_distance_m=self._cfg.association_max_distance_m,
        )
        LOG.info("Selected cross-view object association: %s", association_strategy)

        view_point_results: list[np.ndarray] = []
        view_color_results: list[np.ndarray] = []
        for name, candidate in selected_by_view.items():
            masked_depth_pngs[name] = _encode_depth_png(
                candidate.depth_m,
                candidate.mask,
                label=f"{name} masked: {depth_estimates[name].mode}",
            )
            if name == "overhead":
                overhead_mask = candidate.mask
            elif name == "wrist":
                wrist_mask = candidate.mask
            LOG.info(
                "Selected %s candidate rank=%d detection=%d score=%.3f points=%d centroid=(%.3f, %.3f, %.3f)",
                name,
                candidate.rank,
                candidate.detection.index,
                candidate.detection.score,
                len(candidate.points_base),
                candidate.centroid_base[0],
                candidate.centroid_base[1],
                candidate.centroid_base[2],
            )
            view_point_results.append(candidate.points_base)
            view_color_results.append(candidate.colors)
            view_clouds[name] = candidate.points_base

        fused_cloud = np.concatenate(view_point_results, axis=0)
        fused_colors = np.concatenate(view_color_results, axis=0)
        fused_cloud = self._snap_points_to_support_plane(fused_cloud, "fused_object_cloud")
        filtered_cloud, filtered_colors = self._filter_cloud(fused_cloud, fused_colors)
        if len(filtered_cloud) == 0:
            return make_response(success=False, message="All segmented points were filtered out", object_cloud=filtered_cloud)

        try:
            grasp_results = self._propose_grasps(
                filtered_cloud,
                filtered_colors,
                top_k,
                selected_by_view=selected_by_view,
                intrinsics_by_view={name: estimate.intrinsics for name, estimate in depth_estimates.items()},
                camera_to_base_by_view={
                    "overhead": overhead_camera_to_base,
                    "wrist": wrist_camera_to_base,
                },
            )
            grasp_results = self._filter_grasps_against_support_plane(grasp_results)
        except Exception as exc:  # noqa: BLE001 - keep debug artifacts on backend/service failures.
            LOG.warning("Grasp proposal failed after debug layers were generated: %s", exc)
            return make_response(success=False, message=str(exc), object_cloud=filtered_cloud)

        return make_response(
            success=True,
            message=f"Generated {len(grasp_results['grasp_scores'])} grasp candidates",
            object_cloud=filtered_cloud,
            grasp_results=grasp_results,
        )
