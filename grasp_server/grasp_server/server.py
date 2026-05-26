"""ZeroMQ grasp perception server."""

from __future__ import annotations

import argparse
import gc
import logging
import time
from dataclasses import dataclass

import zmq

from grasp_server.pipeline import DetectionError, GraspPerceptionPipeline, PipelineConfig, PipelineError
from grasp_server.wire import decode_packet, encode_packet


@dataclass
class ServerConfig:
    host: str
    port: int
    pipeline: PipelineConfig
    cuda_empty_cache: bool


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SO-101 remote grasp perception server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host")
    parser.add_argument("--port", type=int, default=8091, help="Bind port")
    parser.add_argument("--device", default="auto", help="Inference device: auto, cuda, or cpu")
    parser.add_argument("--metric-model", default="depth-anything/DA3-LARGE-1.1", help="DA3 checkpoint path or HF model id")
    parser.add_argument("--grounding-model", default="IDEA-Research/grounding-dino-base", help="Grounding DINO checkpoint")
    parser.add_argument("--sam-model", default="facebook/sam-vit-base", help="SAM checkpoint")
    parser.add_argument("--graspnet-root", default="", help="Path to the graspnet-baseline checkout")
    parser.add_argument("--graspnet-checkpoint", default="", help="Path to the GraspNet baseline checkpoint")
    parser.add_argument("--process-res", type=int, default=504, help="DA3 processing resolution")
    parser.add_argument(
        "--da3-conditioning",
        choices=("auto", "required", "disabled"),
        default="auto",
        help="Use DA3 camera-token conditioning from calibrated intrinsics/extrinsics when the model supports it",
    )
    parser.add_argument("--da3-ref-view-strategy", default="first", help="DA3 reference-view strategy for multi-view conditioned depth")
    parser.add_argument("--da3-use-ray-pose", action=argparse.BooleanOptionalAction, default=False, help="Ask DA3 to use ray pose estimation")
    parser.add_argument("--da3-fallback-independent", action=argparse.BooleanOptionalAction, default=True, help="Fallback to per-view focal-scaled depth if conditioned DA3 fails")
    parser.add_argument("--box-threshold", type=float, default=0.25, help="Grounding DINO box threshold")
    parser.add_argument("--text-threshold", type=float, default=0.25, help="Grounding DINO text threshold")
    parser.add_argument("--min-depth-m", type=float, default=0.05, help="Minimum accepted metric depth")
    parser.add_argument("--max-depth-m", type=float, default=1.5, help="Maximum accepted metric depth")
    parser.add_argument("--voxel-size", type=float, default=0.003, help="Voxel size for object-cloud downsampling")
    parser.add_argument("--grasp-backend", choices=("graspnet", "m2t2", "ggcnn"), default="graspnet", help="Grasp generator backend")
    parser.add_argument("--graspnet-num-points", type=int, default=20000, help="Sampled points per GraspNet inference")
    parser.add_argument("--graspnet-num-view", type=int, default=300, help="GraspNet view count")
    parser.add_argument("--graspnet-collision-thresh", type=float, default=0.01, help="Collision threshold for GraspNet post-filtering")
    parser.add_argument("--graspnet-voxel-size", type=float, default=0.01, help="Voxel size for GraspNet collision detector")
    parser.add_argument("--m2t2-url", default="http://127.0.0.1:8123", help="TiPToP/M2T2 HTTP service URL")
    parser.add_argument("--m2t2-grasp-threshold", type=float, default=0.035, help="M2T2 mask threshold")
    parser.add_argument("--m2t2-num-points", type=int, default=16384, help="Point samples per M2T2 request")
    parser.add_argument("--m2t2-num-runs", type=int, default=5, help="M2T2 sampling runs")
    parser.add_argument("--m2t2-apply-bounds", action=argparse.BooleanOptionalAction, default=True, help="Let M2T2 apply its internal bounds filter")
    parser.add_argument("--m2t2-timeout-s", type=float, default=500.0, help="M2T2 HTTP request timeout")
    parser.add_argument("--m2t2-default-width", type=float, default=0.04, help="Reported gripper width for M2T2 poses")
    parser.add_argument("--ggcnn-root", default="/home/grga/Documents/ggcnn", help="Path to the GG-CNN checkout")
    parser.add_argument("--ggcnn-weights", default="/home/grga/Documents/ggcnn/ggcnn_weights_cornell/ggcnn_epoch_23_cornell_statedict.pt", help="Path to GG-CNN state dict")
    parser.add_argument("--ggcnn-primary-view", choices=("overhead", "wrist"), default="overhead", help="Segmented camera view used for planar GG-CNN inference")
    parser.add_argument("--ggcnn-input-size", type=int, default=300, help="Square GG-CNN crop size in pixels")
    parser.add_argument("--ggcnn-quality-threshold", type=float, default=0.10, help="Minimum masked GG-CNN quality peak")
    parser.add_argument("--ggcnn-peak-min-distance-px", type=int, default=24, help="Minimum pixel distance between GG-CNN peaks")
    parser.add_argument("--ggcnn-crop-padding", type=float, default=1.8, help="Object-mask crop padding factor before GG-CNN resize")
    parser.add_argument("--ggcnn-min-width-m", type=float, default=0.015, help="Minimum reported physical grasp width")
    parser.add_argument("--ggcnn-max-width-m", type=float, default=0.075, help="Maximum reported physical grasp width")
    parser.add_argument("--ggcnn-local-z-radius-m", type=float, default=0.035, help="Object-cloud radius used to refine GG-CNN grasp height")
    parser.add_argument("--ggcnn-local-z-percentile", type=float, default=70.0, help="Object-cloud z percentile used for GG-CNN grasp height")
    parser.add_argument("--max-detections-per-view", type=int, default=6, help="Maximum prompt detections segmented per camera for cross-view association")
    parser.add_argument("--association-max-distance-m", type=float, default=0.12, help="Use metric centroid association only below this base-frame distance")
    parser.add_argument("--association-primary-view", choices=("overhead", "wrist"), default="wrist", help="Primary view to keep when calibrated cross-view metric association is untrusted")
    parser.add_argument("--association-axis", choices=("x", "y"), default="x", help="Image axis used for same-rank fallback association")
    parser.add_argument(
        "--workspace-bounds",
        type=float,
        nargs=6,
        default=(-0.30, 0.30, -0.30, 0.30, -0.05, 0.45),
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help="Workspace crop bounds in follower/base_link",
    )
    parser.add_argument("--support-plane-yaml", default="", help="Board/table plane calibration YAML in follower/base_link")
    parser.add_argument("--support-plane-snap", action=argparse.BooleanOptionalAction, default=False, help="Snap segmented object clouds upward to the calibrated support plane")
    parser.add_argument("--support-plane-clearance-m", type=float, default=0.003, help="Target lower-percentile object clearance above the support plane after snapping")
    parser.add_argument("--support-plane-snap-min-correction-m", type=float, default=0.005, help="Minimum support-plane correction to apply")
    parser.add_argument("--support-plane-snap-max-correction-m", type=float, default=0.08, help="Maximum support-plane correction allowed")
    parser.add_argument("--support-plane-min-grasp-clearance-m", type=float, default=-0.002, help="Reject grasp centers below this signed support-plane clearance")
    parser.add_argument("--cuda-empty-cache", action=argparse.BooleanOptionalAction, default=True, help="Run Python GC and torch.cuda.empty_cache() after each request")
    return parser.parse_args()


def _cleanup_request_memory(*, cuda_empty_cache: bool) -> None:
    gc.collect()
    if not cuda_empty_cache:
        return
    try:
        import torch
    except ImportError:
        return
    if not torch.cuda.is_available():
        return
    before_alloc = torch.cuda.memory_allocated()
    before_reserved = torch.cuda.memory_reserved()
    torch.cuda.empty_cache()
    try:
        torch.cuda.ipc_collect()
    except Exception:  # noqa: BLE001 - IPC collection is best-effort across torch/CUDA versions.
        pass
    after_alloc = torch.cuda.memory_allocated()
    after_reserved = torch.cuda.memory_reserved()
    logging.info(
        "CUDA cleanup: allocated %.1f->%.1f MiB, reserved %.1f->%.1f MiB",
        before_alloc / 1048576.0,
        after_alloc / 1048576.0,
        before_reserved / 1048576.0,
        after_reserved / 1048576.0,
    )


class ZmqGraspServer:
    def __init__(self, config: ServerConfig) -> None:
        self._config = config
        self._pipeline = GraspPerceptionPipeline(config.pipeline)

    def _handle_detect(self, message: dict) -> bytes:
        request = {
            "prompt": str(message["scalars"]["prompt"]),
            "top_k": int(message["scalars"]["top_k"]),
            "overhead_intrinsics": message["arrays"]["overhead_intrinsics"],
            "wrist_intrinsics": message["arrays"]["wrist_intrinsics"],
            "overhead_camera_to_base": message["arrays"]["overhead_camera_to_base"],
            "wrist_camera_to_base": message["arrays"]["wrist_camera_to_base"],
            "overhead_jpeg": message["blobs"]["overhead_jpeg"],
            "wrist_jpeg": message["blobs"]["wrist_jpeg"],
        }
        t0 = time.perf_counter()
        result = self._pipeline.detect_grasps(request)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        logging.info("Handled detect_grasps in %.1f ms", elapsed_ms)
        return encode_packet(
            "detect_grasps_response",
            scalars={
                "success": bool(result.get("success", True)),
                "message": result["message"],
                "grasp_sources": result["grasp_sources"],
                "depth_modes": result.get("depth_modes", {}),
            },
            arrays={
                "overhead_object_cloud": result["overhead_object_cloud"],
                "wrist_object_cloud": result["wrist_object_cloud"],
                "object_cloud": result["object_cloud"],
                "grasp_positions": result["grasp_positions"],
                "grasp_quaternions": result["grasp_quaternions"],
                "grasp_scores": result["grasp_scores"],
                "grasp_widths": result["grasp_widths"],
            },
            blobs={
                "overhead_mask_png": result["overhead_mask_png"],
                "wrist_mask_png": result["wrist_mask_png"],
                "overhead_depth_png": result["overhead_depth_png"],
                "wrist_depth_png": result["wrist_depth_png"],
                "overhead_masked_depth_png": result["overhead_masked_depth_png"],
                "wrist_masked_depth_png": result["wrist_masked_depth_png"],
                "ggcnn_quality_png": result.get("ggcnn_quality_png", b""),
                "ggcnn_angle_png": result.get("ggcnn_angle_png", b""),
                "ggcnn_overlay_png": result.get("ggcnn_overlay_png", b""),
                "ggcnn_depth_input_png": result.get("ggcnn_depth_input_png", b""),
            },
        )

    def serve(self) -> None:
        ctx = zmq.Context()
        rep = ctx.socket(zmq.REP)
        rep.bind(f"tcp://{self._config.host}:{self._config.port}")
        logging.info("grasp_server listening on %s:%d", self._config.host, self._config.port)

        try:
            while True:
                raw = rep.recv()
                try:
                    message = decode_packet(raw)
                    if message["type"] != "detect_grasps":
                        raise PipelineError(f"Unknown request type: {message['type']}")
                    reply = self._handle_detect(message)
                except (PipelineError, DetectionError) as exc:
                    logging.warning("Request failed: %s", exc)
                    reply = encode_packet(
                        "detect_grasps_response",
                        scalars={"success": False, "message": str(exc), "grasp_sources": []},
                        arrays={},
                        blobs={},
                    )
                except Exception as exc:  # pragma: no cover - fatal errors still return cleanly
                    logging.exception("Unexpected server error")
                    reply = encode_packet(
                        "detect_grasps_response",
                        scalars={"success": False, "message": str(exc), "grasp_sources": []},
                        arrays={},
                        blobs={},
                    )
                rep.send(reply)
                _cleanup_request_memory(cuda_empty_cache=self._config.cuda_empty_cache)
        finally:
            rep.close(linger=0)
            ctx.term()


def main() -> None:
    args = _parse_args()
    logging.basicConfig(level=logging.INFO)

    pipeline_config = PipelineConfig(
        metric_model=args.metric_model,
        grounding_model=args.grounding_model,
        sam_model=args.sam_model,
        graspnet_root=args.graspnet_root,
        graspnet_checkpoint=args.graspnet_checkpoint,
        device=args.device,
        process_res=args.process_res,
        da3_conditioning=args.da3_conditioning,
        da3_ref_view_strategy=args.da3_ref_view_strategy,
        da3_use_ray_pose=args.da3_use_ray_pose,
        da3_fallback_independent=args.da3_fallback_independent,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        min_depth_m=args.min_depth_m,
        max_depth_m=args.max_depth_m,
        voxel_size=args.voxel_size,
        grasp_backend=args.grasp_backend,
        graspnet_num_points=args.graspnet_num_points,
        graspnet_num_view=args.graspnet_num_view,
        graspnet_collision_thresh=args.graspnet_collision_thresh,
        graspnet_voxel_size=args.graspnet_voxel_size,
        m2t2_url=args.m2t2_url,
        m2t2_grasp_threshold=args.m2t2_grasp_threshold,
        m2t2_num_points=args.m2t2_num_points,
        m2t2_num_runs=args.m2t2_num_runs,
        m2t2_apply_bounds=args.m2t2_apply_bounds,
        m2t2_timeout_s=args.m2t2_timeout_s,
        m2t2_default_width=args.m2t2_default_width,
        ggcnn_root=args.ggcnn_root,
        ggcnn_weights=args.ggcnn_weights,
        ggcnn_primary_view=args.ggcnn_primary_view,
        ggcnn_input_size=args.ggcnn_input_size,
        ggcnn_quality_threshold=args.ggcnn_quality_threshold,
        ggcnn_peak_min_distance_px=args.ggcnn_peak_min_distance_px,
        ggcnn_crop_padding=args.ggcnn_crop_padding,
        ggcnn_min_width_m=args.ggcnn_min_width_m,
        ggcnn_max_width_m=args.ggcnn_max_width_m,
        ggcnn_local_z_radius_m=args.ggcnn_local_z_radius_m,
        ggcnn_local_z_percentile=args.ggcnn_local_z_percentile,
        workspace_bounds=tuple(float(v) for v in args.workspace_bounds),
        support_plane_yaml=args.support_plane_yaml,
        support_plane_snap=args.support_plane_snap,
        support_plane_clearance_m=args.support_plane_clearance_m,
        support_plane_snap_min_correction_m=args.support_plane_snap_min_correction_m,
        support_plane_snap_max_correction_m=args.support_plane_snap_max_correction_m,
        support_plane_min_grasp_clearance_m=args.support_plane_min_grasp_clearance_m,
        max_detections_per_view=args.max_detections_per_view,
        association_max_distance_m=args.association_max_distance_m,
        association_primary_view=args.association_primary_view,
        association_axis=args.association_axis,
    )
    server = ZmqGraspServer(
        ServerConfig(
            host=args.host,
            port=args.port,
            pipeline=pipeline_config,
            cuda_empty_cache=args.cuda_empty_cache,
        )
    )
    server.serve()
