#!/usr/bin/env bash
set -euo pipefail

VENV_PATH="${VENV_PATH:-/home/grga/Documents/Depth-Anything-3/.venv}"
REPO_ROOT="${REPO_ROOT:-/home/grga/Documents/so101-ros-physical-ai}"
GRASPNET_ROOT="${GRASPNET_ROOT:-/home/grga/Documents/graspnet-baseline}"
GRASPNET_CHECKPOINT="${GRASPNET_CHECKPOINT:-${GRASPNET_ROOT}/checkpoint-rs.tar}"

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8091}"
DEVICE="${DEVICE:-auto}"
METRIC_MODEL="${METRIC_MODEL:-/home/grga/Documents/Depth-Anything-3/models/DA3-LARGE-1.1}"
GROUNDING_MODEL="${GROUNDING_MODEL:-IDEA-Research/grounding-dino-base}"
SAM_MODEL="${SAM_MODEL:-facebook/sam-vit-base}"
PROCESS_RES="${PROCESS_RES:-392}"
DA3_CONDITIONING="${DA3_CONDITIONING:-auto}"
DA3_REF_VIEW_STRATEGY="${DA3_REF_VIEW_STRATEGY:-first}"
DA3_USE_RAY_POSE="${DA3_USE_RAY_POSE:-false}"
DA3_FALLBACK_INDEPENDENT="${DA3_FALLBACK_INDEPENDENT:-true}"
BOX_THRESHOLD="${BOX_THRESHOLD:-0.25}"
TEXT_THRESHOLD="${TEXT_THRESHOLD:-0.25}"
GRASP_BACKEND="${GRASP_BACKEND:-graspnet}"
M2T2_URL="${M2T2_URL:-http://127.0.0.1:8123}"
M2T2_GRASP_THRESHOLD="${M2T2_GRASP_THRESHOLD:-0.035}"
M2T2_NUM_POINTS="${M2T2_NUM_POINTS:-16384}"
M2T2_NUM_RUNS="${M2T2_NUM_RUNS:-5}"
M2T2_APPLY_BOUNDS="${M2T2_APPLY_BOUNDS:-true}"
M2T2_TIMEOUT_S="${M2T2_TIMEOUT_S:-500}"
M2T2_DEFAULT_WIDTH="${M2T2_DEFAULT_WIDTH:-0.04}"
GGCNN_ROOT="${GGCNN_ROOT:-/home/grga/Documents/ggcnn}"
GGCNN_WEIGHTS="${GGCNN_WEIGHTS:-${GGCNN_ROOT}/ggcnn_weights_cornell/ggcnn_epoch_23_cornell_statedict.pt}"
GGCNN_PRIMARY_VIEW="${GGCNN_PRIMARY_VIEW:-wrist}"
GGCNN_INPUT_SIZE="${GGCNN_INPUT_SIZE:-300}"
GGCNN_QUALITY_THRESHOLD="${GGCNN_QUALITY_THRESHOLD:-0.10}"
GGCNN_PEAK_MIN_DISTANCE_PX="${GGCNN_PEAK_MIN_DISTANCE_PX:-24}"
GGCNN_CROP_PADDING="${GGCNN_CROP_PADDING:-1.8}"
GGCNN_MIN_WIDTH_M="${GGCNN_MIN_WIDTH_M:-0.015}"
GGCNN_MAX_WIDTH_M="${GGCNN_MAX_WIDTH_M:-0.075}"
GGCNN_LOCAL_Z_RADIUS_M="${GGCNN_LOCAL_Z_RADIUS_M:-0.035}"
GGCNN_LOCAL_Z_PERCENTILE="${GGCNN_LOCAL_Z_PERCENTILE:-70.0}"
MAX_DETECTIONS_PER_VIEW="${MAX_DETECTIONS_PER_VIEW:-6}"
ASSOCIATION_MAX_DISTANCE_M="${ASSOCIATION_MAX_DISTANCE_M:-0.12}"
ASSOCIATION_PRIMARY_VIEW="${ASSOCIATION_PRIMARY_VIEW:-wrist}"
ASSOCIATION_AXIS="${ASSOCIATION_AXIS:-x}"
WORKSPACE_BOUNDS="${WORKSPACE_BOUNDS:--0.60 0.80 -0.60 0.60 -0.40 0.80}"
SUPPORT_PLANE_YAML="${SUPPORT_PLANE_YAML:-${REPO_ROOT}/so101_bringup/config/cameras/extrinsics/board_in_base_touch.yaml}"
SUPPORT_PLANE_SNAP="${SUPPORT_PLANE_SNAP:-true}"
SUPPORT_PLANE_CLEARANCE_M="${SUPPORT_PLANE_CLEARANCE_M:-0.003}"
SUPPORT_PLANE_SNAP_MIN_CORRECTION_M="${SUPPORT_PLANE_SNAP_MIN_CORRECTION_M:-0.005}"
SUPPORT_PLANE_SNAP_MAX_CORRECTION_M="${SUPPORT_PLANE_SNAP_MAX_CORRECTION_M:-0.40}"
SUPPORT_PLANE_MIN_GRASP_CLEARANCE_M="${SUPPORT_PLANE_MIN_GRASP_CLEARANCE_M:--0.30}"

if [[ ! -x "${VENV_PATH}/bin/grasp-server" ]]; then
    echo "grasp-server console script not found in ${VENV_PATH}" >&2
    exit 1
fi

export PYTHONUNBUFFERED=1
export PYTHONPATH="${REPO_ROOT}/grasp_server:${PYTHONPATH:-}"

if [[ "${M2T2_APPLY_BOUNDS}" == "0" || "${M2T2_APPLY_BOUNDS}" == "false" || "${M2T2_APPLY_BOUNDS}" == "False" ]]; then
    M2T2_APPLY_BOUNDS_FLAG="--no-m2t2-apply-bounds"
else
    M2T2_APPLY_BOUNDS_FLAG="--m2t2-apply-bounds"
fi
if [[ "${DA3_USE_RAY_POSE}" == "1" || "${DA3_USE_RAY_POSE}" == "true" || "${DA3_USE_RAY_POSE}" == "True" ]]; then
    DA3_USE_RAY_POSE_FLAG="--da3-use-ray-pose"
else
    DA3_USE_RAY_POSE_FLAG="--no-da3-use-ray-pose"
fi
if [[ "${DA3_FALLBACK_INDEPENDENT}" == "0" || "${DA3_FALLBACK_INDEPENDENT}" == "false" || "${DA3_FALLBACK_INDEPENDENT}" == "False" ]]; then
    DA3_FALLBACK_INDEPENDENT_FLAG="--no-da3-fallback-independent"
else
    DA3_FALLBACK_INDEPENDENT_FLAG="--da3-fallback-independent"
fi
if [[ "${SUPPORT_PLANE_SNAP}" == "0" || "${SUPPORT_PLANE_SNAP}" == "false" || "${SUPPORT_PLANE_SNAP}" == "False" ]]; then
    SUPPORT_PLANE_SNAP_FLAG="--no-support-plane-snap"
else
    SUPPORT_PLANE_SNAP_FLAG="--support-plane-snap"
fi
read -r -a WORKSPACE_BOUNDS_ARGS <<< "${WORKSPACE_BOUNDS}"

exec "${VENV_PATH}/bin/grasp-server" \
    --host "${HOST}" \
    --port "${PORT}" \
    --device "${DEVICE}" \
    --metric-model "${METRIC_MODEL}" \
    --grounding-model "${GROUNDING_MODEL}" \
    --sam-model "${SAM_MODEL}" \
    --graspnet-root "${GRASPNET_ROOT}" \
    --graspnet-checkpoint "${GRASPNET_CHECKPOINT}" \
    --process-res "${PROCESS_RES}" \
    --da3-conditioning "${DA3_CONDITIONING}" \
    --da3-ref-view-strategy "${DA3_REF_VIEW_STRATEGY}" \
    "${DA3_USE_RAY_POSE_FLAG}" \
    "${DA3_FALLBACK_INDEPENDENT_FLAG}" \
    --box-threshold "${BOX_THRESHOLD}" \
    --text-threshold "${TEXT_THRESHOLD}" \
    --grasp-backend "${GRASP_BACKEND}" \
    --m2t2-url "${M2T2_URL}" \
    --m2t2-grasp-threshold "${M2T2_GRASP_THRESHOLD}" \
    --m2t2-num-points "${M2T2_NUM_POINTS}" \
    --m2t2-num-runs "${M2T2_NUM_RUNS}" \
    "${M2T2_APPLY_BOUNDS_FLAG}" \
    --m2t2-timeout-s "${M2T2_TIMEOUT_S}" \
    --m2t2-default-width "${M2T2_DEFAULT_WIDTH}" \
    --ggcnn-root "${GGCNN_ROOT}" \
    --ggcnn-weights "${GGCNN_WEIGHTS}" \
    --ggcnn-primary-view "${GGCNN_PRIMARY_VIEW}" \
    --ggcnn-input-size "${GGCNN_INPUT_SIZE}" \
    --ggcnn-quality-threshold "${GGCNN_QUALITY_THRESHOLD}" \
    --ggcnn-peak-min-distance-px "${GGCNN_PEAK_MIN_DISTANCE_PX}" \
    --ggcnn-crop-padding "${GGCNN_CROP_PADDING}" \
    --ggcnn-min-width-m "${GGCNN_MIN_WIDTH_M}" \
    --ggcnn-max-width-m "${GGCNN_MAX_WIDTH_M}" \
    --ggcnn-local-z-radius-m "${GGCNN_LOCAL_Z_RADIUS_M}" \
    --ggcnn-local-z-percentile "${GGCNN_LOCAL_Z_PERCENTILE}" \
    --max-detections-per-view "${MAX_DETECTIONS_PER_VIEW}" \
    --association-max-distance-m "${ASSOCIATION_MAX_DISTANCE_M}" \
    --association-primary-view "${ASSOCIATION_PRIMARY_VIEW}" \
    --association-axis "${ASSOCIATION_AXIS}" \
    --support-plane-yaml "${SUPPORT_PLANE_YAML}" \
    "${SUPPORT_PLANE_SNAP_FLAG}" \
    --support-plane-clearance-m "${SUPPORT_PLANE_CLEARANCE_M}" \
    --support-plane-snap-min-correction-m "${SUPPORT_PLANE_SNAP_MIN_CORRECTION_M}" \
    --support-plane-snap-max-correction-m "${SUPPORT_PLANE_SNAP_MAX_CORRECTION_M}" \
    --support-plane-min-grasp-clearance-m "${SUPPORT_PLANE_MIN_GRASP_CLEARANCE_M}" \
    --workspace-bounds "${WORKSPACE_BOUNDS_ARGS[@]}"
