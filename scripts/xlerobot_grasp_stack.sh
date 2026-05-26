#!/usr/bin/env bash
set -euo pipefail

XLEROBOT_WS="${XLEROBOT_WS:-/home/dell/Documents/xlerobot-so101-stack}"
if [[ -z "${XLEROBOT_RUN:-}" ]]; then
  XLEROBOT_RUN="$(find "${XLEROBOT_WS}/field_runs" -maxdepth 1 -type d -name 'xlerobot_*' | sort | tail -n 1)"
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
GRASP_SERVER_ADDRESS="${GRASP_SERVER_ADDRESS:-127.0.0.1:8091}"
LOG_DIR="${XLEROBOT_RUN}/logs"
PID_DIR="${XLEROBOT_RUN}/pids"
DEFAULT_PROMPT="${DEFAULT_PROMPT:-pink cube}"
DEFAULT_TOP_K="${DEFAULT_TOP_K:-8}"
DETECT_CALL_TIMEOUT_S="${DETECT_CALL_TIMEOUT_S:-180}"
PLAN_CALL_TIMEOUT_S="${PLAN_CALL_TIMEOUT_S:-180}"
EXECUTE_CALL_TIMEOUT_S="${EXECUTE_CALL_TIMEOUT_S:-300}"
VERIFY_BOARD_BEFORE_EXECUTE="${VERIFY_BOARD_BEFORE_EXECUTE:-true}"
EXECUTION_BACKEND="${EXECUTION_BACKEND:-feedback}"
WRIST_REFINE_BEFORE_GRASP="${WRIST_REFINE_BEFORE_GRASP:-false}"
PREFER_LOW_WRIST_ROLL="${PREFER_LOW_WRIST_ROLL:-true}"
PREFERRED_WRIST_ROLL_DELTA_RAD="${PREFERRED_WRIST_ROLL_DELTA_RAD:-0.35}"
MAX_WRIST_ROLL_DELTA_RAD="${MAX_WRIST_ROLL_DELTA_RAD:-0.90}"
FEEDBACK_POSITION_TOLERANCE_M="${FEEDBACK_POSITION_TOLERANCE_M:-0.015}"
FEEDBACK_MAX_CORRECTION_ITERS="${FEEDBACK_MAX_CORRECTION_ITERS:-4}"
FEEDBACK_MIN_JOINT_DELTA_RAD="${FEEDBACK_MIN_JOINT_DELTA_RAD:-0.035}"
WRIST_CONFIRMATION_BEFORE_DESCENT="${WRIST_CONFIRMATION_BEFORE_DESCENT:-true}"
REQUIRE_WRIST_CONFIRMATION_FOR_EXECUTION="${REQUIRE_WRIST_CONFIRMATION_FOR_EXECUTION:-false}"
REQUIRE_WRIST_CLOUD_FOR_EXECUTION="${REQUIRE_WRIST_CLOUD_FOR_EXECUTION:-false}"
BOARD_VERIFY_PNP_REPROJECTION_ERROR_PX="${BOARD_VERIFY_PNP_REPROJECTION_ERROR_PX:-8.0}"
BOARD_VERIFY_MIN_MARKERS="${BOARD_VERIFY_MIN_MARKERS:-8}"
BOARD_VERIFY_MIN_INLIER_POINTS="${BOARD_VERIFY_MIN_INLIER_POINTS:-24}"
BOARD_VERIFY_MAX_MEAN_PX="${BOARD_VERIFY_MAX_MEAN_PX:-4.5}"
BOARD_VERIFY_MAX_TRANSLATION_M="${BOARD_VERIFY_MAX_TRANSLATION_M:-0.05}"
BOARD_VERIFY_MAX_ROTATION_DEG="${BOARD_VERIFY_MAX_ROTATION_DEG:-8.0}"
EXECUTE_SIDE="${EXECUTE_SIDE:-right}"
ALLOW_NONDEFAULT_EXECUTE="${ALLOW_NONDEFAULT_EXECUTE:-false}"

usage() {
  cat <<EOF
usage: $0 up|detect|plan|execute|snapshot|verify-board|status|down left|right [prompt]

Examples:
  $0 up left
  $0 detect left "pink cube"
  $0 plan left "pink cube"
  $0 execute left "pink cube"
  $0 down left
EOF
}

side="${2:-}"
if [[ "${side}" != "left" && "${side}" != "right" && "${1:-}" != "status" ]]; then
  usage
  exit 2
fi
prompt="${3:-${DEFAULT_PROMPT}}"

ensure_dirs() {
  mkdir -p "${LOG_DIR}" "${PID_DIR}" "${XLEROBOT_RUN}/grasp/${side}"
}

source_ros() {
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1091
  source "${XLEROBOT_WS}/install/setup.bash"
  set -u
}

state_value() {
  python3 - "$XLEROBOT_RUN/run_state.yaml" "$1" <<'PY'
import sys, yaml
state = yaml.safe_load(open(sys.argv[1], "r", encoding="utf-8")) or {}
print(state.get("config", {}).get(sys.argv[2], ""))
PY
}

ensure_runtime_config() {
  source_ros
  ros2 run xlerobot_calibration xlerobot-calib \
    --workspace "${XLEROBOT_WS}" \
    --out "${XLEROBOT_RUN}" \
    run-step generate_grasp_runtime_config \
    --yes >/dev/null
}

start_stack() {
  ensure_dirs
  ensure_runtime_config
  if [[ -s "${PID_DIR}/grasp_${side}.pid" ]] && kill -0 "$(cat "${PID_DIR}/grasp_${side}.pid")" 2>/dev/null; then
    echo "${side} grasp stack already running: pid $(cat "${PID_DIR}/grasp_${side}.pid")"
    return
  fi
  source_ros
  local port
  port="$(state_value "${side}_port")"
  if [[ -z "${port}" ]]; then
    echo "missing ${side}_port in ${XLEROBOT_RUN}/run_state.yaml" >&2
    exit 1
  fi
  (
    cd "${XLEROBOT_WS}"
    ros2 launch so101_bringup xlerobot_grasp_runtime.launch.py \
      side:="${side}" \
      out_dir:="${XLEROBOT_RUN}" \
      "${side}_port:=${port}" \
      grasp_server_address:="${GRASP_SERVER_ADDRESS}" \
      allow_execution:=false \
      execution_backend:="${EXECUTION_BACKEND}" \
      use_rviz:=false
  ) >"${LOG_DIR}/${side}_grasp_stack.log" 2>&1 &
  echo $! > "${PID_DIR}/grasp_${side}.pid"
  sleep 3
  if ! kill -0 "$(cat "${PID_DIR}/grasp_${side}.pid")" 2>/dev/null; then
    echo "${side} grasp stack exited during startup; tail follows:" >&2
    tail -80 "${LOG_DIR}/${side}_grasp_stack.log" >&2 || true
    rm -f "${PID_DIR}/grasp_${side}.pid"
    exit 1
  fi
  echo "started ${side} grasp stack: pid $(cat "${PID_DIR}/grasp_${side}.pid")"
  echo "log: ${LOG_DIR}/${side}_grasp_stack.log"
  wait_for_runtime_tf 45
}

stop_stack() {
  ensure_dirs
  local pid_file="${PID_DIR}/grasp_${side}.pid"
  if [[ -s "${pid_file}" ]]; then
    local pid
    pid="$(cat "${pid_file}")"
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      sleep 2
      kill -9 "${pid}" 2>/dev/null || true
    fi
    rm -f "${pid_file}"
  fi
  pkill -TERM -f "xlerobot_grasp_runtime.launch.py.*side:=${side}" 2>/dev/null || true
  pkill -TERM -f "__ns:=/${side}_grasp" 2>/dev/null || true
  pkill -TERM -f "__ns:=/${side}" 2>/dev/null || true
  pkill -TERM -f "xlerobot_opencv_cam.yaml" 2>/dev/null || true
  pkill -TERM -f "__node:=xlerobot_" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "xlerobot_grasp_runtime.launch.py.*side:=${side}" 2>/dev/null || true
  pkill -KILL -f "__ns:=/${side}_grasp" 2>/dev/null || true
  pkill -KILL -f "__ns:=/${side}" 2>/dev/null || true
  pkill -KILL -f "xlerobot_opencv_cam.yaml" 2>/dev/null || true
  pkill -KILL -f "__node:=xlerobot_" 2>/dev/null || true
}

wait_for_service() {
  source_ros
  local service="$1"
  local deadline=$((SECONDS + 45))
  until ros2 service list | grep -qx "${service}"; do
    if (( SECONDS > deadline )); then
      echo "service not available: ${service}" >&2
      tail -80 "${LOG_DIR}/${side}_grasp_stack.log" >&2 || true
      exit 1
    fi
    sleep 1
  done
}

wait_for_runtime_tf() {
  source_ros
  local timeout_s="${1:-45}"
  python3 - "${side}" "${timeout_s}" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

side = sys.argv[1]
timeout_s = float(sys.argv[2])
pairs = [
    ("world", f"{side}/base_link"),
    (f"{side}/base_link", f"{side}/gripper_frame_link"),
    ("world", f"{side}/wrist_camera_optical_frame"),
    ("world", "center_gopro_optical_frame"),
]

rclpy.init()
node = Node(f"wait_{side}_grasp_runtime_tf")
tf_buffer = Buffer()
TransformListener(tf_buffer, node)
deadline = time.monotonic() + timeout_s
last_errors = {}
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready = True
        for parent, child in pairs:
            try:
                tf_buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.05))
            except Exception as exc:  # noqa: BLE001 - print the concrete TF wait reason.
                last_errors[(parent, child)] = str(exc)
                ready = False
                break
        if ready:
            print(f"{side} runtime TF ready")
            raise SystemExit(0)
    print(f"{side} runtime TF not connected after {timeout_s:.1f}s", file=sys.stderr)
    for parent, child in pairs:
        print(f"  {parent} <- {child}: {last_errors.get((parent, child), 'not checked')}", file=sys.stderr)
    raise SystemExit(1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

call_ros_service() {
  local timeout_s="$1"
  local output_path="$2"
  shift 2
  local stamp archive_path
  local -a tee_paths
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  archive_path="${output_path}"
  if [[ "${output_path}" == *.txt ]]; then
    archive_path="${output_path%.txt}_${stamp}.txt"
  fi
  tee_paths=("${output_path}")
  if [[ "${archive_path}" != "${output_path}" ]]; then
    tee_paths+=("${archive_path}")
  fi
  local rc
  set +e
  timeout --foreground "${timeout_s}" ros2 service call "$@" | tee "${tee_paths[@]}"
  rc=${PIPESTATUS[0]}
  set -e
  if (( rc != 0 )); then
    echo "service call failed or timed out after ${timeout_s}s: $*" >&2
    tail -120 "${LOG_DIR}/${side}_grasp_stack.log" >&2 || true
    exit "${rc}"
  fi
}

apply_planner_runtime_params() {
  ros2 param set "/${side}_grasp/grasp_planner_node" execution_backend "${EXECUTION_BACKEND}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" prefer_low_wrist_roll "${PREFER_LOW_WRIST_ROLL}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" preferred_wrist_roll_delta_rad "${PREFERRED_WRIST_ROLL_DELTA_RAD}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" max_wrist_roll_delta_rad "${MAX_WRIST_ROLL_DELTA_RAD}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" feedback_position_tolerance_m "${FEEDBACK_POSITION_TOLERANCE_M}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" feedback_max_correction_iters "${FEEDBACK_MAX_CORRECTION_ITERS}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" feedback_min_joint_delta_rad "${FEEDBACK_MIN_JOINT_DELTA_RAD}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" wrist_confirmation_before_descent "${WRIST_CONFIRMATION_BEFORE_DESCENT}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" require_wrist_confirmation_for_execution "${REQUIRE_WRIST_CONFIRMATION_FOR_EXECUTION}" >/dev/null
  ros2 param set "/${side}_grasp/grasp_planner_node" require_wrist_cloud_for_execution "${REQUIRE_WRIST_CLOUD_FOR_EXECUTION}" >/dev/null
}

detect() {
  ensure_dirs
  source_ros
  wait_for_service "/${side}_grasp/detect_grasps"
  wait_for_runtime_tf 20
  call_ros_service "${DETECT_CALL_TIMEOUT_S}" "${LOG_DIR}/${side}_detect_grasps.txt" \
    "/${side}_grasp/detect_grasps" so101_grasp_msgs/srv/DetectGrasps \
    "{prompt: '${prompt}', top_k: ${DEFAULT_TOP_K}}"
}

plan() {
  ensure_dirs
  source_ros
  wait_for_service "/${side}_grasp/plan_grasp"
  wait_for_runtime_tf 20
  apply_planner_runtime_params
  echo "execution_backend=${EXECUTION_BACKEND}"
  echo "wrist_roll prefer_low=${PREFER_LOW_WRIST_ROLL} preferred_delta=${PREFERRED_WRIST_ROLL_DELTA_RAD} max_delta=${MAX_WRIST_ROLL_DELTA_RAD}"
  echo "feedback tolerance=${FEEDBACK_POSITION_TOLERANCE_M}m corrections=${FEEDBACK_MAX_CORRECTION_ITERS} min_joint_delta=${FEEDBACK_MIN_JOINT_DELTA_RAD}rad"
  echo "wrist_confirmation before_descent=${WRIST_CONFIRMATION_BEFORE_DESCENT} require_match=${REQUIRE_WRIST_CONFIRMATION_FOR_EXECUTION} require_cloud=${REQUIRE_WRIST_CLOUD_FOR_EXECUTION}"
  call_ros_service "${PLAN_CALL_TIMEOUT_S}" "${LOG_DIR}/${side}_plan_grasp.txt" \
    "/${side}_grasp/plan_grasp" so101_grasp_msgs/srv/PlanGrasp \
    "{prompt: '${prompt}', top_k: ${DEFAULT_TOP_K}, grasp_index: 0, execute: false, pregrasp_offset_m: 0.10, plan_time_s: 30.0}"
}

verify_board() {
  ensure_dirs
  source_ros
  local out_dir="${XLEROBOT_RUN}/grasp/${side}/board_verify"
  mkdir -p "${out_dir}"
  local stamp image output overlay frame
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  image="${out_dir}/center_gopro_world_board_${stamp}.jpg"
  output="${out_dir}/center_gopro_in_world_${stamp}.yaml"
  overlay="${out_dir}/center_gopro_world_board_overlay_${stamp}.jpg"
  frame="${out_dir}/center_gopro_world_board_frame_${stamp}.jpg"
  local gopro_dev
  gopro_dev="$(state_value center_gopro_dev)"
  ffmpeg -y -f v4l2 -input_format yuyv422 -video_size 1280x720 \
    -i "${gopro_dev}" -frames:v 1 "${image}" >/dev/null 2>"${out_dir}/ffmpeg_${stamp}.log"
  python3 "${XLEROBOT_WS}/scripts/solve_camera_extrinsics_from_board.py" \
    --image "${image}" \
    --camera-name center_gopro_optical_frame \
    --camera-info "$(state_value center_gopro_info)" \
    --board-in-base "${XLEROBOT_RUN}/extrinsics/world_board_identity.yaml" \
    --output "${output}" \
    --overlay-output "${overlay}" \
    --frame-output "${frame}" \
    --parent-frame world \
    --cols "$(state_value world_cols)" \
    --rows "$(state_value world_rows)" \
    --square-m "$(state_value world_square_m)" \
    --marker-m "$(state_value world_marker_m)" \
    --start-id "$(state_value world_start_id)" \
    --marker-count "$(state_value world_marker_count)" \
    --aruco-dict "$(state_value world_dict)" \
    --min-markers "${BOARD_VERIFY_MIN_MARKERS}" \
    --pnp-reprojection-error-px "${BOARD_VERIFY_PNP_REPROJECTION_ERROR_PX}" | tee "${out_dir}/solve_${stamp}.log"
  python3 - "${XLEROBOT_RUN}/extrinsics/center_gopro_in_world.yaml" "${output}" \
    "${BOARD_VERIFY_MAX_MEAN_PX}" "${BOARD_VERIFY_MAX_TRANSLATION_M}" "${BOARD_VERIFY_MAX_ROTATION_DEG}" \
    "${BOARD_VERIFY_MIN_MARKERS}" "${BOARD_VERIFY_MIN_INLIER_POINTS}" <<'PY'
import math, sys, yaml
import numpy as np

base = yaml.safe_load(open(sys.argv[1], "r", encoding="utf-8"))
new = yaml.safe_load(open(sys.argv[2], "r", encoding="utf-8"))
max_mean = float(sys.argv[3])
max_translation = float(sys.argv[4])
max_rotation = math.radians(float(sys.argv[5]))
min_markers = int(sys.argv[6])
min_inliers = int(sys.argv[7])

def mat(data):
    t = data["transform"]
    m = np.eye(4)
    m[:3, :3] = np.asarray(t["rotation_matrix"], dtype=float)
    m[:3, 3] = np.asarray(t["translation_xyz"], dtype=float)
    return m

base_m = mat(base)
new_m = mat(new)
delta = np.linalg.inv(base_m) @ new_m
translation = float(np.linalg.norm(delta[:3, 3]))
trace = float(np.clip((np.trace(delta[:3, :3]) - 1.0) * 0.5, -1.0, 1.0))
rotation = float(math.acos(trace))
quality = new.get("quality", {})
markers = int(quality.get("detected_markers", 0))
inliers = int(quality.get("inlier_points", 0))
total = int(quality.get("total_points", 0))
mean = float(quality.get("mean_reprojection_error_px", 999.0))
print(
    f"board verify: markers={markers} inliers={inliers}/{total} mean={mean:.3f}px "
    f"translation_delta={translation:.4f}m rotation_delta={math.degrees(rotation):.2f}deg"
)
reasons = []
if markers < min_markers:
    reasons.append(f"markers {markers} < {min_markers}")
if inliers < min_inliers:
    reasons.append(f"inlier points {inliers} < {min_inliers}")
if mean > max_mean:
    reasons.append(f"mean reprojection {mean:.3f}px > {max_mean:.3f}px")
if translation > max_translation:
    reasons.append(f"translation delta {translation:.4f}m > {max_translation:.4f}m")
if rotation > max_rotation:
    reasons.append(f"rotation delta {math.degrees(rotation):.2f}deg > {math.degrees(max_rotation):.2f}deg")
if reasons:
    raise SystemExit("board verification failed: " + "; ".join(reasons))
PY
}

execute_grasp() {
  ensure_dirs
  if [[ "${side}" != "${EXECUTE_SIDE}" && "${ALLOW_NONDEFAULT_EXECUTE}" != "true" && "${ALLOW_NONDEFAULT_EXECUTE}" != "1" ]]; then
    echo "refusing ${side} execution: default real execution side is ${EXECUTE_SIDE}" >&2
    echo "This avoids two arms competing for an ambiguous object such as one of multiple pink cubes." >&2
    echo "Set ALLOW_NONDEFAULT_EXECUTE=true only after assigning distinct targets." >&2
    exit 1
  fi
  if [[ "${VERIFY_BOARD_BEFORE_EXECUTE}" == "true" || "${VERIFY_BOARD_BEFORE_EXECUTE}" == "1" ]]; then
    verify_board
  fi
  echo "About to command the real ${side} arm using prompt: ${prompt}"
  read -r -p "Type EXECUTE to confirm the workspace is clear: " answer
  if [[ "${answer}" != "EXECUTE" ]]; then
    echo "operator declined execution"
    exit 1
  fi
  source_ros
  wait_for_service "/${side}_grasp/plan_grasp"
  wait_for_runtime_tf 20
  apply_planner_runtime_params
  ros2 param set "/${side}_grasp/grasp_planner_node" wrist_refine_before_grasp "${WRIST_REFINE_BEFORE_GRASP}" >/dev/null
  echo "execution_backend=${EXECUTION_BACKEND}"
  echo "wrist_refine_before_grasp=${WRIST_REFINE_BEFORE_GRASP}"
  echo "wrist_roll prefer_low=${PREFER_LOW_WRIST_ROLL} preferred_delta=${PREFERRED_WRIST_ROLL_DELTA_RAD} max_delta=${MAX_WRIST_ROLL_DELTA_RAD}"
  echo "feedback tolerance=${FEEDBACK_POSITION_TOLERANCE_M}m corrections=${FEEDBACK_MAX_CORRECTION_ITERS} min_joint_delta=${FEEDBACK_MIN_JOINT_DELTA_RAD}rad"
  echo "wrist_confirmation before_descent=${WRIST_CONFIRMATION_BEFORE_DESCENT} require_match=${REQUIRE_WRIST_CONFIRMATION_FOR_EXECUTION} require_cloud=${REQUIRE_WRIST_CLOUD_FOR_EXECUTION}"
  ros2 param set "/${side}_grasp/grasp_planner_node" allow_execution true >/dev/null
  trap 'ros2 param set "/'"${side}"'_grasp/grasp_planner_node" allow_execution false >/dev/null 2>&1 || true' EXIT
  call_ros_service "${EXECUTE_CALL_TIMEOUT_S}" "${LOG_DIR}/${side}_execute_grasp.txt" \
    "/${side}_grasp/plan_grasp" so101_grasp_msgs/srv/PlanGrasp \
    "{prompt: '${prompt}', top_k: ${DEFAULT_TOP_K}, grasp_index: 0, execute: true, pregrasp_offset_m: 0.10, plan_time_s: 45.0}"
  ros2 param set "/${side}_grasp/grasp_planner_node" allow_execution false >/dev/null || true
  trap - EXIT
}

snapshot() {
  ensure_dirs
  source_ros
  wait_for_runtime_tf 20
  python3 "${XLEROBOT_WS}/scripts/capture_stack_layers.py" \
    --side "${side}" \
    --prompt "${prompt}" \
    --top-k "${DEFAULT_TOP_K}" \
    --out-dir "${XLEROBOT_RUN}/grasp/${side}/snapshot_$(date -u +%Y%m%dT%H%M%SZ)"
}

status() {
  echo "XLEROBOT_WS=${XLEROBOT_WS}"
  echo "XLEROBOT_RUN=${XLEROBOT_RUN}"
  echo "EXECUTION_BACKEND=${EXECUTION_BACKEND}"
  for item in left right; do
    local pid_file="${PID_DIR}/grasp_${item}.pid"
    if [[ -s "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" 2>/dev/null; then
      echo "${item}: running pid $(cat "${pid_file}")"
    else
      echo "${item}: stopped"
    fi
  done
}

cmd="${1:-}"
case "${cmd}" in
  up) start_stack ;;
  detect) detect ;;
  plan) plan ;;
  execute) execute_grasp ;;
  snapshot) snapshot ;;
  verify-board) verify_board ;;
  status) status ;;
  down) stop_stack ;;
  *) usage; exit 2 ;;
esac
