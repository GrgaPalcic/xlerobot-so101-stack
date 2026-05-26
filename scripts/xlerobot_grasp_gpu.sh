#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/grga/Documents/so101-ros-physical-ai}"
DELL_HOST="${DELL_HOST:-dell@192.168.1.73}"
DELL_WS="${DELL_WS:-/home/dell/Documents/xlerobot-so101-stack}"
RUN_ID="${RUN_ID:-printed_plate_20260520}"
DELL_RUN="${DELL_RUN:-${DELL_WS}/field_runs/xlerobot_${RUN_ID}}"
GPU_RUN="${GPU_RUN:-${REPO_ROOT}/field_runs/xlerobot_${RUN_ID}}"
LOG_DIR="${LOG_DIR:-${GPU_RUN}/logs}"
PID_DIR="${PID_DIR:-${GPU_RUN}/pids}"

LOCAL_PORT="${LOCAL_PORT:-8091}"
REMOTE_PORT="${REMOTE_PORT:-8091}"

export REPO_ROOT
export VENV_PATH="${VENV_PATH:-/home/grga/Documents/Depth-Anything-3/.venv}"
export METRIC_MODEL="${METRIC_MODEL:-/home/grga/Documents/Depth-Anything-3/models/DA3-LARGE-1.1}"
export GRASP_BACKEND="${GRASP_BACKEND:-ggcnn}"
export GGCNN_PRIMARY_VIEW="${GGCNN_PRIMARY_VIEW:-wrist}"
export DA3_CONDITIONING="${DA3_CONDITIONING:-required}"
export DA3_FALLBACK_INDEPENDENT="${DA3_FALLBACK_INDEPENDENT:-false}"
export PROCESS_RES="${PROCESS_RES:-336}"
export CUDA_EMPTY_CACHE="${CUDA_EMPTY_CACHE:-true}"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True,max_split_size_mb:128}"
export WORKSPACE_BOUNDS="${WORKSPACE_BOUNDS:--0.50 0.50 -0.35 0.35 -0.05 0.50}"
export SUPPORT_PLANE_YAML="${SUPPORT_PLANE_YAML:-${GPU_RUN}/extrinsics/world_support_plane.yaml}"

usage() {
  cat <<EOF
usage: $0 up|server|tunnel|sync|status|down|restart

Environment:
  DELL_HOST=${DELL_HOST}
  DELL_RUN=${DELL_RUN}
  GPU_RUN=${GPU_RUN}
  GRASP_BACKEND=${GRASP_BACKEND}
  PROCESS_RES=${PROCESS_RES}
EOF
}

ensure_dirs() {
  mkdir -p "${GPU_RUN}/extrinsics" "${LOG_DIR}" "${PID_DIR}"
}

sync_calib() {
  ensure_dirs
  scp -q "${DELL_HOST}:${DELL_RUN}/extrinsics/world_support_plane.yaml" \
    "${GPU_RUN}/extrinsics/world_support_plane.yaml"
  scp -q "${DELL_HOST}:${DELL_RUN}/run_state.yaml" "${GPU_RUN}/run_state.yaml" || true
  echo "synced calibration into ${GPU_RUN}"
}

start_server() {
  ensure_dirs
  if [[ -s "${PID_DIR}/grasp_server.pid" ]] && kill -0 "$(cat "${PID_DIR}/grasp_server.pid")" 2>/dev/null; then
    echo "grasp server already running: pid $(cat "${PID_DIR}/grasp_server.pid")"
    return
  fi
  if pgrep -af "grasp-server .*--port ${LOCAL_PORT}" >/dev/null 2>&1; then
    echo "stopping stale grasp server on port ${LOCAL_PORT}:"
    pgrep -af "grasp-server .*--port ${LOCAL_PORT}" || true
    stop_stale_server
    sleep 2
  fi
  if [[ ! -f "${SUPPORT_PLANE_YAML}" ]]; then
    sync_calib
  fi
  (
    cd "${REPO_ROOT}"
    HOST=127.0.0.1 PORT="${LOCAL_PORT}" scripts/run_grasp_server.sh
  ) >"${LOG_DIR}/grasp_server.log" 2>&1 &
  echo $! > "${PID_DIR}/grasp_server.pid"
  echo "started grasp server: pid $(cat "${PID_DIR}/grasp_server.pid"), log ${LOG_DIR}/grasp_server.log"
}

start_tunnel() {
  ensure_dirs
  if [[ -s "${PID_DIR}/grasp_tunnel.pid" ]] && kill -0 "$(cat "${PID_DIR}/grasp_tunnel.pid")" 2>/dev/null; then
    echo "grasp tunnel already running: pid $(cat "${PID_DIR}/grasp_tunnel.pid")"
    return
  fi
  if pgrep -af "ssh .* -R ${REMOTE_PORT}:127.0.0.1:${LOCAL_PORT}" >/dev/null 2>&1; then
    echo "stopping stale reverse tunnel for remote port ${REMOTE_PORT}:"
    pgrep -af "ssh .* -R ${REMOTE_PORT}:127.0.0.1:${LOCAL_PORT}" || true
    stop_stale_tunnel
    sleep 1
  fi
  ssh -N -T \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=4 \
    -R "${REMOTE_PORT}:127.0.0.1:${LOCAL_PORT}" \
    "${DELL_HOST}" >"${LOG_DIR}/grasp_tunnel.log" 2>&1 &
  echo $! > "${PID_DIR}/grasp_tunnel.pid"
  sleep 1
  if ! kill -0 "$(cat "${PID_DIR}/grasp_tunnel.pid")" 2>/dev/null; then
    echo "reverse tunnel exited during startup; tail follows:" >&2
    tail -80 "${LOG_DIR}/grasp_tunnel.log" >&2 || true
    rm -f "${PID_DIR}/grasp_tunnel.pid"
    exit 1
  fi
  echo "started reverse tunnel: pid $(cat "${PID_DIR}/grasp_tunnel.pid"), log ${LOG_DIR}/grasp_tunnel.log"
}

stop_pid() {
  local name="$1"
  local pid_file="${PID_DIR}/${name}.pid"
  if [[ -s "${pid_file}" ]]; then
    local pid
    pid="$(cat "${pid_file}")"
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      sleep 1
      kill -9 "${pid}" 2>/dev/null || true
    fi
    rm -f "${pid_file}"
  fi
}

stop_stale_server() {
  systemctl --user stop so101-grasp-server.service >/dev/null 2>&1 || true
  pkill -f "grasp-server .*--port ${LOCAL_PORT}" 2>/dev/null || true
}

stop_stale_tunnel() {
  systemctl --user stop so101-grasp-tunnel.service >/dev/null 2>&1 || true
  pkill -f "ssh .* -R ${REMOTE_PORT}:127.0.0.1:${LOCAL_PORT}" 2>/dev/null || true
}

status() {
  ensure_dirs
  if systemctl --user is-active --quiet so101-grasp-server.service 2>/dev/null; then
    echo "legacy systemd server: active"
  fi
  if systemctl --user is-active --quiet so101-grasp-tunnel.service 2>/dev/null; then
    echo "legacy systemd tunnel: active"
  fi
  for name in grasp_server grasp_tunnel; do
    if [[ -s "${PID_DIR}/${name}.pid" ]] && kill -0 "$(cat "${PID_DIR}/${name}.pid")" 2>/dev/null; then
      echo "${name}: running pid $(cat "${PID_DIR}/${name}.pid")"
    else
      echo "${name}: stopped"
    fi
  done
  if pgrep -af "grasp-server .*--port ${LOCAL_PORT}" >/dev/null 2>&1; then
    echo "port ${LOCAL_PORT} owner(s):"
    pgrep -af "grasp-server .*--port ${LOCAL_PORT}" || true
  fi
  if pgrep -af "ssh .* -R ${REMOTE_PORT}:127.0.0.1:${LOCAL_PORT}" >/dev/null 2>&1; then
    echo "reverse tunnel owner(s):"
    pgrep -af "ssh .* -R ${REMOTE_PORT}:127.0.0.1:${LOCAL_PORT}" || true
  fi
  echo "support plane: ${SUPPORT_PLANE_YAML}"
  command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi --query-gpu=memory.used,memory.free --format=csv,noheader,nounits || true
}

cmd="${1:-}"
case "${cmd}" in
  up)
    sync_calib
    start_server
    start_tunnel
    ;;
  server)
    start_server
    ;;
  tunnel)
    start_tunnel
    ;;
  sync)
    sync_calib
    ;;
  status)
    status
    ;;
  down)
    stop_pid grasp_tunnel
    stop_stale_tunnel
    stop_pid grasp_server
    stop_stale_server
    status
    ;;
  restart)
    stop_pid grasp_tunnel
    stop_stale_tunnel
    stop_pid grasp_server
    stop_stale_server
    sync_calib
    start_server
    start_tunnel
    ;;
  *)
    usage
    exit 2
    ;;
esac
