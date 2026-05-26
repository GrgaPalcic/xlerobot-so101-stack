#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<EOF
usage: $0 left|right "object prompt"

Runs the grasp stack with supervised wrist-view retries and a short post-grasp lift.
EOF
}

side="${1:-}"
if [[ "${side}" != "left" && "${side}" != "right" ]]; then
  usage
  exit 2
fi
shift
prompt="${*:-${DEFAULT_PROMPT:-pink cube}}"

XLEROBOT_WS="${XLEROBOT_WS:-/home/dell/Documents/xlerobot-so101-stack}"
if [[ -z "${XLEROBOT_RUN:-}" ]]; then
  XLEROBOT_RUN="$(find "${XLEROBOT_WS}/field_runs" -maxdepth 1 -type d -name 'xlerobot_*' | sort | tail -n 1)"
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
DEFAULT_TOP_K="${DEFAULT_TOP_K:-8}"
SUPERVISOR_CAPTURE_SNAPSHOTS="${SUPERVISOR_CAPTURE_SNAPSHOTS:-true}"
SUPERVISOR_RESTART_STACK="${SUPERVISOR_RESTART_STACK:-true}"

export WRIST_REFINE_BEFORE_GRASP="${WRIST_REFINE_BEFORE_GRASP:-true}"
export WRIST_REFINE_REQUIRE_WRIST_CLOUD="${WRIST_REFINE_REQUIRE_WRIST_CLOUD:-true}"
export WRIST_REFINE_MIN_WRIST_CLOUD_POINTS="${WRIST_REFINE_MIN_WRIST_CLOUD_POINTS:-64}"
export WRIST_REFINE_VIEW_STANDOFFS_M="${WRIST_REFINE_VIEW_STANDOFFS_M:-[0.18, 0.22, 0.26]}"
export WRIST_REFINE_VIEW_LATERAL_OFFSETS_M="${WRIST_REFINE_VIEW_LATERAL_OFFSETS_M:-[0.0, 0.035, 0.070]}"
export WRIST_REFINE_MAX_VIEW_ATTEMPTS="${WRIST_REFINE_MAX_VIEW_ATTEMPTS:-3}"
export POST_GRASP_LIFT_M="${POST_GRASP_LIFT_M:-0.04}"

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
session_dir="${XLEROBOT_RUN}/grasp/${side}/supervised_pick_${stamp}"
mkdir -p "${session_dir}"

source_ros() {
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1091
  source "${XLEROBOT_WS}/install/setup.bash"
  set -u
}

capture_snapshot() {
  local name="$1"
  if [[ "${SUPERVISOR_CAPTURE_SNAPSHOTS}" != "true" && "${SUPERVISOR_CAPTURE_SNAPSHOTS}" != "1" ]]; then
    return
  fi
  source_ros
  python3 "${XLEROBOT_WS}/scripts/capture_stack_layers.py" \
    --side "${side}" \
    --prompt "${prompt}" \
    --top-k "${DEFAULT_TOP_K}" \
    --out-dir "${session_dir}/${name}" \
    | tee "${session_dir}/${name}.log"
}

cat >"${session_dir}/supervisor.env" <<EOF
side=${side}
prompt=${prompt}
WRIST_REFINE_VIEW_STANDOFFS_M=${WRIST_REFINE_VIEW_STANDOFFS_M}
WRIST_REFINE_VIEW_LATERAL_OFFSETS_M=${WRIST_REFINE_VIEW_LATERAL_OFFSETS_M}
WRIST_REFINE_MAX_VIEW_ATTEMPTS=${WRIST_REFINE_MAX_VIEW_ATTEMPTS}
WRIST_REFINE_REQUIRE_WRIST_CLOUD=${WRIST_REFINE_REQUIRE_WRIST_CLOUD}
WRIST_REFINE_MIN_WRIST_CLOUD_POINTS=${WRIST_REFINE_MIN_WRIST_CLOUD_POINTS}
POST_GRASP_LIFT_M=${POST_GRASP_LIFT_M}
SUPERVISOR_RESTART_STACK=${SUPERVISOR_RESTART_STACK}
EOF

echo "supervised pick session: ${session_dir}"
if [[ "${SUPERVISOR_RESTART_STACK}" == "true" || "${SUPERVISOR_RESTART_STACK}" == "1" ]]; then
  "${XLEROBOT_WS}/scripts/xlerobot_grasp_stack.sh" down "${side}" | tee "${session_dir}/stack_down.log" || true
fi
"${XLEROBOT_WS}/scripts/xlerobot_grasp_stack.sh" up "${side}" | tee "${session_dir}/stack_up.log"
capture_snapshot "before"

set +e
"${XLEROBOT_WS}/scripts/xlerobot_grasp_stack.sh" execute "${side}" "${prompt}" | tee "${session_dir}/execute.log"
rc=${PIPESTATUS[0]}
set -e

cp -f "${XLEROBOT_RUN}/logs/${side}_execute_grasp.txt" "${session_dir}/${side}_execute_grasp.txt" 2>/dev/null || true
capture_snapshot "after" || true

echo "supervised pick exit_code=${rc}" | tee "${session_dir}/result.txt"
exit "${rc}"
