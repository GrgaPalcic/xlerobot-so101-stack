#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 || "$1" != "left" && "$1" != "right" ]]; then
  echo "usage: $0 left|right [touch-jog options]" >&2
  exit 2
fi

side="$1"
shift

ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws"

source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

exec ros2 run xlerobot_calibration xlerobot-calib \
  --workspace "$ws" \
  touch-jog "$side" "$@"
