#!/usr/bin/env bash

set -euo pipefail

WS_ROOT=${ROS_WS:-"$HOME/ros2_ws"}
UNDERLAY=/opt/ros/jazzy/setup.bash

if [[ ! -f "$UNDERLAY" ]]; then
  echo "ROS 2 Jazzy underlay not found at $UNDERLAY" >&2
  echo "Run ./scripts/install_cachyos_follower_deps.sh first." >&2
  exit 1
fi

if [[ ! -d "$WS_ROOT/src/so101-ros-physical-ai" ]]; then
  echo "Overlay workspace is missing $WS_ROOT/src/so101-ros-physical-ai" >&2
  echo "Run ./scripts/bootstrap_cachyos_follower_overlay.sh first." >&2
  exit 1
fi

source "$UNDERLAY"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not available after sourcing $UNDERLAY" >&2
  exit 1
fi

cd "$WS_ROOT"

colcon build \
  --symlink-install \
  --packages-up-to \
  so101_bringup \
  feetech_ros2_driver \
  controller_manager \
  ros2controlcli \
  joint_state_broadcaster \
  forward_command_controller \
  joint_trajectory_controller \
  diagnostic_updater \
  realtime_tools \
  backward_ros \
  rsl \
  tcb_span \
  generate_parameter_library

cat <<EOF

Build complete.

Source the workspace with:
  source /opt/ros/jazzy/setup.bash
  source $WS_ROOT/install/setup.bash

Then run:
  ros2 launch so101_bringup follower.launch.py hardware_type:=mock use_rviz:=false

EOF
