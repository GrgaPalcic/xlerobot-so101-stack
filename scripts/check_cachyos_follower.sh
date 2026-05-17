#!/usr/bin/env bash

set -euo pipefail

WS_ROOT=${ROS_WS:-"$HOME/ros2_ws"}
UNDERLAY=/opt/ros/jazzy/setup.bash

status() {
  local label=$1
  local ok=$2
  local detail=$3
  if [[ "$ok" == "yes" ]]; then
    printf '[ok]   %-24s %s\n' "$label" "$detail"
  else
    printf '[warn] %-24s %s\n' "$label" "$detail"
  fi
}

if [[ -f "$UNDERLAY" ]]; then
  status "jazzy underlay" "yes" "$UNDERLAY"
else
  status "jazzy underlay" "no" "missing: $UNDERLAY"
fi

if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  status "overlay setup" "yes" "$WS_ROOT/install/setup.bash"
else
  status "overlay setup" "no" "missing: $WS_ROOT/install/setup.bash"
fi

if command -v ros2 >/dev/null 2>&1; then
  status "ros2 cli" "yes" "$(command -v ros2)"
else
  status "ros2 cli" "no" "ros2 not in PATH"
fi

if command -v colcon >/dev/null 2>&1; then
  status "colcon" "yes" "$(command -v colcon)"
else
  status "colcon" "no" "colcon not in PATH"
fi

if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  status "dialout group" "yes" "user is in dialout"
else
  status "dialout group" "no" "user must be added to dialout and re-login"
fi

if [[ -e /dev/so101_follower ]]; then
  status "follower device" "yes" "$(ls -l /dev/so101_follower)"
else
  status "follower device" "no" "missing /dev/so101_follower"
fi

if [[ -d "$WS_ROOT/src/ros2_control/.git" ]]; then
  status "ros2_control src" "yes" "$WS_ROOT/src/ros2_control"
else
  status "ros2_control src" "no" "run bootstrap script"
fi

if [[ -e "$WS_ROOT/src/so101-ros-physical-ai/feetech_ros2_driver/.git" ]]; then
  status "feetech submodule" "yes" "initialized"
else
  status "feetech submodule" "no" "run bootstrap script"
fi

cat <<'EOF'

Mock bringup:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch so101_bringup follower.launch.py hardware_type:=mock use_rviz:=false

Controller check:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 control list_controllers -c /follower/controller_manager

Real bringup:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch so101_bringup follower.launch.py hardware_type:=real usb_port:=/dev/so101_follower use_rviz:=false

EOF
