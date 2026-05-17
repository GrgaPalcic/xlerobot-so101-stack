#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_PATH="${WORKSPACE_PATH:-$HOME/Documents/so101-ros-physical-ai}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_PATH}/install/setup.bash"
set -u

cd "${WORKSPACE_PATH}"
exec ros2 launch so101_bringup grasping.launch.py "$@"
