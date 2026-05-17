#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
WS_ROOT=${ROS_WS:-"$HOME/ros2_ws"}
SRC_DIR="$WS_ROOT/src"
REPO_LINK="$SRC_DIR/so101-ros-physical-ai"

mkdir -p "$SRC_DIR"

if [[ -L "$REPO_LINK" ]]; then
  current_target=$(readlink -f "$REPO_LINK")
  if [[ "$current_target" != "$REPO_ROOT" ]]; then
    echo "Refusing to replace existing symlink: $REPO_LINK -> $current_target" >&2
    exit 1
  fi
elif [[ -e "$REPO_LINK" ]]; then
  echo "Refusing to overwrite existing path: $REPO_LINK" >&2
  exit 1
else
  ln -s "$REPO_ROOT" "$REPO_LINK"
fi

git -C "$REPO_ROOT" submodule update --init --recursive

clone_or_update() {
  local name=$1
  local url=$2
  local branch=${3:-}

  if [[ -d "$SRC_DIR/$name/.git" ]]; then
    git -C "$SRC_DIR/$name" fetch --all --tags --prune
    if [[ -n "$branch" ]]; then
      git -C "$SRC_DIR/$name" checkout "$branch"
      git -C "$SRC_DIR/$name" pull --ff-only origin "$branch"
    fi
    return
  fi

  if [[ -n "$branch" ]]; then
    git clone --branch "$branch" --single-branch "$url" "$SRC_DIR/$name"
  else
    git clone "$url" "$SRC_DIR/$name"
  fi
}

clone_or_update ros2_control https://github.com/ros-controls/ros2_control.git jazzy
clone_or_update ros2_controllers https://github.com/ros-controls/ros2_controllers.git jazzy
clone_or_update realtime_tools https://github.com/ros-controls/realtime_tools.git jazzy
clone_or_update diagnostics https://github.com/ros/diagnostics.git ros2-jazzy
clone_or_update generate_parameter_library https://github.com/PickNikRobotics/generate_parameter_library.git main
clone_or_update RSL https://github.com/PickNikRobotics/RSL.git main
clone_or_update cpp_polyfills https://github.com/PickNikRobotics/cpp_polyfills.git main
clone_or_update backward_ros https://github.com/pal-robotics/backward_ros.git

cat <<EOF

Overlay bootstrap complete.

Workspace root: $WS_ROOT
Source dir:      $SRC_DIR
Repo symlink:    $REPO_LINK

Next step:
  ./scripts/build_cachyos_follower_overlay.sh

EOF
