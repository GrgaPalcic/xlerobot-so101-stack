#!/usr/bin/env bash

set -euo pipefail

aur_flags=(--needed)
pacman_flags=(--needed)

if [[ "${ASSUME_YES:-0}" == "1" ]]; then
  aur_flags+=(--noconfirm)
  pacman_flags+=(--noconfirm)
fi

if ! command -v paru >/dev/null 2>&1; then
  echo "paru is required but was not found in PATH." >&2
  exit 1
fi

echo "Installing AUR/ROS packages..."
paru -S "${aur_flags[@]}" \
  ros2-jazzy \
  ros2-jazzy-control_msgs \
  python-rosdep \
  python-colcon-common-extensions

echo "Installing repo packages..."
sudo pacman -S "${pacman_flags[@]}" \
  python-filelock \
  python-pygraphviz \
  range-v3 \
  tl-expected \
  libserial-git

cat <<'EOF'

Package installation complete.

Next steps:
  1. ./scripts/bootstrap_cachyos_follower_overlay.sh
  2. ./scripts/build_cachyos_follower_overlay.sh
  3. ./scripts/check_cachyos_follower.sh

EOF
