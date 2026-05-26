#!/usr/bin/env bash
set -euo pipefail

REMOTE_HOST="${REMOTE_HOST:-dell@192.168.1.73}"
LOCAL_PORT="${LOCAL_PORT:-8091}"
REMOTE_PORT="${REMOTE_PORT:-8091}"

exec ssh \
    -N \
    -T \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=4 \
    -R "${REMOTE_PORT}:127.0.0.1:${LOCAL_PORT}" \
    "${REMOTE_HOST}"
