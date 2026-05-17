#!/usr/bin/env bash
set -euo pipefail

VIDEO_DEV="${VIDEO_DEV:-/dev/video42}"
UDP_PORT="${UDP_PORT:-8554}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
RES="${GOPRO_RES:-1080}"
FOV_ID="${GOPRO_FOV_ID:-0}"
GOPRO_IP_HINT="${GOPRO_IP:-172.20.144.51}"
GOPRO_HOST_IP="${GOPRO_HOST_IP:-172.20.144.52/24}"
GOPRO_IFACE_PATTERN="${GOPRO_IFACE_PATTERN:-enx}"
LOG="${LOG:-/tmp/gopro_ffmpeg.log}"
PIDFILE="${PIDFILE:-/tmp/gopro_ffmpeg.pid}"
FRAME_PROBE="${FRAME_PROBE:-/tmp/gopro_heal_probe.jpg}"

status() {
  printf '%s\n' "$*"
}

find_gopro_ip() {
  local ip prefix
  while read -r ip; do
    prefix="${ip%.*}"
    if curl -m 1 -fsS "http://${prefix}.51/gp/gpWebcam/STATUS" >/dev/null 2>&1; then
      printf '%s.51\n' "$prefix"
      return 0
    fi
  done < <(ip -o -4 addr show | awk '{print $4}' | cut -d/ -f1 | grep -E '^172\.[0-9]+\.[0-9]+\.')

  if curl -m 1 -fsS "http://${GOPRO_IP_HINT}/gp/gpWebcam/STATUS" >/dev/null 2>&1; then
    printf '%s\n' "$GOPRO_IP_HINT"
    return 0
  fi
  return 1
}

configure_gopro_usb_network() {
  local iface
  iface="$(ip -o link show | awk -F': ' -v pattern="$GOPRO_IFACE_PATTERN" '$2 ~ "^" pattern {print $2; exit}')"
  if [[ -z "$iface" ]]; then
    return 1
  fi

  status "Configuring GoPro USB network on ${iface}: ${GOPRO_HOST_IP}"
  sudo nmcli device set "$iface" managed no >/dev/null 2>&1 || true
  sudo ip link set "$iface" up
  sudo ip addr flush dev "$iface"
  sudo ip addr add "$GOPRO_HOST_IP" dev "$iface"
}

probe_frame() {
  rm -f "$FRAME_PROBE"
  timeout 20 ffmpeg \
    -hide_banner \
    -loglevel warning \
    -f v4l2 \
    -i "$VIDEO_DEV" \
    -frames:v 1 \
    -update 1 \
    -y "$FRAME_PROBE" || true

  if [[ -s "$FRAME_PROBE" ]]; then
    status "probe: saved $FRAME_PROBE"
    return 0
  fi

  status "probe: no frame from $VIDEO_DEV"
  return 3
}

ensure_video_dev() {
  if [[ -e "$VIDEO_DEV" ]]; then
    return 0
  fi

  status "$VIDEO_DEV is missing; loading v4l2loopback requires sudo."
  sudo modprobe v4l2loopback video_nr="${VIDEO_DEV#/dev/video}" card_label=GoPro exclusive_caps=1
}

ffmpeg_pids() {
  ps -eo pid,args | awk -v port="$UDP_PORT" '/[f]fmpeg/ && index($0, "udp://@0.0.0.0:" port) {print $1}'
}

stop_bridge() {
  local pids remaining
  pids="$(ffmpeg_pids | tr '\n' ' ')"
  if [[ -z "${pids// }" ]]; then
    return 0
  fi

  status "Stopping stale GoPro ffmpeg bridge: $pids"
  kill $pids 2>/dev/null || true
  sleep 0.8
  remaining="$(ffmpeg_pids | tr '\n' ' ')"
  if [[ -n "${remaining// }" ]]; then
    status "Some bridge processes require sudo: $remaining"
    sudo kill $remaining 2>/dev/null || true
    sleep 0.8
  fi
}

start_bridge() {
  status "Starting ffmpeg UDP:${UDP_PORT} -> ${VIDEO_DEV} bridge..."
  rm -f "$LOG"
  nohup ffmpeg \
    -nostdin \
    -threads 1 \
    -i "udp://@0.0.0.0:${UDP_PORT}?overrun_nonfatal=1&fifo_size=50000000" \
    -fflags nobuffer \
    -vf "scale=${WIDTH}:${HEIGHT},format=yuyv422" \
    -f v4l2 "$VIDEO_DEV" \
    >"$LOG" 2>&1 &
  printf '%s\n' "$!" > "$PIDFILE"
}

main() {
  status "SO101 GoPro webcam heal"
  status "Target virtual camera: $VIDEO_DEV (${WIDTH}x${HEIGHT}@${FPS})"
  status ""

  ensure_video_dev

  if GOPRO_IP="$(find_gopro_ip)"; then
    status "GoPro API found at ${GOPRO_IP}"
  elif configure_gopro_usb_network && GOPRO_IP="$(find_gopro_ip)"; then
    status "GoPro API found at ${GOPRO_IP} after USB network setup"
  else
    status "ERROR: GoPro USB API is not reachable."
    status "Check USB cable, power, and GoPro webcam mode."
    status "The GoPro must expose an ${GOPRO_IFACE_PATTERN}... USB network interface before this script can heal /dev/video42."
    return 2
  fi

  status "Requesting GoPro webcam stream: res=${RES}"
  curl -m 5 -fsS "http://${GOPRO_IP}/gp/gpWebcam/START?res=${RES}&port=${UDP_PORT}" | tee /tmp/gopro_webcam_start.json
  status ""
  status "Requesting GoPro FOV id=${FOV_ID}"
  curl -m 5 -fsS "http://${GOPRO_IP}/gp/gpWebcam/SETTINGS?fov=${FOV_ID}" | tee /tmp/gopro_webcam_fov.json
  status ""

  if probe_frame; then
    status ""
    status "GoPro webcam already healthy."
    return 0
  fi

  status ""
  status "No valid frame from existing bridge; restarting bridge."
  stop_bridge
  start_bridge
  sleep 5

  if probe_frame; then
    status ""
    status "GoPro webcam healed successfully."
    status "Verified frame: $FRAME_PROBE"
    return 0
  fi

  status ""
  status "ERROR: GoPro API started, but no frame arrived on $VIDEO_DEV."
  status "ffmpeg log follows:"
  tail -n 80 "$LOG" 2>/dev/null || true
  return 3
}

main "$@"
