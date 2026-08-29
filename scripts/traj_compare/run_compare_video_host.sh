#!/usr/bin/env bash
# Launch animated RViz trajectory comparison from the host with correct Docker mounts.
#
# Usage:
#   ./run_compare_video_host.sh MH_01_easy --speed 15
#   ./run_compare_video_host.sh MH_01_easy --speed 15 --record
#   ./run_compare_video_host.sh MH_01_easy --speed 15 --record /home/aze-pc-0266/results/traj_compare/MH_01_easy.mp4

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DOCKER_CATKINWS="${DOCKER_CATKINWS:-/home/aze-pc-0266/workspace/catkin_ws_ov}"
export DOCKER_DATASETS="${DOCKER_DATASETS:-/home/aze-pc-0266/datasets}"
export DOCKER_RESULTS="${DOCKER_RESULTS:-/home/aze-pc-0266/results}"
IMG="${DOCKER_IMAGE:-ov_ros1_20_04:latest}"
RECORD_CRF="${RECORD_CRF:-17}"
RECORD_PRESET="${RECORD_PRESET:-medium}"
RECORD_FPS="${RECORD_FPS:-30}"

SEQ="${1:-MH_01_easy}"
shift || true

RECORD=""
INNER_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --record)
      if [[ $# -ge 2 && "$2" != --* ]]; then
        RECORD="$2"
        shift 2
      else
        RECORD="DEFAULT"
        shift
      fi
      ;;
    *) INNER_ARGS+=("$1"); shift ;;
  esac
done

if [[ "$RECORD" == "DEFAULT" ]]; then
  RECORD="$DOCKER_RESULTS/videos/${SEQ}_msckf_hybrid_gt.mp4"
fi

mkdir -p "$(dirname "$RECORD")"
RECORD="$(cd "$(dirname "$RECORD")" && pwd)/$(basename "$RECORD")"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set. Run from a graphical desktop terminal." >&2
  exit 1
fi
if ! command -v xwininfo >/dev/null 2>&1; then
  echo "ERROR: xwininfo not found. Install with: sudo apt install x11-utils" >&2
  exit 1
fi

xhost +local:docker >/dev/null 2>&1 || true

quote_args() {
  local out="" a
  for a in "$@"; do
    out+=" $(printf '%q' "$a")"
  done
  echo "$out"
}

INNER="set -euo pipefail
export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec /catkin_ws/src/open_vins/scripts/traj_compare/run_compare_video.sh $(printf '%q' "$SEQ")$(quote_args "${INNER_ARGS[@]+"${INNER_ARGS[@]}"}")"
if [[ -n "$RECORD" ]]; then
  INNER+=" --record $(printf '%q' "$RECORD")"
fi

DOCKER_BASE=(
  docker run --rm --net=host
  -e DISPLAY="${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -e ROS_DISTRO=noetic
  -e ROS_VERSION=1
  -e DISABLE_ROS1_EOL_WARNINGS=1
  -e RESULTS_DIR=/results
  -e DATASETS_DIR=/datasets
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  --mount type=bind,source="$DOCKER_CATKINWS",target=/catkin_ws
  --mount type=bind,source="$DOCKER_DATASETS",target=/datasets
  --mount type=bind,source="$DOCKER_RESULTS",target=/results
)

if [[ -z "$RECORD" ]]; then
  IT_FLAGS=()
  [[ -t 0 ]] && IT_FLAGS=(-it)
  exec "${DOCKER_BASE[@]}" "${IT_FLAGS[@]}" -e RECORD_FROM_HOST=0 "$IMG" bash -lc "$INNER"
fi

resolve_ffmpeg() {
  local c
  for c in \
    "$SCRIPT_DIR/bin/ffmpeg" \
    "$HOME/.local/bin/ffmpeg" \
    /tmp/ffmpeg-7.0.2-amd64-static/ffmpeg \
    "$(command -v ffmpeg 2>/dev/null || true)"
  do
    [[ -n "$c" && -x "$c" ]] && { echo "$c"; return 0; }
  done
  return 1
}

find_rviz_window() {
  python3 - <<'PY'
import os, re, subprocess, sys
disp = os.environ.get("DISPLAY", ":0")
try:
    tree = subprocess.check_output(["xwininfo", "-root", "-tree", "-display", disp], text=True, errors="replace")
except Exception:
    sys.exit(1)
pat = re.compile(r'^\s+(0x[0-9a-fA-F]+)\s+"([^"]*)".*\)\s+(\d+)x(\d+)\+(\d+)\+(\d+)')

def wm_class(wid_s):
    try:
        return subprocess.check_output(
            ["xprop", "-display", disp, "-id", wid_s, "WM_CLASS"],
            text=True, errors="replace",
        ).lower()
    except Exception:
        return ""

best = None
for line in tree.splitlines():
    m = pat.search(line)
    if not m:
        continue
    wid_s, title = m.group(1), m.group(2)
    w, h, x, y = map(int, m.group(3, 4, 5, 6))
    title_l = title.lower()
    if "legend" in title_l:
        continue
    if w < 400 or h < 300:
        continue
    klass = wm_class(wid_s)
    is_rviz = ('"rviz"' in klass) or title_l in ("rviz", "rviz_compare") or title_l.startswith("rviz ")
    if not is_rviz:
        continue
    area = w * h
    if best is None or area > best[0]:
        best = (area, int(wid_s, 16), w, h, x, y)
if not best:
    sys.exit(1)
_, wid, w, h, x, y = best
print("0x%x %d %d %d %d" % (wid, w, h, x, y))
PY
}

focus_rviz_window() {
  local wid="$1"
  if command -v xdotool >/dev/null 2>&1; then
    xdotool windowactivate "$wid" 2>/dev/null || true
    xdotool windowraise "$wid" 2>/dev/null || true
  elif command -v wmctrl >/dev/null 2>&1; then
    wmctrl -ia "$wid" 2>/dev/null || true
  fi
}

FFMPEG="$(resolve_ffmpeg)" || {
  echo "ERROR: ffmpeg not found." >&2
  echo "Place a static binary at: $SCRIPT_DIR/bin/ffmpeg" >&2
  echo "Or install: sudo apt install ffmpeg" >&2
  exit 1
}

echo "============================================================"
echo "RViz compare + record"
echo "  Sequence : $SEQ"
echo "  Output   : $RECORD"
echo "  Display  : $DISPLAY"
echo "  Duration : ~45-60 s (keep RViz window visible)"
echo "============================================================"
echo "Starting Docker + RViz..."

"${DOCKER_BASE[@]}" -e RECORD_FROM_HOST=1 "$IMG" bash -lc "$INNER" &
DOCKER_PID=$!

GEOM=""
for i in $(seq 1 80); do
  if GEOM="$(find_rviz_window 2>/dev/null)"; then
    break
  fi
  if (( i % 8 == 0 )); then
    echo "  waiting for RViz window... (${i}/80)"
  fi
  if ! kill -0 "$DOCKER_PID" 2>/dev/null; then
    wait "$DOCKER_PID" || true
    echo "RViz exited before a window appeared; no video written." >&2
    exit 1
  fi
  sleep 0.25
done

if [[ -z "$GEOM" ]]; then
  kill "$DOCKER_PID" 2>/dev/null || true
  wait "$DOCKER_PID" 2>/dev/null || true
  echo "Could not find RViz window to record." >&2
  exit 1
fi

read -r RVIZ_WID RVIZ_W RVIZ_H RVIZ_X RVIZ_Y <<<"$GEOM"
echo "Recording RViz window ${RVIZ_WID} (${RVIZ_W}x${RVIZ_H}+${RVIZ_X}+${RVIZ_Y}) ..."
focus_rviz_window "$RVIZ_WID"
echo "  recording to $RECORD (do not cover/minimize RViz)"
# Let RViz finish OpenGL init and first path frame before capture.
sleep 4

FFMPEG_LOG="${RECORD%.mp4}_ffmpeg.log"
"$FFMPEG" -y \
  -f x11grab -draw_mouse 0 -window_id "$RVIZ_WID" \
  -framerate "$RECORD_FPS" -i "${DISPLAY}" \
  -vf "crop=floor(iw/2)*2:floor(ih/2)*2" \
  -c:v libx264 -pix_fmt yuv420p -preset "$RECORD_PRESET" -crf "$RECORD_CRF" \
  -movflags +faststart \
  "$RECORD" >"$FFMPEG_LOG" 2>&1 &
FFMPEG_PID=$!
sleep 1
if ! kill -0 "$FFMPEG_PID" 2>/dev/null; then
  echo "ffmpeg failed to start. Log:" >&2
  cat "$FFMPEG_LOG" >&2
  kill "$DOCKER_PID" 2>/dev/null || true
  exit 1
fi

set +e
wait "$DOCKER_PID"
RC=$?
set -e

sleep 0.5
kill -INT "$FFMPEG_PID" 2>/dev/null || true
wait "$FFMPEG_PID" 2>/dev/null || true

if [[ -s "$RECORD" ]]; then
  SZ="$(du -h "$RECORD" | cut -f1)"
  echo ""
  echo "SUCCESS: wrote $RECORD ($SZ)"
  echo "Play with: vlc \"$RECORD\""
else
  echo "ERROR: recording failed (no file or empty)." >&2
  echo "Log: $FFMPEG_LOG" >&2
  cat "$FFMPEG_LOG" >&2 || true
  exit 1
fi

exit "$RC"
