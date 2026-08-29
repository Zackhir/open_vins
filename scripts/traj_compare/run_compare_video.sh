#!/usr/bin/env bash
# Animated RViz comparison: MSCKF vs hybrid PO-MSCKF vs GT.
#
# Inside Docker (with /results and /datasets mounted):
#   source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
#   ./src/open_vins/scripts/traj_compare/run_compare_video.sh MH_01_easy
#
# From host (auto-mounts results/datasets + X11):
#   ./scripts/traj_compare/run_compare_video_host.sh MH_01_easy
#
# Override paths when mounts differ:
#   RESULTS_DIR=/home/aze-pc-0266/results DATASETS_DIR=/home/aze-pc-0266/datasets \
#     ./run_compare_video.sh MH_01_easy

set -euo pipefail

export DISABLE_ROS1_EOL_WARNINGS="${DISABLE_ROS1_EOL_WARNINGS:-1}"

SEQ="${1:-MH_01_easy}"
shift || true

RECORD_CRF="${RECORD_CRF:-17}"
RECORD_PRESET="${RECORD_PRESET:-medium}"
RECORD_FPS="${RECORD_FPS:-30}"
SPEED="5.0"
RECORD=""
LOOP="false"
MSCKF_OVERRIDE=""
HYBRID_OVERRIDE=""
GT_OVERRIDE=""
SHUTDOWN_WHEN_DONE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --speed) SPEED="$2"; shift 2 ;;
    --record)
      if [[ $# -ge 2 && "$2" != --* ]]; then
        RECORD="$2"
        shift 2
      else
        RECORD="DEFAULT"
        shift
      fi
      ;;
    --loop) LOOP="true"; shift ;;
    --msckf) MSCKF_OVERRIDE="$2"; shift 2 ;;
    --hybrid) HYBRID_OVERRIDE="$2"; shift 2 ;;
    --gt) GT_OVERRIDE="$2"; shift 2 ;;
    -h|--help)
      cat <<EOF
Usage: $0 [SEQ] [options]

Options:
  --speed N       Replay speed multiplier (default: 5)
  --record [FILE] Record RViz window to MP4 (needs DISPLAY).
                  Default: \$RESULTS_DIR/videos/<SEQ>_msckf_hybrid_gt.mp4
  --loop          Loop replay
  --msckf PATH    Override MSCKF estimate.txt
  --hybrid PATH   Override hybrid estimate.txt
  --gt PATH       Override ground-truth file

Environment:
  RESULTS_DIR     Root folder containing step7_mono_msckf/ and step7_mono_po_hybrid_ik/
  DATASETS_DIR    Root folder containing machine_hall/ and uzhfpv_*/

If files are missing, mount results into the container, e.g.:
  docker run ... -v /home/aze-pc-0266/results:/results -v /home/aze-pc-0266/datasets:/datasets ...
EOF
      exit 0
      ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

resolve_root() {
  local rel="$1"
  shift
  local candidate
  for candidate in "$@"; do
    [[ -z "$candidate" ]] && continue
    if [[ -f "$candidate/$rel" ]]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

resolve_gt_path() {
  local datasets_root="$1"
  case "$SEQ" in
    MH_01_easy) echo "$datasets_root/machine_hall/MH_01_easy/MH_01_easy/mav0/state_groundtruth_estimate0/data.txt" ;;
    MH_02_easy) echo "$datasets_root/machine_hall/MH_02_easy/MH_02_easy/mav0/state_groundtruth_estimate0/data.txt" ;;
    MH_03_medium) echo "$datasets_root/machine_hall/MH_03_medium/MH_03_medium/mav0/state_groundtruth_estimate0/data.txt" ;;
    MH_04_difficult) echo "$datasets_root/machine_hall/MH_04_difficult/MH_04_difficult/mav0/state_groundtruth_estimate0/data.txt" ;;
    MH_05_difficult) echo "$datasets_root/machine_hall/MH_05_difficult/MH_05_difficult/mav0/state_groundtruth_estimate0/data.txt" ;;
    outdoor_forward_1) echo "$datasets_root/uzhfpv_outdoor_forward/forward_1/outdoor_forward_1_snapdragon_with_gt/groundtruth.txt" ;;
    outdoor_forward_3) echo "$datasets_root/uzhfpv_outdoor_forward/forward_3/outdoor_forward_3_snapdragon_with_gt/groundtruth.txt" ;;
    outdoor_forward_5) echo "$datasets_root/uzhfpv_outdoor_forward/forward_5/outdoor_forward_5_snapdragon_with_gt/groundtruth.txt" ;;
    outdoor_45_1) echo "$datasets_root/uzhfpv_outdoor_45/outdoor_45_1_snapdragon_with_gt/groundtruth.txt" ;;
    *)
      echo "Unknown sequence: $SEQ" >&2
      echo "Use --msckf/--hybrid/--gt or roslaunch ov_eval compare_trajectories.launch ..." >&2
      exit 1
      ;;
  esac
}

MSCKF_REL="step7_mono_msckf/$SEQ/estimate.txt"
HYBRID_REL="step7_mono_po_hybrid_ik/$SEQ/estimate.txt"

if [[ -n "$MSCKF_OVERRIDE" ]]; then
  MSCKF="$MSCKF_OVERRIDE"
  RESULTS_ROOT="$(cd "$(dirname "$MSCKF")/../.." && pwd)"
else
  RESULTS_ROOT="${RESULTS_DIR:-}"
  if [[ -z "$RESULTS_ROOT" ]]; then
    RESULTS_ROOT="$(resolve_root "$MSCKF_REL" \
      /results \
      /home/aze-pc-0266/results \
      "$HOME/results" \
      /root/results)" || true
  fi
  if [[ -z "${RESULTS_ROOT:-}" ]]; then
    echo "Missing MSCKF estimate: could not find $MSCKF_REL" >&2
    echo "Tried RESULTS_DIR, /results, /home/aze-pc-0266/results, ~/results" >&2
    echo "Fix: mount results into the container, e.g.:" >&2
    echo "  -v /home/aze-pc-0266/results:/results \\" >&2
    echo "or run from host:" >&2
    echo "  ./scripts/traj_compare/run_compare_video_host.sh $SEQ" >&2
    exit 1
  fi
  MSCKF="$RESULTS_ROOT/$MSCKF_REL"
fi

if [[ -n "$HYBRID_OVERRIDE" ]]; then
  HYBRID="$HYBRID_OVERRIDE"
else
  HYBRID="$RESULTS_ROOT/step7_mono_po_hybrid_ik/$SEQ/estimate.txt"
fi

if [[ -n "$GT_OVERRIDE" ]]; then
  GT="$GT_OVERRIDE"
else
  DATASETS_ROOT="${DATASETS_DIR:-}"
  if [[ -z "$DATASETS_ROOT" ]]; then
    DATASETS_ROOT="$(resolve_root "machine_hall/MH_01_easy/MH_01_easy/mav0/state_groundtruth_estimate0/data.txt" \
      /datasets \
      /home/aze-pc-0266/datasets \
      "$HOME/datasets")" || true
  fi
  if [[ -z "${DATASETS_ROOT:-}" ]]; then
    echo "Missing datasets root; set DATASETS_DIR or mount /datasets" >&2
    exit 1
  fi
  GT="$(resolve_gt_path "$DATASETS_ROOT")"
fi

missing=()
for f in "$MSCKF" "$HYBRID" "$GT"; do
  [[ -f "$f" ]] || missing+=("$f")
done

if [[ ${#missing[@]} -gt 0 ]]; then
  echo "Missing file(s):" >&2
  for f in "${missing[@]}"; do
    echo "  $f" >&2
  done
  echo >&2
  echo "Your results live on the host at ~/results but this container may not have them mounted." >&2
  echo "Either re-start Docker with:" >&2
  echo "  -v /home/aze-pc-0266/results:/results -v /home/aze-pc-0266/datasets:/datasets" >&2
  echo "or from the host run:" >&2
  echo "  ./scripts/traj_compare/run_compare_video_host.sh $SEQ" >&2
  exit 1
fi

if [[ "$RECORD" == "DEFAULT" ]]; then
  RECORD="${RESULTS_ROOT}/videos/${SEQ}_msckf_hybrid_gt.mp4"
fi

# Host path -> container mount
if [[ -n "$RECORD" && "$RECORD" == /home/aze-pc-0266/results/* && -d /results ]]; then
  RECORD="/results/${RECORD#/home/aze-pc-0266/results/}"
fi

if [[ -n "$RECORD" ]]; then
  SHUTDOWN_WHEN_DONE="true"
fi

if ! rostopic list >/dev/null 2>&1; then
  roscore >/tmp/roscore_compare.log 2>&1 &
  sleep 2
fi

echo "============================================================"
echo "Sequence: $SEQ"
echo "  MSCKF : $MSCKF"
echo "  Hybrid: $HYBRID"
echo "  GT    : $GT"
echo "  Speed : ${SPEED}x"
if [[ -n "$RECORD" ]]; then
  echo "  Video : $RECORD"
fi
echo "============================================================"

ensure_record_tools() {
  if command -v ffmpeg >/dev/null 2>&1 && command -v xwininfo >/dev/null 2>&1; then
    return 0
  fi
  echo "Installing ffmpeg + x11-utils for RViz capture..."
  apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ffmpeg x11-utils >/dev/null
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

LAUNCH_ARGS=(
  ov_eval compare_trajectories.launch
  msckf_path:="$MSCKF"
  hybrid_path:="$HYBRID"
  gt_path:="$GT"
  speed:="$SPEED"
  loop:="$LOOP"
  shutdown_when_done:="$SHUTDOWN_WHEN_DONE"
)

if [[ -z "$RECORD" || "${RECORD_FROM_HOST:-}" == "1" ]]; then
  set +e
  roslaunch "${LAUNCH_ARGS[@]}"
  exit $?
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY not set; cannot record RViz." >&2
  exit 1
fi

ensure_record_tools
mkdir -p "$(dirname "$RECORD")"

set +e
roslaunch "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!
set -e

GEOM=""
for _ in $(seq 1 80); do
  if GEOM="$(find_rviz_window 2>/dev/null)"; then
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    wait "$LAUNCH_PID" || true
    echo "RViz exited before a window appeared; no video written." >&2
    exit 1
  fi
  sleep 0.25
done

if [[ -z "$GEOM" ]]; then
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  echo "Could not find RViz window to record." >&2
  exit 1
fi

read -r RVIZ_WID RVIZ_W RVIZ_H RVIZ_X RVIZ_Y <<<"$GEOM"
echo "Recording RViz window ${RVIZ_WID} (${RVIZ_W}x${RVIZ_H}+${RVIZ_X}+${RVIZ_Y}) crf=${RECORD_CRF} preset=${RECORD_PRESET}"
if command -v xdotool >/dev/null 2>&1; then
  xdotool windowactivate "$RVIZ_WID" 2>/dev/null || true
  xdotool windowraise "$RVIZ_WID" 2>/dev/null || true
fi
sleep 4

FFMPEG_LOG="$(dirname "$RECORD")/${SEQ}_ffmpeg.log"
mkdir -p "$(dirname "$RECORD")"
ffmpeg -y \
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
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi

set +e
wait "$LAUNCH_PID"
RC=$?
set -e

sleep 0.5
kill -INT "$FFMPEG_PID" 2>/dev/null || true
wait "$FFMPEG_PID" 2>/dev/null || true

if [[ -s "$RECORD" ]]; then
  echo "Wrote video: $RECORD"
  echo "ffmpeg log: $FFMPEG_LOG"
else
  echo "Recording failed. ffmpeg log:" >&2
  cat "$FFMPEG_LOG" >&2 || true
  exit 1
fi

exit "$RC"
