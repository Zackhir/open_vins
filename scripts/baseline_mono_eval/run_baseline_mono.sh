#!/usr/bin/env bash
# Mono MSCKF baseline runner (OpenVINS serial.launch).
#
# Intended to run INSIDE ov_ros1_20_04 with mounts:
#   /catkin_ws  <- catkin workspace
#   /datasets   <- bags + groundtruth
#   /results    <- output root (or set RESULTS_DIR)
#
# Host launcher example is in BASELINE_EVAL_GUIDE.md

set -euo pipefail

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

RESULTS_ROOT="${RESULTS_DIR:-/results/baseline_mono_msckf}"
DATASETS_ROOT="${DATASETS_DIR:-/datasets}"
mkdir -p "$RESULTS_ROOT"

SUMMARY_CSV="$RESULTS_ROOT/summary.csv"
echo "sequence,config,rmse_pos_m,rmse_ori_deg,ms_per_frame,fps,cpu_mean_pct,cpu_max_pct,n_poses,status" > "$SUMMARY_CSV"

# Keep a single roscore for all runs
if ! rostopic list >/dev/null 2>&1; then
  roscore >/tmp/roscore_baseline.log 2>&1 &
  sleep 2
fi

sample_cpu() {
  local outfile="$1"
  local pattern="$2"
  : > "$outfile"
  echo "# time_sec cpu_percent rss_mb nthreads" >> "$outfile"
  while true; do
    local pid
    pid=$(pgrep -n -f "$pattern" || true)
    if [[ -n "${pid}" ]]; then
      python3 - "$pid" "$outfile" <<'PY'
import sys, time, psutil
pid = int(sys.argv[1])
out = sys.argv[2]
try:
    p = psutil.Process(pid)
except psutil.NoSuchProcess:
    sys.exit(0)
p.cpu_percent(None)
while p.is_running():
    try:
        cpu = p.cpu_percent(0.5)
        mem = p.memory_info().rss / (1024.0 * 1024.0)
        th = p.num_threads()
        with open(out, "a") as f:
            f.write("%.3f %.3f %.2f %d\n" % (time.time(), cpu, mem, th))
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        break
PY
      break
    fi
    sleep 0.2
  done
}

run_one() {
  local name="$1"
  local config="$2"
  local dataset="$3"
  local bag="$4"
  local gt="$5"
  local bag_start="$6"

  local out="$RESULTS_ROOT/$name"
  mkdir -p "$out"
  local est="$out/estimate.txt"
  local timing="$out/timing.txt"
  local cpu="$out/cpu.txt"
  local log="$out/run.log"

  echo "============================================================"
  echo "[RUN] $name  config=$config  bag_start=$bag_start"
  echo "============================================================"

  sample_cpu "$cpu" "ros1_serial_msckf" &
  local cpu_pid=$!

  set +e
  roslaunch ov_msckf serial.launch \
    config:="$config" \
    dataset:="$dataset" \
    bag:="$bag" \
    bag_start:="$bag_start" \
    max_cameras:=1 \
    use_stereo:=false \
    dosave:=true \
    dotime:=true \
    path_est:="$est" \
    path_time:="$timing" \
    >"$log" 2>&1
  local rc=$?
  set -e

  kill "$cpu_pid" 2>/dev/null || true
  wait "$cpu_pid" 2>/dev/null || true

  if [[ $rc -ne 0 || ! -s "$est" ]]; then
    echo "$name,$config,,,,,,,$rc" >> "$SUMMARY_CSV"
    echo "[FAIL] $name (rc=$rc). See $log"
    return 0
  fi

  echo "[OK] wrote $est and $timing"
}

# ---- EuRoC Machine Hall (OpenVINS recommended bag_start) ----
run_one MH_01_easy euroc_mav MH_01_easy \
  "$DATASETS_ROOT/machine_hall/MH_01_easy/MH_01_easy.bag" \
  "$DATASETS_ROOT/machine_hall/MH_01_easy/MH_01_easy/mav0/state_groundtruth_estimate0/data.txt" \
  40

run_one MH_02_easy euroc_mav MH_02_easy \
  "$DATASETS_ROOT/machine_hall/MH_02_easy/MH_02_easy.bag" \
  "$DATASETS_ROOT/machine_hall/MH_02_easy/MH_02_easy/mav0/state_groundtruth_estimate0/data.txt" \
  35

run_one MH_03_medium euroc_mav MH_03_medium \
  "$DATASETS_ROOT/machine_hall/MH_03_medium/MH_03_medium.bag" \
  "$DATASETS_ROOT/machine_hall/MH_03_medium/MH_03_medium/mav0/state_groundtruth_estimate0/data.txt" \
  17.5

run_one MH_04_difficult euroc_mav MH_04_difficult \
  "$DATASETS_ROOT/machine_hall/MH_04_difficult/MH_04_difficult.bag" \
  "$DATASETS_ROOT/machine_hall/MH_04_difficult/MH_04_difficult/mav0/state_groundtruth_estimate0/data.txt" \
  15

run_one MH_05_difficult euroc_mav MH_05_difficult \
  "$DATASETS_ROOT/machine_hall/MH_05_difficult/MH_05_difficult.bag" \
  "$DATASETS_ROOT/machine_hall/MH_05_difficult/MH_05_difficult/mav0/state_groundtruth_estimate0/data.txt" \
  15

# ---- UZH FPV outdoor ----
run_one outdoor_forward_1 uzhfpv_outdoor outdoor_forward_1_snapdragon_with_gt \
  "$DATASETS_ROOT/uzhfpv_outdoor_forward/forward_1/outdoor_forward_1_snapdragon_with_gt.bag" \
  "$DATASETS_ROOT/uzhfpv_outdoor_forward/forward_1/outdoor_forward_1_snapdragon_with_gt/groundtruth.txt" \
  0

run_one outdoor_forward_3 uzhfpv_outdoor outdoor_forward_3_snapdragon_with_gt \
  "$DATASETS_ROOT/uzhfpv_outdoor_forward/forward_3/outdoor_forward_3_snapdragon_with_gt.bag" \
  "$DATASETS_ROOT/uzhfpv_outdoor_forward/forward_3/outdoor_forward_3_snapdragon_with_gt/groundtruth.txt" \
  0

run_one outdoor_forward_5 uzhfpv_outdoor outdoor_forward_5_snapdragon_with_gt \
  "$DATASETS_ROOT/uzhfpv_outdoor_forward/forward_5/outdoor_forward_5_snapdragon_with_gt.bag" \
  "$DATASETS_ROOT/uzhfpv_outdoor_forward/forward_5/outdoor_forward_5_snapdragon_with_gt/groundtruth.txt" \
  0

run_one outdoor_45_1 uzhfpv_outdoor_45 outdoor_45_1_snapdragon_with_gt \
  "$DATASETS_ROOT/uzhfpv_outdoor_45/outdoor_45_1_snapdragon_with_gt.bag" \
  "$DATASETS_ROOT/uzhfpv_outdoor_45/outdoor_45_1_snapdragon_with_gt/groundtruth.txt" \
  0

echo "DONE estimator runs. Outputs in $RESULTS_ROOT"
echo "Next: recompute metrics + PDF (see BASELINE_EVAL_GUIDE.md)"
