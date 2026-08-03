# Baseline Mono MSCKF Evaluation Guide

Reproducible protocol for OpenVINS **mono** MSCKF baseline (Step 1.5), before PO-MSCKF comparison.

Tools live in:

```text
/home/aze-pc-0266/workspace/eval/baseline_mono/
```

Results are written to:

```text
/home/aze-pc-0266/results/baseline_mono_msckf/
```

---

## 1. What this baseline is

| Setting | Value |
|--------|--------|
| Estimator | OpenVINS MSCKF (`serial.launch`) |
| Cameras | **Mono** — `max_cameras:=1`, `use_stereo:=false` (left / `cam0` only) |
| Alignment | `posyaw` via `ov_eval error_singlerun` |
| Table metrics | Trans. RMSE (m), Ori. RMSE (deg), ms/frame, FPS, CPU mean/max % |
| Error plots | OpenVINS `error_dataset`-style RMSE (ori + pos vs time) |
| Trajectories | XY and Z vs GT |

**Do not use** a separate `rosbag play` with `serial.launch` — the serial node already reads the bag.

**FPS note:** offline serial processing throughput = `1 / mean_total_time` from `timing.txt`. This is **not** the log `(xx hz)` print rate.

**CPU:** mean/max `%` of process `ros1_serial_msckf` sampled with psutil (100% ≈ one full core).

**EuRoC GT:** convert CSV → TXT once with `format_converter` (already done on this machine).

---

## 2. Paths / environment

Assumes your usual Docker helper mounts:

| Host | Container |
|------|-----------|
| `$DOCKER_CATKINWS` → usually `~/workspace/catkin_ws_ov` | `/catkin_ws` |
| `$DOCKER_DATASETS` → usually `~/datasets` | `/datasets` |
| `~/results` | `/results` |
| `~/workspace/eval` | `/workspace_eval` (optional; see commands below) |

Export if needed:

```bash
export DOCKER_CATKINWS=/home/aze-pc-0266/workspace/catkin_ws_ov
export DOCKER_DATASETS=/home/aze-pc-0266/datasets
```

---

## 3. Sequences included

### EuRoC Machine Hall

| Name | Config | Bag | bag_start |
|------|--------|-----|-----------|
| MH_01_easy | `euroc_mav` | `/datasets/machine_hall/MH_01_easy/MH_01_easy.bag` | 40 |
| MH_02_easy | `euroc_mav` | `.../MH_02_easy/MH_02_easy.bag` | 35 |
| MH_03_medium | `euroc_mav` | `.../MH_03_medium/MH_03_medium.bag` | 17.5 |
| MH_04_difficult | `euroc_mav` | `.../MH_04_difficult/MH_04_difficult.bag` | 15 |
| MH_05_difficult | `euroc_mav` | `.../MH_05_difficult/MH_05_difficult.bag` | 15 |

GT (already converted):

```text
/datasets/machine_hall/<SEQ>/<SEQ>/mav0/state_groundtruth_estimate0/data.txt
```

### UZH FPV outdoor

| Name | Config | Bag | bag_start |
|------|--------|-----|-----------|
| outdoor_forward_1 | `uzhfpv_outdoor` | `/datasets/uzhfpv_outdoor_forward/forward_1/outdoor_forward_1_snapdragon_with_gt.bag` | 0 |
| outdoor_forward_3 | `uzhfpv_outdoor` | `.../forward_3/...` | 0 |
| outdoor_forward_5 | `uzhfpv_outdoor` | `.../forward_5/...` | 0 |
| outdoor_45_1 | `uzhfpv_outdoor_45` | `/datasets/uzhfpv_outdoor_45/outdoor_45_1_snapdragon_with_gt.bag` | 0 |

Use **`uzhfpv_outdoor_45`** for the 45° sequence (not the forward config).

---

## 4. One-shot full baseline (recommended)

### Terminal A — roscore (only if you run sequences manually)

Not required for the batch script below (it starts `roscore` itself).

```bash
ov_docker ov_ros1_20_04 bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roscore
```

### Host — run all estimator sequences + save timing/CPU

```bash
mkdir -p /home/aze-pc-0266/results/baseline_mono_msckf

docker run --rm --net=host \
  -e MPLBACKEND=Agg \
  -e RESULTS_DIR=/results/baseline_mono_msckf \
  -e DATASETS_DIR=/datasets \
  --mount type=bind,source="$DOCKER_CATKINWS",target=/catkin_ws \
  --mount type=bind,source="$DOCKER_DATASETS",target=/datasets \
  --mount type=bind,source=/home/aze-pc-0266/results,target=/results \
  --mount type=bind,source=/home/aze-pc-0266/workspace/eval,target=/workspace_eval \
  ov_ros1_20_04:latest \
  bash /workspace_eval/baseline_mono/run_baseline_mono.sh
```

This writes per sequence under `~/results/baseline_mono_msckf/<seq>/`:

- `estimate.txt`
- `timing.txt`
- `cpu.txt`
- `run.log`

### Host — recompute official ATE numbers + fill `summary.csv` / `TABLE.md`

```bash
docker run --rm --net=host \
  -e MPLBACKEND=Agg \
  -e RESULTS_DIR=/results/baseline_mono_msckf \
  -e DATASETS_DIR=/datasets \
  --mount type=bind,source="$DOCKER_CATKINWS",target=/catkin_ws \
  --mount type=bind,source="$DOCKER_DATASETS",target=/datasets \
  --mount type=bind,source=/home/aze-pc-0266/results,target=/results \
  --mount type=bind,source=/home/aze-pc-0266/workspace/eval,target=/workspace_eval \
  ov_ros1_20_04:latest \
  bash -lc 'source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; \
    python3 /workspace_eval/baseline_mono/recompute_baseline_metrics.py'
```

### Host — build large readable PDF report

```bash
docker run --rm \
  -e MPLBACKEND=Agg \
  -e RESULTS_DIR=/results/baseline_mono_msckf \
  -e DATASETS_DIR=/datasets \
  --mount type=bind,source="$DOCKER_DATASETS",target=/datasets \
  --mount type=bind,source=/home/aze-pc-0266/results,target=/results \
  --mount type=bind,source=/home/aze-pc-0266/workspace/eval,target=/workspace_eval \
  ov_ros1_20_04:latest \
  bash -lc 'python3 /workspace_eval/baseline_mono/make_hires_pdf.py'
```

**PDF output:**

```text
~/results/baseline_mono_msckf/baseline_mono_msckf_report.pdf
```

---

## 5. Manual single-sequence example (EuRoC MH_01)

Use this when debugging one bag.

### Terminal 1

```bash
ov_docker ov_ros1_20_04 bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roscore
```

### Terminal 2 — estimator (no rosbag play)

```bash
ov_docker ov_ros1_20_04 bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

SEQ=MH_01_easy
OUT=/results/baseline_mono_msckf/${SEQ}
mkdir -p "$OUT"

roslaunch ov_msckf serial.launch \
  config:=euroc_mav \
  dataset:=${SEQ} \
  bag:=/datasets/machine_hall/${SEQ}/${SEQ}.bag \
  bag_start:=40 \
  max_cameras:=1 \
  use_stereo:=false \
  dosave:=true \
  dotime:=true \
  path_est:=${OUT}/estimate.txt \
  path_time:=${OUT}/timing.txt
```

### After run — accuracy

```bash
GT=/datasets/machine_hall/${SEQ}/${SEQ}/mav0/state_groundtruth_estimate0/data.txt
rosrun ov_eval error_singlerun posyaw "$GT" ${OUT}/estimate.txt
rosrun ov_eval plot_trajectories posyaw "$GT" ${OUT}/estimate.txt
```

### After run — ms/frame and FPS

```bash
rosrun ov_eval timing_flamegraph ${OUT}/timing.txt
# ms/frame = mean_time(total) * 1000
# FPS     = 1 / mean_time(total)
```

### UZH outdoor forward_1 (mono)

```bash
SEQ=outdoor_forward_1_snapdragon_with_gt
OUT=/results/baseline_mono_msckf/outdoor_forward_1
mkdir -p "$OUT"

roslaunch ov_msckf serial.launch \
  config:=uzhfpv_outdoor \
  dataset:=${SEQ} \
  bag:=/datasets/uzhfpv_outdoor_forward/forward_1/${SEQ}.bag \
  bag_start:=0 \
  max_cameras:=1 \
  use_stereo:=false \
  dosave:=true \
  dotime:=true \
  path_est:=${OUT}/estimate.txt \
  path_time:=${OUT}/timing.txt
```

```bash
GT=/datasets/uzhfpv_outdoor_forward/forward_1/${SEQ}/groundtruth.txt
rosrun ov_eval error_singlerun posyaw "$GT" ${OUT}/estimate.txt
```

### UZH outdoor 45°

```bash
config:=uzhfpv_outdoor_45
bag:=/datasets/uzhfpv_outdoor_45/outdoor_45_1_snapdragon_with_gt.bag
```

---

## 6. EuRoC groundtruth conversion (once)

Only if `data.txt` is missing:

```bash
rosrun ov_eval format_converter \
  /datasets/machine_hall/MH_01_easy/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv
```

---

## 7. Tools in this folder

| File | Role |
|------|------|
| `run_baseline_mono.sh` | Run all mono serial evaluations + CPU sampling |
| `recompute_baseline_metrics.py` | Official `ov_eval` ATE → `summary.csv`, `TABLE.md`, per-seq metrics |
| `make_hires_pdf.py` | Large PDF: table + trajectory + RMSE plots |
| `BASELINE_EVAL_GUIDE.md` | This guide |

---

## 8. Output layout

```text
~/results/baseline_mono_msckf/
  TABLE.md
  summary.csv
  baseline_mono_msckf_report.pdf
  MH_01_easy/
    estimate.txt
    timing.txt
    cpu.txt
    metrics.txt
    rmse.png
    traj_xy.png          # if regenerated by older scripts
    run.log
    ov_eval_error_singlerun.txt
  MH_02_easy/
  ...
  outdoor_45_1/
```

---

## 9. Later: PO-MSCKF comparison

Repeat the **same** protocol with `use_pose_only_update:=true` (after Step 4+), write to a separate results folder (e.g. `baseline_mono_po_msckf/`), then compare tables/PDFs side by side.

Do **not** change mono/stereo, bag starts, alignment mode, or timing method between MSCKF and PO runs.
