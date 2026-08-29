# PO-MSCKF in OpenVINS (this fork)

Pose-only visual update for OpenVINS MSCKF: same state vector (IMU clones, biases, calib), **no triangulated landmark in the filter**, **no nullspace projection** on the PO path. Classical MSCKF remains available when the flag is off.

**Paper:** Du et al., *Pose-Only Visual Inertial Odometry*, [arXiv:2407.01888](https://arxiv.org/abs/2407.01888)

**Author / fork:** [Zackhir/open_vins](https://github.com/Zackhir/open_vins)

---

## Quick start

Classical MSCKF (default):

```bash
roslaunch ov_msckf serial.launch \
  config:=euroc_mav \
  max_cameras:=1 use_stereo:=false \
  use_pose_only_update:=false \
  bag:=/path/to/bag.bag dataset:=MH_01_easy bag_start:=40
```

PO-MSCKF with the **locked** noise model \(R=\sigma^2(GG^\top+I)\):

```bash
roslaunch ov_msckf serial.launch \
  config:=euroc_mav \
  max_cameras:=1 use_stereo:=false \
  use_pose_only_update:=true \
  po_variant:=hybrid_gg_plus_i \
  bag:=/path/to/bag.bag dataset:=MH_01_easy bag_start:=40
```

**Recommended tuned overrides** (from the 9-sequence param study; YAML stays default otherwise):

```bash
roslaunch ov_msckf serial.launch \
  ... \
  use_pose_only_update:=true \
  po_variant:=hybrid_gg_plus_i \
  max_clones:=15 \
  max_msckf_in_update:=40
```

Simulation (consistency / NEES):

```bash
roslaunch ov_msckf simulation.launch \
  config:=rpng_sim \
  use_pose_only_update:=true \
  po_variant:=hybrid_gg_plus_i
```

YAML keys (example `config/euroc_mav/estimator_config.yaml`):

```yaml
use_pose_only_update: false   # true → UpdaterPO
# po_variant set via launch or rosparam: hybrid_gg_plus_i | hybrid_ik | ...
```

Launch overrides for param study (see `ov_msckf/launch/serial.launch`): `max_clones`, `max_msckf_in_update`, `num_pts`, `max_slam`, `max_slam_in_update`, `fast_threshold`, `min_px_dist`, `up_msckf_chi2_multipler` (`-1` / empty = keep YAML).

---

## What changed in the codebase

| Piece | Role |
|-------|------|
| `use_pose_only_update` | `false` → `UpdaterMSCKF`; `true` → `UpdaterPO` |
| `po_variant` | Exclusive residual / noise recipe (see below) |
| `ov_msckf/src/update/UpdaterPO.{h,cpp}` | PO residual, Jacobians, whitening, χ² → compress → EKF |
| `ov_msckf/src/update/PoseOnlyGeometry.{h,cpp}` | Geometry (virtual stereo / base views \(j,k\)) |
| `ov_msckf/src/state/StateOptions.h` | Options + variant parsing |
| `ov_msckf/src/core/VioManager.cpp` | Routes MSCKF features to PO when flag on |
| Unit / FD tests | `test_pose_only_geometry.cpp`, `test_pose_only_jacobians.cpp`, … |
| RViz path compare | `ov_eval` launch + publisher + `scripts/traj_compare/` (see below) |

Shared pipeline after residual build: **χ² gate → QR compress → EKF update**. SLAM features (if enabled) still use the classical SLAM updater.

---

## PO variants

Set with `po_variant:=...` (ignored unless `use_pose_only_update:=true`).

| `po_variant` | Idea | Status |
|--------------|------|--------|
| `bearing_skip_ik` | Skip base-frame bearing when \(i=k\); \(GG^\top\) whitening | Experimental / default in launch |
| `isotropic_ik` | Include \(i=k\); isotropic \(\sigma^2\) then whiten | Experimental |
| `hybrid_ik` | Include \(i=k\); eig-based floor on \(R\) | Usable; heavier than GG+I |
| **`hybrid_gg_plus_i`** | Same residual/G as hybrid_ik; **\(R=\sigma^2(GG^\top+I)\)** (one Cholesky, no eig) | **Kept / recommended** |

Removed from the locked evaluation story: `tangent_ik` (diverged under calib perturbation), Mini-IEKF (`po_iekf`).

---

## Why \(R=\sigma^2(GG^\top+I)\)

- Classical MSCKF: triangulation + nullspace removes the landmark; OpenVINS then uses isotropic pixel noise.
- PO: residuals still depend on the two base-view pixels → **correlated** measurement noise \(R \propto GG^\top\).
- On the \(i=k\) row, \(G=0\) → need a floor; **`+I`** gives a cheap positive-definite \(R\) without eigendecomposition.

Initial PO with a wrong / isotropic-only \(R\) **diverged** in simulation (huge NEES/RMSE). GG+I restores finite real-data RMSE and keeps CPU occupancy matched to MSCKF.

---

## Locked evaluation protocol (9 sequences)

Fairness: same bags, mono cam0, alignment, and timing between MSCKF and PO.

| Item | Value |
|------|--------|
| Mode | `max_cameras:=1`, `use_stereo:=false` |
| Alignment | `posyaw` via `ov_eval error_singlerun` |
| Sequences | EuRoC MH_01–05; UZH-FPV `outdoor_forward_1/3/5`, `outdoor_45_1` |
| CPU | psutil mean % of `ros1_serial_msckf` ÷ cores (%/core) |
| Latency | mean `timing.txt` total → ms/frame; FPS \(=1/\)mean |

**Headline means (mono, FEJ):**

| Config | CPU %/core | ms/frame | pos RMSE (m) | ori RMSE (°) |
|--------|------------|----------|--------------|--------------|
| MSCKF (YAML) | 5.67 | 4.63 | 0.375 | 1.437 |
| PO GG+I (YAML) | **5.58** | 5.06 | 0.359 | 1.289 |
| PO GG+I **tuned** (`max_clones=15`, `max_msckf_in_update=40`) | 5.64 | 5.85 | **0.325** | **1.252** |

GG+I does not cost extra CPU vs MSCKF; ms/frame rises slightly but stays \(\ll 50\,\mathrm{ms}\) (20 Hz budget).

Param study takeaway: stacking **all** one-axis winners hurts; the **minimal** override pair (`clones=15`, `msckf_in_update=40`) is best.

Simulation (`rpng_sim`): MSCKF NEES stays healthy (~1–3). GG+I has finite RMSE but can remain overconfident (NEES higher than MSCKF)—consistency is a separate track from bag ATE.

---

## Trajectory plots (MSCKF vs PO vs GT)

Animated **RViz** overlay of three TUM trajectories after **posyaw** alignment to ground truth:

| File | Role |
|------|------|
| `ov_eval/launch/compare_trajectories.launch` | Launch publisher + optional RViz |
| `ov_eval/python/path_compare_publisher.py` | Publishes three `nav_msgs/Path` topics (MSCKF / PO / GT) |
| `ov_eval/rviz/compare_three_paths.rviz` | RViz config for the overlay |
| `scripts/traj_compare/run_compare_video.sh` | In-container runner (pick sequence + estimate paths) |
| `scripts/traj_compare/run_compare_video_host.sh` | Host wrapper: Docker + X11 (+ optional `--record` to mp4) |

**Args** (`compare_trajectories.launch`): `msckf_path`, `hybrid_path` (PO estimate; name kept for history), `gt_path`, `speed` (replay multiplier), `loop`, `hold_sec`, `rviz`.

**Host example** (needs Docker image, datasets/results mounts, and a graphical `DISPLAY`):

```bash
cd /path/to/open_vins
./scripts/traj_compare/run_compare_video_host.sh MH_01_easy --speed 15
./scripts/traj_compare/run_compare_video_host.sh MH_03_medium --speed 15 --record
```

Direct launch (paths are absolute TUM `estimate.txt` / GT files):

```bash
roslaunch ov_eval compare_trajectories.launch \
  msckf_path:=/path/to/msckf/estimate.txt \
  hybrid_path:=/path/to/po_gg_plus_i/estimate.txt \
  gt_path:=/path/to/groundtruth.txt \
  speed:=5.0
```

Static XY / Z / RMSE panel plots used in reports are generated outside this package (see `~/workspace/po_msckf_report/scripts/`).

---

## Related paths outside this repo

Local report / eval materials (not part of upstream OpenVINS):

- Technical / weekly reports under `~/workspace/po_msckf_report/`
- Eval runners / summaries under `~/workspace/eval/` and `~/workspace/results/`

---

## Citation

If you use the pose-only idea, cite Du et al. (arXiv:2407.01888). For the base estimator, cite OpenVINS (Geneva et al., ICRA 2020) as in the main `ReadMe.md`.
