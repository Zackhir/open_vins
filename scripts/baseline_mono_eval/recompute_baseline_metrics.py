#!/usr/bin/env python3
"""Recompute mono baseline metrics/plots with OpenVINS-compatible posyaw ATE."""

import math
import os
import re
import subprocess

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_tum(path):
    times, xyz, quat = [], [], []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            times.append(float(p[0]))
            xyz.append([float(p[1]), float(p[2]), float(p[3])])
            # qx qy qz qw (JPL storage order used by OpenVINS files)
            quat.append([float(p[4]), float(p[5]), float(p[6]), float(p[7])])
    return np.asarray(times), np.asarray(xyz), np.asarray(quat)


def associate(est_t, gt_t, max_diff=0.02):
    """OpenVINS-style injective nearest association."""
    gt_pointer = 0
    ie, ig = [], []
    for i, t in enumerate(est_t):
        best_diff = max_diff
        best_gt = -1
        while gt_pointer < len(gt_t) and gt_t[gt_pointer] < t and abs(gt_t[gt_pointer] - t) > max_diff:
            gt_pointer += 1
        while gt_pointer < len(gt_t) and abs(gt_t[gt_pointer] - t) <= max_diff:
            d = abs(gt_t[gt_pointer] - t)
            if d >= best_diff:
                break
            best_diff = d
            best_gt = gt_pointer
            gt_pointer += 1
        if best_gt != -1:
            ie.append(i)
            ig.append(best_gt)
    return np.asarray(ie), np.asarray(ig)


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def jpl_quat_to_R(q):
    """OpenVINS JPL quat_2_Rot: q=[qx,qy,qz,qw]."""
    q = q / np.linalg.norm(q)
    qv = q[:3]
    qw = q[3]
    R = (2 * qw * qw - 1.0) * np.eye(3) - 2 * qw * skew(qv) + 2.0 * np.outer(qv, qv)
    return R


def jpl_quat_multiply(q, p):
    """OpenVINS JPL quat_multiply(q,p)."""
    qw, pw = q[3], p[3]
    qv, pv = q[:3], p[:3]
    out = np.zeros(4)
    out[:3] = qw * pv + pw * qv - np.cross(qv, pv)
    out[3] = qw * pw - qv.dot(pv)
    return out / np.linalg.norm(out)


def jpl_inv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def rot_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def get_best_yaw(C):
    A = C[0, 1] - C[1, 0]
    B = C[0, 0] + C[1, 1]
    return math.atan2(A, B)


def align_posyaw_umeyama(pos_est, pos_gt):
    """OpenVINS AlignUtils::align_umeyama(data=est, model=gt, known_scale, yaw_only)."""
    mu_M = pos_gt.mean(axis=0)
    mu_D = pos_est.mean(axis=0)
    model_zc = pos_gt - mu_M
    data_zc = pos_est - mu_D
    n = float(len(pos_est))
    C = (model_zc.T @ data_zc) / n
    rot_C = n * C.T
    theta = get_best_yaw(rot_C)
    R = rot_z(theta)
    t = mu_M - R @ mu_D
    return R, t, theta


def so3_log_norm(R):
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return abs(math.acos(tr))


def compute_ate(gt_path, est_path):
    t_gt, p_gt, q_gt = load_tum(gt_path)
    t_est, p_est, q_est = load_tum(est_path)
    ie, ig = associate(t_est, t_gt, 0.02)
    if len(ie) < 3:
        raise RuntimeError("not enough associations")
    pe, pg = p_est[ie], p_gt[ig]
    qe, qg = q_est[ie], q_gt[ig]
    tt = t_est[ie]
    R, t, yaw = align_posyaw_umeyama(pe, pg)
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])  # JPL yaw about z

    err_pos, err_ori = [], []
    pe_a, qe_a = [], []
    for i in range(len(pe)):
        xyz_a = R @ pe[i] + t
        # OpenVINS: quat_multiply(est, Inv(q_ESTtoGT)); q_ESTtoGT = rot_2_quat(R)
        q_a = jpl_quat_multiply(qe[i], jpl_inv(q_yaw))
        e_R = jpl_quat_to_R(q_a).T @ jpl_quat_to_R(qg[i])
        err_ori.append(180.0 / math.pi * so3_log_norm(e_R))
        err_pos.append(np.linalg.norm(pg[i] - xyz_a))
        pe_a.append(xyz_a)
        qe_a.append(q_a)
    err_pos = np.asarray(err_pos)
    err_ori = np.asarray(err_ori)
    pe_a = np.asarray(pe_a)
    return {
        "n": int(len(err_pos)),
        "rmse_pos": float(np.sqrt(np.mean(err_pos ** 2))),
        "rmse_ori": float(np.sqrt(np.mean(err_ori ** 2))),
        "t": tt - tt[0],
        "err_pos": err_pos,
        "err_ori": err_ori,
        "pe_xy": pe_a[:, :2],
        "pg_xy": pg[:, :2],
        "pe_z": pe_a[:, 2],
        "pg_z": pg[:, 2],
    }


def parse_timing(path):
    totals = []
    with open(path) as f:
        for i, ln in enumerate(f):
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split(",")
            try:
                totals.append(float(parts[-1]))
            except ValueError:
                continue
    if not totals:
        return None
    arr = np.asarray(totals)
    return {
        "ms_per_frame": float(arr.mean() * 1000.0),
        "fps": float(1.0 / arr.mean()),
        "n": int(arr.size),
    }


def parse_cpu(path):
    cpus = []
    with open(path) as f:
        for ln in f:
            if not ln.strip() or ln.startswith("#"):
                continue
            parts = ln.split()
            if len(parts) >= 2:
                cpus.append(float(parts[1]))
    if not cpus:
        return None
    arr = np.asarray(cpus)
    return {"mean_cpu": float(arr.mean()), "max_cpu": float(arr.max())}


def plot_results(out_dir, name, ate):
    os.makedirs(out_dir, exist_ok=True)
    plt.figure(figsize=(7, 6))
    plt.plot(ate["pg_xy"][:, 0], ate["pg_xy"][:, 1], "k-", label="GT", lw=2)
    plt.plot(ate["pe_xy"][:, 0], ate["pe_xy"][:, 1], "b-", label="Estimate", lw=1.5)
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("%s — Trajectory XY" % name)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "traj_xy.png"), dpi=150)
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(ate["t"], ate["pg_z"], "k-", label="GT")
    plt.plot(ate["t"], ate["pe_z"], "b-", label="Estimate")
    plt.xlabel("time (s)")
    plt.ylabel("z (m)")
    plt.title("%s — Trajectory Z" % name)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "traj_z.png"), dpi=150)
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(ate["t"], ate["err_pos"], "b-")
    plt.xlabel("time (s)")
    plt.ylabel("translation error (m)")
    plt.title("%s — Translation ATE  RMSE=%.3f m" % (name, ate["rmse_pos"]))
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "error_translation.png"), dpi=150)
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(ate["t"], ate["err_ori"], "r-")
    plt.xlabel("time (s)")
    plt.ylabel("orientation error (deg)")
    plt.title("%s — Orientation ATE  RMSE=%.3f deg" % (name, ate["rmse_ori"]))
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "error_orientation.png"), dpi=150)
    plt.close()


def ov_eval_rmse(gt, est):
    cmd = ["rosrun", "ov_eval", "error_singlerun", "posyaw", gt, est]
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    out = p.stdout
    m = re.search(r"rmse_ori\s*=\s*([0-9.]+)\s*\|\s*rmse_pos\s*=\s*([0-9.]+)", out)
    if not m:
        raise RuntimeError("failed to parse ov_eval for %s\n%s" % (est, out[-500:]))
    # also grab a couple RPE lines
    rpe = re.findall(r"seg\s+(\d+)\s+-\s+median_ori\s*=\s*([0-9.]+)\s*\|\s*median_pos\s*=\s*([0-9.]+)", out)
    return float(m.group(1)), float(m.group(2)), rpe, out


DATASETS = os.environ.get("DATASETS_DIR", "/datasets")
RESULTS = os.environ.get("RESULTS_DIR", "/results/baseline_mono_msckf")

SEQS = [
    ("MH_01_easy", f"{DATASETS}/machine_hall/MH_01_easy/MH_01_easy/mav0/state_groundtruth_estimate0/data.txt"),
    ("MH_02_easy", f"{DATASETS}/machine_hall/MH_02_easy/MH_02_easy/mav0/state_groundtruth_estimate0/data.txt"),
    ("MH_03_medium", f"{DATASETS}/machine_hall/MH_03_medium/MH_03_medium/mav0/state_groundtruth_estimate0/data.txt"),
    ("MH_04_difficult", f"{DATASETS}/machine_hall/MH_04_difficult/MH_04_difficult/mav0/state_groundtruth_estimate0/data.txt"),
    ("MH_05_difficult", f"{DATASETS}/machine_hall/MH_05_difficult/MH_05_difficult/mav0/state_groundtruth_estimate0/data.txt"),
    ("outdoor_forward_1", f"{DATASETS}/uzhfpv_outdoor_forward/forward_1/outdoor_forward_1_snapdragon_with_gt/groundtruth.txt"),
    ("outdoor_forward_3", f"{DATASETS}/uzhfpv_outdoor_forward/forward_3/outdoor_forward_3_snapdragon_with_gt/groundtruth.txt"),
    ("outdoor_forward_5", f"{DATASETS}/uzhfpv_outdoor_forward/forward_5/outdoor_forward_5_snapdragon_with_gt/groundtruth.txt"),
    ("outdoor_45_1", f"{DATASETS}/uzhfpv_outdoor_45/outdoor_45_1_snapdragon_with_gt/groundtruth.txt"),
]


def main():
    root = RESULTS
    rows = []
    for name, gt in SEQS:
        out = os.path.join(root, name)
        est = os.path.join(out, "estimate.txt")
        timing = os.path.join(out, "timing.txt")
        cpu = os.path.join(out, "cpu.txt")
        print("====", name, "====")
        rmse_ori, rmse_pos, rpe, raw = ov_eval_rmse(gt, est)
        with open(os.path.join(out, "ov_eval_error_singlerun.txt"), "w") as f:
            f.write(raw)
        ate = compute_ate(gt, est)
        # Prefer official ov_eval RMSE for the table
        ate["rmse_pos"] = rmse_pos
        ate["rmse_ori"] = rmse_ori
        plot_results(out, name, ate)
        timing_m = parse_timing(timing) if os.path.isfile(timing) else None
        cpu_m = parse_cpu(cpu) if os.path.isfile(cpu) else None
        # RPE at 40m if available else last
        rpe40_pos = rpe40_ori = ""
        for seg, ori, pos in rpe:
            if seg == "40":
                rpe40_ori, rpe40_pos = ori, pos
        if not rpe40_pos and rpe:
            rpe40_ori, rpe40_pos = rpe[-1][1], rpe[-1][2]
        row = {
            "sequence": name,
            "rmse_pos_m": rmse_pos,
            "rmse_ori_deg": rmse_ori,
            "rpe40_pos_m": float(rpe40_pos) if rpe40_pos else float("nan"),
            "rpe40_ori_deg": float(rpe40_ori) if rpe40_ori else float("nan"),
            "ms_per_frame": timing_m["ms_per_frame"] if timing_m else float("nan"),
            "fps": timing_m["fps"] if timing_m else float("nan"),
            "cpu_mean_pct": cpu_m["mean_cpu"] if cpu_m else float("nan"),
            "cpu_max_pct": cpu_m["max_cpu"] if cpu_m else float("nan"),
            "n_poses": ate["n"],
        }
        rows.append(row)
        with open(os.path.join(out, "metrics.txt"), "w") as f:
            for k, v in row.items():
                f.write("%s %s\n" % (k, v))
        print(
            "ov_eval ATE rmse_pos=%.3f m rmse_ori=%.3f deg | ms/frame=%.2f FPS=%.1f CPU=%.1f%%"
            % (rmse_pos, rmse_ori, row["ms_per_frame"], row["fps"], row["cpu_mean_pct"])
        )

    csv_path = os.path.join(root, "summary.csv")
    with open(csv_path, "w") as f:
        f.write(
            "sequence,rmse_pos_m,rmse_ori_deg,rpe40_pos_m,rpe40_ori_deg,ms_per_frame,fps,cpu_mean_pct,cpu_max_pct,n_poses\n"
        )
        for r in rows:
            f.write(
                "{sequence},{rmse_pos_m:.6f},{rmse_ori_deg:.6f},{rpe40_pos_m:.6f},{rpe40_ori_deg:.6f},{ms_per_frame:.6f},{fps:.6f},{cpu_mean_pct:.3f},{cpu_max_pct:.3f},{n_poses}\n".format(
                    **r
                )
            )

    md = os.path.join(root, "TABLE.md")
    lines = [
        "# Baseline Mono MSCKF Results",
        "",
        "Settings: `max_cameras:=1`, `use_stereo:=false` (cam0/left only), OpenVINS `serial.launch`.",
        "ATE from official `ov_eval error_singlerun posyaw` (includes orientation even on UZH).",
        "RPE40 = relative pose error median at 40 m segment.",
        "Runtime: mean total time in `timing.txt` → **ms/frame**; **FPS = 1 / mean_total** (offline serial processing throughput, not camera rate).",
        "CPU: mean/max `%` of `ros1_serial_msckf` via psutil during the run (100% ≈ one full core).",
        "",
        "| Sequence | Trans. RMSE (m) | Ori. RMSE (deg) | RPE40 Trans (m) | RPE40 Ori (deg) | ms/frame | FPS | CPU mean % | CPU max % |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in rows:
        lines.append(
            "| {sequence} | {rmse_pos_m:.3f} | {rmse_ori_deg:.3f} | {rpe40_pos_m:.3f} | {rpe40_ori_deg:.3f} | {ms_per_frame:.2f} | {fps:.1f} | {cpu_mean_pct:.1f} | {cpu_max_pct:.1f} |".format(
                **r
            )
        )
    lines += [
        "",
        "Per-sequence folder: `estimate.txt`, `timing.txt`, `cpu.txt`, `traj_xy.png`, `traj_z.png`, `error_translation.png`, `error_orientation.png`, `ov_eval_error_singlerun.txt`.",
        "",
    ]
    open(md, "w").write("\n".join(lines))
    print("Wrote", md)


if __name__ == "__main__":
    main()
