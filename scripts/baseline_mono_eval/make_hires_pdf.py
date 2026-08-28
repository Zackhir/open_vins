#!/usr/bin/env python3
"""Rebuild baseline PDF with large, directly-drawn (non-pixelated) plots."""

import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

ROOT = os.environ.get("RESULTS_DIR", "/results/baseline_mono_msckf")
DATASETS = os.environ.get("DATASETS_DIR", "/datasets")
OUT_PDF = os.path.join(ROOT, "baseline_mono_msckf_report.pdf")
CPU_CORES = int(os.environ.get("CPU_CORES", "24"))

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

# Large landscape pages
PAGE = (16, 10)  # inches
DPI = 150


def load_tum(path):
    times, xyz, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            times.append(float(p[0]))
            xyz.append([float(p[1]), float(p[2]), float(p[3])])
            quat.append([float(p[4]), float(p[5]), float(p[6]), float(p[7])])
    return np.asarray(times), np.asarray(xyz), np.asarray(quat)


def associate(est_t, gt_t, max_diff=0.02):
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
    q = q / np.linalg.norm(q)
    qv, qw = q[:3], q[3]
    return (2 * qw * qw - 1.0) * np.eye(3) - 2 * qw * skew(qv) + 2.0 * np.outer(qv, qv)


def jpl_quat_multiply(q, p):
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
    return math.atan2(C[0, 1] - C[1, 0], C[0, 0] + C[1, 1])


def align_posyaw(pos_est, pos_gt):
    mu_M = pos_gt.mean(axis=0)
    mu_D = pos_est.mean(axis=0)
    n = float(len(pos_est))
    C = ((pos_gt - mu_M).T @ (pos_est - mu_D)) / n
    yaw = get_best_yaw(n * C.T)
    R = rot_z(yaw)
    t = mu_M - R @ mu_D
    return R, t, yaw


def so3_log_norm(R):
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return abs(math.acos(tr))


def compute_series(gt_path, est_path):
    t_gt, p_gt, q_gt = load_tum(gt_path)
    t_est, p_est, q_est = load_tum(est_path)
    ie, ig = associate(t_est, t_gt)
    pe, pg = p_est[ie], p_gt[ig]
    qe, qg = q_est[ie], q_gt[ig]
    tt = t_est[ie]
    R, t, yaw = align_posyaw(pe, pg)
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])

    pe_a, err_pos, err_ori = [], [], []
    for i in range(len(pe)):
        xyz_a = R @ pe[i] + t
        q_a = jpl_quat_multiply(qe[i], jpl_inv(q_yaw))
        e_R = jpl_quat_to_R(q_a).T @ jpl_quat_to_R(qg[i])
        err_ori.append(180.0 / math.pi * so3_log_norm(e_R))
        err_pos.append(np.linalg.norm(pg[i] - xyz_a))
        pe_a.append(xyz_a)
    pe_a = np.asarray(pe_a)
    return {
        "t": tt - tt[0],
        "pe_xy": pe_a[:, :2],
        "pg_xy": pg[:, :2],
        "pe_z": pe_a[:, 2],
        "pg_z": pg[:, 2],
        "err_ori": np.asarray(err_ori),
        "err_pos": np.asarray(err_pos),
    }


def load_rows():
    rows = list(csv.DictReader(open(os.path.join(ROOT, "summary.csv"))))
    by = {r["sequence"]: r for r in rows}
    return [by[s] for s, _ in SEQS if s in by]


def page_table(pdf, rows):
    fig = plt.figure(figsize=PAGE, dpi=DPI)
    fig.suptitle("OpenVINS Baseline — Mono MSCKF Evaluation", fontsize=20, fontweight="bold", y=0.95)
    fig.text(
        0.06,
        0.86,
        "Mode: max_cameras=1, use_stereo=false (left/cam0 only)\n"
        "Table RMSE: ov_eval error_singlerun posyaw\n"
        "Plots: trajectory + error_dataset-style RMSE (ori/pos vs time)\n"
        "Runtime: timing.txt mean total → ms/frame; FPS = 1/mean_total (offline serial)\n"
        f"CPU: psutil mean/max % of ros1_serial_msckf ÷ {CPU_CORES} cores (%/core)",
        fontsize=11,
        va="top",
    )
    col_labels = [
        "Sequence",
        "Trans. RMSE (m)",
        "Ori. RMSE (deg)",
        "ms/frame",
        "FPS",
        "CPU mean %/core",
        "CPU max %/core",
    ]
    cell = [
        [
            r["sequence"],
            f"{float(r['rmse_pos_m']):.3f}",
            f"{float(r['rmse_ori_deg']):.3f}",
            f"{float(r['ms_per_frame']):.2f}",
            f"{float(r['fps']):.1f}",
            f"{float(r['cpu_mean_pct']) / CPU_CORES:.1f}",
            f"{float(r['cpu_max_pct']) / CPU_CORES:.1f}",
        ]
        for r in rows
    ]
    ax = fig.add_axes([0.05, 0.10, 0.90, 0.55])
    ax.axis("off")
    table = ax.table(cellText=cell, colLabels=col_labels, loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1.0, 2.0)
    for (row, col), cell_obj in table.get_celld().items():
        if row == 0:
            cell_obj.set_facecolor("#2c3e50")
            cell_obj.set_text_props(color="white", fontweight="bold")
        elif row % 2 == 0:
            cell_obj.set_facecolor("#f2f2f2")
    pdf.savefig(fig)
    plt.close(fig)


def page_sequence(pdf, name, gt_path, table_row):
    est_path = os.path.join(ROOT, name, "estimate.txt")
    s = compute_series(gt_path, est_path)

    # Page 1: trajectories (large)
    fig = plt.figure(figsize=PAGE, dpi=DPI)
    fig.suptitle(
        f"{name}  |  Trans RMSE={float(table_row['rmse_pos_m']):.3f} m   Ori RMSE={float(table_row['rmse_ori_deg']):.3f} deg",
        fontsize=16,
        fontweight="bold",
    )
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.plot(s["pg_xy"][:, 0], s["pg_xy"][:, 1], "k-", lw=2.0, label="GT")
    ax1.plot(s["pe_xy"][:, 0], s["pe_xy"][:, 1], "b-", lw=1.5, label="Estimate")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.set_xlabel("x (m)", fontsize=12)
    ax1.set_ylabel("y (m)", fontsize=12)
    ax1.set_title("Trajectory (XY)", fontsize=14)
    ax1.legend(fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.tick_params(labelsize=11)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(s["t"], s["pg_z"], "k-", lw=2.0, label="GT")
    ax2.plot(s["t"], s["pe_z"], "b-", lw=1.5, label="Estimate")
    ax2.set_xlabel("time (s)", fontsize=12)
    ax2.set_ylabel("z (m)", fontsize=12)
    ax2.set_title("Trajectory (Z)", fontsize=14)
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.tick_params(labelsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    pdf.savefig(fig)
    plt.close(fig)

    # Page 2: RMSE style (error_dataset layout), full page
    fig = plt.figure(figsize=PAGE, dpi=DPI)
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.set_title(f"Root Mean Squared Error — {name}", fontsize=16)
    ax1.set_ylabel("Error Orientation (deg)", fontsize=13)
    ax1.plot(s["t"], s["err_ori"], lw=1.8)
    ax1.set_xlim(0.0, float(s["t"][-1]) if len(s["t"]) else 1.0)
    ax1.grid(True, alpha=0.3)
    ax1.tick_params(labelsize=11)

    ax2 = fig.add_subplot(2, 1, 2)
    ax2.set_ylabel("Error Position (m)", fontsize=13)
    ax2.set_xlabel("dataset time (s)", fontsize=13)
    ax2.plot(s["t"], s["err_pos"], lw=1.8)
    ax2.set_xlim(0.0, float(s["t"][-1]) if len(s["t"]) else 1.0)
    ax2.grid(True, alpha=0.3)
    ax2.tick_params(labelsize=11)
    fig.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)

    # Also refresh standalone high-res RMSE png for the folder
    fig = plt.figure(figsize=(12, 7), dpi=200)
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.set_title(f"Root Mean Squared Error - mono_msckf ({name})")
    ax1.set_ylabel("Error Orientation (deg)")
    ax1.plot(s["t"], s["err_ori"], lw=1.5)
    ax1.set_xlim(0.0, float(s["t"][-1]) if len(s["t"]) else 1.0)
    ax1.grid(True, alpha=0.3)
    ax2 = fig.add_subplot(2, 1, 2)
    ax2.set_ylabel("Error Position (m)")
    ax2.set_xlabel("dataset time (s)")
    ax2.plot(s["t"], s["err_pos"], lw=1.5)
    ax2.set_xlim(0.0, float(s["t"][-1]) if len(s["t"]) else 1.0)
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    rmse_path = os.path.join(ROOT, name, "rmse.png")
    try:
        fig.savefig(rmse_path, dpi=200)
    except OSError:
        pass  # sequence dirs may be root-owned from Docker runs
    plt.close(fig)


def main():
    rows = load_rows()
    by = {r["sequence"]: r for r in rows}
    with PdfPages(OUT_PDF) as pdf:
        page_table(pdf, rows)
        for name, gt in SEQS:
            print("plotting", name)
            page_sequence(pdf, name, gt, by[name])
        d = pdf.infodict()
        d["Title"] = "OpenVINS Baseline Mono MSCKF Evaluation"
        d["Author"] = "Zackhir"
    print("Wrote", OUT_PDF)


if __name__ == "__main__":
    main()
