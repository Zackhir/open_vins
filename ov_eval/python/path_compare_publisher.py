#!/usr/bin/env python3
"""Publish animated nav_msgs/Path topics for MSCKF vs hybrid vs GT comparison in RViz."""

import math
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header


def load_tum(path):
    times, xyz, quat = [], [], []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            times.append(float(parts[0]))
            xyz.append([float(parts[1]), float(parts[2]), float(parts[3])])
            quat.append([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])])
    if not times:
        raise RuntimeError("no poses loaded from %s" % path)
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


def rot_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def get_best_yaw(c):
    return math.atan2(c[0, 1] - c[1, 0], c[0, 0] + c[1, 1])


def align_posyaw_umeyama(pos_est, pos_gt):
    mu_m = pos_gt.mean(axis=0)
    mu_d = pos_est.mean(axis=0)
    model_zc = pos_gt - mu_m
    data_zc = pos_est - mu_d
    n = float(len(pos_est))
    c = (model_zc.T @ data_zc) / n
    rot_c = n * c.T
    theta = get_best_yaw(rot_c)
    r = rot_z(theta)
    t = mu_m - r @ mu_d
    return r, t


def align_estimate_to_gt(t_est, p_est, t_gt, p_gt):
    ie, ig = associate(t_est, t_gt)
    if len(ie) < 3:
        raise RuntimeError("not enough timestamp associations for posyaw alignment")
    r, t = align_posyaw_umeyama(p_est[ie], p_gt[ig])
    p_aligned = (r @ p_est.T).T + t
    return p_aligned, t_est


def make_path(frame_id, stamp, times, positions):
    path = Path()
    path.header = Header(stamp=stamp, frame_id=frame_id)
    for t_i, pos in zip(times, positions):
        pose = PoseStamped()
        pose.header = Header(stamp=rospy.Time.from_sec(t_i), frame_id=frame_id)
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.position.z = float(pos[2])
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def downsample_traj(times, positions, dt=0.05):
    if len(times) < 2:
        return times, positions
    keep = [0]
    last = times[0]
    for i in range(1, len(times)):
        if times[i] - last >= dt:
            keep.append(i)
            last = times[i]
    if keep[-1] != len(times) - 1:
        keep.append(len(times) - 1)
    idx = np.asarray(keep)
    return times[idx], positions[idx]


def partial_path(frame_id, stamp, times, positions, t_cut):
    mask = times <= t_cut + 1e-9
    if not np.any(mask):
        return make_path(frame_id, stamp, [], [])
    return make_path(frame_id, stamp, times[mask], positions[mask])


class PathComparePublisher:
    def __init__(self):
        rospy.init_node("path_compare_publisher", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.speed = float(rospy.get_param("~speed", 1.0))
        self.loop = bool(rospy.get_param("~loop", False))
        self.hold_sec = float(rospy.get_param("~hold_sec", 2.0))

        msckf_path = rospy.get_param("~msckf_path")
        hybrid_path = rospy.get_param("~hybrid_path")
        gt_path = rospy.get_param("~gt_path")

        t_gt, p_gt, _ = load_tum(gt_path)
        t_msckf, p_msckf, _ = load_tum(msckf_path)
        t_hybrid, p_hybrid, _ = load_tum(hybrid_path)

        p_msckf, t_msckf = align_estimate_to_gt(t_msckf, p_msckf, t_gt, p_gt)
        p_hybrid, t_hybrid = align_estimate_to_gt(t_hybrid, p_hybrid, t_gt, p_gt)
        t_msckf, p_msckf = downsample_traj(t_msckf, p_msckf)
        t_hybrid, p_hybrid = downsample_traj(t_hybrid, p_hybrid)
        t_gt, p_gt = downsample_traj(t_gt, p_gt)

        t_start = max(t_gt[0], t_msckf[0], t_hybrid[0])
        t_end = min(t_gt[-1], t_msckf[-1], t_hybrid[-1])
        if t_end <= t_start:
            raise RuntimeError("no overlapping time window between GT and estimates")

        anim_mask = (t_gt >= t_start) & (t_gt <= t_end)
        t_anim = t_gt[anim_mask]
        p_anim = p_gt[anim_mask]
        # Animate at ~30 Hz of trajectory time so RViz/video stay real-time at high speed.
        step = max(int(round(0.033 / max(np.median(np.diff(t_anim)), 1e-3))), 1)
        self.t_gt = t_anim[::step]
        self.p_gt = p_anim[::step]
        if self.t_gt[-1] < t_anim[-1]:
            self.t_gt = np.append(self.t_gt, t_anim[-1])
            self.p_gt = np.vstack([self.p_gt, p_anim[-1]])
        self.t_msckf = t_msckf
        self.p_msckf = p_msckf
        self.t_hybrid = t_hybrid
        self.p_hybrid = p_hybrid

        self.pub_msckf = rospy.Publisher("/compare/path_msckf", Path, queue_size=1, latch=True)
        self.pub_hybrid = rospy.Publisher("/compare/path_hybrid", Path, queue_size=1, latch=True)
        self.pub_gt = rospy.Publisher("/compare/path_gt", Path, queue_size=1, latch=True)

        duration = self.t_gt[-1] - self.t_gt[0]
        rospy.loginfo(
            "loaded GT=%d msckf=%d hybrid=%d poses; anim window=%.1fs (%.3f-%.3f) speed=%.2fx",
            len(self.t_gt),
            len(t_msckf),
            len(t_hybrid),
            duration,
            t_start,
            t_end,
            self.speed,
        )

    def publish_frame(self, t_cut):
        stamp = rospy.Time.now()
        self.pub_msckf.publish(partial_path(self.frame_id, stamp, self.t_msckf, self.p_msckf, t_cut))
        self.pub_hybrid.publish(partial_path(self.frame_id, stamp, self.t_hybrid, self.p_hybrid, t_cut))
        self.pub_gt.publish(partial_path(self.frame_id, stamp, self.t_gt, self.p_gt, t_cut))

    def run(self):
        while not rospy.is_shutdown():
            for k in range(len(self.t_gt)):
                if rospy.is_shutdown():
                    return
                self.publish_frame(self.t_gt[k])
                if k + 1 < len(self.t_gt):
                    dt = (self.t_gt[k + 1] - self.t_gt[k]) / max(self.speed, 1e-6)
                    rospy.sleep(max(dt, 0.0))
            rospy.sleep(self.hold_sec)
            if not self.loop:
                rospy.loginfo("replay finished")
                return


def main():
    try:
        PathComparePublisher().run()
    except Exception as exc:
        rospy.logerr("path_compare_publisher failed: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
