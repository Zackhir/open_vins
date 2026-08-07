/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "PoseOnlyGeometry.h"

#include "utils/quat_ops.h"

#include <cassert>
#include <cmath>

using namespace ov_msckf;
using namespace ov_core;

namespace {

Eigen::Vector3d normalize_bearing(const Eigen::Vector3d &b) {
  Eigen::Vector3d p = b;
  if (std::abs(p(2)) > 1e-12)
    p /= p(2);
  return p;
}

/// ∂||u||/∂u = u^T / ||u||  (row), returned as column for convenience: u/||u||
Eigen::Vector3d dnorm_du(const Eigen::Vector3d &u) {
  double n = u.norm();
  assert(n > 1e-14);
  return u / n;
}

} // namespace

void PoseOnlyGeometry::relative_pose(const CameraPose &pose_i, const CameraPose &pose_j, Eigen::Matrix3d &R_j_i, Eigen::Vector3d &t_j_i) {
  // R_cj_ci = R_GtoCj * R_GtoCi^T
  // t_cj_ci = R_GtoCj * (p_Ci_in_G - p_Cj_in_G)
  R_j_i = pose_j.R_GtoC * pose_i.R_GtoC.transpose();
  t_j_i = pose_j.R_GtoC * (pose_i.p_CinG - pose_j.p_CinG);
}

double PoseOnlyGeometry::theta(const Eigen::Vector3d &bearing_i, const CameraPose &pose_i, const Eigen::Vector3d &bearing_j,
                               const CameraPose &pose_j) {
  Eigen::Matrix3d R_j_i;
  Eigen::Vector3d t_j_i;
  relative_pose(pose_i, pose_j, R_j_i, t_j_i);
  Eigen::Vector3d p_i = normalize_bearing(bearing_i);
  Eigen::Vector3d p_j = normalize_bearing(bearing_j);
  // θ_(i,j) = ||[p_j ×] R_j_i p_i||
  return (skew_x(p_j) * R_j_i * p_i).norm();
}

bool PoseOnlyGeometry::select_base_views(const std::vector<Eigen::Vector3d> &bearings, const std::vector<CameraPose> &poses, int &base_j,
                                         int &base_k, double *best_theta) {
  assert(bearings.size() == poses.size());
  const int n = (int)bearings.size();
  if (n < 2)
    return false;

  double best = -1.0;
  int best_j = 0;
  int best_k = 1;
  for (int j = 0; j < n; j++) {
    for (int k = j + 1; k < n; k++) {
      double th = theta(bearings[j], poses[j], bearings[k], poses[k]);
      if (th > best) {
        best = th;
        best_j = j;
        best_k = k;
      }
    }
  }
  if (best < 0)
    return false;
  base_j = best_j;
  base_k = best_k;
  if (best_theta != nullptr)
    *best_theta = best;
  return true;
}

Eigen::Vector3d PoseOnlyGeometry::feature_in_camera_po(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j,
                                                       const Eigen::Vector3d &bearing_k, const CameraPose &pose_k, const CameraPose &pose_i) {
  Eigen::Vector3d p_j = normalize_bearing(bearing_j);
  Eigen::Vector3d p_k = normalize_bearing(bearing_k);

  Eigen::Matrix3d R_i_j, R_k_j;
  Eigen::Vector3d t_i_j, t_k_j;
  relative_pose(pose_j, pose_i, R_i_j, t_i_j); // p_i = R_i_j p_j + t_i_j
  relative_pose(pose_j, pose_k, R_k_j, t_k_j); // p_k = R_k_j p_j + t_k_j

  // Paper eq. (11):
  // p_i(PO) = ||[t_k_j ×] p_k|| R_i_j p_j + ||[p_k ×] R_k_j p_j|| t_i_j
  double a = (skew_x(t_k_j) * p_k).norm();
  double b = (skew_x(p_k) * R_k_j * p_j).norm(); // = θ_(j,k)
  return a * (R_i_j * p_j) + b * t_i_j;
}

Eigen::Vector2d PoseOnlyGeometry::project_normalized(const Eigen::Vector3d &p_Cam) {
  assert(std::abs(p_Cam(2)) > 1e-12);
  return p_Cam.head<2>() / p_Cam(2);
}

Eigen::Vector2d PoseOnlyGeometry::residual_normalized(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j,
                                                      const Eigen::Vector3d &bearing_k, const CameraPose &pose_k, const CameraPose &pose_i,
                                                      const Eigen::Vector3d &z_i) {
  Eigen::Vector3d p_po = feature_in_camera_po(bearing_j, pose_j, bearing_k, pose_k, pose_i);
  Eigen::Vector2d z_hat = project_normalized(p_po);
  Eigen::Vector3d z = normalize_bearing(z_i);
  return z_hat - z.head<2>();
}

Eigen::Matrix<double, 2, 3> PoseOnlyGeometry::project_jacobian(const Eigen::Vector3d &p_Cam) {
  // π(p) = [X/Z, Y/Z], ∂π/∂p from paper eq. (28)
  const double Z = p_Cam(2);
  assert(std::abs(Z) > 1e-12);
  const double Zinv = 1.0 / Z;
  const double Zinv2 = Zinv * Zinv;
  Eigen::Matrix<double, 2, 3> J;
  J << Zinv, 0.0, -p_Cam(0) * Zinv2, 0.0, Zinv, -p_Cam(1) * Zinv2;
  return J;
}

void PoseOnlyGeometry::feature_jacobian_poses(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j, const Eigen::Vector3d &bearing_k,
                                              const CameraPose &pose_k, const CameraPose &pose_i, Eigen::Matrix<double, 3, 6> &Dp_Dj,
                                              Eigen::Matrix<double, 3, 6> &Dp_Dk, Eigen::Matrix<double, 3, 6> &Dp_Di) {
  // Treat j, k, i as independent pose variables (even if numerically equal).
  // Error model: R ← Exp(δθ) R  (left),  p ← p + δp
  const Eigen::Vector3d p_j = normalize_bearing(bearing_j);
  const Eigen::Vector3d p_k = normalize_bearing(bearing_k);

  Eigen::Matrix3d R_i_j, R_k_j;
  Eigen::Vector3d t_i_j, t_k_j;
  relative_pose(pose_j, pose_i, R_i_j, t_i_j);
  relative_pose(pose_j, pose_k, R_k_j, t_k_j);

  const Eigen::Vector3d v = R_i_j * p_j;             // R_i_j p_j
  const Eigen::Vector3d w = R_k_j * p_j;             // R_k_j p_j
  const Eigen::Vector3d u_a = t_k_j.cross(p_k);      // [t_k_j ×] p_k
  const Eigen::Vector3d u_b = p_k.cross(w);          // [p_k ×] R_k_j p_j
  const double a = u_a.norm();
  const double b = u_b.norm();
  assert(a > 1e-14 && b > 1e-14);

  const Eigen::Vector3d da_dua = dnorm_du(u_a); // ∂a/∂u_a
  const Eigen::Vector3d db_dub = dnorm_du(u_b); // ∂b/∂u_b

  // ∂a/∂t_k_j : u_a = t×p_k ⇒ ∂u_a/∂t = -[p_k×] ⇒ ∂a/∂t = -da_dua^T [p_k×] = ([p_k×] da_dua)^T
  // As a 1×3 row: da_dt = da_dua^T * (-skew(p_k))
  const Eigen::Matrix<double, 1, 3> da_dtkj = da_dua.transpose() * (-skew_x(p_k));

  // ∂u_b/∂w = [p_k×],  ∂b/∂w = db_dub^T [p_k×]
  const Eigen::Matrix<double, 1, 3> db_dw = db_dub.transpose() * skew_x(p_k);

  // ---- helpers: derivatives of intermediates w.r.t. each pose's δθ, δp ----
  // R_i_j = R_i R_j^T
  //   ∂(R_i_j m)/∂δθ_i = -[ (R_i_j m) × ]
  //   ∂(R_i_j m)/∂δθ_j = R_i_j [m ×]
  // t_i_j = R_i (p_j_cam - p_i)
  //   ∂t_i_j/∂δθ_i = -[t_i_j ×]
  //   ∂t_i_j/∂δp_i = -R_i
  //   ∂t_i_j/∂δp_j =  R_i
  // R_k_j = R_k R_j^T  (same structure)
  // t_k_j = R_k (p_j_cam - p_k)

  const Eigen::Matrix3d &R_i = pose_i.R_GtoC;
  const Eigen::Matrix3d &R_k = pose_k.R_GtoC;

  Dp_Dj.setZero();
  Dp_Dk.setZero();
  Dp_Di.setZero();

  // p_po = a * v + b * t_i_j
  // ∂p_po = da * v + a * dv + db * t_i_j + b * dt_i_j

  // ========== pose i ==========
  {
    // v = R_i_j p_j depends on δθ_i; t_i_j depends on δθ_i, δp_i
    // a, b, R_k_j, t_k_j independent of pose i
    Eigen::Matrix3d dv_dthi = -skew_x(v);          // ∂v/∂δθ_i
    Eigen::Matrix3d dti_j_dthi = -skew_x(t_i_j);   // ∂t_i_j/∂δθ_i
    Eigen::Matrix3d dti_j_dpi = -R_i;              // ∂t_i_j/∂δp_i

    Dp_Di.block<3, 3>(0, 0) = a * dv_dthi + b * dti_j_dthi;
    Dp_Di.block<3, 3>(0, 3) = b * dti_j_dpi;
  }

  // ========== pose k ==========
  {
    // a depends on t_k_j; b depends on w = R_k_j p_j; v, t_i_j independent of k
    // ∂t_k_j/∂δθ_k = -[t_k_j×], ∂t_k_j/∂δp_k = -R_k
    // ∂w/∂δθ_k = -[w×]
    Eigen::Matrix3d dtkj_dthk = -skew_x(t_k_j);
    Eigen::Matrix3d dtkj_dpk = -R_k;
    Eigen::Matrix3d dw_dthk = -skew_x(w);

    // da/∂δθ_k = da_dtkj * dtkj_dthk   (1×3)
    Eigen::Matrix<double, 1, 3> da_dthk = da_dtkj * dtkj_dthk;
    Eigen::Matrix<double, 1, 3> da_dpk = da_dtkj * dtkj_dpk;
    Eigen::Matrix<double, 1, 3> db_dthk = db_dw * dw_dthk;
    // b does not depend on p_k (camera center); bearings fixed

    Dp_Dk.block<3, 3>(0, 0) = v * da_dthk + t_i_j * db_dthk;
    Dp_Dk.block<3, 3>(0, 3) = v * da_dpk;
  }

  // ========== pose j ==========
  {
    // v depends on δθ_j; t_i_j depends on δp_j; a depends on t_k_j (δp_j); b depends on w (δθ_j)
    // ∂v/∂δθ_j = R_i_j [p_j ×]
    // ∂t_i_j/∂δp_j = R_i
    // ∂t_k_j/∂δp_j = R_k
    // ∂w/∂δθ_j = R_k_j [p_j ×]
    Eigen::Matrix3d dv_dthj = R_i_j * skew_x(p_j);
    Eigen::Matrix3d dti_j_dpj = R_i;
    Eigen::Matrix3d dtkj_dpj = R_k;
    Eigen::Matrix3d dw_dthj = R_k_j * skew_x(p_j);

    Eigen::Matrix<double, 1, 3> da_dpj = da_dtkj * dtkj_dpj;
    Eigen::Matrix<double, 1, 3> db_dthj = db_dw * dw_dthj;

    // ∂p_po/∂δθ_j = da/∂δθ_j * v + a * dv/∂δθ_j + db/∂δθ_j * t_i_j + b * dt_i_j/∂δθ_j
    // t_i_j independent of δθ_j; a independent of δθ_j (t_k_j independent of δθ_j)
    Dp_Dj.block<3, 3>(0, 0) = a * dv_dthj + t_i_j * db_dthj;
    // ∂p_po/∂δp_j = da/∂δp_j * v + b * dt_i_j/∂δp_j   (b, v independent of δp_j)
    Dp_Dj.block<3, 3>(0, 3) = v * da_dpj + b * dti_j_dpj;
  }
}

PoseOnlyGeometry::ResidualPoseJacobians PoseOnlyGeometry::residual_jacobian_poses(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j,
                                                                                  const Eigen::Vector3d &bearing_k, const CameraPose &pose_k,
                                                                                  const CameraPose &pose_i, const Eigen::Vector3d &z_i) {
  (void)z_i; // residual = π(p_po) - z; z independent of poses
  Eigen::Vector3d p_po = feature_in_camera_po(bearing_j, pose_j, bearing_k, pose_k, pose_i);
  Eigen::Matrix<double, 2, 3> dpi_dp = project_jacobian(p_po);

  Eigen::Matrix<double, 3, 6> Dp_Dj, Dp_Dk, Dp_Di;
  feature_jacobian_poses(bearing_j, pose_j, bearing_k, pose_k, pose_i, Dp_Dj, Dp_Dk, Dp_Di);

  ResidualPoseJacobians J;
  J.H_j = dpi_dp * Dp_Dj;
  J.H_k = dpi_dp * Dp_Dk;
  J.H_i = dpi_dp * Dp_Di;
  return J;
}

PoseOnlyGeometry::CameraPose PoseOnlyGeometry::compose_camera_pose(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                                                                   const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC) {
  CameraPose pose;
  pose.R_GtoC = R_ItoC * R_GtoI;
  pose.p_CinG = p_IinG - pose.R_GtoC.transpose() * p_IinC;
  return pose;
}

Eigen::Matrix<double, 2, 6> PoseOnlyGeometry::chain_camera_H_to_imu_jpl(const Eigen::Matrix<double, 2, 6> &H_cam, const Eigen::Matrix3d &R_GtoI,
                                                                       const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC) {
  // Camera: R'≈Exp(δθ)R; JPL IMU: R'≈Exp(-δθ)R ⇒ with R_GtoC=R_ItoC R_GtoI, δθ_cam ≈ -R_ItoC δθ_I
  const Eigen::Vector3d lever_I = R_ItoC.transpose() * p_IinC;
  Eigen::Matrix<double, 2, 3> H_th = H_cam.block<2, 3>(0, 0);
  Eigen::Matrix<double, 2, 3> H_p = H_cam.block<2, 3>(0, 3);

  Eigen::Matrix<double, 2, 6> H_imu = Eigen::Matrix<double, 2, 6>::Zero();
  H_imu.block<2, 3>(0, 0) = H_th * (-R_ItoC) + H_p * (R_GtoI.transpose() * skew_x(lever_I));
  H_imu.block<2, 3>(0, 3) = H_p;
  return H_imu;
}

Eigen::Matrix<double, 2, 6> PoseOnlyGeometry::chain_camera_H_to_calib_jpl(const Eigen::Matrix<double, 2, 6> &H_cam,
                                                                         const Eigen::Matrix3d &R_GtoI, const Eigen::Matrix3d &R_ItoC,
                                                                         const Eigen::Vector3d &p_IinC) {
  // R_GtoC = R_ItoC R_GtoI; JPL calib: R_ItoC'≈Exp(-δθ_c)R_ItoC ⇒ δθ_cam ≈ -δθ_c
  // p_CinG = p_IinG - R_GtoC^T p_IinC
  const Eigen::Matrix3d R_GtoC = R_ItoC * R_GtoI;
  Eigen::Matrix<double, 2, 3> H_th = H_cam.block<2, 3>(0, 0);
  Eigen::Matrix<double, 2, 3> H_p = H_cam.block<2, 3>(0, 3);

  Eigen::Matrix<double, 2, 6> H_calib = Eigen::Matrix<double, 2, 6>::Zero();
  H_calib.block<2, 3>(0, 0) = -H_th + H_p * (R_GtoC.transpose() * skew_x(p_IinC));
  H_calib.block<2, 3>(0, 3) = H_p * (-R_GtoC.transpose());
  return H_calib;
}
