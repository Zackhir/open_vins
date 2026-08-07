/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * Finite-difference checks for pose-only residual Jacobians (Step 3).
 */

#include "update/PoseOnlyGeometry.h"
#include "utils/quat_ops.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace ov_msckf;
using namespace ov_core;

static int g_failed = 0;

#define EXPECT_TRUE(cond)                                                                                                                \
  do {                                                                                                                                   \
    if (!(cond)) {                                                                                                                       \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " : " << #cond << std::endl;                                                \
      g_failed++;                                                                                                                        \
    }                                                                                                                                    \
  } while (0)

static PoseOnlyGeometry::CameraPose make_pose(const Eigen::Matrix3d &R_GtoC, const Eigen::Vector3d &p_CinG) {
  PoseOnlyGeometry::CameraPose pose;
  pose.R_GtoC = R_GtoC;
  pose.p_CinG = p_CinG;
  return pose;
}

static Eigen::Vector3d project_bearing(const PoseOnlyGeometry::CameraPose &pose, const Eigen::Vector3d &p_FinG) {
  Eigen::Vector3d p_FinC = pose.R_GtoC * (p_FinG - pose.p_CinG);
  EXPECT_TRUE(p_FinC(2) > 1e-6);
  return p_FinC / p_FinC(2);
}

static PoseOnlyGeometry::CameraPose perturb_pose(const PoseOnlyGeometry::CameraPose &pose, const Eigen::Vector3d &dtheta,
                                                 const Eigen::Vector3d &dp) {
  PoseOnlyGeometry::CameraPose out = pose;
  out.R_GtoC = exp_so3(dtheta) * pose.R_GtoC;
  out.p_CinG = pose.p_CinG + dp;
  return out;
}

/// Finite-difference 2×6 Jacobian of residual w.r.t. one pose slot (j, k, or i).
enum class PoseSlot { J, K, I };

static Eigen::Matrix<double, 2, 6> fd_jacobian_slot(const Eigen::Vector3d &zj, const PoseOnlyGeometry::CameraPose &pose_j,
                                                    const Eigen::Vector3d &zk, const PoseOnlyGeometry::CameraPose &pose_k,
                                                    const PoseOnlyGeometry::CameraPose &pose_i, const Eigen::Vector3d &zi, PoseSlot slot,
                                                    double eps) {
  Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
  for (int c = 0; c < 6; c++) {
    Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    if (c < 3)
      dtheta(c) = eps;
    else
      dp(c - 3) = eps;

    PoseOnlyGeometry::CameraPose pj = pose_j, pk = pose_k, pi = pose_i;
    if (slot == PoseSlot::J)
      pj = perturb_pose(pose_j, dtheta, dp);
    else if (slot == PoseSlot::K)
      pk = perturb_pose(pose_k, dtheta, dp);
    else
      pi = perturb_pose(pose_i, dtheta, dp);
    Eigen::Vector2d r_plus = PoseOnlyGeometry::residual_normalized(zj, pj, zk, pk, pi, zi);

    pj = pose_j;
    pk = pose_k;
    pi = pose_i;
    if (slot == PoseSlot::J)
      pj = perturb_pose(pose_j, -dtheta, -dp);
    else if (slot == PoseSlot::K)
      pk = perturb_pose(pose_k, -dtheta, -dp);
    else
      pi = perturb_pose(pose_i, -dtheta, -dp);
    Eigen::Vector2d r_minus = PoseOnlyGeometry::residual_normalized(zj, pj, zk, pk, pi, zi);

    H.col(c) = (r_plus - r_minus) / (2.0 * eps);
  }
  return H;
}

static void check_jacobians(const std::string &name, const PoseOnlyGeometry::CameraPose &pose_j, const PoseOnlyGeometry::CameraPose &pose_k,
                            const PoseOnlyGeometry::CameraPose &pose_i, const Eigen::Vector3d &p_FinG, double tol) {
  std::cout << "[TEST] FD Jacobians: " << name << std::endl;
  Eigen::Vector3d zj = project_bearing(pose_j, p_FinG);
  Eigen::Vector3d zk = project_bearing(pose_k, p_FinG);
  Eigen::Vector3d zi = project_bearing(pose_i, p_FinG);

  auto J = PoseOnlyGeometry::residual_jacobian_poses(zj, pose_j, zk, pose_k, pose_i, zi);

  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 6> H_j_fd = fd_jacobian_slot(zj, pose_j, zk, pose_k, pose_i, zi, PoseSlot::J, eps);
  Eigen::Matrix<double, 2, 6> H_k_fd = fd_jacobian_slot(zj, pose_j, zk, pose_k, pose_i, zi, PoseSlot::K, eps);
  Eigen::Matrix<double, 2, 6> H_i_fd = fd_jacobian_slot(zj, pose_j, zk, pose_k, pose_i, zi, PoseSlot::I, eps);

  double ej = (J.H_j - H_j_fd).cwiseAbs().maxCoeff();
  double ek = (J.H_k - H_k_fd).cwiseAbs().maxCoeff();
  double ei = (J.H_i - H_i_fd).cwiseAbs().maxCoeff();
  std::cout << "  max|H_j-H_j_fd|=" << ej << "  max|H_k-H_k_fd|=" << ek << "  max|H_i-H_i_fd|=" << ei << std::endl;
  EXPECT_TRUE(ej < tol);
  EXPECT_TRUE(ek < tol);
  EXPECT_TRUE(ei < tol);
}

static void check_shared_pose_i_equals_j(const std::string &name, const PoseOnlyGeometry::CameraPose &pose_j,
                                         const PoseOnlyGeometry::CameraPose &pose_k, const Eigen::Vector3d &p_FinG, double tol) {
  // When residual camera is the left base, FD w.r.t. that physical pose must match H_j + H_i.
  std::cout << "[TEST] FD shared pose i==j: " << name << std::endl;
  const PoseOnlyGeometry::CameraPose &pose_i = pose_j;
  Eigen::Vector3d zj = project_bearing(pose_j, p_FinG);
  Eigen::Vector3d zk = project_bearing(pose_k, p_FinG);
  Eigen::Vector3d zi = zj;

  auto J = PoseOnlyGeometry::residual_jacobian_poses(zj, pose_j, zk, pose_k, pose_i, zi);
  Eigen::Matrix<double, 2, 6> H_combined = J.H_j + J.H_i;

  const double eps = 1e-6;
  // Perturb the shared pose once (apply to both j and i slots together)
  Eigen::Matrix<double, 2, 6> H_fd = Eigen::Matrix<double, 2, 6>::Zero();
  for (int c = 0; c < 6; c++) {
    Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    if (c < 3)
      dtheta(c) = eps;
    else
      dp(c - 3) = eps;

    PoseOnlyGeometry::CameraPose pj_p = perturb_pose(pose_j, dtheta, dp);
    PoseOnlyGeometry::CameraPose pj_m = perturb_pose(pose_j, -dtheta, -dp);
    Eigen::Vector2d r_plus = PoseOnlyGeometry::residual_normalized(zj, pj_p, zk, pose_k, pj_p, zi);
    Eigen::Vector2d r_minus = PoseOnlyGeometry::residual_normalized(zj, pj_m, zk, pose_k, pj_m, zi);
    H_fd.col(c) = (r_plus - r_minus) / (2.0 * eps);
  }

  double e = (H_combined - H_fd).cwiseAbs().maxCoeff();
  double ek = (J.H_k - fd_jacobian_slot(zj, pose_j, zk, pose_k, pose_i, zi, PoseSlot::K, eps)).cwiseAbs().maxCoeff();
  std::cout << "  max|(H_j+H_i)-H_fd|=" << e << "  max|H_k-H_k_fd|=" << ek << std::endl;
  EXPECT_TRUE(e < tol);
  EXPECT_TRUE(ek < tol);
}

static void check_bearing_jacobians(const char *name, const PoseOnlyGeometry::CameraPose &pose_j,
                                    const PoseOnlyGeometry::CameraPose &pose_k, const PoseOnlyGeometry::CameraPose &pose_i,
                                    const Eigen::Vector3d &p_FinG, double tol) {
  Eigen::Vector3d zj = project_bearing(pose_j, p_FinG);
  Eigen::Vector3d zk = project_bearing(pose_k, p_FinG);
  const double eps = 1e-7;

  Eigen::Matrix<double, 2, 2> DH_Dbj, DH_Dbk;
  PoseOnlyGeometry::prediction_jacobian_bearings(zj, pose_j, zk, pose_k, pose_i, DH_Dbj, DH_Dbk);

  auto predict = [&](const Eigen::Vector3d &bj, const Eigen::Vector3d &bk) {
    Eigen::Vector3d p_po = PoseOnlyGeometry::feature_in_camera_po(bj, pose_j, bk, pose_k, pose_i);
    return PoseOnlyGeometry::project_normalized(p_po);
  };

  Eigen::Matrix<double, 2, 2> Hbj_fd = Eigen::Matrix<double, 2, 2>::Zero();
  Eigen::Matrix<double, 2, 2> Hbk_fd = Eigen::Matrix<double, 2, 2>::Zero();
  for (int c = 0; c < 2; c++) {
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    d(c) = eps;
    Hbj_fd.col(c) = (predict(zj + d, zk) - predict(zj - d, zk)) / (2.0 * eps);
    Hbk_fd.col(c) = (predict(zj, zk + d) - predict(zj, zk - d)) / (2.0 * eps);
  }

  double ej = (DH_Dbj - Hbj_fd).cwiseAbs().maxCoeff();
  double ek = (DH_Dbk - Hbk_fd).cwiseAbs().maxCoeff();
  std::cout << "bearing_jac [" << name << "] max|Dbj-fd|=" << ej << " max|Dbk-fd|=" << ek << std::endl;
  EXPECT_TRUE(ej < tol);
  EXPECT_TRUE(ek < tol);
}

int main() {
  const double tol = 1e-5; // slightly loose vs 1e-6 for central FD / conditioning

  // Case 1: three distinct translating cameras (identity rotations)
  {
    auto pose_j = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
    auto pose_k = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.8, 0.0, 0.0));
    auto pose_i = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.3, 0.2, 0.0));
    check_jacobians("distinct_identity_rots", pose_j, pose_k, pose_i, Eigen::Vector3d(0.1, -0.15, 4.0), tol);
  }

  // Case 2: rotated cameras (ov_core::rot_x / rot_y)
  {
    auto pose_j = make_pose(rot_y(0.05), Eigen::Vector3d(0.0, 0.0, 0.0));
    auto pose_k = make_pose(rot_y(-0.04) * rot_x(0.03), Eigen::Vector3d(1.0, 0.05, -0.02));
    auto pose_i = make_pose(rot_x(-0.02), Eigen::Vector3d(0.4, -0.1, 0.03));
    check_jacobians("distinct_rotated", pose_j, pose_k, pose_i, Eigen::Vector3d(-0.2, 0.1, 5.0), tol);
  }

  // Case 3: residual on left base (i == j), combined Jacobian
  {
    auto pose_j = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
    auto pose_k = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.9, 0.1, 0.0));
    check_shared_pose_i_equals_j("i_equals_j", pose_j, pose_k, Eigen::Vector3d(0.25, 0.05, 3.5), tol);
  }

  // Case 4: residual on right base (i == k) — independent FD on slots still valid
  {
    auto pose_j = make_pose(rot_y(0.02), Eigen::Vector3d(0.0, 0.0, 0.0));
    auto pose_k = make_pose(rot_y(-0.01), Eigen::Vector3d(0.7, 0.0, 0.0));
    check_jacobians("i_equals_k_symbolic_slots", pose_j, pose_k, pose_k, Eigen::Vector3d(0.0, 0.2, 4.5), tol);
  }

  // Case 5: ∂π(p_PO)/∂bearing for noise whitening
  {
    auto pose_j = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
    auto pose_k = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.8, 0.0, 0.0));
    auto pose_i = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.3, 0.2, 0.0));
    check_bearing_jacobians("noise_G_bearings", pose_j, pose_k, pose_i, Eigen::Vector3d(0.1, -0.15, 4.0), tol);
  }

  if (g_failed == 0) {
    std::cout << "ALL PoseOnlyJacobian TESTS PASSED" << std::endl;
    return 0;
  }
  std::cerr << g_failed << " assertion(s) failed" << std::endl;
  return 1;
}
