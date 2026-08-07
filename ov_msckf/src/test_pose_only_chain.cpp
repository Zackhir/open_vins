/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * Finite-difference checks for IMU/calib JPL chain of pose-only camera Jacobians (Step 5).
 */

#include "update/PoseOnlyGeometry.h"
#include "utils/quat_ops.h"

#include <cmath>
#include <iostream>

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

/// Left-Exp camera error coords of pose relative to nominal: [log(R R0^T); p - p0]
static Eigen::Matrix<double, 6, 1> cam_error(const PoseOnlyGeometry::CameraPose &pose, const PoseOnlyGeometry::CameraPose &pose0) {
  Eigen::Matrix<double, 6, 1> e;
  e.segment<3>(0) = log_so3(pose.R_GtoC * pose0.R_GtoC.transpose());
  e.segment<3>(3) = pose.p_CinG - pose0.p_CinG;
  return e;
}

static void check_imu_chain(const std::string &name, const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                            const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC, double tol) {
  std::cout << "[TEST] FD IMU chain: " << name << std::endl;
  Eigen::Matrix<double, 2, 6> H_cam;
  H_cam << 1.2, -0.3, 0.4, 0.1, -0.7, 0.5, -0.2, 0.9, 0.1, 0.6, 0.3, -0.4;

  auto pose0 = PoseOnlyGeometry::compose_camera_pose(R_GtoI, p_IinG, R_ItoC, p_IinC);
  auto H_imu = PoseOnlyGeometry::chain_camera_H_to_imu_jpl(H_cam, R_GtoI, R_ItoC, p_IinC);

  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 6> H_fd = Eigen::Matrix<double, 2, 6>::Zero();
  for (int c = 0; c < 6; c++) {
    Eigen::Vector3d dth = Eigen::Vector3d::Zero();
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    if (c < 3)
      dth(c) = eps;
    else
      dp(c - 3) = eps;

    // JPL: R' = Exp(-δθ) R
    Eigen::Matrix3d R_p = exp_so3(-dth) * R_GtoI;
    Eigen::Matrix3d R_m = exp_so3(dth) * R_GtoI;
    auto pose_p = PoseOnlyGeometry::compose_camera_pose(R_p, p_IinG + dp, R_ItoC, p_IinC);
    auto pose_m = PoseOnlyGeometry::compose_camera_pose(R_m, p_IinG - dp, R_ItoC, p_IinC);
    Eigen::Vector2d r_p = H_cam * cam_error(pose_p, pose0);
    Eigen::Vector2d r_m = H_cam * cam_error(pose_m, pose0);
    H_fd.col(c) = (r_p - r_m) / (2.0 * eps);
  }

  double err = (H_imu - H_fd).cwiseAbs().maxCoeff();
  std::cout << "  max|H_imu - H_fd|=" << err << std::endl;
  EXPECT_TRUE(err < tol);
}

static void check_calib_chain(const std::string &name, const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                              const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC, double tol) {
  std::cout << "[TEST] FD calib chain: " << name << std::endl;
  Eigen::Matrix<double, 2, 6> H_cam;
  H_cam << 0.5, 0.2, -0.8, 1.1, 0.0, -0.3, 0.7, -0.4, 0.6, -0.2, 0.9, 0.1;

  auto pose0 = PoseOnlyGeometry::compose_camera_pose(R_GtoI, p_IinG, R_ItoC, p_IinC);
  auto H_calib = PoseOnlyGeometry::chain_camera_H_to_calib_jpl(H_cam, R_GtoI, R_ItoC, p_IinC);

  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 6> H_fd = Eigen::Matrix<double, 2, 6>::Zero();
  for (int c = 0; c < 6; c++) {
    Eigen::Vector3d dth = Eigen::Vector3d::Zero();
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    if (c < 3)
      dth(c) = eps;
    else
      dp(c - 3) = eps;

    Eigen::Matrix3d R_p = exp_so3(-dth) * R_ItoC;
    Eigen::Matrix3d R_m = exp_so3(dth) * R_ItoC;
    auto pose_p = PoseOnlyGeometry::compose_camera_pose(R_GtoI, p_IinG, R_p, p_IinC + dp);
    auto pose_m = PoseOnlyGeometry::compose_camera_pose(R_GtoI, p_IinG, R_m, p_IinC - dp);
    Eigen::Vector2d r_p = H_cam * cam_error(pose_p, pose0);
    Eigen::Vector2d r_m = H_cam * cam_error(pose_m, pose0);
    H_fd.col(c) = (r_p - r_m) / (2.0 * eps);
  }

  double err = (H_calib - H_fd).cwiseAbs().maxCoeff();
  std::cout << "  max|H_calib - H_fd|=" << err << std::endl;
  EXPECT_TRUE(err < tol);
}

int main() {
  Eigen::Matrix3d R_GtoI = exp_so3(Eigen::Vector3d(0.1, -0.2, 0.05));
  Eigen::Vector3d p_IinG(0.3, -0.1, 0.8);
  Eigen::Matrix3d R_ItoC = exp_so3(Eigen::Vector3d(-0.02, 0.03, 0.01));
  Eigen::Vector3d p_IinC(0.02, -0.04, 0.01);

  const double tol = 1e-5;
  check_imu_chain("nominal", R_GtoI, p_IinG, R_ItoC, p_IinC, tol);
  check_calib_chain("nominal", R_GtoI, p_IinG, R_ItoC, p_IinC, tol);

  // Identity extrinsics
  check_imu_chain("identity_calib", R_GtoI, p_IinG, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), tol);
  check_calib_chain("identity_calib", R_GtoI, p_IinG, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), tol);

  // Larger lever arm
  check_imu_chain("large_lever", R_GtoI, p_IinG, R_ItoC, Eigen::Vector3d(0.2, 0.1, -0.05), tol);
  check_calib_chain("large_lever", R_GtoI, p_IinG, R_ItoC, Eigen::Vector3d(0.2, 0.1, -0.05), tol);

  if (g_failed == 0) {
    std::cout << "ALL PASSED" << std::endl;
    return 0;
  }
  std::cerr << g_failed << " CHECK(S) FAILED" << std::endl;
  return 1;
}
