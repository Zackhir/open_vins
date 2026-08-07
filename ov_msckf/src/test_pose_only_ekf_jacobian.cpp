/*
 * End-to-end FD check: PO prediction h and OpenVINS-style H=∂h/∂x (IMU JPL), r=z-h.
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

struct ImuPose {
  Eigen::Matrix3d R_GtoI = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
};

static PoseOnlyGeometry::CameraPose cam_from_imu(const ImuPose &imu, const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC) {
  return PoseOnlyGeometry::compose_camera_pose(imu.R_GtoI, imu.p_IinG, R_ItoC, p_IinC);
}

static Eigen::Vector3d bearing_of(const PoseOnlyGeometry::CameraPose &pose, const Eigen::Vector3d &p_FinG) {
  Eigen::Vector3d p_FinC = pose.R_GtoC * (p_FinG - pose.p_CinG);
  return p_FinC / p_FinC(2);
}

static Eigen::Vector2d predict_h(const Eigen::Vector3d &bj, const PoseOnlyGeometry::CameraPose &pj, const Eigen::Vector3d &bk,
                                 const PoseOnlyGeometry::CameraPose &pk, const PoseOnlyGeometry::CameraPose &pi) {
  return PoseOnlyGeometry::project_normalized(PoseOnlyGeometry::feature_in_camera_po(bj, pj, bk, pk, pi));
}

static void check_imu_slot(const std::string &name, int which, // 0=j,1=k,2=i
                           const ImuPose &imu_j, const ImuPose &imu_k, const ImuPose &imu_i, const Eigen::Matrix3d &R_ItoC,
                           const Eigen::Vector3d &p_IinC, const Eigen::Vector3d &p_FinG, double tol) {
  std::cout << "[TEST] EKF-style FD IMU chain: " << name << std::endl;
  auto pj = cam_from_imu(imu_j, R_ItoC, p_IinC);
  auto pk = cam_from_imu(imu_k, R_ItoC, p_IinC);
  auto pi = cam_from_imu(imu_i, R_ItoC, p_IinC);
  Eigen::Vector3d bj = bearing_of(pj, p_FinG);
  Eigen::Vector3d bk = bearing_of(pk, p_FinG);
  Eigen::Vector3d bi = bearing_of(pi, p_FinG);
  (void)bi;

  // Analytic H = ∂h/∂δx_imu for roles (OpenVINS convention)
  auto J = PoseOnlyGeometry::residual_jacobian_poses(bj, pj, bk, pk, pi, bi); // ∂(h-z)/∂cam = ∂h/∂cam
  const ImuPose *imu = (which == 0) ? &imu_j : (which == 1) ? &imu_k : &imu_i;
  Eigen::Matrix<double, 2, 6> H_cam = (which == 0) ? J.H_j : (which == 1) ? J.H_k : J.H_i;
  Eigen::Matrix<double, 2, 6> H_imu = PoseOnlyGeometry::chain_camera_H_to_imu_jpl(H_cam, imu->R_GtoI, R_ItoC, p_IinC);

  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 6> H_fd = Eigen::Matrix<double, 2, 6>::Zero();
  for (int c = 0; c < 6; c++) {
    Eigen::Vector3d dth = Eigen::Vector3d::Zero();
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    if (c < 3)
      dth(c) = eps;
    else
      dp(c - 3) = eps;

    auto pert = [&](const ImuPose &base, const Eigen::Vector3d &th, const Eigen::Vector3d &p) {
      ImuPose out = base;
      out.R_GtoI = exp_so3(-th) * base.R_GtoI; // JPL
      out.p_IinG = base.p_IinG + p;
      return out;
    };

    ImuPose j_p = imu_j, k_p = imu_k, i_p = imu_i;
    ImuPose j_m = imu_j, k_m = imu_k, i_m = imu_i;
    if (which == 0) {
      j_p = pert(imu_j, dth, dp);
      j_m = pert(imu_j, -dth, -dp);
    } else if (which == 1) {
      k_p = pert(imu_k, dth, dp);
      k_m = pert(imu_k, -dth, -dp);
    } else {
      i_p = pert(imu_i, dth, dp);
      i_m = pert(imu_i, -dth, -dp);
    }

    // Bearings fixed (measurements); only poses move — matches updater
    Eigen::Vector2d h_p = predict_h(bj, cam_from_imu(j_p, R_ItoC, p_IinC), bk, cam_from_imu(k_p, R_ItoC, p_IinC),
                                    cam_from_imu(i_p, R_ItoC, p_IinC));
    Eigen::Vector2d h_m = predict_h(bj, cam_from_imu(j_m, R_ItoC, p_IinC), bk, cam_from_imu(k_m, R_ItoC, p_IinC),
                                    cam_from_imu(i_m, R_ItoC, p_IinC));
    H_fd.col(c) = (h_p - h_m) / (2.0 * eps);
  }

  double err = (H_imu - H_fd).cwiseAbs().maxCoeff();
  std::cout << "  max|H_imu - H_fd|=" << err << std::endl;
  EXPECT_TRUE(err < tol);
}

int main() {
  Eigen::Matrix3d R_ItoC = exp_so3(Eigen::Vector3d(0.02, -0.01, 0.03));
  Eigen::Vector3d p_IinC(0.03, -0.05, 0.02);
  Eigen::Vector3d p_FinG(0.4, -0.2, 5.0);

  ImuPose imu_j, imu_k, imu_i;
  imu_j.R_GtoI = exp_so3(Eigen::Vector3d(0.05, 0.0, -0.02));
  imu_j.p_IinG = Eigen::Vector3d(0.0, 0.0, 0.0);
  imu_k.R_GtoI = exp_so3(Eigen::Vector3d(0.04, 0.01, -0.01));
  imu_k.p_IinG = Eigen::Vector3d(0.6, 0.05, 0.0);
  imu_i.R_GtoI = exp_so3(Eigen::Vector3d(0.03, -0.02, 0.0));
  imu_i.p_IinG = Eigen::Vector3d(0.25, -0.1, 0.02);

  const double tol = 5e-5;
  check_imu_slot("role_j", 0, imu_j, imu_k, imu_i, R_ItoC, p_IinC, p_FinG, tol);
  check_imu_slot("role_k", 1, imu_j, imu_k, imu_i, R_ItoC, p_IinC, p_FinG, tol);
  check_imu_slot("role_i", 2, imu_j, imu_k, imu_i, R_ItoC, p_IinC, p_FinG, tol);

  // Residual ~0 at truth
  auto pj = cam_from_imu(imu_j, R_ItoC, p_IinC);
  auto pk = cam_from_imu(imu_k, R_ItoC, p_IinC);
  auto pi = cam_from_imu(imu_i, R_ItoC, p_IinC);
  Eigen::Vector3d bj = bearing_of(pj, p_FinG);
  Eigen::Vector3d bk = bearing_of(pk, p_FinG);
  Eigen::Vector3d bi = bearing_of(pi, p_FinG);
  Eigen::Vector2d r = bi.head<2>() - predict_h(bj, pj, bk, pk, pi);
  std::cout << "[TEST] |r| at truth=" << r.norm() << std::endl;
  EXPECT_TRUE(r.norm() < 1e-9);

  if (g_failed == 0) {
    std::cout << "ALL PASSED" << std::endl;
    return 0;
  }
  std::cerr << g_failed << " CHECK(S) FAILED" << std::endl;
  return 1;
}
