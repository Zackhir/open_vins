/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * Synthetic unit tests for pose-only geometry helpers (Step 2).
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

#define EXPECT_NEAR(a, b, tol)                                                                                                           \
  do {                                                                                                                                   \
    if (std::abs((a) - (b)) > (tol)) {                                                                                                   \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " : |" << (a) << " - " << (b) << "| > " << (tol) << std::endl;             \
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
  return p_FinC / p_FinC(2); // normalized [x,y,1]
}

static void test_residual_zero_at_truth() {
  std::cout << "[TEST] residual ~ 0 at true poses" << std::endl;
  // Three cameras looking roughly +Z_global with baseline translation
  PoseOnlyGeometry::CameraPose pose0 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose1 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.5, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose2 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.1, 0.0));
  Eigen::Vector3d p_FinG(0.2, -0.1, 4.0);

  Eigen::Vector3d z0 = project_bearing(pose0, p_FinG);
  Eigen::Vector3d z1 = project_bearing(pose1, p_FinG);
  Eigen::Vector3d z2 = project_bearing(pose2, p_FinG);

  // Fixed bases (0,1) for this residual check
  Eigen::Vector2d r0 = PoseOnlyGeometry::residual_normalized(z0, pose0, z1, pose1, pose0, z0);
  Eigen::Vector2d r1 = PoseOnlyGeometry::residual_normalized(z0, pose0, z1, pose1, pose1, z1);
  Eigen::Vector2d r2 = PoseOnlyGeometry::residual_normalized(z0, pose0, z1, pose1, pose2, z2);

  EXPECT_NEAR(r0.norm(), 0.0, 1e-9);
  EXPECT_NEAR(r1.norm(), 0.0, 1e-9);
  EXPECT_NEAR(r2.norm(), 0.0, 1e-9);
}

static void test_residual_grows_when_pose_wrong() {
  std::cout << "[TEST] residual grows when a pose is perturbed" << std::endl;
  PoseOnlyGeometry::CameraPose pose0 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose1 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.5, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose2 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));
  Eigen::Vector3d p_FinG(0.0, 0.0, 5.0);

  Eigen::Vector3d z0 = project_bearing(pose0, p_FinG);
  Eigen::Vector3d z1 = project_bearing(pose1, p_FinG);
  Eigen::Vector3d z2 = project_bearing(pose2, p_FinG);

  Eigen::Vector2d r_true = PoseOnlyGeometry::residual_normalized(z0, pose0, z1, pose1, pose2, z2);
  EXPECT_NEAR(r_true.norm(), 0.0, 1e-9);

  PoseOnlyGeometry::CameraPose pose2_bad = pose2;
  pose2_bad.p_CinG += Eigen::Vector3d(0.05, -0.02, 0.0);
  Eigen::Vector2d r_bad = PoseOnlyGeometry::residual_normalized(z0, pose0, z1, pose1, pose2_bad, z2);
  EXPECT_TRUE(r_bad.norm() > 1e-3);
  EXPECT_TRUE(r_bad.norm() > 10.0 * r_true.norm() + 1e-6);
}

static void test_base_view_selection_prefers_large_baseline() {
  std::cout << "[TEST] base-view selection prefers large parallax pair" << std::endl;
  // Views 0 and 2 have largest baseline; view 1 is almost co-located with 0
  PoseOnlyGeometry::CameraPose pose0 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose1 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.01, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose2 = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0));
  Eigen::Vector3d p_FinG(0.3, 0.2, 6.0);

  std::vector<PoseOnlyGeometry::CameraPose> poses = {pose0, pose1, pose2};
  std::vector<Eigen::Vector3d> bearings = {project_bearing(pose0, p_FinG), project_bearing(pose1, p_FinG), project_bearing(pose2, p_FinG)};

  int j = -1, k = -1;
  double th = 0;
  EXPECT_TRUE(PoseOnlyGeometry::select_base_views(bearings, poses, j, k, &th));
  EXPECT_TRUE((j == 0 && k == 2) || (j == 2 && k == 0));
  EXPECT_TRUE(th > PoseOnlyGeometry::theta(bearings[0], poses[0], bearings[1], poses[1]));
}

static void test_po_matches_true_camera_point_up_to_scale() {
  std::cout << "[TEST] PO feature direction matches true camera point (up to scale)" << std::endl;
  PoseOnlyGeometry::CameraPose pose_j = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose_k = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.8, 0.0, 0.0));
  PoseOnlyGeometry::CameraPose pose_i = make_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.3, 0.2, 0.0));
  Eigen::Vector3d p_FinG(-0.1, 0.15, 3.5);

  Eigen::Vector3d z_j = project_bearing(pose_j, p_FinG);
  Eigen::Vector3d z_k = project_bearing(pose_k, p_FinG);
  Eigen::Vector3d p_true_i = pose_i.R_GtoC * (p_FinG - pose_i.p_CinG);
  Eigen::Vector3d p_po = PoseOnlyGeometry::feature_in_camera_po(z_j, pose_j, z_k, pose_k, pose_i);

  // Same ray: p_po × p_true ≈ 0 and same depth sign
  EXPECT_NEAR((p_po.cross(p_true_i)).norm() / (p_po.norm() * p_true_i.norm() + 1e-12), 0.0, 1e-9);
  EXPECT_TRUE(p_po(2) * p_true_i(2) > 0.0);
}

int main() {
  test_residual_zero_at_truth();
  test_residual_grows_when_pose_wrong();
  test_base_view_selection_prefers_large_baseline();
  test_po_matches_true_camera_point_up_to_scale();

  if (g_failed == 0) {
    std::cout << "ALL PoseOnlyGeometry TESTS PASSED" << std::endl;
    return 0;
  }
  std::cerr << g_failed << " assertion(s) failed" << std::endl;
  return 1;
}
