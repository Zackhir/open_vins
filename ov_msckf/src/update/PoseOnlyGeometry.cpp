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
  Eigen::Vector3d p_i = bearing_i;
  Eigen::Vector3d p_j = bearing_j;
  // Normalize bearings to plane z=1 for a stable θ definition
  if (std::abs(p_i(2)) > 1e-12)
    p_i /= p_i(2);
  if (std::abs(p_j(2)) > 1e-12)
    p_j /= p_j(2);
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
  Eigen::Vector3d p_j = bearing_j;
  Eigen::Vector3d p_k = bearing_k;
  if (std::abs(p_j(2)) > 1e-12)
    p_j /= p_j(2);
  if (std::abs(p_k(2)) > 1e-12)
    p_k /= p_k(2);

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
  Eigen::Vector3d z = z_i;
  if (std::abs(z(2)) > 1e-12)
    z /= z(2);
  return z_hat - z.head<2>();
}
