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

#ifndef OV_MSCKF_POSE_ONLY_GEOMETRY_H
#define OV_MSCKF_POSE_ONLY_GEOMETRY_H

#include <Eigen/Eigen>
#include <utility>
#include <vector>

namespace ov_msckf {

/**
 * @brief Standalone pose-only (PO) multi-view geometry helpers (PO-MSCKF).
 *
 * Camera pose convention matches OpenVINS FeatureInitializer::ClonePose:
 *  - R_GtoC: rotation from global to camera
 *  - p_CinG: camera center in global
 *
 * Bearings are normalized image coordinates p = [x, y, 1]^T (or any scale; z>0).
 *
 * Jacobians use left-multiplicative SO(3) error on R_GtoC and additive error on p_CinG:
 *   R ← Exp(δθ) R ,  p ← p + δp
 * Each pose Jacobian is 2×6 = [∂r/∂δθ | ∂r/∂δp].
 * If residual camera i coincides with a base view, total ∂r/∂pose = sum of the role blocks.
 */
class PoseOnlyGeometry {
public:
  /// Camera pose in OpenVINS clone convention
  struct CameraPose {
    Eigen::Matrix3d R_GtoC = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_CinG = Eigen::Vector3d::Zero();
  };

  /// Analytical residual Jacobians w.r.t. base-left j, base-right k, and residual camera i
  struct ResidualPoseJacobians {
    Eigen::Matrix<double, 2, 6> H_j = Eigen::Matrix<double, 2, 6>::Zero();
    Eigen::Matrix<double, 2, 6> H_k = Eigen::Matrix<double, 2, 6>::Zero();
    Eigen::Matrix<double, 2, 6> H_i = Eigen::Matrix<double, 2, 6>::Zero();
  };

  /**
   * @brief Relative pose of camera i expressed in camera j (paper: R_cj_ci, t_cj_ci).
   * Maps a point from frame i into frame j: p_j = R * p_i + t
   */
  static void relative_pose(const CameraPose &pose_i, const CameraPose &pose_j, Eigen::Matrix3d &R_j_i, Eigen::Vector3d &t_j_i);

  /**
   * @brief Parallax-like score θ_(i,j) = ||[p_j ×] R_j_i p_i|| (paper eq. 4 / 8).
   */
  static double theta(const Eigen::Vector3d &bearing_i, const CameraPose &pose_i, const Eigen::Vector3d &bearing_j,
                      const CameraPose &pose_j);

  /**
   * @brief Select left/right base views (j,k) = argmax θ_(j,k) over unique pairs (paper eq. 8).
   * @return true if a valid pair was found
   */
  static bool select_base_views(const std::vector<Eigen::Vector3d> &bearings, const std::vector<CameraPose> &poses, int &base_j,
                                int &base_k, double *best_theta = nullptr);

  /**
   * @brief Unnormalized PO feature position in camera i (paper eq. 11).
   *
   * p_i(PO) = ||[t_k_j ×] p_k|| R_i_j p_j + ||[p_k ×] R_k_j p_j|| t_i_j
   * Overall scale is arbitrary (cancelled by perspective divide).
   */
  static Eigen::Vector3d feature_in_camera_po(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j, const Eigen::Vector3d &bearing_k,
                                              const CameraPose &pose_k, const CameraPose &pose_i);

  /**
   * @brief Perspective divide to normalized plane: [X/Z, Y/Z]^T (requires Z != 0).
   */
  static Eigen::Vector2d project_normalized(const Eigen::Vector3d &p_Cam);

  /**
   * @brief PO reprojection residual in view i (paper eq. 10): π(p_i(PO)) - z_i
   * @param z_i measured normalized bearing (uses first two components; z may be 1)
   */
  static Eigen::Vector2d residual_normalized(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j, const Eigen::Vector3d &bearing_k,
                                             const CameraPose &pose_k, const CameraPose &pose_i, const Eigen::Vector3d &z_i);

  /**
   * @brief Perspective Jacobian ∂π/∂p (2×3), paper eq. (28).
   */
  static Eigen::Matrix<double, 2, 3> project_jacobian(const Eigen::Vector3d &p_Cam);

  /**
   * @brief Jacobian of p_i(PO) w.r.t. poses j,k,i (each 3×6 = [∂/∂δθ | ∂/∂δp]).
   */
  static void feature_jacobian_poses(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j, const Eigen::Vector3d &bearing_k,
                                     const CameraPose &pose_k, const CameraPose &pose_i, Eigen::Matrix<double, 3, 6> &Dp_Dj,
                                     Eigen::Matrix<double, 3, 6> &Dp_Dk, Eigen::Matrix<double, 3, 6> &Dp_Di);

  /**
   * @brief Jacobian of residual r_i w.r.t. poses j,k,i (paper eqs. 28–38 chain rule).
   */
  static ResidualPoseJacobians residual_jacobian_poses(const Eigen::Vector3d &bearing_j, const CameraPose &pose_j,
                                                       const Eigen::Vector3d &bearing_k, const CameraPose &pose_k, const CameraPose &pose_i,
                                                       const Eigen::Vector3d &z_i);
};

} // namespace ov_msckf

#endif // OV_MSCKF_POSE_ONLY_GEOMETRY_H
