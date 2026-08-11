/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "UpdaterPO.h"

#include "PoseOnlyGeometry.h"
#include "UpdaterHelper.h"

#include "feat/Feature.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <map>
#include <unordered_map>
#include <vector>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace {

struct PoView {
  size_t cam_id = 0;
  double timestamp = 0;
  Eigen::Vector3d bearing = Eigen::Vector3d::Zero(); // normalized [x,y,1]
  Eigen::Vector2d uv_ms = Eigen::Vector2d::Zero();   // raw pixel measurement
  PoseOnlyGeometry::CameraPose cam_pose;             // current estimate (residual)
  PoseOnlyGeometry::CameraPose cam_pose_lin;         // FEJ if enabled, else current (Jacobians)
  Eigen::Matrix3d R_GtoI_lin = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_ItoC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinC = Eigen::Vector3d::Zero();
  std::shared_ptr<PoseJPL> clone_imu;
  std::shared_ptr<PoseJPL> calib_imu_to_cam;
  std::shared_ptr<Vec> cam_intrinsics;
};

/**
 * @brief Build PO residual and Hx for one feature, whitened to unit isotropic noise.
 *
 * Unlike MSCKF we never triangulate a world point and never nullspace-project Hf.
 * Flow:
 *  - Flatten every (camera, time) observation into a view list (per-view calib)
 *  - Pick base views (j,k) with largest parallax θ
 *  - Residual from current IMU⊕calib camera poses
 *  - Jacobians at FEJ clones when do_fej (calib uses current, matching UpdaterHelper)
 *  - Chain camera H → IMU clones (+ calib columns if do_calib_camera_pose)
 *  - Build pixel noise Jacobian G = ∂r/∂{all uv}: own-view I plus shared base bearings z_j,z_k
 *    via ∂π(p_PO)/∂bearing, then whiten with R = σ² G Gᵀ so stacked update may use R = I
 * If i coincides with j or k, both role blocks land on the same state and are summed.
 *
 * @return false if the feature should be skipped (too few views, bad geometry, non-finite)
 */
bool build_po_linear_system(std::shared_ptr<State> state, const std::shared_ptr<Feature> &feature, Eigen::MatrixXd &H_x, Eigen::VectorXd &res,
                            std::vector<std::shared_ptr<Type>> &Hx_order, double sigma_pix_sq) {

  const bool use_fej = state->_options.do_fej;
  const bool calib_extrinsics = state->_options.do_calib_camera_pose;
  const bool calib_intrinsics = state->_options.do_calib_camera_intrinsics;
  const auto po_variant = state->_options.po_variant;
  const bool include_base_k = state->_options.po_includes_base_k();
  const bool use_bearing_G = state->_options.po_uses_bearing_G();
  const bool isotropic_all = (po_variant == StateOptions::PoVariant::ISOTROPIC_IK);
  const bool hybrid_ik = (po_variant == StateOptions::PoVariant::HYBRID_IK);
  const double hybrid_ik_scale = state->_options.po_hybrid_ik_scale;

  // One PoView per observation: bearing + pixel + camera pose from that time's IMU clone ⊕ that cam's calib
  std::vector<PoView> views;
  for (const auto &pair : feature->timestamps) {
    const size_t cam_id = pair.first;
    if (state->_calib_IMUtoCAM.find(cam_id) == state->_calib_IMUtoCAM.end())
      return false;
    if (state->_cam_intrinsics_cameras.find(cam_id) == state->_cam_intrinsics_cameras.end())
      return false;
    if (state->_cam_intrinsics.find(cam_id) == state->_cam_intrinsics.end())
      return false;
    auto calib = state->_calib_IMUtoCAM.at(cam_id);
    auto intrins = state->_cam_intrinsics.at(cam_id);
    for (size_t m = 0; m < pair.second.size(); m++) {
      double t = pair.second.at(m);
      if (state->_clones_IMU.find(t) == state->_clones_IMU.end())
        return false;
      PoView v;
      v.cam_id = cam_id;
      v.timestamp = t;
      v.clone_imu = state->_clones_IMU.at(t);
      v.calib_imu_to_cam = calib;
      v.cam_intrinsics = intrins;
      v.R_ItoC = calib->Rot();
      v.p_IinC = calib->pos();
      v.cam_pose = PoseOnlyGeometry::compose_camera_pose(v.clone_imu->Rot(), v.clone_imu->pos(), v.R_ItoC, v.p_IinC);
      if (use_fej) {
        v.R_GtoI_lin = v.clone_imu->Rot_fej();
        v.cam_pose_lin = PoseOnlyGeometry::compose_camera_pose(v.R_GtoI_lin, v.clone_imu->pos_fej(), v.R_ItoC, v.p_IinC);
      } else {
        v.R_GtoI_lin = v.clone_imu->Rot();
        v.cam_pose_lin = v.cam_pose;
      }
      Eigen::VectorXf uvn = feature->uvs_norm.at(cam_id).at(m);
      v.bearing << uvn(0), uvn(1), 1.0;
      Eigen::VectorXf uv = feature->uvs.at(cam_id).at(m);
      v.uv_ms << uv(0), uv(1);
      views.push_back(v);
    }
  }
  // Need j, k, and at least one residual view. Paper D includes i=k; default skips both bases.
  if ((int)views.size() < (include_base_k ? 2 : 3))
    return false;

  std::vector<Eigen::Vector3d> bearings;
  std::vector<PoseOnlyGeometry::CameraPose> poses_cur;
  std::vector<PoseOnlyGeometry::CameraPose> poses_lin;
  bearings.reserve(views.size());
  poses_cur.reserve(views.size());
  poses_lin.reserve(views.size());
  for (const auto &v : views) {
    bearings.push_back(v.bearing);
    poses_cur.push_back(v.cam_pose);
    poses_lin.push_back(v.cam_pose_lin);
  }

  // Virtual stereo pair for this track (paper eq. 8) — use current geometry
  int base_j = 0, base_k = 1;
  double best_theta = 0;
  if (!PoseOnlyGeometry::select_base_views(bearings, poses_cur, base_j, base_k, &best_theta) || best_theta < 1e-6)
    return false;

  // Reject degenerate PPO pairs: need both scale factors in eq. (11) well-conditioned
  {
    Eigen::Matrix3d R_k_j;
    Eigen::Vector3d t_k_j;
    PoseOnlyGeometry::relative_pose(poses_cur[base_j], poses_cur[base_k], R_k_j, t_k_j);
    const Eigen::Vector3d p_j = bearings[base_j];
    const Eigen::Vector3d p_k = bearings[base_k];
    const double a = (t_k_j.cross(p_k)).norm();
    const double b = (p_k.cross(R_k_j * p_j)).norm();
    if (a < 1e-6 || b < 1e-6)
      return false;
  }

  // Hx columns: per-cam extrinsics / intrinsics (optional) then unique IMU clones — mirrors UpdaterHelper
  std::unordered_map<std::shared_ptr<Type>, size_t> map_hx;
  Hx_order.clear();
  int total_hx = 0;
  for (const auto &pair : feature->timestamps) {
    auto calib = state->_calib_IMUtoCAM.at(pair.first);
    auto intrins = state->_cam_intrinsics.at(pair.first);
    if (calib_extrinsics && map_hx.find(calib) == map_hx.end()) {
      map_hx.insert({calib, total_hx});
      Hx_order.push_back(calib);
      total_hx += (int)calib->size();
    }
    if (calib_intrinsics && map_hx.find(intrins) == map_hx.end()) {
      map_hx.insert({intrins, total_hx});
      Hx_order.push_back(intrins);
      total_hx += (int)intrins->size();
    }
    for (size_t m = 0; m < pair.second.size(); m++) {
      auto clone = state->_clones_IMU.at(pair.second.at(m));
      if (map_hx.find(clone) == map_hx.end()) {
        map_hx.insert({clone, total_hx});
        Hx_order.push_back(clone);
        total_hx += (int)clone->size();
      }
    }
  }

  const int n_meas = (int)views.size();
  // Skip both base views; allocate full then conservativeResize
  res = Eigen::VectorXd::Zero(2 * n_meas);
  H_x = Eigen::MatrixXd::Zero(2 * n_meas, total_hx);
  // G = ∂r/∂uv for every view in the track (shared z_j,z_k couple the rows)
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2 * n_meas, 2 * n_meas);
  int ct_row = 0;

  const auto &pose_j_cur = poses_cur[base_j];
  const auto &pose_k_cur = poses_cur[base_k];
  const auto &pose_j_lin = poses_lin[base_j];
  const auto &pose_k_lin = poses_lin[base_k];
  const Eigen::Vector3d &bearing_j = bearings[base_j];
  const Eigen::Vector3d &bearing_k = bearings[base_k];

  // ∂bearing/∂uv ≈ (∂distort/∂zn)^{-1} at the measured normalized point
  auto bearing_from_uv_jacobian = [&](int view_idx) -> Eigen::Matrix2d {
    Eigen::MatrixXd dz_dzn, dz_dzeta;
    Eigen::Vector2d zn = bearings[view_idx].head<2>();
    state->_cam_intrinsics_cameras.at(views[view_idx].cam_id)->compute_distort_jacobian(zn, dz_dzn, dz_dzeta);
    Eigen::Matrix2d J = dz_dzn.topLeftCorner<2, 2>();
    Eigen::FullPivLU<Eigen::Matrix2d> lu(J);
    if (!lu.isInvertible())
      return Eigen::Matrix2d::Identity() * 1e-3; // extremely defensive
    return lu.inverse();
  };
  Eigen::Matrix2d Dbj_Duv = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d Dbk_Duv = Eigen::Matrix2d::Identity();
  if (use_bearing_G) {
    Dbj_Duv = bearing_from_uv_jacobian(base_j);
    Dbk_Duv = bearing_from_uv_jacobian(base_k);
  }

  // Row indices (in units of 2-vector blocks) that came from i=k — hybrid inflate target.
  std::vector<int> ik_rows;

  for (int i = 0; i < n_meas; i++) {
    // Always skip i=j (paper D(j,k)). bearing_skip_ik also skips i=k.
    if (i == base_j)
      continue;
    if (!include_base_k && i == base_k)
      continue;

    // Residual at current estimate
    Eigen::Vector3d p_po = PoseOnlyGeometry::feature_in_camera_po(bearing_j, pose_j_cur, bearing_k, pose_k_cur, poses_cur[i]);
    // Require feature in front of camera (paper assumes Z>0); behind-camera PO points corrupt the update
    if (p_po(2) < 1e-6)
      return false;

    Eigen::Vector2d z_hat = PoseOnlyGeometry::project_normalized(p_po);
    Eigen::MatrixXd dz_dzn, dz_dzeta;
    state->_cam_intrinsics_cameras.at(views[i].cam_id)->compute_distort_jacobian(z_hat, dz_dzn, dz_dzeta);
    Eigen::Vector2d uv_dist = state->_cam_intrinsics_cameras.at(views[i].cam_id)->distort_d(z_hat);
    res.segment<2>(2 * ct_row) = views[i].uv_ms - uv_dist;

    // Jacobians at FEJ (or current) linearization poses
    // PoseOnlyGeometry::residual_jacobian_poses returns ∂(z_hat - z_norm)/∂cam = ∂h_norm/∂cam.
    // OpenVINS EKFUpdate uses res = z - h(x) with H = ∂h/∂x (same as UpdaterMSCKF), so do NOT negate.
    auto Jcam = PoseOnlyGeometry::residual_jacobian_poses(bearing_j, pose_j_lin, bearing_k, pose_k_lin, poses_lin[i], bearings[i]);
    Eigen::Matrix<double, 2, 6> H_cam_i = Jcam.H_i;
    Eigen::Matrix<double, 2, 6> H_cam_j = Jcam.H_j;
    Eigen::Matrix<double, 2, 6> H_cam_k = Jcam.H_k;

    auto accumulate = [&](int view_idx, const Eigen::Matrix<double, 2, 6> &H_cam_role) {
      const PoView &v = views[view_idx];
      Eigen::Matrix<double, 2, 6> H_imu =
          PoseOnlyGeometry::chain_camera_H_to_imu_jpl(H_cam_role, v.R_GtoI_lin, v.R_ItoC, v.p_IinC);
      H_x.block(2 * ct_row, map_hx[v.clone_imu], 2, 6) += dz_dzn * H_imu;

      if (calib_extrinsics) {
        Eigen::Matrix<double, 2, 6> H_calib =
            PoseOnlyGeometry::chain_camera_H_to_calib_jpl(H_cam_role, v.R_GtoI_lin, v.R_ItoC, v.p_IinC);
        H_x.block(2 * ct_row, map_hx[v.calib_imu_to_cam], 2, 6) += dz_dzn * H_calib;
      }
    };

    accumulate(i, H_cam_i);
    accumulate(base_j, H_cam_j);
    accumulate(base_k, H_cam_k);

    // Intrinsics: h = distort(z_hat; ζ) ⇒ ∂h/∂ζ = dz_dzeta (same as UpdaterHelper)
    if (calib_intrinsics) {
      H_x.block(2 * ct_row, map_hx[views[i].cam_intrinsics], 2, views[i].cam_intrinsics->size()) = dz_dzeta;
    }

    // Bearing noise map G for rows that use GGᵀ whitening.
    // hybrid_ik: leave G rows for i=k at zero → floored whitening ≈ soft σ²I on that block.
    if (use_bearing_G && i != base_k) {
      Eigen::Matrix<double, 2, 2> DH_Dbj, DH_Dbk;
      PoseOnlyGeometry::prediction_jacobian_bearings(bearing_j, pose_j_lin, bearing_k, pose_k_lin, poses_lin[i], DH_Dbj, DH_Dbk);
      Eigen::Matrix2d dz_dzn2 = dz_dzn.topLeftCorner<2, 2>();
      G.block<2, 2>(2 * ct_row, 2 * i) += Eigen::Matrix2d::Identity();
      G.block<2, 2>(2 * ct_row, 2 * base_j) += -dz_dzn2 * DH_Dbj * Dbj_Duv;
      G.block<2, 2>(2 * ct_row, 2 * base_k) += -dz_dzn2 * DH_Dbk * Dbk_Duv;
    }

    if (hybrid_ik && i == base_k)
      ik_rows.push_back(ct_row);

    ct_row++;
  }

  if (ct_row < 1)
    return false;
  res.conservativeResize(2 * ct_row);
  H_x.conservativeResize(2 * ct_row, total_hx);
  G.conservativeResize(2 * ct_row, 2 * n_meas);

  if (!res.allFinite() || !H_x.allFinite())
    return false;

  // isotropic_ik: R=σ²I → whiten by σ so χ² / QR / EKFUpdate all use R=I
  if (isotropic_all) {
    const double inv_sigma = 1.0 / std::sqrt(sigma_pix_sq);
    res *= inv_sigma;
    H_x *= inv_sigma;
    return res.allFinite() && H_x.allFinite();
  }

  if (!G.allFinite())
    return false;

  // Whiten with R = σ² · Q max(Λ, I) Qᵀ where G Gᵀ = Q Λ Qᵀ.
  // Shared base bearings make some stacked-PO directions nearly noise-free under the
  // linear model; flooring eigenvalues at σ² prevents those directions from dominating
  // the update (overconfident P → χ² blackout → IMU drift). After whitening, R_eff = I.
  // hybrid_ik zero-G i=k rows → floored eig ≈ soft isotropic σ² on that block (Problem A safe).
  Eigen::MatrixXd GG = G * G.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(GG);
  if (eig.info() != Eigen::Success)
    return false;
  Eigen::VectorXd lam = eig.eigenvalues().cwiseMax(1.0);
  Eigen::MatrixXd R = sigma_pix_sq * (eig.eigenvectors() * lam.asDiagonal() * eig.eigenvectors().transpose());
  Eigen::LLT<Eigen::MatrixXd> llt(R);
  if (llt.info() != Eigen::Success)
    return false;
  Eigen::MatrixXd L = llt.matrixL();
  res = L.triangularView<Eigen::Lower>().solve(res);
  H_x = L.triangularView<Eigen::Lower>().solve(H_x);

  // Fix A: hybrid_ik only — inflate i=k to R_ik = α σ² I by scaling whitened rows by 1/√α.
  // α=1 leaves baseline soft-floor behavior unchanged.
  if (hybrid_ik && hybrid_ik_scale > 1.0 + 1e-12) {
    const double inv_sqrt_alpha = 1.0 / std::sqrt(hybrid_ik_scale);
    for (int row : ik_rows) {
      res.segment<2>(2 * row) *= inv_sqrt_alpha;
      H_x.block(2 * row, 0, 2, H_x.cols()) *= inv_sqrt_alpha;
    }
  }

  if (!res.allFinite() || !H_x.allFinite())
    return false;
  return true;
}

} // namespace

UpdaterPO::UpdaterPO(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options) : _options(options) {
  (void)feat_init_options;
  _options.sigma_pix_sq = std::pow(_options.sigma_pix, 2);
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterPO::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // High-level PO-MSCKF visual update (enabled via use_pose_only_update):
  //   clean tracks → per-feature PO r/Hx (no tri / nullspace) → chi2 → compress → EKFUpdate
  // State vector is unchanged vs classical OpenVINS; only the measurement model differs.

  if (feature_vec.empty())
    return;

  boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Clone timestamps that still exist in the sliding window
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Drop observations without clones; need >= 3 views (or >= 2 if including i=k)
  const int min_views = state->_options.po_includes_base_k() ? 2 : 3;
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {
    (*it0)->clean_old_measurements(clonetimes);
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (int)(*it0)->timestamps[pair.first].size();
    }
    if (ct_meas < min_views) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }
  size_t max_hx_size = state->max_covariance_size();
  for (auto &landmark : state->_features_SLAM) {
    max_hx_size -= landmark.second->size();
  }

  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  size_t ct_jacob = 0;
  size_t ct_meas = 0;
  size_t ct_features_used = 0;
  size_t ct_build_fail = 0;
  size_t ct_chi2_reject = 0;
  double sum_res_norm_accept = 0.0;
  double sum_res_norm_reject = 0.0;
  double sum_chi2_accept = 0.0;
  double sum_chi2_reject = 0.0;
  double max_res_norm_accept = 0.0;

  // 2. Linearize each feature with PO model; reject outliers with chi2 (same gate idea as MSCKF)
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;
    if (!build_po_linear_system(state, *it2, H_x, res, Hx_order, _options.sigma_pix_sq)) {
      ct_build_fail++;
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Chi2 gate on whitened residual (all variants leave effective R = I)
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
    S.diagonal() += Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));
    const double res_norm = res.norm();

    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    }

    if (chi2 > _options.chi2_multipler * chi2_check) {
      ct_chi2_reject++;
      sum_res_norm_reject += res_norm;
      sum_chi2_reject += chi2;
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Append into the stacked system for this update epoch
    size_t ct_hx = 0;
    for (const auto &var : Hx_order) {
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }
      Hx_big.block(ct_meas, Hx_mapping[var], H_x.rows(), var->size()) = H_x.block(0, ct_hx, H_x.rows(), var->size());
      ct_hx += var->size();
    }
    res_big.block(ct_meas, 0, res.rows(), 1) = res;
    ct_meas += res.rows();
    ct_features_used++;
    sum_res_norm_accept += res_norm;
    sum_chi2_accept += chi2;
    max_res_norm_accept = std::max(max_res_norm_accept, res_norm);
    it2++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // MSCKF features are consumed by this update (same policy as UpdaterMSCKF)
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  if (ct_meas < 1) {
    PRINT_INFO("[PO-UP]: no features passed gating (build_fail=%zu chi2_reject=%zu)\n", ct_build_fail, ct_chi2_reject);
    return;
  }
  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  res_big.conservativeResize(ct_meas, 1);
  Hx_big.conservativeResize(ct_meas, ct_jacob);

  // 3. QR measurement compression then standard EKF update (r ≈ Hx δx + n)
  UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
  if (Hx_big.rows() < 1) {
    return;
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // All po_variant paths whiten to unit isotropic noise before stacking
  Eigen::MatrixXd R_big = Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());
  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
  rT4 = boost::posix_time::microsec_clock::local_time();

  const double n_acc = std::max(1.0, (double)ct_features_used);
  const double n_rej = std::max(1.0, (double)ct_chi2_reject);
  PRINT_INFO("[PO-UP]: variant=%s ik_scale=%.2f used=%zu build_fail=%zu chi2_reject=%zu | "
             "res_norm mean_acc=%.2f max_acc=%.2f mean_rej=%.2f | chi2 mean_acc=%.1f mean_rej=%.1f | "
             "sigma_pix=%.2f chi2_mult=%.2f\n",
             StateOptions::po_variant_to_string(state->_options.po_variant), state->_options.po_hybrid_ik_scale,
             ct_features_used, ct_build_fail, ct_chi2_reject, sum_res_norm_accept / n_acc, max_res_norm_accept,
             sum_res_norm_reject / n_rej, sum_chi2_accept / n_acc, sum_chi2_reject / n_rej, _options.sigma_pix,
             _options.chi2_multipler);
  PRINT_ALL("[PO-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[PO-UP]: %.4f seconds create system (%d features used)\n", (rT2 - rT1).total_microseconds() * 1e-6, (int)ct_features_used);
  PRINT_ALL("[PO-UP]: %.4f seconds compress system\n", (rT3 - rT2).total_microseconds() * 1e-6);
  PRINT_ALL("[PO-UP]: %.4f seconds update state (%d size)\n", (rT4 - rT3).total_microseconds() * 1e-6, (int)res_big.rows());
  PRINT_ALL("[PO-UP]: %.4f seconds total\n", (rT4 - rT0).total_microseconds() * 1e-6);
}
