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
};

/**
 * @brief Build PO residual and Hx for one feature (standard EKF form r ≈ Hx δx + n).
 *
 * Unlike MSCKF we never triangulate a world point and never nullspace-project Hf.
 * Flow:
 *  - Flatten every (camera, time) observation into a view list (per-view calib)
 *  - Pick base views (j,k) with largest parallax θ
 *  - Residual from current IMU⊕calib camera poses
 *  - Jacobians at FEJ clones when do_fej (calib uses current, matching UpdaterHelper)
 *  - Chain camera H → IMU clones (+ calib columns if do_calib_camera_pose)
 * If i coincides with j or k, both role blocks land on the same state and are summed.
 *
 * @return false if the feature should be skipped (too few views, bad geometry, non-finite)
 */
bool build_po_linear_system(std::shared_ptr<State> state, const std::shared_ptr<Feature> &feature, Eigen::MatrixXd &H_x, Eigen::VectorXd &res,
                            std::vector<std::shared_ptr<Type>> &Hx_order) {

  const bool use_fej = state->_options.do_fej;
  const bool calib_extrinsics = state->_options.do_calib_camera_pose;

  // One PoView per observation: bearing + pixel + camera pose from that time's IMU clone ⊕ that cam's calib
  std::vector<PoView> views;
  for (const auto &pair : feature->timestamps) {
    const size_t cam_id = pair.first;
    if (state->_calib_IMUtoCAM.find(cam_id) == state->_calib_IMUtoCAM.end())
      return false;
    if (state->_cam_intrinsics_cameras.find(cam_id) == state->_cam_intrinsics_cameras.end())
      return false;
    auto calib = state->_calib_IMUtoCAM.at(cam_id);
    for (size_t m = 0; m < pair.second.size(); m++) {
      double t = pair.second.at(m);
      if (state->_clones_IMU.find(t) == state->_clones_IMU.end())
        return false;
      PoView v;
      v.cam_id = cam_id;
      v.timestamp = t;
      v.clone_imu = state->_clones_IMU.at(t);
      v.calib_imu_to_cam = calib;
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
  if (views.size() < 2)
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

  // Hx columns: per-cam calib (optional) then unique IMU clones — mirrors UpdaterHelper ordering intent
  std::unordered_map<std::shared_ptr<Type>, size_t> map_hx;
  Hx_order.clear();
  int total_hx = 0;
  for (const auto &pair : feature->timestamps) {
    auto calib = state->_calib_IMUtoCAM.at(pair.first);
    if (calib_extrinsics && map_hx.find(calib) == map_hx.end()) {
      map_hx.insert({calib, total_hx});
      Hx_order.push_back(calib);
      total_hx += (int)calib->size();
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
  // At most one skipped row (i == base_j); allocate full then conservativeResize
  res = Eigen::VectorXd::Zero(2 * n_meas);
  H_x = Eigen::MatrixXd::Zero(2 * n_meas, total_hx);
  int ct_row = 0;

  const auto &pose_j_cur = poses_cur[base_j];
  const auto &pose_k_cur = poses_cur[base_k];
  const auto &pose_j_lin = poses_lin[base_j];
  const auto &pose_k_lin = poses_lin[base_k];
  const Eigen::Vector3d &bearing_j = bearings[base_j];
  const Eigen::Vector3d &bearing_k = bearings[base_k];

  for (int i = 0; i < n_meas; i++) {
    // Paper multi-view set D(j,k) uses i ≠ j (eq. 7); i=j is structurally zero and adds no information
    if (i == base_j)
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
    ct_row++;
  }

  if (ct_row < 1)
    return false;
  res.conservativeResize(2 * ct_row);
  H_x.conservativeResize(2 * ct_row, total_hx);

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

  // 1. Drop observations without clones; need >= 2 views (PO needs a base pair)
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {
    (*it0)->clean_old_measurements(clonetimes);
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (int)(*it0)->timestamps[pair.first].size();
    }
    if (ct_meas < 2) {
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

  // 2. Linearize each feature with PO model; reject outliers with chi2 (same gate idea as MSCKF)
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;
    if (!build_po_linear_system(state, *it2, H_x, res, Hx_order)) {
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Chi2 gate: is this residual plausible given P and pixel noise?
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
    S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));

    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    }

    if (chi2 > _options.chi2_multipler * chi2_check) {
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
    it2++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // MSCKF features are consumed by this update (same policy as UpdaterMSCKF)
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  if (ct_meas < 1) {
    PRINT_ALL("[PO-UP]: no features passed gating\n");
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

  Eigen::MatrixXd R_big = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());
  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
  rT4 = boost::posix_time::microsec_clock::local_time();

  PRINT_ALL("[PO-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[PO-UP]: %.4f seconds create system (%d features used)\n", (rT2 - rT1).total_microseconds() * 1e-6, (int)ct_features_used);
  PRINT_ALL("[PO-UP]: %.4f seconds compress system\n", (rT3 - rT2).total_microseconds() * 1e-6);
  PRINT_ALL("[PO-UP]: %.4f seconds update state (%d size)\n", (rT4 - rT3).total_microseconds() * 1e-6, (int)res_big.rows());
  PRINT_ALL("[PO-UP]: %.4f seconds total\n", (rT4 - rT0).total_microseconds() * 1e-6);
}
