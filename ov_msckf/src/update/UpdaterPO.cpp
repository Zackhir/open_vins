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
  PoseOnlyGeometry::CameraPose cam_pose;
  std::shared_ptr<PoseJPL> clone_imu;
  std::shared_ptr<PoseJPL> calib_imu_to_cam;
};

PoseOnlyGeometry::CameraPose camera_pose_from_imu(const std::shared_ptr<PoseJPL> &clone_imu,
                                                  const std::shared_ptr<PoseJPL> &calib) {
  // Same composition as UpdaterMSCKF / FeatureInitializer
  PoseOnlyGeometry::CameraPose pose;
  pose.R_GtoC = calib->Rot() * clone_imu->Rot();
  pose.p_CinG = clone_imu->pos() - pose.R_GtoC.transpose() * calib->pos();
  return pose;
}

/**
 * @brief Map camera-pose Jacobian (our left-Exp R_GtoC model) to OpenVINS IMU clone error.
 *
 * OpenVINS JPL orientation uses R'≈(I-[δθ×])R; our Step-3 model uses R'≈Exp(δθ)R=(I+[δθ×])R.
 * With R_GtoC = R_ItoC R_GtoI that implies δθ_C_ours ≈ -R_ItoC δθ_I.
 * Extrinsics are treated as fixed (Step 5 can add calib columns).
 */
Eigen::Matrix<double, 2, 6> chain_camera_H_to_imu(const Eigen::Matrix<double, 2, 6> &H_cam, const std::shared_ptr<PoseJPL> &clone_imu,
                                                  const std::shared_ptr<PoseJPL> &calib) {
  const Eigen::Matrix3d R_ItoC = calib->Rot();
  const Eigen::Matrix3d R_GtoI = clone_imu->Rot();
  const Eigen::Vector3d lever_I = R_ItoC.transpose() * calib->pos(); // R_ItoC^T p_IinC

  Eigen::Matrix<double, 2, 3> H_th = H_cam.block<2, 3>(0, 0);
  Eigen::Matrix<double, 2, 3> H_p = H_cam.block<2, 3>(0, 3);

  Eigen::Matrix<double, 2, 6> H_imu = Eigen::Matrix<double, 2, 6>::Zero();
  H_imu.block<2, 3>(0, 0) = H_th * (-R_ItoC) + H_p * (R_GtoI.transpose() * skew_x(lever_I));
  H_imu.block<2, 3>(0, 3) = H_p;
  return H_imu;
}

bool build_po_linear_system(std::shared_ptr<State> state, const std::shared_ptr<Feature> &feature, Eigen::MatrixXd &H_x, Eigen::VectorXd &res,
                            std::vector<std::shared_ptr<Type>> &Hx_order) {

  // Flatten all camera/time observations for this feature
  std::vector<PoView> views;
  for (const auto &pair : feature->timestamps) {
    const size_t cam_id = pair.first;
    if (state->_calib_IMUtoCAM.find(cam_id) == state->_calib_IMUtoCAM.end())
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
      v.cam_pose = camera_pose_from_imu(v.clone_imu, calib);
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
  std::vector<PoseOnlyGeometry::CameraPose> poses;
  bearings.reserve(views.size());
  poses.reserve(views.size());
  for (const auto &v : views) {
    bearings.push_back(v.bearing);
    poses.push_back(v.cam_pose);
  }

  int base_j = 0, base_k = 1;
  double best_theta = 0;
  if (!PoseOnlyGeometry::select_base_views(bearings, poses, base_j, base_k, &best_theta) || best_theta < 1e-8)
    return false;

  // Unique IMU clones involved
  std::unordered_map<std::shared_ptr<Type>, size_t> map_hx;
  Hx_order.clear();
  int total_hx = 0;
  for (const auto &v : views) {
    if (map_hx.find(v.clone_imu) == map_hx.end()) {
      map_hx.insert({v.clone_imu, total_hx});
      Hx_order.push_back(v.clone_imu);
      total_hx += (int)v.clone_imu->size();
    }
  }

  const int n_meas = (int)views.size();
  res = Eigen::VectorXd::Zero(2 * n_meas);
  H_x = Eigen::MatrixXd::Zero(2 * n_meas, total_hx);

  const auto &pose_j = poses[base_j];
  const auto &pose_k = poses[base_k];
  const Eigen::Vector3d &bearing_j = bearings[base_j];
  const Eigen::Vector3d &bearing_k = bearings[base_k];

  for (int i = 0; i < n_meas; i++) {
    Eigen::Vector3d p_po = PoseOnlyGeometry::feature_in_camera_po(bearing_j, pose_j, bearing_k, pose_k, poses[i]);
    if (std::abs(p_po(2)) < 1e-6)
      return false;

    Eigen::Vector2d z_hat = PoseOnlyGeometry::project_normalized(p_po);

    // Pixel residual: uv_m - distort(z_hat)
    Eigen::MatrixXd dz_dzn, dz_dzeta;
    state->_cam_intrinsics_cameras.at(views[i].cam_id)->compute_distort_jacobian(z_hat, dz_dzn, dz_dzeta);
    Eigen::Vector2d uv_dist = state->_cam_intrinsics_cameras.at(views[i].cam_id)->distort_d(z_hat);
    res.segment<2>(2 * i) = views[i].uv_ms - uv_dist;

    // Camera-role Jacobians of (z_hat - z_norm) → flip for (z - z_hat), then distort
    auto Jcam = PoseOnlyGeometry::residual_jacobian_poses(bearing_j, pose_j, bearing_k, pose_k, poses[i], bearings[i]);
    Eigen::Matrix<double, 2, 6> H_cam_i = -Jcam.H_i;
    Eigen::Matrix<double, 2, 6> H_cam_j = -Jcam.H_j;
    Eigen::Matrix<double, 2, 6> H_cam_k = -Jcam.H_k;

    auto accumulate = [&](int view_idx, const Eigen::Matrix<double, 2, 6> &H_cam_role) {
      Eigen::Matrix<double, 2, 6> H_imu = chain_camera_H_to_imu(H_cam_role, views[view_idx].clone_imu, views[view_idx].calib_imu_to_cam);
      Eigen::MatrixXd H_pix = dz_dzn * H_imu;
      size_t col = map_hx[views[view_idx].clone_imu];
      H_x.block(2 * i, col, 2, 6) += H_pix;
    };

    accumulate(i, H_cam_i);
    accumulate(base_j, H_cam_j);
    accumulate(base_k, H_cam_k);
  }

  // Drop near-zero rows / NaNs
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

  if (feature_vec.empty())
    return;

  boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Valid clone times
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean measurements / require >= 2 views (no triangulation)
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

  // 2. PO residual + Jacobian per feature (no nullspace)
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

    // Chi2 gate (same structure as MSCKF)
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

  // 3. Compress + EKF update
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
