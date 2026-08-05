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

#include "UpdaterMSCKF.h"

#include "utils/print.h"

using namespace ov_core;
using namespace ov_msckf;

UpdaterPO::UpdaterPO(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options) : _options(options) {
  // Scaffold: reuse classical MSCKF update until PO residuals/Jacobians land.
  msckf_fallback = std::make_shared<UpdaterMSCKF>(options, feat_init_options);
}

void UpdaterPO::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // STEP1 scaffold only — behavior must match UpdaterMSCKF.
  // Later steps will:
  //  - select base views from normalized bearings
  //  - form pose-only reprojection residuals (no 3D triangulation)
  //  - build PO Jacobians w.r.t. clone poses (no nullspace projection)
  //  - keep chi2 gating, measurement compression, and EKFUpdate
  PRINT_ALL("[PO-UP]: scaffold active — delegating to classical MSCKF update\n");
  msckf_fallback->update(state, feature_vec);
}
