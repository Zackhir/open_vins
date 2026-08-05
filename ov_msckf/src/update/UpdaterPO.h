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

#ifndef OV_MSCKF_UPDATER_PO_H
#define OV_MSCKF_UPDATER_PO_H

#include <Eigen/Eigen>
#include <map>
#include <memory>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"

namespace ov_core {
class Feature;
} // namespace ov_core

namespace ov_msckf {

class State;

/**
 * @brief Pose-only (PO-MSCKF) visual updater.
 *
 * Replaces classical MSCKF triangulation + nullspace projection with pose-only
 * reprojection residuals (Du et al.). Camera poses are formed from IMU clones
 * and calib_IMUtoCAM; Jacobians are chained onto IMU clones (extrinsics-in-H
 * and FEJ polish deferred to a later step).
 */
class UpdaterPO {

public:
  /**
   * @brief Default constructor for our pose-only updater
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Unused for PO path (kept for API parity with MSCKF)
   */
  UpdaterPO(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Update the state using pose-only residuals for the given features.
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

protected:
  /// Options used during update (pixel noise, chi2 multiplier, ...)
  UpdaterOptions _options;

  /// Chi-squared 95% thresholds keyed by residual dof
  std::map<int, double> chi_squared_table;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_PO_H
