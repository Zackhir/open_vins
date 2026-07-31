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
#include <memory>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"

namespace ov_core {
class Feature;
class FeatureInitializer;
} // namespace ov_core

namespace ov_msckf {

class State;
class UpdaterMSCKF;

/**
 * @brief Pose-only (PO-MSCKF) visual updater.
 *
 * Will replace classical MSCKF triangulation + nullspace projection with
 * pose-only reprojection residuals (Du et al., PO-MSCKF).
 *
 * Step 1 scaffold: when enabled, this class currently delegates to the
 * classical MSCKF updater so plumbing can be tested with identical results.
 * Later steps replace the update body with PO residuals and Jacobians.
 */
class UpdaterPO {

public:
  /**
   * @brief Default constructor for our pose-only updater
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Feature initializer options
   */
  UpdaterPO(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked features, this will try to use them to update the state.
   *
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Temporary MSCKF fallback used by the Step 1 scaffold (identical behavior)
  std::shared_ptr<UpdaterMSCKF> msckf_fallback;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_PO_H
