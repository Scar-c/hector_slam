// Copyright 2026 The Cartographer Authors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_ITERATED_ERROR_STATE_UPDATE_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_ITERATED_ERROR_STATE_UPDATE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {

// World-frame additive position/velocity errors and SO(2) yaw error. In this
// chart the reset Jacobian after injection is identity, including across +/-pi.
using PlanarErrorVector = Eigen::Matrix<double, 8, 1>;
using PlanarErrorCovariance = Eigen::Matrix<double, 8, 8>;

struct ErrorStateMeasurement {
  std::unique_ptr<ceres::CostFunction> cost;
  // Each measurement owns its loss; scalar lidar blocks get per-point Huber.
  double huber_scale = 0.;
};

struct IteratedErrorStateUpdateSummary {
  bool accepted = false;
  int iterations = 0;
  int measurement_residuals = 0;
  double initial_cost = 0.;
  double final_cost = 0.;
};

// Solve against one frozen IMU prior. Measurement costs consume an 8D error
// relative to that prior and are relinearized by Ceres, without intermediate
// nominal-state injections or repeated covariance conditioning. Outputs remain
// untouched on failure. The final robust GN covariance is computed only once.
IteratedErrorStateUpdateSummary SolveIteratedErrorStateUpdate(
    const PlanarErrorCovariance& prior_covariance,
    const proto::PlanarIteratedUpdateOptions& options,
    std::vector<ErrorStateMeasurement> measurements,
    PlanarErrorVector* correction,
    PlanarErrorCovariance* posterior_covariance);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_ITERATED_ERROR_STATE_UPDATE_H_
