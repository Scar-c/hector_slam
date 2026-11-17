// Copyright 2026 The Cartographer Authors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0

#include "cartographer/mapping/internal/2d/iterated_error_state_update.h"

#include <algorithm>
#include <cmath>

#include "Eigen/Cholesky"
#include "ceres/covariance.h"

namespace cartographer {
namespace mapping {
namespace {

class ImuPriorCost final : public ceres::SizedCostFunction<8, 8> {
 public:
  explicit ImuPriorCost(const PlanarErrorCovariance& square_root_information)
      : square_root_information_(square_root_information) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    Eigen::Map<PlanarErrorVector> residual(residuals);
    residual = square_root_information_ *
               Eigen::Map<const PlanarErrorVector>(parameters[0]);
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> jacobian(
          jacobians[0]);
      jacobian = square_root_information_;
    }
    return residual.allFinite();
  }

 private:
  const PlanarErrorCovariance square_root_information_;
};

}  // namespace

IteratedErrorStateUpdateSummary SolveIteratedErrorStateUpdate(
    const PlanarErrorCovariance& prior_covariance,
    const proto::PlanarIteratedUpdateOptions& options,
    std::vector<ErrorStateMeasurement> measurements,
    PlanarErrorVector* correction,
    PlanarErrorCovariance* posterior_covariance) {
  IteratedErrorStateUpdateSummary result;
  if (correction == nullptr || posterior_covariance == nullptr ||
      !prior_covariance.allFinite() || options.max_num_iterations() <= 0 ||
      !std::isfinite(options.max_translation_update()) ||
      !std::isfinite(options.max_yaw_update()) ||
      options.max_translation_update() <= 0. || options.max_yaw_update() <= 0.) {
    return result;
  }
  const PlanarErrorCovariance symmetric_prior =
      0.5 * (prior_covariance + prior_covariance.transpose());
  const Eigen::LLT<PlanarErrorCovariance> prior_factor(symmetric_prior);
  if (prior_factor.info() != Eigen::Success) return result;
  if (measurements.empty()) {
    correction->setZero();
    *posterior_covariance = prior_covariance;
    result.accepted = true;
    return result;
  }

  const PlanarErrorCovariance square_root_information =
      prior_factor.matrixL().solve(PlanarErrorCovariance::Identity());
  PlanarErrorVector delta = PlanarErrorVector::Zero();
  ceres::Problem problem;
  problem.AddResidualBlock(new ImuPriorCost(square_root_information), nullptr,
                           delta.data());
  for (auto& measurement : measurements) {
    if (measurement.cost == nullptr ||
        measurement.cost->parameter_block_sizes().size() != 1 ||
        measurement.cost->parameter_block_sizes()[0] != 8 ||
        measurement.cost->num_residuals() <= 0 ||
        !std::isfinite(measurement.huber_scale) || measurement.huber_scale < 0.) {
      return result;
    }
    result.measurement_residuals += measurement.cost->num_residuals();
    ceres::LossFunction* loss = measurement.huber_scale > 0.
        ? new ceres::HuberLoss(measurement.huber_scale) : nullptr;
    problem.AddResidualBlock(measurement.cost.release(), loss, delta.data());
  }

  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = options.max_num_iterations();
  solver_options.num_threads = 1;
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.function_tolerance = 1e-12;
  solver_options.gradient_tolerance = 1e-12;
  solver_options.parameter_tolerance = 1e-10;
  solver_options.initial_trust_region_radius = 1e8;
  solver_options.jacobi_scaling = false;
  solver_options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  result.iterations = static_cast<int>(summary.iterations.size());
  result.initial_cost = summary.initial_cost;
  result.final_cost = summary.final_cost;
  if (!summary.IsSolutionUsable() || !delta.allFinite() ||
      !std::isfinite(result.final_cost) ||
      result.final_cost > result.initial_cost +
                              1e-9 * std::max(1., result.initial_cost) ||
      delta.head<2>().norm() > options.max_translation_update() ||
      std::abs(delta(4)) > options.max_yaw_update()) {
    return result;
  }

  // The prior appears once in this problem. Repeated residual evaluations are
  // nonlinear relinearizations, not additional observations of the same scan.
  ceres::Covariance::Options covariance_options;
  covariance_options.algorithm_type = ceres::DENSE_SVD;
  covariance_options.num_threads = 1;
  covariance_options.apply_loss_function = true;
  ceres::Covariance covariance(covariance_options);
  const std::vector<std::pair<const double*, const double*>> blocks = {
      {delta.data(), delta.data()}};
  Eigen::Matrix<double, 8, 8, Eigen::RowMajor> posterior;
  if (!covariance.Compute(blocks, &problem) ||
      !covariance.GetCovarianceBlock(delta.data(), delta.data(), posterior.data()) ||
      !posterior.allFinite()) {
    return result;
  }
  const PlanarErrorCovariance symmetric_posterior =
      0.5 * (posterior + posterior.transpose());
  if (Eigen::LLT<PlanarErrorCovariance>(symmetric_posterior).info() !=
      Eigen::Success) {
    return result;
  }
  *correction = delta;
  *posterior_covariance = symmetric_posterior;
  result.accepted = true;
  return result;
}

}  // namespace mapping
}  // namespace cartographer
