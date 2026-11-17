// Copyright 2026 The Cartographer Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cartographer/mapping/internal/2d/iterated_error_state_update.h"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

proto::PlanarIteratedUpdateOptions TestOptions() {
  proto::PlanarIteratedUpdateOptions options;
  options.set_enabled(true);
  options.set_max_num_iterations(10);
  options.set_grid_residual_stddev(0.15);
  options.set_huber_scale(2.5);
  options.set_max_translation_update(10.);
  options.set_max_yaw_update(3.);
  return options;
}

// The residuals supplied to the solver are already whitened. The option's
// grid standard deviation belongs to the caller constructing lidar costs.
class LinearCost final : public ceres::SizedCostFunction<1, 8> {
 public:
  LinearCost(const PlanarErrorVector& h, double observation, double stddev)
      : h_(h / stddev), observation_(observation / stddev) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    residuals[0] = h_.dot(Eigen::Map<const PlanarErrorVector>(parameters[0])) -
                   observation_;
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      for (int i = 0; i < 8; ++i) jacobians[0][i] = h_(i);
    }
    return true;
  }

 private:
  const PlanarErrorVector h_;
  const double observation_;
};

class ExponentialCost final : public ceres::SizedCostFunction<1, 8> {
 public:
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double value = std::exp(parameters[0][0]);
    residuals[0] = value - 2.;
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      for (int i = 0; i < 8; ++i) jacobians[0][i] = 0.;
      jacobians[0][0] = value;
    }
    return true;
  }
};

class InvalidCost final : public ceres::SizedCostFunction<1, 8> {
 public:
  enum Failure { kEvaluateFailure, kNaNResidual, kNaNJacobian };
  explicit InvalidCost(Failure failure) : failure_(failure) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    residuals[0] = failure_ == kNaNResidual
                       ? std::numeric_limits<double>::quiet_NaN()
                       : parameters[0][0] - 1.;
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      for (int i = 0; i < 8; ++i) jacobians[0][i] = 0.;
      jacobians[0][0] = failure_ == kNaNJacobian
                            ? std::numeric_limits<double>::quiet_NaN()
                            : 1.;
    }
    return failure_ != kEvaluateFailure;
  }

 private:
  const Failure failure_;
};

void AddLinear(const PlanarErrorVector& h, double observation, double stddev,
               double huber_scale,
               std::vector<ErrorStateMeasurement>* measurements) {
  ErrorStateMeasurement measurement;
  measurement.cost.reset(new LinearCost(h, observation, stddev));
  measurement.huber_scale = huber_scale;
  measurements->push_back(std::move(measurement));
}

std::vector<ErrorStateMeasurement> ExponentialMeasurements() {
  std::vector<ErrorStateMeasurement> measurements;
  ErrorStateMeasurement measurement;
  measurement.cost.reset(new ExponentialCost);
  measurements.push_back(std::move(measurement));
  return measurements;
}

PlanarErrorCovariance CorrelatedPrior() {
  PlanarErrorCovariance prior = PlanarErrorCovariance::Zero();
  PlanarErrorVector direction;
  direction << 1., -0.8, 0.6, -0.5, 0.4, -0.3, 0.7, -0.9;
  for (int i = 0; i < 8; ++i) prior(i, i) = 0.5 + 0.2 * i;
  prior += 0.7 * direction * direction.transpose();
  return prior;
}

void ExpectUnchanged(const PlanarErrorVector& correction,
                     const PlanarErrorCovariance& posterior) {
  EXPECT_TRUE((correction.array() == 17.).all());
  EXPECT_TRUE((posterior.array() == -23.).all());
}

TEST(IteratedErrorStateUpdateTest, LinearMapAndFullPosteriorMatchKalmanUpdate) {
  const PlanarErrorCovariance prior = CorrelatedPrior();
  Eigen::Matrix<double, 3, 8> h = Eigen::Matrix<double, 3, 8>::Zero();
  h(0, 0) = 1.;
  h(1, 1) = 1.;
  h(2, 4) = 1.;
  const Eigen::Vector3d observation(0.7, -0.4, 0.25);
  const Eigen::Vector3d variance(0.09, 0.16, 0.04);
  const Eigen::Matrix3d noise = variance.asDiagonal();
  const Eigen::Matrix3d innovation = h * prior * h.transpose() + noise;
  const Eigen::Matrix<double, 8, 3> gain =
      prior * h.transpose() * innovation.ldlt().solve(Eigen::Matrix3d::Identity());
  const PlanarErrorVector expected_correction = gain * observation;
  // Joseph form is an independent Kalman covariance oracle and includes every
  // velocity/bias cross covariance, despite observing only x, y, and yaw.
  const PlanarErrorCovariance a = PlanarErrorCovariance::Identity() - gain * h;
  const PlanarErrorCovariance expected_posterior =
      a * prior * a.transpose() + gain * noise * gain.transpose();
  std::vector<ErrorStateMeasurement> measurements;
  for (int i = 0; i < 3; ++i) {
    AddLinear(h.row(i).transpose(), observation(i), std::sqrt(variance(i)),
              0., &measurements);
  }
  PlanarErrorVector correction;
  PlanarErrorCovariance posterior;
  const auto summary = SolveIteratedErrorStateUpdate(
      prior, TestOptions(), std::move(measurements), &correction, &posterior);
  ASSERT_TRUE(summary.accepted);
  EXPECT_EQ(summary.measurement_residuals, 3);
  EXPECT_GT(summary.iterations, 0);
  EXPECT_LT((correction - expected_correction).norm(), 1e-6);
  EXPECT_LT((posterior - expected_posterior).norm(), 1e-9);
  for (const int i : {2, 3, 5, 6, 7}) {
    EXPECT_GT(std::abs(expected_correction(i)), 1e-3);
    EXPECT_LT(posterior(i, i), prior(i, i));
  }
  const double expected_initial_cost =
      0.5 * observation.dot(noise.ldlt().solve(observation));
  const double expected_final_cost =
      0.5 * observation.dot(innovation.ldlt().solve(observation));
  EXPECT_NEAR(summary.initial_cost, expected_initial_cost, 1e-10);
  EXPECT_NEAR(summary.final_cost, expected_final_cost, 1e-8);
}

TEST(IteratedErrorStateUpdateTest, IterationBudgetDoesNotReconditionPrior) {
  const PlanarErrorCovariance prior = CorrelatedPrior();
  PlanarErrorVector h;
  h << 1., -0.5, 0.2, 0.1, 0.4, -0.3, 0.25, 0.15;
  const double variance = 0.36;
  const PlanarErrorVector ph = prior * h;
  const PlanarErrorCovariance expected =
      prior - ph * ph.transpose() / (variance + h.dot(ph));
  PlanarErrorCovariance posteriors[2];
  const int budgets[] = {1, 10};
  for (int i = 0; i < 2; ++i) {
    auto options = TestOptions();
    options.set_max_num_iterations(budgets[i]);
    std::vector<ErrorStateMeasurement> measurements;
    AddLinear(h, 0.8, std::sqrt(variance), 0., &measurements);
    PlanarErrorVector correction;
    const auto summary = SolveIteratedErrorStateUpdate(
        prior, options, std::move(measurements), &correction, &posteriors[i]);
    ASSERT_TRUE(summary.accepted) << "budget=" << budgets[i];
    EXPECT_LT((posteriors[i] - expected).norm(), 1e-9);
  }
  EXPECT_LT((posteriors[0] - posteriors[1]).norm(), 1e-10);
}

TEST(IteratedErrorStateUpdateTest, NonlinearRelinearizationFindsScalarMap) {
  PlanarErrorCovariance prior = PlanarErrorCovariance::Identity();
  prior(0, 0) = 4.;
  auto single_options = TestOptions();
  single_options.set_max_num_iterations(1);
  PlanarErrorVector single, iterated;
  PlanarErrorCovariance single_posterior, iterated_posterior;
  const auto single_summary = SolveIteratedErrorStateUpdate(
      prior, single_options, ExponentialMeasurements(), &single,
      &single_posterior);
  const auto iterated_summary = SolveIteratedErrorStateUpdate(
      prior, TestOptions(), ExponentialMeasurements(), &iterated,
      &iterated_posterior);
  ASSERT_TRUE(single_summary.accepted);
  ASSERT_TRUE(iterated_summary.accepted);
  // On [0, log(2)], f'(x) = x/4 + exp(x)*(exp(x)-2) is strictly
  // increasing. Bisection supplies a MAP oracle without a least-squares solve.
  double lower = 0.;
  double upper = std::log(2.);
  for (int i = 0; i < 80; ++i) {
    const double x = 0.5 * (lower + upper);
    const double value = std::exp(x);
    if (x / 4. + value * (value - 2.) > 0.) {
      upper = x;
    } else {
      lower = x;
    }
  }
  const double expected = 0.5 * (lower + upper);
  EXPECT_NEAR(iterated(0), expected, 2e-5);
  EXPECT_LT(std::abs(iterated(0) - expected),
            0.01 * std::abs(single(0) - expected));
  EXPECT_LT(iterated_summary.final_cost, single_summary.final_cost - 1e-3);
  EXPECT_GT(iterated_summary.iterations, single_summary.iterations);
  EXPECT_NEAR(iterated_summary.initial_cost, 0.5, 1e-12);
  EXPECT_LT(iterated.tail<7>().norm(), 1e-12);
}

TEST(IteratedErrorStateUpdateTest, NonlinearPosteriorUsesFinalJacobian) {
  PlanarErrorCovariance prior = PlanarErrorCovariance::Identity();
  prior(0, 0) = 4.;
  PlanarErrorVector correction;
  PlanarErrorCovariance posterior;
  const auto summary = SolveIteratedErrorStateUpdate(
      prior, TestOptions(), ExponentialMeasurements(), &correction, &posterior);
  ASSERT_TRUE(summary.accepted);
  // Independent local measurement variance: dr/dx = exp(x). The exact
  // objective Hessian also contains r*d2r/dx2; the required covariance is GN.
  PlanarErrorCovariance expected = PlanarErrorCovariance::Identity();
  expected(0, 0) = 1. / (0.25 + std::exp(2. * correction(0)));
  EXPECT_LT((posterior - expected).norm(), 1e-9);
  EXPECT_GT(std::abs(posterior(0, 0) - 1. / 1.25), 0.1);
}

TEST(IteratedErrorStateUpdateTest, RankOneGeometryPreservesNullspace) {
  const PlanarErrorCovariance prior = 2. * PlanarErrorCovariance::Identity();
  PlanarErrorVector h = PlanarErrorVector::Zero();
  h(0) = 1.;
  h(1) = 2.;
  std::vector<ErrorStateMeasurement> measurements;
  AddLinear(h, 1., 0.5, 0., &measurements);
  AddLinear(h, 1., 0.5, 0., &measurements);
  PlanarErrorVector correction;
  PlanarErrorCovariance posterior;
  const auto summary = SolveIteratedErrorStateUpdate(
      prior, TestOptions(), std::move(measurements), &correction, &posterior);
  ASSERT_TRUE(summary.accepted);
  PlanarErrorVector tangent = PlanarErrorVector::Zero();
  tangent(0) = -2.;
  tangent(1) = 1.;
  tangent.normalize();
  EXPECT_NEAR(tangent.dot(posterior * tangent), 2., 1e-10);
  // LM may stop at machine-level objective improvement with a sub-micrometer
  // tangent remainder. Covariance must still retain the exact null direction.
  EXPECT_NEAR(tangent.dot(correction), 0., 1e-6);
  for (int i = 2; i < 8; ++i) {
    EXPECT_NEAR(posterior(i, i), 2., 1e-10);
    EXPECT_NEAR(correction(i), 0., 1e-10);
  }
  const PlanarErrorVector normal = h.normalized();
  EXPECT_NEAR(normal.dot(posterior * normal), 1. / (0.5 + 2. * 5. / 0.25),
              1e-10);
}

TEST(IteratedErrorStateUpdateTest, ZeroGeometryLeavesCorrelatedPriorIntact) {
  const PlanarErrorCovariance prior = CorrelatedPrior();
  std::vector<ErrorStateMeasurement> measurements;
  AddLinear(PlanarErrorVector::Zero(), 3., 1., 0., &measurements);
  PlanarErrorVector correction = PlanarErrorVector::Constant(17.);
  PlanarErrorCovariance posterior;
  const auto summary = SolveIteratedErrorStateUpdate(
      prior, TestOptions(), std::move(measurements), &correction, &posterior);
  ASSERT_TRUE(summary.accepted);
  EXPECT_EQ(summary.measurement_residuals, 1);
  EXPECT_LT(correction.norm(), 1e-12);
  EXPECT_LT((posterior - prior).norm(), 1e-9);
  EXPECT_NEAR(summary.initial_cost, 4.5, 1e-12);
  EXPECT_NEAR(summary.final_cost, 4.5, 1e-12);
}

TEST(IteratedErrorStateUpdateTest, NoMeasurementsReturnsZeroAndOriginalPrior) {
  const PlanarErrorCovariance prior = CorrelatedPrior();
  PlanarErrorVector correction = PlanarErrorVector::Constant(17.);
  PlanarErrorCovariance posterior = PlanarErrorCovariance::Constant(-23.);
  const auto summary = SolveIteratedErrorStateUpdate(
      prior, TestOptions(), {}, &correction, &posterior);
  ASSERT_TRUE(summary.accepted);
  EXPECT_EQ(summary.iterations, 0);
  EXPECT_EQ(summary.measurement_residuals, 0);
  EXPECT_DOUBLE_EQ(summary.initial_cost, 0.);
  EXPECT_DOUBLE_EQ(summary.final_cost, 0.);
  EXPECT_DOUBLE_EQ(correction.norm(), 0.);
  EXPECT_DOUBLE_EQ((posterior - prior).norm(), 0.);
}

TEST(IteratedErrorStateUpdateTest, PosteriorIsSymmetricPositiveDefinite) {
  const PlanarErrorCovariance prior = CorrelatedPrior();
  std::vector<ErrorStateMeasurement> measurements;
  for (int i = 0; i < 8; ++i) {
    PlanarErrorVector h = PlanarErrorVector::Zero();
    h(i) = 1.;
    h((i + 1) % 8) = 0.3;
    AddLinear(h, 0.01 * (i + 1), 0.05 * (i + 1), 0., &measurements);
  }
  PlanarErrorVector correction;
  PlanarErrorCovariance posterior;
  const auto summary = SolveIteratedErrorStateUpdate(
      prior, TestOptions(), std::move(measurements), &correction, &posterior);
  ASSERT_TRUE(summary.accepted);
  EXPECT_TRUE(correction.allFinite());
  EXPECT_TRUE(posterior.allFinite());
  EXPECT_LT((posterior - posterior.transpose()).norm(), 1e-12);
  const Eigen::LLT<PlanarErrorCovariance> factor(posterior);
  EXPECT_EQ(factor.info(), Eigen::Success);
  const Eigen::SelfAdjointEigenSolver<PlanarErrorCovariance> eigen(posterior);
  ASSERT_EQ(eigen.info(), Eigen::Success);
  EXPECT_GT(eigen.eigenvalues().minCoeff(), 0.);
  const Eigen::SelfAdjointEigenSolver<PlanarErrorCovariance> reduction(
      prior - posterior);
  ASSERT_EQ(reduction.info(), Eigen::Success);
  EXPECT_GE(reduction.eigenvalues().minCoeff(), -1e-10);
}

TEST(IteratedErrorStateUpdateTest, PerPointHuberLimitsOutlierInfluence) {
  const auto options = TestOptions();
  const PlanarErrorCovariance prior = PlanarErrorCovariance::Identity();
  const PlanarErrorVector h = PlanarErrorVector::Unit(0);
  PlanarErrorVector corrections[2];
  PlanarErrorCovariance posteriors[2];
  for (int robust = 0; robust < 2; ++robust) {
    std::vector<ErrorStateMeasurement> measurements;
    const double scale = robust ? options.huber_scale() : 0.;
    for (int i = 0; i < 20; ++i) {
      AddLinear(h, 0.5, 1., scale, &measurements);
    }
    for (int i = 0; i < 2; ++i) {
      AddLinear(h, 100., 1., scale, &measurements);
    }
    const auto summary = SolveIteratedErrorStateUpdate(
        prior, options, std::move(measurements), &corrections[robust],
        &posteriors[robust]);
    ASSERT_TRUE(summary.accepted) << "robust=" << robust;
    EXPECT_EQ(summary.measurement_residuals, 22);
    EXPECT_LE(summary.final_cost, summary.initial_cost);
  }
  // Each outlier contributes -a to the Huber score; all twenty inliers
  // remain quadratic. Thus x + 20*(x-0.5) - 2*a = 0 exactly.
  const double expected_robust = (20. * 0.5 + 2. * options.huber_scale()) / 21.;
  EXPECT_NEAR(corrections[1](0), expected_robust, 1e-4);
  EXPECT_NEAR(corrections[0](0), (20. * 0.5 + 2. * 100.) / 23., 1e-5);
  EXPECT_LT(std::abs(corrections[1](0) - 0.5),
            0.1 * std::abs(corrections[0](0) - 0.5));
  EXPECT_GT(posteriors[1](0, 0), posteriors[0](0, 0));
}

TEST(IteratedErrorStateUpdateTest, InvalidEvaluationDoesNotMutateOutputs) {
  for (const auto failure : {InvalidCost::kEvaluateFailure,
                             InvalidCost::kNaNResidual,
                             InvalidCost::kNaNJacobian}) {
    SCOPED_TRACE(static_cast<int>(failure));
    std::vector<ErrorStateMeasurement> measurements;
    ErrorStateMeasurement measurement;
    measurement.cost.reset(new InvalidCost(failure));
    measurements.push_back(std::move(measurement));
    PlanarErrorVector correction = PlanarErrorVector::Constant(17.);
    PlanarErrorCovariance posterior = PlanarErrorCovariance::Constant(-23.);
    const auto summary = SolveIteratedErrorStateUpdate(
        PlanarErrorCovariance::Identity(), TestOptions(),
        std::move(measurements), &correction, &posterior);
    EXPECT_FALSE(summary.accepted);
    ExpectUnchanged(correction, posterior);
  }
}

TEST(IteratedErrorStateUpdateTest, NonPositiveDefinitePriorDoesNotMutateOutputs) {
  for (const double bad_variance : {0., -0.1}) {
    SCOPED_TRACE(bad_variance);
    PlanarErrorCovariance prior = PlanarErrorCovariance::Identity();
    prior(6, 6) = bad_variance;
    std::vector<ErrorStateMeasurement> measurements;
    AddLinear(PlanarErrorVector::Unit(0), 1., 1., 0., &measurements);
    PlanarErrorVector correction = PlanarErrorVector::Constant(17.);
    PlanarErrorCovariance posterior = PlanarErrorCovariance::Constant(-23.);
    const auto summary = SolveIteratedErrorStateUpdate(
        prior, TestOptions(), std::move(measurements), &correction, &posterior);
    EXPECT_FALSE(summary.accepted);
    ExpectUnchanged(correction, posterior);
  }
}

TEST(IteratedErrorStateUpdateTest, TranslationNormBoundRejectsWithoutMutation) {
  auto options = TestOptions();
  options.set_max_translation_update(0.5);
  std::vector<ErrorStateMeasurement> measurements;
  // Each component fits the limit, but the 2D norm exceeds it. The posterior
  // MAP components are 0.45/1.01, so this detects componentwise gating too.
  AddLinear(PlanarErrorVector::Unit(0), 0.45, 0.1, 0., &measurements);
  AddLinear(PlanarErrorVector::Unit(1), 0.45, 0.1, 0., &measurements);
  PlanarErrorVector correction = PlanarErrorVector::Constant(17.);
  PlanarErrorCovariance posterior = PlanarErrorCovariance::Constant(-23.);
  const auto summary = SolveIteratedErrorStateUpdate(
      PlanarErrorCovariance::Identity(), options, std::move(measurements),
      &correction, &posterior);
  EXPECT_FALSE(summary.accepted);
  ExpectUnchanged(correction, posterior);
}

TEST(IteratedErrorStateUpdateTest, BothYawSignsRejectWithoutMutation) {
  auto options = TestOptions();
  options.set_max_yaw_update(0.1);
  for (const double yaw : {-0.4, 0.4}) {
    SCOPED_TRACE(yaw);
    std::vector<ErrorStateMeasurement> measurements;
    AddLinear(PlanarErrorVector::Unit(4), yaw, 0.1, 0., &measurements);
    PlanarErrorVector correction = PlanarErrorVector::Constant(17.);
    PlanarErrorCovariance posterior = PlanarErrorCovariance::Constant(-23.);
    const auto summary = SolveIteratedErrorStateUpdate(
        PlanarErrorCovariance::Identity(), options, std::move(measurements),
        &correction, &posterior);
    EXPECT_FALSE(summary.accepted);
    ExpectUnchanged(correction, posterior);
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
