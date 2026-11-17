// Copyright 2026 The Cartographer Authors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0

#include "cartographer/mapping/internal/2d/planar_imu_pose_extrapolator.h"

#include <algorithm>
#include <cmath>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

Eigen::Quaterniond YawRotation(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

Eigen::Quaterniond WithoutYaw(const Eigen::Quaterniond& rotation) {
  return (YawRotation(-transform::GetYaw(rotation)) * rotation).normalized();
}

Eigen::Quaterniond RotationIncrement(const Eigen::Vector3d& rotation_vector) {
  return transform::AngleAxisVectorToRotationQuaternion(rotation_vector);
}

double Square(double value) { return value * value; }

// One scalar block per return allows a separate robust loss for each beam.
// Reuse Cartographer's existing bicubic/autodiff likelihood and lift its pose
// Jacobian into the full error state. No intermediate pose observation is made.
class PlanarLidarResidual final : public ceres::SizedCostFunction<1, 8> {
 public:
  PlanarLidarResidual(const sensor::RangefinderPoint& point, const Grid2D& grid,
                     const PlanarErrorVector& nominal, double stddev)
      : nominal_(nominal) {
    point_.push_back(point);
    if (grid.GetGridType() == GridType::PROBABILITY_GRID) {
      cost_.reset(scan_matching::CreateOccupiedSpaceCostFunction2D(
          1. / stddev, point_, grid));
    } else {
      cost_.reset(scan_matching::CreateTSDFMatchCostFunction2D(
          1. / stddev, point_, static_cast<const TSDF2D&>(grid)));
    }
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double* delta = parameters[0];
    const double pose[3] = {nominal_(0) + delta[0], nominal_(1) + delta[1],
                            nominal_(4) + delta[4]};
    double const* pose_parameters[] = {pose};
    double pose_jacobian[3];
    double* pose_jacobians[] = {pose_jacobian};
    const bool need_jacobian = jacobians != nullptr && jacobians[0] != nullptr;
    if (!cost_->Evaluate(pose_parameters, residuals,
                         need_jacobian ? pose_jacobians : nullptr)) return false;
    if (need_jacobian) {
      Eigen::Map<Eigen::Matrix<double, 1, 8>> jacobian(jacobians[0]);
      jacobian.setZero();
      jacobian(0) = pose_jacobian[0];
      jacobian(1) = pose_jacobian[1];
      jacobian(4) = pose_jacobian[2];
      if (!jacobian.allFinite()) return false;
    }
    return std::isfinite(residuals[0]);
  }

 private:
  const PlanarErrorVector nominal_;
  sensor::PointCloud point_;
  std::unique_ptr<ceres::CostFunction> cost_;
};

struct BodyLateralVelocityResidual {
  BodyLateralVelocityResidual(const PlanarErrorVector& nominal, double stddev)
      : nominal(nominal), inverse_stddev(1. / stddev) {}

  template <typename T>
  bool operator()(const T* delta, T* residual) const {
    using std::cos;
    using std::sin;
    const T yaw = T(nominal(4)) + delta[4];
    const T vx = T(nominal(2)) + delta[2];
    const T vy = T(nominal(3)) + delta[3];
    residual[0] = T(inverse_stddev) * (-sin(yaw) * vx + cos(yaw) * vy);
    return true;
  }

  const PlanarErrorVector nominal;
  const double inverse_stddev;
};

}  // namespace

PlanarImuPoseExtrapolator::PlanarImuPoseExtrapolator(
    const proto::PlanarImuDeskewOptions& options,
    double gravity_time_constant)
    : options_(options), gravity_time_constant_(gravity_time_constant) {
  CHECK_GT(gravity_time_constant_, 0.);
  CHECK_GT(options_.gyro_noise(), 0.);
  CHECK_GT(options_.acceleration_noise(), 0.);
  CHECK_GT(options_.gyro_bias_noise(), 0.);
  CHECK_GT(options_.acceleration_bias_noise(), 0.);
  CHECK_GT(options_.scan_translation_stddev(), 0.);
  CHECK_GT(options_.scan_rotation_stddev(), 0.);
  CHECK_GT(options_.nhc_velocity_stddev(), 0.);
  CHECK_GT(options_.max_imu_gap_sec(), 0.);
  CHECK_GT(options_.max_extrapolation_sec(), 0.);
  if (options_.iterated_update().enabled()) {
    CHECK_GT(options_.iterated_update().max_num_iterations(), 0);
    CHECK_GT(options_.iterated_update().grid_residual_stddev(), 0.);
    CHECK_GE(options_.iterated_update().huber_scale(), 0.);
    CHECK_GT(options_.iterated_update().max_translation_update(), 0.);
    CHECK_GT(options_.iterated_update().max_yaw_update(), 0.);
    LOG(INFO) << "Planar IESKF enabled: direct per-point lidar factors, frozen "
                 "full IMU prior, joint NHC, single posterior covariance update.";
  }
  state_.covariance.diagonal() << 0.0001, 0.0001, 1., 1., 0.0009,
      0.0025, 0.04, 0.04;
  LOG(INFO) << "Planar IMU deskew enabled: midpoint propagation, frame-end "
               "compensation, SE(2) scan feedback; soft body NHC="
            << options_.use_nhc();
}

PlanarImuPoseExtrapolator::~PlanarImuPoseExtrapolator() {
  LOG(INFO) << "Planar IMU deskew: corrections=" << num_corrections_
            << " fallback_frames=" << num_fallbacks_
            << " gyro_bias_z=" << state_.gyro_bias
            << " accel_bias_xy=" << state_.accel_bias.transpose();
  if (options_.iterated_update().enabled()) {
    LOG(INFO) << "Planar IESKF summary: updates=" << num_tight_updates_
              << " rejected=" << num_tight_rejections_
              << " solver_iterations=" << num_tight_iterations_
              << " lidar_residuals=" << num_tight_lidar_residuals_;
  }
}

common::Time PlanarImuPoseExtrapolator::GetLastPoseTime() const {
  return state_.time;
}

common::Time PlanarImuPoseExtrapolator::GetLastExtrapolatedTime() const {
  return last_extrapolated_time_;
}

void PlanarImuPoseExtrapolator::AddImuData(const sensor::ImuData& data) {
  if (!data.linear_acceleration.allFinite() ||
      !data.angular_velocity.allFinite()) {
    LOG(WARNING) << "Ignoring non-finite IMU measurement.";
    return;
  }
  if (!imu_data_.empty() && data.time < imu_data_.back().time) {
    LOG(WARNING) << "Ignoring out-of-order IMU measurement.";
    return;
  }
  if (!imu_data_.empty() && data.time == imu_data_.back().time) {
    imu_data_.back() = data;
  } else {
    imu_data_.push_back(data);
  }
  if (!gravity_tracker_) {
    CHECK_GE(data.time, state_.time);
    gravity_tracker_ = absl::make_unique<ImuTracker>(gravity_time_constant_,
                                                   data.time);
    if (data.linear_acceleration.norm() > 1.) {
      gravity_tracker_->AddImuLinearAccelerationObservation(
          data.linear_acceleration);
    }
    state_.tilt = WithoutYaw(gravity_tracker_->orientation());
  }
  trajectory_valid_ = false;
  TrimImuData();
}

void PlanarImuPoseExtrapolator::AddOdometryData(
    const sensor::OdometryData&) {
  LOG_FIRST_N(WARNING, 1)
      << "Planar IMU deskew does not fuse odometry; use scan-matching feedback "
         "and optional NHC, or select the legacy extrapolator.";
}

sensor::ImuData PlanarImuPoseExtrapolator::MeasurementAt(
    common::Time time) const {
  auto upper = std::upper_bound(
      imu_data_.begin(), imu_data_.end(), time,
      [](common::Time t, const sensor::ImuData& sample) { return t < sample.time; });
  if (upper == imu_data_.begin()) return imu_data_.front();
  if (upper == imu_data_.end()) return imu_data_.back();
  const auto& lower = *std::prev(upper);
  const double fraction = common::ToSeconds(time - lower.time) /
                          common::ToSeconds(upper->time - lower.time);
  return sensor::ImuData{
      time,
      (1. - fraction) * lower.linear_acceleration +
          fraction * upper->linear_acceleration,
      (1. - fraction) * lower.angular_velocity + fraction * upper->angular_velocity};
}

bool PlanarImuPoseExtrapolator::MeasurementSupported(common::Time time) const {
  auto upper = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), time,
      [](const sensor::ImuData& sample, common::Time t) { return sample.time < t; });
  if (upper != imu_data_.end() && upper->time == time) return true;
  if (upper == imu_data_.begin()) {
    return common::ToSeconds(upper->time - time) <= options_.max_extrapolation_sec();
  }
  if (upper == imu_data_.end()) {
    return common::ToSeconds(time - imu_data_.back().time) <=
           options_.max_extrapolation_sec();
  }
  return common::ToSeconds(upper->time - std::prev(upper)->time) <=
         options_.max_imu_gap_sec();
}

void PlanarImuPoseExtrapolator::BuildTrajectory(common::Time end_time) {
  CHECK_NE(state_.time, common::Time::min());
  CHECK_GE(end_time, state_.time);
  CHECK(gravity_tracker_ != nullptr);
  CHECK(!imu_data_.empty());
  if (trajectory_valid_ && predicted_.time == end_time) return;
  segments_.clear();
  predicted_ = state_;
  predicted_gravity_tracker_ = absl::make_unique<ImuTracker>(*gravity_tracker_);
  trajectory_healthy_ = true;

  std::vector<common::Time> boundaries;
  for (const auto& sample : imu_data_) {
    if (sample.time > state_.time && sample.time < end_time) {
      boundaries.push_back(sample.time);
    }
  }
  if (end_time > state_.time) boundaries.push_back(end_time);
  for (const auto& next_time : boundaries) {
    const State start = predicted_;
    const double dt = common::ToSeconds(next_time - start.time);
    const auto head = MeasurementAt(start.time);
    const auto tail = MeasurementAt(next_time);
    const bool healthy = dt <= options_.max_imu_gap_sec() &&
                         MeasurementSupported(start.time) &&
                         MeasurementSupported(next_time);
    trajectory_healthy_ = trajectory_healthy_ && healthy;
    Eigen::Vector3d omega = 0.5 * (head.angular_velocity + tail.angular_velocity);
    omega.z() -= start.gyro_bias;
    const Eigen::Vector3d measured_acceleration =
        0.5 * (head.linear_acceleration + tail.linear_acceleration);

    predicted_gravity_tracker_->AddImuAngularVelocityObservation(omega);
    predicted_gravity_tracker_->Advance(next_time);
    if (healthy && measured_acceleration.norm() > 1.) {
      predicted_gravity_tracker_->AddImuLinearAccelerationObservation(
          measured_acceleration -
          Eigen::Vector3d(start.accel_bias.x(), start.accel_bias.y(), 0.));
    }
    predicted_.tilt = WithoutYaw(predicted_gravity_tracker_->orientation());
    const Eigen::Quaterniond start_rotation = YawRotation(start.yaw) * start.tilt;
    predicted_.yaw = transform::GetYaw(start_rotation * RotationIncrement(omega * dt));
    const Eigen::Quaterniond integrated_end = start_rotation * RotationIncrement(omega * dt);
    const Eigen::Quaterniond gravity_correction = integrated_end.conjugate() *
        (YawRotation(predicted_.yaw) * predicted_.tilt);
    const Eigen::Quaterniond midpoint_rotation =
        start_rotation * RotationIncrement(0.5 * omega * dt) *
        Eigen::Quaterniond::Identity().slerp(0.5, gravity_correction);
    const Eigen::Matrix3d rotation = midpoint_rotation.toRotationMatrix();
    Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
    if (healthy) {
      // World gravity is vertical; only its horizontal projection is retained.
      acceleration = (midpoint_rotation *
          (measured_acceleration -
           Eigen::Vector3d(start.accel_bias.x(), start.accel_bias.y(), 0.))).head<2>();
    }
    predicted_.position += start.velocity * dt + 0.5 * acceleration * dt * dt;
    predicted_.velocity += acceleration * dt;
    predicted_.time = next_time;

    Covariance transition = Covariance::Identity();
    transition.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * dt;
    if (healthy) {
      const Eigen::Vector2d yaw_derivative(-acceleration.y(), acceleration.x());
      transition.block<2, 1>(0, 4) = 0.5 * dt * dt * yaw_derivative;
      transition.block<2, 1>(2, 4) = dt * yaw_derivative;
      transition.block<2, 2>(0, 6) = -0.5 * dt * dt * rotation.topLeftCorner<2, 2>();
      transition.block<2, 2>(2, 6) = -dt * rotation.topLeftCorner<2, 2>();
    }
    const Eigen::Matrix3d start_matrix = start_rotation.toRotationMatrix();
    const double cos_pitch_squared =
        std::max(0.01, 1. - Square(start_matrix(2, 0)));
    transition(4, 5) = -dt * start_matrix(2, 2) / cos_pitch_squared;
    Covariance noise = Covariance::Zero();
    const double acceleration_variance =
        Square(options_.acceleration_noise()) * (healthy ? 1. : 100.);
    noise.block<2, 2>(0, 0).diagonal().setConstant(acceleration_variance * dt * dt * dt / 3.);
    noise.block<2, 2>(0, 2).diagonal().setConstant(acceleration_variance * dt * dt / 2.);
    noise.block<2, 2>(2, 0) = noise.block<2, 2>(0, 2);
    noise.block<2, 2>(2, 2).diagonal().setConstant(acceleration_variance * dt);
    noise(4, 4) = Square(options_.gyro_noise()) * dt * (healthy ? 1. : 100.);
    noise(5, 5) = Square(options_.gyro_bias_noise()) * dt;
    noise.block<2, 2>(6, 6).diagonal().setConstant(
        Square(options_.acceleration_bias_noise()) * dt);
    predicted_.covariance = transition * start.covariance * transition.transpose() + noise;
    segments_.push_back(Segment{start, predicted_, omega, acceleration});
  }
  trajectory_valid_ = true;
  if (!trajectory_healthy_) {
    LOG_EVERY_N(WARNING, 100)
        << "Planar IMU deskew: IMU gap exceeds configured limit; using "
           "constant velocity on missing intervals, without bias learning.";
  }
}

transform::Rigid3d PlanarImuPoseExtrapolator::ToPose(const State& state) {
  return transform::Rigid3d(
      Eigen::Vector3d(state.position.x(), state.position.y(), 0.),
      YawRotation(state.yaw) * state.tilt);
}

transform::Rigid3d PlanarImuPoseExtrapolator::PoseAt(
    common::Time time, size_t* segment_index) const {
  CHECK_GE(time, state_.time);
  CHECK_LE(time, predicted_.time);
  if (time == predicted_.time) return ToPose(predicted_);
  if (segments_.empty()) return ToPose(state_);
  while (*segment_index > 0 && time < segments_[*segment_index].start.time) {
    --*segment_index;
  }
  const Segment& segment = segments_[*segment_index];
  const double dt = common::ToSeconds(time - segment.start.time);
  const double fraction = dt / common::ToSeconds(segment.end.time - segment.start.time);
  State point = segment.start;
  point.position += point.velocity * dt + 0.5 * segment.acceleration * dt * dt;
  const Eigen::Quaterniond start_rotation =
      YawRotation(segment.start.yaw) * segment.start.tilt;
  const Eigen::Quaterniond integrated_end = start_rotation * RotationIncrement(
      segment.angular_velocity * common::ToSeconds(segment.end.time - segment.start.time));
  const Eigen::Quaterniond correction = integrated_end.conjugate() *
      (YawRotation(segment.end.yaw) * segment.end.tilt);
  const Eigen::Quaterniond rotation = start_rotation *
      RotationIncrement(segment.angular_velocity * dt) *
      Eigen::Quaterniond::Identity().slerp(fraction, correction);
  return transform::Rigid3d(
      Eigen::Vector3d(point.position.x(), point.position.y(), 0.), rotation.normalized());
}

PoseExtrapolatorInterface::ExtrapolationResult
PlanarImuPoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  CHECK(!times.empty());
  CHECK(std::is_sorted(times.begin(), times.end()));
  BuildTrajectory(times.back());
  CHECK_GE(times.front(), state_.time);
  std::vector<transform::Rigid3f> poses(times.size() - 1);
  size_t segment = segments_.empty() ? 0 : segments_.size() - 1;
  // Walk the cached IMU segments backward, evaluating each point at its own time.
  for (size_t i = poses.size(); i > 0; --i) {
    poses[i - 1] = PoseAt(times[i - 1], &segment).cast<float>();
  }
  last_extrapolated_time_ = times.back();
  return ExtrapolationResult{
      std::move(poses), ToPose(predicted_),
      Eigen::Vector3d(predicted_.velocity.x(), predicted_.velocity.y(), 0.),
      predicted_.tilt};
}

transform::Rigid3d PlanarImuPoseExtrapolator::ExtrapolatePose(common::Time time) {
  return ExtrapolatePosesWithGravity({time}).current_pose;
}

Eigen::Quaterniond PlanarImuPoseExtrapolator::EstimateGravityOrientation(
    common::Time time) {
  BuildTrajectory(time);
  return predicted_.tilt;
}

void PlanarImuPoseExtrapolator::Correct(
    const Eigen::VectorXd& innovation, const Eigen::MatrixXd& jacobian,
    const Eigen::MatrixXd& noise, bool update_biases) {
  const Eigen::MatrixXd covariance_times_h = state_.covariance * jacobian.transpose();
  const Eigen::MatrixXd innovation_covariance = jacobian * covariance_times_h + noise;
  Eigen::MatrixXd gain = innovation_covariance.ldlt().solve(
      covariance_times_h.transpose()).transpose();
  if (!update_biases) gain.bottomRows(3).setZero();
  const Eigen::Matrix<double, 8, 1> correction = gain * innovation;
  state_.position += correction.head<2>();
  state_.velocity += correction.segment<2>(2);
  state_.yaw = common::NormalizeAngleDifference(state_.yaw + correction(4));
  state_.gyro_bias += correction(5);
  state_.accel_bias += correction.tail<2>();
  const Covariance residual = Covariance::Identity() - gain * jacobian;
  // Joseph form preserves covariance symmetry/positive semidefiniteness.
  state_.covariance = residual * state_.covariance * residual.transpose() +
                      gain * noise * gain.transpose();
  state_.covariance =
      (0.5 * (state_.covariance + state_.covariance.transpose())).eval();
}

void PlanarImuPoseExtrapolator::AddPose(
    common::Time time, const transform::Rigid3d& pose) {
  UpdateWithPose(time, pose, nullptr, nullptr);
}

void PlanarImuPoseExtrapolator::AddPoseWithInformation(
    common::Time time, const transform::Rigid3d& pose,
    const Eigen::Matrix3d& information, const Eigen::Vector3d& gradient) {
  UpdateWithPose(time, pose, &information, &gradient);
}

void PlanarImuPoseExtrapolator::AnchorToScanPose(
    common::Time time, const transform::Rigid3d& pose) {
  CHECK_EQ(state_.time, time);
  state_.position = pose.translation().head<2>();
  state_.yaw = transform::GetYaw(pose.rotation());
  trajectory_valid_ = false;
}

IteratedErrorStateUpdateSummary PlanarImuPoseExtrapolator::UpdateWithLidar(
    common::Time time, const sensor::PointCloud& gravity_aligned_points,
    const Grid2D* grid) {
  CHECK(options_.iterated_update().enabled());
  BuildTrajectory(time);
  // Preserve propagation even if the lidar solve is rejected. A failed update
  // must not turn into a partly applied nominal state or overconfident prior.
  state_ = predicted_;
  gravity_tracker_ = absl::make_unique<ImuTracker>(*predicted_gravity_tracker_);
  IteratedErrorStateUpdateSummary result;
  if (!trajectory_healthy_) {
    ++num_tight_rejections_;
    FinishUpdate();
    return result;
  }

  PlanarErrorVector nominal;
  nominal << state_.position, state_.velocity, state_.yaw, state_.gyro_bias,
      state_.accel_bias;
  std::vector<ErrorStateMeasurement> measurements;
  measurements.reserve(gravity_aligned_points.size() + 1);
  int lidar_residuals = 0;
  const auto& iterated = options_.iterated_update();
  if (grid != nullptr) {
    for (const auto& point : gravity_aligned_points) {
      if (!point.position.allFinite()) continue;
      ErrorStateMeasurement measurement;
      measurement.cost = absl::make_unique<PlanarLidarResidual>(
          point, *grid, nominal, iterated.grid_residual_stddev());
      measurement.huber_scale = iterated.huber_scale();
      measurements.push_back(std::move(measurement));
      ++lidar_residuals;
    }
  }
  if (options_.use_nhc()) {
    ErrorStateMeasurement measurement;
    measurement.cost.reset(
        new ceres::AutoDiffCostFunction<BodyLateralVelocityResidual, 1, 8>(
            new BodyLateralVelocityResidual(nominal, options_.nhc_velocity_stddev())));
    measurements.push_back(std::move(measurement));
  }
  PlanarErrorVector delta;
  PlanarErrorCovariance posterior;
  result = SolveIteratedErrorStateUpdate(state_.covariance, iterated,
                                        std::move(measurements), &delta, &posterior);
  if (result.accepted) {
    state_.position += delta.head<2>();
    state_.velocity += delta.segment<2>(2);
    state_.yaw = common::NormalizeAngleDifference(state_.yaw + delta(4));
    state_.gyro_bias += delta(5);
    state_.accel_bias += delta.tail<2>();
    state_.covariance = posterior;
    ++num_tight_updates_;
  } else {
    ++num_tight_rejections_;
    LOG_EVERY_N(WARNING, 100)
        << "Planar IESKF rejected lidar update; retaining inertial prediction.";
  }
  num_tight_iterations_ += result.iterations;
  num_tight_lidar_residuals_ += lidar_residuals;
  LOG_EVERY_N(INFO, 200) << "Planar IESKF update: accepted=" << result.accepted
      << " iterations=" << result.iterations << " lidar_points=" << lidar_residuals
      << " cost=" << result.initial_cost << " -> " << result.final_cost;
  FinishUpdate();
  return result;
}

void PlanarImuPoseExtrapolator::UpdateWithPose(
    common::Time time, const transform::Rigid3d& pose,
    const Eigen::Matrix3d* information, const Eigen::Vector3d* gradient) {
  if (state_.time == common::Time::min()) {
    state_.time = time;
    state_.position = pose.translation().head<2>();
    state_.yaw = transform::GetYaw(pose.rotation());
    state_.tilt = WithoutYaw(pose.rotation());
    return;
  }
  BuildTrajectory(time);
  state_ = predicted_;
  gravity_tracker_ = absl::make_unique<ImuTracker>(*predicted_gravity_tracker_);
  Eigen::Vector3d innovation;
  innovation.head<2>() = pose.translation().head<2>() - state_.position;
  innovation.z() = common::NormalizeAngleDifference(
      transform::GetYaw(pose.rotation()) - state_.yaw);
  Eigen::Matrix<double, 3, 8> jacobian = Eigen::Matrix<double, 3, 8>::Zero();
  jacobian.block<2, 2>(0, 0).setIdentity();
  jacobian(2, 4) = 1.;
  Eigen::Matrix3d noise = Eigen::Matrix3d::Zero();
  noise.diagonal() << Square(options_.scan_translation_stddev()),
      Square(options_.scan_translation_stddev()), Square(options_.scan_rotation_stddev());
  if (information == nullptr) {
    Correct(innovation, jacobian, noise, trajectory_healthy_);
  } else if (information->allFinite() && gradient->allFinite()) {
    // Work in pose-noise-normalized coordinates so meters and radians are
    // comparable. Preserve weak/absent geometric directions as unobserved.
    const Eigen::Matrix3d scale = noise.diagonal().cwiseSqrt().asDiagonal();
    const Eigen::Matrix3d normalized = scale * (*information) * scale;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(normalized);
    const double maximum = eigen.eigenvalues().maxCoeff();
    if (eigen.info() == Eigen::Success && maximum > 1e-9) {
      std::vector<int> observed;
      for (int i = 0; i < 3; ++i) {
        if (eigen.eigenvalues()(i) > 1e-3 * maximum) observed.push_back(i);
      }
      Eigen::MatrixXd projected(observed.size(), 8);
      Eigen::VectorXd residual(observed.size());
      Eigen::MatrixXd projected_noise = Eigen::MatrixXd::Zero(observed.size(), observed.size());
      for (size_t row = 0; row < observed.size(); ++row) {
        const int axis = observed[row];
        const Eigen::Vector3d direction = eigen.eigenvectors().col(axis);
        projected.row(row) = direction.transpose() * scale.inverse() * jacobian;
        // Remove the scan matcher's motion-prior contribution using the pure
        // lidar gradient, rather than treating its MAP pose as independent.
        residual(row) = direction.dot(scale.inverse() * innovation) -
            direction.dot(scale * (*gradient)) / eigen.eigenvalues()(axis);
        projected_noise(row, row) = maximum / eigen.eigenvalues()(axis);
      }
      Correct(residual, projected, projected_noise, trajectory_healthy_);
    }
  }
  if (options_.use_nhc()) {
    const Eigen::Vector2d lateral(-std::sin(state_.yaw), std::cos(state_.yaw));
    const Eigen::Vector2d forward(std::cos(state_.yaw), std::sin(state_.yaw));
    Eigen::Matrix<double, 1, 8> nhc_jacobian = Eigen::Matrix<double, 1, 8>::Zero();
    nhc_jacobian.block<1, 2>(0, 2) = lateral.transpose();
    nhc_jacobian(0, 4) = -forward.dot(state_.velocity);
    Correct(Eigen::VectorXd::Constant(1, -lateral.dot(state_.velocity)),
            nhc_jacobian,
            Eigen::MatrixXd::Constant(1, 1, Square(options_.nhc_velocity_stddev())),
            trajectory_healthy_);
  }
  FinishUpdate();
}

void PlanarImuPoseExtrapolator::FinishUpdate() {
  ++num_corrections_;
  if (!trajectory_healthy_) ++num_fallbacks_;
  LOG_EVERY_N(INFO, 200) << "Planar IMU feedback: frame=" << num_corrections_
      << " velocity_xy=" << state_.velocity.transpose()
      << " gyro_bias_z=" << state_.gyro_bias
      << " accel_bias_xy=" << state_.accel_bias.transpose();
  trajectory_valid_ = false;
  last_extrapolated_time_ = state_.time;
  TrimImuData();
}

void PlanarImuPoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && imu_data_[1].time <= state_.time) {
    imu_data_.pop_front();
  }
  // Bound memory if scans stop; propagation across pruned data uses the gap guard.
  while (imu_data_.size() > 10000) imu_data_.pop_front();
}

}  // namespace mapping
}  // namespace cartographer
