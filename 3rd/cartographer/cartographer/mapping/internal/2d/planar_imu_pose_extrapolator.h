// Copyright 2026 The Cartographer Authors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_PLANAR_IMU_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_PLANAR_IMU_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>
#include <vector>

#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/iterated_error_state_update.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"

namespace cartographer {
namespace mapping {

// Planar inertial navigation with SO(3) gravity alignment for range deskewing.
// State/error ordering: px, py, vx, vy, yaw, gyro_bias_z, accel_bias_x/y.
// No height, vertical velocity, or independently observable tilt is assumed.
class PlanarImuPoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  PlanarImuPoseExtrapolator(const proto::PlanarImuDeskewOptions& options,
                            double gravity_time_constant);
  ~PlanarImuPoseExtrapolator() override;

  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;
  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  // The quadratic is from lidar residuals only, excluding matching priors.
  void AddPoseWithInformation(common::Time time, const transform::Rigid3d& pose,
                              const Eigen::Matrix3d& information,
                              const Eigen::Vector3d& gradient);
  // Direct raw lidar likelihood + frozen IMU prior + optional joint NHC.
  // A null map seeds the first submap from the inertial prediction.
  IteratedErrorStateUpdateSummary UpdateWithLidar(
      common::Time time, const sensor::PointCloud& gravity_aligned_points,
      const Grid2D* grid);
  // Legacy update mode publishes the original scan-match pose; retain learned
  // motion/biases but align the next deskew trajectory to that same pose.
  void AnchorToScanPose(common::Time time, const transform::Rigid3d& pose);
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;
  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

  Eigen::Vector2d velocity() const { return state_.velocity; }
  Eigen::Vector2d acceleration_bias() const { return state_.accel_bias; }
  double gyro_bias() const { return state_.gyro_bias; }
  Eigen::Matrix<double, 8, 8> covariance() const { return state_.covariance; }
  bool last_prediction_uses_acceleration() const { return trajectory_healthy_; }

 private:
  using Covariance = Eigen::Matrix<double, 8, 8>;
  struct State {
    common::Time time = common::Time::min();
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double yaw = 0.;
    double gyro_bias = 0.;
    Eigen::Vector2d accel_bias = Eigen::Vector2d::Zero();
    Eigen::Quaterniond tilt = Eigen::Quaterniond::Identity();
    Covariance covariance = Covariance::Identity();
  };
  struct Segment {
    State start;
    State end;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector2d acceleration;
  };

  void BuildTrajectory(common::Time end_time);
  sensor::ImuData MeasurementAt(common::Time time) const;
  bool MeasurementSupported(common::Time time) const;
  void UpdateWithPose(common::Time time, const transform::Rigid3d& pose,
                      const Eigen::Matrix3d* information,
                      const Eigen::Vector3d* gradient);
  transform::Rigid3d PoseAt(common::Time time, size_t* segment_index) const;
  static transform::Rigid3d ToPose(const State& state);
  void Correct(const Eigen::VectorXd& innovation,
               const Eigen::MatrixXd& jacobian,
               const Eigen::MatrixXd& noise, bool update_biases);
  void TrimImuData();
  void FinishUpdate();

  const proto::PlanarImuDeskewOptions options_;
  const double gravity_time_constant_;
  State state_;
  State predicted_;
  std::unique_ptr<ImuTracker> gravity_tracker_;
  std::unique_ptr<ImuTracker> predicted_gravity_tracker_;
  std::deque<sensor::ImuData> imu_data_;
  std::vector<Segment> segments_;
  bool trajectory_valid_ = false;
  bool trajectory_healthy_ = true;
  common::Time last_extrapolated_time_ = common::Time::min();
  int num_corrections_ = 0;
  int num_fallbacks_ = 0;
  int num_tight_updates_ = 0;
  int num_tight_rejections_ = 0;
  int num_tight_iterations_ = 0;
  int num_tight_lidar_residuals_ = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_PLANAR_IMU_POSE_EXTRAPOLATOR_H_
