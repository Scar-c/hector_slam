/*
 * Copyright 2026 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/planar_imu_pose_extrapolator.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr double kGravity = 9.81;
constexpr double kImuPeriod = 0.005;  // 200 Hz.
constexpr double kPi = 3.14159265358979323846;
// Isolate inertial integration/bias observability from the accelerometer's
// inherently ambiguous distinction between sustained acceleration and tilt.
constexpr double kInertialTestGravityTimeConstant = 1e6;

common::Time Time(double seconds) {
  return common::FromUniversal(123456789) + common::FromSeconds(seconds);
}

Eigen::Quaterniond Yaw(double angle) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

transform::Rigid3d PlanarPose(const Eigen::Vector2d& position, double yaw) {
  return transform::Rigid3d(Eigen::Vector3d(position.x(), position.y(), 0.),
                            Yaw(yaw));
}

proto::PlanarImuDeskewOptions TestOptions(bool use_nhc = false) {
  proto::PlanarImuDeskewOptions options;
  options.set_enabled(true);
  options.set_gyro_noise(0.01);
  options.set_acceleration_noise(0.1);
  options.set_gyro_bias_noise(0.0001);
  options.set_acceleration_bias_noise(0.001);
  options.set_scan_translation_stddev(0.01);
  options.set_scan_rotation_stddev(0.005);
  options.set_use_nhc(use_nhc);
  options.set_nhc_velocity_stddev(0.05);
  options.set_max_imu_gap_sec(0.05);
  options.set_max_extrapolation_sec(0.02);
  return options;
}

void Seed(PlanarImuPoseExtrapolator* extrapolator, double yaw = 0.) {
  extrapolator->AddPose(Time(0.), PlanarPose(Eigen::Vector2d::Zero(), yaw));
  extrapolator->AddImuData(sensor::ImuData{
      Time(0.), Eigen::Vector3d(0., 0., kGravity), Eigen::Vector3d::Zero()});
}

void ExpectFinite(const transform::Rigid3d& pose) {
  EXPECT_TRUE(pose.translation().allFinite());
  EXPECT_TRUE(pose.rotation().coeffs().allFinite());
  EXPECT_NEAR(1., pose.rotation().norm(), 1e-10);
}

// Independent world-frame truth: p(t) = a*t^2/2, yaw(t) = omega*t.
// No filter recurrence or production integration helper is used as an oracle.
class AnalyticalDeskewTest : public ::testing::TestWithParam<int> {};

TEST_P(AnalyticalDeskewTest, ResolvesPointTimesAndDeskewsStraightWall) {
  const Eigen::Vector2d acceleration =
      GetParam() == 0 ? Eigen::Vector2d::Zero() : Eigen::Vector2d(4., -1.5);
  const double angular_velocity = GetParam() == 1 ? 0. : 1.3;
  PlanarImuPoseExtrapolator extrapolator(
      TestOptions(), kInertialTestGravityTimeConstant);
  Seed(&extrapolator);
  // The first stationary observation establishes gravity before motion starts.
  // Replacing the t=0 measurement models an acceleration step without asking
  // the startup gravity estimator to infer tilt from that acceleration.
  for (int i = 0; i <= 22; ++i) {
    const double t = i * kImuPeriod;
    const Eigen::Vector3d specific_force =
        Yaw(angular_velocity * t).conjugate() *
        Eigen::Vector3d(acceleration.x(), acceleration.y(), kGravity);
    extrapolator.AddImuData(sensor::ImuData{
        Time(t), specific_force, Eigen::Vector3d(0., 0., angular_velocity)});
  }

  std::vector<common::Time> times;
  // 27.5 us is exactly 275 of Cartographer's 100 ns clock ticks. Integer
  // timestamps avoid floating-point rounding or accidental IMU-rate queries.
  for (int i = 0; i <= 4000; ++i) {
    times.push_back(Time(0.) + common::Duration(275 * i));
  }
  ASSERT_EQ(common::Duration(275), times[1] - times[0]);
  const auto result = extrapolator.ExtrapolatePosesWithGravity(times);
  ASSERT_EQ(times.size() - 1, result.previous_poses.size());
  const double end = common::ToSeconds(times.back() - Time(0.));
  const auto truth = [&](double t) {
    return PlanarPose(0.5 * acceleration * t * t, angular_velocity * t);
  };
  EXPECT_THAT(result.current_pose, transform::IsNearly(truth(end), 2e-5));
  EXPECT_NEAR(
      0., (result.current_velocity.head<2>() - acceleration * end).norm(), 1e-4);
  EXPECT_EQ(times.back(), extrapolator.GetLastExtrapolatedTime());
  EXPECT_EQ(Time(0.), extrapolator.GetLastPoseTime());
  EXPECT_TRUE(extrapolator.last_prediction_uses_acceleration());

  const auto end_from_world = truth(end).inverse();
  const Eigen::Vector3d normal =
      end_from_world.rotation() * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d wall_origin =
      end_from_world * Eigen::Vector3d(8., 0., 0.);
  double corrected_squared_error = 0.;
  double raw_squared_error = 0.;
  double max_corrected_error = 0.;
  for (size_t i = 0; i < times.size(); ++i) {
    SCOPED_TRACE(i);
    const double t = common::ToSeconds(times[i] - Time(0.));
    const auto expected_pose = truth(t);
    const transform::Rigid3d estimated_pose =
        i < result.previous_poses.size()
            ? result.previous_poses[i].cast<double>()
            : result.current_pose;
    EXPECT_THAT(estimated_pose, transform::IsNearly(expected_pose, 2e-5));
    const Eigen::Vector3d world_point(8., -4. + 8. * i / 4000., 0.);
    const Eigen::Vector3d raw_point = expected_pose.inverse() * world_point;
    const Eigen::Vector3d corrected_point =
        result.current_pose.inverse() * (estimated_pose * raw_point);
    const double corrected_error = normal.dot(corrected_point - wall_origin);
    const double raw_error = normal.dot(raw_point - wall_origin);
    corrected_squared_error += corrected_error * corrected_error;
    raw_squared_error += raw_error * raw_error;
    max_corrected_error =
        std::max(max_corrected_error, std::abs(corrected_error));
  }
  const double corrected_rms =
      std::sqrt(corrected_squared_error / times.size());
  const double raw_rms = std::sqrt(raw_squared_error / times.size());
  EXPECT_GT(raw_rms, 0.005);  // Ensure the scene actually reveals distortion.
  EXPECT_LT(corrected_rms, 0.001);  // One millimeter point-to-wall error.
  EXPECT_LT(max_corrected_error, 0.002);
  EXPECT_LT(corrected_rms, 0.02 * raw_rms);
  // A cached frame and a singleton query must preserve the batch endpoint.
  EXPECT_THAT(extrapolator.ExtrapolatePose(times.back()),
              transform::IsNearly(result.current_pose, 1e-10));
}

INSTANTIATE_TEST_CASE_P(RotationAccelerationAndCombined, AnalyticalDeskewTest,
                        ::testing::Values(0, 1, 2));

TEST(PlanarImuPoseExtrapolatorTest, AlignsRollPitchGravityWithoutLeakingYaw) {
  for (const double sign : {-1., 1.}) {
    SCOPED_TRACE(sign);
    const Eigen::Quaterniond tilt =
        Eigen::AngleAxisd(sign * 0.23, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-sign * 0.31, Eigen::Vector3d::UnitX());
    const Eigen::Quaterniond orientation = Yaw(1.1) * tilt;
    const Eigen::Vector3d measured_gravity =
        orientation.conjugate() * Eigen::Vector3d(0., 0., kGravity);
    PlanarImuPoseExtrapolator extrapolator(TestOptions(), 0.1);
    // A planar laser pose supplies yaw; the IMU must supply both tilt axes.
    extrapolator.AddPose(Time(0.), PlanarPose(Eigen::Vector2d::Zero(), 1.1));
    for (int i = 0; i <= 100; ++i) {
      extrapolator.AddImuData(sensor::ImuData{
          Time(i * kImuPeriod), measured_gravity, Eigen::Vector3d::Zero()});
    }
    const auto result = extrapolator.ExtrapolatePosesWithGravity({Time(0.5)});
    EXPECT_TRUE(result.previous_poses.empty());
    EXPECT_NEAR(0., result.gravity_from_tracking.angularDistance(tilt), 1e-8);
    EXPECT_NEAR(0., transform::GetYaw(result.gravity_from_tracking), 1e-8);
    EXPECT_NEAR(0., (result.gravity_from_tracking * measured_gravity -
                     Eigen::Vector3d(0., 0., kGravity)).norm(), 1e-8);
    EXPECT_NEAR(0., result.current_pose.rotation().angularDistance(orientation),
                1e-8);
    EXPECT_NEAR(0., result.current_pose.translation().norm(), 1e-8);
    EXPECT_NEAR(0., result.current_velocity.norm(), 1e-8);
    EXPECT_NEAR(0., extrapolator.EstimateGravityOrientation(Time(0.5))
                        .angularDistance(tilt), 1e-8);
  }
}

TEST(PlanarImuPoseExtrapolatorTest, CrossesBothYawBranchCutsContinuously) {
  for (const double sign : {-1., 1.}) {
    SCOPED_TRACE(sign);
    const double start_yaw = sign * (kPi - 0.03);
    const double rate = sign * 0.8;
    PlanarImuPoseExtrapolator extrapolator(TestOptions(), 10.);
    Seed(&extrapolator, start_yaw);
    for (int i = 0; i <= 20; ++i) {
      extrapolator.AddImuData(sensor::ImuData{
          Time(i * kImuPeriod), Eigen::Vector3d(0., 0., kGravity),
          Eigen::Vector3d(0., 0., rate)});
    }
    const auto result = extrapolator.ExtrapolatePosesWithGravity(
        {Time(0.03), Time(0.04), Time(0.1)});
    ASSERT_EQ(2, result.previous_poses.size());
    EXPECT_NEAR(0., result.previous_poses[0].rotation().cast<double>()
                        .angularDistance(Yaw(start_yaw + rate * 0.03)), 1e-6);
    EXPECT_NEAR(0., result.previous_poses[1].rotation().cast<double>()
                        .angularDistance(Yaw(start_yaw + rate * 0.04)), 1e-6);
    EXPECT_NEAR(0.008, result.previous_poses[0].rotation().angularDistance(
                           result.previous_poses[1].rotation()), 1e-6);
    EXPECT_NEAR(0., result.current_pose.rotation().angularDistance(
                       Yaw(start_yaw + rate * 0.1)), 1e-10);
    // The laser quaternion uses the opposite signed representation of yaw.
    // Its physically zero innovation must not create a 2*pi bias correction.
    extrapolator.AddPose(
        Time(0.1),
        PlanarPose(Eigen::Vector2d::Zero(), start_yaw + rate * 0.1));
    EXPECT_NEAR(0., extrapolator.gyro_bias(), 1e-8);
    EXPECT_THAT(extrapolator.ExtrapolatePose(Time(0.1)),
                transform::IsNearly(result.current_pose, 1e-8));
  }
}

TEST(PlanarImuPoseExtrapolatorTest,
     LaserFeedbackLearnsVelocityAndObservableBiases) {
  PlanarImuPoseExtrapolator corrected(TestOptions(),
                                      kInertialTestGravityTimeConstant);
  PlanarImuPoseExtrapolator unaided(TestOptions(),
                                    kInertialTestGravityTimeConstant);
  const double yaw = 0.6;
  const Eigen::Vector2d velocity(0.9, -0.35);
  const Eigen::Vector2d acceleration_bias(0.12, -0.08);
  constexpr double kGyroBias = 0.025;
  Seed(&corrected, yaw);
  Seed(&unaided, yaw);
  // Full SE(2) scan poses observe heading, making z gyro bias observable.
  // A featureless infinite wall alone would not provide these constraints.
  for (int i = 0; i <= 4000; ++i) {
    const double t = i * kImuPeriod;
    const sensor::ImuData data{
        Time(t), Eigen::Vector3d(acceleration_bias.x(), acceleration_bias.y(),
                                kGravity), Eigen::Vector3d(0., 0., kGyroBias)};
    corrected.AddImuData(data);
    unaided.AddImuData(data);
    if (i > 0 && i % 10 == 0) {
      corrected.AddPose(Time(t), PlanarPose(velocity * t, yaw));
    }
  }
  EXPECT_NEAR(0., (corrected.velocity() - velocity).norm(), 0.03);
  EXPECT_NEAR(0., (corrected.acceleration_bias() - acceleration_bias).norm(),
              0.025);
  EXPECT_NEAR(kGyroBias, corrected.gyro_bias(), 0.003);
  const auto expected = PlanarPose(velocity * 20., yaw);
  const auto corrected_pose = corrected.ExtrapolatePose(Time(20.));
  const auto unaided_pose = unaided.ExtrapolatePose(Time(20.));
  // Absolute meters/radians, independent of the distance from the world origin.
  EXPECT_LT((corrected_pose.translation() - expected.translation()).norm(), 0.01);
  EXPECT_LT(corrected_pose.rotation().angularDistance(expected.rotation()),
            0.005);
  EXPECT_GT((unaided_pose.translation() - expected.translation()).norm(), 1.);
  EXPECT_GT(unaided_pose.rotation().angularDistance(expected.rotation()), 0.2);
  // Also check prediction between scans, where pinning output to laser poses
  // cannot hide a wrong velocity or a failure to learn inertial biases.
  for (int i = 1; i <= 4; ++i) {
    corrected.AddImuData(sensor::ImuData{
        Time(20. + i * kImuPeriod),
        Eigen::Vector3d(acceleration_bias.x(), acceleration_bias.y(), kGravity),
        Eigen::Vector3d(0., 0., kGyroBias)});
  }
  const auto prediction = corrected.ExtrapolatePose(Time(20.02));
  const auto future_truth = PlanarPose(velocity * 20.02, yaw);
  EXPECT_LT((prediction.translation() - future_truth.translation()).norm(), 0.01);
  EXPECT_LT(prediction.rotation().angularDistance(future_truth.rotation()),
            0.005);
}

TEST(PlanarImuPoseExtrapolatorTest,
     NhcConstrainsBodyLateralVelocityAtAnyHeading) {
  for (const double yaw : {0., kPi / 2., -1.1}) {
    SCOPED_TRACE(yaw);
    PlanarImuPoseExtrapolator extrapolator(
        TestOptions(true), kInertialTestGravityTimeConstant);
    Seed(&extrapolator, yaw);
    const Eigen::Vector2d forward(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d lateral(-std::sin(yaw), std::cos(yaw));
    for (int i = 0; i <= 1200; ++i) {
      const double t = i * kImuPeriod;
      extrapolator.AddImuData(sensor::ImuData{
          Time(t), Eigen::Vector3d(0., 0.2, kGravity),
          Eigen::Vector3d::Zero()});
      if (i > 0 && i % 10 == 0) {
        extrapolator.AddPose(Time(t), PlanarPose(1.2 * t * forward, yaw));
      }
    }
    EXPECT_NEAR(1.2, forward.dot(extrapolator.velocity()), 0.03);
    EXPECT_NEAR(0., lateral.dot(extrapolator.velocity()), 0.03);
    if (yaw == kPi / 2.) {
      EXPECT_GT(extrapolator.velocity().y(), 1.1);
      EXPECT_NEAR(0., extrapolator.velocity().x(), 0.03);
    }
  }
}

TEST(PlanarImuPoseExtrapolatorTest,
     MissingImuUsesFiniteConstantVelocityAndFreezesBiases) {
  // Cover an internal missing interval, and stale extrapolation beyond the
  // 20 ms horizon even though the interval is shorter than max_imu_gap_sec.
  for (const bool internal_gap : {false, true}) {
    SCOPED_TRACE(internal_gap);
    PlanarImuPoseExtrapolator extrapolator(
        TestOptions(), kInertialTestGravityTimeConstant);
    Seed(&extrapolator);
    for (int i = 1; i <= 400; ++i) {
      const double t = i * kImuPeriod;
      extrapolator.AddImuData(sensor::ImuData{
          Time(t), Eigen::Vector3d(0., 0., kGravity), Eigen::Vector3d::Zero()});
      if (i % 10 == 0) {
        extrapolator.AddPose(Time(t), PlanarPose(Eigen::Vector2d(t, 0.), 0.));
      }
    }
    const auto start = extrapolator.ExtrapolatePose(Time(2.));
    const Eigen::Vector2d velocity = extrapolator.velocity();
    ASSERT_GT(velocity.x(), 0.9);
    const Eigen::Vector2d acceleration_bias = extrapolator.acceleration_bias();
    const double gyro_bias = extrapolator.gyro_bias();
    const double gap = internal_gap ? 0.2 : 0.025;
    // A large but finite force makes silently integrating across the gap
    // detectable. A duplicate sample changes force only after gravity startup.
    extrapolator.AddImuData(sensor::ImuData{
        Time(2.), Eigen::Vector3d(8., -3., kGravity), Eigen::Vector3d::Zero()});
    if (internal_gap) {
      extrapolator.AddImuData(sensor::ImuData{
          Time(2. + gap), Eigen::Vector3d(8., -3., kGravity),
          Eigen::Vector3d::Zero()});
    }
    const auto result =
        extrapolator.ExtrapolatePosesWithGravity({Time(2. + gap)});
    EXPECT_FALSE(extrapolator.last_prediction_uses_acceleration());
    ExpectFinite(result.current_pose);
    EXPECT_TRUE(result.current_velocity.allFinite());
    EXPECT_NEAR(0.,
                (result.current_pose.translation().head<2>() -
                 start.translation().head<2>() - velocity * gap).norm(),
                1e-10);
    EXPECT_NEAR(0., (result.current_velocity.head<2>() - velocity).norm(),
                1e-10);
    // Nonzero laser innovations must not be interpreted as IMU bias evidence
    // when the interval was missing. Position/velocity corrections may occur.
    extrapolator.AddPose(
        Time(2. + gap),
        PlanarPose(result.current_pose.translation().head<2>() +
                       Eigen::Vector2d(0.15, -0.1), 0.08));
    EXPECT_NEAR(gyro_bias, extrapolator.gyro_bias(), 1e-12);
    EXPECT_NEAR(
        0., (extrapolator.acceleration_bias() - acceleration_bias).norm(), 1e-12);
    ExpectFinite(extrapolator.ExtrapolatePose(Time(2. + gap)));
  }
}

TEST(PlanarImuPoseExtrapolatorTest,
     AcceptsShortExtrapolationWithinConfiguredHorizon) {
  PlanarImuPoseExtrapolator extrapolator(
      TestOptions(), kInertialTestGravityTimeConstant);
  Seed(&extrapolator);
  extrapolator.AddImuData(sensor::ImuData{
      Time(0.), Eigen::Vector3d(2., 0., kGravity), Eigen::Vector3d::Zero()});
  const auto result = extrapolator.ExtrapolatePosesWithGravity({Time(0.015)});
  EXPECT_TRUE(extrapolator.last_prediction_uses_acceleration());
  EXPECT_NEAR(0.5 * 2. * 0.015 * 0.015,
              result.current_pose.translation().x(), 1e-8);
  EXPECT_NEAR(2. * 0.015, result.current_velocity.x(), 1e-7);
}

TEST(PlanarImuPoseExtrapolatorTest,
     SupportingSampleGapRejectsShortQueriesInsideHole) {
  PlanarImuPoseExtrapolator extrapolator(
      TestOptions(), kInertialTestGravityTimeConstant);
  Seed(&extrapolator);
  // A 20 ms query must not disguise the 200 ms supporting-sample hole.
  for (const double t : {0., 0.2}) {
    extrapolator.AddImuData(sensor::ImuData{
        Time(t), Eigen::Vector3d(8., -3., kGravity),
        Eigen::Vector3d(0., 0., 0.3)});
  }
  const auto prediction = extrapolator.ExtrapolatePosesWithGravity(
      {Time(0.005), Time(0.015), Time(0.02)});
  ASSERT_EQ(2, prediction.previous_poses.size());
  EXPECT_FALSE(extrapolator.last_prediction_uses_acceleration());
  for (const auto& pose : prediction.previous_poses) {
    EXPECT_NEAR(0., pose.translation().norm(), 1e-12);
  }
  EXPECT_NEAR(0., prediction.current_pose.translation().norm(), 1e-12);
  EXPECT_NEAR(0., prediction.current_velocity.norm(), 1e-12);
  EXPECT_NEAR(
      0., prediction.current_pose.rotation().angularDistance(Yaw(0.006)),
      1e-10);
  ExpectFinite(prediction.current_pose);
}

TEST(PlanarImuPoseExtrapolatorTest,
     RepeatedLaserFeedbackCannotTurnSupportingSampleHoleHealthy) {
  PlanarImuPoseExtrapolator extrapolator(
      TestOptions(), kInertialTestGravityTimeConstant);
  Seed(&extrapolator);
  for (const double t : {0., 0.2}) {
    extrapolator.AddImuData(sensor::ImuData{
        Time(t), Eigen::Vector3d(8., -3., kGravity),
        Eigen::Vector3d::Zero()});
  }
  auto previous_pose = extrapolator.ExtrapolatePose(Time(0.));
  auto previous_time = Time(0.);
  for (int frame = 1; frame <= 9; ++frame) {
    SCOPED_TRACE(frame);
    const auto time = Time(0.02 * frame);
    const Eigen::Vector2d previous_velocity = extrapolator.velocity();
    const Eigen::Vector2d previous_bias = extrapolator.acceleration_bias();
    const double previous_gyro_bias = extrapolator.gyro_bias();
    const double dt = common::ToSeconds(time - previous_time);
    const auto prediction = extrapolator.ExtrapolatePosesWithGravity({time});
    EXPECT_FALSE(extrapolator.last_prediction_uses_acceleration());
    EXPECT_NEAR(0., (prediction.current_pose.translation().head<2>() -
                     previous_pose.translation().head<2>() -
                     dt * previous_velocity).norm(), 1e-10);
    EXPECT_NEAR(0.,
                (prediction.current_velocity.head<2>() - previous_velocity)
                    .norm(),
                1e-10);
    // Nonzero innovations expose incorrectly enabled bias learning.
    extrapolator.AddPose(
        time, PlanarPose(prediction.current_pose.translation().head<2>() +
                             Eigen::Vector2d(0.1, -0.07),
                         transform::GetYaw(prediction.current_pose.rotation()) +
                             0.03));
    EXPECT_FALSE(extrapolator.last_prediction_uses_acceleration());
    EXPECT_NEAR(previous_gyro_bias, extrapolator.gyro_bias(), 1e-12);
    EXPECT_NEAR(0., (extrapolator.acceleration_bias() - previous_bias).norm(),
                1e-12);
    previous_pose = extrapolator.ExtrapolatePose(time);
    previous_time = time;
    ExpectFinite(previous_pose);
  }
}

TEST(PlanarImuPoseExtrapolatorTest,
     FallbackHeadingUncertaintyIncludesIntegratedGyroBias) {
  const auto options = TestOptions();
  PlanarImuPoseExtrapolator extrapolator(
      options, kInertialTestGravityTimeConstant);
  Seed(&extrapolator);
  extrapolator.AddImuData(sensor::ImuData{
      Time(0.), Eigen::Vector3d(0., 0., kGravity),
      Eigen::Vector3d(0., 0., 0.3)});
  const auto initial_covariance = extrapolator.covariance();
  constexpr double kGap = 0.2;
  const auto predicted_pose = extrapolator.ExtrapolatePose(Time(kGap));
  ASSERT_FALSE(extrapolator.last_prediction_uses_acceleration());
  EXPECT_NEAR(0., predicted_pose.rotation().angularDistance(Yaw(0.3 * kGap)),
              1e-10);
  // Commit propagation without adding a scan measurement or NHC information.
  extrapolator.AddPoseWithInformation(Time(kGap), predicted_pose,
                                     Eigen::Matrix3d::Zero(),
                                     Eigen::Vector3d::Zero());
  const auto covariance = extrapolator.covariance();
  // theta(t) = theta(0) + (omega-b)*t: uncertainty in constant b contributes
  // t^2*Var(b) and Cov(theta,b) = -t*Var(b), even with missing measurements.
  const double gyro_noise_variance =
      options.gyro_noise() * options.gyro_noise();
  const double fallback_white_noise = 100. * gyro_noise_variance * kGap;
  const double bias_heading_variance =
      kGap * kGap * initial_covariance(5, 5);
  EXPECT_GE(covariance(4, 4) - initial_covariance(4, 4) - fallback_white_noise,
            bias_heading_variance - 1e-12);
  EXPECT_NEAR(-kGap * initial_covariance(5, 5), covariance(4, 5), 1e-9);
  EXPECT_LT(covariance(4, 5), -1e-4);
  EXPECT_NEAR(covariance(4, 5), covariance(5, 4), 1e-12);
}

TEST(PlanarImuPoseExtrapolatorTest,
     DynamicTiltMatchesAnalyticalSo3AtKnotsAndInteriorTimes) {
  // Cover 200 Hz data and an interval at the allowed gap limit, where the old
  // split yaw/tilt interpolation clearly exceeds a 1 urad error budget.
  for (const double period : {kImuPeriod, 0.05}) {
    SCOPED_TRACE(period);
    const auto interval =
        common::FromMilliseconds(std::lround(1000. * period));
    PlanarImuPoseExtrapolator extrapolator(
        TestOptions(), kInertialTestGravityTimeConstant);
    const Eigen::Quaterniond initial_rotation =
        Yaw(0.7) * Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
    const Eigen::Vector3d body_rates(1., 2., 0.5);
    const Eigen::Vector3d startup_gravity =
        initial_rotation.conjugate() * Eigen::Vector3d(0., 0., kGravity);
    const transform::Rigid3d initial_pose(Eigen::Vector3d::Zero(),
                                          initial_rotation);
    extrapolator.AddPose(Time(0.), initial_pose);
    extrapolator.AddImuData(sensor::ImuData{
        Time(0.), startup_gravity, Eigen::Vector3d::Zero()});
    const int num_intervals = static_cast<int>(std::lround(0.2 / period));
    for (int i = 0; i <= num_intervals; ++i) {
      // A fixed specific force isolates gyro integration with slow gravity
      // correction. It is fictitious during rotation: translation has no
      // stationary physical truth here and is deliberately unchecked.
      extrapolator.AddImuData(sensor::ImuData{
          Time(0.) + i * interval, startup_gravity, body_rates});
    }
    std::vector<common::Time> times;
    for (int i = 0; i < num_intervals; ++i) {
      for (const double fraction : {0., 0.23, 0.5, 0.91}) {
        times.push_back(Time(0.) + i * interval +
                        common::FromSeconds(fraction * period));
      }
    }
    times.push_back(Time(0.2));
    const auto result = extrapolator.ExtrapolatePosesWithGravity(times);
    ASSERT_EQ(times.size() - 1, result.previous_poses.size());
    ASSERT_TRUE(extrapolator.last_prediction_uses_acceleration());
    for (size_t i = 0; i < times.size(); ++i) {
      SCOPED_TRACE(i);
      const double t = common::ToSeconds(times[i] - Time(0.));
      // Closed-form right multiplication for constant BODY angular velocity.
      const Eigen::Quaterniond truth =
          initial_rotation * Eigen::AngleAxisd(body_rates.norm() * t,
                                              body_rates.normalized());
      const Eigen::Quaterniond rotation =
          i < result.previous_poses.size()
              ? result.previous_poses[i].rotation().cast<double>()
              : result.current_pose.rotation();
      EXPECT_LT(rotation.angularDistance(truth), 1e-6);
    }
    EXPECT_NEAR(0., extrapolator.gyro_bias(), 1e-12);
    EXPECT_NEAR(0., extrapolator.acceleration_bias().norm(), 1e-12);
  }
}

TEST(PlanarImuPoseExtrapolatorTest,
     ZeroLidarInformationPreservesFreePredictionAndCovariance) {
  const auto options = TestOptions(false);
  PlanarImuPoseExtrapolator extrapolator(
      options, kInertialTestGravityTimeConstant);
  Seed(&extrapolator);
  const auto initial_covariance = extrapolator.covariance();
  constexpr double kDt = 0.001;
  const auto prediction = extrapolator.ExtrapolatePosesWithGravity({Time(kDt)});
  // An arbitrary matcher pose is meaningless when lidar has no information.
  extrapolator.AddPoseWithInformation(
      Time(kDt), PlanarPose(Eigen::Vector2d(10., -20.), 0.8),
      Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_THAT(extrapolator.ExtrapolatePose(Time(kDt)),
              transform::IsNearly(prediction.current_pose, 1e-12));
  EXPECT_NEAR(0., (extrapolator.velocity() -
                   prediction.current_velocity.head<2>()).norm(), 1e-12);
  EXPECT_NEAR(0., extrapolator.gyro_bias(), 1e-12);
  EXPECT_NEAR(0., extrapolator.acceleration_bias().norm(), 1e-12);

  // Independent continuous-time moments for p'=v, v'=noise-b, b'=bias_noise.
  // At 1 ms, within-step bias random-walk terms omitted by a discrete
  // first-order propagator are smaller than the 1e-12 absolute tolerance.
  auto expected_covariance = initial_covariance;
  const double qa = options.acceleration_noise() * options.acceleration_noise();
  const double qb =
      options.acceleration_bias_noise() * options.acceleration_bias_noise();
  const double t2 = kDt * kDt;
  const double t3 = t2 * kDt;
  const double t4 = t3 * kDt;
  const double t5 = t4 * kDt;
  for (int axis = 0; axis < 2; ++axis) {
    const int p = axis;
    const int v = 2 + axis;
    const int b = 6 + axis;
    const double velocity_variance = initial_covariance(v, v);
    const double bias_variance = initial_covariance(b, b);
    expected_covariance(p, p) += velocity_variance * t2 +
        bias_variance * t4 / 4. + qa * t3 / 3. + qb * t5 / 20.;
    expected_covariance(v, v) +=
        bias_variance * t2 + qa * kDt + qb * t3 / 3.;
    expected_covariance(b, b) += qb * kDt;
    expected_covariance(p, v) = expected_covariance(v, p) =
        velocity_variance * kDt + bias_variance * t3 / 2. +
        qa * t2 / 2. + qb * t4 / 8.;
    expected_covariance(p, b) = expected_covariance(b, p) =
        -bias_variance * t2 / 2. - qb * t3 / 6.;
    expected_covariance(v, b) = expected_covariance(b, v) =
        -bias_variance * kDt - qb * t2 / 2.;
  }
  const double qg = options.gyro_noise() * options.gyro_noise();
  const double qbg = options.gyro_bias_noise() * options.gyro_bias_noise();
  expected_covariance(4, 4) +=
      initial_covariance(5, 5) * t2 + qg * kDt + qbg * t3 / 3.;
  expected_covariance(5, 5) += qbg * kDt;
  expected_covariance(4, 5) = expected_covariance(5, 4) =
      -initial_covariance(5, 5) * kDt - qbg * t2 / 2.;
  EXPECT_LT((extrapolator.covariance() - expected_covariance)
                .cwiseAbs().maxCoeff(), 1e-12);

  // At a committed time even process noise cannot explain any further change.
  const auto before = extrapolator.covariance();
  extrapolator.AddPoseWithInformation(
      Time(kDt), PlanarPose(Eigen::Vector2d(-30., 40.), -0.9),
      Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_LT((extrapolator.covariance() - before).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_THAT(extrapolator.ExtrapolatePose(Time(kDt)),
              transform::IsNearly(prediction.current_pose, 1e-12));

  // Also preserve an established nonzero mean, including learned biases.
  extrapolator.AddPose(Time(kDt),
                       PlanarPose(Eigen::Vector2d(0.1, -0.2), 0.05));
  const auto corrected_pose = extrapolator.ExtrapolatePose(Time(kDt));
  const Eigen::Vector2d velocity = extrapolator.velocity();
  const Eigen::Vector2d bias = extrapolator.acceleration_bias();
  const double gyro_bias = extrapolator.gyro_bias();
  const auto covariance = extrapolator.covariance();
  ASSERT_GT(velocity.norm(), 1e-3);
  ASSERT_GT(bias.norm(), 1e-8);
  ASSERT_GT(std::abs(gyro_bias), 1e-5);
  extrapolator.AddPoseWithInformation(
      Time(kDt), PlanarPose(Eigen::Vector2d(50., -60.), 1.),
      Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_THAT(extrapolator.ExtrapolatePose(Time(kDt)),
              transform::IsNearly(corrected_pose, 1e-12));
  EXPECT_NEAR(0., (extrapolator.velocity() - velocity).norm(), 1e-12);
  EXPECT_NEAR(0., (extrapolator.acceleration_bias() - bias).norm(), 1e-12);
  EXPECT_NEAR(gyro_bias, extrapolator.gyro_bias(), 1e-12);
  EXPECT_LT((extrapolator.covariance() - covariance).cwiseAbs().maxCoeff(),
            1e-15);
}

TEST(PlanarImuPoseExtrapolatorTest,
     WallInformationDoesNotConstrainAlongWallState) {
  PlanarImuPoseExtrapolator wall(TestOptions(false),
                                kInertialTestGravityTimeConstant);
  PlanarImuPoseExtrapolator unfused(TestOptions(false),
                                   kInertialTestGravityTimeConstant);
  Seed(&wall);
  Seed(&unfused);
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  information(0, 0) = 1.;
  information(2, 2) = 1.;
  // Level, stationary motion keeps y/vy/bay independent of x/yaw, so no
  // legitimate cross-correlation can constrain the unobserved direction.
  for (int i = 1; i <= 100; ++i) {
    const auto time = Time(i * kImuPeriod);
    const sensor::ImuData data{
        time, Eigen::Vector3d(0., 0., kGravity), Eigen::Vector3d::Zero()};
    wall.AddImuData(data);
    unfused.AddImuData(data);
    if (i % 10 != 0) continue;
    SCOPED_TRACE(i);
    unfused.AddPoseWithInformation(time, transform::Rigid3d::Identity(),
                                   Eigen::Matrix3d::Zero(),
                                   Eigen::Vector3d::Zero());
    // Displacement and gradient in the null direction supply no evidence.
    wall.AddPoseWithInformation(
        time, PlanarPose(Eigen::Vector2d(0., 100.), 0.), information,
        Eigen::Vector3d(0., 17., 0.));
    EXPECT_NEAR(0., wall.ExtrapolatePose(time).translation().y(), 1e-12);
    EXPECT_NEAR(0., wall.velocity().y(), 1e-12);
    EXPECT_NEAR(0., wall.acceleration_bias().y(), 1e-12);
    const auto wall_covariance = wall.covariance();
    const auto free_covariance = unfused.covariance();
    for (const int row : {1, 3, 7}) {
      for (int column = 0; column < 8; ++column) {
        EXPECT_NEAR(free_covariance(row, column),
                    wall_covariance(row, column), 1e-12);
      }
    }
    // Positive controls: measured directions really did gain confidence.
    EXPECT_LT(wall_covariance(0, 0), free_covariance(0, 0));
    EXPECT_LT(wall_covariance(4, 4), free_covariance(4, 4));
  }
}

TEST(PlanarImuPoseExtrapolatorTest,
     PureLidarGradientRemovesMatcherPriorBias) {
  PlanarImuPoseExtrapolator at_lidar_optimum(
      TestOptions(false), kInertialTestGravityTimeConstant);
  PlanarImuPoseExtrapolator at_matcher_map(
      TestOptions(false), kInertialTestGravityTimeConstant);
  PlanarImuPoseExtrapolator ignoring_gradient(
      TestOptions(false), kInertialTestGravityTimeConstant);
  for (auto* extrapolator :
       {&at_lidar_optimum, &at_matcher_map, &ignoring_gradient}) {
    Seed(extrapolator);
    for (int i = 1; i <= 20; ++i) {
      extrapolator->AddImuData(sensor::ImuData{
          Time(i * kImuPeriod), Eigen::Vector3d(0., 0., kGravity),
          Eigen::Vector3d::Zero()});
    }
  }
  Eigen::Matrix3d information;
  information << 400., 20., 10., 20., 600., -15., 10., -15., 1200.;
  const Eigen::Vector3d lidar_optimum(0.12, -0.08, 0.04);
  const Eigen::Vector3d matcher_map(0.025, -0.01, 0.008);
  // The SAME lidar quadratic 1/2*(z-z_lidar)'*H*(z-z_lidar), expanded at its
  // optimum or at a matcher pose pulled toward the motion prediction.
  const Eigen::Vector3d gradient = information * (matcher_map - lidar_optimum);
  at_lidar_optimum.AddPoseWithInformation(
      Time(0.1), PlanarPose(lidar_optimum.head<2>(), lidar_optimum.z()),
      information, Eigen::Vector3d::Zero());
  at_matcher_map.AddPoseWithInformation(
      Time(0.1), PlanarPose(matcher_map.head<2>(), matcher_map.z()),
      information, gradient);
  ignoring_gradient.AddPoseWithInformation(
      Time(0.1), PlanarPose(matcher_map.head<2>(), matcher_map.z()),
      information, Eigen::Vector3d::Zero());
  const auto expected = at_lidar_optimum.ExtrapolatePose(Time(0.1));
  const auto actual = at_matcher_map.ExtrapolatePose(Time(0.1));
  EXPECT_NEAR(0., (actual.translation() - expected.translation()).norm(),
              1e-10);
  EXPECT_NEAR(0., actual.rotation().angularDistance(expected.rotation()),
              1e-10);
  EXPECT_NEAR(
      0., (at_matcher_map.velocity() - at_lidar_optimum.velocity()).norm(),
      1e-10);
  EXPECT_NEAR(at_lidar_optimum.gyro_bias(), at_matcher_map.gyro_bias(), 1e-10);
  EXPECT_NEAR(0., (at_matcher_map.acceleration_bias() -
                   at_lidar_optimum.acceleration_bias()).norm(), 1e-10);
  EXPECT_LT((at_matcher_map.covariance() - at_lidar_optimum.covariance())
                .cwiseAbs().maxCoeff(), 1e-12);
  // A zero-gradient implementation would retain the matching-prior bias.
  EXPECT_GT((ignoring_gradient.ExtrapolatePose(Time(0.1)).translation() -
             expected.translation()).norm(), 0.05);
  // Nonzero cross-covariances also exercise velocity and bias learning.
  EXPECT_GT(at_lidar_optimum.velocity().norm(), 0.1);
  EXPECT_GT(at_lidar_optimum.acceleration_bias().norm(), 1e-5);
  EXPECT_GT(std::abs(at_lidar_optimum.gyro_bias()), 1e-5);
}

proto::PlanarImuDeskewOptions TightOptions(bool nhc = false) {
  auto options = TestOptions(nhc);
  auto* tight = options.mutable_iterated_update();
  tight->set_enabled(true);
  tight->set_max_num_iterations(10);
  tight->set_grid_residual_stddev(0.15);
  tight->set_huber_scale(2.5);
  tight->set_max_translation_update(0.5);
  tight->set_max_yaw_update(0.35);
  return options;
}

TEST(PlanarTightUpdateTest, UnknownGridDoesNotConditionPrior) {
  PlanarImuPoseExtrapolator control(TightOptions(), 10.);
  PlanarImuPoseExtrapolator filter(TightOptions(), 10.);
  Seed(&control);
  Seed(&filter);
  ValueConversionTables tables;
  ProbabilityGrid grid(MapLimits(0.1, Eigen::Vector2d(1., 1.),
                                CellLimits(20, 20)), &tables);
  sensor::PointCloud points;
  points.push_back({Eigen::Vector3f(0.3f, 0.2f, 0.f)});
  const auto time = Time(0.005);
  const auto prediction = control.ExtrapolatePose(time);
  control.AddPoseWithInformation(time, prediction, Eigen::Matrix3d::Zero(),
                                  Eigen::Vector3d::Zero());
  const auto update = filter.UpdateWithLidar(time, points, &grid);
  EXPECT_TRUE(update.accepted);
  EXPECT_EQ(1, update.measurement_residuals);
  EXPECT_THAT(filter.ExtrapolatePose(time), transform::IsNearly(prediction, 1e-10));
  EXPECT_LT((filter.covariance() - control.covariance()).norm(), 1e-10);
}

TEST(PlanarTightUpdateTest, RawGridGradientUpdatesVelocityAndBias) {
  PlanarImuPoseExtrapolator filter(TightOptions(), 1e6);
  Seed(&filter);
  for (int i = 1; i <= 20; ++i) {
    filter.AddImuData({Time(i * 0.005), Eigen::Vector3d(0., 0., kGravity),
                       Eigen::Vector3d::Zero()});
  }
  ValueConversionTables tables;
  ProbabilityGrid grid(MapLimits(0.1, Eigen::Vector2d(1., 1.),
                                CellLimits(20, 20)), &tables);
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      const Eigen::Array2i cell(x, y);
      const auto center = grid.limits().GetCellCenter(cell);
      const double squared = std::pow(center.x() - 0.2, 2) +
                             std::pow(center.y() - 0.1, 2);
      grid.SetProbability(cell, std::max(0.15, 0.85 - 0.25 * squared));
    }
  }
  sensor::PointCloud points;
  points.push_back({Eigen::Vector3f::Zero()});
  const auto update = filter.UpdateWithLidar(Time(0.1), points, &grid);
  ASSERT_TRUE(update.accepted);
  EXPECT_LT(update.final_cost, update.initial_cost);
  EXPECT_GT(filter.ExtrapolatePose(Time(0.1)).translation().x(), 1e-4);
  EXPECT_GT(filter.velocity().x(), 1e-4);
  EXPECT_LT(filter.acceleration_bias().x(), 0.);
}

TEST(PlanarTightUpdateTest, JointNhcConditionsBodyLateralDirection) {
  PlanarImuPoseExtrapolator filter(TightOptions(true), 10.);
  Seed(&filter, kPi / 2.);
  const auto update = filter.UpdateWithLidar(Time(0.005), {}, nullptr);
  ASSERT_TRUE(update.accepted);
  EXPECT_EQ(1, update.measurement_residuals);
  EXPECT_LT(filter.covariance()(2, 2), 0.02);  // World x is body lateral.
  EXPECT_GT(filter.covariance()(3, 3), 0.9);   // World y is body forward.
}

TEST(PlanarTightUpdateTest, MissingImuRejectsWithoutLidarConditioning) {
  PlanarImuPoseExtrapolator filter(TightOptions(true), 10.);
  Seed(&filter);
  const auto prediction = filter.ExtrapolatePose(Time(0.2));
  const auto update = filter.UpdateWithLidar(Time(0.2), {}, nullptr);
  EXPECT_FALSE(update.accepted);
  EXPECT_EQ(0, update.iterations);
  EXPECT_THAT(filter.ExtrapolatePose(Time(0.2)), transform::IsNearly(prediction, 1e-10));
  EXPECT_GT(filter.covariance()(3, 3), 1.);
  EXPECT_DOUBLE_EQ(0., filter.gyro_bias());
}

TEST(PlanarImuPoseExtrapolatorOptionsTest,
     LegacyLuaWithoutDeskewRemainsDisabled) {
  // Deliberately omit the new block: existing user Lua must keep parsing.
  auto dictionary = common::MakeDictionary(R"lua(
    return {
      use_imu_data = true,
      min_range = 0., max_range = 30., min_z = -0.8, max_z = 2.,
      missing_data_ray_length = 5., num_accumulated_range_data = 1,
      voxel_filter_size = 0.025,
      adaptive_voxel_filter = {
        max_length = 0.5, min_num_points = 200, max_range = 50.,
      },
      loop_closure_adaptive_voxel_filter = {
        max_length = 0.9, min_num_points = 100, max_range = 50.,
      },
      use_online_correlative_scan_matching = false,
      real_time_correlative_scan_matcher = {
        linear_search_window = 0.1, angular_search_window = 0.35,
        translation_delta_cost_weight = 0.1, rotation_delta_cost_weight = 0.1,
      },
      ceres_scan_matcher = {
        occupied_space_weight = 1., translation_weight = 10.,
        rotation_weight = 40.,
        ceres_solver_options = {
          use_nonmonotonic_steps = false,
          max_num_iterations = 20, num_threads = 1,
        },
      },
      motion_filter = {
        max_time_seconds = 5., max_distance_meters = 0.2,
        max_angle_radians = 0.017,
      },
      imu_gravity_time_constant = 10.,
      pose_extrapolator = {
        use_imu_based = false,
        constant_velocity = {
          imu_gravity_time_constant = 10., pose_queue_duration = 0.001,
        },
        imu_based = {
          pose_queue_duration = 5., gravity_constant = 9.806,
          pose_translation_weight = 1., pose_rotation_weight = 1.,
          imu_acceleration_weight = 1., imu_rotation_weight = 1.,
          odometry_translation_weight = 1., odometry_rotation_weight = 1.,
          solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 10, num_threads = 1,
          },
        },
      },
      submaps = {
        num_range_data = 90,
        grid_options_2d = { grid_type = "PROBABILITY_GRID", resolution = 0.05 },
        range_data_inserter = {
          range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
          probability_grid_range_data_inserter = {
            insert_free_space = true,
            hit_probability = 0.55, miss_probability = 0.49,
          },
          tsdf_range_data_inserter = {
            truncation_distance = 0.3, maximum_weight = 10.,
            update_free_space = false,
            normal_estimation_options = {
              num_normal_samples = 4, sample_radius = 0.5,
            },
            project_sdf_distance_to_scan_normal = true,
            update_weight_range_exponent = 0,
            update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
            update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
          },
        },
      },
    }
  )lua");
  const auto options = CreateLocalTrajectoryBuilderOptions2D(dictionary.get());
  EXPECT_TRUE(options.use_imu_data());
  EXPECT_FALSE(options.pose_extrapolator_options().use_imu_based());
  EXPECT_DOUBLE_EQ(0.001, options.pose_extrapolator_options()
                                .constant_velocity().pose_queue_duration());
  EXPECT_FLOAT_EQ(30., options.max_range());
  EXPECT_FALSE(options.planar_imu_deskew().enabled());
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
