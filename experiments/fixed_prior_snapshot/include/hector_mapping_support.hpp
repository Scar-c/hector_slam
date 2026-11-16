// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <hector_slam_lib/scan/DataPointContainer.h>

namespace hector_mapping_ros2 {
// Points and ray origin must both be in base coordinates, scaled to finest-map cells.
inline bool convertScan(const sensor_msgs::msg::LaserScan &scan,
  const tf2::Transform &base_from_laser, float scale, double min_range,
  double max_range, double z_min, double z_max, hectorslam::DataContainer &data)
{
  data.clear();
  const auto origin = base_from_laser.getOrigin();
  data.setOrigo(Eigen::Vector2f(origin.x(), origin.y()) * scale);
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
      !std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0 ||
      !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0 || scan.range_max <= scan.range_min) return false;
  const double lower = std::max(min_range, static_cast<double>(scan.range_min));
  const double upper = std::min(max_range, static_cast<double>(scan.range_max));
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r <= lower || r >= upper) continue;
    const double angle = scan.angle_min + i * static_cast<double>(scan.angle_increment);
    const auto point = base_from_laser * tf2::Vector3(r * std::cos(angle), r * std::sin(angle), 0);
    const double z = point.z() - origin.z();
    if (z < z_min || z > z_max) continue;
    data.add(Eigen::Vector2f(point.x(), point.y()) * scale);
  }
  return data.getSize() != 0;
}

// Upstream's "covariance" accessor actually returns the scan-matcher Hessian.
// Invert in observable directions and convert cell units to metres. This is a
// local approximation, not a calibrated sensor covariance for sensor fusion.
inline Eigen::Matrix3d poseCovariance(const Eigen::Matrix3f &hessian, double resolution)
{
  if (!hessian.allFinite()) return Eigen::Matrix3d::Identity() * 1e6;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(hessian.cast<double>());
  if (solver.info() != Eigen::Success) return Eigen::Matrix3d::Identity() * 1e6;
  Eigen::Vector3d inverse;
  for (int i = 0; i < 3; ++i) {
    inverse[i] = solver.eigenvalues()[i] > 1e-6 ? 1.0 / solver.eigenvalues()[i] : 1e6;
  }
  Eigen::Matrix3d units = Eigen::Vector3d(resolution, resolution, 1).asDiagonal();
  return units * solver.eigenvectors() * inverse.asDiagonal() *
         solver.eigenvectors().transpose() * units;
}
}  // namespace hector_mapping_ros2
