// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <optional>
#include <Eigen/Core>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/utils.hpp>
namespace hector_mapping_ros2 {
// Preserve the last raw map correction and apply only the odometry increment.
// No low-pass filter, pose averaging, or correction blending is performed.
class OdometryPosePrior {
public:
  void reset() { map_from_odom_.reset(); }
  std::optional<Eigen::Vector3f> predict(const tf2::Transform& odom_from_base) const {
    if(!map_from_odom_) return std::nullopt;
    const auto pose=(*map_from_odom_)*odom_from_base;
    return Eigen::Vector3f(pose.getOrigin().x(),pose.getOrigin().y(),tf2::getYaw(pose.getRotation()));
  }
  tf2::Transform correct(const tf2::Transform& map_from_base,const tf2::Transform& odom_from_base) {
    map_from_odom_=map_from_base*odom_from_base.inverse();
    return *map_from_odom_;
  }
private:
  std::optional<tf2::Transform> map_from_odom_;
};
}
