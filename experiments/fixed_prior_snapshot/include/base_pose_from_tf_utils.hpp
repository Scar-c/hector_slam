#ifndef ZX_LOCALIZATION__BASE_POSE_FROM_TF_UTILS_HPP_
#define ZX_LOCALIZATION__BASE_POSE_FROM_TF_UTILS_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/time.hpp"

namespace zx_localization
{

inline bool is_transform_fresh(
  const geometry_msgs::msg::TransformStamped & transform,
  const rclcpp::Time & now,
  const double max_tf_age_sec)
{
  if (!std::isfinite(max_tf_age_sec) || max_tf_age_sec <= 0.0) {
    return false;
  }

  const rclcpp::Time stamp(transform.header.stamp, now.get_clock_type());
  if (stamp.nanoseconds() <= 0) {
    return false;
  }

  const double time_offset_sec = (now - stamp).seconds();
  return std::isfinite(time_offset_sec) && std::abs(time_offset_sec) <= max_tf_age_sec;
}

inline geometry_msgs::msg::PoseStamped pose_from_transform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

}  // namespace zx_localization

#endif  // ZX_LOCALIZATION__BASE_POSE_FROM_TF_UTILS_HPP_
