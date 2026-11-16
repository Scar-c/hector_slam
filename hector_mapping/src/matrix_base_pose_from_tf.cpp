// Matrix adapter uses the same TF query logic at 10 Hz of ROS time.
// Adapted from zx_localization/src/base_pose_from_tf.cpp; only AMCL lifecycle gating removed.
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "base_pose_from_tf_utils.hpp"

namespace zx_localization
{

class BasePoseFromTf : public rclcpp::Node
{
public:
  BasePoseFromTf()
  : Node("localization_base_pose_from_tf"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    source_frame_ = declare_parameter<std::string>("source_frame", "base_link");
    base_pose_topic_ = declare_parameter<std::string>("base_pose_topic", "/base_pose");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    lookup_timeout_sec_ = declare_parameter<double>("lookup_timeout_sec", 0.05);
    max_tf_age_sec_ = declare_parameter<double>("max_tf_age_sec", 1.0);

    validate_parameters();

    base_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      base_pose_topic_, rclcpp::QoS(10));
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_timer(period, [this]() {publish_base_pose();});

    RCLCPP_INFO(
      get_logger(),
      "Base pose TF publisher started: %s <- %s, topic=%s, rate=%.1fHz, max_age=%.2fs",
      target_frame_.c_str(), source_frame_.c_str(), base_pose_topic_.c_str(),
      publish_rate_hz_, max_tf_age_sec_);
  }

private:
  void validate_parameters() const
  {
    if (target_frame_.empty() || source_frame_.empty() || base_pose_topic_.empty()) {
      throw std::invalid_argument("frame and topic parameters must not be empty");
    }
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (!std::isfinite(lookup_timeout_sec_) || lookup_timeout_sec_ < 0.0) {
      throw std::invalid_argument("lookup_timeout_sec must be non-negative");
    }
    if (!std::isfinite(max_tf_age_sec_) || max_tf_age_sec_ <= 0.0) {
      throw std::invalid_argument("max_tf_age_sec must be positive");
    }
  }

  void publish_base_pose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero,
        tf2::durationFromSec(lookup_timeout_sec_));
      const auto now = get_clock()->now();
      if (!is_transform_fresh(transform, now, max_tf_age_sec_)) {
        const rclcpp::Time stamp(transform.header.stamp, now.get_clock_type());
        const double time_offset_sec = (now - stamp).seconds();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TF %s <- %s is stale or invalid: time_offset=%.3fs, max=%.3fs",
          target_frame_.c_str(), source_frame_.c_str(), time_offset_sec, max_tf_age_sec_);
        return;
      }

      base_pose_publisher_->publish(pose_from_transform(transform));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for TF %s <- %s: %s",
        target_frame_.c_str(), source_frame_.c_str(), exception.what());
    }
  }

  std::string target_frame_;
  std::string source_frame_;
  std::string base_pose_topic_;
  double publish_rate_hz_{10.0};
  double lookup_timeout_sec_{0.05};
  double max_tf_age_sec_{1.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace zx_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<zx_localization::BasePoseFromTf>();
  try {
    rclcpp::spin(node);
  } catch (const rclcpp::exceptions::RCLError &) {
    if (rclcpp::ok()) {
      throw;
    }
  }
  rclcpp::shutdown();
  return 0;
}
