// Adapted from zx_lidar_filter dual_scan_merger_node.cpp.
// Align each source frame to a shared reference timestamp using EKF odom TF.
#include <chrono>
#include <deque>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include <sstream>
#include <iomanip>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace zx_lidar_filter
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp_positive(const double value, const double fallback)
{
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

}  // namespace

class DualScanMergerNode : public rclcpp::Node
{
public:
  DualScanMergerNode()
  : Node("dual_scan_merger")
  {
    frame_alignment_=declare_parameter<bool>("apply_frame_alignment",true);
    beam_deskew_=declare_parameter<bool>("apply_beam_deskew",false);
    beam_offset_=declare_parameter<double>("beam_time_offset_sec",0.0);
    beam_direction_=declare_parameter<double>("beam_time_direction",1.0);
    if(!std::isfinite(beam_offset_) || std::abs(beam_offset_)>.2 || (beam_direction_!=1 && beam_direction_!=-1))
      throw std::invalid_argument("Invalid beam timing configuration");
    fixed_frame_=declare_parameter<std::string>("motion_compensation_fixed_frame", "odom");
    alignment_wait_sec_=declare_parameter<double>("alignment_wait_sec",0.5);
    if(fixed_frame_.empty() || !std::isfinite(alignment_wait_sec_) || alignment_wait_sec_<=0 || alignment_wait_sec_>2)
      throw std::invalid_argument("Invalid motion compensation parameters");
    front_scan_topic_ = declare_parameter<std::string>("front_scan_topic", "/scan");
    back_scan_topic_ = declare_parameter<std::string>("back_scan_topic", "/scan_back");
    output_scan_topic_ = declare_parameter<std::string>("output_scan_topic", "/scan_amcl");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    angle_min_ = declare_parameter<double>("angle_min", -kPi);
    angle_max_ = declare_parameter<double>("angle_max", kPi);
    angle_increment_ = clamp_positive(
      declare_parameter<double>("angle_increment", 0.00872665), 0.00872665);
    range_min_ = clamp_positive(declare_parameter<double>("range_min", 0.05), 0.05);
    range_max_ = clamp_positive(declare_parameter<double>("range_max", 30.0), 30.0);
    publish_rate_hz_ = clamp_positive(
      declare_parameter<double>("publish_rate_hz", 10.0), 10.0);
    scan_timeout_sec_ = clamp_positive(
      declare_parameter<double>("scan_timeout_sec", 0.3), 0.3);
    max_scan_time_skew_sec_ = clamp_positive(
      declare_parameter<double>("max_scan_time_skew_sec", 0.10), 0.10);
    tf_timeout_sec_ = clamp_positive(declare_parameter<double>("tf_timeout_sec", 0.05), 0.05);
    stats_log_period_sec_ = declare_parameter<double>("stats_log_period_sec", 5.0);

    if (angle_max_ <= angle_min_) {
      RCLCPP_WARN(
        get_logger(),
        "dual scan merger angle_max <= angle_min, fallback to [-pi, pi]");
      angle_min_ = -kPi;
      angle_max_ = kPi;
    }
    if (range_max_ <= range_min_) {
      RCLCPP_WARN(
        get_logger(),
        "dual scan merger range_max <= range_min, fallback to [0.05, 30.0]");
      range_min_ = 0.05;
      range_max_ = 30.0;
    }

    beam_count_ = static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1U;
    beam_count_ = std::max<std::size_t>(beam_count_, 1U);
    angle_max_ = angle_min_ + static_cast<double>(beam_count_ - 1U) * angle_increment_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::SensorDataQoS();
    front_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      front_scan_topic_, qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        front_scan_ = msg;
        front_received_at_ = now();
      });
    back_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      back_scan_topic_, qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        back_scan_ = msg;
        back_received_at_ = now();
      });
    merged_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_scan_topic_, qos);

    alignment_pub_=create_publisher<std_msgs::msg::String>("/scan_alignment_status",100);
    cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/points_aligned",qos);
    xyz_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/points_aligned_xyz",qos);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(100),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
      if(!first_odom_ns_) {first_odom_ns_=rclcpp::Time(msg->header.stamp).nanoseconds();process_pending();}
    });
    pending_timer_=create_wall_timer(std::chrono::milliseconds(5),[this](){process_pending();});
    timer_ = create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      [this]() { publish_merged_scan(); });

    last_stats_log_at_ = now();
    RCLCPP_INFO(
      get_logger(),
      "dual scan merger started: front=%s back=%s output=%s target_frame=%s beams=%zu "
      "angle=[%.3f, %.3f] increment=%.6f range=[%.3f, %.3f] max_skew=%.3fs",
      front_scan_topic_.c_str(), back_scan_topic_.c_str(), output_scan_topic_.c_str(),
      target_frame_.c_str(), beam_count_, angle_min_, angle_max_, angle_increment_,
      range_min_, range_max_, max_scan_time_skew_sec_);
  }

private:
  struct MergeStats
  {
    std::size_t front_points{0};
    std::size_t back_points{0};
    std::size_t merged_points{0};
    std::size_t tf_failures{0};
    std::size_t stale_front{0};
    std::size_t stale_back{0};
    std::size_t skipped_cycles{0};
    std::size_t skew_rejections{0};
    std::size_t skew_front_dropped{0};
    std::size_t skew_back_dropped{0};
    double max_skew_sec{0.0};
  };

  bool scan_is_fresh(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    const rclcpp::Time & received_at,
    std::size_t * stale_counter) const
  {
    if (!scan) {
      return false;
    }
    const double age_sec = (now() - received_at).seconds();
    if (age_sec < 0.0 || age_sec > scan_timeout_sec_) {
      if (stale_counter != nullptr) {
        ++(*stale_counter);
      }
      return false;
    }
    return true;
  }

  struct Pending {
    rclcpp::Time reference;
    sensor_msgs::msg::LaserScan::SharedPtr front,back;
    std::chrono::steady_clock::time_point arrival;
  };
  rclcpp::Time source_time(const sensor_msgs::msg::LaserScan& scan,size_t index=0) const {
    auto t=rclcpp::Time(scan.header.stamp,get_clock()->get_clock_type());
    if(beam_deskew_) t=t+rclcpp::Duration::from_seconds(beam_offset_+beam_direction_*index*scan.time_increment);
    return t;
  }
  rclcpp::Time anchor_time(const sensor_msgs::msg::LaserScan& scan,const rclcpp::Time& reference) const {
    return frame_alignment_?reference:rclcpp::Time(scan.header.stamp,get_clock()->get_clock_type());
  }
  geometry_msgs::msg::TransformStamped point_transform(const sensor_msgs::msg::LaserScan& scan,const rclcpp::Time& reference,size_t index=0) const {
    if(!frame_alignment_ && !beam_deskew_)
      return tf_buffer_->lookupTransform(target_frame_,scan.header.frame_id,rclcpp::Time(scan.header.stamp,get_clock()->get_clock_type()),rclcpp::Duration::from_seconds(0));
    return tf_buffer_->lookupTransform(target_frame_,anchor_time(scan,reference),scan.header.frame_id,source_time(scan,index),fixed_frame_,rclcpp::Duration::from_seconds(0));
  }
  bool alignment_ready(const sensor_msgs::msg::LaserScan::SharedPtr& scan,const rclcpp::Time& reference) const {
    if(!scan) return true;
    if(scan->header.stamp.sec==0 && scan->header.stamp.nanosec==0) return false;
    if(beam_deskew_ && (!std::isfinite(scan->time_increment)||scan->time_increment<0)) return false;
    try {
      point_transform(*scan,reference);
      if(beam_deskew_ && !scan->ranges.empty()) point_transform(*scan,reference,scan->ranges.size()-1);
      return true;
    } catch(const tf2::TransformException&) {return false;}
  }
  void publish_merged_scan() {
    bool use_front=scan_is_fresh(front_scan_,front_received_at_,&stats_.stale_front);
    bool use_back=scan_is_fresh(back_scan_,back_received_at_,&stats_.stale_back);
    apply_scan_time_skew_filter(use_front,use_back);
    if(!use_front && !use_back) {++stats_.skipped_cycles;maybe_log_stats();return;}
    const auto reference=now();
    if(last_reference_ns_ && reference.nanoseconds()<=*last_reference_ns_) return;
    last_reference_ns_=reference.nanoseconds();
    if(pending_.size()>=50) {pending_.pop_front();++alignment_dropped_;}
    pending_.push_back({reference,use_front?front_scan_:nullptr,use_back?back_scan_:nullptr,std::chrono::steady_clock::now()});
    process_pending();
  }
  void process_pending() {
    while(!pending_.empty()) {
      const auto& pending=pending_.front();
      const double waited=std::chrono::duration<double>(std::chrono::steady_clock::now()-pending.arrival).count();
      bool permanently_too_early=false;
      if(first_odom_ns_ && (frame_alignment_||beam_deskew_)) {
        for(const auto& scan:{pending.front,pending.back}) if(scan) {
          auto earliest=std::min(source_time(*scan).nanoseconds(),source_time(*scan,scan->ranges.empty()?0:scan->ranges.size()-1).nanoseconds());
          earliest=std::min(earliest,anchor_time(*scan,pending.reference).nanoseconds());
          permanently_too_early |= earliest<*first_odom_ns_;
        }
      }
      if(permanently_too_early) {++alignment_dropped_;pending_.pop_front();continue;}
      if(!alignment_ready(pending.front,pending.reference) || !alignment_ready(pending.back,pending.reference)) {
        if(waited<alignment_wait_sec_) break;
        ++alignment_dropped_;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Alignment skipped: no EKF transforms at both acquisition and reference times");
        pending_.pop_front();continue;
      }
      sensor_msgs::msg::LaserScan merged;
      merged.header.stamp=pending.reference;merged.header.frame_id=target_frame_;
      merged.angle_min=angle_min_;merged.angle_max=angle_max_;merged.angle_increment=angle_increment_;
      // A common-time snapshot after frame-level motion compensation. No per-beam deskew is assumed.
      merged.time_increment=0;merged.scan_time=1.0/publish_rate_hz_;
      merged.range_min=range_min_;merged.range_max=range_max_;
      merged.ranges.assign(beam_count_,std::numeric_limits<float>::infinity());
      xyz_points_.clear();
      if(pending.front) project_scan(*pending.front,merged,stats_.front_points);
      if(pending.back) project_scan(*pending.back,merged,stats_.back_points);
      const auto finite=std::count_if(merged.ranges.begin(),merged.ranges.end(),[](float v){return std::isfinite(v);});
      stats_.merged_points+=finite;
      std::ostringstream json;json<<std::setprecision(15)
        <<"{\"reference_stamp_ns\":"<<pending.reference.nanoseconds()
        <<",\"front_stamp_ns\":"<<(pending.front?rclcpp::Time(pending.front->header.stamp).nanoseconds():0)
        <<",\"back_stamp_ns\":"<<(pending.back?rclcpp::Time(pending.back->header.stamp).nanoseconds():0)
        <<",\"wait_ms\":"<<waited*1000<<",\"points\":"<<finite
        <<",\"dropped\":"<<alignment_dropped_
        <<",\"time_alignment\":"<<(frame_alignment_?"true":"false")
        <<",\"beam_deskew\":"<<(beam_deskew_?"true":"false")
        <<",\"beam_time_offset_sec\":"<<beam_offset_<<",\"beam_time_direction\":"<<beam_direction_
        <<",\"mode\":\"independent_time_and_deskew_switches\"}";
      std_msgs::msg::String status;status.data=json.str();alignment_pub_->publish(status);
      merged_pub_->publish(merged);
      sensor_msgs::msg::PointCloud2 cloud;cloud.header=merged.header;
      sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(finite);
      sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
      for(size_t i=0;i<merged.ranges.size();++i) if(std::isfinite(merged.ranges[i])) {
        const double a=merged.angle_min+i*merged.angle_increment;
        *x=merged.ranges[i]*std::cos(a);*y=merged.ranges[i]*std::sin(a);*z=0;++x;++y;++z;
      }
      cloud_pub_->publish(cloud);
      sensor_msgs::msg::PointCloud2 xyz;xyz.header=merged.header;
      sensor_msgs::PointCloud2Modifier xyz_modifier(xyz);xyz_modifier.setPointCloud2FieldsByString(1,"xyz");xyz_modifier.resize(xyz_points_.size());
      sensor_msgs::PointCloud2Iterator<float> xx(xyz,"x"),xy(xyz,"y"),xz(xyz,"z");
      for(const auto&p:xyz_points_){*xx=p.x();*xy=p.y();*xz=p.z();++xx;++xy;++xz;}
      xyz.is_dense=true;xyz_pub_->publish(xyz);
      pending_.pop_front();maybe_log_stats();
    }
  }

  void apply_scan_time_skew_filter(bool & use_front, bool & use_back)
  {
    if (!use_front || !use_back || !front_scan_ || !back_scan_) {
      return;
    }

    const auto front_time = effective_scan_time(*front_scan_, front_received_at_);
    const auto back_time = effective_scan_time(*back_scan_, back_received_at_);
    const double signed_skew_sec = (front_time - back_time).seconds();
    const double skew_sec = std::abs(signed_skew_sec);
    stats_.max_skew_sec = std::max(stats_.max_skew_sec, skew_sec);

    if (skew_sec <= max_scan_time_skew_sec_) {
      return;
    }

    ++stats_.skew_rejections;
    if (signed_skew_sec >= 0.0) {
      use_back = false;
      ++stats_.skew_back_dropped;
    } else {
      use_front = false;
      ++stats_.skew_front_dropped;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "dual scan merger scan time skew %.3fs exceeds %.3fs, using %s scan only",
      skew_sec, max_scan_time_skew_sec_, signed_skew_sec >= 0.0 ? "front" : "back");
  }

  rclcpp::Time effective_scan_time(
    const sensor_msgs::msg::LaserScan & scan,
    const rclcpp::Time & received_at) const
  {
    if (scan.header.stamp.sec == 0 && scan.header.stamp.nanosec == 0U) {
      return received_at;
    }
    return rclcpp::Time(scan.header.stamp, get_clock()->get_clock_type());
  }

  void project_scan(const sensor_msgs::msg::LaserScan& scan,sensor_msgs::msg::LaserScan& merged,std::size_t& accepted_points) {
    const rclcpp::Time reference(merged.header.stamp,get_clock()->get_clock_type());
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform=point_transform(scan,reference);
      for(size_t i=0;i<scan.ranges.size();++i) {
        const float range=scan.ranges[i];if(!valid_input_range(range,scan)) continue;
        if(beam_deskew_) transform=point_transform(scan,reference,i);
        const auto&t=transform.transform.translation;const auto&q=transform.transform.rotation;
        const tf2::Matrix3x3 rotation(tf2::Quaternion(q.x,q.y,q.z,q.w));
        const double angle=scan.angle_min+i*static_cast<double>(scan.angle_increment);
        const tf2::Vector3 point=rotation*tf2::Vector3(range*std::cos(angle),range*std::sin(angle),0)+tf2::Vector3(t.x,t.y,t.z);
        insert_point(point,merged,accepted_points);
      }
    } catch(const tf2::TransformException& e) {
      ++stats_.tf_failures;RCLCPP_ERROR(get_logger(),"Projection transform failed: %s",e.what());
    }
  }

  bool valid_input_range(
    const float range,
    const sensor_msgs::msg::LaserScan & scan) const
  {
    if (!std::isfinite(range)) {
      return false;
    }
    const double input_min = std::max<double>(scan.range_min, range_min_);
    const double input_max = std::min<double>(scan.range_max, range_max_);
    return static_cast<double>(range) >= input_min && static_cast<double>(range) <= input_max;
  }

  void insert_point(
    const tf2::Vector3 & point,
    sensor_msgs::msg::LaserScan & merged,
    std::size_t & accepted_points)
  {
    const double x = point.x();
    const double y = point.y();
    const double range = std::hypot(x, y);
    if (!std::isfinite(range) || range < range_min_ || range > range_max_) {
      return;
    }

    const double angle = std::atan2(y, x);
    if (angle < angle_min_ || angle > angle_max_) {
      return;
    }

    const auto index = static_cast<std::size_t>(
      std::llround((angle - angle_min_) / angle_increment_));
    if (index >= merged.ranges.size()) {
      return;
    }

    xyz_points_.push_back(point);
    auto & current = merged.ranges[index];
    if (!std::isfinite(current) || range < static_cast<double>(current)) {
      current = static_cast<float>(range);
    }
    ++accepted_points;
  }

  void maybe_log_stats()
  {
    if (stats_log_period_sec_ <= 0.0) {
      return;
    }
    const auto now_time = now();
    if ((now_time - last_stats_log_at_).seconds() < stats_log_period_sec_) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "dual scan merger stats: front_points=%zu back_points=%zu merged_beams=%zu "
      "tf_failures=%zu stale_front=%zu stale_back=%zu skipped_cycles=%zu "
      "skew_rejections=%zu skew_front_dropped=%zu skew_back_dropped=%zu max_skew_sec=%.3f",
      stats_.front_points, stats_.back_points, stats_.merged_points,
      stats_.tf_failures, stats_.stale_front, stats_.stale_back, stats_.skipped_cycles,
      stats_.skew_rejections, stats_.skew_front_dropped, stats_.skew_back_dropped,
      stats_.max_skew_sec);
    stats_ = MergeStats{};
    last_stats_log_at_ = now_time;
  }

  std::string front_scan_topic_;
  std::string back_scan_topic_;
  std::string output_scan_topic_;
  std::string target_frame_;
  double angle_min_{-kPi};
  double angle_max_{kPi};
  double angle_increment_{0.00872665};
  double range_min_{0.05};
  double range_max_{30.0};
  double publish_rate_hz_{10.0};
  double scan_timeout_sec_{0.3};
  double max_scan_time_skew_sec_{0.10};
  double tf_timeout_sec_{0.05};
  double stats_log_period_sec_{5.0};
  std::size_t beam_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr back_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_,pending_timer_;
  std::string fixed_frame_;
  bool frame_alignment_,beam_deskew_;
  double beam_offset_,beam_direction_;
  double alignment_wait_sec_;
  uint64_t alignment_dropped_=0;
  std::deque<Pending> pending_;
  std::optional<int64_t> last_reference_ns_,first_odom_ns_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alignment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_,xyz_pub_;
  std::vector<tf2::Vector3> xyz_points_;

  sensor_msgs::msg::LaserScan::SharedPtr front_scan_;
  sensor_msgs::msg::LaserScan::SharedPtr back_scan_;
  rclcpp::Time front_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time back_received_at_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  MergeStats stats_;
  rclcpp::Time last_stats_log_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace zx_lidar_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zx_lidar_filter::DualScanMergerNode>());
  rclcpp::shutdown();
  return 0;
}
