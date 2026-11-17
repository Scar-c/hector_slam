// SPDX-License-Identifier: BSD-3-Clause
// ROS 2 Jazzy adapter for TU Darmstadt's Hector SLAM core (see LICENSE/UPSTREAM.md).
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <map>
#include "cartographer/sensor/internal/voxel_filter.h"
#include "imu_orientation_filter.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <hector_mapping/srv/reset_mapping.hpp>
#include <hector_slam_lib/slam_main/HectorSlamProcessor.h>
#include "hector_mapping_support.hpp"
#include "prior_map.hpp"
#include "odometry_pose_prior.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>

using namespace std::chrono_literals;
class HectorMappingNode : public rclcpp::Node {
public:
  HectorMappingNode() : Node("hector_mapping"), buffer_(get_clock()), listener_(buffer_),
    broadcaster_(*this), last_scan_(0, 0, get_clock()->get_clock_type())
  {
    localization_only_ = parameter("localization_only", false);
    const auto prior_file = parameter<std::string>("prior_map_yaml", "");
    global_relocalization_ = parameter("global_relocalization", false);
    search_radius_ = parameter("relocalization_xy_radius", 1.0);
    search_yaw_ = parameter("relocalization_yaw_radius", 0.35);
    min_score_ = parameter("localization_min_score", 0.45);
    declare_parameter<bool>("enforce_localization_score",true);
    max_step_ = parameter("localization_max_step", 0.8);
    if (!std::isfinite(search_radius_) || search_radius_<0 || search_radius_>10 ||
        !std::isfinite(search_yaw_) || search_yaw_<0 || search_yaw_>M_PI ||
        !std::isfinite(min_score_) || min_score_<=0 || min_score_>1 ||
        !std::isfinite(max_step_) || max_step_<=0) throw std::invalid_argument("Invalid localization parameters");
    if (localization_only_) {
      if(prior_file.empty()) throw std::invalid_argument("localization_only requires prior_map_yaml");
      map_=hector_mapping_ros2::loadPriorMap(prior_file);
      prior_yaw_=tf2::getYaw(map_.info.origin.orientation);
      prior_origin_=Eigen::Vector2f(map_.info.origin.position.x,map_.info.origin.position.y);
      prior_matcher_=std::make_unique<hector_mapping_ros2::PriorMatcher>(map_);
    }
    map_frame_ = parameter<std::string>("map_frame", "map");
    base_frame_ = parameter<std::string>("base_frame", "base_link");
    // No odometry is required by default: publish map -> base_link directly.
    odom_frame_ = parameter<std::string>("odom_frame", "base_link");
    pub_tf_ = parameter("pub_map_odom_transform", true);
    pub_scanmatch_tf_ = parameter("pub_map_scanmatch_transform", false);
    scanmatch_frame_ = parameter<std::string>("tf_map_scanmatch_transform_frame_name", "scanmatcher_frame");
    use_scan_tf_ = parameter("use_tf_scan_transformation", true);
    odom_tf_only_ = parameter("allow_odom_tf_without_pose_prior", false);
    use_odom_hint_ = parameter("use_odom_pose_start_estimate", false);
    transform_tolerance_ = parameter("transform_tolerance", 0.0);
    if(!std::isfinite(transform_tolerance_) || transform_tolerance_<0 || transform_tolerance_>1)
      throw std::invalid_argument("Invalid transform_tolerance");
    use_pose_tf_ = parameter("use_tf_pose_start_estimate", false);
    known_poses_ = parameter("map_with_known_poses", false);
    const bool pub_odom = parameter("pub_odometry", false);
    use_imu_ = parameter("use_imu", false);
    imu_tolerance_ = parameter("imu_time_tolerance", 0.05);
    imu_gravity_ = parameter("imu_gravity", 9.80665);
    imu_time_constant_ = parameter("imu_gravity_time_constant", 0.5);
    const auto imu_topic = parameter<std::string>("imu_topic", "imu");
    timing_ = parameter("output_timing", false);
    const auto scan_topic = parameter<std::string>("scan_topic", "scan");
    voxel_size_=parameter("cloud_voxel_size",.025);
    ray_origin_=Eigen::Vector3d(parameter("ray_origin_x",.1007),parameter("ray_origin_y",0.),parameter("ray_origin_z",.0558));
    if(!std::isfinite(voxel_size_)||voxel_size_<.005||voxel_size_>.5)throw std::invalid_argument("Invalid cloud voxel size");
    const auto sys_topic = parameter<std::string>("sys_msg_topic", "syscommand");
    const auto pose_topic = parameter<std::string>("pose_update_topic", "poseupdate");
    const auto map_topic = parameter<std::string>("map_topic", "map");
    resolution_ = parameter("map_resolution", 0.05);
    const int size = parameter("map_size", 1024);
    const int levels = parameter("map_multi_res_levels", 3);
    const double start_x = parameter("map_start_x", 0.5);
    const double start_y = parameter("map_start_y", 0.5);
    const double period = parameter("map_pub_period", 1.0);
    min_range_ = parameter("laser_min_dist", 0.1);
    max_range_ = parameter("laser_max_dist", 30.0);
    z_min_ = parameter("laser_z_min_value", -1.0);
    z_max_ = parameter("laser_z_max_value", 1.0);
    tf_timeout_ = parameter("tf_timeout", 0.1);
    min_points_ = parameter("min_scan_points", 20);
    const int queue = parameter("scan_subscriber_queue_size", 5);
    const double free_factor = parameter("update_factor_free", 0.4);
    const double occupied_factor = parameter("update_factor_occupied", 0.9);
    const double distance = parameter("map_update_distance_thresh", 0.4);
    const double angle = parameter("map_update_angle_thresh", 0.13);
    const auto finite = [](double v) { return std::isfinite(v); };
    if (!finite(imu_tolerance_) || imu_tolerance_ <= 0 || imu_tolerance_ > 0.5 ||
        !finite(imu_gravity_) || imu_gravity_ <= 0 || !finite(imu_time_constant_) || imu_time_constant_ <= 0)
      throw std::invalid_argument("Invalid IMU filter parameters");
    if (!finite(resolution_) || resolution_ <= 0 || size < 8 || size > 8192 ||
        levels < 1 || levels > 10 || size % (1 << (levels - 1)) != 0 ||
        size / (1 << (levels - 1)) < 8 || !finite(period) || period < 0.01 ||
        !finite(start_x) || !finite(start_y) || start_x < 0 || start_x > 1 ||
        start_y < 0 || start_y > 1 || !finite(min_range_) || !finite(max_range_) ||
        min_range_ < 0 || max_range_ <= min_range_ || !finite(z_min_) ||
        !finite(z_max_) || z_min_ >= z_max_ || min_points_ < 3 || queue < 1 ||
        !finite(tf_timeout_) || tf_timeout_ < 0 || !finite(free_factor) ||
        !finite(occupied_factor) || free_factor <= 0 || free_factor >= 0.5 ||
        occupied_factor <= 0.5 || occupied_factor >= 1 || !finite(distance) ||
        !finite(angle) || distance < 0 || angle < 0) {
      throw std::invalid_argument("Invalid map, scan, update or timeout parameters; see README.md");
    }
    for (const auto &frame : {map_frame_, base_frame_, odom_frame_, scanmatch_frame_}) {
      if (frame.empty() || frame.front() == '/') throw std::invalid_argument("TF frames must be nonempty, without leading '/'");
    }
    if (map_frame_ == base_frame_ || map_frame_ == odom_frame_ ||
        (pub_scanmatch_tf_ && (scanmatch_frame_ == base_frame_ ||
         scanmatch_frame_ == odom_frame_ || scanmatch_frame_ == map_frame_))) {
      throw std::invalid_argument("TF frame configuration would create a conflict");
    }
    if (known_poses_ && !use_pose_tf_) throw std::invalid_argument("map_with_known_poses requires use_tf_pose_start_estimate");
    if (known_poses_ && (pub_tf_ || pub_scanmatch_tf_)) throw std::invalid_argument("Known-pose mapping requires TF publishing disabled");

    if(localization_only_ && (known_poses_ || use_pose_tf_ || (odom_frame_!=base_frame_ && !use_odom_hint_ && !odom_tf_only_)))
      throw std::invalid_argument("Fixed-map localization requires explicit odometry hint mode for an odom frame");
    if(odom_tf_only_ && (odom_frame_==base_frame_ || !pub_tf_ || !localization_only_ || use_pose_tf_))
      throw std::invalid_argument("Transform-only odometry requires fixed-map mode and distinct odom frame");
    if(use_odom_hint_ && (odom_frame_==base_frame_ || !pub_tf_ || use_pose_tf_ || !localization_only_))
      throw std::invalid_argument("Odometry hint requires fixed-map mode, distinct odom frame, TF output and no external map pose TF");
    if (localization_only_) {
      resolution_=map_.info.resolution;
      const int stride=1 << (levels-1);
      const int width=(map_.info.width+stride-1)/stride*stride;
      const int height=(map_.info.height+stride-1)/stride*stride;
      if(width/stride<8 || height/stride<8) throw std::invalid_argument("Prior map too small for configured pyramid");
      processor_=std::make_unique<hectorslam::HectorSlamProcessor>(resolution_,width,height,Eigen::Vector2f::Zero(),levels);
      processor_->loadStaticMap(map_.data,map_.info.width,map_.info.height);
      prior_fingerprint_=coreFingerprint();
      map_.header.frame_id=map_frame_;
      RCLCPP_INFO(get_logger(),"Fixed prior loaded: %u x %u, %.4f m/cell; ALL map updates disabled",map_.info.width,map_.info.height,resolution_);
    } else {
      processor_ = std::make_unique<hectorslam::HectorSlamProcessor>(resolution_, size, size,
        Eigen::Vector2f(start_x, start_y), levels);
    }
    processor_->setUpdateFactorFree(free_factor);
    processor_->setUpdateFactorOccupied(occupied_factor);
    processor_->setMapUpdateMinDistDiff(distance);
    processor_->setMapUpdateMinAngleDiff(angle);
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos);
    metadata_pub_ = create_publisher<nav_msgs::msg::MapMetaData>(map_topic + "_metadata", map_qos);
    localization_status_pub_=create_publisher<std_msgs::msg::String>("localization_status",10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("slam_out_pose", 10);
    cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10);
    if (pub_odom) odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("scanmatch_odom", 10);
    if(!use_imu_ || use_odom_hint_ || use_pose_tf_ || odom_frame_!=base_frame_)
      throw std::invalid_argument("This b0 cloud adapter requires exact reference attitude and no external odometry");
    reference_sub_=create_subscription<sensor_msgs::msg::Imu>("/deskew_reference_attitude",1000,[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
      Eigen::Quaterniond q(m->orientation.w,m->orientation.x,m->orientation.y,m->orientation.z);
      if(m->header.frame_id!=base_frame_ || m->orientation_covariance[0]<0 || !q.coeffs().allFinite() || std::abs(q.norm()-1)>.001)return;
      references_[rclcpp::Time(m->header.stamp).nanoseconds()]=q.normalized();
      while(references_.size()>2000)references_.erase(references_.begin());
      processClouds();
    });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/points_carto",rclcpp::SensorDataQoS().keep_last(queue),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
      if(m->header.frame_id!=base_frame_)throw std::runtime_error("Deskew cloud must be in base frame");
      if(clouds_.size()>=200)throw std::runtime_error("Reference attitude/cloud pairing queue overflow");
      clouds_.push_back(m);processClouds();
    });
    cloud_status_pub_=create_publisher<std_msgs::msg::String>("/cloud_input_status",1000);
    matching_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/hector_matching_points",rclcpp::SensorDataQoS());
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        try {
          if (pose.header.frame_id.empty()) throw std::invalid_argument("initialpose frame_id is empty");
          if (pose.header.frame_id != map_frame_) pose = buffer_.transform(pose, map_frame_, tf2::durationFromSec(tf_timeout_));
          setInitialPose(pose.pose);
        } catch (const std::exception &e) { RCLCPP_WARN(get_logger(), "Rejected initialpose: %s", e.what()); }
      });
    command_sub_ = create_subscription<std_msgs::msg::String>(sys_topic, 2,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { if (msg->data == "reset") reset(); });
    if (parameter("advertise_map_service", true)) {
      map_srv_ = create_service<nav_msgs::srv::GetMap>("dynamic_map",
        [this](const std::shared_ptr<nav_msgs::srv::GetMap::Request>, std::shared_ptr<nav_msgs::srv::GetMap::Response> res) {
          refreshMap(); res->map = map_;
        });
    }
    reset_srv_ = create_service<std_srvs::srv::Trigger>("reset_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        reset(); res->success = true; res->message = "Map and pose reset";
      });
    pause_srv_ = create_service<std_srvs::srv::SetBool>("pause_mapping",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        paused_ = req->data; res->success = true; res->message = paused_ ? "Mapping paused" : "Mapping resumed";
      });
    restart_srv_ = create_service<hector_mapping::srv::ResetMapping>("restart_mapping_with_new_pose",
      [this](const std::shared_ptr<hector_mapping::srv::ResetMapping::Request> req,
             std::shared_ptr<hector_mapping::srv::ResetMapping::Response>) {
        // Validate before changing state. The upstream service has an empty response.
        try {
          validatePose(req->initial_pose);
          reset(); setInitialPose(req->initial_pose); paused_ = false;
        } catch (const std::exception &e) {
          RCLCPP_WARN(get_logger(), "Rejected restart pose: %s", e.what());
        }
      });
    pending_timer_ = create_wall_timer(10ms, [this] { processClouds(); processPendingScans(); });
    map_.info.map_load_time = now();
    map_timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] { publishMap(); });
    publishMap();
    RCLCPP_INFO(get_logger(), "Hector XYZ mapping ready (shared reference attitude): debug scan=%s, TF=%s -> %s, odometry input %s",
      scan_topic.c_str(), map_frame_.c_str(), odom_frame_.c_str(),
      odom_frame_ == base_frame_ ? "not required" : "required for map->odom TF");
  }

private:
  void processClouds(){
    while(!clouds_.empty()){
      const auto cloud=clouds_.front();const auto t=rclcpp::Time(cloud->header.stamp).nanoseconds();
      const auto it=references_.find(t);
      if(it==references_.end()) {
        if(!references_.empty() && references_.rbegin()->first-t>500000000LL) {
          ++unpaired_clouds_;clouds_.pop_front();
          RCLCPP_WARN(get_logger(),"Dropping unpaired cloud stamp=%ld: exact reference attitude missing; total=%lu",t,unpaired_clouds_);
          continue;
        }
        return;
      }
      const auto q=it->second;clouds_.pop_front();sensor_msgs::msg::LaserScan scan;scan.header=cloud->header;
      onScan(scan,cloud.get(),&q);references_.erase(references_.begin(),references_.upper_bound(t));
    }
  }
  uint64_t unpaired_clouds_=0;
  double voxel_size_;
  Eigen::Vector3d ray_origin_;
  std::map<int64_t,Eigen::Quaterniond> references_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr reference_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cloud_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr matching_pub_;
  uint64_t coreFingerprint() const {
    uint64_t hash=14695981039346656037ULL;
    for(int level=0;level<processor_->getMapLevels();++level) {
      const auto& grid=processor_->getGridMap(level);
      for(int i=0;i<grid.getSizeX()*grid.getSizeY();++i) {
        uint32_t bits; const float value=grid.getCell(i).getValue();
        std::memcpy(&bits,&value,sizeof(bits));
        hash^=bits;hash*=1099511628211ULL;
      }
    }
    return hash;
  }
  uint64_t prior_fingerprint_=0;
  Eigen::Vector3f toPrior(const Eigen::Vector3f& pose) const {
    if(!localization_only_) return pose;
    const Eigen::Vector2f p=Eigen::Rotation2Df(-prior_yaw_)*(pose.head<2>()-prior_origin_);
    return Eigen::Vector3f(p.x(),p.y(),pose.z()-prior_yaw_);
  }
  Eigen::Vector3f toMap(const Eigen::Vector3f& pose) const {
    if(!localization_only_) return pose;
    const Eigen::Vector2f p=prior_origin_+Eigen::Rotation2Df(prior_yaw_)*pose.head<2>();
    return Eigen::Vector3f(p.x(),p.y(),pose.z()+prior_yaw_);
  }
  bool localization_only_=false, global_relocalization_=false, needs_relocalization_=true;
  int rejected_=0;
  double prior_yaw_=0, search_radius_, search_yaw_, min_score_, max_step_;
  Eigen::Vector2f prior_origin_=Eigen::Vector2f::Zero();
  std::unique_ptr<hector_mapping_ros2::PriorMatcher> prior_matcher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localization_status_pub_;
  template<typename T> T parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  static void validatePose(const geometry_msgs::msg::Pose &pose) {
    const auto &q = pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(norm) || norm < 1e-12) throw std::invalid_argument("Pose must be finite with a nonzero quaternion");
  }
  void setInitialPose(const geometry_msgs::msg::Pose &pose) {
    validatePose(pose);
    tf2::Quaternion q; tf2::fromMsg(pose.orientation, q); q.normalize();
    initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, tf2::getYaw(q));
    has_initial_ = true; needs_relocalization_=true; last_imu_yaw_.reset(); odometry_prior_.reset();
  }
  void reset() {
    processor_->reset(); odometry_prior_.reset(); has_initial_ = false; needs_relocalization_=true; rejected_=0; last_imu_yaw_.reset(); imu_filter_.reset(); pending_scans_.clear();clouds_.clear();references_.clear();
    last_scan_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    map_.info.map_load_time = now(); dirty_ = true; publishMap();
  }
  void refreshMap() {
    if(localization_only_) {
      if(coreFingerprint()!=prior_fingerprint_) throw std::runtime_error("IMMUTABILITY VIOLATION: Hector prior pyramid changed");
      map_.header.stamp=last_scan_.nanoseconds()?last_scan_:now(); return;
    }
    const auto &grid = processor_->getGridMap();
    if (dirty_ || map_index_ != grid.getUpdateIndex()) {
      map_.header.frame_id = map_frame_;
      map_.info.resolution = grid.getCellLength();
      map_.info.width = grid.getSizeX(); map_.info.height = grid.getSizeY();
      // Grid coordinates denote cell centres; OccupancyGrid origin is a corner.
      const Eigen::Vector2f origin = grid.getWorldCoords(Eigen::Vector2f::Zero());
      map_.info.origin.position.x = origin.x() - resolution_ * 0.5;
      map_.info.origin.position.y = origin.y() - resolution_ * 0.5;
      map_.info.origin.orientation.w = 1.0;
      map_.data.assign(static_cast<size_t>(grid.getSizeX()) * grid.getSizeY(), -1);
      for (size_t i = 0; i < map_.data.size(); ++i) {
        if (grid.isOccupied(i)) map_.data[i] = 100;
        else if (grid.isFree(i)) map_.data[i] = 0;
      }
      dirty_ = false; map_index_ = grid.getUpdateIndex();
    }
    map_.header.stamp = last_scan_.nanoseconds() ? last_scan_ : now();
  }
  void publishMap() { refreshMap(); map_pub_->publish(map_); metadata_pub_->publish(map_.info); }
  void onImu(const sensor_msgs::msg::Imu &msg) {
    if (msg.angular_velocity_covariance[0] < 0) return;
    try {
      const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
      const auto transform=buffer_.lookupTransform(base_frame_, msg.header.frame_id, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
      tf2::Quaternion base_q_imu; tf2::fromMsg(transform.transform.rotation, base_q_imu);
      const auto rotate=[&](const geometry_msgs::msg::Vector3 &v) {
        const auto value=tf2::quatRotate(base_q_imu,tf2::Vector3(v.x,v.y,v.z));
        return Eigen::Vector3d(value.x(),value.y(),value.z());
      };
      const auto gyro=rotate(msg.angular_velocity);
      auto acceleration=rotate(msg.linear_acceleration);
      if (msg.linear_acceleration_covariance[0] < 0) acceleration.setConstant(NAN);
      std::optional<Eigen::Quaterniond> orientation;
      if (msg.orientation_covariance[0] >= 0) {
        tf2::Quaternion q; tf2::fromMsg(msg.orientation,q);
        if (q.length2()>1e-12 && std::isfinite(q.length2())) {
          q.normalize(); q=q*base_q_imu.inverse();
          orientation=Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z());
        }
      }
      if (!imu_filter_.add(stamp.seconds(),gyro,acceleration,imu_gravity_,imu_time_constant_,orientation)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU sample rejected: invalid data, time order, or no gravity initialization");
      } else { processPendingScans(); }
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU transform unavailable: %s", e.what());
    }
  }
  void queueScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    if (paused_) return;
    if (!use_imu_ && !(pub_tf_ && odom_frame_!=base_frame_)) { onScan(*scan); return; }
    if (pending_scans_.size() >= 200) {
      RCLCPP_WARN(get_logger(), "Pending scan queue full: dropping oldest scan");
      pending_scans_.pop_front();
    }
    pending_scans_.push_back({scan,std::chrono::steady_clock::now()});
    processPendingScans();
  }
  void processPendingScans() {
    while (!pending_scans_.empty()) {
      const auto &pending=pending_scans_.front();
      const double time=rclcpp::Time(pending.scan->header.stamp).seconds();
      // Wait for the same preceding/following IMU samples on every replay.
      // Never choose interpolation vs extrapolation based on callback arrival.
      if (use_imu_ && !imu_filter_.at(time,imu_tolerance_,false)) {
        const auto latest=imu_filter_.latestTime();
        const bool expired=std::chrono::steady_clock::now()-pending.arrival>500ms;
        if (!expired && (!latest || *latest<time)) break;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Scan dropped: no bracketing IMU samples within synchronization limits");
        pending_scans_.pop_front(); continue;
      }
      if(pub_tf_ && odom_frame_!=base_frame_ && !buffer_.canTransform(odom_frame_,base_frame_,
          rclcpp::Time(pending.scan->header.stamp,get_clock()->get_clock_type()),tf2::durationFromSec(0))) {
        if(std::chrono::steady_clock::now()-pending.arrival<500ms) break;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Scan dropped: EKF odometry does not bracket scan timestamp");
        pending_scans_.pop_front(); continue;
      }
      const auto scan=pending.scan;
      pending_scans_.pop_front();
      onScan(*scan);
    }
  }
  void onScan(const sensor_msgs::msg::LaserScan &scan, const sensor_msgs::msg::PointCloud2* cloud=nullptr, const Eigen::Quaterniond* reference=nullptr) {
    if (paused_) return;
    const auto start = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(scan.header.stamp, get_clock()->get_clock_type());
    if (last_scan_.nanoseconds() && stamp <= last_scan_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping out-of-order scan; call reset_map after rewinding a bag"); return;
    }
    tf2::Transform base_from_laser; base_from_laser.setIdentity();
    tf2::Transform odom_from_base; odom_from_base.setIdentity();
    if(localization_only_ && needs_relocalization_ && !has_initial_ && !last_scan_.nanoseconds() && !global_relocalization_) return;
    Eigen::Vector3f hint = has_initial_ ? toPrior(initial_pose_) : processor_->getLastScanMatchPose();
    try {
      if (use_scan_tf_ && !cloud) {
        tf2::fromMsg(buffer_.lookupTransform(base_frame_, scan.header.frame_id, stamp,
          rclcpp::Duration::from_seconds(tf_timeout_)).transform, base_from_laser);
      }
      if (pub_tf_ && odom_frame_ != base_frame_) {
        tf2::fromMsg(buffer_.lookupTransform(odom_frame_, base_frame_, stamp,
          rclcpp::Duration::from_seconds(tf_timeout_)).transform, odom_from_base);
      }
      if(use_odom_hint_ && !has_initial_) {
        const auto prediction=odometry_prior_.predict(odom_from_base);
        if(prediction) hint=toPrior(*prediction);
      }
      if (use_pose_tf_ && !has_initial_) {
        const auto transform = buffer_.lookupTransform(map_frame_, base_frame_, stamp,
          rclcpp::Duration::from_seconds(tf_timeout_));
        hint = Eigen::Vector3f(transform.transform.translation.x, transform.transform.translation.y,
          tf2::getYaw(transform.transform.rotation));
      }
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Scan skipped: %s", e.what()); return;
    }
    tf2::Quaternion tilt(0,0,0,1);
    std::optional<double> imu_yaw;
    if (use_imu_) {
      const auto attitude=reference ? std::optional<Eigen::Quaterniond>(*reference) : imu_filter_.at(stamp.seconds(),imu_tolerance_,false);
      if (!attitude) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Scan skipped: no synchronized IMU attitude"); return;
      }
      imu_yaw=hector_mapping_ros2::ImuOrientationFilter::yaw(*attitude);
      const Eigen::Quaterniond tilt_q=hector_mapping_ros2::ImuOrientationFilter::heading(-*imu_yaw)*(*attitude);
      tilt=tf2::Quaternion(tilt_q.x(),tilt_q.y(),tilt_q.z(),tilt_q.w());
      // Match in a gravity-aligned plane; retain full attitude in published TF.
      base_from_laser=tf2::Transform(tilt,tf2::Vector3(0,0,0))*base_from_laser;
      if (last_imu_yaw_ && !has_initial_ && !use_pose_tf_ && !use_odom_hint_) {
        hint.z() += std::atan2(std::sin(*imu_yaw-*last_imu_yaw_),std::cos(*imu_yaw-*last_imu_yaw_));
      }
    }
    bool converted=false;
    if(cloud && reference) {
      const auto before=cloud->width*cloud->height;
      const auto qtilt=hector_mapping_ros2::ImuOrientationFilter::heading(-*imu_yaw)*(*reference);
      cartographer::sensor::PointCloud gravity_points;
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z");
      for(size_t i=0;i<before;++i,++x,++y,++z){
        Eigen::Vector3d point(*x,*y,*z);
        if(!point.allFinite())continue;
        point=qtilt*point;
        if(point.z()<z_min_ || point.z()>z_max_)continue;
        gravity_points.push_back({point.cast<float>()});
      }
      const auto cropped=gravity_points.size();
      const auto filtered=cartographer::sensor::VoxelFilter(gravity_points,voxel_size_);
      data_.clear();data_.setOrigo((qtilt*ray_origin_).head<2>().cast<float>()*processor_->getScaleToMap());
      sensor_msgs::msg::PointCloud2 used;used.header=cloud->header;
      sensor_msgs::PointCloud2Modifier modifier(used);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(filtered.size());
      sensor_msgs::PointCloud2Iterator<float> ux(used,"x"),uy(used,"y"),uz(used,"z");
      for(const auto&point:filtered){
        data_.add(point.position.head<2>()*processor_->getScaleToMap());
        // Undo tilt in the message frame: map->base carries tilt, so RViz sees
        // exactly the flattened XY points used by Hector, on the map plane.
        const Eigen::Vector3d body=qtilt.conjugate()*Eigen::Vector3d(point.position.x(),point.position.y(),0);
        *ux=body.x();*uy=body.y();*uz=body.z();++ux;++uy;++uz;
      }
      used.is_dense=true;matching_pub_->publish(used);
      std_msgs::msg::String status;std::ostringstream os;
      os<<"{\"stamp_ns\":"<<stamp.nanoseconds()<<",\"reference_stamp_ns\":"<<stamp.nanoseconds()<<",\"raw_points\":"<<before<<",\"cropped_points\":"<<cropped<<",\"voxel_points\":"<<filtered.size()<<",\"unpaired_clouds\":"<<unpaired_clouds_<<"}";
      status.data=os.str();cloud_status_pub_->publish(status);
      converted=data_.getSize()>=min_points_;
    } else converted=hector_mapping_ros2::convertScan(scan,base_from_laser,processor_->getScaleToMap(),min_range_,max_range_,z_min_,z_max_,data_);
    if(!converted || data_.getSize()<min_points_){RCLCPP_WARN(get_logger(),"Invalid/insufficient cloud");return;}
    if(localization_only_) {
      const auto candidate=needs_relocalization_
        ? prior_matcher_->relocalize(*processor_,data_,hint,search_radius_,search_yaw_,global_relocalization_)
        : processor_->matchStatic(data_,hint);
      const double score=prior_matcher_->score(data_,candidate);
      const double step=(candidate.head<2>()-hint.head<2>()).norm();
      const double yaw_step=std::abs(std::atan2(std::sin(candidate.z()-hint.z()),std::cos(candidate.z()-hint.z())));
      const bool enforce_score=get_parameter("enforce_localization_score").as_bool();
      const bool accepted=candidate.allFinite() && (!enforce_score || score>=min_score_) &&
        (needs_relocalization_ || (step<=max_step_ && yaw_step<=0.6));
      std_msgs::msg::String status;
      std::ostringstream json;
      json<<"{\"stamp\":"<<std::fixed<<std::setprecision(9)<<stamp.seconds()
          <<",\"score\":"<<score<<",\"accepted\":"<<(accepted?"true":"false")
          <<",\"score_below_threshold\":"<<(score<min_score_?"true":"false")
          <<",\"score_gate_enforced\":"<<(enforce_score?"true":"false")
          <<",\"relocalizing\":"<<(needs_relocalization_?"true":"false")<<",\"points\":"<<data_.getSize()<<"}";
      status.data=json.str(); localization_status_pub_->publish(status);
      if(!accepted) {
        last_scan_=stamp;
        if(++rejected_>=3) needs_relocalization_=true;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Prior match rejected: score=%.3f, step=%.3f",score,step);
        return;
      }
      // Matching from an already refined seed may move again; commit the validated result directly.
      processor_->commitStaticPose(candidate,data_);
      needs_relocalization_=false; rejected_=0;
    } else processor_->update(data_, hint, known_poses_);
    has_initial_ = false; last_scan_ = stamp; last_imu_yaw_ = imu_yaw;
    const auto &grid = processor_->getGridMap();
    const Eigen::Vector2f cell = grid.getMapCoords(processor_->getLastScanMatchPose().head<2>());
    const double clearance = std::min({cell.x(), cell.y(),
      grid.getSizeX()-1.0f-cell.x(), grid.getSizeY()-1.0f-cell.y()}) * resolution_;
    if (!localization_only_ && clearance < max_range_ + data_.getOrigo().norm()/processor_->getScaleToMap()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "Map boundary within laser range: pose clearance %.2f m, laser range %.2f m; enlarge map_size or adjust map_start_x/y",
        clearance, max_range_);
    }
    if (!known_poses_) {
      const auto pose = toMap(processor_->getLastScanMatchPose());
      geometry_msgs::msg::PoseStamped out;
      out.header.stamp = stamp; out.header.frame_id = map_frame_;
      out.pose.position.x = pose.x(); out.pose.position.y = pose.y();
      tf2::Quaternion q; q.setRPY(0, 0, pose.z()); if(odom_frame_==base_frame_) q = q * tilt; q.normalize(); out.pose.orientation = tf2::toMsg(q);
      pose_pub_->publish(out);
      geometry_msgs::msg::PoseWithCovarianceStamped with_cov;
      with_cov.header = out.header; with_cov.pose.pose = out.pose;
      with_cov.pose.covariance[14] = with_cov.pose.covariance[21] = with_cov.pose.covariance[28] = 1e6;
      Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
      if(localization_only_) rotation.block<2,2>(0,0)=Eigen::Rotation2Dd(prior_yaw_).toRotationMatrix();
      const Eigen::Matrix3d covariance=rotation*hector_mapping_ros2::poseCovariance(processor_->getLastScanMatchCovariance(), resolution_)*rotation.transpose();
      constexpr int axes[] = {0, 1, 5};
      for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
        with_cov.pose.covariance[axes[r]*6 + axes[c]] = covariance(r, c);
      cov_pub_->publish(with_cov);
      if (odom_pub_) {
        nav_msgs::msg::Odometry odom;
        odom.header = out.header; odom.child_frame_id = base_frame_; odom.pose = with_cov.pose;
        for (int i = 0; i < 6; ++i) odom.twist.covariance[7*i] = 1e6;
        odom_pub_->publish(odom);
      }
      tf2::Transform map_from_base(q, tf2::Vector3(pose.x(), pose.y(), 0));
      const auto map_from_odom=use_odom_hint_ ? odometry_prior_.correct(map_from_base,odom_from_base)
        : map_from_base*odom_from_base.inverse();
      if (pub_tf_) sendTransform(map_from_odom,stamp+rclcpp::Duration::from_seconds(transform_tolerance_),odom_frame_);
      if (pub_scanmatch_tf_) sendTransform(map_from_base, stamp, scanmatch_frame_);
    }
    if (timing_) RCLCPP_INFO(get_logger(), "Scan: %d points, %.3f ms", data_.getSize(),
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }
  void sendTransform(const tf2::Transform &transform, const rclcpp::Time &stamp, const std::string &child) {
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = stamp; out.header.frame_id = map_frame_; out.child_frame_id = child;
    out.transform = tf2::toMsg(transform); broadcaster_.sendTransform(out);
  }
  struct PendingScan {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    std::chrono::steady_clock::time_point arrival;
  };
  std::deque<PendingScan> pending_scans_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  bool use_imu_, use_odom_hint_, odom_tf_only_;
  double transform_tolerance_;
  hector_mapping_ros2::OdometryPosePrior odometry_prior_;
  double imu_tolerance_, imu_gravity_, imu_time_constant_;
  hector_mapping_ros2::ImuOrientationFilter imu_filter_;
  std::optional<double> last_imu_yaw_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::unique_ptr<hectorslam::HectorSlamProcessor> processor_;
  hectorslam::DataContainer data_;
  std::string map_frame_, base_frame_, odom_frame_, scanmatch_frame_;
  double resolution_, min_range_, max_range_, z_min_, z_max_, tf_timeout_;
  int min_points_, map_index_ = -100;
  bool pub_tf_, pub_scanmatch_tf_, use_scan_tf_, use_pose_tf_, known_poses_, timing_;
  bool paused_ = false, has_initial_ = false, dirty_ = true;
  Eigen::Vector3f initial_pose_ = Eigen::Vector3f::Zero();
  rclcpp::Time last_scan_;
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr metadata_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cov_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp::Service<hector_mapping::srv::ResetMapping>::SharedPtr restart_srv_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    // All map/scan/service callbacks are serialized; TF listener has its own thread.
    rclcpp::spin(std::make_shared<HectorMappingNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("hector_mapping"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
