// Adapted from the existing rotation_deskew/src/deskew.cpp for one MultiEcho laser.
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>
#include <limits>
#include <sstream>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <Eigen/Geometry>
#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace cc = cartographer::common;
using Scan = sensor_msgs::msg::LaserScan;
int64_t ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec)*1000000000LL+t.nanosec; }
cc::Time ct(int64_t t) {return cc::FromUniversal(t/100+cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL);}
struct Attitude {int64_t t; Eigen::Quaterniond q;};
struct Velocity {int64_t t; Eigen::Vector3d v_map; Eigen::Quaterniond map_orientation;};

class Deskew : public rclcpp::Node {
 public:
  Deskew() : Node("carto_rotation_deskew"), buffer_(get_clock()), listener_(buffer_) {
    single_=declare_parameter<bool>("single_scan",true);
    pose_frame_=declare_parameter<std::string>("historical_pose_frame","map");
    frame_=declare_parameter<std::string>("tracking_frame","base_link");
    max_skew_=declare_parameter<double>("max_pair_skew_sec",.1);
    max_gap_=declare_parameter<double>("max_imu_gap_sec",.03);
    gravity_=declare_parameter<double>("imu_gravity_time_constant",10.);
    front_offset_=declare_parameter<double>("front_stamp_offset_sec",0.);
    back_offset_=declare_parameter<double>("back_stamp_offset_sec",0.);
    compensate_=declare_parameter<bool>("compensate_rotation",true);
    crop_stamp_=declare_parameter<bool>("crop_stamp_compensation",false);
    publish_scan_=declare_parameter<bool>("publish_scan",false);
    translation_=declare_parameter<bool>("compensate_translation",false);
    max_pose_age_=declare_parameter<double>("max_pose_age_sec",.5);
    max_speed_=declare_parameter<double>("max_translation_speed",2.0);
    max_jump_=declare_parameter<double>("max_pose_jump_m",.4);
    const auto pose_topic=declare_parameter<std::string>("historical_pose_topic","/amcl_pose");
    amcl_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic,100,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){amclPose(m);});
    initial_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr){resetVelocity();});
    reset_=create_service<std_srvs::srv::Trigger>("/deskew/reset_velocity",[this](
        std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){
      resetVelocity();r->success=true;r->message="Historical pose velocity cleared";});
    attitude_pub_=create_publisher<sensor_msgs::msg::Imu>("/deskew_reference_attitude",1000);
    raw_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/points_carto_raw",rclcpp::SensorDataQoS());
    min_=declare_parameter<double>("min_range",.1); max_=declare_parameter<double>("max_range",30.);
    if(max_skew_<=0 || max_gap_<=0 || gravity_<=0 || min_<0 || max_<=min_) throw std::runtime_error("Invalid deskew parameters");
    auto qos=rclcpp::SensorDataQoS().keep_last(1000);
    raw_scan_pub_=create_publisher<Scan>("/scan",qos);
    status_pub_=create_publisher<std_msgs::msg::String>("/deskew_status",1000);
    multi_=create_subscription<sensor_msgs::msg::MultiEchoLaserScan>("/horizontal_laser_2d",qos,[this](sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr m){
      auto s=std::make_shared<Scan>();s->header=m->header;s->angle_min=m->angle_min;s->angle_max=m->angle_max;s->angle_increment=m->angle_increment;s->time_increment=m->time_increment;s->scan_time=m->scan_time;s->range_min=m->range_min;s->range_max=m->range_max;
      for(size_t i=0;i<m->ranges.size();++i){s->ranges.push_back(m->ranges[i].echoes.empty()?std::numeric_limits<float>::infinity():m->ranges[i].echoes.front());s->intensities.push_back(i<m->intensities.size()&&!m->intensities[i].echoes.empty()?m->intensities[i].echoes.front():0.f);}
      raw_scan_pub_->publish(*s);scan(s,0);
    });
    if(!single_) front_=create_subscription<Scan>("/scan",qos,[this](Scan::ConstSharedPtr s){scan(s,0);});
    if(!single_) back_=create_subscription<Scan>("/scan_back",qos,[this](Scan::ConstSharedPtr s){scan(s,1);});
    imu_=create_subscription<sensor_msgs::msg::Imu>("/imu",qos,[this](sensor_msgs::msg::Imu::ConstSharedPtr m){imu(m);});
    pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/points_carto",rclcpp::SensorDataQoS());
    if(publish_scan_) scan_pub_=create_publisher<Scan>("/scan_carto",rclcpp::SensorDataQoS());
    timer_=create_wall_timer(std::chrono::seconds(5),[this]{
      RCLCPP_INFO(get_logger(),"front=%zu back=%zu imu=%zu clouds=%zu points=%zu dropped=%zu invalid=%zu tf_wait=%zu rotation=%s translation_clouds=%zu zero_velocity_clouds=%zu velocity_resets=%zu",
        received_[0],received_[1],imu_count_,clouds_,points_,dropped_,invalid_,tf_wait_,compensate_?"ON":"OFF",translated_,zero_velocity_,velocity_resets_);});
    RCLCPP_INFO(get_logger(),"Cartographer ImuTracker + causal historical pose constant velocity; no Cartographer matching, maps, odometry, or TF publication");
  }
 private:
  std::string pose_frame_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr attitude_pub_;
  void resetVelocity(){velocities_.clear();previous_pose_.reset();++velocity_resets_;}
  void amclPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
    if(!translation_)return;
    const auto &p=m->pose.pose.position;const auto &o=m->pose.pose.orientation;
    Eigen::Vector3d position(p.x,p.y,p.z);Eigen::Quaterniond q(o.w,o.x,o.y,o.z);
    if(m->header.frame_id!=pose_frame_ || !position.allFinite() || !q.coeffs().allFinite() || std::abs(q.norm()-1)>.01 ||
       !std::isfinite(m->pose.covariance[0]) || !std::isfinite(m->pose.covariance[7]) ||
       m->pose.covariance[0]<0 || m->pose.covariance[7]<0 ||
       std::max(m->pose.covariance[0],m->pose.covariance[7])>.25){resetVelocity();return;}
    if(!previous_pose_){previous_pose_=m;return;}
    const auto &a=previous_pose_->pose.pose;int64_t t=ns(m->header.stamp),previous=ns(previous_pose_->header.stamp);
    if(t==previous)return;
    double dt=(t-previous)*1e-9;
    if(dt<0){resetVelocity();previous_pose_=m;return;}
    if(dt<.02)return;
    Eigen::Vector3d delta=position-Eigen::Vector3d(a.position.x,a.position.y,a.position.z);delta.z()=0;
    Eigen::Quaterniond old(a.orientation.w,a.orientation.x,a.orientation.y,a.orientation.z);
    double yaw_change=2*std::acos(std::min(1.,std::abs(q.normalized().dot(old.normalized()))));
    if(dt>.5 || delta.head<2>().norm()>max_jump_ || delta.norm()/dt>max_speed_ || yaw_change/dt>3.){
      resetVelocity();previous_pose_=m;return;}
    velocities_.push_back({t,delta/dt,q.normalized()});previous_pose_=m;
    while(velocities_.size()>100)velocities_.pop_front();
  }
  Eigen::Vector3d translationVelocity(int64_t first,int64_t reference){
    if(translation_){
      for(auto it=velocities_.rbegin();it!=velocities_.rend();++it){
        if(it->t>=first)continue; // Only poses strictly older than every point in this cloud.
        if((reference-it->t)*1e-9>max_pose_age_ || it->t<history_.front().t)break;
        try{
          auto imu_to_map=it->map_orientation*orientation(it->t).conjugate();
          ++translated_;last_velocity_stamp_=it->t;
          return orientation(reference).conjugate()*(imu_to_map.conjugate()*it->v_map);
        }catch(const std::runtime_error&){break;}
      }
    }
    last_velocity_stamp_=0;++zero_velocity_;return Eigen::Vector3d::Zero();
  }
  Eigen::Isometry3d extrinsic(const std::string &source) {
    auto t=buffer_.lookupTransform(frame_,source,tf2::TimePointZero);
    auto &v=t.transform.translation; auto &q=t.transform.rotation;
    Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
    Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
    result.linear()=quat.normalized().toRotationMatrix();result.translation()=Eigen::Vector3d(v.x,v.y,v.z);return result;
  }
  void imu(sensor_msgs::msg::Imu::ConstSharedPtr m) {
    ++imu_count_;const int64_t t=ns(m->header.stamp);
    if(!history_.empty() && t<=history_.back().t){++invalid_;return;}
    Eigen::Isometry3d tf;
    try {tf=extrinsic(m->header.frame_id);} catch(const tf2::TransformException&){++tf_wait_;return;}
    if(tf.translation().norm()>1e-5){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"IMU must be colocated with tracking frame");return;}
    Eigen::Vector3d a(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z);
    Eigen::Vector3d w(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);
    a=tf.linear()*a;w=tf.linear()*w;
    if(!a.allFinite() || !w.allFinite() || a.norm()<1e-3 || m->linear_acceleration_covariance[0]<0 || m->angular_velocity_covariance[0]<0){++invalid_;return;}
    if(!tracker_) tracker_=std::make_unique<cartographer::mapping::ImuTracker>(gravity_,ct(t));
    tracker_->Advance(ct(t));tracker_->AddImuLinearAccelerationObservation(a);tracker_->AddImuAngularVelocityObservation(w);
    history_.push_back({t,tracker_->orientation()});
    while(history_.size()>2 && t-history_[1].t>5000000000LL)history_.pop_front();
    process();
  }
  void scan(Scan::ConstSharedPtr s,int index){
    ++received_[index];
    if(s->ranges.empty() || !std::isfinite(s->time_increment) || s->time_increment<=0 || !std::isfinite(s->angle_increment)){++invalid_;return;}
    auto &queue=scans_[index];
    if(!queue.empty() && ns(s->header.stamp)<=ns(queue.back()->header.stamp)){++invalid_;return;}
    queue.push_back(s);if(queue.size()>100){queue.pop_front();++dropped_;}process();
  }
  int64_t start(const Scan &s,int index){
    double offset=index?back_offset_:front_offset_;
    // Replay-only adapter for the verified driver: full scan starts at -pi,
    // ranges follow increasing acquisition angles, cropping kept the full stamp.
    if(crop_stamp_){
      if(s.angle_increment<=0)throw std::runtime_error("Crop stamp compensation requires increasing acquisition angles");
      offset+=std::max(0LL,std::llround((s.angle_min+M_PI)/s.angle_increment))*double(s.time_increment);
    }
    return ns(s.header.stamp)+int64_t(std::llround(offset*1e9));
  }
  int64_t end(const Scan &s,int index){return start(s,index)+int64_t(std::llround((s.ranges.size()-1)*double(s.time_increment)*1e9));}
  Eigen::Quaterniond orientation(int64_t t){
    auto it=std::lower_bound(history_.begin(),history_.end(),t,[](const Attitude&a,int64_t v){return a.t<v;});
    if(it==history_.end())throw std::runtime_error("IMU future coverage missing");
    if(it->t==t)return it->q;
    if(it==history_.begin())throw std::runtime_error("IMU past coverage missing");
    auto before=std::prev(it);if((it->t-before->t)*1e-9>max_gap_)throw std::runtime_error("IMU gap");
    return before->q.slerp(double(t-before->t)/double(it->t-before->t),it->q);
  }
  void process(){
    while(!scans_[0].empty() && (single_ || !scans_[1].empty()) && history_.size()>1){
      auto f=scans_[0].front(),b=single_?f:scans_[1].front();
      const int64_t fs=start(*f,0),bs=single_?fs:start(*b,1);
      if(std::abs(double(fs-bs))*1e-9>max_skew_){scans_[fs<bs?0:1].pop_front();++dropped_;continue;}
      const int64_t first=std::min(fs,bs),reference=single_?end(*f,0):std::max(end(*f,0),end(*b,1));
      if(first<history_.front().t){scans_[0].pop_front();if(!single_)scans_[1].pop_front();++dropped_;continue;}
      if(reference>history_.back().t)return;
      std::array<Eigen::Isometry3d,2> tf;
      try{tf={extrinsic(f->header.frame_id),extrinsic(b->header.frame_id)};}catch(const tf2::TransformException&){++tf_wait_;return;}
      struct Point {Eigen::Vector3d xyz;float intensity;};std::vector<Point> points,raw_points;
      try{
        const auto inverse=orientation(reference).conjugate();
        const Eigen::Vector3d velocity=translationVelocity(first,reference);
        for(int k=0;k<(single_?1:2);++k){const auto&s=k?*b:*f;const auto begin=start(s,k);
          for(size_t i=0;i<s.ranges.size();++i){
            const double r=s.ranges[i];if(!std::isfinite(r) || r<std::max(double(s.range_min),min_) || r>std::min(double(s.range_max),max_))continue;
            const double angle=s.angle_min+i*double(s.angle_increment);
            Eigen::Vector3d p=tf[k]*Eigen::Vector3d(r*std::cos(angle),r*std::sin(angle),0);
            raw_points.push_back({p,i<s.intensities.size()?s.intensities[i]:0.f});
            if(compensate_)p=inverse*(orientation(begin+int64_t(std::llround(i*double(s.time_increment)*1e9)))*p);
            if(translation_)p+=velocity*((begin+int64_t(std::llround(i*double(s.time_increment)*1e9))-reference)*1e-9);
            points.push_back({p,i<s.intensities.size()?s.intensities[i]:0.f});
          }
        }
      }catch(const std::runtime_error&){scans_[0].pop_front();if(!single_)scans_[1].pop_front();++dropped_;continue;}
      sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id=frame_;cloud.header.stamp=rclcpp::Time(reference);
      sensor_msgs::PointCloud2Modifier mod(cloud);mod.setPointCloud2Fields(4,"x",1,7,"y",1,7,"z",1,7,"intensity",1,7);mod.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float>x(cloud,"x"),y(cloud,"y"),z(cloud,"z"),intensity(cloud,"intensity");
      for(const auto&p:points){*x=p.xyz.x();*y=p.xyz.y();*z=p.xyz.z();*intensity=p.intensity;++x;++y;++z;++intensity;}
      cloud.is_dense=true;
      auto raw_cloud=cloud;
      sensor_msgs::PointCloud2Iterator<float>rx(raw_cloud,"x"),ry(raw_cloud,"y"),rz(raw_cloud,"z");
      for(const auto&p:raw_points){*rx=p.xyz.x();*ry=p.xyz.y();*rz=p.xyz.z();++rx;++ry;++rz;}
      sensor_msgs::msg::Imu attitude;attitude.header=cloud.header;
      const auto q_reference=orientation(reference);
      attitude.orientation.x=q_reference.x();attitude.orientation.y=q_reference.y();attitude.orientation.z=q_reference.z();attitude.orientation.w=q_reference.w();
      // This message carries the exact deskew reference attitude, not raw gyro data.
      attitude.angular_velocity_covariance[0]=-1;attitude.linear_acceleration_covariance[0]=-1;
      attitude_pub_->publish(attitude);
      raw_pub_->publish(raw_cloud);pub_->publish(cloud);++clouds_;points_+=points.size();
      if(publish_scan_){Scan scan;scan.header=cloud.header;scan.angle_min=-M_PI;scan.angle_max=M_PI;scan.angle_increment=M_PI/360.;scan.range_min=min_;scan.range_max=max_;scan.time_increment=0;scan.scan_time=last_reference_>0?float((reference-last_reference_)*1e-9):0.f;scan.ranges.assign(721,std::numeric_limits<float>::infinity());
        for(const auto&p:points){const auto r=std::hypot(p.xyz.x(),p.xyz.y());if(r<min_ || r>max_)continue;int bin=std::lround((std::atan2(p.xyz.y(),p.xyz.x())+M_PI)/scan.angle_increment);if(bin>=0 && bin<721)scan.ranges[bin]=std::min(scan.ranges[bin],float(r));}scan_pub_->publish(scan);}
      std::ostringstream text;text<<"{\"reference_stamp_ns\":"<<reference<<",\"source_first_stamp_ns\":"<<first<<",\"velocity_pose_stamp_ns\":"<<last_velocity_stamp_<<",\"rotation_enabled\":"<<(compensate_?"true":"false")<<",\"translation_enabled\":"<<(translation_?"true":"false")<<",\"translation_valid\":"<<(last_velocity_stamp_?"true":"false")<<",\"clouds\":"<<clouds_<<",\"dropped\":"<<dropped_<<",\"points\":"<<points.size()<<"}";
      std_msgs::msg::String status;status.data=text.str();status_pub_->publish(status);
      last_reference_=reference;
      scans_[0].pop_front();if(!single_)scans_[1].pop_front();
    }
  }
  bool single_;int64_t last_velocity_stamp_=0;
  rclcpp::Subscription<sensor_msgs::msg::MultiEchoLaserScan>::SharedPtr multi_;
  rclcpp::Publisher<Scan>::SharedPtr raw_scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  bool translation_;double max_pose_age_,max_speed_,max_jump_;
  int64_t last_reference_=0;size_t translated_=0,zero_velocity_=0,velocity_resets_=0;
  std::deque<Velocity> velocities_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr previous_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_,initial_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  std::unique_ptr<cartographer::mapping::ImuTracker>tracker_;
  std::deque<Attitude>history_;std::array<std::deque<Scan::ConstSharedPtr>,2>scans_;
  std::array<size_t,2>received_{};size_t imu_count_=0,clouds_=0,points_=0,dropped_=0,invalid_=0,tf_wait_=0;
  std::string frame_;double max_skew_,max_gap_,gravity_,front_offset_,back_offset_,min_,max_;bool compensate_,publish_scan_,crop_stamp_;
  rclcpp::Subscription<Scan>::SharedPtr front_,back_;rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;rclcpp::Publisher<Scan>::SharedPtr scan_pub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Deskew>());rclcpp::shutdown();}
