// ROS 2 I/O adapter reproducing pinned HectorMappingRos.cpp scan processing.
// The upstream hector_slam_lib headers are compiled byte-for-byte unchanged.
#include <iostream>
#include <cfloat>
#include <chrono>
#include <deque>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <hector_slam_lib/slam_main/HectorSlamProcessor.h>
using namespace std::chrono_literals;
class UpstreamBehavior:public rclcpp::Node{
public:
 UpstreamBehavior():Node("hector_upstream_behavior"),buffer_(get_clock()),listener_(buffer_){
  base_=declare_parameter<std::string>("base_frame","base_link");
  map_frame_=declare_parameter<std::string>("map_frame","map");
  double resolution=declare_parameter("map_resolution",.05);
  int size=declare_parameter("map_size",4096);
  h_=std::make_unique<hectorslam::HectorSlamProcessor>(resolution,size,size,Eigen::Vector2f(.5,.5),declare_parameter("map_multi_res_levels",3));
  h_->setUpdateFactorFree(.4);h_->setUpdateFactorOccupied(.9);
  h_->setMapUpdateMinDistDiff(declare_parameter("map_update_distance_thresh",.2));
  h_->setMapUpdateMinAngleDiff(declare_parameter("map_update_angle_thresh",.08));
  const double min=declare_parameter("laser_min_dist",.5);min2_=min*min;max2_=30.*30.;
  map_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(1).transient_local().reliable());
  pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("pose",1000);
  status_pub_=create_publisher<std_msgs::msg::String>("input_status",1000);
  scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/upstream_scan",rclcpp::SensorDataQoS().keep_last(declare_parameter("scan_subscriber_queue_size",1000)),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr s){pending_.push_back({s,std::chrono::steady_clock::now()});drain();});
  timer_=create_wall_timer(5ms,[this]{drain();});map_timer_=create_wall_timer(2s,[this]{publishMap();});publishMap();
 }
private:
 void drain(){
  while(!pending_.empty()){
   const auto p=pending_.front();const auto&s=*p.scan;
   const rclcpp::Time t(s.header.stamp,get_clock()->get_clock_type());
   if(!buffer_.canTransform(base_,s.header.frame_id,t,0s)){
    if(std::chrono::steady_clock::now()-p.arrival<500ms)return;
    RCLCPP_ERROR(get_logger(),"Input dropped: TF does not cover scan time");++dropped_;pending_.pop_front();continue;
   }
   pending_.pop_front();process(s);
  }
 }
 void process(const sensor_msgs::msg::LaserScan&s){
  const rclcpp::Time stamp(s.header.stamp,get_clock()->get_clock_type());
  tf2::Transform laser_transform;tf2::fromMsg(buffer_.lookupTransform(base_,s.header.frame_id,stamp,0s).transform,laser_transform);
  sensor_msgs::msg::PointCloud2 cloud;projector_.projectLaser(s,cloud,30.0);
  hectorslam::DataContainer data;
  const tf2::Vector3 laserPos=laser_transform.getOrigin();const float scale=h_->getScaleToMap();
  data.setOrigo(Eigen::Vector2f(laserPos.x(),laserPos.y())*scale);
  sensor_msgs::PointCloud2ConstIterator<float>x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
  for(size_t i=0;i<cloud.width*cloud.height;++i,++x,++y,++z){
   // Keep upstream order, strict inequalities, rear near-field removal,
   // TF transform, relative-height [-1,1], and lack of voxel filtering.
   const float dist_sqr=*x * *x+*y * *y;
   if(dist_sqr>min2_ && dist_sqr<max2_){
    if(*x<0.f && dist_sqr<.50f)continue;
    const tf2::Vector3 point=laser_transform*tf2::Vector3(*x,*y,*z);
    const float relative_z=point.z()-laserPos.z();
    if(relative_z>-1.f && relative_z<1.f)data.add(Eigen::Vector2f(point.x(),point.y())*scale);
   }
  }
  // Exact upstream default initial estimate: previous matched pose, no gyro yaw.
  h_->update(data,h_->getLastScanMatchPose());
  const auto pose=h_->getLastScanMatchPose();if(!pose.allFinite())throw std::runtime_error("Nonfinite upstream result");
  geometry_msgs::msg::PoseStamped out;out.header=s.header;out.header.frame_id=map_frame_;
  out.pose.position.x=pose.x();out.pose.position.y=pose.y();out.pose.orientation.z=std::sin(.5*pose.z());out.pose.orientation.w=std::cos(.5*pose.z());pose_pub_->publish(out);
  last_stamp_=s.header.stamp;
  std_msgs::msg::String status;status.data="{\"stamp_ns\":"+std::to_string(stamp.nanoseconds())+",\"input_beams\":"+std::to_string(s.ranges.size())+",\"projected_points\":"+std::to_string(cloud.width*cloud.height)+",\"matching_points\":"+std::to_string(data.getSize())+",\"tf_drops\":"+std::to_string(dropped_)+"}";status_pub_->publish(status);
 }
 void publishMap(){
  const auto&grid=h_->getGridMap();if(map_index_!=grid.getUpdateIndex()){
   map_.header.frame_id=map_frame_;map_.info.resolution=grid.getCellLength();map_.info.width=grid.getSizeX();map_.info.height=grid.getSizeY();
   // Preserve upstream's map origin convention for this behavior baseline.
   const Eigen::Vector2f origin=grid.getWorldCoords(Eigen::Vector2f::Zero());
   map_.info.origin.position.x=origin.x()-grid.getCellLength()/1000*.5f;map_.info.origin.position.y=origin.y()-grid.getCellLength()/1000*.5f;map_.info.origin.orientation.w=1;
   map_.data.resize(size_t(grid.getSizeX())*grid.getSizeY());
   for(size_t i=0;i<map_.data.size();++i)map_.data[i]=grid.isFree(i)?0:(grid.isOccupied(i)?100:-1);
   map_index_=grid.getUpdateIndex();
  }
  map_.header.stamp=last_stamp_;map_pub_->publish(map_);
 }
 struct Pending{sensor_msgs::msg::LaserScan::ConstSharedPtr scan;std::chrono::steady_clock::time_point arrival;};
 std::deque<Pending>pending_;std::unique_ptr<hectorslam::HectorSlamProcessor>h_;laser_geometry::LaserProjection projector_;
 tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;std::string base_,map_frame_;double min2_,max2_;int map_index_=-100;size_t dropped_=0;
 builtin_interfaces::msg::Time last_stamp_;nav_msgs::msg::OccupancyGrid map_;
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
 rclcpp::TimerBase::SharedPtr timer_,map_timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<UpstreamBehavior>());}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("upstream"),"%s",e.what());return 1;}rclcpp::shutdown();}
