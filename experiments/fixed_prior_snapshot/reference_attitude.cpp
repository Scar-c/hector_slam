// Exact-time gravity reference for the final XYZ Hector adapter.
// Motion deskew remains exclusively the original EKF TF calculation.
#include <deque>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"

namespace cc = cartographer::common;
static cc::Time ct(int64_t t) {
  return cc::FromUniversal(t/100+cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL);
}
class ReferenceAttitude : public rclcpp::Node {
public:
  ReferenceAttitude(): Node("fixed_prior_reference_attitude"),buffer_(get_clock()),listener_(buffer_) {
    gravity_=declare_parameter<double>("imu_gravity_time_constant",10.);
    frame_=declare_parameter<std::string>("base_frame","base_link");
    if(!std::isfinite(gravity_) || gravity_<=0)throw std::invalid_argument("Invalid gravity time constant");
    pub_=create_publisher<sensor_msgs::msg::Imu>("/deskew_reference_attitude",1000);
    imu_=create_subscription<sensor_msgs::msg::Imu>("/imu",rclcpp::SensorDataQoS().keep_last(2000),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){imu(m);});
    cloud_=create_subscription<sensor_msgs::msg::PointCloud2>("/points_aligned_xyz",rclcpp::SensorDataQoS().keep_last(200),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
        if(m->header.frame_id!=frame_)throw std::runtime_error("Unexpected XYZ cloud frame");
        if(pending_.size()>=200)throw std::runtime_error("Attitude pending queue overflow");
        pending_.push_back(m->header);process();
      });
  }
private:
  void imu(sensor_msgs::msg::Imu::ConstSharedPtr m){
    const int64_t t=rclcpp::Time(m->header.stamp).nanoseconds();
    if(!history_.empty() && t<=history_.rbegin()->first)return;
    if(m->linear_acceleration_covariance[0]<0 || m->angular_velocity_covariance[0]<0)return;
    Eigen::Vector3d a(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z);
    Eigen::Vector3d w(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);
    if(!a.allFinite() || !w.allFinite() || a.norm()<1e-3)return;
    try {
      const auto tf=buffer_.lookupTransform(frame_,m->header.frame_id,tf2::TimePointZero);
      const auto&q=tf.transform.rotation;
      const Eigen::Quaterniond base_from_imu(q.w,q.x,q.y,q.z);
      a=base_from_imu*a;w=base_from_imu*w;
    }catch(const tf2::TransformException&e){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"IMU extrinsic unavailable: %s",e.what());return;
    }
    // Same tracker, gravity time constant and update order as the final b0 adapter.
    if(!tracker_)tracker_=std::make_unique<cartographer::mapping::ImuTracker>(gravity_,ct(t));
    tracker_->Advance(ct(t));
    tracker_->AddImuLinearAccelerationObservation(a);
    tracker_->AddImuAngularVelocityObservation(w);
    history_[t]=tracker_->orientation();
    process();
    while(history_.size()>2 && t-std::next(history_.begin())->first>5000000000LL)history_.erase(history_.begin());
  }
  void process(){
    while(!pending_.empty() && !history_.empty()){
      const auto header=pending_.front();const int64_t t=rclcpp::Time(header.stamp).nanoseconds();
      auto upper=history_.lower_bound(t);
      if(upper==history_.end())return;
      if(upper==history_.begin() && upper->first!=t)throw std::runtime_error("No preceding reference IMU sample");
      Eigen::Quaterniond q=upper->second;
      if(upper->first!=t){
        const auto lower=std::prev(upper);
        if(upper->first-lower->first>100000000LL)throw std::runtime_error("Reference IMU gap exceeds .1s");
        q=lower->second.slerp(double(t-lower->first)/double(upper->first-lower->first),q);
      }
      sensor_msgs::msg::Imu ref;ref.header=header;
      ref.orientation.w=q.w();ref.orientation.x=q.x();ref.orientation.y=q.y();ref.orientation.z=q.z();
      ref.angular_velocity_covariance[0]=-1;ref.linear_acceleration_covariance[0]=-1;
      pub_->publish(ref);pending_.pop_front();
    }
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  double gravity_;
  std::string frame_;
  std::unique_ptr<cartographer::mapping::ImuTracker> tracker_;
  std::map<int64_t,Eigen::Quaterniond> history_;
  std::deque<std_msgs::msg::Header> pending_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ReferenceAttitude>());rclcpp::shutdown();}
