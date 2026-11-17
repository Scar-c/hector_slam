// Hector online frontend / workspace-local Cartographer-derived backend adapter.
// The frontend local pose never receives optimized pose feedback.
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <hector_backend_msgs/msg/submap_list.hpp>
#include <hector_backend_msgs/srv/submap_query.hpp>
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/io/submap_painter.h"
namespace cc=cartographer::common;namespace cm=cartographer::mapping;namespace cs=cartographer::sensor;namespace ct=cartographer::transform;
static int64_t ns(const std_msgs::msg::Header&h){return int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;}
static geometry_msgs::msg::Pose pose_msg(const ct::Rigid3d&p){geometry_msgs::msg::Pose m;m.position.x=p.translation().x();m.position.y=p.translation().y();m.position.z=p.translation().z();m.orientation.x=p.rotation().x();m.orientation.y=p.rotation().y();m.orientation.z=p.rotation().z();m.orientation.w=p.rotation().w();return m;}
class Backend:public rclcpp::Node{
public:
 Backend():Node("hector_online_backend"),pool_(4),tf_(*this){
  global_=declare_parameter<std::string>("map_frame","map");local_=declare_parameter<std::string>("local_frame","hector_local_map");output_=declare_parameter<std::string>("output_directory","");if(global_==local_)throw std::invalid_argument("Backend global/local frames must differ");
  const auto config=declare_parameter<std::string>("configuration_directory",ament_index_cpp::get_package_share_directory("hector_backend")+"/config");
  auto resolver=std::make_unique<cc::ConfigurationFileResolver>(std::vector<std::string>{config,ament_index_cpp::get_package_share_directory("hector_backend_core")+"/configuration_files"});const auto code=resolver->GetFileContentOrDie("backend.lua");cc::LuaParameterDictionary dict(code,std::move(resolver));auto pd=dict.GetDictionary("pose_graph");auto options=cm::CreatePoseGraphOptions(pd.get());auto sd=dict.GetDictionary("submaps");auto smopts=cm::CreateSubmapsOptions2D(sd.get());
  graph_=std::make_unique<cm::PoseGraph2D>(options,std::make_unique<cm::optimization::OptimizationProblem2D>(options.optimization_problem_options()),&pool_);submaps_=std::make_unique<cm::ActiveSubmaps2D>(smopts);
  const auto qos=rclcpp::QoS(2000).reliable();
  pose_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/slam_out_pose",qos,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){if(m->header.frame_id!=local_)throw std::runtime_error("Hector pose frame does not match local_frame");poses_[ns(m->header)]=m;process();});
  ref_sub_=create_subscription<sensor_msgs::msg::Imu>("/deskew_reference_attitude",qos,[this](sensor_msgs::msg::Imu::ConstSharedPtr m){references_[ns(m->header)]=m;process();});
  cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/hector_matching_points",rclcpp::SensorDataQoS().keep_last(1000),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){clouds_[ns(m->header)]=m;if(clouds_.size()>1000)throw std::runtime_error("Backend input pairing overflow");process();});
  list_pub_=create_publisher<hector_backend_msgs::msg::SubmapList>("/submap_list",rclcpp::QoS(1).reliable().transient_local());path_pub_=create_publisher<nav_msgs::msg::Path>("/backend_path",rclcpp::QoS(1).transient_local());map_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local());status_pub_=create_publisher<std_msgs::msg::String>("/backend_status",10);
  query_=create_service<hector_backend_msgs::srv::SubmapQuery>("/submap_query",[this](std::shared_ptr<hector_backend_msgs::srv::SubmapQuery::Request> req,std::shared_ptr<hector_backend_msgs::srv::SubmapQuery::Response> res){const auto d=graph_->GetSubmapData({req->trajectory_id,req->submap_index});if(!d.submap){res->status.code=5;res->status.message="Unknown submap";return;}cm::proto::SubmapQuery::Response p;d.submap->ToResponseProto(d.pose,&p);res->submap_version=p.submap_version();for(const auto&t:p.textures()){hector_backend_msgs::msg::SubmapTexture tex;tex.cells.assign(t.cells().begin(),t.cells().end());tex.width=t.width();tex.height=t.height();tex.resolution=t.resolution();tex.slice_pose=pose_msg(ct::ToRigid3(t.slice_pose()));res->textures.push_back(std::move(tex));}res->status.code=0;res->status.message="OK";});
  finish_=create_service<std_srvs::srv::Trigger>("/finish_hector_backend",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){if(!nodes_){res->success=false;res->message="No graph nodes yet";return;}if(!finished_){process();graph_->FinishTrajectory(0);graph_->RunFinalOptimization();finished_=true;publish();save();}res->success=true;res->message="Final optimization and save complete";});
  timer_=create_wall_timer(std::chrono::milliseconds(500),[this]{publish();});
  RCLCPP_INFO(get_logger(),"Online Hector backend ready: %s -> %s; workspace-local core and submap API",global_.c_str(),local_.c_str());
 }
private:
 void process(){
  if(finished_)return;
  while(!clouds_.empty()){
   const int64_t t=clouds_.begin()->first;auto pit=poses_.find(t);auto rit=references_.find(t);if(pit==poses_.end()||rit==references_.end())break;
   auto cloud=clouds_.begin()->second;const auto pm=pit->second->pose;const auto qr=rit->second->orientation;Eigen::Quaterniond ref(qr.w,qr.x,qr.y,qr.z);const auto tilt=Eigen::Quaterniond(Eigen::AngleAxisd(-ct::GetYaw(ref),Eigen::Vector3d::UnitZ()))*ref;const auto qh=pm.orientation;const Eigen::Quaterniond full(qh.w,qh.x,qh.y,qh.z);const double heading=ct::GetYaw(full*tilt.conjugate());const ct::Rigid3d local_pose(Eigen::Vector3d(pm.position.x,pm.position.y,0),Eigen::Quaterniond(Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ())));
   clouds_.erase(clouds_.begin());poses_.erase(poses_.begin(),std::next(pit));references_.erase(references_.begin(),std::next(rit));++frames_;
   const auto delta=last_pose_.inverse()*local_pose;if(nodes_&&(t-last_stamp_)*1e-9<=5&&delta.translation().norm()<=.2&&std::abs(Eigen::AngleAxisd(delta.rotation()).angle())<=M_PI/180.)continue;
   cs::PointCloud points;sensor_msgs::PointCloud2ConstIterator<float>x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z");for(size_t i=0;i<cloud->width*cloud->height;++i,++x,++y,++z){auto p=tilt*Eigen::Vector3d(*x,*y,*z);if(p.allFinite())points.push_back({Eigen::Vector3f(p.x(),p.y(),0)});}
   const Eigen::Vector3d origin=tilt*Eigen::Vector3d(.1007,0,.0558);const cs::RangeData range{Eigen::Vector3f(origin.x(),origin.y(),0),points,{}};auto active=submaps_->InsertRangeData(cs::TransformRangeData(range,local_pose.cast<float>()));auto data=std::make_shared<cm::TrajectoryNode::Data>();data->time=cc::FromUniversal(t/100+cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL);data->gravity_alignment=Eigen::Quaterniond::Identity();data->local_pose=local_pose;data->filtered_gravity_aligned_point_cloud=std::move(points);graph_->AddNode(data,0,active);if(!nodes_){geometry_msgs::msg::TransformStamped initial_tf;initial_tf.header.frame_id=global_;initial_tf.child_frame_id=local_;initial_tf.header.stamp=cloud->header.stamp;initial_tf.transform.rotation.w=1.;tf_.sendTransform(initial_tf);}last_pose_=local_pose;last_stamp_=t;++nodes_;
  }
 }
 void publish(){
  if(!nodes_)return;
  hector_backend_msgs::msg::SubmapList list;list.header.frame_id=global_;list.header.stamp=now();for(const auto&e:graph_->GetAllSubmapPoses()){hector_backend_msgs::msg::SubmapEntry m;m.trajectory_id=e.id.trajectory_id;m.submap_index=e.id.submap_index;m.submap_version=e.data.version;m.pose=pose_msg(e.data.pose);m.is_frozen=false;list.submap.push_back(m);}list_pub_->publish(list);
  nav_msgs::msg::Path path;path.header=list.header;ct::Rigid3d correction=ct::Rigid3d::Identity();for(const auto&e:graph_->GetTrajectoryNodes()){geometry_msgs::msg::PoseStamped p;p.header=list.header;const auto ticks=cc::ToUniversal(e.data.time())-cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL;p.header.stamp=rclcpp::Time(ticks*100);p.pose=pose_msg(e.data.global_pose);path.poses.push_back(p);correction=e.data.global_pose*e.data.constant_data->local_pose.inverse();}path_pub_->publish(path);
  geometry_msgs::msg::TransformStamped tr;tr.header=list.header;tr.header.stamp=(now()+rclcpp::Duration::from_seconds(.6));tr.child_frame_id=local_;auto p=pose_msg(correction);tr.transform.translation.x=p.position.x;tr.transform.translation.y=p.position.y;tr.transform.translation.z=p.position.z;tr.transform.rotation=p.orientation;tf_.sendTransform(tr);
  std_msgs::msg::String s;s.data="{\"frames\":"+std::to_string(frames_)+",\"nodes\":"+std::to_string(nodes_)+",\"submaps\":"+std::to_string(list.submap.size())+",\"finished\":"+(finished_?"true":"false")+"}";status_pub_->publish(s);
  if(map_pub_->get_subscription_count() && ++publish_count_%4==0)publish_map();
 }
 void publish_map(){
  std::map<cm::SubmapId,cartographer::io::SubmapSlice>slices;cm::ValueConversionTables tables;for(const auto&e:graph_->GetAllSubmapData())cartographer::io::FillSubmapSlice(e.data.pose,e.data.submap->ToProto(true),&slices[e.id],&tables);if(slices.empty())return;auto image=cartographer::io::PaintSubmapSlices(slices,.05);cairo_surface_flush(image.surface.get());const int w=cairo_image_surface_get_width(image.surface.get()),h=cairo_image_surface_get_height(image.surface.get()),stride=cairo_image_surface_get_stride(image.surface.get());const auto*bytes=cairo_image_surface_get_data(image.surface.get());nav_msgs::msg::OccupancyGrid map;map.header.frame_id=global_;map.header.stamp=now();map.info.resolution=.05;map.info.width=w;map.info.height=h;map.info.origin.position.x=-image.origin.x()*.05;map.info.origin.position.y=(image.origin.y()-h)*.05;map.info.origin.orientation.w=1;map.data.resize(w*h);for(int y=0;y<h;++y)for(int x=0;x<w;++x){uint32_t pixel;std::memcpy(&pixel,bytes+y*stride+x*4,4);const int green=(pixel>>8)&255,red=(pixel>>16)&255;map.data[(h-y-1)*w+x]=green?std::clamp(int(std::lround((255-red)*100./255.)),0,100):-1;}map_pub_->publish(map);
 }
 void save(){
  if(output_.empty())return;std::filesystem::create_directories(output_);std::ofstream file(output_+"/trajectory_optimized.tum");file<<std::setprecision(17);for(const auto&e:graph_->GetTrajectoryNodes()){const auto&p=e.data.global_pose;const auto&q=p.rotation();file<<(cc::ToUniversal(e.data.time())-cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL)*1e-7<<" "<<p.translation().x()<<" "<<p.translation().y()<<" "<<p.translation().z()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";}
  std::map<cm::SubmapId,cartographer::io::SubmapSlice>slices;cm::ValueConversionTables tables;for(const auto&e:graph_->GetAllSubmapData())cartographer::io::FillSubmapSlice(e.data.pose,e.data.submap->ToProto(true),&slices[e.id],&tables);if(!slices.empty()){auto image=cartographer::io::PaintSubmapSlices(slices,.05);cairo_surface_write_to_png(image.surface.get(),(output_+"/map.png").c_str());cairo_surface_flush(image.surface.get());const int w=cairo_image_surface_get_width(image.surface.get()),h=cairo_image_surface_get_height(image.surface.get()),stride=cairo_image_surface_get_stride(image.surface.get());const auto*bytes=cairo_image_surface_get_data(image.surface.get());std::ofstream pgm(output_+"/map.pgm",std::ios::binary);pgm<<"P5\n"<<w<<" "<<h<<"\n255\n";for(int y=0;y<h;++y)for(int x=0;x<w;++x){uint32_t pixel;std::memcpy(&pixel,bytes+y*stride+x*4,4);const char gray=(pixel>>16)&255;pgm.write(&gray,1);}std::ofstream yaml(output_+"/map.yaml");yaml<<std::setprecision(17)<<"image: map.pgm\nresolution: 0.05\norigin: ["<<-image.origin.x()*.05<<", "<<(image.origin.y()-h)*.05<<", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";}
  size_t loops=0;for(const auto&c:graph_->constraints())if(c.tag==cm::PoseGraphInterface::Constraint::INTER_SUBMAP)++loops;std::ofstream summary(output_+"/summary.json");summary<<"{\"frames\":"<<frames_<<",\"nodes\":"<<nodes_<<",\"loop_constraints\":"<<loops<<",\"finished\":true}\n";
 }
 cc::ThreadPool pool_;std::unique_ptr<cm::PoseGraph2D>graph_;std::unique_ptr<cm::ActiveSubmaps2D>submaps_;tf2_ros::TransformBroadcaster tf_;std::string global_,local_,output_;bool finished_=false;size_t frames_=0,nodes_=0,publish_count_=0;int64_t last_stamp_=0;ct::Rigid3d last_pose_=ct::Rigid3d::Identity();
 std::map<int64_t,geometry_msgs::msg::PoseStamped::ConstSharedPtr>poses_;std::map<int64_t,sensor_msgs::msg::Imu::ConstSharedPtr>references_;std::map<int64_t,sensor_msgs::msg::PointCloud2::ConstSharedPtr>clouds_;
 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ref_sub_;rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;rclcpp::Publisher<hector_backend_msgs::msg::SubmapList>::SharedPtr list_pub_;rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;rclcpp::Service<hector_backend_msgs::srv::SubmapQuery>::SharedPtr query_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){google::InitGoogleLogging(argv[0]);rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Backend>());rclcpp::shutdown();}
