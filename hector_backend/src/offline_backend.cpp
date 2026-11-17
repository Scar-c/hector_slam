#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <glog/logging.h>
#include <cairo/cairo.h>
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/io/submap_painter.h"
namespace cc=cartographer::common;namespace cm=cartographer::mapping;namespace cs=cartographer::sensor;namespace ct=cartographer::transform;
int main(int argc,char**argv){
 google::InitGoogleLogging(argv[0]);FLAGS_logtostderr=true;
 if(argc<4){std::cerr<<"backend inputs.bin config_dir output_dir [max_seconds]\n";return 2;}
 std::filesystem::create_directories(argv[3]);const std::string out=argv[3];
 auto resolver=std::make_unique<cc::ConfigurationFileResolver>(std::vector<std::string>{argv[2],ament_index_cpp::get_package_share_directory("hector_backend_core")+"/configuration_files"});
 const auto code=resolver->GetFileContentOrDie("backend.lua");cc::LuaParameterDictionary dict(code,std::move(resolver));auto pgdict=dict.GetDictionary("pose_graph");auto options=cm::CreatePoseGraphOptions(pgdict.get());auto smdict=dict.GetDictionary("submaps");auto smopts=cm::CreateSubmapsOptions2D(smdict.get());
 cc::ThreadPool pool(4);cm::PoseGraph2D graph(options,std::make_unique<cm::optimization::OptimizationProblem2D>(options.optimization_problem_options()),&pool);cm::ActiveSubmaps2D submaps(smopts);
 std::ifstream input(argv[1],std::ios::binary);int64_t stamp=0,first=0,last_stamp=0;uint32_t n;double v[5];size_t frames=0,inserted=0;ct::Rigid3d last=ct::Rigid3d::Identity();std::ofstream raw(out+"/trajectory_local_nodes.tum");raw<<std::setprecision(17);
 while(input.read(reinterpret_cast<char*>(&stamp),8)){
  input.read(reinterpret_cast<char*>(v),sizeof(v));input.read(reinterpret_cast<char*>(&n),4);std::vector<float> values(n*3);input.read(reinterpret_cast<char*>(values.data()),values.size()*4);CHECK(input.good());
  if(!first)first=stamp;if(argc>4 && (stamp-first)*1e-9>std::stod(argv[4]))break;++frames;
  const ct::Rigid3d pose(Eigen::Vector3d(v[0],v[1],0),Eigen::Quaterniond(Eigen::AngleAxisd(v[2],Eigen::Vector3d::UnitZ())));
  const auto delta=last.inverse()*pose;if(inserted && (stamp-last_stamp)*1e-9<=5 && delta.translation().norm()<=.2 && std::abs(Eigen::AngleAxisd(delta.rotation()).angle())<=M_PI/180.)continue;
  cs::PointCloud points;for(size_t i=0;i<n;++i)points.push_back({Eigen::Vector3f(values[3*i],values[3*i+1],0)});
  const cs::RangeData range{Eigen::Vector3f(v[3],v[4],0),points,{}};auto active=submaps.InsertRangeData(cs::TransformRangeData(range,pose.cast<float>()));
  auto data=std::make_shared<cm::TrajectoryNode::Data>();data->time=cc::FromUniversal(stamp/100+cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL);data->gravity_alignment=Eigen::Quaterniond::Identity();data->local_pose=pose;data->filtered_gravity_aligned_point_cloud=std::move(points);graph.AddNode(data,0,active);
  raw<<stamp*1e-9<<" "<<v[0]<<" "<<v[1]<<" 0 0 0 "<<pose.rotation().z()<<" "<<pose.rotation().w()<<"\n";
  last=pose;last_stamp=stamp;++inserted;if(inserted%500==0)std::cout<<"Inserted "<<inserted<<" nodes, bag "<<(stamp-first)*1e-9<<"s"<<std::endl;
 }
 graph.FinishTrajectory(0);graph.RunFinalOptimization();auto nodes=graph.GetTrajectoryNodes();std::ofstream traj(out+"/trajectory_optimized.tum");traj<<std::setprecision(17);
 for(const auto&entry:nodes){const auto&node=entry.data;const auto&p=node.global_pose;const auto&q=p.rotation();traj<<(cc::ToUniversal(node.time())-cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL)*1e-7<<" "<<p.translation().x()<<" "<<p.translation().y()<<" "<<p.translation().z()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";}
 size_t loops=0,intra=0;std::ofstream constraints(out+"/constraints.csv");constraints<<"submap,node,type,x,y,yaw\n";
 for(const auto&c:graph.constraints()){if(c.tag==cm::PoseGraphInterface::Constraint::INTER_SUBMAP)++loops;else++intra;constraints<<c.submap_id.submap_index<<","<<c.node_id.node_index<<","<<(c.tag==cm::PoseGraphInterface::Constraint::INTER_SUBMAP?"loop":"intra")<<","<<c.pose.zbar_ij.translation().x()<<","<<c.pose.zbar_ij.translation().y()<<","<<ct::GetYaw(c.pose.zbar_ij.rotation())<<"\n";}
 std::map<cm::SubmapId,cartographer::io::SubmapSlice> slices;cm::ValueConversionTables tables;
 for(const auto&e:graph.GetAllSubmapData())cartographer::io::FillSubmapSlice(e.data.pose,e.data.submap->ToProto(true),&slices[e.id],&tables);
 auto painted=cartographer::io::PaintSubmapSlices(slices,.05);cairo_surface_write_to_png(painted.surface.get(),(out+"/map_rgba.png").c_str());
 std::ofstream summary(out+"/summary.json");summary<<"{\"input_frames\":"<<frames<<",\"nodes\":"<<inserted<<",\"submaps\":"<<slices.size()<<",\"loop_constraints\":"<<loops<<",\"intra_constraints\":"<<intra<<",\"map_resolution\":0.05,\"map_origin_pixel\":["<<painted.origin.x()<<","<<painted.origin.y()<<"]}\n";
 std::cout<<"Finished nodes="<<inserted<<" loops="<<loops<<std::endl;
}
