// Search a complete fixed prior with a scan only: no initial-pose argument exists.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include "prior_map.hpp"
#include "map2map_matcher/core.hpp"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
using Clock=std::chrono::steady_clock;
double seconds(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double>(b-a).count();}
namespace cm=cartographer::mapping;
namespace cs=cartographer::sensor;
namespace ct=cartographer::transform;
int main(int argc,char**argv){
 if(argc!=5){std::cerr<<"Usage: global_initialize map.yaml scan.xy output_directory fcsm|fft|hector_search\n";return 2;}
 const auto begin=Clock::now();const std::string method=argv[4];const std::filesystem::path output=argv[3];std::filesystem::create_directories(output);
 auto prior=hector_mapping_ros2::loadPriorMap(argv[1]);const double res=prior.info.resolution;const int w=prior.info.width,h=prior.info.height;
 map2map_matcher::PointCloud source;cs::PointCloud cloud;hectorslam::DataContainer data;data.setOrigo(Eigen::Vector2f::Zero());
 std::ifstream in(argv[2]);double x,y;while(in>>x>>y){source.emplace_back(x,y);cloud.push_back({Eigen::Vector3f(x,y,0)});data.add(Eigen::Vector2f(x/res,y/res));}if(source.empty())throw std::runtime_error("No scan points");
 hectorslam::HectorSlamProcessor processor(res,(w+3)/4*4,(h+3)/4*4,Eigen::Vector2f::Zero(),3);processor.loadStaticMap(prior.data,w,h);hector_mapping_ros2::PriorMatcher matcher(prior);
 const auto common_ready=Clock::now();YAML::Node report;report["method"]=method;report["map"]=argv[1];report["scan"]=argv[2];report["source_points"]=source.size();report["scope"]="complete prior XY and 360 degree yaw; no pose input";
 report["map_import_occupied_threshold"]=YAML::LoadFile(argv[1])["occupied_thresh"].as<double>();report["common_preprocessing_s"]=seconds(begin,common_ready);
 Eigen::Vector3f candidate=Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());bool returned=false;auto search_begin=Clock::now();
 if(method=="fcsm"){
  cm::ValueConversionTables tables;cm::ProbabilityGrid grid(cm::MapLimits(res,Eigen::Vector2d(w*res,h*res),cm::CellLimits(h,w)),&tables);
  for(int row=0;row<h;++row)for(int col=0;col<w;++col){const auto value=prior.data[row*w+col];if(value<0)continue;grid.SetProbability(grid.limits().GetCellIndex(Eigen::Vector2f((col+.5)*res,(row+.5)*res)),value>50?.9f:.1f);}
  cm::scan_matching::proto::FastCorrelativeScanMatcherOptions2D options;options.set_branch_and_bound_depth(8);options.set_linear_search_window(1.);options.set_angular_search_window(M_PI);
  cm::scan_matching::FastCorrelativeScanMatcher2D search(grid,options);search_begin=Clock::now();report["algorithm_preprocessing_s"]=seconds(common_ready,search_begin);report["branch_and_bound_depth"]=8;report["resolution_m"]=res;report["fcsm_min_score"]=.45;
  float score=0;ct::Rigid2d pose=ct::Rigid2d::Identity();returned=search.MatchFullSubmap(cloud,.45f,&score,&pose);report["native_score"]=score;
  if(returned)candidate=Eigen::Vector3f(pose.translation().x(),pose.translation().y(),pose.rotation().angle());
 }else if(method=="fft"){
  map2map_matcher::PointCloud target;for(int row=0;row<h;++row)for(int col=0;col<w;++col)if(prior.data[row*w+col]>50)target.emplace_back((col+.5)*res,(row+.5)*res);
  const auto bounds=map2map_matcher::calculate_bounds(target);map2map_matcher::FftMatcherConfig config;config.search_range_x=std::max(bounds.width*1.2,w*res);config.search_range_y=std::max(bounds.height*1.2,h*res);config.maximum_workers=10; // Original business 0.3deg, 0.9->0.3m, 3-stage settings.
  search_begin=Clock::now();report["algorithm_preprocessing_s"]=seconds(common_ready,search_begin);const auto result=map2map_matcher::AdaptiveFftMatcher(config).match(source,target);candidate=Eigen::Vector3f(result.translation.x(),result.translation.y(),map2map_matcher::normalize_angle(result.angle_radians));returned=true;report["native_score"]=result.correlation;report["rotation_step_degrees"]=config.rotation_step_degrees;report["workers"]=config.maximum_workers;report["business_default_minimum_points"]=1000;report["business_service_point_count_gate_passes"]=source.size()>=1000;report["fft_scope"]="Unmodified algorithm core; business service quality gate reported separately";
  for(const auto&stage:result.stages){YAML::Node v;v["stage"]=stage.stage;v["elapsed_s"]=stage.elapsed_seconds;v["resolution"]=stage.resolution;v["candidates"]=stage.input_candidates;v["grid_width"]=stage.grid_width;v["grid_height"]=stage.grid_height;report["stages"].push_back(v);}
 }else if(method=="hector_search"){
  struct Candidate{float score;Eigen::Vector3f pose;};std::vector<Candidate> pool;size_t evaluated=0;
  // Same existing self-written grid search, but no hint candidate or hinted incumbent.
  search_begin=Clock::now();report["algorithm_preprocessing_s"]=seconds(common_ready,search_begin);
  for(float py=.2f;py<h*res;py+=.4f)for(float px=.2f;px<w*res;px+=.4f){if(prior.data[int(py/res)*w+int(px/res)]!=0)continue;for(float a=-M_PI;a<M_PI;a+=M_PI/18){Eigen::Vector3f pose(px,py,a);float score=matcher.score(data,pose,4);++evaluated;if(pool.size()<48||score>pool.back().score){pool.push_back({score,pose});std::sort(pool.begin(),pool.end(),[](const auto&a,const auto&b){return a.score>b.score;});if(pool.size()>48)pool.pop_back();}}}
  float best=-1;for(const auto&c:pool){const auto p=processor.matchStatic(data,c.pose);const float score=matcher.score(data,p);if(score>best){best=score;candidate=p;returned=true;}}
  report["native_score"]=best;report["evaluated_grid_candidates"]=evaluated;report["xy_step_m"]=.4;report["yaw_step_degrees"]=10;report["retained_candidates"]=pool.size();
 }else throw std::runtime_error("Unknown method");
 const auto search_end=Clock::now();report["search_s"]=seconds(search_begin,search_end);report["returned_candidate"]=returned;
 if(returned&&candidate.allFinite()){
  for(int i=0;i<3;++i)report["coarse_prior_pose"].push_back(candidate[i]);report["common_score_before_refine"]=matcher.score(data,candidate);
  const auto refined=processor.matchStatic(data,candidate);if(matcher.score(data,refined)>matcher.score(data,candidate))candidate=refined;
  const auto& q=prior.info.origin.orientation;const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));const Eigen::Vector2f world=Eigen::Vector2f(prior.info.origin.position.x,prior.info.origin.position.y)+Eigen::Rotation2Df(yaw)*candidate.head<2>();
  const double heading=map2map_matcher::normalize_angle(candidate.z()+yaw);report["pose"].push_back(world.x());report["pose"].push_back(world.y());report["pose"].push_back(heading);const float quality=matcher.score(data,candidate);report["common_match_score"]=quality;report["passes_match_threshold"]=quality>=.45;
  if(quality>=.45){std::ofstream seed(output/(method+"_seed.json"));seed<<std::setprecision(17)<<"{\"method\":\""<<method<<"\",\"pose\":["<<world.x()<<","<<world.y()<<","<<heading<<"],\"global_search\":true,\"manual_seed\":false}\n";}
 }else report["passes_match_threshold"]=false;
 const auto end=Clock::now();report["refine_and_verify_s"]=seconds(search_end,end);report["total_s"]=seconds(begin,end);std::ofstream file(output/(method+".yaml"));file<<report<<"\n";std::cout<<report<<std::endl;
}
