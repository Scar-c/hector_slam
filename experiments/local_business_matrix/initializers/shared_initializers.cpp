#include <iostream>
#include <fstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include "map2map_matcher/core.hpp"
#include "prior_map.hpp"
int main(int argc,char**argv) {
 if(argc!=4){std::cerr<<"Usage: shared_initializers map.yaml input_cloud.xy output_dir\n";return 2;}
 const auto output=std::filesystem::path(argv[3]);std::filesystem::create_directories(output);
 map2map_matcher::PointCloud cloud;std::ifstream input(argv[2]);double x,y;
 while(input>>x>>y)cloud.emplace_back(x,y);
 if(cloud.empty())throw std::runtime_error("Empty initial snapshot");
 const auto main_map=map2map_matcher::MapLoader::load(argv[1]);
 const auto yaml=YAML::LoadFile("/home/lc/ZHUXING_V2WS-master/src/algorithm/map2map_matcher/map2map_matcher/config/config.yaml");
 const auto adaptive=yaml["adaptive_fft"],qcfg=adaptive["input_quality"],res=adaptive["resolution"];
 auto qconfig=map2map_matcher::QualityConfig{};
 qconfig.minimum_points=qcfg["min_small_points"].as<int>();qconfig.minimum_fine_cells=qcfg["min_fine_occupied_cells"].as<int>();qconfig.minimum_coarse_cells=qcfg["min_coarse_occupied_cells"].as<int>();
 qconfig.minimum_long_side=qcfg["min_bbox_long_side_m"].as<double>();qconfig.minimum_area=qcfg["min_bbox_area_m2"].as<double>();
 qconfig.fine_resolution=res["fine"].as<double>();qconfig.coarse_resolution=res["coarse"].as<double>();const auto quality=map2map_matcher::calculate_quality(cloud,qconfig);map2map_matcher::validate_quality(quality,qconfig);
 auto config=map2map_matcher::FftMatcherConfig{};
 config.rotation_step_degrees=yaml["rotation_model"]["rotation_step"].as<double>();
 config.coarse_resolution=res["coarse"].as<double>();config.fine_resolution=res["fine"].as<double>();config.resolution_steps=res["steps"].as<int>();
 config.maximum_workers=adaptive["maximum_workers"].as<int>();config.candidate_retention_ratio=adaptive["candidate_retention"]["ratio"].as<double>();config.minimum_candidate_count=adaptive["candidate_retention"]["min_count"].as<int>();
 const auto bounds=map2map_matcher::calculate_bounds(main_map.points);config.search_range_x=bounds.width*adaptive["search_range"]["margin_factor"].as<double>();config.search_range_y=bounds.height*adaptive["search_range"]["margin_factor"].as<double>();
 const auto fft=map2map_matcher::AdaptiveFftMatcher(config).match(cloud,main_map.points);
 const auto probability=map2map_matcher::gaussian_match_probability(cloud,main_map.points,map2map_matcher::make_rigid_transform(fft.angle_radians,fft.translation),map2map_matcher::MatchProbabilityConfig{});
 if(probability<40)throw std::runtime_error("Business FFT initial match below 40 percent Gaussian support");
 {
  std::ofstream out(output/"business_fft.json");out<<std::setprecision(15)
   <<"{\"method\":\"business_fft\",\"initial_pose_x\":"<<fft.translation.x()<<",\"initial_pose_y\":"<<fft.translation.y()
   <<",\"initial_pose_yaw\":"<<map2map_matcher::normalize_angle(fft.angle_radians)<<",\"gaussian_match_probability\":"<<probability
   <<",\"correlation\":"<<fft.correlation<<",\"elapsed_seconds\":"<<fft.elapsed_seconds<<",\"source_points\":"<<cloud.size()<<",\"coarse_resolution\":"<<config.coarse_resolution
   <<",\"fine_resolution\":"<<config.fine_resolution<<",\"rotation_step_degrees\":"<<config.rotation_step_degrees<<"}\n";
 }
 const auto prior=hector_mapping_ros2::loadPriorMap(argv[1]);int width=(prior.info.width+3)/4*4,height=(prior.info.height+3)/4*4;
 hectorslam::HectorSlamProcessor processor(prior.info.resolution,width,height,Eigen::Vector2f::Zero(),3);processor.loadStaticMap(prior.data,prior.info.width,prior.info.height);
 hectorslam::DataContainer data;data.setOrigo(Eigen::Vector2f::Zero());
 for(const auto&p:cloud)data.add(p.cast<float>()/prior.info.resolution);
 hector_mapping_ros2::PriorMatcher matcher(prior);const auto pose=matcher.relocalize(processor,data,Eigen::Vector3f::Zero(),1,.35,true);
 const double score=matcher.score(data,pose);if(score<.45)throw std::runtime_error("Hector search initial match below threshold");
 // This map's origin yaw is zero; keep general map metadata transformation.
 const Eigen::Vector2d world=main_map.metadata.origin+Eigen::Rotation2Dd(main_map.metadata.origin_yaw)*pose.head<2>().cast<double>();
 std::ofstream out(output/"hector_search.json");out<<std::setprecision(15)<<"{\"method\":\"hector_search\",\"initial_pose_x\":"<<world.x()<<",\"initial_pose_y\":"<<world.y()
  <<",\"initial_pose_yaw\":"<<map2map_matcher::normalize_angle(pose.z()+main_map.metadata.origin_yaw)<<",\"match_score\":"<<score<<",\"source_points\":"<<cloud.size()<<"}\n";
 std::cout<<"Shared initializers complete: business FFT probability="<<probability<<", Hector score="<<score<<"\n";
}
