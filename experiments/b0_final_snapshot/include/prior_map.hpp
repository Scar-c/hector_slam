// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <hector_slam_lib/slam_main/HectorSlamProcessor.h>

namespace hector_mapping_ros2 {
inline std::string pgmToken(std::istream& in) {
  std::string token;
  while (in) {
    int c=in.peek();
    if (c=='#') { in.ignore(std::numeric_limits<std::streamsize>::max(),'\n'); continue; }
    if (c!=EOF && std::isspace(static_cast<unsigned char>(c))) { in.get(); continue; }
    break;
  }
  while(in && in.peek()!=EOF && !std::isspace(static_cast<unsigned char>(in.peek())) && in.peek()!='#') token+=in.get();
  if(token.empty()) throw std::runtime_error("Truncated PGM header/data");
  return token;
}
inline nav_msgs::msg::OccupancyGrid loadPriorMap(const std::string& filename) {
  const auto yaml=YAML::LoadFile(filename);
  const auto mode=yaml["mode"] ? yaml["mode"].as<std::string>() : "trinary";
  if(mode!="trinary") throw std::invalid_argument("Prior map supports trinary mode only");
  const auto origin=yaml["origin"].as<std::vector<double>>();
  const double resolution=yaml["resolution"].as<double>();
  const double occupied=yaml["occupied_thresh"].as<double>(), free=yaml["free_thresh"].as<double>();
  const int negate=yaml["negate"].as<int>();
  if(origin.size()!=3 || !std::all_of(origin.begin(),origin.end(),[](double x){return std::isfinite(x);}) ||
    !std::isfinite(resolution) || resolution<=0 || !std::isfinite(occupied) || !std::isfinite(free) ||
    free<0 || occupied>1 || free>=occupied || (negate!=0 && negate!=1)) throw std::invalid_argument("Invalid prior map YAML");
  auto image=std::filesystem::path(yaml["image"].as<std::string>());
  if(image.is_relative()) image=std::filesystem::path(filename).parent_path()/image;
  std::ifstream in(image,std::ios::binary);
  if(!in) throw std::runtime_error("Cannot open prior PGM: "+image.string());
  const auto magic=pgmToken(in);
  if(magic!="P5" && magic!="P2") throw std::invalid_argument("Prior map image must be P5 or P2 PGM");
  const int width=std::stoi(pgmToken(in)), height=std::stoi(pgmToken(in)), maxval=std::stoi(pgmToken(in));
  if(width<8 || height<8 || width>8192 || height>8192 || maxval<1 || maxval>65535) throw std::invalid_argument("Invalid PGM dimensions/maxval");
  if(magic=="P5") {
    const int separator=in.get();
    if(separator==EOF || !std::isspace(static_cast<unsigned char>(separator))) throw std::runtime_error("Missing PGM data separator");
    if(separator=='\r' && in.peek()=='\n') in.get();
  }
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution=resolution; map.info.width=width; map.info.height=height;
  map.info.origin.position.x=origin[0]; map.info.origin.position.y=origin[1];
  map.info.origin.orientation.z=std::sin(origin[2]/2); map.info.origin.orientation.w=std::cos(origin[2]/2);
  map.data.resize(static_cast<size_t>(width)*height);
  for(int row=0;row<height;++row) for(int x=0;x<width;++x) {
    int value;
    if(magic=="P2") value=std::stoi(pgmToken(in));
    else {
      value=in.get();
      if(value==EOF) throw std::runtime_error("Truncated PGM raster");
      if(maxval>255) {int low=in.get(); if(low==EOF) throw std::runtime_error("Truncated 16-bit PGM raster"); value=value*256+low;}
    }
    if(value<0 || value>maxval) throw std::invalid_argument("Invalid PGM pixel");
    const double probability=negate ? double(value)/maxval : 1.0-double(value)/maxval;
    map.data[(height-row-1)*width+x]=probability>occupied ? 100 : (probability<free ? 0 : -1);
  }
  return map;
}

// Static endpoint-distance field is used only to select initial Hector seeds and
// reject poor matches. All continuous pose optimization uses Hector matchData.
class PriorMatcher {
public:
  explicit PriorMatcher(const nav_msgs::msg::OccupancyGrid& map)
    : width_(map.info.width), height_(map.info.height), resolution_(map.info.resolution), cells_(map.data), distance_(cells_.size(),1e6f) {
    for(size_t i=0;i<cells_.size();++i) if(cells_[i]>50) distance_[i]=0;
    const auto relax=[&](int x,int y,int nx,int ny,float cost) {
      if(nx>=0 && ny>=0 && nx<width_ && ny<height_)
        distance_[y*width_+x]=std::min(distance_[y*width_+x],distance_[ny*width_+nx]+cost*resolution_);
    };
    for(int y=0;y<height_;++y) for(int x=0;x<width_;++x) {
      relax(x,y,x-1,y,1); relax(x,y,x,y-1,1); relax(x,y,x-1,y-1,1.414214f); relax(x,y,x+1,y-1,1.414214f);
    }
    for(int y=height_-1;y>=0;--y) for(int x=width_-1;x>=0;--x) {
      relax(x,y,x+1,y,1); relax(x,y,x,y+1,1); relax(x,y,x+1,y+1,1.414214f); relax(x,y,x-1,y+1,1.414214f);
    }
  }
  float score(const hectorslam::DataContainer& scan, const Eigen::Vector3f& pose, int sample_stride=1) const {
    if(!pose.allFinite() || scan.getSize()==0) return 0;
    const Eigen::Rotation2Df rotation(pose.z());
    float sum=0; int count=0;
    for(int i=0;i<scan.getSize();i+=sample_stride) {
      const Eigen::Vector2f p=pose.head<2>()/resolution_+rotation*scan.getVecEntry(i);
      ++count;
      if(!p.allFinite() || p.x()<0 || p.y()<0 || p.x()>=width_ || p.y()>=height_) continue;
      const int index=int(p.y())*width_+int(p.x());
      if(cells_[index]<0) continue;
      const float d=distance_[index];
      sum+=std::exp(-0.5f*d*d/(0.15f*0.15f));
    }
    return count ? sum/count : 0;
  }
  Eigen::Vector3f relocalize(hectorslam::HectorSlamProcessor& processor,
    const hectorslam::DataContainer& scan, const Eigen::Vector3f& hint,
    float radius, float yaw_radius, bool global=false) const {
    struct Candidate {float score; Eigen::Vector3f pose;};
    std::vector<Candidate> candidates;
    auto add=[&](const Eigen::Vector3f& p) {
      const float s=score(scan,p,4);
      if(candidates.size()<48 || s>candidates.back().score) {
        candidates.push_back({s,p});
        std::sort(candidates.begin(),candidates.end(),[](const Candidate&a,const Candidate&b){return a.score>b.score;});
        if(candidates.size()>48) candidates.pop_back();
      }
    };
    if(global) {
      for(float y=.2f;y<height_*resolution_;y+=.4f) for(float x=.2f;x<width_*resolution_;x+=.4f) {
        if(cells_[int(y/resolution_)*width_+int(x/resolution_)]!=0) continue;
        for(float a=-M_PI;a<M_PI;a+=M_PI/18) add(Eigen::Vector3f(x,y,a));
      }
    } else {
      for(float dy=-radius;dy<=radius+.001f;dy+=.2f) for(float dx=-radius;dx<=radius+.001f;dx+=.2f)
        for(float da=-yaw_radius;da<=yaw_radius+.001f;da+=.0872665f) add(hint+Eigen::Vector3f(dx,dy,da));
    }
    add(hint);
    Eigen::Vector3f best=hint;
    float best_score=score(scan,hint);
    for(const auto& candidate:candidates) {
      const auto p=processor.matchStatic(scan,candidate.pose);
      const float s=score(scan,p);
      if(s>best_score) {best_score=s;best=p;}
    }
    return best;
  }
private:
  int width_,height_; float resolution_;
  std::vector<int8_t> cells_;
  std::vector<float> distance_;
};
} // namespace hector_mapping_ros2
