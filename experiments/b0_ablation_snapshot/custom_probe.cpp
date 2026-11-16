#include <fstream>
#include <iostream>
#include <iomanip>
#include <deque>
#include <vector>
#include <optional>
#include "hector_slam_lib/slam_main/HectorSlamProcessor.h"
#include "imu_orientation_filter.hpp"
#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/sensor/internal/voxel_filter.h"
namespace cc=cartographer::common;
cc::Time ct(double t){return cc::FromUniversal(std::llround(t*1e7)+cc::kUtsEpochOffsetFromUnixEpochInSeconds*10000000LL);}
struct Imu{double t;Eigen::Vector3d g,a;};struct Att{double t;Eigen::Quaterniond q;};
template<class T>T read(std::ifstream&f){T v;f.read((char*)&v,sizeof(v));return v;}
int main(int argc,char**argv){
 std::string dir=argv[1],out=argv[2];bool carto=std::stoi(argv[3]);double zmin=std::stod(argv[4]),zmax=std::stod(argv[5]);bool origin=argc>6&&std::stoi(argv[6]);
 bool xyz=argc<13||std::stoi(argv[12]);
 std::ifstream imuf(dir+"/../b0_hector_deskew/diagnostics/data/imu.txt");std::vector<Imu>imus;Imu row;while(imuf>>row.t>>row.g.x()>>row.g.y()>>row.g.z()>>row.a.x()>>row.a.y()>>row.a.z())imus.push_back(row);if(imus.empty())return 2;
 hector_mapping_ros2::ImuOrientationFilter filter;cartographer::mapping::ImuTracker tracker(10.,ct(imus.front().t));std::vector<Att> history;
 for(const auto&m:imus){if(carto){tracker.Advance(ct(m.t));tracker.AddImuLinearAccelerationObservation(m.a);tracker.AddImuAngularVelocityObservation(m.g);history.push_back({m.t,tracker.orientation()});}else{filter.add(m.t,m.g,m.a,9.80665,.5);}}
 auto attitude=[&](double t)->Eigen::Quaterniond{
  if(!carto)return *filter.at(t,.05,false);
  auto it=std::lower_bound(history.begin(),history.end(),t,[](const Att&a,double t){return a.t<t;});if(it==history.begin())return it->q;auto before=std::prev(it);return before->q.slerp((t-before->t)/(it->t-before->t),it->q);
 };
 // Keep all native-filter states: feed IMU incrementally below (the filter stores only 5s).
 filter.reset();size_t ii=0;std::optional<double>lastyaw;
 hectorslam::HectorSlamProcessor h(.05,4096,4096,Eigen::Vector2f(.5,.5),argc>11?std::stoi(argv[11]):3);h.setUpdateFactorFree(argc>9?std::stod(argv[9]):.4);h.setUpdateFactorOccupied(argc>10?std::stod(argv[10]):.9);h.setMapUpdateMinDistDiff(.2);h.setMapUpdateMinAngleDiff(.08);
 std::ifstream input(dir+"/points.bin",std::ios::binary);std::ifstream scans(dir+"/../b0_hector_deskew/diagnostics/data/scans.bin",std::ios::binary);read<uint32_t>(scans);auto count=read<uint32_t>(input);std::ofstream poses(out+".tum"),stats(out+"_stats.csv");poses<<std::setprecision(15);stats<<std::setprecision(15);
 for(uint32_t j=0;j<count;++j){double t=read<double>(input);auto n=read<uint32_t>(input);std::vector<float>p(n*3);input.read((char*)p.data(),p.size()*4);
  const double scan_t=read<double>(scans);const auto sn=read<uint32_t>(scans);const float amin=read<float>(scans),inc=read<float>(scans),rmin=read<float>(scans),rmax=read<float>(scans);std::vector<float>ranges(sn);scans.read((char*)ranges.data(),sn*4);if(std::abs(scan_t-t)>1e-5)throw std::runtime_error("Input stamps mismatch");
  std::vector<Eigen::Vector3d> source;
  if(xyz){for(size_t k=0;k<n;++k)source.emplace_back(p[k*3],p[k*3+1],p[k*3+2]);}
  else {for(size_t k=0;k<sn;++k){double r=ranges[k];if(!std::isfinite(r)||r<=std::max(.5,double(rmin))||r>=std::min(30.,double(rmax)))continue;const double a=amin+k*double(inc);source.emplace_back(r*cos(a),r*sin(a),0);}}
  n=source.size();
  if(!carto)while(ii<imus.size()&&(ii==0||imus[ii-1].t<t)){const auto&m=imus[ii++];filter.add(m.t,m.g,m.a,9.80665,.5);}
  auto q=attitude(t);double yaw=filter.yaw(q);auto tilt=filter.heading(-yaw)*q;auto hint=h.getLastScanMatchPose();if(lastyaw)hint.z()+=atan2(sin(yaw-*lastyaw),cos(yaw-*lastyaw));
  hectorslam::DataContainer data;Eigen::Vector3d o=origin?tilt*Eigen::Vector3d(.1007,0,.0558):Eigen::Vector3d::Zero();data.setOrigo(o.head<2>().cast<float>()*h.getScaleToMap());
  cartographer::sensor::PointCloud points;
  for(size_t k=0;k<n;++k){Eigen::Vector3d point=tilt*source[k];if(point.z()<zmin||point.z()>zmax)continue;points.push_back({point.cast<float>()});}
  if(argc>7 && std::stod(argv[7])>0)points=cartographer::sensor::VoxelFilter(points,std::stof(argv[7]));
  if(argc>8 && std::stoi(argv[8])){cartographer::sensor::proto::AdaptiveVoxelFilterOptions opt;opt.set_max_length(.5);opt.set_min_num_points(200);opt.set_max_range(50);points=cartographer::sensor::AdaptiveVoxelFilter(points,opt);}
  for(const auto&point:points)data.add(point.position.head<2>()*h.getScaleToMap());
  if(data.getSize()<20)continue;h.update(data,hint,false);lastyaw=yaw;auto v=h.getLastScanMatchPose();auto orientation=filter.heading(v.z())*tilt;
  poses<<t<<" "<<v.x()<<" "<<v.y()<<" 0 "<<orientation.x()<<" "<<orientation.y()<<" "<<orientation.z()<<" "<<orientation.w()<<"\n";stats<<t<<","<<n<<","<<data.getSize()<<"\n";
 }
 if(argc>13 && std::stoi(argv[13])){auto&grid=h.getGridMap();std::ofstream image(out+".pgm",std::ios::binary);image<<"P5\n4096 4096\n255\n";for(int y=4095;y>=0;--y)for(int x=0;x<4096;++x){unsigned char v=grid.isOccupied(x,y)?0:(grid.isFree(x,y)?254:205);image.write((char*)&v,1);}
}
}
