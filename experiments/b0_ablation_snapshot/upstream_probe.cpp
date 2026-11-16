// Same static-TF scan path as upstream_node.cpp, offline raw scan fixture.
#include <iostream>
#include <cfloat>
#include <fstream>
#include <iomanip>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <hector_slam_lib/slam_main/HectorSlamProcessor.h>
#include "cartographer/sensor/internal/voxel_filter.h"
template<class T>T read(std::ifstream&f){T x;f.read((char*)&x,sizeof(x));return x;}
int main(int argc,char**argv){
 std::ifstream input(argv[1],std::ios::binary);const auto count=read<uint32_t>(input);std::ofstream out(argv[2]);out<<std::setprecision(15);
 const double resolution=std::stod(argv[3]),dist=std::stod(argv[4]),angle=std::stod(argv[5]),min=std::stod(argv[6]);
 const int levels=argc>7?std::stoi(argv[7]):3,size=argc>8?std::stoi(argv[8]):4096;const double voxel=argc>9?std::stod(argv[9]):0,zlo=argc>10?std::stod(argv[10]):-1,zhi=argc>11?std::stod(argv[11]):1;
 hectorslam::HectorSlamProcessor h(resolution,size,size,Eigen::Vector2f(.5,.5),levels);h.setUpdateFactorFree(.4);h.setUpdateFactorOccupied(.9);h.setMapUpdateMinDistDiff(dist);h.setMapUpdateMinAngleDiff(angle);
 laser_geometry::LaserProjection projector;tf2::Transform tf;tf.setIdentity();tf.setOrigin(tf2::Vector3(.1007,0,.0558));
 for(uint32_t j=0;j<count;++j){
  const double t=read<double>(input);const auto n=read<uint32_t>(input);sensor_msgs::msg::LaserScan s;s.angle_min=read<float>(input);s.angle_increment=read<float>(input);s.range_min=read<float>(input);s.range_max=read<float>(input);s.angle_max=s.angle_min+(n-1)*s.angle_increment;s.ranges.resize(n);input.read((char*)s.ranges.data(),n*4);
  sensor_msgs::msg::PointCloud2 cloud;projector.projectLaser(s,cloud,30.);hectorslam::DataContainer data;const float scale=h.getScaleToMap();const auto laserPos=tf.getOrigin();data.setOrigo(Eigen::Vector2f(laserPos.x(),laserPos.y())*scale);
  cartographer::sensor::PointCloud points;
  sensor_msgs::PointCloud2ConstIterator<float>x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
  for(size_t i=0;i<cloud.width*cloud.height;++i,++x,++y,++z){const float d=*x * *x+*y * *y;if(d>min*min&&d<900.){if(*x<0.f&&d<.50f)continue;const auto p=tf*tf2::Vector3(*x,*y,*z);const float dz=p.z()-laserPos.z();if(dz>zlo&&dz<zhi)points.push_back({Eigen::Vector3f(p.x(),p.y(),p.z())});}}
  if(voxel>0)points=cartographer::sensor::VoxelFilter(points,voxel);
  for(const auto&p:points)data.add(p.position.head<2>()*scale);
  h.update(data,h.getLastScanMatchPose());const auto p=h.getLastScanMatchPose();out<<t<<" "<<p.x()<<" "<<p.y()<<" 0 0 0 "<<sin(.5*p.z())<<" "<<cos(.5*p.z())<<"\n";
 }
}
