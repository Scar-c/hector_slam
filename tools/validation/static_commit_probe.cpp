#include <cstdlib>
#include <new>
#include <chrono>
#include <fstream>
#include <iostream>
#include "prior_map.hpp"
static bool track=false;static size_t bytes=0,count=0;
void* operator new[](std::size_t n){if(track){bytes+=n;++count;}if(auto p=std::malloc(n))return p;throw std::bad_alloc();}
void operator delete[](void*p) noexcept {std::free(p);}void operator delete[](void*p,std::size_t) noexcept {std::free(p);}
int main(int argc,char**argv){auto map=hector_mapping_ros2::loadPriorMap(argv[1]);hectorslam::HectorSlamProcessor processor(map.info.resolution,(map.info.width+3)/4*4,(map.info.height+3)/4*4,Eigen::Vector2f::Zero(),3);processor.loadStaticMap(map.data,map.info.width,map.info.height);hectorslam::DataContainer cloud;std::ifstream f(argv[2]);float x,y;while(f>>x>>y)cloud.add(Eigen::Vector2f(x,y)/map.info.resolution);Eigen::Vector3f pose(90,60,0);processor.commitStaticPose(pose,cloud);auto t=std::chrono::steady_clock::now();track=true;for(int i=0;i<10;++i)processor.commitStaticPose(pose,cloud);track=false;std::cout<<"{\"repeated_commits\":10,\"new_array_allocations\":"<<count<<",\"allocated_bytes\":"<<bytes<<",\"elapsed_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count()<<"}"<<std::endl;return count==0?0:1;}
