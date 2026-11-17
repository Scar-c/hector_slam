// Exercise the actual fixed-prior call path that previously allocated a map-sized
// covariance lookup cache on every accepted scan.
#include <cstdlib>
#include <new>
#include <gtest/gtest.h>
#include <hector_slam_lib/slam_main/HectorSlamProcessor.h>
namespace {bool track_arrays=false;size_t large_arrays=0;}
void* operator new[](std::size_t n){if(track_arrays&&n>=128*1024)++large_arrays;if(auto p=std::malloc(n))return p;throw std::bad_alloc();}
void operator delete[](void*p) noexcept{std::free(p);}
void operator delete[](void*p,std::size_t) noexcept{std::free(p);}
TEST(StaticCommit, RepeatedScansDoNotAllocateMapSizedCaches){
 hectorslam::HectorSlamProcessor p(.05,256,256,Eigen::Vector2f::Zero(),3);
 std::vector<int8_t> cells(256*256,0);for(int y=0;y<256;++y)cells[y*256+150]=100;p.loadStaticMap(cells,256,256);
 hectorslam::DataContainer scan;for(int i=0;i<80;++i)scan.add(Eigen::Vector2f(50, i-40));
 p.commitStaticPose(Eigen::Vector3f(5,5,0),scan);
 large_arrays=0;track_arrays=true;for(int i=0;i<10;++i)p.commitStaticPose(Eigen::Vector3f(5,5,.001*i),scan);track_arrays=false;
 EXPECT_EQ(large_arrays,0u);
}
TEST(StaticCommit, CachedHessianMatchesFreshEvaluationAcrossReload){
 hectorslam::HectorSlamProcessor p(.05,64,64,Eigen::Vector2f::Zero(),3);
 hectorslam::DataContainer scan;for(int i=0;i<60;++i)scan.add(Eigen::Vector2f(10,(i-30)*.3));
 for(int wall:{30,34}){
  std::vector<int8_t> cells(64*64,0);for(int y=0;y<64;++y)cells[y*64+wall]=100;p.loadStaticMap(cells,64,64);
  for(float angle:{0.f,.02f,.1f}){
   const Eigen::Vector3f pose(1.01,1.5,angle);p.commitStaticPose(pose,scan);
   hectorslam::OccGridMapUtilConfig<hectorslam::GridMap> fresh(const_cast<hectorslam::GridMap*>(&p.getGridMap(0)));
   Eigen::Matrix3f expected;Eigen::Vector3f derivatives;fresh.getCompleteHessianDerivs(p.getGridMap(0).getMapCoordsPose(pose),scan,expected,derivatives);
   EXPECT_EQ((p.getLastScanMatchCovariance()-expected).squaredNorm(),0.f);
   EXPECT_EQ((p.getLastScanMatchPose()-pose).squaredNorm(),0.f);
  }
 }
}
