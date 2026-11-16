#include <gtest/gtest.h>
#include <fstream>
#include "prior_map.hpp"

TEST(PriorMap, ParsesPixelsThresholdsFlipOriginAndNegate) {
  const auto dir=std::filesystem::temp_directory_path()/"hector_prior_parser_test";
  std::filesystem::create_directories(dir);
  std::ofstream(dir/"map.pgm",std::ios::binary)<<"P5\n# raster test\n8 8\n255\n"<<std::string(32,'\0')<<std::string(32,char(255));
  std::ofstream(dir/"map.yaml")<<"image: map.pgm\nresolution: 0.1\norigin: [2, -3, 1.57]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  auto map=hector_mapping_ros2::loadPriorMap((dir/"map.yaml").string());
  EXPECT_EQ(map.data.front(),0); EXPECT_EQ(map.data.back(),100);
  EXPECT_DOUBLE_EQ(map.info.origin.position.x,2);
  EXPECT_NEAR(2*std::atan2(map.info.origin.orientation.z,map.info.origin.orientation.w),1.57,1e-10);
  std::ofstream(dir/"map.pgm",std::ios::binary)<<"P5\n8 8\n255\n"<<std::string(64,char(205));
  map=hector_mapping_ros2::loadPriorMap((dir/"map.yaml").string());
  EXPECT_EQ(map.data.front(),-1);
  std::ofstream pgm(dir/"map.pgm"); pgm<<"P2\n8 8\n255\n";
  for(int i=0;i<8;++i) pgm<<"0 255 205 0 255 205 0 255\n"; pgm.close();
  std::ofstream(dir/"map.yaml")<<"image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 1\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  map=hector_mapping_ros2::loadPriorMap((dir/"map.yaml").string());
  EXPECT_EQ(map.data[0],0); EXPECT_EQ(map.data[1],100);
  std::ofstream(dir/"map.pgm")<<"P5\n8 8\n255\nshort";
  EXPECT_THROW(hector_mapping_ros2::loadPriorMap((dir/"map.yaml").string()),std::runtime_error);
  std::filesystem::remove_all(dir);
}
TEST(PriorMap, RectangularPyramidPreservesUnknownAndCellCenters) {
  hectorslam::HectorSlamProcessor p(.1,40,48,Eigen::Vector2f::Zero(),3);
  std::vector<int8_t> map(37*45,0); map[0]=100; map[1]=-1; map[2]=-1;
  p.loadStaticMap(map,37,45);
  for(int l=0;l<3;++l) {
    const auto& g=p.getGridMap(l);
    EXPECT_NEAR(g.getWorldCoords(Eigen::Vector2f::Zero()).x(),.05*(1<<l),1e-6);
    EXPECT_TRUE(g.isOccupied(0,0));
    EXPECT_FLOAT_EQ(g.getCell(g.getSizeX()-1,g.getSizeY()-1).getValue(),0);
  }
  EXPECT_FLOAT_EQ(p.getGridMap(1).getCell(1,0).getValue(),0);
}
TEST(PriorMap, MatchingAndResetNeverModifyAnyPriorLevel) {
  hectorslam::HectorSlamProcessor p(.1,64,64,Eigen::Vector2f::Zero(),3);
  std::vector<int8_t> map(64*64,0);
  for(int y=0;y<64;++y) for(int x=0;x<64;++x) if(x==10 || x==50 || y==8 || y==55) map[y*64+x]=100;
  p.loadStaticMap(map,64,64);
  std::vector<std::vector<float>> before(3);
  for(int l=0;l<3;++l) {const auto& g=p.getGridMap(l);for(int i=0;i<g.getSizeX()*g.getSizeY();++i) before[l].push_back(g.getCell(i).getValue());}
  hectorslam::DataContainer scan; scan.setOrigo(Eigen::Vector2f::Zero());
  for(int i=0;i<100;++i) scan.add(Eigen::Vector2f(20,std::sin(i*.05)*20));
  for(int i=0;i<20;++i) {p.update(scan,Eigen::Vector3f(3,3,.01*i));p.update(scan,Eigen::Vector3f(3,3,.01*i),true);}
  p.reset();
  for(int l=0;l<3;++l) {const auto& g=p.getGridMap(l);for(size_t i=0;i<before[l].size();++i) ASSERT_FLOAT_EQ(g.getCell(i).getValue(),before[l][i]);}
}

TEST(PriorMap, RecoversPerturbedPoseUsingOnlyFrozenRasterAndScan) {
  const int size=128;
  nav_msgs::msg::OccupancyGrid map;map.info.width=size;map.info.height=size;map.info.resolution=.1;
  map.data.assign(size*size,0);
  for(int y=0;y<size;++y) for(int x=0;x<size;++x)
    if(x==10 || x==110 || y==20 || y==90) map.data[y*size+x]=100;
  hectorslam::HectorSlamProcessor p(.1,size,size,Eigen::Vector2f::Zero(),3);
  p.loadStaticMap(map.data,size,size);
  hectorslam::DataContainer scan;scan.setOrigo(Eigen::Vector2f::Zero());
  const float px=5.325f,py=5.625f,yaw=.12f;
  for(int i=0;i<720;++i) {
    const float a=-M_PI+i*2*M_PI/720,dx=std::cos(a+yaw),dy=std::sin(a+yaw);
    const float tx=std::abs(dx)<1e-7?INFINITY:(dx>0?(11.05f-px)/dx:(1.05f-px)/dx);
    const float ty=std::abs(dy)<1e-7?INFINITY:(dy>0?(9.05f-py)/dy:(2.05f-py)/dy);
    scan.add(Eigen::Vector2f(std::cos(a),std::sin(a))*std::min(tx,ty)*10);
  }
  hector_mapping_ros2::PriorMatcher matcher(map);
  const auto result=matcher.relocalize(p,scan,Eigen::Vector3f(px+.6f,py-.4f,yaw+.2f),1,.35);
  EXPECT_NEAR(result.x(),px,.12);EXPECT_NEAR(result.y(),py,.12);EXPECT_NEAR(result.z(),yaw,.03);
  EXPECT_GT(matcher.score(scan,result),.85);
}

#include "odometry_pose_prior.hpp"
TEST(OdometryPrior, AppliesMotionAndDirectCorrectionWithoutSmoothing) {
  hector_mapping_ros2::OdometryPosePrior prior;
  tf2::Quaternion map_q;map_q.setRPY(0,0,.6);
  tf2::Quaternion odom_q;odom_q.setRPY(0,0,.2);
  const tf2::Transform map_base(map_q,tf2::Vector3(10,-3,0));
  const tf2::Transform odom_base(odom_q,tf2::Vector3(1,2,0));
  ASSERT_FALSE(prior.predict(odom_base));
  const auto correction=prior.correct(map_base,odom_base);
  EXPECT_LT(((correction*odom_base).getOrigin()-map_base.getOrigin()).length(),1e-10);
  tf2::Quaternion delta_q;delta_q.setRPY(0,0,.1);
  const tf2::Transform delta(delta_q,tf2::Vector3(.2,-.1,0));
  const auto prediction=prior.predict(odom_base*delta);ASSERT_TRUE(prediction);
  const auto expected=map_base*delta;
  EXPECT_NEAR(prediction->x(),expected.getOrigin().x(),1e-6);
  EXPECT_NEAR(prediction->y(),expected.getOrigin().y(),1e-6);
  EXPECT_NEAR(prediction->z(),.7,1e-6);
  const tf2::Transform new_map_base(map_q,tf2::Vector3(13,-1,0));
  const auto new_correction=prior.correct(new_map_base,odom_base);
  // A 3m map correction is applied in full immediately, never low-pass blended.
  EXPECT_LT(((new_correction*odom_base).getOrigin()-new_map_base.getOrigin()).length(),1e-10);
  prior.reset();EXPECT_FALSE(prior.predict(odom_base));
}
