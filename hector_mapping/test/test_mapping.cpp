#include <iostream>
#include <gtest/gtest.h>
#include <hector_slam_lib/slam_main/HectorSlamProcessor.h>
#include "hector_mapping_support.hpp"

TEST(ScanConversion, RejectsInvalidRangesAndTransformsOrigin) {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0; scan.angle_increment = 0.1; scan.range_min = 0.1; scan.range_max = 10;
  scan.ranges = {1, NAN, INFINITY, -1, 0.05f, 11};
  tf2::Quaternion q; q.setRPY(0, 0, M_PI / 2);
  tf2::Transform t(q, tf2::Vector3(0.5, 0.2, 0.3));
  hectorslam::DataContainer data;
  ASSERT_TRUE(hector_mapping_ros2::convertScan(scan, t, 20, 0.1, 10, -1, 1, data));
  ASSERT_EQ(data.getSize(), 1);
  EXPECT_NEAR(data.getOrigo().x(), 10, 1e-5);
  EXPECT_NEAR(data.getOrigo().y(), 4, 1e-5);
  EXPECT_NEAR(data.getVecEntry(0).x(), 10, 1e-5);
  EXPECT_NEAR(data.getVecEntry(0).y(), 24, 1e-5);
  scan.angle_increment = NAN;
  EXPECT_FALSE(hector_mapping_ros2::convertScan(scan, t, 20, 0.1, 10, -1, 1, data));
}

TEST(Covariance, InvertsHessianAndScalesTranslation) {
  Eigen::Matrix3f h = Eigen::Vector3f(4, 4, 2).asDiagonal();
  const auto covariance = hector_mapping_ros2::poseCovariance(h, 0.05);
  EXPECT_NEAR(covariance(0, 0), 0.000625, 1e-9);
  EXPECT_NEAR(covariance(2, 2), 0.5, 1e-9);
  EXPECT_TRUE(hector_mapping_ros2::poseCovariance(Eigen::Matrix3f::Zero(), 0.05).allFinite());
}

// Ray cast an asymmetric room: x in [-4,6], y in [-3,4].
hectorslam::DataContainer room(float x, float y, float yaw) {
  hectorslam::DataContainer data;
  data.setOrigo(Eigen::Vector2f::Zero());
  for (int i = 0; i < 720; ++i) {
    const float a = -M_PI + i * 2 * M_PI / 720;
    const float dx = std::cos(a + yaw), dy = std::sin(a + yaw);
    const float tx = std::abs(dx) < 1e-7 ? INFINITY : (dx > 0 ? (6-x)/dx : (-4-x)/dx);
    const float ty = std::abs(dy) < 1e-7 ? INFINITY : (dy > 0 ? (4-y)/dy : (-3-y)/dy);
    const float range = std::min(tx, ty);
    data.add(Eigen::Vector2f(std::cos(a)*range, std::sin(a)*range) * 20);
  }
  return data;
}
TEST(HectorCore, TracksMotionWithoutOdometryAndResets) {
  hectorslam::HectorSlamProcessor processor(0.05, 512, 512, Eigen::Vector2f(0.5, 0.5), 3);
  processor.setUpdateFactorOccupied(0.9);
  processor.setUpdateFactorFree(0.4);
  processor.setMapUpdateMinDistDiff(0.05);
  processor.setMapUpdateMinAngleDiff(0.02);
  for (int i = 0; i < 25; ++i) {
    const float x = i * 0.02, y = i * 0.01, yaw = i * 0.004;
    const auto data = room(x, y, yaw);
    processor.update(data, processor.getLastScanMatchPose());
  }
  const auto pose = processor.getLastScanMatchPose();
  ASSERT_TRUE(pose.allFinite());
  EXPECT_NEAR(pose.x(), 0.48, 0.08);
  EXPECT_NEAR(pose.y(), 0.24, 0.08);
  EXPECT_NEAR(pose.z(), 0.096, 0.03);
  const auto &grid = processor.getGridMap();
  int occupied = 0, free = 0;
  for (int i = 0; i < grid.getSizeX()*grid.getSizeY(); ++i) {
    occupied += grid.isOccupied(i); free += grid.isFree(i);
  }
  EXPECT_GT(occupied, 100); EXPECT_GT(free, 1000);
  processor.reset();
  EXPECT_TRUE(processor.getLastScanMatchPose().isZero());
  EXPECT_TRUE(processor.getLastScanMatchCovariance().isZero());
  for (int i = 0; i < grid.getSizeX()*grid.getSizeY(); ++i) {
    EXPECT_FALSE(grid.isOccupied(i)); EXPECT_FALSE(grid.isFree(i));
  }
}
TEST(HectorCore, DegenerateScanDoesNotProduceNaN) {
  hectorslam::HectorSlamProcessor processor(0.05, 128, 128, Eigen::Vector2f(0.5, 0.5), 2);
  hectorslam::DataContainer data; data.setOrigo(Eigen::Vector2f::Zero());
  for (int i = 0; i < 30; ++i) data.add(Eigen::Vector2f(20, 0));
  for (int i = 0; i < 10; ++i) processor.update(data, processor.getLastScanMatchPose());
  EXPECT_TRUE(processor.getLastScanMatchPose().allFinite());
}

TEST(HectorCore, KnownPoseMappingUpdatesEveryResolution) {
  hectorslam::HectorSlamProcessor processor(0.05, 512, 512, Eigen::Vector2f(0.5, 0.5), 3);
  processor.update(room(0, 0, 0), Eigen::Vector3f::Zero(), true);
  for (int level = 0; level < 3; ++level) {
    const auto &grid = processor.getGridMap(level);
    int occupied = 0;
    for (int i = 0; i < grid.getSizeX()*grid.getSizeY(); ++i) occupied += grid.isOccupied(i);
    EXPECT_GT(occupied, 50) << "level " << level;
  }
}

#include "imu_orientation_filter.hpp"
TEST(ImuFusion, IntegratesGyroYawAndRejectsStaleData) {
  hector_mapping_ros2::ImuOrientationFilter filter;
  const Eigen::Vector3d gyro(0,0,M_PI/2), gravity(0,0,9.80665);
  for (int i=0;i<=200;++i) ASSERT_TRUE(filter.add(i*0.005,gyro,gravity,9.80665,0.5));
  const auto q=filter.at(1.0,0.05); ASSERT_TRUE(q);
  EXPECT_NEAR(hector_mapping_ros2::ImuOrientationFilter::yaw(*q),M_PI/2,1e-6);
  EXPECT_FALSE(filter.at(1.2,0.05));
  EXPECT_FALSE(filter.add(0.5,gyro,gravity,9.80665,0.5));
  filter.reset(); EXPECT_FALSE(filter.at(1.0,0.05));
}
TEST(ImuFusion, AlignsGravityAndPreservesYaw) {
  hector_mapping_ros2::ImuOrientationFilter filter;
  const Eigen::Quaterniond tilt(Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.3,Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d gravity=tilt.inverse()*Eigen::Vector3d(0,0,9.80665);
  ASSERT_TRUE(filter.add(0,Eigen::Vector3d::Zero(),gravity,9.80665,0.5));
  const auto q=filter.at(0,0.05); ASSERT_TRUE(q);
  EXPECT_LT(((*q)*gravity-Eigen::Vector3d(0,0,9.80665)).norm(),1e-6);
  const auto midpoint=filter.at(0.01,0.05); ASSERT_TRUE(midpoint);
  EXPECT_LT(q->angularDistance(*midpoint),1e-6);
}

TEST(ImuFusion, StrictSynchronizationRequiresFollowingSample) {
  hector_mapping_ros2::ImuOrientationFilter filter;
  const Eigen::Vector3d gyro(0,0,1),gravity(0,0,9.80665);
  ASSERT_TRUE(filter.add(0,gyro,gravity,9.80665,0.5));
  EXPECT_FALSE(filter.at(0.005,0.05,false));
  ASSERT_TRUE(filter.add(0.01,gyro,gravity,9.80665,0.5));
  const auto q=filter.at(0.005,0.05,false);ASSERT_TRUE(q);
  EXPECT_NEAR(hector_mapping_ros2::ImuOrientationFilter::yaw(*q),0.005,1e-10);
  EXPECT_FALSE(filter.at(0.02,0.05,false));
}
