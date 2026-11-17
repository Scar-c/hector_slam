/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.h"

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/internal/voxel_filter.h"

namespace cartographer {
namespace mapping {

proto::LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions2D options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_min_z(parameter_dictionary->GetDouble("min_z"));
  options.set_max_z(parameter_dictionary->GetDouble("max_z"));
  options.set_missing_data_ray_length(
      parameter_dictionary->GetDouble("missing_data_ray_length"));
  options.set_num_accumulated_range_data(
      parameter_dictionary->GetInt("num_accumulated_range_data"));
  options.set_voxel_filter_size(
      parameter_dictionary->GetDouble("voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_loop_closure_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("loop_closure_adaptive_voxel_filter")
              .get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
      mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      mapping::scan_matching::CreateCeresScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() = mapping::CreateMotionFilterOptions(
      parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_extrapolator_options() = CreatePoseExtrapolatorOptions(
      parameter_dictionary->GetDictionary("pose_extrapolator").get());
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  *options.mutable_submaps_options() = CreateSubmapsOptions2D(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  if (parameter_dictionary->HasKey("planar_imu_deskew")) {
    auto dictionary = parameter_dictionary->GetDictionary("planar_imu_deskew");
    auto* deskew = options.mutable_planar_imu_deskew();
    deskew->set_enabled(dictionary->GetBool("enabled"));
    if (dictionary->HasKey("use_legacy_deskew")) {
      deskew->set_use_legacy_deskew(dictionary->GetBool("use_legacy_deskew"));
    }
    deskew->set_gyro_noise(dictionary->GetDouble("gyro_noise"));
    deskew->set_acceleration_noise(dictionary->GetDouble("acceleration_noise"));
    deskew->set_gyro_bias_noise(dictionary->GetDouble("gyro_bias_noise"));
    deskew->set_acceleration_bias_noise(dictionary->GetDouble("acceleration_bias_noise"));
    deskew->set_scan_translation_stddev(dictionary->GetDouble("scan_translation_stddev"));
    deskew->set_scan_rotation_stddev(dictionary->GetDouble("scan_rotation_stddev"));
    deskew->set_use_nhc(dictionary->GetBool("use_nhc"));
    deskew->set_nhc_velocity_stddev(dictionary->GetDouble("nhc_velocity_stddev"));
    deskew->set_max_imu_gap_sec(dictionary->GetDouble("max_imu_gap_sec"));
    deskew->set_max_extrapolation_sec(dictionary->GetDouble("max_extrapolation_sec"));
    if (dictionary->HasKey("iterated_update")) {
      auto iterated_dictionary = dictionary->GetDictionary("iterated_update");
      auto* iterated = deskew->mutable_iterated_update();
      iterated->set_enabled(iterated_dictionary->GetBool("enabled"));
      iterated->set_max_num_iterations(iterated_dictionary->GetInt("max_num_iterations"));
      iterated->set_grid_residual_stddev(iterated_dictionary->GetDouble("grid_residual_stddev"));
      iterated->set_huber_scale(iterated_dictionary->GetDouble("huber_scale"));
      iterated->set_max_translation_update(iterated_dictionary->GetDouble("max_translation_update"));
      iterated->set_max_yaw_update(iterated_dictionary->GetDouble("max_yaw_update"));
      CHECK(!iterated->enabled() || deskew->enabled())
          << "Iterated lidar update requires planar_imu_deskew.enabled=true.";
    }
    CHECK(!deskew->enabled() || options.use_imu_data())
        << "Planar IMU deskew requires use_imu_data=true.";
  }
  return options;
}

}  // namespace mapping
}  // namespace cartographer
