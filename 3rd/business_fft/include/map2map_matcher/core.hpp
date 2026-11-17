#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace map2map_matcher
{

using Point2d = Eigen::Vector2d;
using PointCloud = std::vector<Point2d>;

struct MapMetadata
{
  std::filesystem::path yaml_path;
  std::filesystem::path image_path;
  double resolution{0.05};
  Point2d origin{Point2d::Zero()};
  double origin_yaw{0.0};
  double occupied_threshold{0.65};
  double free_threshold{0.196};
  std::filesystem::file_time_type yaml_mtime{};
  std::filesystem::file_time_type image_mtime{};
};

struct LoadedMap
{
  MapMetadata metadata;
  PointCloud points;
  int image_width{0};
  int image_height{0};
};

struct Bounds2d
{
  Point2d minimum{Point2d::Zero()};
  Point2d maximum{Point2d::Zero()};
  Point2d center{Point2d::Zero()};
  double width{0.0};
  double height{0.0};
  double area{0.0};
};

struct QualityConfig
{
  int minimum_points{1000};
  int minimum_fine_cells{25};
  int minimum_coarse_cells{5};
  double fine_resolution{0.3};
  double coarse_resolution{0.9};
  double minimum_long_side{1.5};
  double minimum_area{0.8};
};

struct QualityStats
{
  std::size_t point_count{0};
  std::size_t fine_cells{0};
  std::size_t coarse_cells{0};
  Bounds2d bounds;
};

struct FftMatcherConfig
{
  double rotation_step_degrees{0.3};
  double coarse_resolution{0.9};
  double fine_resolution{0.3};
  int resolution_steps{3};
  double search_range_x{10.0};
  double search_range_y{10.0};
  double candidate_retention_ratio{0.33};
  int minimum_candidate_count{5};
  int maximum_workers{10};
};

struct MatchProbabilityConfig
{
  double sigma{0.2};
  double cell_size{0.2};
  double search_radius{0.4};
  int source_downsample{2};
};

struct CandidateResult
{
  int angle_index{0};
  double angle_degrees{0.0};
  double angle_radians{0.0};
  Point2d translation_delta{Point2d::Zero()};
  double correlation{0.0};
  double resolution{0.0};
  int stage{0};
};

struct StageDiagnostics
{
  int stage{0};
  double resolution{0.0};
  int grid_width{0};
  int grid_height{0};
  std::size_t input_candidates{0};
  std::size_t retained_candidates{0};
  double elapsed_seconds{0.0};
  std::vector<CandidateResult> top_candidates;
};

struct MatchResult
{
  double angle_degrees{0.0};
  double angle_radians{0.0};
  Point2d translation{Point2d::Zero()};
  Point2d translation_delta{Point2d::Zero()};
  Point2d fft_translation_offset{Point2d::Zero()};
  double correlation{0.0};
  double elapsed_seconds{0.0};
  std::vector<StageDiagnostics> stages;
};

class MapLoader
{
public:
  static MapMetadata read_metadata(const std::filesystem::path & yaml_path);
  static LoadedMap load(const std::filesystem::path & yaml_path);
};

PointCloud occupancy_grid_to_points(
  const std::vector<std::int8_t> & data,
  std::uint32_t width,
  std::uint32_t height,
  double resolution,
  const Point2d & origin,
  int occupied_threshold = 50);

PointCloud downsample_points(const PointCloud & points, int factor);
Bounds2d calculate_bounds(const PointCloud & points);
Point2d calculate_center(const PointCloud & points);
QualityStats calculate_quality(const PointCloud & points, const QualityConfig & config);
void validate_quality(const QualityStats & stats, const QualityConfig & config);

class AdaptiveFftMatcher
{
public:
  explicit AdaptiveFftMatcher(FftMatcherConfig config);
  MatchResult match(const PointCloud & source_points, const PointCloud & target_points) const;

private:
  FftMatcherConfig config_;
};

Eigen::Isometry2d make_rigid_transform(double angle_radians, const Point2d & translation);

double gaussian_match_probability(
  const PointCloud & source_points,
  const PointCloud & target_points,
  const Eigen::Isometry2d & source_to_target,
  const MatchProbabilityConfig & config);

double normalize_angle(double angle_radians);

}  // namespace map2map_matcher
