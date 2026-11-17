#include "map2map_matcher/core.hpp"

#include <fftw3.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace map2map_matcher
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

struct CellIndex
{
  std::int64_t x{0};
  std::int64_t y{0};

  bool operator==(const CellIndex & other) const noexcept
  {
    return x == other.x && y == other.y;
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex & cell) const noexcept
  {
    const auto hx = std::hash<std::int64_t>{}(cell.x);
    const auto hy = std::hash<std::int64_t>{}(cell.y);
    return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6U) + (hx >> 2U));
  }
};

CellIndex point_cell(const Point2d & point, const double resolution)
{
  return CellIndex{
    static_cast<std::int64_t>(std::floor(point.x() / resolution)),
    static_cast<std::int64_t>(std::floor(point.y() / resolution))};
}

std::filesystem::path resolve_image_path(
  const std::filesystem::path & yaml_path,
  const std::string & image_value)
{
  if (image_value.empty()) {
    throw std::runtime_error("地图YAML缺少image字段");
  }

  auto image_path = std::filesystem::path(image_value);
  if (image_path.is_relative()) {
    image_path = yaml_path.parent_path() / image_path;
  }
  image_path = std::filesystem::absolute(image_path).lexically_normal();
  if (std::filesystem::exists(image_path)) {
    return image_path;
  }

  static constexpr const char * kExtensions[] = {
    ".pgm", ".png", ".jpg", ".jpeg", ".bmp", ".tiff"};
  const auto base = image_path.parent_path() / image_path.stem();
  for (const auto * extension : kExtensions) {
    const auto candidate = std::filesystem::path(base.string() + extension);
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error("地图图片不存在: " + image_path.string());
}

int next_power_of_two(const int value)
{
  if (value <= 1) {
    return 1;
  }
  int result = 1;
  while (result < value) {
    if (result > std::numeric_limits<int>::max() / 2) {
      throw std::overflow_error("FFT栅格尺寸过大");
    }
    result *= 2;
  }
  return result;
}

bool is_fft_friendly(int value)
{
  int exponent_two = 0;
  int exponent_three = 0;
  int exponent_five = 0;
  while (value > 1 && value % 2 == 0) {
    value /= 2;
    ++exponent_two;
  }
  while (value > 1 && value % 3 == 0) {
    value /= 3;
    ++exponent_three;
  }
  while (value > 1 && value % 5 == 0) {
    value /= 5;
    ++exponent_five;
  }
  return value == 1 && exponent_two <= 13 && exponent_three <= 7 && exponent_five <= 4;
}

int fft_friendly_size(const int value)
{
  if (value <= 1) {
    return 1;
  }
  for (int candidate = value; candidate <= 10000; ++candidate) {
    if (is_fft_friendly(candidate)) {
      return candidate;
    }
  }
  return next_power_of_two(value);
}

struct GridGeometry
{
  int base_width{0};
  int base_height{0};
  int fft_width{0};
  int fft_height{0};
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
  double resolution{0.0};
};

GridGeometry make_grid_geometry(
  const Point2d & center,
  const double range_x,
  const double range_y,
  const double resolution)
{
  if (!(range_x > 0.0) || !(range_y > 0.0) || !(resolution > 0.0)) {
    throw std::invalid_argument("FFT搜索范围和分辨率必须为正数");
  }
  GridGeometry geometry;
  geometry.base_width = static_cast<int>(2.0 * range_x / resolution);
  geometry.base_height = static_cast<int>(2.0 * range_y / resolution);
  geometry.base_width = std::max(2, geometry.base_width + geometry.base_width % 2);
  geometry.base_height = std::max(2, geometry.base_height + geometry.base_height % 2);
  geometry.fft_width = fft_friendly_size(geometry.base_width);
  geometry.fft_height = fft_friendly_size(geometry.base_height);
  geometry.x_min = center.x() - range_x;
  geometry.x_max = center.x() + range_x;
  geometry.y_min = center.y() - range_y;
  geometry.y_max = center.y() + range_y;
  geometry.resolution = resolution;
  return geometry;
}

void rasterize_point(
  const Point2d & point,
  const GridGeometry & geometry,
  double * const grid)
{
  if (point.x() < geometry.x_min || point.x() > geometry.x_max ||
    point.y() < geometry.y_min || point.y() > geometry.y_max)
  {
    return;
  }

  const double x_scaled =
    (point.x() - geometry.x_min) / (geometry.x_max - geometry.x_min) * geometry.base_width;
  const double y_scaled =
    (point.y() - geometry.y_min) / (geometry.y_max - geometry.y_min) * geometry.base_height;
  const int x = x_scaled >= geometry.base_width ?
    geometry.base_width - 1 : static_cast<int>(std::floor(x_scaled));
  const int y = y_scaled >= geometry.base_height ?
    geometry.base_height - 1 : static_cast<int>(std::floor(y_scaled));
  if (x >= 0 && x < geometry.base_width && y >= 0 && y < geometry.base_height) {
    grid[static_cast<std::size_t>(y) * geometry.fft_width + x] = 1.0;
  }
}

void rasterize_target(
  const PointCloud & points,
  const GridGeometry & geometry,
  double * const grid)
{
  const auto size = static_cast<std::size_t>(geometry.fft_width) * geometry.fft_height;
  std::fill(grid, grid + size, 0.0);
  for (const auto & point : points) {
    rasterize_point(point, geometry, grid);
  }
}

void rasterize_rotated_source(
  const PointCloud & points,
  const Eigen::Rotation2Dd & rotation,
  const Point2d & fft_offset,
  const GridGeometry & geometry,
  double * const grid)
{
  const auto size = static_cast<std::size_t>(geometry.fft_width) * geometry.fft_height;
  std::fill(grid, grid + size, 0.0);
  for (const auto & point : points) {
    rasterize_point(rotation * point + fft_offset, geometry, grid);
  }
}

struct FftwDeleter
{
  void operator()(void * pointer) const noexcept
  {
    fftw_free(pointer);
  }
};

using RealBuffer = std::unique_ptr<double, FftwDeleter>;
using ComplexBuffer = std::unique_ptr<fftw_complex, FftwDeleter>;

struct ComplexValue
{
  double real{0.0};
  double imaginary{0.0};
};

class FftWorkspace
{
public:
  FftWorkspace(const int height, const int width)
  : height_(height), width_(width),
    real_size_(static_cast<std::size_t>(height) * width),
    complex_size_(static_cast<std::size_t>(height) * (width / 2 + 1)),
    input_(static_cast<double *>(fftw_alloc_real(real_size_))),
    forward_output_(static_cast<fftw_complex *>(fftw_alloc_complex(complex_size_))),
    inverse_input_(static_cast<fftw_complex *>(fftw_alloc_complex(complex_size_))),
    inverse_output_(static_cast<double *>(fftw_alloc_real(real_size_)))
  {
    if (!input_ || !forward_output_ || !inverse_input_ || !inverse_output_) {
      throw std::bad_alloc();
    }
    forward_plan_ = fftw_plan_dft_r2c_2d(
      height_, width_, input_.get(), forward_output_.get(), FFTW_ESTIMATE);
    inverse_plan_ = fftw_plan_dft_c2r_2d(
      height_, width_, inverse_input_.get(), inverse_output_.get(), FFTW_ESTIMATE);
    if (forward_plan_ == nullptr || inverse_plan_ == nullptr) {
      throw std::runtime_error("FFTW计划创建失败");
    }
  }

  ~FftWorkspace()
  {
    if (forward_plan_ != nullptr) {
      fftw_destroy_plan(forward_plan_);
    }
    if (inverse_plan_ != nullptr) {
      fftw_destroy_plan(inverse_plan_);
    }
  }

  FftWorkspace(const FftWorkspace &) = delete;
  FftWorkspace & operator=(const FftWorkspace &) = delete;

  double * input() noexcept {return input_.get();}

  std::vector<ComplexValue> forward_copy()
  {
    fftw_execute(forward_plan_);
    std::vector<ComplexValue> result(complex_size_);
    for (std::size_t index = 0; index < complex_size_; ++index) {
      result[index] = ComplexValue{forward_output_.get()[index][0], forward_output_.get()[index][1]};
    }
    return result;
  }

  std::pair<Point2d, double> correlate(
    const std::vector<ComplexValue> & target_spectrum,
    const double resolution)
  {
    if (target_spectrum.size() != complex_size_) {
      throw std::invalid_argument("FFT目标频谱尺寸不匹配");
    }
    fftw_execute(forward_plan_);
    for (std::size_t index = 0; index < complex_size_; ++index) {
      const double source_real = forward_output_.get()[index][0];
      const double source_imaginary = forward_output_.get()[index][1];
      const auto & target = target_spectrum[index];
      inverse_input_.get()[index][0] =
        source_real * target.real + source_imaginary * target.imaginary;
      inverse_input_.get()[index][1] =
        source_imaginary * target.real - source_real * target.imaginary;
    }
    fftw_execute(inverse_plan_);

    const double normalization = 1.0 / static_cast<double>(real_size_);
    double best_value = -std::numeric_limits<double>::infinity();
    int best_shifted_x = 0;
    int best_shifted_y = 0;
    const int shift_x = width_ / 2;
    const int shift_y = height_ / 2;
    for (int shifted_y = 0; shifted_y < height_; ++shifted_y) {
      const int raw_y = (shifted_y - shift_y + height_) % height_;
      for (int shifted_x = 0; shifted_x < width_; ++shifted_x) {
        const int raw_x = (shifted_x - shift_x + width_) % width_;
        const double value = inverse_output_.get()[
          static_cast<std::size_t>(raw_y) * width_ + raw_x] * normalization;
        if (value > best_value) {
          best_value = value;
          best_shifted_x = shifted_x;
          best_shifted_y = shifted_y;
        }
      }
    }

    const double dx = -(best_shifted_x - width_ / 2) * resolution;
    const double dy = -(best_shifted_y - height_ / 2) * resolution;
    return {Point2d(dx, dy), best_value};
  }

private:
  int height_{0};
  int width_{0};
  std::size_t real_size_{0};
  std::size_t complex_size_{0};
  RealBuffer input_;
  ComplexBuffer forward_output_;
  ComplexBuffer inverse_input_;
  RealBuffer inverse_output_;
  fftw_plan forward_plan_{nullptr};
  fftw_plan inverse_plan_{nullptr};
};

std::vector<double> resolution_pyramid(const FftMatcherConfig & config)
{
  if (config.resolution_steps <= 0) {
    throw std::invalid_argument("FFT分辨率层数必须为正数");
  }
  if (!(config.coarse_resolution > 0.0) || !(config.fine_resolution > 0.0)) {
    throw std::invalid_argument("FFT分辨率必须为正数");
  }
  if (config.resolution_steps == 1) {
    return {config.coarse_resolution};
  }

  std::vector<double> resolutions;
  resolutions.reserve(static_cast<std::size_t>(config.resolution_steps));
  const double coarse_log = std::log(config.coarse_resolution);
  const double fine_log = std::log(config.fine_resolution);
  for (int index = 0; index < config.resolution_steps; ++index) {
    const double ratio = static_cast<double>(index) / (config.resolution_steps - 1);
    resolutions.push_back(std::exp(coarse_log + ratio * (fine_log - coarse_log)));
  }
  return resolutions;
}

bool candidate_better(const CandidateResult & lhs, const CandidateResult & rhs)
{
  if (lhs.correlation != rhs.correlation) {
    return lhs.correlation > rhs.correlation;
  }
  return lhs.angle_index < rhs.angle_index;
}

}  // namespace

MapMetadata MapLoader::read_metadata(const std::filesystem::path & yaml_path_value)
{
  if (yaml_path_value.empty()) {
    throw std::invalid_argument("主地图YAML路径为空");
  }
  const auto yaml_path = std::filesystem::absolute(yaml_path_value).lexically_normal();
  if (!std::filesystem::exists(yaml_path)) {
    throw std::runtime_error("主地图YAML不存在: " + yaml_path.string());
  }

  const YAML::Node yaml = YAML::LoadFile(yaml_path.string());
  MapMetadata metadata;
  metadata.yaml_path = yaml_path;
  metadata.image_path = resolve_image_path(yaml_path, yaml["image"].as<std::string>(""));
  metadata.resolution = yaml["resolution"].as<double>(0.05);
  if (!(metadata.resolution > 0.0)) {
    throw std::runtime_error("地图resolution必须为正数");
  }
  const auto origin = yaml["origin"];
  if (origin && origin.IsSequence() && origin.size() >= 2) {
    metadata.origin.x() = origin[0].as<double>(0.0);
    metadata.origin.y() = origin[1].as<double>(0.0);
    if (origin.size() >= 3) {
      metadata.origin_yaw = origin[2].as<double>(0.0);
    }
  }
  metadata.occupied_threshold = yaml["occupied_thresh"].as<double>(0.65);
  metadata.free_threshold = yaml["free_thresh"].as<double>(0.196);
  metadata.yaml_mtime = std::filesystem::last_write_time(metadata.yaml_path);
  metadata.image_mtime = std::filesystem::last_write_time(metadata.image_path);
  return metadata;
}

LoadedMap MapLoader::load(const std::filesystem::path & yaml_path)
{
  LoadedMap loaded;
  loaded.metadata = read_metadata(yaml_path);
  const cv::Mat image = cv::imread(loaded.metadata.image_path.string(), cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    throw std::runtime_error("地图图片读取失败: " + loaded.metadata.image_path.string());
  }
  if (image.type() != CV_8UC1) {
    throw std::runtime_error("地图图片不是8位灰度图");
  }

  loaded.image_width = image.cols;
  loaded.image_height = image.rows;
  loaded.points.reserve(static_cast<std::size_t>(image.rows) * image.cols / 8U);
  for (int y = 0; y < image.rows; ++y) {
    const auto * row = image.ptr<std::uint8_t>(y);
    for (int x = 0; x < image.cols; ++x) {
      if (row[x] < 128) {
        loaded.points.emplace_back(
          loaded.metadata.origin.x() + x * loaded.metadata.resolution,
          loaded.metadata.origin.y() + (image.rows - 1 - y) * loaded.metadata.resolution);
      }
    }
  }
  if (loaded.points.empty()) {
    throw std::runtime_error("主地图转换后没有占用点");
  }
  return loaded;
}

PointCloud occupancy_grid_to_points(
  const std::vector<std::int8_t> & data,
  const std::uint32_t width,
  const std::uint32_t height,
  const double resolution,
  const Point2d & origin,
  const int occupied_threshold)
{
  const auto expected_size = static_cast<std::size_t>(width) * height;
  if (width == 0 || height == 0 || data.size() != expected_size) {
    throw std::invalid_argument("OccupancyGrid尺寸与data长度不一致");
  }
  if (!(resolution > 0.0)) {
    throw std::invalid_argument("OccupancyGrid resolution必须为正数");
  }

  PointCloud points;
  points.reserve(expected_size / 8U);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto occupancy = data[static_cast<std::size_t>(y) * width + x];
      if (occupancy > occupied_threshold) {
        points.emplace_back(
          origin.x() + x * resolution,
          origin.y() + y * resolution);
      }
    }
  }
  return points;
}

PointCloud downsample_points(const PointCloud & points, const int factor)
{
  if (factor <= 1) {
    return points;
  }
  PointCloud result;
  result.reserve((points.size() + static_cast<std::size_t>(factor) - 1U) / factor);
  for (std::size_t index = 0; index < points.size(); index += static_cast<std::size_t>(factor)) {
    result.push_back(points[index]);
  }
  return result;
}

Bounds2d calculate_bounds(const PointCloud & points)
{
  if (points.empty()) {
    return Bounds2d{};
  }
  Bounds2d result;
  result.minimum = points.front();
  result.maximum = points.front();
  for (const auto & point : points) {
    if (!point.allFinite()) {
      continue;
    }
    result.minimum = result.minimum.cwiseMin(point);
    result.maximum = result.maximum.cwiseMax(point);
  }
  result.center = (result.minimum + result.maximum) * 0.5;
  result.width = result.maximum.x() - result.minimum.x();
  result.height = result.maximum.y() - result.minimum.y();
  result.area = result.width * result.height;
  return result;
}

Point2d calculate_center(const PointCloud & points)
{
  if (points.empty()) {
    return Point2d::Zero();
  }
  Point2d sum = Point2d::Zero();
  std::size_t count = 0;
  for (const auto & point : points) {
    if (point.allFinite()) {
      sum += point;
      ++count;
    }
  }
  if (count == 0) {
    return Point2d::Zero();
  }
  return sum / static_cast<double>(count);
}

QualityStats calculate_quality(const PointCloud & points, const QualityConfig & config)
{
  QualityStats stats;
  PointCloud finite_points;
  finite_points.reserve(points.size());
  std::unordered_set<CellIndex, CellIndexHash> fine_cells;
  std::unordered_set<CellIndex, CellIndexHash> coarse_cells;
  for (const auto & point : points) {
    if (!point.allFinite()) {
      continue;
    }
    finite_points.push_back(point);
    fine_cells.insert(point_cell(point, config.fine_resolution));
    coarse_cells.insert(point_cell(point, config.coarse_resolution));
  }
  stats.point_count = finite_points.size();
  stats.fine_cells = fine_cells.size();
  stats.coarse_cells = coarse_cells.size();
  stats.bounds = calculate_bounds(finite_points);
  return stats;
}

void validate_quality(const QualityStats & stats, const QualityConfig & config)
{
  const double long_side = std::max(stats.bounds.width, stats.bounds.height);
  if (stats.point_count >= static_cast<std::size_t>(std::max(0, config.minimum_points)) &&
    stats.fine_cells >= static_cast<std::size_t>(std::max(0, config.minimum_fine_cells)) &&
    stats.coarse_cells >= static_cast<std::size_t>(std::max(0, config.minimum_coarse_cells)) &&
    long_side >= config.minimum_long_side && stats.bounds.area >= config.minimum_area)
  {
    return;
  }

  std::ostringstream message;
  message << "小地图特征不足，无法执行地图匹配: points=" << stats.point_count << "/" <<
    config.minimum_points << ", fine_cells=" << stats.fine_cells << "/" <<
    config.minimum_fine_cells << "@" << config.fine_resolution << "m, coarse_cells=" <<
    stats.coarse_cells << "/" << config.minimum_coarse_cells << "@" <<
    config.coarse_resolution << "m, bbox=" << stats.bounds.width << "x" <<
    stats.bounds.height << "m, long_side=" << long_side << "/" <<
    config.minimum_long_side << "m, area=" << stats.bounds.area << "/" <<
    config.minimum_area << "m2";
  throw std::runtime_error(message.str());
}

AdaptiveFftMatcher::AdaptiveFftMatcher(FftMatcherConfig config)
: config_(std::move(config))
{
  if (!(config_.rotation_step_degrees > 0.0) || config_.rotation_step_degrees > 360.0) {
    throw std::invalid_argument("旋转步长必须位于(0, 360]度");
  }
  config_.maximum_workers = std::max(1, config_.maximum_workers);
  config_.minimum_candidate_count = std::max(1, config_.minimum_candidate_count);
  config_.candidate_retention_ratio = std::clamp(config_.candidate_retention_ratio, 0.0, 1.0);
}

MatchResult AdaptiveFftMatcher::match(
  const PointCloud & source_points,
  const PointCloud & target_points) const
{
  if (source_points.empty() || target_points.empty()) {
    throw std::invalid_argument("FFT匹配点云不能为空");
  }
  const auto match_started = std::chrono::steady_clock::now();
  const Point2d target_center = calculate_center(target_points);
  const auto resolutions = resolution_pyramid(config_);
  const int angle_count = static_cast<int>(std::ceil(360.0 / config_.rotation_step_degrees));
  std::vector<int> candidates(static_cast<std::size_t>(angle_count));
  std::iota(candidates.begin(), candidates.end(), 0);
  std::vector<std::optional<CandidateResult>> all_results(static_cast<std::size_t>(angle_count));
  std::vector<StageDiagnostics> diagnostics;
  diagnostics.reserve(resolutions.size());

  for (std::size_t stage_index = 0; stage_index < resolutions.size(); ++stage_index) {
    const auto stage_started = std::chrono::steady_clock::now();
    const double resolution = resolutions[stage_index];
    const auto geometry = make_grid_geometry(
      target_center, config_.search_range_x, config_.search_range_y, resolution);

    FftWorkspace target_workspace(geometry.fft_height, geometry.fft_width);
    rasterize_target(target_points, geometry, target_workspace.input());
    const auto target_spectrum = target_workspace.forward_copy();

    const unsigned int hardware_threads = std::max(1U, std::thread::hardware_concurrency());
    const std::size_t worker_count = std::min<std::size_t>(
      candidates.size(),
      static_cast<std::size_t>(std::min<int>(config_.maximum_workers, hardware_threads)));
    std::vector<std::unique_ptr<FftWorkspace>> workspaces;
    workspaces.reserve(worker_count);
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
      workspaces.push_back(std::make_unique<FftWorkspace>(geometry.fft_height, geometry.fft_width));
    }

    std::vector<CandidateResult> stage_results(candidates.size());
    std::atomic<std::size_t> next_candidate{0};
    std::exception_ptr worker_error;
    std::mutex worker_error_mutex;
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
      workers.emplace_back([&, worker]() {
        try {
          while (true) {
            const std::size_t candidate_position = next_candidate.fetch_add(1);
            if (candidate_position >= candidates.size()) {
              return;
            }
            const int angle_index = candidates[candidate_position];
            const double angle_degrees = angle_index * config_.rotation_step_degrees;
            const double angle_radians = angle_degrees * kPi / 180.0;
            auto & workspace = *workspaces[worker];
            rasterize_rotated_source(
              source_points, Eigen::Rotation2Dd(angle_radians), target_center,
              geometry, workspace.input());
            const auto [translation_delta, correlation] =
              workspace.correlate(target_spectrum, resolution);
            stage_results[candidate_position] = CandidateResult{
              angle_index,
              angle_degrees,
              angle_radians,
              translation_delta,
              correlation,
              resolution,
              static_cast<int>(stage_index + 1)};
          }
        } catch (...) {
          std::lock_guard<std::mutex> lock(worker_error_mutex);
          if (worker_error == nullptr) {
            worker_error = std::current_exception();
          }
        }
      });
    }
    for (auto & worker : workers) {
      worker.join();
    }
    if (worker_error != nullptr) {
      std::rethrow_exception(worker_error);
    }

    for (const auto & result : stage_results) {
      all_results[static_cast<std::size_t>(result.angle_index)] = result;
    }
    std::stable_sort(stage_results.begin(), stage_results.end(), candidate_better);

    StageDiagnostics stage;
    stage.stage = static_cast<int>(stage_index + 1);
    stage.resolution = resolution;
    stage.grid_width = geometry.fft_width;
    stage.grid_height = geometry.fft_height;
    stage.input_candidates = candidates.size();
    const auto top_count = std::min<std::size_t>(5, stage_results.size());
    stage.top_candidates.assign(stage_results.begin(), stage_results.begin() + top_count);

    if (stage_index + 1 < resolutions.size()) {
      const auto retained_count = std::min<std::size_t>(
        stage_results.size(),
        std::max<std::size_t>(
          static_cast<std::size_t>(config_.minimum_candidate_count),
          static_cast<std::size_t>(
            stage_results.size() * config_.candidate_retention_ratio)));
      candidates.clear();
      candidates.reserve(retained_count);
      for (std::size_t index = 0; index < retained_count; ++index) {
        candidates.push_back(stage_results[index].angle_index);
      }
      stage.retained_candidates = retained_count;
    } else {
      stage.retained_candidates = stage_results.size();
    }
    stage.elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - stage_started).count();
    diagnostics.push_back(std::move(stage));
  }

  std::optional<CandidateResult> best;
  for (const auto & result : all_results) {
    if (result && (!best || result->correlation > best->correlation)) {
      best = result;
    }
  }
  if (!best) {
    throw std::runtime_error("FFT匹配没有产生有效候选");
  }

  MatchResult result;
  result.angle_degrees = best->angle_degrees;
  result.angle_radians = best->angle_radians;
  result.translation_delta = best->translation_delta;
  result.fft_translation_offset = target_center;
  result.translation = best->translation_delta + target_center;
  result.correlation = best->correlation;
  result.stages = std::move(diagnostics);
  result.elapsed_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - match_started).count();
  return result;
}

Eigen::Isometry2d make_rigid_transform(
  const double angle_radians,
  const Point2d & translation)
{
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  transform.linear() = Eigen::Rotation2Dd(angle_radians).toRotationMatrix();
  transform.translation() = translation;
  return transform;
}

double gaussian_match_probability(
  const PointCloud & source_points,
  const PointCloud & target_points,
  const Eigen::Isometry2d & source_to_target,
  const MatchProbabilityConfig & config)
{
  if (source_points.empty() || target_points.empty()) {
    throw std::invalid_argument("Gaussian Score点云不能为空");
  }
  if (!(config.sigma > 0.0) || !(config.cell_size > 0.0) || config.search_radius < 0.0) {
    throw std::invalid_argument("Gaussian Score参数无效");
  }

  std::unordered_map<CellIndex, PointCloud, CellIndexHash> grid;
  grid.reserve(target_points.size());
  for (const auto & point : target_points) {
    grid[point_cell(point, config.cell_size)].push_back(point);
  }

  const int neighbor_cells = static_cast<int>(
    std::ceil(config.search_radius / config.cell_size));
  const int source_step = std::max(1, config.source_downsample);
  const double denominator = 2.0 * config.sigma * config.sigma;
  double probability_sum = 0.0;
  std::size_t evaluated = 0;
  for (std::size_t index = 0; index < source_points.size(); index += source_step) {
    const Point2d transformed = source_to_target * source_points[index];
    const CellIndex center = point_cell(transformed, config.cell_size);
    double best_distance_squared = std::numeric_limits<double>::infinity();
    for (int dx = -neighbor_cells; dx <= neighbor_cells; ++dx) {
      for (int dy = -neighbor_cells; dy <= neighbor_cells; ++dy) {
        const auto iterator = grid.find(CellIndex{center.x + dx, center.y + dy});
        if (iterator == grid.end()) {
          continue;
        }
        for (const auto & target : iterator->second) {
          best_distance_squared = std::min(
            best_distance_squared, (target - transformed).squaredNorm());
        }
      }
    }
    if (std::isfinite(best_distance_squared)) {
      probability_sum += std::exp(-best_distance_squared / denominator);
    }
    ++evaluated;
  }
  if (evaluated == 0) {
    throw std::runtime_error("Gaussian Score没有可评估的源点");
  }
  return std::clamp(probability_sum / evaluated * 100.0, 0.0, 100.0);
}

double normalize_angle(double angle_radians)
{
  while (angle_radians > kPi) {
    angle_radians -= 2.0 * kPi;
  }
  while (angle_radians < -kPi) {
    angle_radians += 2.0 * kPi;
  }
  return angle_radians;
}

}  // namespace map2map_matcher
