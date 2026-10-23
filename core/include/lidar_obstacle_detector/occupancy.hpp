/* occupancy.hpp

 * Copyright (C) 2021-2026 SS47816

 * Occupancy outputs: a 2D bird's-eye grid with free / occupied / unknown cells
 * (free space by ray casting from the sensor), 3D occupied voxels and 2.5D
 * obstacle pillars fused over time

**/

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/ground_segmentation.hpp"
#include "lidar_obstacle_detector/preprocess.hpp"
#include "lidar_obstacle_detector/types.hpp"

namespace lidar_obstacle_detector {

// Same layout as nav_msgs/OccupancyGrid: data[iy * width + ix], -1 unknown,
// 0 free, 100 occupied; cell (0, 0) has its lower corner at `origin`.
struct OccupancyGrid2D {
  static constexpr int8_t kUnknown = -1, kFree = 0, kOccupied = 100;
  float resolution = 0.f;
  Eigen::Vector2f origin = Eigen::Vector2f::Zero();
  int width = 0, height = 0;
  std::vector<int8_t> data;

  bool cellOf(float x, float y, int *ix, int *iy) const {
    *ix = static_cast<int>(std::floor((x - origin.x()) / resolution));
    *iy = static_cast<int>(std::floor((y - origin.y()) / resolution));
    return *ix >= 0 && *iy >= 0 && *ix < width && *iy < height;
  }
  int8_t at(int ix, int iy) const { return data[static_cast<size_t>(iy) * width + ix]; }
  int8_t atPoint(float x, float y) const {
    int ix, iy;
    return cellOf(x, y, &ix, &iy) ? at(ix, iy) : kUnknown;
  }
};

// Builds the 2D grid from occupied and ground-observed cells. The geometry of
// the grid (each cell's bearing and range from the sensor) is cached between
// calls, so ray casting costs one pass over the cells per scan.
class OccupancyMapper {
 public:
  // `occupied` / `ground`: x-y of obstacle points in the height band and of
  // ground points, in the sensor frame (sensor at the origin)
  void build(const std::vector<Eigen::Vector2f> &occupied,
             const std::vector<Eigen::Vector2f> &ground, const Eigen::Vector2f &lower,
             const Eigen::Vector2f &upper, float resolution, OccupancyGrid2D *grid);

 private:
  void prepare(const Eigen::Vector2f &lower, const Eigen::Vector2f &upper, float resolution);

  static constexpr int kBins = 720;  // 0.5 deg bearing bins for ray casting
  Eigen::Vector2f lower_ = Eigen::Vector2f::Zero(), upper_ = Eigen::Vector2f::Zero();
  float resolution_ = 0.f;
  int width_ = 0, height_ = 0;
  std::vector<int16_t> cell_bin_;
  std::vector<float> cell_range_;
};

// 2.5D pillars: per-scan obstacle columns accumulated in a fixed frame with
// log-odds evidence. Cells hit in this scan are always reported; remembered
// cells fade when the sensor sees free space through them, slowly when they
// are occluded, and at once when a recognised object now covers them.
class PillarMapper {
 public:
  // `samples`: x, y, local ground z, point z of obstacle points, in the
  // tracking frame. `grid` (sensor frame) tells which cells are seen free.
  // `objects`: boxes in the sensor frame whose area is not a pillar.
  // `memory` [s] <= 0 reports this scan only. Pillars are written in the
  // sensor frame.
  void update(const std::vector<Eigen::Vector4f> &samples, const OccupancyGrid2D &grid,
              const std::vector<Box> &objects, const Eigen::Isometry3f &tracking_T_sensor,
              double stamp, float resolution, float memory, std::vector<Pillar> *pillars);
  void reset() { cells_.clear(); }
  size_t size() const { return cells_.size(); }

 private:
  struct Cell {
    int64_t key = 0;  // packed cell indices in the tracking frame
    float log_odds = 0.f;
    float z_min = 0.f, z_max = 0.f;
    float scan_min = 0.f, scan_max = 0.f;  // column of this scan
    bool hit = false;                      // in this scan
  };
  // Cells live in a vector, found through an open-addressing index that is
  // rebuilt every scan (cheaper than a node-based hash map at this size)
  Cell &find(int64_t key);
  void reindex();

  std::vector<Cell> cells_;
  std::vector<int32_t> index_;  // into cells_, -1 = empty slot
  float resolution_ = 0.f;
  double last_stamp_ = 0.0;
};

// Computes the occupancy outputs for one scan. Obstacle points count as
// occupied when they lie between min_height and max_height above the local
// ground (so overhanging branches and the ground itself are ignored).
// `pillar_samples` (optional) receives x, y, local ground z and z (sensor frame)
// of the occupied points that `pillar_mask` (per obstacle point, null = all)
// selects for the pillar map.
template <typename PointT>
void computeOccupancy(const pcl::PointCloud<PointT> &ground,
                      const pcl::PointCloud<PointT> &obstacles, const PreprocessConfig &roi,
                      const OccupancyConfig &cfg, OccupancyMapper *mapper, OccupancyGrid2D *grid,
                      pcl::PointCloud<PointT> *voxels,
                      const std::vector<uint8_t> *pillar_mask = nullptr,
                      std::vector<Eigen::Vector4f> *pillar_samples = nullptr) {
  // Local ground height over the ROI
  pcl::PointCloud<PointT> bounds_and_ground = ground;
  PointT lo, hi;
  lo.x = roi.roi_min.x(), lo.y = roi.roi_min.y(), lo.z = 0.f;
  hi.x = roi.roi_max.x(), hi.y = roi.roi_max.y(), hi.z = 0.f;
  bounds_and_ground.push_back(lo);
  bounds_and_ground.push_back(hi);
  pcl::Indices ground_idx(ground.size());
  for (size_t i = 0; i < ground.size(); ++i) ground_idx[i] = static_cast<int>(i);
  GroundHeightMap height_map(1.f, 5);
  height_map.build(bounds_and_ground, ground_idx);
  float fallback = 0.f;
  for (const auto &p : ground.points) fallback += p.z;
  const bool have_fallback = !ground.empty();
  if (have_fallback) fallback /= static_cast<float>(ground.size());

  pcl::PointCloud<PointT> band;
  band.reserve(obstacles.size());
  std::vector<Eigen::Vector2f> occupied, observed_ground;
  occupied.reserve(obstacles.size());
  if (pillar_samples) pillar_samples->clear();
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const PointT &p = obstacles.points[i];
    float h;
    if (!height_map.height(p.x, p.y, &h)) {
      if (!have_fallback) continue;
      h = fallback;
    }
    const float above = p.z - h;
    if (above < cfg.min_height || above > cfg.max_height) continue;
    band.push_back(p);
    occupied.emplace_back(p.x, p.y);
    if (pillar_samples && (!pillar_mask || (*pillar_mask)[i]))
      pillar_samples->emplace_back(p.x, p.y, h, p.z);
  }
  observed_ground.reserve(ground.size());
  for (const auto &p : ground.points) observed_ground.emplace_back(p.x, p.y);

  mapper->build(occupied, observed_ground, roi.roi_min.head<2>(), roi.roi_max.head<2>(),
                cfg.resolution, grid);
  voxels->clear();
  if (!band.empty()) *voxels = *voxelDownsample(band, cfg.voxel_size, roi.roi_min);
}

}  // namespace lidar_obstacle_detector
