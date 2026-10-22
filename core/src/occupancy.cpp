/* occupancy.cpp

 * Copyright (C) 2021-2026 SS47816

 * 2D occupancy grid with polar ray casting for free space

**/

#include "lidar_obstacle_detector/occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_obstacle_detector {

void OccupancyMapper::prepare(const Eigen::Vector2f &lower, const Eigen::Vector2f &upper,
                              float resolution) {
  if (lower == lower_ && upper == upper_ && resolution == resolution_ && !cell_bin_.empty()) return;
  lower_ = lower;
  upper_ = upper;
  resolution_ = resolution;
  width_ = std::max(1, static_cast<int>(std::ceil((upper.x() - lower.x()) / resolution)));
  height_ = std::max(1, static_cast<int>(std::ceil((upper.y() - lower.y()) / resolution)));
  cell_bin_.resize(static_cast<size_t>(width_) * height_);
  cell_range_.resize(cell_bin_.size());
  const float bins_per_rad = kBins / (2.f * static_cast<float>(M_PI));
  for (int iy = 0; iy < height_; ++iy) {
    for (int ix = 0; ix < width_; ++ix) {
      const float x = lower.x() + (ix + 0.5f) * resolution;
      const float y = lower.y() + (iy + 0.5f) * resolution;
      const size_t c = static_cast<size_t>(iy) * width_ + ix;
      const float bearing = std::atan2(y, x) + static_cast<float>(M_PI);  // [0, 2 pi]
      cell_bin_[c] =
          static_cast<int16_t>(std::min(kBins - 1, static_cast<int>(bearing * bins_per_rad)));
      cell_range_[c] = std::hypot(x, y);
    }
  }
}

void OccupancyMapper::build(const std::vector<Eigen::Vector2f> &occupied,
                            const std::vector<Eigen::Vector2f> &ground,
                            const Eigen::Vector2f &lower, const Eigen::Vector2f &upper,
                            float resolution, OccupancyGrid2D *grid) {
  prepare(lower, upper, resolution);
  grid->resolution = resolution;
  grid->origin = lower;
  grid->width = width_;
  grid->height = height_;
  grid->data.assign(cell_bin_.size(), OccupancyGrid2D::kUnknown);

  std::vector<float> first_hit(kBins, std::numeric_limits<float>::infinity());
  std::vector<float> seen_until(kBins, 0.f);  // farthest ground return per bearing
  int ix, iy;
  for (const auto &p : occupied) {
    if (!grid->cellOf(p.x(), p.y(), &ix, &iy)) continue;
    const size_t c = static_cast<size_t>(iy) * width_ + ix;
    grid->data[c] = OccupancyGrid2D::kOccupied;
    first_hit[cell_bin_[c]] = std::min(first_hit[cell_bin_[c]], cell_range_[c]);
  }
  for (const auto &p : ground) {
    if (!grid->cellOf(p.x(), p.y(), &ix, &iy)) continue;
    const size_t c = static_cast<size_t>(iy) * width_ + ix;
    seen_until[cell_bin_[c]] = std::max(seen_until[cell_bin_[c]], cell_range_[c]);
    if (grid->data[c] != OccupancyGrid2D::kOccupied) grid->data[c] = OccupancyGrid2D::kFree;
  }
  // Ray casting: along each bearing, cells before the first obstacle and within
  // the observed range are free; cells behind it stay unknown
  for (size_t c = 0; c < grid->data.size(); ++c) {
    if (grid->data[c] != OccupancyGrid2D::kUnknown) continue;
    const int b = cell_bin_[c];
    if (cell_range_[c] < first_hit[b] && cell_range_[c] <= seen_until[b] + resolution)
      grid->data[c] = OccupancyGrid2D::kFree;
  }
}

}  // namespace lidar_obstacle_detector
