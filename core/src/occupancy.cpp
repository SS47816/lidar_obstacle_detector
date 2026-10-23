/* occupancy.cpp

 * Copyright (C) 2021-2026 SS47816

 * 2D occupancy grid with polar ray casting for free space, and the 2.5D
 * pillar map

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

namespace {

// Log-odds evidence of the pillar map
constexpr float kHit = 1.f, kMiss = -1.5f, kMax = 3.f, kShow = 0.5f;
constexpr float kHeightSmoothing = 0.5f;  // EMA weight of a new ground height
constexpr float kTopSink = 0.1f;          // EMA weight of a lower column top
constexpr float kObjectMargin = 0.3f;     // [m] around recognised objects

int64_t cellKey(int ix, int iy) {
  return (static_cast<int64_t>(ix) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(iy));
}

bool insideBox(const Box &b, const Eigen::Vector2f &p, float margin) {
  const Eigen::Vector2f d = p - b.center.head<2>();
  const float c = std::cos(b.yaw), s = std::sin(b.yaw);
  return std::abs(c * d.x() + s * d.y()) <= 0.5f * b.size.x() + margin &&
         std::abs(-s * d.x() + c * d.y()) <= 0.5f * b.size.y() + margin;
}

uint64_t hashKey(int64_t key) {  // splitmix64 finaliser
  uint64_t x = static_cast<uint64_t>(key) + 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

}  // namespace

void PillarMapper::reindex() {
  size_t capacity = 1024;
  while (capacity < 2 * cells_.size() + 2) capacity *= 2;
  index_.assign(capacity, -1);
  const uint64_t mask = capacity - 1;
  for (size_t k = 0; k < cells_.size(); ++k) {
    uint64_t slot = hashKey(cells_[k].key) & mask;
    while (index_[slot] >= 0) slot = (slot + 1) & mask;
    index_[slot] = static_cast<int32_t>(k);
  }
}

PillarMapper::Cell &PillarMapper::find(int64_t key) {
  if (2 * cells_.size() + 2 > index_.size()) reindex();
  const uint64_t mask = index_.size() - 1;
  uint64_t slot = hashKey(key) & mask;
  while (index_[slot] >= 0) {
    Cell &c = cells_[index_[slot]];
    if (c.key == key) return c;
    slot = (slot + 1) & mask;
  }
  index_[slot] = static_cast<int32_t>(cells_.size());
  Cell c;
  c.key = key;
  cells_.push_back(c);
  return cells_.back();
}

void PillarMapper::update(const std::vector<Eigen::Vector4f> &samples, const OccupancyGrid2D &grid,
                          const std::vector<Box> &objects,
                          const Eigen::Isometry3f &tracking_T_sensor, double stamp,
                          float resolution, float memory, std::vector<Pillar> *pillars) {
  if (memory <= 0.f || resolution != resolution_ || stamp < last_stamp_) cells_.clear();
  const float dt = cells_.empty() ? 0.f : std::min(1.f, static_cast<float>(stamp - last_stamp_));
  resolution_ = resolution;
  last_stamp_ = stamp;
  for (auto &c : cells_) c.hit = false;
  reindex();

  // This scan's columns (lowest ground, highest point per cell). Consecutive
  // samples often fall into the same cell, so the last cell is cached.
  cells_.reserve(cells_.size() + samples.size() / 4);
  const float inv_res = 1.f / resolution;
  int64_t last_key = 0;
  int last = -1;
  for (const auto &smp : samples) {
    const int64_t key = cellKey(static_cast<int>(std::floor(smp.x() * inv_res)),
                                static_cast<int>(std::floor(smp.y() * inv_res)));
    if (last < 0 || key != last_key) {
      last = static_cast<int>(&find(key) - cells_.data());
      last_key = key;
    }
    Cell &c = cells_[last];
    if (!c.hit) {
      c.hit = true;
      c.scan_min = smp.z();
      c.scan_max = smp.w();
    } else {
      c.scan_min = std::min(c.scan_min, smp.z());
      c.scan_max = std::max(c.scan_max, smp.w());
    }
  }

  const Eigen::Isometry3f sensor_T_tracking = tracking_T_sensor.inverse();
  const float decay = memory > 0.f ? (kMax - kShow) / memory : 0.f;
  pillars->clear();
  pillars->reserve(cells_.size());
  size_t kept = 0;
  for (size_t k = 0; k < cells_.size(); ++k) {
    Cell c = cells_[k];
    if (c.hit) {
      if (c.log_odds <= 0.f) {  // new cell
        c.log_odds = kHit;
        c.z_min = c.scan_min;
        c.z_max = c.scan_max;
      } else {
        // The top rises at once (safety) and sinks slowly (sparse returns)
        c.log_odds = std::min(kMax, c.log_odds + kHit);
        c.z_min += kHeightSmoothing * (c.scan_min - c.z_min);
        c.z_max = c.scan_max > c.z_max ? c.scan_max : c.z_max + kTopSink * (c.scan_max - c.z_max);
      }
    }
    const int ix = static_cast<int>(c.key >> 32);
    const int iy = static_cast<int>(static_cast<int32_t>(c.key & 0xffffffff));
    const Eigen::Vector3f lo =
        sensor_T_tracking *
        Eigen::Vector3f((ix + 0.5f) * resolution, (iy + 0.5f) * resolution, c.z_min);
    const float height = sensor_T_tracking.linear()(2, 2) * (c.z_max - c.z_min);
    const Eigen::Vector2f xy = lo.head<2>();
    // Remembered cells: free space clears them, occlusion lets them fade, and
    // a recognised object covering them removes them
    if (!c.hit) {
      int gx, gy;
      if (!grid.cellOf(xy.x(), xy.y(), &gx, &gy)) continue;  // left the region of interest
      const bool covered = std::any_of(objects.begin(), objects.end(), [&](const Box &b) {
        return insideBox(b, xy, kObjectMargin);
      });
      if (covered) continue;
      c.log_odds += grid.at(gx, gy) == OccupancyGrid2D::kFree ? kMiss : -decay * dt;
      if (c.log_odds < kShow) continue;
    }
    Pillar p;
    p.center = xy;
    p.z_min = lo.z();
    p.z_max = lo.z() + std::max(0.f, height);
    p.observed = c.hit;
    pillars->push_back(p);
    cells_[kept++] = c;
  }
  cells_.resize(memory > 0.f ? kept : 0);
}

}  // namespace lidar_obstacle_detector
