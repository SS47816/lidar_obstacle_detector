/* ground_segmentation.hpp

 * Copyright (C) 2021-2026 SS47816

 * Ground segmentation: tilt-constrained RANSAC plane, or Patchwork++

**/

#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/pcl_compat.hpp"

namespace lidar_obstacle_detector {

template <typename PointT>
class GroundSegmenter {
 public:
  virtual ~GroundSegmenter() = default;
  // Splits the cloud into ground and non-ground (obstacle) point indices
  virtual void segment(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       pcl::Indices *ground, pcl::Indices *obstacles) = 0;
};

// Single ground plane fitted with RANSAC. The plane normal is constrained to
// be within `max_slope_deg` of the z axis, so walls and vehicle sides cannot be
// picked as "ground" (90 deg reproduces the unconstrained v1 behaviour).
template <typename PointT>
class RansacGroundSegmenter : public GroundSegmenter<PointT> {
 public:
  explicit RansacGroundSegmenter(const GroundConfig &cfg) : cfg_(cfg) {}

  void segment(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, pcl::Indices *ground,
               pcl::Indices *obstacles) override {
    ground->clear();
    obstacles->clear();
    if (cloud->size() < 3) {
      for (size_t i = 0; i < cloud->size(); ++i) obstacles->push_back(static_cast<int>(i));
      return;
    }
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    seg.setOptimizeCoefficients(true);
    if (cfg_.max_slope_deg < 90.f) {
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setAxis(Eigen::Vector3f::UnitZ());
      seg.setEpsAngle(cfg_.max_slope_deg * static_cast<float>(M_PI) / 180.f);
    } else {
      seg.setModelType(pcl::SACMODEL_PLANE);
    }
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(cfg_.max_iterations);
    seg.setDistanceThreshold(cfg_.distance_threshold);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    std::vector<bool> is_ground(cloud->size(), false);
    for (const auto i : inliers.indices) is_ground[i] = true;
    ground->reserve(inliers.indices.size());
    obstacles->reserve(cloud->size() - inliers.indices.size());
    for (size_t i = 0; i < cloud->size(); ++i)
      (is_ground[i] ? ground : obstacles)->push_back(static_cast<int>(i));
  }

 private:
  GroundConfig cfg_;
};

// Patchwork++ (Lee et al., IROS 2022): concentric-zone piecewise ground
// planes with adaptive thresholds; robust to slopes, kerbs and hills.
// Non-template wrapper so the vendored header stays inside one source file.
class PatchworkppSegmenterImpl {
 public:
  explicit PatchworkppSegmenterImpl(const GroundConfig &cfg);
  ~PatchworkppSegmenterImpl();
  // xyz: N x 3; fills ground / obstacle row indices
  void segment(const Eigen::MatrixXf &xyz, pcl::Indices *ground, pcl::Indices *obstacles);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

template <typename PointT>
class PatchworkppGroundSegmenter : public GroundSegmenter<PointT> {
 public:
  explicit PatchworkppGroundSegmenter(const GroundConfig &cfg) : impl_(cfg) {}

  void segment(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, pcl::Indices *ground,
               pcl::Indices *obstacles) override {
    Eigen::MatrixXf xyz(cloud->size(), 3);
    for (size_t i = 0; i < cloud->size(); ++i) {
      const auto &p = cloud->points[i];
      xyz.row(static_cast<Eigen::Index>(i)) << p.x, p.y, p.z;
    }
    impl_.segment(xyz, ground, obstacles);
  }

 private:
  PatchworkppSegmenterImpl impl_;
};

// Coarse BEV map of the local ground height: mean z of the ground points per
// cell; queries fall back to the nearest ring of cells that has ground.
class GroundHeightMap {
 public:
  explicit GroundHeightMap(float cell = 1.f, int search_cells = 3)
      : cell_(cell), search_(search_cells) {}

  // The map covers the x-y bounds of `cloud`; `ground` selects the ground points
  template <typename PointT>
  bool build(const pcl::PointCloud<PointT> &cloud, const pcl::Indices &ground) {
    nx_ = ny_ = 0;
    if (cloud.empty() || ground.empty()) return false;
    min_x_ = min_y_ = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest(), max_y = max_x;
    for (const auto &p : cloud.points) {
      min_x_ = std::min(min_x_, p.x);
      min_y_ = std::min(min_y_, p.y);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
    }
    nx_ = static_cast<int>((max_x - min_x_) / cell_) + 1;
    ny_ = static_cast<int>((max_y - min_y_) / cell_) + 1;
    if (static_cast<int64_t>(nx_) * ny_ > (int64_t{1} << 24)) {
      nx_ = ny_ = 0;
      return false;
    }
    sum_.assign(static_cast<size_t>(nx_) * ny_, 0.f);
    count_.assign(sum_.size(), 0);
    for (const auto i : ground) {
      const int c = cellIndex(cloud.points[i].x, cloud.points[i].y);
      if (c < 0) continue;
      sum_[c] += cloud.points[i].z;
      count_[c]++;
    }
    return true;
  }

  // Ground height near (x, y); false if there is no ground within the search radius
  bool height(float x, float y, float *h) const {
    if (nx_ == 0) return false;
    const int ix = static_cast<int>(std::floor((x - min_x_) / cell_));
    const int iy = static_cast<int>(std::floor((y - min_y_) / cell_));
    for (int r = 0; r <= search_; ++r) {
      float s = 0.f;
      int n = 0;
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) continue;  // ring only
          const int x2 = ix + dx, y2 = iy + dy;
          if (x2 < 0 || y2 < 0 || x2 >= nx_ || y2 >= ny_) continue;
          s += sum_[x2 * ny_ + y2];
          n += count_[x2 * ny_ + y2];
        }
      }
      if (n > 0) {
        *h = s / static_cast<float>(n);
        return true;
      }
    }
    return false;
  }

 private:
  int cellIndex(float x, float y) const {
    const int ix = static_cast<int>((x - min_x_) / cell_),
              iy = static_cast<int>((y - min_y_) / cell_);
    return (ix < 0 || iy < 0 || ix >= nx_ || iy >= ny_) ? -1 : ix * ny_ + iy;
  }

  float cell_;
  int search_;
  float min_x_ = 0.f, min_y_ = 0.f;
  int nx_ = 0, ny_ = 0;
  std::vector<float> sum_;
  std::vector<int> count_;
};

// Local elevation-map refinement. Moves non-ground points that lie within
// `threshold` of the local ground height back to the ground set. This repairs
// ground patches that a segmenter rejected as a whole (e.g. a Patchwork++ patch
// containing a vehicle), which would otherwise merge into nearby clusters.
template <typename PointT>
void refineGround(const pcl::PointCloud<PointT> &cloud, float threshold, pcl::Indices *ground,
                  pcl::Indices *obstacles, float cell = 1.f, int search_cells = 3) {
  if (ground->empty() || obstacles->empty()) return;
  GroundHeightMap map(cell, search_cells);
  if (!map.build(cloud, *ground)) return;
  pcl::Indices still_obstacles;
  still_obstacles.reserve(obstacles->size());
  for (const auto i : *obstacles) {
    const auto &p = cloud.points[i];
    float height;
    if (map.height(p.x, p.y, &height) && p.z < height + threshold)
      ground->push_back(i);
    else
      still_obstacles.push_back(i);
  }
  obstacles->swap(still_obstacles);
}

template <typename PointT>
std::unique_ptr<GroundSegmenter<PointT>> makeGroundSegmenter(const GroundConfig &cfg) {
  switch (cfg.method) {
    case GroundMethod::kPatchworkpp:
      return std::make_unique<PatchworkppGroundSegmenter<PointT>>(cfg);
    case GroundMethod::kRansac:
    default:
      return std::make_unique<RansacGroundSegmenter<PointT>>(cfg);
  }
}

}  // namespace lidar_obstacle_detector
