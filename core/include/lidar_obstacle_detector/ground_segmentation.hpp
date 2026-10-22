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

  void segment(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
               pcl::Indices *ground, pcl::Indices *obstacles) override {
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

  void segment(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
               pcl::Indices *ground, pcl::Indices *obstacles) override {
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

// Local elevation-map refinement. Builds a coarse BEV height map (mean z of
// the ground points per cell) and moves non-ground points that lie within
// `threshold` of the local ground height back to the ground set. This repairs
// ground patches that a segmenter rejected as a whole (e.g. a Patchwork++ patch
// containing a vehicle), which would otherwise merge into nearby clusters.
template <typename PointT>
void refineGround(const pcl::PointCloud<PointT> &cloud, float threshold, pcl::Indices *ground,
                  pcl::Indices *obstacles, float cell = 1.f, int search_cells = 3) {
  if (ground->empty() || obstacles->empty()) return;
  float min_x = std::numeric_limits<float>::max(), min_y = min_x;
  float max_x = std::numeric_limits<float>::lowest(), max_y = max_x;
  for (const auto &p : cloud.points) {
    min_x = std::min(min_x, p.x);
    min_y = std::min(min_y, p.y);
    max_x = std::max(max_x, p.x);
    max_y = std::max(max_y, p.y);
  }
  const int nx = static_cast<int>((max_x - min_x) / cell) + 1;
  const int ny = static_cast<int>((max_y - min_y) / cell) + 1;
  if (static_cast<int64_t>(nx) * ny > (int64_t{1} << 24)) return;
  std::vector<float> sum(static_cast<size_t>(nx) * ny, 0.f);
  std::vector<int> count(sum.size(), 0);
  const auto cellOf = [&](const PointT &p, int *ix, int *iy) {
    *ix = static_cast<int>((p.x - min_x) / cell);
    *iy = static_cast<int>((p.y - min_y) / cell);
  };
  for (const auto i : *ground) {
    int ix, iy;
    cellOf(cloud.points[i], &ix, &iy);
    sum[ix * ny + iy] += cloud.points[i].z;
    count[ix * ny + iy]++;
  }

  pcl::Indices still_obstacles;
  still_obstacles.reserve(obstacles->size());
  for (const auto i : *obstacles) {
    const auto &p = cloud.points[i];
    int ix, iy;
    cellOf(p, &ix, &iy);
    // nearest ring of cells that contains ground
    float height = std::numeric_limits<float>::quiet_NaN();
    for (int r = 0; r <= search_cells && std::isnan(height); ++r) {
      float s = 0.f;
      int n = 0;
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) continue;  // ring only
          const int x = ix + dx, y = iy + dy;
          if (x < 0 || y < 0 || x >= nx || y >= ny) continue;
          s += sum[x * ny + y];
          n += count[x * ny + y];
        }
      }
      if (n > 0) height = s / static_cast<float>(n);
    }
    if (!std::isnan(height) && p.z < height + threshold)
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
