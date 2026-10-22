/* clustering.hpp

 * Copyright (C) 2021-2026 SS47816

 * Obstacle clustering: Euclidean (PCL), range-adaptive Euclidean, BEV grid

**/

#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/pcl_compat.hpp"

namespace lidar_obstacle_detector {

namespace detail {

inline void filterBySize(const ClusterConfig &cfg, std::vector<pcl::Indices> *clusters) {
  clusters->erase(std::remove_if(clusters->begin(), clusters->end(),
                                 [&](const pcl::Indices &c) {
                                   return static_cast<int>(c.size()) < cfg.min_size ||
                                          static_cast<int>(c.size()) > cfg.max_size;
                                 }),
                  clusters->end());
}

// Union-find with path halving
class DisjointSet {
 public:
  explicit DisjointSet(size_t n) : parent_(n) { std::iota(parent_.begin(), parent_.end(), 0); }
  int find(int x) {
    while (parent_[x] != x) x = parent_[x] = parent_[parent_[x]];
    return x;
  }
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) parent_[std::max(a, b)] = std::min(a, b);
  }

 private:
  std::vector<int> parent_;
};

}  // namespace detail

// Fixed-radius Euclidean clustering (PCL), as in v1
template <typename PointT>
std::vector<pcl::Indices> euclideanClustering(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const ClusterConfig &cfg) {
  std::vector<pcl::Indices> clusters;
  if (cloud->empty()) return clusters;
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cfg.tolerance);
  ec.setMinClusterSize(cfg.min_size);
  ec.setMaxClusterSize(cfg.max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  clusters.reserve(cluster_indices.size());
  for (auto &c : cluster_indices) clusters.push_back(std::move(c.indices));
  return clusters;
}

// Euclidean region growing whose radius grows with range,
// r(p) = max(tolerance, range_gain * |p_xy|), compensating for the beam
// divergence that makes distant objects sparse and fall apart.
template <typename PointT>
std::vector<pcl::Indices> adaptiveEuclideanClustering(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const ClusterConfig &cfg) {
  std::vector<pcl::Indices> clusters;
  if (cloud->empty()) return clusters;
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(cloud);

  std::vector<bool> visited(cloud->size(), false);
  pcl::Indices neighbours;
  std::vector<float> sq_dists;
  for (size_t seed = 0; seed < cloud->size(); ++seed) {
    if (visited[seed]) continue;
    pcl::Indices cluster{static_cast<int>(seed)};
    visited[seed] = true;
    for (size_t k = 0; k < cluster.size(); ++k) {
      const auto &p = cloud->points[cluster[k]];
      const float radius = std::max(cfg.tolerance, cfg.range_gain * std::hypot(p.x, p.y));
      tree.radiusSearch(p, radius, neighbours, sq_dists);
      for (const auto n : neighbours) {
        if (!visited[n]) {
          visited[n] = true;
          cluster.push_back(n);
        }
      }
    }
    clusters.push_back(std::move(cluster));
  }
  detail::filterBySize(cfg, &clusters);
  return clusters;
}

// Connected components on a 2D (bird's-eye-view) occupancy grid. Occupied
// cells are linked to occupied neighbours within floor(tolerance / cell_size)
// cells (more at long range when range_gain > 0). Runs in O(n) and is much
// faster than KD-tree based clustering on dense scans.
template <typename PointT>
std::vector<pcl::Indices> bevGridClustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                            const ClusterConfig &cfg) {
  std::vector<pcl::Indices> clusters;
  if (cloud->empty()) return clusters;
  const float cell = cfg.cell_size;

  float min_x = cloud->points[0].x, min_y = cloud->points[0].y;
  float max_x = min_x, max_y = min_y;
  for (const auto &p : cloud->points) {
    min_x = std::min(min_x, p.x);
    min_y = std::min(min_y, p.y);
    max_x = std::max(max_x, p.x);
    max_y = std::max(max_y, p.y);
  }
  const int nx = static_cast<int>((max_x - min_x) / cell) + 1;
  const int ny = static_cast<int>((max_y - min_y) / cell) + 1;
  // Guard against huge ROIs with tiny cells (dense grid > 64 MB)
  if (static_cast<int64_t>(nx) * ny > (int64_t{1} << 24))
    return adaptiveEuclideanClustering<PointT>(cloud, cfg);

  // Counting sort of the points into cells
  std::vector<int> point_cell(cloud->size());
  std::vector<int> cell_of(static_cast<size_t>(nx) * ny, -1);  // grid -> occupied idx
  std::vector<int> occupied;                                   // occupied idx -> grid
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &p = cloud->points[i];
    const int g =
        static_cast<int>((p.x - min_x) / cell) * ny + static_cast<int>((p.y - min_y) / cell);
    if (cell_of[g] < 0) {
      cell_of[g] = static_cast<int>(occupied.size());
      occupied.push_back(g);
    }
    point_cell[i] = cell_of[g];
  }

  detail::DisjointSet sets(occupied.size());
  for (size_t c = 0; c < occupied.size(); ++c) {
    const int gx = occupied[c] / ny, gy = occupied[c] % ny;
    float tol = cfg.tolerance;
    if (cfg.range_gain > 0.f) {
      const float cx = min_x + (gx + 0.5f) * cell, cy = min_y + (gy + 0.5f) * cell;
      tol = std::max(tol, cfg.range_gain * std::hypot(cx, cy));
    }
    const int k = std::max(1, static_cast<int>(tol / cell));
    // Visit only the "forward" half of the neighbourhood; links are symmetric
    for (int dx = 0; dx <= k; ++dx) {
      const int x = gx + dx;
      if (x >= nx) break;
      for (int dy = (dx == 0 ? 1 : -k); dy <= k; ++dy) {
        const int y = gy + dy;
        if (y < 0 || y >= ny) continue;
        const int other = cell_of[x * ny + y];
        if (other >= 0) sets.unite(static_cast<int>(c), other);
      }
    }
  }

  std::vector<int> label(occupied.size(), -1);
  for (size_t i = 0; i < cloud->size(); ++i) {
    const int root = sets.find(point_cell[i]);
    if (label[root] < 0) {
      label[root] = static_cast<int>(clusters.size());
      clusters.emplace_back();
    }
    clusters[label[root]].push_back(static_cast<int>(i));
  }
  detail::filterBySize(cfg, &clusters);
  return clusters;
}

template <typename PointT>
std::vector<pcl::Indices> clusterCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                       const ClusterConfig &cfg) {
  switch (cfg.method) {
    case ClusterMethod::kAdaptiveEuclidean:
      return adaptiveEuclideanClustering<PointT>(cloud, cfg);
    case ClusterMethod::kBevGrid:
      return bevGridClustering<PointT>(cloud, cfg);
    case ClusterMethod::kEuclidean:
    default:
      return euclideanClustering<PointT>(cloud, cfg);
  }
}

}  // namespace lidar_obstacle_detector
