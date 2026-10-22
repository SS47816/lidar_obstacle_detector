/* clustering.hpp

 * Copyright (C) 2021-2026 SS47816

 * Obstacle clustering: Euclidean (PCL), range-adaptive Euclidean, BEV grid

**/

#pragma once

#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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
  const auto cellRange = [&](int g) {
    const float cx = min_x + (g / ny + 0.5f) * cell, cy = min_y + (g % ny + 0.5f) * cell;
    return std::hypot(cx, cy);
  };

  // Split every cell's column into height segments at vertical gaps, so that
  // e.g. a tree canopy does not merge with the car below it. The gap grows with
  // range because the vertical spacing of lidar beams does (~0.7 m at 30 m).
  std::vector<int> cell_start(occupied.size() + 1, 0), order(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) cell_start[point_cell[i] + 1]++;
  for (size_t c = 0; c < occupied.size(); ++c) cell_start[c + 1] += cell_start[c];
  {
    std::vector<int> fill(cell_start.begin(), cell_start.end() - 1);
    for (size_t i = 0; i < cloud->size(); ++i) order[fill[point_cell[i]]++] = static_cast<int>(i);
  }
  struct Segment {
    float z_min, z_max;
  };
  std::vector<Segment> segments;
  std::vector<int> seg_start(occupied.size() + 1, 0), point_seg(cloud->size());
  std::vector<float> seg_gap(occupied.size(), 0.f);
  for (size_t c = 0; c < occupied.size(); ++c) {
    seg_start[c] = static_cast<int>(segments.size());
    auto first = order.begin() + cell_start[c], last = order.begin() + cell_start[c + 1];
    if (cfg.z_gap <= 0.f) {
      Segment seg{std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()};
      for (auto it = first; it != last; ++it) {
        seg.z_min = std::min(seg.z_min, cloud->points[*it].z);
        seg.z_max = std::max(seg.z_max, cloud->points[*it].z);
        point_seg[*it] = static_cast<int>(segments.size());
      }
      segments.push_back(seg);
      continue;
    }
    std::sort(first, last, [&](int a, int b) { return cloud->points[a].z < cloud->points[b].z; });
    const float gap = std::max(cfg.z_gap, 0.04f * cellRange(occupied[c]));
    seg_gap[c] = gap;
    for (auto it = first; it != last; ++it) {
      const float z = cloud->points[*it].z;
      if (it == first || z - segments.back().z_max > gap) segments.push_back({z, z});
      segments.back().z_max = z;
      point_seg[*it] = static_cast<int>(segments.size()) - 1;
    }
  }
  seg_start[occupied.size()] = static_cast<int>(segments.size());

  detail::DisjointSet sets(segments.size());
  for (size_t c = 0; c < occupied.size(); ++c) {
    const int gx = occupied[c] / ny, gy = occupied[c] % ny;
    float tol = cfg.tolerance;
    if (cfg.range_gain > 0.f) tol = std::max(tol, cfg.range_gain * cellRange(occupied[c]));
    const int k = std::max(1, static_cast<int>(tol / cell));
    const float gap = seg_gap[c];
    // Visit only the "forward" half of the neighbourhood; links are symmetric
    for (int dx = 0; dx <= k; ++dx) {
      const int x = gx + dx;
      if (x >= nx) break;
      for (int dy = (dx == 0 ? 1 : -k); dy <= k; ++dy) {
        const int y = gy + dy;
        if (y < 0 || y >= ny) continue;
        const int other = cell_of[x * ny + y];
        if (other < 0) continue;
        for (int a = seg_start[c]; a < seg_start[c + 1]; ++a) {
          for (int b = seg_start[other]; b < seg_start[other + 1]; ++b) {
            if (cfg.z_gap <= 0.f || (segments[a].z_min <= segments[b].z_max + gap &&
                                     segments[b].z_min <= segments[a].z_max + gap))
              sets.unite(a, b);
          }
        }
      }
    }
  }

  std::vector<int> label(segments.size(), -1);
  for (size_t i = 0; i < cloud->size(); ++i) {
    const int root = sets.find(point_seg[i]);
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

namespace detail {

template <typename PointT>
bool isOversized(const pcl::PointCloud<PointT> &cloud, const pcl::Indices &indices,
                 const ClusterConfig &cfg) {
  if (static_cast<int>(indices.size()) > cfg.max_size) return true;
  // Cheap yaw-only PCA extent; L-shape fitting on huge clusters is not needed here
  float sxx = 0.f, sxy = 0.f, syy = 0.f, mx = 0.f, my = 0.f;
  for (const auto i : indices) {
    mx += cloud.points[i].x;
    my += cloud.points[i].y;
  }
  mx /= indices.size();
  my /= indices.size();
  for (const auto i : indices) {
    const float dx = cloud.points[i].x - mx, dy = cloud.points[i].y - my;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  const float theta = 0.5f * std::atan2(2.f * sxy, sxx - syy);
  const float c = std::cos(theta), s = std::sin(theta);
  float lo1 = std::numeric_limits<float>::max(), hi1 = std::numeric_limits<float>::lowest();
  float lo2 = lo1, hi2 = hi1;
  for (const auto i : indices) {
    const float dx = cloud.points[i].x - mx, dy = cloud.points[i].y - my;
    const float u = c * dx + s * dy, v = -s * dx + c * dy;
    lo1 = std::min(lo1, u), hi1 = std::max(hi1, u);
    lo2 = std::min(lo2, v), hi2 = std::max(hi2, v);
  }
  const float length = std::max(hi1 - lo1, hi2 - lo2), width = std::min(hi1 - lo1, hi2 - lo2);
  return length > cfg.object_max_length || width > cfg.object_max_width;
}

template <typename PointT>
void splitRecursively(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                      const pcl::Indices &indices, ClusterConfig cfg, int depth,
                      std::vector<pcl::Indices> *objects, std::vector<pcl::Indices> *structures) {
  if (!isOversized(*cloud, indices, cfg)) {
    objects->push_back(indices);
    return;
  }
  cfg.tolerance *= 0.5f;
  cfg.cell_size = std::min(cfg.cell_size, cfg.tolerance);
  cfg.range_gain *= 0.5f;
  if (!cfg.split_oversized || depth >= 3 || cfg.tolerance < 0.2f) {
    structures->push_back(indices);
    return;
  }
  typename pcl::PointCloud<PointT>::Ptr sub(new pcl::PointCloud<PointT>);
  pcl::copyPointCloud(*cloud, indices, *sub);
  ClusterConfig sub_cfg = cfg;
  sub_cfg.max_size = std::numeric_limits<int>::max();
  const std::vector<pcl::Indices> pieces = clusterCloud<PointT>(sub, sub_cfg);
  if (pieces.size() < 2) {
    structures->push_back(indices);
    return;
  }
  for (const auto &piece : pieces) {
    pcl::Indices mapped;
    mapped.reserve(piece.size());
    for (const auto i : piece) mapped.push_back(indices[i]);
    splitRecursively<PointT>(cloud, mapped, cfg, depth + 1, objects, structures);
  }
}

}  // namespace detail

// Clusters larger than an object (by points or by extent) are re-clustered
// with half the tolerance, recursively; what remains oversized is moved to
// `structures` (walls, vegetation, buildings).
template <typename PointT>
void splitOversizedClusters(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const ClusterConfig &cfg, std::vector<pcl::Indices> *clusters,
                            std::vector<pcl::Indices> *structures) {
  std::vector<pcl::Indices> objects;
  objects.reserve(clusters->size());
  for (const auto &c : *clusters)
    detail::splitRecursively<PointT>(cloud, c, cfg, 0, &objects, structures);
  clusters->swap(objects);
}

}  // namespace lidar_obstacle_detector
