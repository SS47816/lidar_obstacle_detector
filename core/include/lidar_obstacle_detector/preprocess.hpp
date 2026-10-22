/* preprocess.hpp

 * Copyright (C) 2021-2026 SS47816

 * Region-of-interest cropping, ego-vehicle removal and voxel downsampling

**/

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "lidar_obstacle_detector/config.hpp"

namespace lidar_obstacle_detector {

// Replaces the points of each occupied voxel by their centroid (x, y, z and,
// if present, intensity), like pcl::VoxelGrid on the same global grid, but in a
// single hashed pass instead of a sort. `lower` is a lower bound of the points.
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxelDownsample(const pcl::PointCloud<PointT> &cloud,
                                                      float leaf, const Eigen::Vector3f &lower) {
  constexpr bool kHasIntensity = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
  struct Cell {
    double x = 0, y = 0, z = 0, intensity = 0;
    uint32_t n = 0, first = 0;
  };
  const float inv = 1.f / leaf;
  // Shift so that all voxel coordinates are >= 0 and truncation == floor
  const Eigen::Vector3f origin = (lower * inv).array().floor() - 1.f;
  // Open-addressing hash table (linear probing) from voxel key to cell slot,
  // kept at most half full and small enough to stay in cache
  size_t capacity = 1024;
  while (capacity < cloud.size() / 2) capacity <<= 1;
  constexpr uint64_t kEmpty = ~uint64_t{0};
  std::vector<uint64_t> keys(capacity, kEmpty);
  std::vector<uint32_t> slots(capacity);
  std::vector<Cell> cells;
  cells.reserve(capacity / 2);
  const auto find = [&](uint64_t key) {
    const size_t mask = keys.size() - 1;
    size_t h = (key * 0x9E3779B97F4A7C15ull) >> 24 & mask;
    while (keys[h] != kEmpty && keys[h] != key) h = (h + 1) & mask;
    return h;
  };
  for (uint32_t i = 0; i < cloud.size(); ++i) {
    const auto &p = cloud.points[i];
    // 21 bits per axis: 2 M voxels per axis, plenty for any ROI
    const auto ix = static_cast<uint64_t>(p.x * inv - origin.x()) & 0x1FFFFF;
    const auto iy = static_cast<uint64_t>(p.y * inv - origin.y()) & 0x1FFFFF;
    const auto iz = static_cast<uint64_t>(p.z * inv - origin.z()) & 0x1FFFFF;
    const uint64_t key = (ix << 42) | (iy << 21) | iz;
    size_t h = find(key);
    if (keys[h] == kEmpty) {
      if (2 * (cells.size() + 1) > keys.size()) {  // grow and rehash
        std::vector<uint64_t> old_keys(keys.size() * 2, kEmpty);
        old_keys.swap(keys);
        std::vector<uint32_t> old_slots(keys.size());
        old_slots.swap(slots);
        for (size_t j = 0; j < old_keys.size(); ++j) {
          if (old_keys[j] == kEmpty) continue;
          const size_t k = find(old_keys[j]);
          keys[k] = old_keys[j];
          slots[k] = old_slots[j];
        }
        h = find(key);
      }
      keys[h] = key;
      slots[h] = static_cast<uint32_t>(cells.size());
      cells.emplace_back();
      cells.back().first = i;
    }
    Cell &c = cells[slots[h]];
    c.x += p.x;
    c.y += p.y;
    c.z += p.z;
    if constexpr (kHasIntensity) c.intensity += p.intensity;
    ++c.n;
  }

  typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
  out->reserve(cells.size());
  for (const auto &c : cells) {
    PointT p = cloud.points[c.first];  // keeps any other fields of the first point
    p.x = static_cast<float>(c.x / c.n);
    p.y = static_cast<float>(c.y / c.n);
    p.z = static_cast<float>(c.z / c.n);
    if constexpr (kHasIntensity) p.intensity = static_cast<float>(c.intensity / c.n);
    out->push_back(p);
  }
  out->header = cloud.header;
  return out;
}

// Crops the ROI and removes the ego vehicle in a single pass, then
// voxel-downsamples what is left (cropping first keeps the voxel grid small).
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr preprocessCloud(const pcl::PointCloud<PointT> &cloud,
                                                      const PreprocessConfig &cfg) {
  typename pcl::PointCloud<PointT>::Ptr cropped(new pcl::PointCloud<PointT>);
  cropped->reserve(cloud.size());
  for (const auto &p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.x < cfg.roi_min.x() || p.x > cfg.roi_max.x() || p.y < cfg.roi_min.y() ||
        p.y > cfg.roi_max.y() || p.z < cfg.roi_min.z() || p.z > cfg.roi_max.z())
      continue;
    if (cfg.remove_ego && p.x >= cfg.ego_min.x() && p.x <= cfg.ego_max.x() &&
        p.y >= cfg.ego_min.y() && p.y <= cfg.ego_max.y() && p.z >= cfg.ego_min.z() &&
        p.z <= cfg.ego_max.z())
      continue;
    cropped->push_back(p);
  }
  cropped->header = cloud.header;
  if (cfg.voxel_size <= 0.f || cropped->empty()) return cropped;
  return voxelDownsample(*cropped, cfg.voxel_size, cfg.roi_min);
}

}  // namespace lidar_obstacle_detector
