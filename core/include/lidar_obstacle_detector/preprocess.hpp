/* preprocess.hpp

 * Copyright (C) 2021-2026 SS47816

 * Region-of-interest cropping, ego-vehicle removal and voxel downsampling

**/

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

#include "lidar_obstacle_detector/config.hpp"

namespace lidar_obstacle_detector {

// Crops the ROI and removes the ego vehicle in a single pass, then
// voxel-downsamples what is left (cropping first keeps the voxel grid small).
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr preprocessCloud(
    const pcl::PointCloud<PointT> &cloud, const PreprocessConfig &cfg) {
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

  typename pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cropped);
  vg.setLeafSize(cfg.voxel_size, cfg.voxel_size, cfg.voxel_size);
  vg.filter(*downsampled);
  return downsampled;
}

}  // namespace lidar_obstacle_detector
