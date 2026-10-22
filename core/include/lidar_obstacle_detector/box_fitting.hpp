/* box_fitting.hpp

 * Copyright (C) 2021-2026 SS47816

 * Bounding box fitting: axis-aligned, PCA (yaw only) and L-shape fitting

**/

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/pcl_compat.hpp"
#include "lidar_obstacle_detector/types.hpp"

namespace lidar_obstacle_detector {

namespace detail {

template <typename PointT>
void zExtent(const pcl::PointCloud<PointT> &cloud, const pcl::Indices &indices, float *z_min,
             float *z_max) {
  *z_min = std::numeric_limits<float>::max();
  *z_max = std::numeric_limits<float>::lowest();
  for (const auto i : indices) {
    *z_min = std::min(*z_min, cloud.points[i].z);
    *z_max = std::max(*z_max, cloud.points[i].z);
  }
}

// Smallest rectangle aligned with heading `theta` that encloses the points;
// the longer side becomes the length and yaw is wrapped to (-pi/2, pi/2].
inline Box rectangleAt(const std::vector<Eigen::Vector2f> &xy, float theta, float z_min,
                       float z_max) {
  const Eigen::Vector2f e1(std::cos(theta), std::sin(theta));
  const Eigen::Vector2f e2(-e1.y(), e1.x());
  float min1 = std::numeric_limits<float>::max(), max1 = std::numeric_limits<float>::lowest();
  float min2 = min1, max2 = max1;
  for (const auto &p : xy) {
    const float c1 = p.dot(e1), c2 = p.dot(e2);
    min1 = std::min(min1, c1);
    max1 = std::max(max1, c1);
    min2 = std::min(min2, c2);
    max2 = std::max(max2, c2);
  }
  const Eigen::Vector2f ctr = 0.5f * (min1 + max1) * e1 + 0.5f * (min2 + max2) * e2;
  float length = max1 - min1, width = max2 - min2, yaw = theta;
  if (width > length) {
    std::swap(length, width);
    yaw += static_cast<float>(M_PI_2);
  }
  yaw = wrapAngle(yaw);
  if (yaw <= -static_cast<float>(M_PI_2)) yaw += static_cast<float>(M_PI);
  if (yaw > static_cast<float>(M_PI_2)) yaw -= static_cast<float>(M_PI);
  return Box(0, Eigen::Vector3f(ctr.x(), ctr.y(), 0.5f * (z_min + z_max)),
             Eigen::Vector3f(length, width, z_max - z_min), yaw);
}

// Closeness criterion of Zhang et al. (IV 2017): points of an L-shaped scan
// lie close to two perpendicular edges of the correct rectangle.
inline float closenessScore(const std::vector<Eigen::Vector2f> &xy, float theta) {
  const Eigen::Vector2f e1(std::cos(theta), std::sin(theta));
  const Eigen::Vector2f e2(-e1.y(), e1.x());
  const size_t n = xy.size();
  std::vector<float> c1(n), c2(n);
  float min1 = std::numeric_limits<float>::max(), max1 = std::numeric_limits<float>::lowest();
  float min2 = min1, max2 = max1;
  for (size_t i = 0; i < n; ++i) {
    c1[i] = xy[i].dot(e1);
    c2[i] = xy[i].dot(e2);
    min1 = std::min(min1, c1[i]);
    max1 = std::max(max1, c1[i]);
    min2 = std::min(min2, c2[i]);
    max2 = std::max(max2, c2[i]);
  }
  // For each axis use the edge (min or max side) the points are closer to
  float n1_min = 0.f, n1_max = 0.f, n2_min = 0.f, n2_max = 0.f;
  for (size_t i = 0; i < n; ++i) {
    n1_min += (c1[i] - min1) * (c1[i] - min1);
    n1_max += (max1 - c1[i]) * (max1 - c1[i]);
    n2_min += (c2[i] - min2) * (c2[i] - min2);
    n2_max += (max2 - c2[i]) * (max2 - c2[i]);
  }
  const bool use_min1 = n1_min < n1_max, use_min2 = n2_min < n2_max;
  constexpr float kD0 = 0.01f;
  float score = 0.f;
  for (size_t i = 0; i < n; ++i) {
    const float d1 = use_min1 ? c1[i] - min1 : max1 - c1[i];
    const float d2 = use_min2 ? c2[i] - min2 : max2 - c2[i];
    score += 1.f / std::max(std::min(d1, d2), kD0);
  }
  return score;
}

template <typename PointT>
std::vector<Eigen::Vector2f> centredXY(const pcl::PointCloud<PointT> &cloud,
                                       const pcl::Indices &indices, Eigen::Vector2f *mean) {
  std::vector<Eigen::Vector2f> xy;
  xy.reserve(indices.size());
  Eigen::Vector2f m = Eigen::Vector2f::Zero();
  for (const auto i : indices) {
    xy.emplace_back(cloud.points[i].x, cloud.points[i].y);
    m += xy.back();
  }
  m /= static_cast<float>(std::max<size_t>(xy.size(), 1));
  for (auto &p : xy) p -= m;  // improves float precision far from the sensor
  *mean = m;
  return xy;
}

}  // namespace detail

template <typename PointT>
Box fitAxisAlignedBox(const pcl::PointCloud<PointT> &cloud, const pcl::Indices &indices) {
  Eigen::Vector3f lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f hi = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  for (const auto i : indices) {
    const Eigen::Vector3f p = cloud.points[i].getVector3fMap();
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  return Box(0, 0.5f * (lo + hi), hi - lo, 0.f);
}

// Yaw from the principal axis of the x-y covariance (no roll / pitch, input
// is not modified -- both were bugs of the v1 PCA box).
template <typename PointT>
Box fitPcaBox(const pcl::PointCloud<PointT> &cloud, const pcl::Indices &indices) {
  float z_min, z_max;
  detail::zExtent(cloud, indices, &z_min, &z_max);
  Eigen::Vector2f mean;
  const auto xy = detail::centredXY(cloud, indices, &mean);
  float cxx = 0.f, cxy = 0.f, cyy = 0.f;
  for (const auto &p : xy) {
    cxx += p.x() * p.x();
    cxy += p.x() * p.y();
    cyy += p.y() * p.y();
  }
  const float theta = 0.5f * std::atan2(2.f * cxy, cxx - cyy);
  Box box = detail::rectangleAt(xy, theta, z_min, z_max);
  box.center.head<2>() += mean;
  return box;
}

// Search-based L-shape fitting (Zhang et al., "Efficient L-Shape Fitting for
// Vehicle Detection Using Laser Scanners", IV 2017), closeness criterion, with
// a fine search around the best coarse angle.
template <typename PointT>
Box fitLShapeBox(const pcl::PointCloud<PointT> &cloud, const pcl::Indices &indices,
                 float angle_step_deg = 1.f) {
  if (indices.size() < 3) return fitAxisAlignedBox(cloud, indices);
  float z_min, z_max;
  detail::zExtent(cloud, indices, &z_min, &z_max);
  Eigen::Vector2f mean;
  const auto xy = detail::centredXY(cloud, indices, &mean);

  const float step = angle_step_deg * static_cast<float>(M_PI) / 180.f;
  float best_theta = 0.f, best_score = -1.f;
  for (float theta = 0.f; theta < static_cast<float>(M_PI_2); theta += step) {
    const float s = detail::closenessScore(xy, theta);
    if (s > best_score) {
      best_score = s;
      best_theta = theta;
    }
  }
  const float fine = step / 10.f;
  const float coarse_best = best_theta;
  for (float theta = coarse_best - step + fine; theta < coarse_best + step; theta += fine) {
    const float s = detail::closenessScore(xy, theta);
    if (s > best_score) {
      best_score = s;
      best_theta = theta;
    }
  }
  Box box = detail::rectangleAt(xy, best_theta, z_min, z_max);
  box.center.head<2>() += mean;
  return box;
}

template <typename PointT>
Box fitBox(const pcl::PointCloud<PointT> &cloud, const pcl::Indices &indices,
           const BoxConfig &cfg) {
  switch (cfg.method) {
    case BoxMethod::kPca:
      return fitPcaBox(cloud, indices);
    case BoxMethod::kLShape:
      return fitLShapeBox(cloud, indices, cfg.lshape_angle_step_deg);
    case BoxMethod::kAabb:
    default:
      return fitAxisAlignedBox(cloud, indices);
  }
}

}  // namespace lidar_obstacle_detector
