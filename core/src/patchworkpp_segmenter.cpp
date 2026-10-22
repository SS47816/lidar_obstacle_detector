/* patchworkpp_segmenter.cpp

 * Copyright (C) 2021-2026 SS47816

 * Adapter around the vendored Patchwork++ (third_party/patchworkpp, BSD-2)

**/

#include <algorithm>
#include <cmath>
#include <vector>

#include "lidar_obstacle_detector/ground_segmentation.hpp"
#include "patchwork/patchworkpp.h"

namespace lidar_obstacle_detector {

struct PatchworkppSegmenterImpl::Impl {
  Impl(const patchwork::Params &params, float ground_tolerance)
      : pw(params), params(params), ground_tolerance(ground_tolerance) {}
  patchwork::PatchWorkpp pw;
  patchwork::Params params;
  float ground_tolerance;
};

PatchworkppSegmenterImpl::PatchworkppSegmenterImpl(const GroundConfig &cfg) {
  patchwork::Params params;
  params.verbose = false;
  params.enable_RNR = false;  // needs calibrated intensities; not portable across sensors
  params.sensor_height = cfg.sensor_height;
  impl_ = std::make_unique<Impl>(params, cfg.distance_threshold);
}

PatchworkppSegmenterImpl::~PatchworkppSegmenterImpl() = default;

void PatchworkppSegmenterImpl::segment(const Eigen::MatrixXf &xyz, pcl::Indices *ground,
                                       pcl::Indices *obstacles) {
  ground->clear();
  obstacles->clear();
  if (xyz.rows() == 0) return;
  impl_->pw.estimateGround(xyz);
  const Eigen::VectorXi g = impl_->pw.getGroundIndices();

  std::vector<bool> is_ground(static_cast<size_t>(xyz.rows()), false);
  for (Eigen::Index i = 0; i < g.size(); ++i) is_ground[static_cast<size_t>(g[i])] = true;

  // Patchwork++ reports every point outside (min_range, max_range] as
  // non-ground; there, fall back to a flat-ground height test so the road right
  // next to the vehicle does not turn into obstacles.
  const auto &p = impl_->params;
  const float ground_z = static_cast<float>(-p.sensor_height) + impl_->ground_tolerance;
  for (Eigen::Index i = 0; i < xyz.rows(); ++i) {
    const double r = std::hypot(xyz(i, 0), xyz(i, 1));
    if ((r <= p.min_range || r > p.max_range) && xyz(i, 2) < ground_z)
      is_ground[static_cast<size_t>(i)] = true;
  }
  for (Eigen::Index i = 0; i < xyz.rows(); ++i)
    (is_ground[static_cast<size_t>(i)] ? ground : obstacles)->push_back(static_cast<int>(i));
}

}  // namespace lidar_obstacle_detector
