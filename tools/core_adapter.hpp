/* core_adapter.hpp

 * Copyright (C) 2021-2026 SS47816

 * Runs the v2 core pipeline inside the benchmark CLI

**/

#pragma once

#include <string>

#include "benchmark_types.hpp"
#include "lidar_obstacle_detector/obstacle_detector.hpp"

namespace lidar_obstacle_detector {
namespace bench {

class CoreAdapter : public PipelineAdapter {
 public:
  CoreAdapter(const std::string &preset, const ParamMap &params) {
    PipelineConfig cfg = PipelineConfig::preset(preset);
    for (const auto &kv : params) setParameter(&cfg, kv.first, kv.second);
    detector_.setConfig(cfg);
  }

  FrameOutput process(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, double stamp,
                      const Eigen::Isometry3d &world_T_sensor) override {
    // Track in a world-anchored frame. Shift the origin to the first pose so
    // that float precision is not wasted on large map coordinates.
    if (!has_origin_) {
      origin_ = world_T_sensor.translation();
      has_origin_ = true;
    }
    Eigen::Isometry3d local = world_T_sensor;
    local.translation() -= origin_;

    const auto res = detector_.process(cloud, stamp, local.cast<float>());

    FrameOutput out;
    for (const auto &b : res.boxes) {
      OutputBox ob;
      ob.id = b.id;
      ob.center = b.center.cast<double>();
      ob.size = b.size.cast<double>();
      ob.yaw = b.yaw;
      ob.has_velocity = b.has_velocity;
      ob.velocity = b.velocity.cast<double>();  // already world-aligned
      out.boxes.push_back(ob);
    }
    for (const auto &b : res.structure_boxes) {
      OutputBox ob;
      ob.center = b.center.cast<double>();
      ob.size = b.size.cast<double>();
      ob.yaw = b.yaw;
      out.structures.push_back(ob);
    }
    out.occ_resolution = res.occupancy.resolution;
    out.occ_origin_x = res.occupancy.origin.x();
    out.occ_origin_y = res.occupancy.origin.y();
    out.occ_width = res.occupancy.width;
    out.occ_height = res.occupancy.height;
    out.occupancy = res.occupancy.data;
    out.ground = toXYZ(*res.ground);
    out.obstacles = toXYZ(*res.obstacles);
    const auto &t = res.timings;
    out.timing_ms = {{"preprocess", t.preprocess},
                     {"ground", t.ground},
                     {"cluster", t.cluster},
                     {"box", t.box},
                     {"track", t.track},
                     {"occupancy", t.occupancy},
                     {"total", t.total}};
    return out;
  }

 private:
  ObstacleDetector<pcl::PointXYZI> detector_;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  bool has_origin_ = false;
};

}  // namespace bench
}  // namespace lidar_obstacle_detector
