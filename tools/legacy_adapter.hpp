/* legacy_adapter.hpp

 * Copyright (C) 2021-2026 SS47816

 * Runs the v1 algorithm stack exactly as the v1 ROS node did, so the benchmark
 * can report an honest baseline.

**/

#pragma once

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "benchmark_types.hpp"
#include "lidar_obstacle_detector/legacy/obstacle_detector.hpp"

namespace lidar_obstacle_detector {
namespace bench {

class LegacyAdapter : public PipelineAdapter {
 public:
  explicit LegacyAdapter(const ParamMap &params) {
    // Defaults are the v1 dynamic_reconfigure defaults (cfg/obstacle_detector.cfg)
    use_pca_box_ = getBool(params, "use_pca_box", false);
    use_tracking_ = getBool(params, "use_tracking", true);
    voxel_grid_size_ = getFloat(params, "voxel_grid_size", 0.2f);
    roi_min_ = Eigen::Vector4f(getFloat(params, "roi_min_x", -30.f),
                               getFloat(params, "roi_min_y", -30.f),
                               getFloat(params, "roi_min_z", -2.5f), 1.f);
    roi_max_ = Eigen::Vector4f(getFloat(params, "roi_max_x", 70.f),
                               getFloat(params, "roi_max_y", 30.f),
                               getFloat(params, "roi_max_z", 1.f), 1.f);
    ground_thresh_ = getFloat(params, "ground_threshold", 0.3f);
    cluster_thresh_ = getFloat(params, "cluster_threshold", 0.6f);
    cluster_min_size_ = getInt(params, "cluster_min_size", 10);
    cluster_max_size_ = getInt(params, "cluster_max_size", 5000);
    displacement_thresh_ = getFloat(params, "displacement_threshold", 1.0f);
    iou_thresh_ = getFloat(params, "iou_threshold", 1.0f);
  }

  FrameOutput process(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input,
                      double /*stamp*/,
                      const Eigen::Isometry3d & /*world_T_sensor*/) override {
    using Clock = std::chrono::steady_clock;
    FrameOutput out;
    const auto t0 = Clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input, *raw);
    const auto t1 = Clock::now();

    auto filtered =
        detector_.filterCloud(raw, voxel_grid_size_, roi_min_, roi_max_);
    const auto t2 = Clock::now();
    auto segmented = detector_.segmentPlane(filtered, 30, ground_thresh_);
    const auto t3 = Clock::now();
    auto clusters = detector_.clustering(segmented.first, cluster_thresh_,
                                         cluster_min_size_, cluster_max_size_);
    const auto t4 = Clock::now();

    std::vector<legacy::Box> curr_boxes;
    for (auto &cluster : clusters) {
      curr_boxes.emplace_back(
          use_pca_box_
              ? detector_.pcaBoundingBox(cluster, obstacle_id_)
              : detector_.axisAlignedBoundingBox(cluster, obstacle_id_));
      ++obstacle_id_;
    }
    const auto t5 = Clock::now();
    if (use_tracking_) {
      detector_.obstacleTracking(prev_boxes_, &curr_boxes, displacement_thresh_,
                                 iou_thresh_);
    }
    const auto t6 = Clock::now();

    for (const auto &b : curr_boxes) {
      OutputBox ob;
      ob.id = static_cast<int64_t>(b.id);
      ob.center = b.position.cast<double>();
      ob.size = b.dimension.cast<double>();
      const Eigen::Matrix3f rot = b.quaternion.normalized().toRotationMatrix();
      ob.yaw = std::atan2(rot(1, 0), rot(0, 0));
      out.boxes.push_back(ob);
    }
    prev_boxes_ = curr_boxes;

    out.ground = toXYZ(*segmented.second);
    out.obstacles = toXYZ(*segmented.first);
    out.timing_ms = {{"convert", ms(t0, t1)}, {"preprocess", ms(t1, t2)},
                     {"ground", ms(t2, t3)},  {"cluster", ms(t3, t4)},
                     {"box", ms(t4, t5)},     {"track", ms(t5, t6)},
                     {"total", ms(t0, t6)}};
    return out;
  }

 private:
  legacy::ObstacleDetector<pcl::PointXYZ> detector_;
  std::vector<legacy::Box> prev_boxes_;
  int obstacle_id_ = 0;

  bool use_pca_box_, use_tracking_;
  float voxel_grid_size_, ground_thresh_, cluster_thresh_;
  float displacement_thresh_, iou_thresh_;
  int cluster_min_size_, cluster_max_size_;
  Eigen::Vector4f roi_min_, roi_max_;
};

}  // namespace bench
}  // namespace lidar_obstacle_detector
