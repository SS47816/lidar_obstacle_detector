/* obstacle_detector.hpp

 * Copyright (C) 2021-2026 SS47816

 * ROS-agnostic 3D LiDAR obstacle detection & tracking pipeline:
 *   preprocess (ROI, ego removal, voxel) -> ground segmentation ->
 *   clustering -> bounding boxes -> multi-object tracking

**/

#pragma once

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <vector>

#include "lidar_obstacle_detector/box_fitting.hpp"
#include "lidar_obstacle_detector/clustering.hpp"
#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/ground_segmentation.hpp"
#include "lidar_obstacle_detector/preprocess.hpp"
#include "lidar_obstacle_detector/tracker.hpp"
#include "lidar_obstacle_detector/types.hpp"

namespace lidar_obstacle_detector {

template <typename PointT>
class ObstacleDetector {
 public:
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;

  struct Result {
    CloudPtr ground;                     // ground points (preprocessed)
    CloudPtr obstacles;                  // non-ground points (preprocessed)
    std::vector<pcl::Indices> clusters;  // indices into `obstacles`, one per box
    std::vector<Box> boxes;              // in the sensor frame
    StageTimings timings;
  };

  explicit ObstacleDetector(const PipelineConfig &config = PipelineConfig())
      : tracker_(config.tracker) {
    setConfig(config);
  }

  // Can be called between frames (e.g. from a parameter callback). Ground
  // segmenter state is rebuilt only if its settings changed; tracks survive.
  void setConfig(const PipelineConfig &config) {
    const auto &g = config.ground;
    const auto &o = config_.ground;
    if (!ground_segmenter_ || g.method != o.method || g.distance_threshold != o.distance_threshold ||
        g.max_iterations != o.max_iterations || g.max_slope_deg != o.max_slope_deg ||
        g.sensor_height != o.sensor_height)
      ground_segmenter_ = makeGroundSegmenter<PointT>(g);
    config_ = config;
    tracker_.setConfig(config.tracker);
  }
  const PipelineConfig &config() const { return config_; }

  void reset() { tracker_.reset(); }

  // Runs the whole pipeline on one scan. `tracking_T_sensor` places the sensor
  // in a fixed frame (e.g. odom) for tracking; with the identity, objects are
  // tracked relative to the sensor and velocities include ego motion.
  Result process(const CloudConstPtr &cloud, double stamp,
                 const Eigen::Isometry3f &tracking_T_sensor = Eigen::Isometry3f::Identity()) {
    using Clock = std::chrono::steady_clock;
    const auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    Result res;
    const auto t0 = Clock::now();

    const CloudPtr filtered = preprocessCloud(*cloud, config_.preprocess);
    const auto t1 = Clock::now();

    pcl::Indices ground_idx, obstacle_idx;
    ground_segmenter_->segment(filtered, &ground_idx, &obstacle_idx);
    if (config_.ground.refine)
      refineGround(*filtered, config_.ground.refine_threshold, &ground_idx, &obstacle_idx);
    res.ground.reset(new Cloud);
    res.obstacles.reset(new Cloud);
    pcl::copyPointCloud(*filtered, ground_idx, *res.ground);
    pcl::copyPointCloud(*filtered, obstacle_idx, *res.obstacles);
    const auto t2 = Clock::now();

    res.clusters = clusterCloud<PointT>(res.obstacles, config_.cluster);
    const auto t3 = Clock::now();

    res.boxes.resize(res.clusters.size());
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) if (res.clusters.size() > 8)
#endif
    for (int i = 0; i < static_cast<int>(res.clusters.size()); ++i) {
      res.boxes[i] = fitBox(*res.obstacles, res.clusters[i], config_.box);
      res.boxes[i].id = static_cast<uint32_t>(i);
    }
    const auto t4 = Clock::now();

    if (config_.tracker.enabled) track(stamp, tracking_T_sensor, &res.boxes);
    const auto t5 = Clock::now();

    res.timings.preprocess = ms(t0, t1);
    res.timings.ground = ms(t1, t2);
    res.timings.cluster = ms(t2, t3);
    res.timings.box = ms(t3, t4);
    res.timings.track = ms(t4, t5);
    res.timings.total = ms(t0, t5);
    return res;
  }

 private:
  // Tracks in the fixed frame, then maps the refined boxes back to the sensor
  // frame (velocities stay in the tracking frame).
  void track(double stamp, const Eigen::Isometry3f &tracking_T_sensor, std::vector<Box> *boxes) {
    const Eigen::Matrix3f rot = tracking_T_sensor.rotation();
    const float dyaw = std::atan2(rot(1, 0), rot(0, 0));
    for (auto &b : *boxes) {
      b.center = tracking_T_sensor * b.center;
      b.yaw = wrapAngle(b.yaw + dyaw);
    }
    tracker_.update(stamp, boxes);
    const Eigen::Isometry3f sensor_T_tracking = tracking_T_sensor.inverse();
    for (auto &b : *boxes) {
      b.center = sensor_T_tracking * b.center;
      b.yaw = wrapAngle(b.yaw - dyaw);
    }
  }

  PipelineConfig config_;
  std::unique_ptr<GroundSegmenter<PointT>> ground_segmenter_;
  MultiObjectTracker tracker_;
};

extern template class ObstacleDetector<pcl::PointXYZ>;
extern template class ObstacleDetector<pcl::PointXYZI>;

}  // namespace lidar_obstacle_detector
