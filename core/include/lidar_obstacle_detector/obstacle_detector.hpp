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
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

#include "lidar_obstacle_detector/box_fitting.hpp"
#include "lidar_obstacle_detector/clustering.hpp"
#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/ground_segmentation.hpp"
#include "lidar_obstacle_detector/learned_detector.hpp"
#include "lidar_obstacle_detector/occupancy.hpp"
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
    // Oversized clusters (walls, vegetation, ...) when `separate_structures` is
    // set: indices into `obstacles` and a yaw-only PCA box each, not tracked
    std::vector<pcl::Indices> structures;
    std::vector<Box> structure_boxes;
    // When `occupancy.enabled`: 2D grid and occupied voxel centres (sensor frame)
    OccupancyGrid2D occupancy;
    CloudPtr occupied_voxels;
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
    if (!ground_segmenter_ || g.method != o.method ||
        g.distance_threshold != o.distance_threshold || g.max_iterations != o.max_iterations ||
        g.max_slope_deg != o.max_slope_deg || g.sensor_height != o.sensor_height)
      ground_segmenter_ = makeGroundSegmenter<PointT>(g);
    const auto &d = config.detector;
    const auto &od = config_.detector;
    const bool learned = d.method != DetectorMethod::kClustering;
    if (!learned) {
      if (!custom_learned_) learned_.reset();
    } else if (!custom_learned_ &&
               (!learned_ || d.method != od.method || d.score_threshold != od.score_threshold ||
                d.model_dir != od.model_dir || d.engine_cache != od.engine_cache ||
                g.sensor_height != o.sensor_height)) {
      learned_ = makeLearnedDetector(d, g.sensor_height);  // throws if unavailable
    }
    config_ = config;
    tracker_.setConfig(config.tracker);
  }
  const PipelineConfig &config() const { return config_; }

  void reset() { tracker_.reset(); }

  // Plugs in your own learned detector (any network that returns labelled boxes
  // in the sensor frame); used when detector.method is centerpoint or hybrid.
  void setLearnedDetector(std::unique_ptr<LearnedDetector> detector) {
    learned_ = std::move(detector);
    custom_learned_ = learned_ != nullptr;
  }

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

    // Learned detector (on the cropped, not downsampled, scan)
    const auto method = config_.detector.method;
    std::vector<Box> learned_boxes;
    if (learned_ && method != DetectorMethod::kClustering) {
      PreprocessConfig crop = config_.preprocess;
      crop.voxel_size = 0.f;
      const CloudPtr cropped = preprocessCloud(*cloud, crop);
      std::vector<Eigen::Vector4f> pts;
      pts.reserve(cropped->size());
      for (const auto &p : cropped->points) pts.emplace_back(p.x, p.y, p.z, intensityOf(p));
      learned_boxes = learned_->detect(pts);
    }
    const auto t_learned = Clock::now();

    const auto &cc = config_.cluster;
    if (method == DetectorMethod::kCenterPoint) {
      // no clustering: objects come from the learned detector only
    } else if (cc.split_oversized || cc.separate_structures) {
      ClusterConfig run_cfg = cc;
      run_cfg.max_size = std::numeric_limits<int>::max();  // handled by the split
      res.clusters = clusterCloud<PointT>(res.obstacles, run_cfg);
      splitOversizedClusters<PointT>(res.obstacles, cc, &res.clusters, &res.structures);
      if (!cc.separate_structures) {
        res.clusters.insert(res.clusters.end(), res.structures.begin(), res.structures.end());
        res.structures.clear();
      }
    } else {
      res.clusters = clusterCloud<PointT>(res.obstacles, cc);
    }
    const auto t3 = Clock::now();

    res.boxes.resize(res.clusters.size());
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) if (res.clusters.size() > 8)
#endif
    for (int i = 0; i < static_cast<int>(res.clusters.size()); ++i) {
      res.boxes[i] = fitBox(*res.obstacles, res.clusters[i], config_.box);
      res.boxes[i].id = static_cast<uint32_t>(i);
    }
    res.structure_boxes.reserve(res.structures.size());
    for (const auto &st : res.structures)
      res.structure_boxes.push_back(fitPcaBox(*res.obstacles, st));
    if (method != DetectorMethod::kClustering) mergeLearned(learned_boxes, &res);
    const auto t4 = Clock::now();

    if (config_.tracker.enabled) track(stamp, tracking_T_sensor, &res.boxes);
    const auto t5 = Clock::now();

    res.occupied_voxels.reset(new Cloud);
    if (config_.occupancy.enabled)
      computeOccupancy(*res.ground, *res.obstacles, config_.preprocess, config_.occupancy,
                       &occupancy_mapper_, &res.occupancy, res.occupied_voxels.get());
    const auto t6 = Clock::now();

    res.timings.preprocess = ms(t0, t1);
    res.timings.ground = ms(t1, t2);
    res.timings.learned = ms(t2, t_learned);
    res.timings.cluster = ms(t_learned, t3);
    res.timings.box = ms(t3, t4);
    res.timings.track = ms(t4, t5);
    res.timings.occupancy = ms(t5, t6);
    res.timings.total = ms(t0, t6);
    return res;
  }

 private:
  template <typename P>
  static auto intensityOf(const P &p) -> decltype(p.intensity) {
    return p.intensity;
  }
  template <typename P, typename... Ignored>
  static float intensityOf(const P &, Ignored...) {
    return 0.f;
  }

  // Learned boxes are the objects; in hybrid mode, clusters that no learned box
  // covers are kept as unknown obstacles (things the network was not trained on).
  static void mergeLearned(const std::vector<Box> &raw, Result *res) {
    // Class-agnostic NMS: networks with per-class heads (CenterPoint) may report
    // the same object twice, e.g. as car and as truck
    std::vector<Box> learned = raw;
    std::sort(learned.begin(), learned.end(),
              [](const Box &a, const Box &b) { return a.score > b.score; });
    std::vector<Box> kept;
    for (const auto &b : learned) {
      bool duplicate = false;
      for (const auto &k : kept) {
        if ((b.center - k.center).head<2>().norm() <
                0.5f * (b.size.head<2>().norm() + k.size.head<2>().norm()) &&
            bevIoU(b, k) > 0.3f) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) kept.push_back(b);
    }
    learned.swap(kept);
    std::vector<Box> boxes = learned;
    std::vector<pcl::Indices> clusters(learned.size());  // no point indices for learned boxes
    for (size_t i = 0; i < res->boxes.size(); ++i) {
      const Box &c = res->boxes[i];
      bool covered = false;
      for (const auto &l : learned) {
        Box grown = l;
        grown.size.head<2>() += Eigen::Vector2f::Constant(1.f);  // 0.5 m margin
        const Eigen::Vector2f d = c.center.head<2>() - l.center.head<2>();
        const float cs = std::cos(l.yaw), sn = std::sin(l.yaw);
        const bool center_inside = std::abs(cs * d.x() + sn * d.y()) <= 0.5f * grown.size.x() &&
                                   std::abs(-sn * d.x() + cs * d.y()) <= 0.5f * grown.size.y();
        if (center_inside || bevIoU(c, l) > 0.1f) {
          covered = true;
          break;
        }
      }
      if (!covered) {
        boxes.push_back(c);
        clusters.push_back(res->clusters[i]);
      }
    }
    for (size_t i = 0; i < boxes.size(); ++i) boxes[i].id = static_cast<uint32_t>(i);
    res->boxes.swap(boxes);
    res->clusters.swap(clusters);
  }

  // Tracks in the fixed frame, then maps the refined boxes back to the sensor
  // frame (velocities stay in the tracking frame).
  void track(double stamp, const Eigen::Isometry3f &tracking_T_sensor, std::vector<Box> *boxes) {
    const Eigen::Matrix3f rot = tracking_T_sensor.rotation();
    const float dyaw = std::atan2(rot(1, 0), rot(0, 0));
    for (auto &b : *boxes) {
      b.center = tracking_T_sensor * b.center;
      b.yaw = wrapAngle(b.yaw + dyaw);
    }
    tracker_.update(stamp, boxes, tracking_T_sensor.translation().head<2>());
    const Eigen::Isometry3f sensor_T_tracking = tracking_T_sensor.inverse();
    for (auto &b : *boxes) {
      b.center = sensor_T_tracking * b.center;
      b.yaw = wrapAngle(b.yaw - dyaw);
    }
  }

  PipelineConfig config_;
  std::unique_ptr<GroundSegmenter<PointT>> ground_segmenter_;
  MultiObjectTracker tracker_;
  OccupancyMapper occupancy_mapper_;
  std::unique_ptr<LearnedDetector> learned_;
  bool custom_learned_ = false;
};

extern template class ObstacleDetector<pcl::PointXYZ>;
extern template class ObstacleDetector<pcl::PointXYZI>;

}  // namespace lidar_obstacle_detector
