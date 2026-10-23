/* obstacle_detector.hpp

 * Copyright (C) 2021-2026 SS47816

 * ROS-agnostic 3D LiDAR obstacle detection & tracking pipeline:
 *   preprocess (ROI, ego removal, voxel) -> ground segmentation ->
 *   clustering and / or a learned detector -> bounding boxes ->
 *   multi-object tracking and prediction -> 2.5D pillars and occupancy

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
    CloudPtr ground;     // ground points (preprocessed)
    CloudPtr obstacles;  // non-ground points (preprocessed)
    // One entry per box: indices into `obstacles` of the points that belong to
    // the object (learned boxes: the clusters they absorbed)
    std::vector<pcl::Indices> clusters;
    std::vector<Box> boxes;  // recognised / clustered objects, in the sensor frame
    // Oversized clusters (walls, vegetation, ...) when `separate_structures` is
    // set: indices into `obstacles` and a yaw-only PCA box each, not tracked
    std::vector<pcl::Indices> structures;
    std::vector<Box> structure_boxes;
    // When `occupancy.enabled`: 2.5D pillars of everything that is not a
    // learned object (all obstacles in clustering mode, none in learning
    // mode), 2D grid and occupied voxel centres (sensor frame)
    std::vector<Pillar> pillars;
    float pillar_resolution = 0.f;
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
               (!learned_ || od.method == DetectorMethod::kClustering ||
                d.score_threshold != od.score_threshold || d.model_dir != od.model_dir ||
                d.engine_cache != od.engine_cache || g.sensor_height != o.sensor_height)) {
      learned_ = makeLearnedDetector(d, g.sensor_height);  // throws if unavailable
    }
    config_ = config;
    tracker_.setConfig(config.tracker);
  }
  const PipelineConfig &config() const { return config_; }

  void reset() {
    tracker_.reset();
    pillar_mapper_.reset();
    sweeps_.clear();
  }

  // Plugs in your own learned detector (any network that returns labelled boxes
  // in the sensor frame); used when detector.method is learning or fused.
  void setLearnedDetector(std::unique_ptr<LearnedDetector> detector) {
    learned_ = std::move(detector);
    custom_learned_ = learned_ != nullptr;
  }

  // Runs the whole pipeline on one scan. `tracking_T_sensor` places the sensor
  // in a fixed frame (e.g. odom) for tracking, multi-scan network input and
  // pillar fusion; with the identity, objects are tracked relative to the
  // sensor (velocities include ego motion) and nothing is accumulated.
  Result process(const CloudConstPtr &cloud, double stamp,
                 const Eigen::Isometry3f &tracking_T_sensor = Eigen::Isometry3f::Identity()) {
    using Clock = std::chrono::steady_clock;
    const auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    Result res;
    const bool have_odom = !tracking_T_sensor.matrix().isIdentity(1e-6f);
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
    if (learned_ && method != DetectorMethod::kClustering)
      learned_boxes = suppressDuplicates(
          learned_->detect(networkInput(*cloud, stamp, tracking_T_sensor, have_odom)));
    const auto t_learned = Clock::now();

    const auto &cc = config_.cluster;
    if (method == DetectorMethod::kLearning) {
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
    if (method == DetectorMethod::kClustering && cc.track_guided && config_.tracker.enabled)
      guideByTracks(stamp, tracking_T_sensor, &res);
    const auto t3 = Clock::now();

    // Points that belong to a learned object are not pillars
    std::vector<uint8_t> pillar_mask(res.obstacles->size(), 1);
    if (method == DetectorMethod::kClustering) {
      res.boxes.resize(res.clusters.size());
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) if (res.clusters.size() > 8)
#endif
      for (int i = 0; i < static_cast<int>(res.clusters.size()); ++i) {
        res.boxes[i] = fitBox(*res.obstacles, res.clusters[i], config_.box);
        res.boxes[i].id = static_cast<uint32_t>(i);
      }
    } else {
      absorbClusters(learned_boxes, &res, &pillar_mask);
      if (method == DetectorMethod::kLearning) pillar_mask.assign(pillar_mask.size(), 0);
    }
    res.structure_boxes.reserve(res.structures.size());
    for (const auto &st : res.structures)
      res.structure_boxes.push_back(fitPcaBox(*res.obstacles, st));
    const auto t4 = Clock::now();

    if (config_.tracker.enabled) track(stamp, tracking_T_sensor, &res.boxes);
    const auto t5 = Clock::now();

    res.occupied_voxels.reset(new Cloud);
    if (config_.occupancy.enabled) {
      std::vector<Eigen::Vector4f> samples;
      computeOccupancy(*res.ground, *res.obstacles, config_.preprocess, config_.occupancy,
                       &occupancy_mapper_, &res.occupancy, res.occupied_voxels.get(), &pillar_mask,
                       &samples);
      const float rzz = tracking_T_sensor.linear()(2, 2);
      for (auto &s : samples) {  // to the tracking frame
        const Eigen::Vector3f p = tracking_T_sensor * Eigen::Vector3f(s.x(), s.y(), s.w());
        s = Eigen::Vector4f(p.x(), p.y(), p.z() + rzz * (s.z() - s.w()), p.z());
      }
      const std::vector<Box> objects =
          method == DetectorMethod::kClustering ? std::vector<Box>() : res.boxes;
      pillar_mapper_.update(samples, res.occupancy, objects, tracking_T_sensor, stamp,
                            config_.occupancy.resolution,
                            have_odom ? config_.occupancy.pillar_memory : 0.f, &res.pillars);
      res.pillar_resolution = config_.occupancy.resolution;
    }
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
  struct Sweep {
    double stamp;
    Eigen::Isometry3f tracking_T_sensor;
    std::vector<Eigen::Vector4f> points;  // x, y, z, intensity
  };

  template <typename P>
  static auto intensityOf(const P &p) -> decltype(p.intensity) {
    return p.intensity;
  }
  template <typename P, typename... Ignored>
  static float intensityOf(const P &, Ignored...) {
    return 0.f;
  }

  static bool inside(const Box &b, const Eigen::Vector2f &p, float margin) {
    const Eigen::Vector2f d = p - b.center.head<2>();
    const float c = std::cos(b.yaw), s = std::sin(b.yaw);
    return std::abs(c * d.x() + s * d.y()) <= 0.5f * b.size.x() + margin &&
           std::abs(-s * d.x() + c * d.y()) <= 0.5f * b.size.y() + margin;
  }

  // The cropped scan plus up to `sweeps - 1` earlier scans of the last 0.5 s,
  // moved into the current sensor frame
  std::vector<SweepPoint> networkInput(const Cloud &cloud, double stamp,
                                       const Eigen::Isometry3f &tracking_T_sensor, bool have_odom) {
    constexpr double kWindow = 0.55;  // [s] (nuScenes training: 10 sweeps at 20 Hz)
    PreprocessConfig crop = config_.preprocess;
    crop.voxel_size = 0.f;
    const CloudPtr cropped = preprocessCloud(cloud, crop);
    Sweep current{stamp, tracking_T_sensor, {}};
    current.points.reserve(cropped->size());
    for (const auto &p : cropped->points)
      current.points.emplace_back(p.x, p.y, p.z, intensityOf(p));

    std::vector<SweepPoint> pts;
    pts.reserve(current.points.size() * static_cast<size_t>(std::min(config_.detector.sweeps, 4)));
    for (const auto &p : current.points) pts.push_back({p.x(), p.y(), p.z(), p.w(), 0.f});
    const int max_old = have_odom ? config_.detector.sweeps - 1 : 0;
    while (!sweeps_.empty() &&
           (static_cast<int>(sweeps_.size()) > max_old || stamp - sweeps_.front().stamp > kWindow ||
            stamp < sweeps_.back().stamp))
      sweeps_.erase(sweeps_.begin());
    const Eigen::Isometry3f sensor_T_tracking = tracking_T_sensor.inverse();
    for (auto it = sweeps_.rbegin(); it != sweeps_.rend(); ++it) {
      const Eigen::Isometry3f now_T_then = sensor_T_tracking * it->tracking_T_sensor;
      const float lag = static_cast<float>(stamp - it->stamp);
      for (const auto &p : it->points) {
        const Eigen::Vector3f q = now_T_then * Eigen::Vector3f(p.x(), p.y(), p.z());
        pts.push_back({q.x(), q.y(), q.z(), p.w(), lag});
      }
    }
    if (max_old > 0) {
      sweeps_.push_back(std::move(current));
      if (static_cast<int>(sweeps_.size()) > max_old) sweeps_.erase(sweeps_.begin());
    }
    return pts;
  }

  // Class-agnostic NMS: networks with per-class heads (CenterPoint) may report
  // the same object twice, e.g. as car and as truck
  static std::vector<Box> suppressDuplicates(std::vector<Box> learned) {
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
    return kept;
  }

  // Learned boxes become the objects. Each takes the obstacle points inside it
  // (grown by fused_margin) and every cluster that lies mostly inside it, so a
  // truck the network boxed too short does not leave fragments behind. Points
  // not taken stay in `pillar_mask`.
  void absorbClusters(const std::vector<Box> &learned, Result *res,
                      std::vector<uint8_t> *pillar_mask) const {
    const float margin = config_.detector.fused_margin;
    const auto &pts = res->obstacles->points;
    std::vector<int> owner(pts.size(), -1);
    std::vector<float> reach(learned.size());
    for (size_t k = 0; k < learned.size(); ++k)
      reach[k] = 0.5f * learned[k].size.head<2>().norm() + margin;
    for (size_t i = 0; i < pts.size(); ++i) {
      const Eigen::Vector2f p(pts[i].x, pts[i].y);
      for (size_t k = 0; k < learned.size(); ++k) {
        if ((p - learned[k].center.head<2>()).squaredNorm() > reach[k] * reach[k]) continue;
        if (inside(learned[k], p, margin)) {
          owner[i] = static_cast<int>(k);
          break;
        }
      }
    }
    for (const auto &cluster : res->clusters) {
      std::vector<int> votes(learned.size(), 0);
      for (int i : cluster)
        if (owner[i] >= 0) ++votes[owner[i]];
      const auto best = std::max_element(votes.begin(), votes.end());
      if (best == votes.end() ||
          *best < config_.detector.fused_min_overlap * static_cast<float>(cluster.size()))
        continue;
      const int k = static_cast<int>(best - votes.begin());
      for (int i : cluster) owner[i] = k;
    }
    std::vector<pcl::Indices> members(learned.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      if (owner[i] < 0) continue;
      members[owner[i]].push_back(static_cast<int>(i));
      (*pillar_mask)[i] = 0;
    }
    res->boxes = learned;
    for (size_t k = 0; k < res->boxes.size(); ++k) res->boxes[k].id = static_cast<uint32_t>(k);
    res->clusters.swap(members);
  }

  // Clustering mode: the predicted footprints of confirmed vehicle-sized tracks
  // re-join fragments of one object (gaps in the scan of a long truck) and
  // separate a cluster that covers two tracked objects (cars that touch).
  void guideByTracks(double stamp, const Eigen::Isometry3f &tracking_T_sensor, Result *res) const {
    constexpr float kMinLength = 3.f;      // [m] only vehicle-sized tracks
    constexpr float kMostlyInside = 0.8f;  // fragment re-join threshold
    constexpr float kShare = 0.2f;         // each track's share of a cluster to split it
    std::vector<Box> fp = tracker_.predictedFootprints(stamp, kMinLength);
    if (fp.empty()) return;
    const Eigen::Isometry3f sensor_T_tracking = tracking_T_sensor.inverse();
    const Eigen::Matrix3f rot = sensor_T_tracking.rotation();
    const float dyaw = std::atan2(rot(1, 0), rot(0, 0));
    for (auto &b : fp) {
      b.center = sensor_T_tracking * b.center;
      b.yaw = wrapAngle(b.yaw + dyaw);
    }
    const float margin = config_.cluster.track_guided_margin;
    const auto &pts = res->obstacles->points;
    auto footprintOf = [&](int i) {
      const Eigen::Vector2f p(pts[i].x, pts[i].y);
      for (size_t k = 0; k < fp.size(); ++k)
        if (inside(fp[k], p, margin)) return static_cast<int>(k);
      return -1;
    };

    std::vector<pcl::Indices> out;
    std::vector<pcl::Indices> joined(fp.size());
    for (auto &cluster : res->clusters) {
      std::vector<int> where(cluster.size());
      std::vector<int> votes(fp.size(), 0);
      for (size_t j = 0; j < cluster.size(); ++j) {
        where[j] = footprintOf(cluster[j]);
        if (where[j] >= 0) ++votes[where[j]];
      }
      const float n = static_cast<float>(cluster.size());
      std::vector<int> sharing;
      for (size_t k = 0; k < fp.size(); ++k)
        if (votes[k] >= std::max(kShare * n, static_cast<float>(config_.cluster.min_size)))
          sharing.push_back(static_cast<int>(k));
      if (sharing.size() >= 2) {
        // Split: points go to the footprint they are in, else the nearest one
        std::vector<pcl::Indices> parts(fp.size());
        for (size_t j = 0; j < cluster.size(); ++j) {
          int k = where[j];
          if (k < 0 || votes[k] == 0) {
            float best = std::numeric_limits<float>::max();
            for (int s : sharing) {
              const float d =
                  (Eigen::Vector2f(pts[cluster[j]].x, pts[cluster[j]].y) - fp[s].center.head<2>())
                      .squaredNorm();
              if (d < best) best = d, k = s;
            }
          }
          parts[k].push_back(cluster[j]);
        }
        for (int s : sharing) joined[s].insert(joined[s].end(), parts[s].begin(), parts[s].end());
        for (size_t k = 0; k < fp.size(); ++k)  // stray points in other footprints
          if (!parts[k].empty() && std::find(sharing.begin(), sharing.end(), k) == sharing.end())
            joined[k].insert(joined[k].end(), parts[k].begin(), parts[k].end());
        continue;
      }
      const auto best = std::max_element(votes.begin(), votes.end());
      if (*best >= kMostlyInside * n) {
        auto &j = joined[best - votes.begin()];
        j.insert(j.end(), cluster.begin(), cluster.end());
      } else {
        out.push_back(std::move(cluster));
      }
    }
    for (auto &j : joined)
      if (static_cast<int>(j.size()) >= config_.cluster.min_size) out.push_back(std::move(j));
    res->clusters.swap(out);
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
      for (auto &p : b.predicted_path) {
        p.position = sensor_T_tracking * p.position;
        p.yaw = wrapAngle(p.yaw - dyaw);
      }
    }
  }

  PipelineConfig config_;
  std::unique_ptr<GroundSegmenter<PointT>> ground_segmenter_;
  MultiObjectTracker tracker_;
  OccupancyMapper occupancy_mapper_;
  PillarMapper pillar_mapper_;
  std::unique_ptr<LearnedDetector> learned_;
  bool custom_learned_ = false;
  std::vector<Sweep> sweeps_;  // earlier network inputs, oldest first
};

extern template class ObstacleDetector<pcl::PointXYZ>;
extern template class ObstacleDetector<pcl::PointXYZI>;

}  // namespace lidar_obstacle_detector
