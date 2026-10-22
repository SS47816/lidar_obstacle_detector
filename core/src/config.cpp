/* config.cpp

 * Copyright (C) 2021-2026 SS47816

 * Presets and the flat parameter table

**/

#include "lidar_obstacle_detector/config.hpp"

#include <algorithm>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace lidar_obstacle_detector {

PipelineConfig PipelineConfig::preset(const std::string &name) {
  PipelineConfig c;
  if (name == "legacy") {
    // v1 defaults: unconstrained RANSAC, fixed-radius clustering, axis-aligned boxes
    c.ground.max_iterations = 30;
    c.ground.max_slope_deg = 90.f;
    c.cluster.method = ClusterMethod::kEuclidean;
    c.box.method = BoxMethod::kAabb;
  } else if (name == "fast") {
    c.ground.method = GroundMethod::kPatchworkpp;
    c.ground.refine = true;
    c.cluster.method = ClusterMethod::kBevGrid;
    c.cluster.range_gain = 0.02f;
    c.box.method = BoxMethod::kLShape;
  } else if (name == "accurate") {
    c.ground.method = GroundMethod::kPatchworkpp;
    c.ground.refine = true;
    c.cluster.method = ClusterMethod::kAdaptiveEuclidean;
    c.cluster.range_gain = 0.02f;
    c.box.method = BoxMethod::kLShape;
  } else {
    throw std::invalid_argument("unknown preset '" + name +
                                "' (expected legacy, fast or accurate)");
  }
  return c;
}

namespace {

struct Entry {
  ParamInfo info;
  std::function<std::string(const PipelineConfig &)> get;
  std::function<void(PipelineConfig &, const std::string &)> set;
};

std::string toString(double v) {
  std::ostringstream ss;
  ss << v;
  return ss.str();
}

double parseNumber(const ParamInfo &info, const std::string &s) {
  size_t pos = 0;
  double v = 0.0;
  try {
    v = std::stod(s, &pos);
  } catch (const std::exception &) {
    pos = 0;
  }
  if (pos == 0 || pos != s.size())
    throw std::invalid_argument(info.name + ": not a number: '" + s + "'");
  if (v < info.min || v > info.max)
    throw std::invalid_argument(info.name + ": " + s + " out of range [" +
                                toString(info.min) + ", " + toString(info.max) + "]");
  if (info.type == ParamInfo::Type::kInt && v != static_cast<int>(v))
    throw std::invalid_argument(info.name + ": not an integer: '" + s + "'");
  return v;
}

bool parseBool(const ParamInfo &info, const std::string &s) {
  if (s == "true" || s == "True" || s == "1") return true;
  if (s == "false" || s == "False" || s == "0") return false;
  throw std::invalid_argument(info.name + ": not a bool: '" + s + "'");
}

template <typename T>
Entry number(const std::string &name, ParamInfo::Type type, const std::string &desc,
             double min, double max, std::function<T &(PipelineConfig &)> field) {
  ParamInfo info{name, type, desc, min, max, {}};
  return {info,
          [field](const PipelineConfig &c) {
            return toString(static_cast<double>(field(const_cast<PipelineConfig &>(c))));
          },
          [info, field](PipelineConfig &c, const std::string &s) {
            field(c) = static_cast<T>(parseNumber(info, s));
          }};
}

Entry real(const std::string &name, const std::string &desc, double min, double max,
           std::function<float &(PipelineConfig &)> field) {
  return number<float>(name, ParamInfo::Type::kDouble, desc, min, max, field);
}

Entry integer(const std::string &name, const std::string &desc, int min, int max,
              std::function<int &(PipelineConfig &)> field) {
  return number<int>(name, ParamInfo::Type::kInt, desc, min, max, field);
}

Entry boolean(const std::string &name, const std::string &desc,
              std::function<bool &(PipelineConfig &)> field) {
  ParamInfo info{name, ParamInfo::Type::kBool, desc, 0, 1, {}};
  return {info,
          [field](const PipelineConfig &c) {
            return std::string(field(const_cast<PipelineConfig &>(c)) ? "true" : "false");
          },
          [info, field](PipelineConfig &c, const std::string &s) {
            field(c) = parseBool(info, s);
          }};
}

template <typename E>
Entry choice(const std::string &name, const std::string &desc,
             const std::vector<std::pair<std::string, E>> &options,
             std::function<E &(PipelineConfig &)> field) {
  ParamInfo info{name, ParamInfo::Type::kString, desc, 0, 0, {}};
  for (const auto &o : options) info.choices.push_back(o.first);
  return {info,
          [options, field](const PipelineConfig &c) {
            const E v = field(const_cast<PipelineConfig &>(c));
            for (const auto &o : options)
              if (o.second == v) return o.first;
            return std::string();
          },
          [info, options, field](PipelineConfig &c, const std::string &s) {
            for (const auto &o : options) {
              if (o.first == s) {
                field(c) = o.second;
                return;
              }
            }
            std::string allowed;
            for (const auto &o : options) allowed += (allowed.empty() ? "" : ", ") + o.first;
            throw std::invalid_argument(info.name + ": '" + s + "' is not one of " + allowed);
          }};
}

const std::vector<Entry> &entries() {
  using C = PipelineConfig;
  static const std::vector<Entry> table = {
      // --- preprocessing ---
      real("voxel_grid_size", "Voxel grid leaf size [m], 0 disables downsampling", 0, 1,
           [](C &c) -> float & { return c.preprocess.voxel_size; }),
      real("roi_min_x", "ROI min x [m]", -200, 0, [](C &c) -> float & { return c.preprocess.roi_min.x(); }),
      real("roi_min_y", "ROI min y [m]", -200, 0, [](C &c) -> float & { return c.preprocess.roi_min.y(); }),
      real("roi_min_z", "ROI min z [m]", -10, 0, [](C &c) -> float & { return c.preprocess.roi_min.z(); }),
      real("roi_max_x", "ROI max x [m]", 0, 200, [](C &c) -> float & { return c.preprocess.roi_max.x(); }),
      real("roi_max_y", "ROI max y [m]", 0, 200, [](C &c) -> float & { return c.preprocess.roi_max.y(); }),
      real("roi_max_z", "ROI max z [m]", -5, 10, [](C &c) -> float & { return c.preprocess.roi_max.z(); }),
      boolean("remove_ego", "Remove points inside the ego-vehicle box",
              [](C &c) -> bool & { return c.preprocess.remove_ego; }),
      real("ego_min_x", "Ego box min x [m]", -20, 20, [](C &c) -> float & { return c.preprocess.ego_min.x(); }),
      real("ego_min_y", "Ego box min y [m]", -20, 20, [](C &c) -> float & { return c.preprocess.ego_min.y(); }),
      real("ego_min_z", "Ego box min z [m]", -10, 10, [](C &c) -> float & { return c.preprocess.ego_min.z(); }),
      real("ego_max_x", "Ego box max x [m]", -20, 20, [](C &c) -> float & { return c.preprocess.ego_max.x(); }),
      real("ego_max_y", "Ego box max y [m]", -20, 20, [](C &c) -> float & { return c.preprocess.ego_max.y(); }),
      real("ego_max_z", "Ego box max z [m]", -10, 10, [](C &c) -> float & { return c.preprocess.ego_max.z(); }),
      // --- ground segmentation ---
      choice<GroundMethod>("ground_method", "Ground segmentation: ransac | patchworkpp",
                           {{"ransac", GroundMethod::kRansac}, {"patchworkpp", GroundMethod::kPatchworkpp}},
                           [](C &c) -> GroundMethod & { return c.ground.method; }),
      real("ground_threshold", "RANSAC plane distance threshold [m]", 0, 2,
           [](C &c) -> float & { return c.ground.distance_threshold; }),
      integer("ground_max_iterations", "RANSAC iterations", 1, 10000,
              [](C &c) -> int & { return c.ground.max_iterations; }),
      real("ground_max_slope", "RANSAC: max tilt of the ground plane [deg], 90 = unconstrained", 0, 90,
           [](C &c) -> float & { return c.ground.max_slope_deg; }),
      real("sensor_height", "Patchwork++: lidar height above the ground [m]", 0, 10,
           [](C &c) -> float & { return c.ground.sensor_height; }),
      boolean("ground_refine", "Relabel points close to the local ground height as ground",
              [](C &c) -> bool & { return c.ground.refine; }),
      real("ground_refine_threshold", "Height above the local ground still counted as ground [m]", 0, 1,
           [](C &c) -> float & { return c.ground.refine_threshold; }),
      // --- clustering ---
      choice<ClusterMethod>("cluster_method", "Clustering: euclidean | adaptive | bev_grid",
                            {{"euclidean", ClusterMethod::kEuclidean},
                             {"adaptive", ClusterMethod::kAdaptiveEuclidean},
                             {"bev_grid", ClusterMethod::kBevGrid}},
                            [](C &c) -> ClusterMethod & { return c.cluster.method; }),
      real("cluster_threshold", "Cluster distance tolerance [m] (minimum when adaptive)", 0.05, 5,
           [](C &c) -> float & { return c.cluster.tolerance; }),
      real("cluster_range_gain", "Adaptive tolerance = max(cluster_threshold, gain * range)", 0, 0.2,
           [](C &c) -> float & { return c.cluster.range_gain; }),
      real("cluster_cell_size", "BEV grid cell size [m]", 0.05, 2,
           [](C &c) -> float & { return c.cluster.cell_size; }),
      integer("cluster_min_size", "Min points per cluster", 1, 100000,
              [](C &c) -> int & { return c.cluster.min_size; }),
      integer("cluster_max_size", "Max points per cluster", 1, 10000000,
              [](C &c) -> int & { return c.cluster.max_size; }),
      // --- bounding boxes ---
      choice<BoxMethod>("box_method", "Box fitting: aabb | pca | lshape",
                        {{"aabb", BoxMethod::kAabb}, {"pca", BoxMethod::kPca}, {"lshape", BoxMethod::kLShape}},
                        [](C &c) -> BoxMethod & { return c.box.method; }),
      real("lshape_angle_step", "L-shape search step [deg]", 0.1, 10,
           [](C &c) -> float & { return c.box.lshape_angle_step_deg; }),
      // --- tracking ---
      boolean("use_tracking", "Track obstacles across frames",
              [](C &c) -> bool & { return c.tracker.enabled; }),
      choice<TrackCost>("track_cost", "Association cost: center | giou",
                        {{"center", TrackCost::kCenterDistance}, {"giou", TrackCost::kGIoU}},
                        [](C &c) -> TrackCost & { return c.tracker.cost; }),
      real("track_max_distance", "Association gate on centre distance [m]", 0.1, 20,
           [](C &c) -> float & { return c.tracker.max_distance; }),
      real("track_min_giou", "Association gate on GIoU (track_cost=giou)", -1, 1,
           [](C &c) -> float & { return c.tracker.min_giou; }),
      integer("track_min_hits", "Updates before a track is confirmed", 1, 100,
              [](C &c) -> int & { return c.tracker.min_hits; }),
      real("track_max_age", "Delete tracks not updated for this long [s]", 0, 10,
           [](C &c) -> float & { return c.tracker.max_age; }),
      real("track_accel_noise", "Kalman process noise (acceleration) [m/s^2]", 0.01, 50,
           [](C &c) -> float & { return c.tracker.accel_noise; }),
      real("track_meas_noise", "Kalman measurement noise (centre) [m]", 0.01, 10,
           [](C &c) -> float & { return c.tracker.meas_noise; }),
      real("track_size_smoothing", "EMA weight of new size measurements", 0, 1,
           [](C &c) -> float & { return c.tracker.size_smoothing; }),
  };
  return table;
}

const Entry &find(const std::string &name) {
  const auto &table = entries();
  const auto it = std::find_if(table.begin(), table.end(),
                               [&](const Entry &e) { return e.info.name == name; });
  if (it == table.end()) throw std::invalid_argument("unknown parameter '" + name + "'");
  return *it;
}

}  // namespace

const std::vector<ParamInfo> &parameterTable() {
  static const std::vector<ParamInfo> infos = [] {
    std::vector<ParamInfo> v;
    for (const auto &e : entries()) v.push_back(e.info);
    return v;
  }();
  return infos;
}

std::string getParameter(const PipelineConfig &config, const std::string &name) {
  return find(name).get(config);
}

void setParameter(PipelineConfig *config, const std::string &name, const std::string &value) {
  find(name).set(*config, value);
}

}  // namespace lidar_obstacle_detector
