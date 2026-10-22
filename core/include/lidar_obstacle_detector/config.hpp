/* config.hpp

 * Copyright (C) 2021-2026 SS47816

 * Configuration of every pipeline stage, plus a flat parameter table that the
 * ROS 1 / ROS 2 wrappers and the benchmark CLI share (same names everywhere).

**/

#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace lidar_obstacle_detector {

enum class GroundMethod { kRansac, kPatchworkpp };
enum class ClusterMethod { kEuclidean, kAdaptiveEuclidean, kBevGrid };
enum class BoxMethod { kAabb, kPca, kLShape };
enum class TrackCost { kCenterDistance, kGIoU };

struct PreprocessConfig {
  // Region of interest in the sensor frame (x forward, y left, z up)
  Eigen::Vector3f roi_min{-30.f, -30.f, -2.5f};
  Eigen::Vector3f roi_max{70.f, 30.f, 1.f};
  // Points inside this box (the ego vehicle) are removed
  bool remove_ego = true;
  Eigen::Vector3f ego_min{-1.5f, -1.7f, -1.f};
  Eigen::Vector3f ego_max{2.6f, 1.7f, -0.4f};
  // Voxel grid leaf size [m]; <= 0 disables downsampling
  float voxel_size = 0.2f;
};

struct GroundConfig {
  GroundMethod method = GroundMethod::kRansac;
  // RANSAC: plane distance threshold [m], iterations, max plane tilt [deg]
  float distance_threshold = 0.3f;
  int max_iterations = 100;
  float max_slope_deg = 15.f;
  // Patchwork++: mounting height of the lidar above the ground [m]
  float sensor_height = 1.73f;
  // Local elevation-map refinement: non-ground points within this height of
  // the surrounding ground are relabelled as ground
  bool refine = false;
  float refine_threshold = 0.2f;
};

struct ClusterConfig {
  ClusterMethod method = ClusterMethod::kEuclidean;
  float tolerance = 0.6f;   // [m] neighbour distance (min. value when adaptive)
  float range_gain = 0.0f;  // adaptive: tolerance = max(tolerance, gain * range)
  float cell_size = 0.25f;  // BEV grid cell size [m]
  int min_size = 10;
  int max_size = 5000;
};

struct BoxConfig {
  BoxMethod method = BoxMethod::kAabb;
  float lshape_angle_step_deg = 1.f;
};

struct TrackerConfig {
  bool enabled = true;
  TrackCost cost = TrackCost::kCenterDistance;
  float max_distance = 2.f;     // [m] association gate on predicted centre distance
  float min_giou = -0.5f;       // GIoU gate (cost = kGIoU)
  int min_hits = 2;             // updates before a track is confirmed
  float max_age = 0.5f;         // [s] keep a lost track alive this long
  float accel_noise = 3.f;      // [m/s^2] process noise of the constant-velocity model
  float meas_noise = 0.3f;      // [m] measurement noise of the box centre
  float size_smoothing = 0.3f;  // EMA weight of a new size measurement
};

struct PipelineConfig {
  PreprocessConfig preprocess;
  GroundConfig ground;
  ClusterConfig cluster;
  BoxConfig box;
  TrackerConfig tracker;

  // Named presets: "legacy" (v1 behaviour minus its bugs), "fast", "accurate"
  static PipelineConfig preset(const std::string &name);
};

// ------------------------------------------------------------------------------
// Flat parameter table
// ------------------------------------------------------------------------------
struct ParamInfo {
  enum class Type { kBool, kInt, kDouble, kString };
  std::string name;
  Type type;
  std::string description;
  double min = 0.0, max = 0.0;       // numeric range (kInt / kDouble)
  std::vector<std::string> choices;  // allowed values (kString)
};

// All parameters, in a stable order
const std::vector<ParamInfo> &parameterTable();

// Read / write a parameter by name, values as strings ("true", "0.3", "lshape").
// setParameter() throws std::invalid_argument on unknown names or bad values.
std::string getParameter(const PipelineConfig &config, const std::string &name);
void setParameter(PipelineConfig *config, const std::string &name, const std::string &value);

}  // namespace lidar_obstacle_detector
