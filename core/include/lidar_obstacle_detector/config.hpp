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
enum class DetectorMethod { kClustering, kLearning, kFused };

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
  // BEV grid: split cell columns at vertical gaps larger than max(z_gap, 0.04 * range) [m];
  // 0 disables the split
  float z_gap = 0.f;
  // Clusters whose box exceeds these limits are re-clustered with a tighter
  // tolerance; what is still too large is reported as a structure (wall,
  // vegetation, ...) instead of an object when separate_structures is set
  bool split_oversized = false;
  bool separate_structures = false;
  float object_max_length = 16.f;  // [m]
  float object_max_width = 5.f;    // [m]
  // Use the predicted footprints of confirmed vehicle-sized tracks to re-join
  // fragments of one object and to separate two objects that touch
  bool track_guided = false;
  float track_guided_margin = 0.f;  // [m] footprint growth
  int min_size = 10;
  int max_size = 5000;
};

struct BoxConfig {
  BoxMethod method = BoxMethod::kAabb;
  float lshape_angle_step_deg = 1.f;
};

struct TrackerConfig {
  bool enabled = true;
  TrackCost cost = TrackCost::kGIoU;
  float max_distance = 2.f;     // [m] association gate on predicted centre distance
  float min_giou = -0.5f;       // GIoU gate (cost = kGIoU)
  int min_hits = 2;             // updates before a track is confirmed
  float max_age = 0.5f;         // [s] keep a lost track alive this long
  float accel_noise = 3.f;      // [m/s^2] process noise of the constant-velocity model
  float meas_noise = 0.3f;      // [m] measurement noise of the box centre
  float size_smoothing = 0.3f;  // EMA weight of a new size measurement
  // Moving, stably tracked objects take their yaw from the velocity direction
  bool heading_from_motion = true;
  float heading_min_speed = 2.f;  // [m/s]
  // Partially visible objects keep the size seen earlier in the track,
  // anchored at the face that is visible to the sensor
  bool shape_completion = true;
  // Predicted path of moving tracks (constant turn rate and velocity); 0 = off
  float prediction_horizon = 3.f;  // [s]
  float prediction_step = 0.5f;    // [s]
};

struct OccupancyConfig {
  bool enabled = false;
  float resolution = 0.2f;  // 2D grid cell size [m]
  float min_height = 0.2f;  // obstacle points counted between these heights
  float max_height = 2.5f;  // above the local ground [m]
  float voxel_size = 0.2f;  // 3D occupied voxels [m]
  // 2.5D pillars of unrecognised obstacles are fused over this time window in
  // the tracking frame (needs a tracking frame, e.g. odom); 0 = this scan only
  float pillar_memory = 2.f;  // [s]
};

struct DetectorConfig {
  // clustering: model-free (CPU); every cluster becomes a tracked box.
  // learning: NVIDIA CUDA-CenterPoint (needs a build with LOD_WITH_TENSORRT);
  //   only the classes it was trained on are reported.
  // fused: learned boxes for recognised objects (absorbing the clusters that
  //   belong to them), everything else as 2.5D pillars.
  DetectorMethod method = DetectorMethod::kClustering;
  float score_threshold = 0.3f;
  // Scans accumulated (ego-motion compensated, over at most 0.5 s) as network
  // input; needs a tracking frame, otherwise a single scan is used
  int sweeps = 10;
  // fused: a cluster joins a learned box if this fraction of its points lies
  // inside the box grown by fused_margin
  float fused_margin = 0.3f;  // [m]
  float fused_min_overlap = 0.5f;
  std::string model_dir;     // "" = the models downloaded at build time
  std::string engine_cache;  // "" = ~/.cache/lidar_obstacle_detector
};

struct PipelineConfig {
  PreprocessConfig preprocess;
  GroundConfig ground;
  ClusterConfig cluster;
  BoxConfig box;
  TrackerConfig tracker;
  OccupancyConfig occupancy;
  DetectorConfig detector;

  // Named presets: "legacy" (classic RANSAC + Euclidean + axis-aligned boxes), "fast", "accurate"
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
  std::vector<std::string> choices;  // allowed values (kString); empty = free text
};

// All parameters, in a stable order
const std::vector<ParamInfo> &parameterTable();

// Read / write a parameter by name, values as strings ("true", "0.3", "lshape").
// setParameter() throws std::invalid_argument on unknown names or bad values.
std::string getParameter(const PipelineConfig &config, const std::string &name);
void setParameter(PipelineConfig *config, const std::string &name, const std::string &value);

}  // namespace lidar_obstacle_detector
