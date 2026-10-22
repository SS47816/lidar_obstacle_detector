/* types.hpp

 * Copyright (C) 2021-2026 SS47816

 * Basic types shared by all stages of the obstacle detection pipeline

**/

#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <vector>

namespace lidar_obstacle_detector {

// Object classes of the learned detectors; the clustering detector only
// produces kUnknown (any obstacle)
enum class ObjectClass : uint8_t {
  kUnknown = 0,
  kCar,
  kTruck,
  kBus,
  kTrailer,
  kConstructionVehicle,
  kPedestrian,
  kMotorcycle,
  kBicycle,
  kBarrier,
  kTrafficCone,
};
const char *objectClassName(ObjectClass c);

// An upright 3D bounding box (rotation about z only), as produced by the box
// fitting stage and refined by the tracker.
struct Box {
  uint32_t id = 0;  // detection index, or the track id once tracked
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f size = Eigen::Vector3f::Zero();  // length (along yaw), width, height
  float yaw = 0.f;                                 // [rad] heading of the length axis

  // Filled in by the tracker (velocity is expressed in the tracking frame)
  bool tracked = false;       // belongs to a confirmed track
  bool has_velocity = false;  // velocity estimate is available
  Eigen::Vector2f velocity = Eigen::Vector2f::Zero();
  uint32_t age = 0;  // number of frames the track has been updated
  // Yaw is the direction of travel (front/back resolved from the motion or by a
  // learned detector); when false, yaw is only known modulo 180 deg
  bool heading_known = false;
  ObjectClass label = ObjectClass::kUnknown;
  float score = 1.f;  // detection confidence (1 for clustering)

  Box() = default;
  Box(uint32_t id, const Eigen::Vector3f &center, const Eigen::Vector3f &size, float yaw = 0.f)
      : id(id), center(center), size(size), yaw(yaw) {}

  Eigen::Quaternionf orientation() const {
    return Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  }
};

// Wall-clock time [ms] spent in each pipeline stage for the last frame
struct StageTimings {
  double preprocess = 0.0;
  double ground = 0.0;
  double learned = 0.0;  // learned detector (GPU), if enabled
  double cluster = 0.0;
  double box = 0.0;
  double track = 0.0;
  double occupancy = 0.0;
  double total = 0.0;
};

// Wrap an angle to (-pi, pi]
float wrapAngle(float a);

// BEV (x-y) footprint corners of a box, counter-clockwise
std::array<Eigen::Vector2f, 4> bevCorners(const Box &box);

// BEV intersection-over-union of two rotated boxes, in [0, 1]
float bevIoU(const Box &a, const Box &b);

// BEV generalised IoU (Rezatofighi et al. 2019), in [-1, 1]
float bevGIoU(const Box &a, const Box &b);

}  // namespace lidar_obstacle_detector
