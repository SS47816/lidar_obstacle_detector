/* tracker.hpp

 * Copyright (C) 2021-2026 SS47816

 * Multi-object tracker: constant-velocity Kalman filter per track, optimal
 * (Hungarian) association on centre distance or BEV GIoU, track lifecycle.

**/

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/types.hpp"

namespace lidar_obstacle_detector {

class MultiObjectTracker {
 public:
  explicit MultiObjectTracker(const TrackerConfig &cfg = TrackerConfig());

  void setConfig(const TrackerConfig &cfg) { cfg_ = cfg; }
  const TrackerConfig &config() const { return cfg_; }

  // Associates the detections (in a fixed tracking frame, e.g. odom) with the
  // existing tracks and updates them. Every detection is kept; on return each
  // box carries its track id, confirmation state, velocity and a yaw / size
  // made consistent with its track.
  void update(double stamp, std::vector<Box> *boxes);

  void reset();
  size_t numTracks() const { return tracks_.size(); }

 private:
  struct Track {
    uint32_t id;
    Eigen::Vector4f x;  // px, py, vx, vy
    Eigen::Matrix4f P;
    Eigen::Vector3f size;
    float yaw;
    float z;
    int hits;
    double last_update;
  };

  void predict(Track *t, float dt) const;
  float cost(const Track &t, const Box &det) const;
  void correct(Track *t, Box *det, double stamp) const;
  Track spawn(const Box &det, double stamp);

  TrackerConfig cfg_;
  std::vector<Track> tracks_;
  uint32_t next_id_ = 0;
  double last_stamp_ = 0.0;
  bool initialized_ = false;
};

}  // namespace lidar_obstacle_detector
