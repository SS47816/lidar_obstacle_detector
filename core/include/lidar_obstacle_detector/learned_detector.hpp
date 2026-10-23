/* learned_detector.hpp

 * Copyright (C) 2021-2026 SS47816

 * Interface of the optional learned (GPU) detectors, e.g. NVIDIA CUDA-CenterPoint

**/

#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "lidar_obstacle_detector/config.hpp"
#include "lidar_obstacle_detector/types.hpp"

namespace lidar_obstacle_detector {

class LearnedDetector {
 public:
  virtual ~LearnedDetector() = default;
  // `points`: x, y, z, intensity in the sensor frame (x forward, y left, z up).
  // Returns labelled, scored boxes in the same frame; heading_known is set.
  virtual std::vector<Box> detect(const std::vector<Eigen::Vector4f> &points) = 0;
  virtual std::string name() const = 0;
};

// True if this build includes a learned detector backend (LOD_WITH_TENSORRT)
bool learnedDetectorsAvailable();

// Creates the detector named by cfg.method (e.g. centerpoint); throws
// std::runtime_error if the backend is not built in or fails to load.
std::unique_ptr<LearnedDetector> makeLearnedDetector(const DetectorConfig &cfg,
                                                     float sensor_height);

}  // namespace lidar_obstacle_detector
