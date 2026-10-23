/* learned_detector.cpp

 * Copyright (C) 2021-2026 SS47816

 * Factory for the optional learned detectors

**/

#include "lidar_obstacle_detector/learned_detector.hpp"

#include <stdexcept>

namespace lidar_obstacle_detector {

#ifdef LOD_WITH_TENSORRT
// core/gpu/centerpoint_detector.cpp
std::unique_ptr<LearnedDetector> makeCenterPointDetector(const DetectorConfig &cfg,
                                                         float sensor_height);
#endif

bool learnedDetectorsAvailable() {
#ifdef LOD_WITH_TENSORRT
  return true;
#else
  return false;
#endif
}

std::unique_ptr<LearnedDetector> makeLearnedDetector(const DetectorConfig &cfg,
                                                     float sensor_height) {
  if (cfg.method == DetectorMethod::kClustering) return nullptr;
#ifdef LOD_WITH_TENSORRT
  return makeCenterPointDetector(cfg, sensor_height);
#else
  (void)sensor_height;
  throw std::runtime_error(
      "detector_method 'learning' / 'fused' needs a build with -DLOD_WITH_TENSORRT=ON "
      "(CUDA + TensorRT)");
#endif
}

}  // namespace lidar_obstacle_detector
