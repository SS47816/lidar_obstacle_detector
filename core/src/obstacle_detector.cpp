/* obstacle_detector.cpp

 * Copyright (C) 2021-2026 SS47816

 * Explicit instantiations for the common PCL point types

**/

#include "lidar_obstacle_detector/obstacle_detector.hpp"

namespace lidar_obstacle_detector {

template class ObstacleDetector<pcl::PointXYZ>;
template class ObstacleDetector<pcl::PointXYZI>;

}  // namespace lidar_obstacle_detector
