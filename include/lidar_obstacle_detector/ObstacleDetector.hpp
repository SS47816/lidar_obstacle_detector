/* ObstacleDetector.hpp

 * Copyright (C) 2021 SS47816
 
 * Implementation of 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#pragma once
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

namespace lidar_obstacle_detector 
{
template<typename PointT>
class ObstacleDetector
{
 public:
  ObstacleDetector();
  virtual ~ObstacleDetector();
  
 private:
};

} // namespace lidar_obstacle_detector