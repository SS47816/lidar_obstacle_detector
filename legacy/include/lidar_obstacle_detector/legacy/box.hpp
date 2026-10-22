/* box.hpp (v1 legacy copy, kept only as the benchmark baseline)

 * Copyright (C) 2021 SS47816

 * Implementation of the Box and BoxQ class

**/

#pragma once

#include <Eigen/Geometry>

namespace lidar_obstacle_detector {
namespace legacy {

struct Box {
 public:
  int id;
  Eigen::Vector3f position;
  Eigen::Vector3f dimension;
  Eigen::Quaternionf quaternion;

  Box() {}

  Box(int id, Eigen::Vector3f position, Eigen::Vector3f dimension)
      : id(id),
        position(position),
        dimension(dimension),
        quaternion(Eigen::Quaternionf(1, 0, 0, 0)) {}

  Box(int id, Eigen::Vector3f position, Eigen::Vector3f dimension,
      Eigen::Quaternionf quaternion)
      : id(id),
        position(position),
        dimension(dimension),
        quaternion(quaternion) {}
};

}  // namespace legacy
}  // namespace lidar_obstacle_detector
