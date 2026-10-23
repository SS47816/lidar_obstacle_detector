/* benchmark_types.hpp

 * Copyright (C) 2021-2026 SS47816

 * Output of one frame as written by the benchmark CLI.

**/

#pragma once

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar_obstacle_detector {
namespace bench {

using ParamMap = std::map<std::string, std::string>;

struct OutputBox {
  int64_t id = -1;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Zero();  // length, width, height
  double yaw = 0.0;
  bool has_velocity = false;
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();  // world frame
  std::string label = "unknown";
  double score = 1.0;
  std::vector<Eigen::Vector2d> path;  // predicted positions (sensor frame)
};

struct FrameOutput {
  std::vector<OutputBox> boxes;
  std::vector<OutputBox> structures;               // oversized clusters reported apart from objects
  std::vector<Eigen::Vector3f> ground, obstacles;  // processed clouds
  std::vector<std::pair<std::string, double>> timing_ms;
  // 2D occupancy grid (row-major, -1/0/100), empty if not computed
  float occ_resolution = 0.f, occ_origin_x = 0.f, occ_origin_y = 0.f;
  int occ_width = 0, occ_height = 0;
  std::vector<int8_t> occupancy;
  // 2.5D pillars: x, y, z_min, z_max (sensor frame), observed (1) / remembered (0)
  std::vector<std::array<float, 5>> pillars;
};

class PipelineAdapter {
 public:
  virtual ~PipelineAdapter() = default;
  virtual FrameOutput process(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, double stamp,
                              const Eigen::Isometry3d &world_T_sensor) = 0;
};

template <typename PointT>
std::vector<Eigen::Vector3f> toXYZ(const pcl::PointCloud<PointT> &cloud) {
  std::vector<Eigen::Vector3f> out;
  out.reserve(cloud.size());
  for (const auto &p : cloud.points) out.emplace_back(p.x, p.y, p.z);
  return out;
}

}  // namespace bench
}  // namespace lidar_obstacle_detector
