/* benchmark_types.hpp

 * Copyright (C) 2021-2026 SS47816

 * Common interface between the benchmark CLI and the pipelines it compares.

**/

#pragma once

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
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
};

struct FrameOutput {
  std::vector<OutputBox> boxes;
  std::vector<Eigen::Vector3f> ground, obstacles;  // processed clouds
  std::vector<std::pair<std::string, double>> timing_ms;
};

class PipelineAdapter {
 public:
  virtual ~PipelineAdapter() = default;
  virtual FrameOutput process(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, double stamp,
                              const Eigen::Isometry3d &world_T_sensor) = 0;
};

inline double ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

template <typename PointT>
std::vector<Eigen::Vector3f> toXYZ(const pcl::PointCloud<PointT> &cloud) {
  std::vector<Eigen::Vector3f> out;
  out.reserve(cloud.size());
  for (const auto &p : cloud.points) out.emplace_back(p.x, p.y, p.z);
  return out;
}

inline float getFloat(const ParamMap &m, const std::string &k, float def) {
  auto it = m.find(k);
  return it == m.end() ? def : std::stof(it->second);
}
inline int getInt(const ParamMap &m, const std::string &k, int def) {
  auto it = m.find(k);
  return it == m.end() ? def : std::stoi(it->second);
}
inline bool getBool(const ParamMap &m, const std::string &k, bool def) {
  auto it = m.find(k);
  if (it == m.end()) return def;
  if (it->second == "true" || it->second == "1") return true;
  if (it->second == "false" || it->second == "0") return false;
  throw std::invalid_argument("bad bool for " + k + ": " + it->second);
}

}  // namespace bench
}  // namespace lidar_obstacle_detector
