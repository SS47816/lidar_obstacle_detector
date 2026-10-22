#include <gtest/gtest.h>

#include <cmath>

#include "lidar_obstacle_detector/occupancy.hpp"

using namespace lidar_obstacle_detector;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

TEST(Occupancy, ObstacleOccupiedFreeBeforeUnknownBehind) {
  Cloud ground, obstacles;
  // Flat ground, except in the shadow of a 4 m wide wall 10 m ahead
  for (float x = -19.9f; x < 20.f; x += 0.3f)
    for (float y = -19.9f; y < 20.f; y += 0.3f) {
      const bool shadow = x > 10.f && std::abs(y) < 2.f * x / 10.f;
      if (!shadow) ground.push_back({x, y, -1.8f});
    }
  for (float y = -2.f; y <= 2.f; y += 0.1f)
    for (float z = -1.7f; z <= 0.f; z += 0.1f) obstacles.push_back({10.f, y, z});
  obstacles.push_back({-5.f, 5.f, 2.f});  // an overhanging branch: above the height band

  PreprocessConfig roi;
  roi.roi_min = Eigen::Vector3f(-20.f, -20.f, -2.5f);
  roi.roi_max = Eigen::Vector3f(20.f, 20.f, 3.f);
  OccupancyConfig cfg;
  cfg.enabled = true;
  OccupancyMapper mapper;
  OccupancyGrid2D grid;
  Cloud voxels;
  computeOccupancy(ground, obstacles, roi, cfg, &mapper, &grid, &voxels);

  EXPECT_EQ(grid.width, 200);
  EXPECT_EQ(grid.height, 200);
  EXPECT_EQ(grid.atPoint(10.05f, 0.f), OccupancyGrid2D::kOccupied);
  EXPECT_EQ(grid.atPoint(5.f, 0.f), OccupancyGrid2D::kFree);
  EXPECT_EQ(grid.atPoint(15.f, 0.f), OccupancyGrid2D::kUnknown);  // occluded
  EXPECT_EQ(grid.atPoint(15.f, 10.f), OccupancyGrid2D::kFree);    // beside the shadow
  EXPECT_EQ(grid.atPoint(-5.f, 5.f), OccupancyGrid2D::kFree);     // branch is too high
  EXPECT_FALSE(voxels.empty());
  for (const auto &v : voxels.points) EXPECT_NEAR(v.x, 10.f, 0.2f);
}
