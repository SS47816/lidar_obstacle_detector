#include <gtest/gtest.h>

#include <cmath>
#include <random>

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

namespace {

// A 10 m wall at x = 8 in the odom frame, scanned while driving past it, with
// most points dropped at random each scan (sparse returns)
std::vector<Eigen::Vector4f> wallSamples(const Eigen::Isometry3f &odom_T_sensor,
                                         std::mt19937 *rng) {
  std::bernoulli_distribution keep(0.2);
  std::vector<Eigen::Vector4f> out;
  for (float y = -5.f; y <= 5.f; y += 0.2f)
    for (float z = -1.5f; z <= 0.5f; z += 0.5f)
      if (keep(*rng)) out.emplace_back(8.f, y + 0.05f, -1.8f, z);
  (void)odom_T_sensor;
  return out;
}

OccupancyGrid2D unknownGrid() {
  OccupancyGrid2D grid;
  grid.resolution = 0.2f;
  grid.origin = Eigen::Vector2f(-30.f, -30.f);
  grid.width = grid.height = 300;
  grid.data.assign(300 * 300, OccupancyGrid2D::kUnknown);
  return grid;
}

}  // namespace

TEST(Pillars, FusionKeepsSparseStaticStructuresStable) {
  std::mt19937 rng(9);
  const OccupancyGrid2D grid = unknownGrid();
  PillarMapper fused, single;
  std::vector<Pillar> a, b;
  size_t fused_min = 1000, single_min = 1000;
  for (int k = 0; k < 20; ++k) {
    Eigen::Isometry3f odom_T_sensor = Eigen::Isometry3f::Identity();
    odom_T_sensor.translation().x() = 0.5f * k;  // samples are already in odom
    const auto samples = wallSamples(odom_T_sensor, &rng);
    fused.update(samples, grid, {}, odom_T_sensor, 0.1 * k, 0.2f, 2.f, &a);
    single.update(samples, grid, {}, odom_T_sensor, 0.1 * k, 0.2f, 0.f, &b);
    if (k >= 5) {
      fused_min = std::min(fused_min, a.size());
      single_min = std::min(single_min, b.size());
    }
  }
  EXPECT_GE(fused_min, 49u);   // every cell of the wall, every scan
  EXPECT_LT(single_min, 45u);  // single scans miss cells
  for (const auto &p : a) {
    EXPECT_NEAR(p.center.x() + 0.5f * 19, 8.1f, 0.11f);  // sensor frame
    EXPECT_NEAR(p.z_min, -1.8f, 1e-4);
    EXPECT_GT(p.z_max, -1.f);
  }
}

TEST(Pillars, FreeSpaceAndRecognisedObjectsClearRememberedCells) {
  OccupancyGrid2D grid = unknownGrid();
  PillarMapper mapper;
  std::vector<Pillar> out;
  const Eigen::Isometry3f I = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f odom = I;
  odom.translation().x() = 1.f;
  const std::vector<Eigen::Vector4f> two{{6.05f, 0.05f, -1.8f, 0.f}, {6.05f, 4.05f, -1.8f, 0.f}};
  for (int k = 0; k < 3; ++k) mapper.update(two, grid, {}, odom, 0.1 * k, 0.2f, 2.f, &out);
  ASSERT_EQ(out.size(), 2u);
  // Next scan: nothing returned; the first cell is seen free, the second is
  // now covered by a recognised car
  int gx, gy;
  ASSERT_TRUE(grid.cellOf(5.1f, 0.1f, &gx, &gy));
  grid.data[static_cast<size_t>(gy) * grid.width + gx] = OccupancyGrid2D::kFree;
  const Box car(0, Eigen::Vector3f(5.f, 4.f, 0.f), Eigen::Vector3f(4.f, 2.f, 1.5f), 0.f);
  mapper.update({}, grid, {}, odom, 0.3, 0.2f, 2.f, &out);
  mapper.update({}, grid, {}, odom, 0.4, 0.2f, 2.f, &out);
  ASSERT_EQ(out.size(), 1u);  // free space cleared the first cell
  EXPECT_FALSE(out[0].observed);
  EXPECT_NEAR(out[0].center.y(), 4.1f, 1e-4);
  mapper.update({}, grid, {car}, odom, 0.5, 0.2f, 2.f, &out);
  EXPECT_TRUE(out.empty());
}
