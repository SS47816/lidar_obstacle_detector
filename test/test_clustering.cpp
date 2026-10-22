#include <gtest/gtest.h>

#include <random>

#include "lidar_obstacle_detector/clustering.hpp"

using namespace lidar_obstacle_detector;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

namespace {

void addBlob(Cloud *cloud, float x, float y, int n, float spread, std::mt19937 *rng) {
  std::uniform_real_distribution<float> u(-spread, spread);
  for (int i = 0; i < n; ++i) cloud->push_back({x + u(*rng), y + u(*rng), u(*rng)});
}

size_t totalPoints(const std::vector<pcl::Indices> &clusters) {
  size_t n = 0;
  for (const auto &c : clusters) n += c.size();
  return n;
}

class AllMethods : public ::testing::TestWithParam<ClusterMethod> {};

}  // namespace

TEST_P(AllMethods, SeparatesWellSeparatedBlobs) {
  std::mt19937 rng(1);
  Cloud::Ptr cloud(new Cloud);
  addBlob(cloud.get(), 0.f, 0.f, 200, 0.5f, &rng);
  addBlob(cloud.get(), 5.f, 0.f, 150, 0.5f, &rng);
  addBlob(cloud.get(), 0.f, -6.f, 100, 0.5f, &rng);
  addBlob(cloud.get(), 20.f, 20.f, 3, 0.1f, &rng);  // too small
  ClusterConfig cfg;
  cfg.method = GetParam();
  cfg.tolerance = 0.6f;
  cfg.min_size = 10;
  const auto clusters = clusterCloud<pcl::PointXYZ>(cloud, cfg);
  ASSERT_EQ(clusters.size(), 3u);
  EXPECT_EQ(totalPoints(clusters), 450u);
  std::vector<size_t> sizes;
  for (const auto &c : clusters) sizes.push_back(c.size());
  std::sort(sizes.begin(), sizes.end());
  EXPECT_EQ(sizes, (std::vector<size_t>{100, 150, 200}));
}

TEST_P(AllMethods, RespectsMaxSize) {
  std::mt19937 rng(2);
  Cloud::Ptr cloud(new Cloud);
  addBlob(cloud.get(), 0.f, 0.f, 500, 0.5f, &rng);
  addBlob(cloud.get(), 10.f, 0.f, 50, 0.5f, &rng);
  ClusterConfig cfg;
  cfg.method = GetParam();
  cfg.max_size = 100;
  const auto clusters = clusterCloud<pcl::PointXYZ>(cloud, cfg);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 50u);
}

TEST_P(AllMethods, EmptyCloud) {
  ClusterConfig cfg;
  cfg.method = GetParam();
  EXPECT_TRUE(clusterCloud<pcl::PointXYZ>(Cloud::Ptr(new Cloud), cfg).empty());
}

INSTANTIATE_TEST_SUITE_P(Clustering, AllMethods,
                         ::testing::Values(ClusterMethod::kEuclidean,
                                           ClusterMethod::kAdaptiveEuclidean,
                                           ClusterMethod::kBevGrid));

TEST(Clustering, AdaptiveToleranceKeepsSparseFarObjectsTogether) {
  // A distant object whose points are 0.9 m apart (beam divergence)
  Cloud::Ptr cloud(new Cloud);
  for (int i = 0; i < 12; ++i) cloud->push_back({40.f, -5.f + 0.9f * i, 0.f});
  ClusterConfig cfg;
  cfg.tolerance = 0.6f;
  cfg.min_size = 3;
  cfg.method = ClusterMethod::kEuclidean;
  EXPECT_EQ(clusterCloud<pcl::PointXYZ>(cloud, cfg).size(), 0u);  // all singletons
  cfg.method = ClusterMethod::kAdaptiveEuclidean;
  cfg.range_gain = 0.03f;  // 1.2 m at 40 m
  EXPECT_EQ(clusterCloud<pcl::PointXYZ>(cloud, cfg).size(), 1u);
  cfg.method = ClusterMethod::kBevGrid;
  EXPECT_EQ(clusterCloud<pcl::PointXYZ>(cloud, cfg).size(), 1u);
}

namespace {
// Points on the outer surface of an upright box
void addBoxSurface(Cloud *cloud, float x0, float x1, float y0, float y1, float z0, float z1,
                   float step = 0.1f) {
  for (float x = x0; x <= x1; x += step)
    for (float z = z0; z <= z1; z += step) {
      cloud->push_back({x, y0, z});
      cloud->push_back({x, y1, z});
    }
  for (float y = y0; y <= y1; y += step)
    for (float z = z0; z <= z1; z += step) {
      cloud->push_back({x0, y, z});
      cloud->push_back({x1, y, z});
    }
  for (float x = x0; x <= x1; x += step)
    for (float y = y0; y <= y1; y += step) cloud->push_back({x, y, z1});
}
}  // namespace

TEST(Clustering, HeightGapSeparatesCanopyFromCarBelow) {
  Cloud::Ptr cloud(new Cloud);
  addBoxSurface(cloud.get(), 8.f, 12.5f, -0.9f, 0.9f, -1.6f, -0.2f);  // car
  for (float x = 6.f; x <= 15.f; x += 0.2f)                           // canopy above it
    for (float y = -3.f; y <= 3.f; y += 0.2f) cloud->push_back({x, y, 0.9f});
  ClusterConfig cfg;
  cfg.method = ClusterMethod::kBevGrid;
  cfg.max_size = 100000;
  EXPECT_EQ(clusterCloud<pcl::PointXYZ>(cloud, cfg).size(), 1u);  // merged in BEV
  cfg.z_gap = 0.8f;
  EXPECT_EQ(clusterCloud<pcl::PointXYZ>(cloud, cfg).size(), 2u);
}

TEST(Clustering, HeightGapKeepsSparseDistantCarTogether) {
  // Two beams 0.9 m apart vertically on a car 30 m away must not split it
  Cloud::Ptr cloud(new Cloud);
  for (float y = -1.f; y <= 1.f; y += 0.1f) {
    cloud->push_back({30.f, y, -1.3f});
    cloud->push_back({30.f, y, -0.4f});
  }
  ClusterConfig cfg;
  cfg.method = ClusterMethod::kBevGrid;
  cfg.z_gap = 0.8f;
  cfg.min_size = 5;
  EXPECT_EQ(clusterCloud<pcl::PointXYZ>(cloud, cfg).size(), 1u);
}

TEST(Clustering, OversizedClustersAreSplitOrBecomeStructures) {
  Cloud::Ptr cloud(new Cloud);
  // four cars in a row, 0.5 m apart: one 19.5 m cluster at tolerance 0.6
  for (int k = 0; k < 4; ++k)
    addBoxSurface(cloud.get(), 5.f * k, 5.f * k + 4.5f, 5.f, 6.8f, -1.6f, -0.2f);
  addBoxSurface(cloud.get(), -15.f, -3.f, -8.f, -5.5f, -1.6f, 1.f);  // a 12 m bus
  for (float x = -20.f; x <= 20.f; x += 0.15f)                       // a 40 m wall
    for (float z = -1.6f; z <= 1.f; z += 0.15f) cloud->push_back({x, -20.f, z});

  ClusterConfig cfg;
  cfg.method = ClusterMethod::kBevGrid;
  cfg.max_size = 1000000;
  auto clusters = clusterCloud<pcl::PointXYZ>(cloud, cfg);
  ASSERT_EQ(clusters.size(), 3u);

  cfg.split_oversized = true;
  cfg.separate_structures = true;
  std::vector<pcl::Indices> structures;
  splitOversizedClusters<pcl::PointXYZ>(cloud, cfg, &clusters, &structures);
  EXPECT_EQ(clusters.size(), 5u);    // four cars + the bus
  ASSERT_EQ(structures.size(), 1u);  // the wall
  for (const auto i : structures[0]) EXPECT_FLOAT_EQ(cloud->points[i].y, -20.f);
}
