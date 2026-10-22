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
