#include <gtest/gtest.h>

#include <random>

#include "lidar_obstacle_detector/obstacle_detector.hpp"

using namespace lidar_obstacle_detector;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

namespace {

constexpr float kSensorHeight = 1.8f;

// Flat ground plus two box-shaped obstacles (all surfaces sampled)
Cloud::Ptr syntheticScene(float car_x, std::mt19937 *rng) {
  std::normal_distribution<float> n(0.f, 0.01f);
  Cloud::Ptr cloud(new Cloud);
  auto add = [&](float x, float y, float z) {
    pcl::PointXYZI p;
    p.x = x + n(*rng);
    p.y = y + n(*rng);
    p.z = z + n(*rng);
    p.intensity = 0.f;
    cloud->push_back(p);
  };
  for (float x = -30.f; x <= 30.f; x += 0.3f)
    for (float y = -20.f; y <= 20.f; y += 0.3f) add(x, y, -kSensorHeight);
  // a car (4.4 x 1.8 x 1.5) at (car_x, 5) and a pedestrian-sized pole at (-8, -4)
  for (float x = -2.2f; x <= 2.2f; x += 0.1f)
    for (float z = 0.1f; z <= 1.5f; z += 0.1f) {
      add(car_x + x, 5.f - 0.9f, z - kSensorHeight);
      add(car_x + x, 5.f + 0.9f, z - kSensorHeight);
    }
  for (float y = -0.9f; y <= 0.9f; y += 0.1f)
    for (float z = 0.1f; z <= 1.5f; z += 0.1f) {
      add(car_x - 2.2f, 5.f + y, z - kSensorHeight);
      add(car_x + 2.2f, 5.f + y, z - kSensorHeight);
    }
  for (float z = 0.1f; z <= 1.7f; z += 0.05f)
    for (float a = 0.f; a < 6.28f; a += 0.5f)
      add(-8.f + 0.25f * std::cos(a), -4.f + 0.25f * std::sin(a), z - kSensorHeight);
  return cloud;
}

PipelineConfig sceneConfig(const std::string &preset) {
  PipelineConfig cfg = PipelineConfig::preset(preset);
  cfg.preprocess.roi_min = Eigen::Vector3f(-30.f, -20.f, -2.5f);
  cfg.preprocess.roi_max = Eigen::Vector3f(30.f, 20.f, 1.f);
  cfg.ground.sensor_height = kSensorHeight;
  return cfg;
}

class Presets : public ::testing::TestWithParam<std::string> {};

}  // namespace

TEST_P(Presets, DetectsCarAndPoleAndKeepsTheirIds) {
  ObstacleDetector<pcl::PointXYZI> detector(sceneConfig(GetParam()));
  std::mt19937 rng(5);
  uint32_t car_id = 0;
  for (int k = 0; k < 6; ++k) {
    const float car_x = 10.f + 1.f * k;  // 10 m/s at 10 Hz
    const auto res = detector.process(syntheticScene(car_x, &rng), 0.1 * k);
    ASSERT_EQ(res.boxes.size(), 2u) << "frame " << k;
    ASSERT_EQ(res.clusters.size(), 2u);
    const Box &car = res.boxes[0].size.x() > res.boxes[1].size.x() ? res.boxes[0] : res.boxes[1];
    EXPECT_NEAR(car.center.x(), car_x, 0.3f);
    EXPECT_NEAR(car.center.y(), 5.f, 0.3f);
    EXPECT_NEAR(car.size.x(), 4.4f, 0.4f);
    EXPECT_NEAR(car.size.y(), 1.8f, 0.4f);
    if (k == 0) car_id = car.id;
    EXPECT_EQ(car.id, car_id);
    if (k >= 4) EXPECT_NEAR(car.velocity.x(), 10.f, 2.f);
    EXPECT_GT(res.ground->size(), res.obstacles->size());
    EXPECT_GE(res.timings.total, res.timings.cluster);
  }
}

INSTANTIATE_TEST_SUITE_P(Pipeline, Presets, ::testing::Values("legacy", "fast", "accurate"));

TEST(Pipeline, TracksInFixedFrameWhileEgoMoves) {
  // The ego drives at 5 m/s alongside a parked car: in the sensor frame the car
  // moves backwards, but in the tracking frame its velocity must be ~0.
  ObstacleDetector<pcl::PointXYZI> detector(sceneConfig("fast"));
  std::mt19937 rng(9);
  Box last;
  for (int k = 0; k < 8; ++k) {
    const float ego_x = 0.5f * k;
    Eigen::Isometry3f odom_T_sensor = Eigen::Isometry3f::Identity();
    odom_T_sensor.translation().x() = ego_x;
    const auto res = detector.process(syntheticScene(15.f - ego_x, &rng), 0.1 * k, odom_T_sensor);
    ASSERT_EQ(res.boxes.size(), 2u);
    last = res.boxes[0].size.x() > res.boxes[1].size.x() ? res.boxes[0] : res.boxes[1];
    EXPECT_NEAR(last.center.x(), 15.f - ego_x, 0.3f);  // output stays in the sensor frame
  }
  EXPECT_NEAR(last.velocity.norm(), 0.f, 0.8f);
}

TEST(Pipeline, ConfigChangesApplyBetweenFrames) {
  ObstacleDetector<pcl::PointXYZI> detector(sceneConfig("fast"));
  std::mt19937 rng(1);
  auto cfg = detector.config();
  cfg.cluster.min_size = 100000;
  detector.setConfig(cfg);
  EXPECT_TRUE(detector.process(syntheticScene(10.f, &rng), 0.0).boxes.empty());
}

TEST(Preprocess, VoxelDownsampleAveragesEachVoxel) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  auto add = [&](float x, float y, float z, float i) {
    pcl::PointXYZI p;
    p.x = x, p.y = y, p.z = z, p.intensity = i;
    cloud.push_back(p);
  };
  add(0.01f, 0.01f, 0.01f, 1.f);
  add(0.09f, 0.05f, 0.03f, 3.f);   // same 0.1 m voxel
  add(-0.05f, 0.05f, 0.05f, 5.f);  // negative side: another voxel
  add(0.55f, 0.55f, 0.55f, 7.f);
  const auto out = voxelDownsample(cloud, 0.1f, Eigen::Vector3f(-1.f, -1.f, -1.f));
  ASSERT_EQ(out->size(), 3u);
  EXPECT_NEAR(out->points[0].x, 0.05f, 1e-6);
  EXPECT_NEAR(out->points[0].y, 0.03f, 1e-6);
  EXPECT_NEAR(out->points[0].intensity, 2.f, 1e-6);
  EXPECT_NEAR(out->points[1].x, -0.05f, 1e-6);
  EXPECT_NEAR(out->points[2].intensity, 7.f, 1e-6);
}

TEST(Pipeline, StructuresAndOccupancyAreReported) {
  ObstacleDetector<pcl::PointXYZI> detector(sceneConfig("fast"));
  std::mt19937 rng(3);
  auto cloud = syntheticScene(10.f, &rng);
  for (float x = -25.f; x <= 25.f; x += 0.1f)  // a 50 m wall along the road
    for (float z = 0.1f; z <= 2.f; z += 0.1f) {
      pcl::PointXYZI p;
      p.x = x, p.y = -12.f, p.z = z - kSensorHeight, p.intensity = 0.f;
      cloud->push_back(p);
    }
  const auto res = detector.process(cloud, 0.0);
  EXPECT_EQ(res.boxes.size(), 2u);  // car and pole; the wall is not an object
  ASSERT_EQ(res.structures.size(), 1u);
  EXPECT_GT(res.structure_boxes[0].size.x(), 40.f);
  EXPECT_EQ(res.occupancy.atPoint(10.f, -12.f), OccupancyGrid2D::kOccupied);
  EXPECT_EQ(res.occupancy.atPoint(5.f, 0.f), OccupancyGrid2D::kFree);
  EXPECT_FALSE(res.occupied_voxels->empty());
}

namespace {

// A 12 m truck side at y = 6 (x = 4..16); with `gap`, a 1.4 m stretch of it
// returns no points (e.g. between cab and trailer), splitting the cluster
Cloud::Ptr truckScene(bool gap, std::mt19937 *rng) {
  std::normal_distribution<float> n(0.f, 0.01f);
  Cloud::Ptr cloud(new Cloud);
  auto add = [&](float x, float y, float z) {
    pcl::PointXYZI p;
    p.x = x + n(*rng), p.y = y + n(*rng), p.z = z + n(*rng), p.intensity = 0.f;
    cloud->push_back(p);
  };
  for (float x = -30.f; x <= 30.f; x += 0.3f)
    for (float y = -20.f; y <= 20.f; y += 0.3f) add(x, y, -kSensorHeight);
  for (float x = 4.f; x <= 16.f; x += 0.1f) {
    if (gap && x > 8.3f && x < 9.7f) continue;
    for (float z = 0.3f; z <= 3.f; z += 0.15f) add(x, 6.f, z - kSensorHeight);
  }
  for (float y = 6.f; y <= 8.4f; y += 0.1f)  // its rear face
    for (float z = 0.3f; z <= 3.f; z += 0.15f) add(4.f, y, z - kSensorHeight);
  return cloud;
}

}  // namespace

TEST(Pipeline, TrackGuidedClusteringRejoinsFragmentsOfATrackedTruck) {
  for (bool guided : {false, true}) {
    PipelineConfig cfg = sceneConfig("fast");
    cfg.cluster.track_guided = guided;
    ObstacleDetector<pcl::PointXYZI> detector(cfg);
    std::mt19937 rng(11);
    size_t boxes = 0;
    for (int k = 0; k < 8; ++k) {
      const auto res = detector.process(truckScene(k >= 5, &rng), 0.1 * k);
      if (k < 5) ASSERT_EQ(res.boxes.size(), 1u);
      boxes = res.boxes.size();
      if (guided && k >= 5) {
        ASSERT_EQ(boxes, 1u) << "frame " << k;
        EXPECT_NEAR(res.boxes[0].size.x(), 12.f, 0.5f);
      }
    }
    if (!guided) EXPECT_EQ(boxes, 2u);  // without the tracks, the gap splits it
  }
}

TEST(Pipeline, ClusteringModeReportsAllObstaclesAsPillarsToo) {
  ObstacleDetector<pcl::PointXYZI> detector(sceneConfig("fast"));
  std::mt19937 rng(12);
  const auto res = detector.process(syntheticScene(10.f, &rng), 0.0);
  ASSERT_EQ(res.boxes.size(), 2u);
  EXPECT_GT(res.pillar_resolution, 0.f);
  int car = 0, pole = 0;
  for (const auto &p : res.pillars) {
    if (std::abs(p.center.x() - 10.f) < 2.5f && std::abs(p.center.y() - 5.f) < 1.2f) ++car;
    if ((p.center - Eigen::Vector2f(-8.f, -4.f)).norm() < 0.5f) ++pole;
    EXPECT_LT(p.z_min, p.z_max);
  }
  EXPECT_GT(car, 10);
  EXPECT_GT(pole, 0);
}
