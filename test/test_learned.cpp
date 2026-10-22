#include <gtest/gtest.h>

#include <random>

#include "lidar_obstacle_detector/obstacle_detector.hpp"

using namespace lidar_obstacle_detector;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

namespace {

constexpr float kSensorHeight = 1.8f;

// Reports a fixed list of boxes, like a network that recognises some objects
class FakeDetector : public LearnedDetector {
 public:
  explicit FakeDetector(std::vector<Box> boxes) : boxes_(std::move(boxes)) {}
  std::vector<Box> detect(const std::vector<Eigen::Vector4f> &points) override {
    last_points = points.size();
    return boxes_;
  }
  std::string name() const override { return "fake"; }
  size_t last_points = 0;

 private:
  std::vector<Box> boxes_;
};

// Ground, a car at (10, 5) and a pole at (-8, -4)
Cloud::Ptr scene(std::mt19937 *rng) {
  std::normal_distribution<float> n(0.f, 0.01f);
  Cloud::Ptr cloud(new Cloud);
  auto add = [&](float x, float y, float z) {
    pcl::PointXYZI p;
    p.x = x + n(*rng), p.y = y + n(*rng), p.z = z + n(*rng), p.intensity = 10.f;
    cloud->push_back(p);
  };
  for (float x = -30.f; x <= 30.f; x += 0.3f)
    for (float y = -20.f; y <= 20.f; y += 0.3f) add(x, y, -kSensorHeight);
  for (float x = 7.8f; x <= 12.2f; x += 0.1f)
    for (float z = 0.1f; z <= 1.5f; z += 0.1f) {
      add(x, 4.1f, z - kSensorHeight);
      add(x, 5.9f, z - kSensorHeight);
    }
  for (float z = 0.1f; z <= 1.7f; z += 0.05f)
    for (float a = 0.f; a < 6.28f; a += 0.5f)
      add(-8.f + 0.25f * std::cos(a), -4.f + 0.25f * std::sin(a), z - kSensorHeight);
  return cloud;
}

Box learnedCar() {
  Box car(0, Eigen::Vector3f(10.f, 5.f, -1.0f), Eigen::Vector3f(4.6f, 1.9f, 1.5f), 0.02f);
  car.label = ObjectClass::kCar;
  car.score = 0.9f;
  car.heading_known = true;
  return car;
}

PipelineConfig config(DetectorMethod method) {
  PipelineConfig cfg = PipelineConfig::preset("fast");
  cfg.preprocess.roi_min = Eigen::Vector3f(-30.f, -20.f, -2.5f);
  cfg.preprocess.roi_max = Eigen::Vector3f(30.f, 20.f, 1.f);
  cfg.ground.sensor_height = kSensorHeight;
  cfg.detector.method = method;
  return cfg;
}

}  // namespace

TEST(Learned, FactoryReportsAvailability) {
  DetectorConfig cfg;
  EXPECT_EQ(makeLearnedDetector(cfg, 1.8f), nullptr);  // clustering: no network
  cfg.method = DetectorMethod::kCenterPoint;
  if (!learnedDetectorsAvailable())
    EXPECT_THROW(makeLearnedDetector(cfg, 1.8f), std::runtime_error);
}

TEST(Learned, CenterPointModeUsesOnlyTheNetwork) {
  ObstacleDetector<pcl::PointXYZI> det;
  det.setLearnedDetector(std::make_unique<FakeDetector>(std::vector<Box>{learnedCar()}));
  det.setConfig(config(DetectorMethod::kCenterPoint));
  std::mt19937 rng(1);
  const auto res = det.process(scene(&rng), 0.0);
  ASSERT_EQ(res.boxes.size(), 1u);
  EXPECT_EQ(res.boxes[0].label, ObjectClass::kCar);
  EXPECT_TRUE(res.boxes[0].heading_known);
  EXPECT_NEAR(res.boxes[0].yaw, 0.02f, 1e-5);
  EXPECT_NEAR(res.boxes[0].size.x(), 4.6f, 1e-5);  // no 90 deg re-alignment
}

TEST(Learned, HybridKeepsUncoveredClustersAsUnknown) {
  auto fake = std::make_unique<FakeDetector>(std::vector<Box>{learnedCar()});
  FakeDetector *raw = fake.get();
  ObstacleDetector<pcl::PointXYZI> det;
  det.setLearnedDetector(std::move(fake));
  det.setConfig(config(DetectorMethod::kHybrid));
  std::mt19937 rng(2);
  const auto res = det.process(scene(&rng), 0.0);
  ASSERT_EQ(res.boxes.size(), 2u);  // the learned car + the pole cluster
  int cars = 0, unknown = 0;
  for (const auto &b : res.boxes) {
    if (b.label == ObjectClass::kCar) ++cars;
    if (b.label == ObjectClass::kUnknown) {
      ++unknown;
      EXPECT_NEAR(b.center.x(), -8.f, 0.3f);
    }
  }
  EXPECT_EQ(cars, 1);
  EXPECT_EQ(unknown, 1);
  EXPECT_GT(raw->last_points, 1000u);  // the network sees the full-resolution crop
}

TEST(Learned, TrackKeepsClassWhenOnlyClustersFollow) {
  MultiObjectTracker tracker;
  Box car = learnedCar();
  std::vector<Box> a{car};
  tracker.update(0.0, &a);
  Box cluster(0, car.center + Eigen::Vector3f(0.1f, 0.f, 0.f), Eigen::Vector3f(4.4f, 1.8f, 1.4f),
              1.6f);
  std::vector<Box> b{cluster};
  tracker.update(0.1, &b);
  EXPECT_EQ(b[0].id, a[0].id);
  EXPECT_EQ(b[0].label, ObjectClass::kCar);  // inherited from the track
}

TEST(Learned, TracksDoNotSwitchClassGroups) {
  MultiObjectTracker tracker;
  Box ped(0, Eigen::Vector3f(5.f, 0.f, 0.f), Eigen::Vector3f(0.6f, 0.6f, 1.7f), 0.f);
  ped.label = ObjectClass::kPedestrian;
  std::vector<Box> a{ped};
  tracker.update(0.0, &a);
  Box car = learnedCar();
  car.center = ped.center;
  std::vector<Box> b{car};
  tracker.update(0.1, &b);
  EXPECT_NE(b[0].id, a[0].id);
}

TEST(Learned, DuplicateDetectionsAreSuppressed) {
  Box car = learnedCar();
  Box truck = car;  // same object, second class head
  truck.label = ObjectClass::kTruck;
  truck.score = 0.5f;
  truck.size.x() = 5.f;
  ObstacleDetector<pcl::PointXYZI> det;
  det.setLearnedDetector(std::make_unique<FakeDetector>(std::vector<Box>{truck, car}));
  det.setConfig(config(DetectorMethod::kCenterPoint));
  std::mt19937 rng(3);
  const auto res = det.process(scene(&rng), 0.0);
  ASSERT_EQ(res.boxes.size(), 1u);
  EXPECT_EQ(res.boxes[0].label, ObjectClass::kCar);  // the higher score wins
}

TEST(Learned, MovingTrackFixesFlippedLearnedHeading) {
  MultiObjectTracker tracker;
  for (int k = 0; k < 10; ++k) {
    Box car = learnedCar();
    car.center.x() = 10.f + 1.f * k;                          // driving +x at 10 m/s
    car.yaw = (k % 3 == 2) ? static_cast<float>(M_PI) : 0.f;  // network flips now and then
    std::vector<Box> boxes{car};
    tracker.update(0.1 * k, &boxes);
    if (k >= 4) EXPECT_NEAR(std::abs(wrapAngle(boxes[0].yaw)), 0.f, 1e-4);
  }
}
