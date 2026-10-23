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
  std::vector<Box> detect(const std::vector<SweepPoint> &points) override {
    last_points = points.size();
    input = points;
    return boxes_;
  }
  std::string name() const override { return "fake"; }
  size_t last_points = 0;
  std::vector<SweepPoint> input;

 private:
  std::vector<Box> boxes_;
};

// Ground, a car at (10, 5) and a pole at (-8, -4); `truck` adds a 12 m truck
// at y = -8 with a 1.2 m gap in its side (two clusters)
Cloud::Ptr scene(std::mt19937 *rng, bool truck = false) {
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
  if (truck) {
    for (float x = 4.f; x <= 16.f; x += 0.1f) {
      if (x > 9.4f && x < 10.6f) continue;
      for (float z = 0.3f; z <= 3.f; z += 0.15f) add(x, -6.8f, z - kSensorHeight);
    }
  }
  return cloud;
}

Box learnedTruck() {
  // the network boxes only x = 9..15 of the 12 m truck (x = 4..16)
  Box b(0, Eigen::Vector3f(12.f, -7.8f, -0.2f), Eigen::Vector3f(6.f, 2.4f, 3.f), 0.f);
  b.label = ObjectClass::kTruck;
  b.score = 0.8f;
  b.heading_known = true;
  return b;
}

bool pillarNear(const std::vector<Pillar> &pillars, float x, float y, float r) {
  for (const auto &p : pillars)
    if ((p.center - Eigen::Vector2f(x, y)).norm() < r) return true;
  return false;
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
  cfg.method = DetectorMethod::kLearning;
  if (!learnedDetectorsAvailable())
    EXPECT_THROW(makeLearnedDetector(cfg, 1.8f), std::runtime_error);
}

TEST(Learned, LearningModeUsesOnlyTheNetwork) {
  ObstacleDetector<pcl::PointXYZI> det;
  det.setLearnedDetector(std::make_unique<FakeDetector>(std::vector<Box>{learnedCar()}));
  det.setConfig(config(DetectorMethod::kLearning));
  std::mt19937 rng(1);
  const auto res = det.process(scene(&rng), 0.0);
  ASSERT_EQ(res.boxes.size(), 1u);
  EXPECT_EQ(res.boxes[0].label, ObjectClass::kCar);
  EXPECT_TRUE(res.boxes[0].heading_known);
  EXPECT_NEAR(res.boxes[0].yaw, 0.02f, 1e-5);
  EXPECT_NEAR(res.boxes[0].size.x(), 4.6f, 1e-5);  // no 90 deg re-alignment
  EXPECT_TRUE(res.pillars.empty());                // unrecognised obstacles are not reported
}

TEST(Learned, FusedReportsUnrecognisedObstaclesAsPillars) {
  auto fake = std::make_unique<FakeDetector>(std::vector<Box>{learnedCar()});
  FakeDetector *raw = fake.get();
  ObstacleDetector<pcl::PointXYZI> det;
  det.setLearnedDetector(std::move(fake));
  det.setConfig(config(DetectorMethod::kFused));
  std::mt19937 rng(2);
  const auto res = det.process(scene(&rng), 0.0);
  ASSERT_EQ(res.boxes.size(), 1u);  // only the recognised car is an object
  EXPECT_EQ(res.boxes[0].label, ObjectClass::kCar);
  EXPECT_GT(res.clusters[0].size(), 50u);                  // with the car's points
  EXPECT_TRUE(pillarNear(res.pillars, -8.f, -4.f, 0.5f));  // the pole
  EXPECT_FALSE(pillarNear(res.pillars, 10.f, 5.f, 2.f));   // nothing left of the car
  EXPECT_GT(raw->last_points, 1000u);  // the network sees the full-resolution crop
}

TEST(Learned, FusedAbsorbsFragmentsOfAnUndersizedBox) {
  ObstacleDetector<pcl::PointXYZI> det;
  det.setLearnedDetector(std::make_unique<FakeDetector>(std::vector<Box>{learnedTruck()}));
  det.setConfig(config(DetectorMethod::kFused));
  std::mt19937 rng(4);
  const auto res = det.process(scene(&rng, true), 0.0);
  ASSERT_EQ(res.boxes.size(), 1u);
  // The front cluster (x 10.6..16) is mostly inside the box: absorbed whole,
  // including the end that sticks out of the box
  EXPECT_FALSE(pillarNear(res.pillars, 15.8f, -6.8f, 0.3f));
  // The rear cluster (x 4..9.4) is mostly outside: it stays an unrecognised
  // obstacle, except for its points inside the box
  EXPECT_TRUE(pillarNear(res.pillars, 6.f, -6.8f, 0.5f));
  EXPECT_FALSE(pillarNear(res.pillars, 9.2f, -6.8f, 0.2f));
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
  det.setConfig(config(DetectorMethod::kLearning));
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

TEST(Learned, MultiSweepInputIsMotionCompensated) {
  auto fake = std::make_unique<FakeDetector>(std::vector<Box>{});
  FakeDetector *raw = fake.get();
  ObstacleDetector<pcl::PointXYZI> det;
  det.setLearnedDetector(std::move(fake));
  det.setConfig(config(DetectorMethod::kLearning));
  std::mt19937 rng(6);
  // Driving +x at 10 m/s past a static pole at world (2, -4)
  for (int k = 0; k < 4; ++k) {
    Eigen::Isometry3f odom_T_sensor = Eigen::Isometry3f::Identity();
    odom_T_sensor.translation().x() = 10.f + 1.f * k;
    Cloud::Ptr cloud(new Cloud);
    for (float z = 0.1f; z <= 1.7f; z += 0.1f)
      for (float a = 0.f; a < 6.28f; a += 0.5f) {
        pcl::PointXYZI p;
        p.x = 2.f - odom_T_sensor.translation().x() + 0.25f * std::cos(a);
        p.y = -4.f + 0.25f * std::sin(a);
        p.z = z - kSensorHeight;
        p.intensity = 1.f;
        cloud->push_back(p);
      }
    det.process(cloud, 0.1 * k, odom_T_sensor);
  }
  const float pole_x = 2.f - 13.f;  // in the current sensor frame
  float max_lag = 0.f;
  for (const auto &p : raw->input) {
    max_lag = std::max(max_lag, p.time_lag);
    EXPECT_NEAR(p.x, pole_x, 0.3f) << "lag " << p.time_lag;
  }
  EXPECT_NEAR(max_lag, 0.3f, 1e-4);  // three earlier scans

  // Without a tracking frame there is no ego motion to compensate: one scan
  det.reset();
  det.process(scene(&rng), 1.0);
  det.process(scene(&rng), 1.1);
  for (const auto &p : raw->input) EXPECT_EQ(p.time_lag, 0.f);
}
