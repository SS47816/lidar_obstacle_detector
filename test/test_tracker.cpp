#include <gtest/gtest.h>

#include "lidar_obstacle_detector/tracker.hpp"

using namespace lidar_obstacle_detector;

namespace {
Box det(float x, float y, float yaw = 0.f, float l = 4.f, float w = 2.f) {
  return Box(999, Eigen::Vector3f(x, y, 0.f), Eigen::Vector3f(l, w, 1.5f), yaw);
}
}  // namespace

TEST(Tracker, ConstantVelocityTargetKeepsIdAndVelocity) {
  MultiObjectTracker tracker;
  uint32_t id = 0;
  for (int k = 0; k < 30; ++k) {
    const double t = 0.1 * k;
    std::vector<Box> boxes{det(5.f * t, 2.f - 1.f * t)};
    tracker.update(t, &boxes);
    ASSERT_EQ(boxes.size(), 1u);
    if (k == 0) id = boxes[0].id;
    EXPECT_EQ(boxes[0].id, id);
    EXPECT_EQ(boxes[0].tracked, k >= 1);  // min_hits = 2
    if (k >= 15) {
      EXPECT_NEAR(boxes[0].velocity.x(), 5.f, 0.3f);
      EXPECT_NEAR(boxes[0].velocity.y(), -1.f, 0.3f);
    }
  }
}

TEST(Tracker, NewDetectionsGetFreshIdsAndAreAllKept) {
  MultiObjectTracker tracker;
  std::vector<Box> a{det(0, 0), det(10, 0)};
  tracker.update(0.0, &a);
  std::vector<Box> b{det(0.1f, 0), det(10.1f, 0), det(30, 30)};
  tracker.update(0.1, &b);
  ASSERT_EQ(b.size(), 3u);
  EXPECT_EQ(b[0].id, a[0].id);
  EXPECT_EQ(b[1].id, a[1].id);
  EXPECT_NE(b[2].id, a[0].id);
  EXPECT_NE(b[2].id, a[1].id);
  EXPECT_FALSE(b[2].tracked);
}

TEST(Tracker, PicksTheNearestTrackNotTheFirst) {
  // v1's unweighted matching could swap these
  MultiObjectTracker tracker;
  std::vector<Box> a{det(0, 0, 0, 1, 1), det(1.5f, 0, 0, 1, 1)};
  tracker.update(0.0, &a);
  std::vector<Box> b{det(1.4f, 0, 0, 1, 1), det(0.1f, 0, 0, 1, 1)};
  tracker.update(0.1, &b);
  EXPECT_EQ(b[0].id, a[1].id);
  EXPECT_EQ(b[1].id, a[0].id);
}

TEST(Tracker, CoastsThroughShortOcclusionsAndForgetsLongOnes) {
  TrackerConfig cfg;
  cfg.max_age = 0.35f;
  MultiObjectTracker tracker(cfg);
  uint32_t id = 0;
  for (int k = 0; k < 5; ++k) {
    std::vector<Box> boxes{det(2.f * 0.1f * k, 0)};
    tracker.update(0.1 * k, &boxes);
    id = boxes[0].id;
  }
  for (int k = 5; k < 8; ++k) {  // 0.3 s without detections
    std::vector<Box> none;
    tracker.update(0.1 * k, &none);
  }
  std::vector<Box> back{det(2.f * 0.8f, 0)};
  tracker.update(0.8, &back);
  EXPECT_EQ(back[0].id, id);  // predicted forward and re-associated

  for (int k = 9; k < 20; ++k) {
    std::vector<Box> none;
    tracker.update(0.1 * k, &none);
  }
  EXPECT_EQ(tracker.numTracks(), 0u);
}

TEST(Tracker, AlignsYawAmbiguityWithTrack) {
  MultiObjectTracker tracker;
  std::vector<Box> a{det(0, 0, 0.1f, 4.f, 2.f)};
  tracker.update(0.0, &a);
  // Same object, but the box fitter flipped length / width (yaw + 90 deg)
  std::vector<Box> b{det(0, 0, 0.1f + static_cast<float>(M_PI_2), 2.f, 4.f)};
  tracker.update(0.1, &b);
  EXPECT_NEAR(b[0].yaw, 0.1f, 1e-5);
  EXPECT_FLOAT_EQ(b[0].size.x(), 4.f);
  EXPECT_FLOAT_EQ(b[0].size.y(), 2.f);
}

TEST(Tracker, GIoUCostGatesNonOverlappingBoxes) {
  TrackerConfig cfg;
  cfg.cost = TrackCost::kGIoU;
  cfg.min_giou = 0.f;
  MultiObjectTracker tracker(cfg);
  std::vector<Box> a{det(0, 0, 0, 1, 1)};
  tracker.update(0.0, &a);
  std::vector<Box> b{det(1.5f, 0, 0, 1, 1)};  // within max_distance, but GIoU < 0
  tracker.update(0.1, &b);
  EXPECT_NE(b[0].id, a[0].id);
}

TEST(Tracker, ResetsWhenTimeJumpsBack) {
  MultiObjectTracker tracker;
  std::vector<Box> a{det(0, 0)};
  tracker.update(5.0, &a);
  std::vector<Box> b{det(0, 0)};
  tracker.update(1.0, &b);
  EXPECT_NE(a[0].id, b[0].id);
  EXPECT_EQ(tracker.numTracks(), 1u);
}
