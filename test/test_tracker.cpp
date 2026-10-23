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
  // greedy / unweighted matching could swap these
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

TEST(Tracker, MovingTargetTakesHeadingFromMotion) {
  MultiObjectTracker tracker;
  const float course = 0.6f;  // [rad] direction of travel
  for (int k = 0; k < 15; ++k) {
    const float s = 8.f * 0.1f * k;
    // The box fitter only knows the yaw modulo 90 deg: report it rotated by 90
    std::vector<Box> boxes{det(s * std::cos(course), s * std::sin(course),
                               course + static_cast<float>(M_PI_2), 2.f, 4.5f)};
    tracker.update(0.1 * k, &boxes);
    if (k >= 8) {
      EXPECT_TRUE(boxes[0].heading_known);
      EXPECT_NEAR(boxes[0].yaw, course, 0.05f);    // front/back resolved too
      EXPECT_NEAR(boxes[0].size.x(), 4.5f, 1e-4);  // length along the motion
    }
  }
}

TEST(Tracker, ParkedTargetKeepsBoxYaw) {
  MultiObjectTracker tracker;
  for (int k = 0; k < 10; ++k) {
    std::vector<Box> boxes{det(10.f, 3.f, 0.3f, 4.5f, 1.8f)};
    tracker.update(0.1 * k, &boxes);
    EXPECT_FALSE(boxes[0].heading_known);
    EXPECT_NEAR(boxes[0].yaw, 0.3f, 1e-4);
  }
}

TEST(Tracker, ShapeCompletionKeepsFullSizeOfPartialView) {
  MultiObjectTracker tracker;
  // A parked car at x = 10..14.5, seen whole for a while
  for (int k = 0; k < 8; ++k) {
    std::vector<Box> boxes{det(12.25f, 0.f, 0.f, 4.5f, 1.8f)};
    tracker.update(0.1 * k, &boxes);
  }
  // Now only its near end (x = 10..11.5) is visible, e.g. occluded by another car
  std::vector<Box> partial{det(10.75f, 0.f, 0.f, 1.5f, 1.8f)};
  tracker.update(0.8, &partial);
  EXPECT_NEAR(partial[0].size.x(), 4.5f, 0.3f);
  // The face towards the sensor (x = 10) stays put; the box extends away from it
  EXPECT_NEAR(partial[0].center.x() - 0.5f * partial[0].size.x(), 10.f, 0.1f);

  TrackerConfig off;
  off.shape_completion = false;
  MultiObjectTracker plain(off);
  for (int k = 0; k < 8; ++k) {
    std::vector<Box> boxes{det(12.25f, 0.f, 0.f, 4.5f, 1.8f)};
    plain.update(0.1 * k, &boxes);
  }
  std::vector<Box> raw{det(10.75f, 0.f, 0.f, 1.5f, 1.8f)};
  plain.update(0.8, &raw);
  EXPECT_NEAR(raw[0].size.x(), 1.5f, 1e-4);
}

TEST(Tracker, PredictsTurningTargetWithConstantTurnRate) {
  // 10 m/s on a circle of radius 40 m (0.25 rad/s), detections every 0.1 s
  constexpr float kSpeed = 10.f, kRadius = 40.f, kRate = kSpeed / kRadius;
  auto pos = [&](float t) {
    return Eigen::Vector2f(kRadius * std::sin(kRate * t), kRadius * (1.f - std::cos(kRate * t)));
  };
  MultiObjectTracker tracker;
  std::vector<Box> boxes;
  float t = 0.f;
  for (int k = 0; k < 40; ++k) {
    t = 0.1f * k;
    const Eigen::Vector2f p = pos(t);
    boxes = {det(p.x(), p.y(), kRate * t)};
    tracker.update(t, &boxes);
  }
  const auto &path = boxes[0].predicted_path;
  ASSERT_EQ(path.size(), 6u);  // 3 s at 0.5 s
  EXPECT_NEAR(path.back().t, 3.f, 1e-4);
  const Eigen::Vector2f truth = pos(t + 3.f);
  // vs. extrapolating the same filtered velocity in a straight line
  const Eigen::Vector2f straight = boxes[0].center.head<2>() + 3.f * boxes[0].velocity;
  const float err = (path.back().position.head<2>() - truth).norm();
  EXPECT_LT(err, 4.f);
  EXPECT_LT(err, 0.4f * (straight - truth).norm());
}

TEST(Tracker, NoPredictionForStationaryObjects) {
  MultiObjectTracker tracker;
  std::vector<Box> boxes;
  for (int k = 0; k < 10; ++k) {
    boxes = {det(5.f, 5.f)};
    tracker.update(0.1 * k, &boxes);
  }
  EXPECT_TRUE(boxes[0].tracked);
  EXPECT_TRUE(boxes[0].predicted_path.empty());
}
