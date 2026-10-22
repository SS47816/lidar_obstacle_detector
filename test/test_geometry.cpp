#include <gtest/gtest.h>

#include <cmath>

#include "lidar_obstacle_detector/types.hpp"

using lidar_obstacle_detector::bevGIoU;
using lidar_obstacle_detector::bevIoU;
using lidar_obstacle_detector::Box;
using lidar_obstacle_detector::wrapAngle;

namespace {
Box box(float x, float y, float l, float w, float yaw) {
  return Box(0, Eigen::Vector3f(x, y, 0.f), Eigen::Vector3f(l, w, 1.f), yaw);
}
}  // namespace

TEST(Geometry, WrapAngle) {
  EXPECT_NEAR(wrapAngle(0.f), 0.f, 1e-6);
  EXPECT_NEAR(wrapAngle(3.f * M_PI), M_PI, 1e-5);
  EXPECT_NEAR(wrapAngle(-M_PI), M_PI, 1e-5);
  EXPECT_NEAR(wrapAngle(-0.5f * M_PI), -0.5f * M_PI, 1e-6);
}

TEST(Geometry, IoUIdenticalAndDisjoint) {
  EXPECT_NEAR(bevIoU(box(1, 2, 4, 2, 0.3f), box(1, 2, 4, 2, 0.3f)), 1.f, 1e-5);
  EXPECT_NEAR(bevIoU(box(0, 0, 1, 1, 0), box(5, 0, 1, 1, 0)), 0.f, 1e-6);
}

TEST(Geometry, IoUHalfOverlap) {
  // 2x2 squares shifted by 1 m: intersection 2, union 6
  EXPECT_NEAR(bevIoU(box(0, 0, 2, 2, 0), box(1, 0, 2, 2, 0)), 1.f / 3.f, 1e-5);
}

TEST(Geometry, IoUInvariantToSymmetries) {
  const Box a = box(0, 0, 4, 2, 0.f);
  // same footprint expressed with yaw + 90 deg and swapped length / width
  EXPECT_NEAR(bevIoU(a, box(0, 0, 2, 4, M_PI_2)), 1.f, 1e-5);
  EXPECT_NEAR(bevIoU(a, box(0, 0, 4, 2, M_PI)), 1.f, 1e-5);
}

TEST(Geometry, RotatedSquareInsideSquare) {
  // Unit square rotated 45 deg vs. axis-aligned unit square: IoU = 1/(4*sqrt2-3)... check
  // against the analytic intersection area 2*(sqrt2-1)
  const float inter = 2.f * (std::sqrt(2.f) - 1.f);
  EXPECT_NEAR(bevIoU(box(0, 0, 1, 1, 0), box(0, 0, 1, 1, M_PI / 4)), inter / (2.f - inter), 1e-4);
}

TEST(Geometry, GIoUPenalisesDistance) {
  const Box a = box(0, 0, 1, 1, 0);
  EXPECT_NEAR(bevGIoU(a, a), 1.f, 1e-5);
  const float near = bevGIoU(a, box(2, 0, 1, 1, 0));
  const float far = bevGIoU(a, box(5, 0, 1, 1, 0));
  EXPECT_LT(near, 0.f);
  EXPECT_LT(far, near);
  EXPECT_GE(far, -1.f);
  // disjoint unit squares 1 m apart: hull 3, union 2 -> -1/3
  EXPECT_NEAR(near, -1.f / 3.f, 1e-5);
}
