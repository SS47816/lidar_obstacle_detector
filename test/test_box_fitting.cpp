#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "lidar_obstacle_detector/box_fitting.hpp"

using namespace lidar_obstacle_detector;

namespace {

constexpr float kDeg = static_cast<float>(M_PI) / 180.f;

// Orientation error of two rectangles, modulo 90 deg, in degrees
float yawErrorMod90(float a, float b) {
  float d = std::fmod(std::abs(a - b), static_cast<float>(M_PI_2));
  return std::min(d, static_cast<float>(M_PI_2) - d) / kDeg;
}

// The two sides of a car facing a sensor at the origin (an "L" in BEV)
pcl::PointCloud<pcl::PointXYZ> lShapedScan(float cx, float cy, float yaw, float l, float w,
                                           float noise, std::mt19937 *rng) {
  std::normal_distribution<float> n(0.f, noise);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const Eigen::Vector2f c(cx, cy), e1(std::cos(yaw), std::sin(yaw)), e2(-e1.y(), e1.x());
  // pick the corner closest to the sensor and sample the two edges meeting there
  float best = 1e9f;
  Eigen::Vector2f corner, s1, s2;
  for (int a : {-1, 1}) {
    for (int b : {-1, 1}) {
      const Eigen::Vector2f k = c + 0.5f * a * l * e1 + 0.5f * b * w * e2;
      if (k.norm() < best) {
        best = k.norm();
        corner = k;
        s1 = -a * e1 * l;
        s2 = -b * e2 * w;
      }
    }
  }
  for (float z : {-1.2f, -0.8f, -0.4f}) {
    for (float t = 0.f; t <= 1.f; t += 0.02f) {
      const Eigen::Vector2f p1 = corner + t * s1, p2 = corner + t * s2;
      cloud.push_back(pcl::PointXYZ(p1.x() + n(*rng), p1.y() + n(*rng), z));
      cloud.push_back(pcl::PointXYZ(p2.x() + n(*rng), p2.y() + n(*rng), z));
    }
  }
  return cloud;
}

pcl::Indices all(size_t n) {
  pcl::Indices idx(n);
  for (size_t i = 0; i < n; ++i) idx[i] = static_cast<int>(i);
  return idx;
}

}  // namespace

TEST(BoxFitting, AxisAligned) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({-1.f, -2.f, 0.f});
  cloud.push_back({3.f, 2.f, 1.f});
  cloud.push_back({0.f, 0.f, 0.5f});
  const Box b = fitAxisAlignedBox(cloud, all(cloud.size()));
  EXPECT_TRUE(b.center.isApprox(Eigen::Vector3f(1.f, 0.f, 0.5f)));
  EXPECT_TRUE(b.size.isApprox(Eigen::Vector3f(4.f, 4.f, 1.f)));
  EXPECT_EQ(b.yaw, 0.f);
}

TEST(BoxFitting, LShapeRecoversYawOfPartiallyVisibleCar) {
  std::mt19937 rng(7);
  for (float yaw_deg : {0.f, 12.f, 33.f, 45.f, 60.f, -25.f, 80.f}) {
    for (const auto &pos : {Eigen::Vector2f(10.f, 4.f), Eigen::Vector2f(-6.f, -15.f)}) {
      const float yaw = yaw_deg * kDeg;
      const auto cloud = lShapedScan(pos.x(), pos.y(), yaw, 4.5f, 1.8f, 0.02f, &rng);
      const Box b = fitLShapeBox(cloud, all(cloud.size()), 1.f);
      EXPECT_LT(yawErrorMod90(b.yaw, yaw), 2.f) << "yaw " << yaw_deg;
      EXPECT_NEAR(b.size.x(), 4.5f, 0.15f) << "yaw " << yaw_deg;
      EXPECT_NEAR(b.size.y(), 1.8f, 0.15f) << "yaw " << yaw_deg;
      EXPECT_LT((b.center.head<2>() - pos).norm(), 0.15f) << "yaw " << yaw_deg;
      EXPECT_NEAR(b.size.z(), 0.8f, 1e-4);
      EXPECT_GT(b.yaw, -M_PI_2 - 1e-5);
      EXPECT_LE(b.yaw, M_PI_2 + 1e-5);
    }
  }
}

TEST(BoxFitting, LShapeBeatsPcaOnLShapedScans) {
  std::mt19937 rng(3);
  float err_l = 0.f, err_pca = 0.f;
  for (float yaw_deg = -80.f; yaw_deg < 90.f; yaw_deg += 10.f) {
    const auto cloud = lShapedScan(12.f, 5.f, yaw_deg * kDeg, 4.5f, 1.8f, 0.03f, &rng);
    err_l += yawErrorMod90(fitLShapeBox(cloud, all(cloud.size())).yaw, yaw_deg * kDeg);
    err_pca += yawErrorMod90(fitPcaBox(cloud, all(cloud.size())).yaw, yaw_deg * kDeg);
  }
  EXPECT_LT(err_l, 0.25f * err_pca);
}

TEST(BoxFitting, PcaOnFullRectangleOutline) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const float yaw = 30.f * kDeg;
  const Eigen::Vector2f e1(std::cos(yaw), std::sin(yaw)), e2(-e1.y(), e1.x());
  for (float t = -0.5f; t <= 0.5f; t += 0.01f) {
    for (float s : {-0.5f, 0.5f}) {
      const Eigen::Vector2f a = 4.f * t * e1 + 1.6f * s * e2, b = 4.f * s * e1 + 1.6f * t * e2;
      cloud.push_back({a.x(), a.y(), 0.f});
      cloud.push_back({b.x(), b.y(), 1.f});
    }
  }
  const Box box = fitPcaBox(cloud, all(cloud.size()));
  EXPECT_LT(yawErrorMod90(box.yaw, yaw), 1.f);
  EXPECT_NEAR(box.size.x(), 4.f, 0.05f);
  EXPECT_NEAR(box.size.y(), 1.6f, 0.05f);
  EXPECT_NEAR(box.center.z(), 0.5f, 1e-5);  // the extent centre, not the centroid
}

TEST(BoxFitting, TinyClustersFallBackGracefully) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({1.f, 1.f, 0.f});
  const Box b = fitLShapeBox(cloud, all(1));
  EXPECT_TRUE(b.center.isApprox(Eigen::Vector3f(1.f, 1.f, 0.f)));
  EXPECT_TRUE(b.size.isZero());
}
