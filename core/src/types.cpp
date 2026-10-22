/* types.cpp

 * Copyright (C) 2021-2026 SS47816

 * BEV geometry helpers: rotated box IoU / GIoU via convex polygon clipping

**/

#include "lidar_obstacle_detector/types.hpp"

#include <algorithm>
#include <cmath>

namespace lidar_obstacle_detector {

namespace {

using Polygon = std::vector<Eigen::Vector2f>;

float cross(const Eigen::Vector2f &o, const Eigen::Vector2f &a,
            const Eigen::Vector2f &b) {
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

float area(const Polygon &poly) {
  float s = 0.f;
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto &p = poly[i];
    const auto &q = poly[(i + 1) % poly.size()];
    s += p.x() * q.y() - q.x() * p.y();
  }
  return 0.5f * std::abs(s);
}

// Sutherland-Hodgman clipping of `subject` by the convex CCW polygon `clip`
Polygon clipConvex(Polygon subject, const std::array<Eigen::Vector2f, 4> &clip) {
  for (size_t e = 0; e < clip.size() && !subject.empty(); ++e) {
    const auto &a = clip[e];
    const auto &b = clip[(e + 1) % clip.size()];
    Polygon out;
    out.reserve(subject.size() + 2);
    for (size_t i = 0; i < subject.size(); ++i) {
      const auto &p = subject[i];
      const auto &q = subject[(i + 1) % subject.size()];
      const float sp = cross(a, b, p);
      const float sq = cross(a, b, q);
      if (sp >= 0.f) out.push_back(p);
      if ((sp >= 0.f) != (sq >= 0.f)) out.push_back(p + (sp / (sp - sq)) * (q - p));
    }
    subject.swap(out);
  }
  return subject;
}

// Andrew's monotone chain convex hull
Polygon convexHull(Polygon pts) {
  std::sort(pts.begin(), pts.end(), [](const auto &a, const auto &b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  Polygon hull(2 * pts.size());
  size_t k = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0.f) --k;
    hull[k++] = pts[i];
  }
  for (size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i - 1]) <= 0.f) --k;
    hull[k++] = pts[i - 1];
  }
  hull.resize(k > 1 ? k - 1 : k);
  return hull;
}

float intersectionArea(const Box &a, const Box &b) {
  const auto ca = bevCorners(a);
  const auto cb = bevCorners(b);
  return area(clipConvex(Polygon(ca.begin(), ca.end()), cb));
}

}  // namespace

float wrapAngle(float a) {
  a = std::fmod(a + static_cast<float>(M_PI), 2.f * static_cast<float>(M_PI));
  if (a <= 0.f) a += 2.f * static_cast<float>(M_PI);
  return a - static_cast<float>(M_PI);
}

std::array<Eigen::Vector2f, 4> bevCorners(const Box &box) {
  const float c = std::cos(box.yaw), s = std::sin(box.yaw);
  const float l = 0.5f * box.size.x(), w = 0.5f * box.size.y();
  const Eigen::Vector2f ctr = box.center.head<2>();
  const Eigen::Vector2f ex(c, s), ey(-s, c);
  return {ctr + l * ex + w * ey, ctr - l * ex + w * ey, ctr - l * ex - w * ey,
          ctr + l * ex - w * ey};
}

float bevIoU(const Box &a, const Box &b) {
  const float inter = intersectionArea(a, b);
  const float uni = a.size.x() * a.size.y() + b.size.x() * b.size.y() - inter;
  return uni > 0.f ? inter / uni : 0.f;
}

float bevGIoU(const Box &a, const Box &b) {
  const float inter = intersectionArea(a, b);
  const float uni = a.size.x() * a.size.y() + b.size.x() * b.size.y() - inter;
  const auto ca = bevCorners(a);
  const auto cb = bevCorners(b);
  Polygon all(ca.begin(), ca.end());
  all.insert(all.end(), cb.begin(), cb.end());
  const float hull = area(convexHull(all));
  if (uni <= 0.f || hull <= 0.f) return -1.f;
  return inter / uni - (hull - uni) / hull;
}

}  // namespace lidar_obstacle_detector
