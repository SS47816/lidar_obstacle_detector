/* tracker.cpp

 * Copyright (C) 2021-2026 SS47816

 * Multi-object tracker implementation

**/

#include "lidar_obstacle_detector/tracker.hpp"

#include <algorithm>
#include <cmath>

#include "lidar_obstacle_detector/hungarian.hpp"

namespace lidar_obstacle_detector {

namespace {

constexpr float kPi = static_cast<float>(M_PI);
constexpr double kInvalid = 1e6;  // cost of gated-out pairs
constexpr float kMaxDt = 1.f;     // [s] clamp for gaps in the input stream
constexpr float kInitVelStd = 10.f;

// Rotate the detection by a multiple of 90 deg (swapping length / width for odd
// multiples) so its yaw is as close as possible to the reference yaw. Box
// fitting cannot tell length from width or front from back.
void alignYaw(float reference, Box *det) {
  float best = det->yaw, best_err = 10.f;
  int best_k = 0;
  for (int k = 0; k < 4; ++k) {
    const float yaw = wrapAngle(det->yaw + k * 0.5f * kPi);
    const float err = std::abs(wrapAngle(yaw - reference));
    if (err < best_err) {
      best_err = err;
      best = yaw;
      best_k = k;
    }
  }
  if (best_k % 2 == 1) std::swap(det->size.x(), det->size.y());
  det->yaw = best;
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(const TrackerConfig &cfg) : cfg_(cfg) {}

void MultiObjectTracker::reset() {
  tracks_.clear();
  initialized_ = false;
}

void MultiObjectTracker::predict(Track *t, float dt) const {
  Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
  F(0, 2) = F(1, 3) = dt;
  // Discrete white-noise acceleration model
  const float q = cfg_.accel_noise * cfg_.accel_noise;
  const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
  Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
  Q(0, 0) = Q(1, 1) = 0.25f * dt4 * q;
  Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = 0.5f * dt3 * q;
  Q(2, 2) = Q(3, 3) = dt2 * q;
  t->x = F * t->x;
  t->P = F * t->P * F.transpose() + Q;
}

float MultiObjectTracker::cost(const Track &t, const Box &det) const {
  const float dist = (t.x.head<2>() - det.center.head<2>()).norm();
  if (dist > cfg_.max_distance) return static_cast<float>(kInvalid);
  if (cfg_.cost == TrackCost::kCenterDistance) return dist;
  const Box predicted(t.id, Eigen::Vector3f(t.x(0), t.x(1), t.z), t.size, t.yaw);
  const float giou = bevGIoU(predicted, det);
  return giou < cfg_.min_giou ? static_cast<float>(kInvalid) : 1.f - giou;
}

void MultiObjectTracker::correct(Track *t, Box *det, double stamp) const {
  // Kalman update with the measured centre
  const float r = cfg_.meas_noise * cfg_.meas_noise;
  const Eigen::Vector2f innovation = det->center.head<2>() - t->x.head<2>();
  const Eigen::Matrix2f S = t->P.topLeftCorner<2, 2>() + r * Eigen::Matrix2f::Identity();
  const Eigen::Matrix<float, 4, 2> K = t->P.leftCols<2>() * S.inverse();
  t->x += K * innovation;
  t->P = (Eigen::Matrix4f::Identity() - K * Eigen::Matrix<float, 2, 4>::Identity()) * t->P;

  // Orientation and size
  alignYaw(t->yaw, det);
  const float a = cfg_.size_smoothing;
  t->yaw = wrapAngle(t->yaw + a * wrapAngle(det->yaw - t->yaw));
  t->size = (1.f - a) * t->size + a * det->size;
  t->z = det->center.z();
  t->hits++;
  t->last_update = stamp;
}

MultiObjectTracker::Track MultiObjectTracker::spawn(const Box &det, double stamp) {
  Track t;
  t.id = next_id_++;
  t.x << det.center.x(), det.center.y(), 0.f, 0.f;
  t.P = Eigen::Matrix4f::Zero();
  t.P(0, 0) = t.P(1, 1) = cfg_.meas_noise * cfg_.meas_noise;
  t.P(2, 2) = t.P(3, 3) = kInitVelStd * kInitVelStd;
  t.size = det.size;
  t.yaw = det.yaw;
  t.z = det.center.z();
  t.hits = 1;
  t.last_update = stamp;
  return t;
}

void MultiObjectTracker::update(double stamp, std::vector<Box> *boxes) {
  const float dt =
      initialized_ ? std::clamp(static_cast<float>(stamp - last_stamp_), 0.f, kMaxDt) : 0.f;
  if (initialized_ && stamp < last_stamp_) reset();  // time jumped back (e.g. bag loop)
  initialized_ = true;
  last_stamp_ = stamp;

  for (auto &t : tracks_) predict(&t, dt);

  // Optimal assignment of detections to predicted tracks
  std::vector<int> det_to_track(boxes->size(), -1);
  if (!tracks_.empty() && !boxes->empty()) {
    Eigen::MatrixXd c(tracks_.size(), boxes->size());
    for (size_t i = 0; i < tracks_.size(); ++i)
      for (size_t j = 0; j < boxes->size(); ++j) c(i, j) = cost(tracks_[i], (*boxes)[j]);
    const std::vector<int> assignment = solveAssignment(c);
    for (size_t i = 0; i < assignment.size(); ++i) {
      const int j = assignment[i];
      if (j >= 0 && c(i, j) < kInvalid) det_to_track[j] = static_cast<int>(i);
    }
  }

  std::vector<bool> matched(tracks_.size(), false);
  std::vector<Track> born;
  for (size_t j = 0; j < boxes->size(); ++j) {
    Box &det = (*boxes)[j];
    const Track *t = nullptr;
    if (det_to_track[j] >= 0) {
      Track &track = tracks_[det_to_track[j]];
      matched[det_to_track[j]] = true;
      correct(&track, &det, stamp);
      t = &track;
    } else {
      born.push_back(spawn(det, stamp));
      t = &born.back();
    }
    det.id = t->id;
    det.age = static_cast<uint32_t>(t->hits);
    det.tracked = t->hits >= cfg_.min_hits;
    det.has_velocity = t->hits >= 2;
    det.velocity = det.has_velocity ? Eigen::Vector2f(t->x.tail<2>()) : Eigen::Vector2f::Zero();
  }

  // Drop tentative tracks that missed and confirmed tracks lost for too long
  std::vector<Track> alive;
  alive.reserve(tracks_.size() + born.size());
  for (size_t i = 0; i < tracks_.size(); ++i) {
    const Track &t = tracks_[i];
    const bool confirmed = t.hits >= cfg_.min_hits;
    if (matched[i] || (confirmed && stamp - t.last_update <= cfg_.max_age)) alive.push_back(t);
  }
  alive.insert(alive.end(), born.begin(), born.end());
  tracks_.swap(alive);
}

}  // namespace lidar_obstacle_detector
