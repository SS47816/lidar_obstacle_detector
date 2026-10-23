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
constexpr float kMaxHeadingStd = 10.f * kPi / 180.f;       // velocity heading uncertainty
constexpr float kHeadingConsistency = 20.f * kPi / 180.f;  // frame-to-frame agreement
constexpr int kMinHeadingStreak = 2;
// Shape completion: only for objects up to roughly vehicle size
constexpr float kMaxCompletedLength = 16.f, kMaxCompletedWidth = 4.f;
constexpr float kExtentGrow = 0.3f, kExtentShrink = 0.05f;

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

// Coarse class groups: a track never switches between them
int classGroup(ObjectClass c) {
  switch (c) {
    case ObjectClass::kCar:
    case ObjectClass::kTruck:
    case ObjectClass::kBus:
    case ObjectClass::kTrailer:
    case ObjectClass::kConstructionVehicle:
      return 1;
    case ObjectClass::kMotorcycle:
    case ObjectClass::kBicycle:
      return 2;
    case ObjectClass::kPedestrian:
      return 3;
    case ObjectClass::kBarrier:
    case ObjectClass::kTrafficCone:
      return 4;
    case ObjectClass::kUnknown:
    default:
      return 0;
  }
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
  const int gt = classGroup(t.label), gd = classGroup(det.label);
  if (gt != 0 && gd != 0 && gt != gd) return static_cast<float>(kInvalid);
  if (cfg_.cost == TrackCost::kCenterDistance) return dist;
  const Box predicted(t.id, Eigen::Vector3f(t.x(0), t.x(1), t.z), t.size, t.yaw);
  const float giou = bevGIoU(predicted, det);
  return giou < cfg_.min_giou ? static_cast<float>(kInvalid) : 1.f - giou;
}

// Direction of travel of a confirmed track, if it moves fast enough and its
// velocity estimate is certain and consistent over consecutive frames.
// Otherwise the box keeps the L-shape yaw (stationary, slow, new or jittery
// tracks); `hint` still reports a usable direction of travel for tracks that
// move but are not yet stable, so at least front / back can be resolved.
bool MultiObjectTracker::motionHeading(Track *t, float *heading, bool *hint) const {
  *hint = false;
  const Eigen::Vector2f v = t->x.tail<2>();
  const float speed = v.norm();
  if (speed < cfg_.heading_min_speed || t->hits < 2) {
    t->heading_streak = 0;
    return false;
  }
  const float h = std::atan2(v.y(), v.x());
  *hint = cfg_.heading_from_motion;
  // Heading std from the velocity covariance, perpendicular to the motion
  const Eigen::Vector2f n(-v.y() / speed, v.x() / speed);
  const Eigen::Matrix2f Pv = t->P.bottomRightCorner<2, 2>();
  const float heading_std = std::sqrt(std::max(0.f, n.dot(Pv * n))) / speed;
  const bool consistent =
      t->heading_streak > 0 && std::abs(wrapAngle(h - t->motion_heading)) < kHeadingConsistency;
  t->heading_streak = heading_std < kMaxHeadingStd ? (consistent ? t->heading_streak + 1 : 1) : 0;
  t->motion_heading = h;
  *heading = h;
  return cfg_.heading_from_motion && t->hits + 1 >= 3 && t->heading_streak >= kMinHeadingStreak;
}

// Partially visible objects look smaller than they are and their box centre
// sits on the visible part. Keep the face towards the sensor fixed and extend
// the box away from it to the size remembered for the track.
void MultiObjectTracker::completeShape(const Track &t, const Eigen::Vector2f &sensor_xy,
                                       Box *det) const {
  const Eigen::Vector2f e1(std::cos(det->yaw), std::sin(det->yaw)), e2(-e1.y(), e1.x());
  const Eigen::Vector2f to_sensor = sensor_xy - det->center.head<2>();
  Eigen::Vector2f c = det->center.head<2>();
  for (int axis = 0; axis < 2; ++axis) {
    const float seen = det->size[axis], full = t.extent[axis];
    if (full <= seen + 0.2f) continue;
    const Eigen::Vector2f e = axis == 0 ? e1 : e2;
    const float towards = to_sensor.dot(e) >= 0.f ? 1.f : -1.f;
    c -= towards * 0.5f * (full - seen) * e;
    det->size[axis] = full;
  }
  det->center.head<2>() = c;
}

void MultiObjectTracker::correct(Track *t, Box *det, double stamp,
                                 const Eigen::Vector2f &sensor_xy) const {
  // Orientation: travel direction for moving, stable tracks, otherwise the
  // box-fitting yaw made consistent with the track (90 / 180 deg ambiguity)
  float heading = 0.f;
  bool hint = false;
  bool from_motion = motionHeading(t, &heading, &hint);
  // Learned detections come with a full heading and full extents: keep them
  const bool learned = det->label != ObjectClass::kUnknown;
  if (learned) {
    from_motion = false;
    det->heading_known = true;
    // ...except that a moving object drives forwards: fix a flipped heading
    if (hint && std::abs(wrapAngle(det->yaw - heading)) > 0.5f * kPi)
      det->yaw = wrapAngle(det->yaw + kPi);
    t->yaw = det->yaw;
    t->extent = det->size.head<2>();
  } else if (from_motion) {
    // If the reference orientation turns by an odd multiple of 90 deg, length
    // and width swap roles: keep the size memory consistent with the new axes
    const int quarter_turns =
        static_cast<int>(std::lround(wrapAngle(heading - t->yaw) / (0.5f * kPi)));
    if (quarter_turns % 2 != 0) {
      std::swap(t->extent.x(), t->extent.y());
      std::swap(t->size.x(), t->size.y());
    }
    alignYaw(heading, det);  // length axis along the motion
    det->yaw = heading;
    det->heading_known = true;
  } else {
    // Box-fitting yaw; if the object moves, at least point it forwards
    if (hint && std::abs(wrapAngle(t->yaw - heading)) > 0.5f * kPi)
      t->yaw = wrapAngle(t->yaw + kPi);
    alignYaw(t->yaw, det);
  }

  // Size memory (grows fast, shrinks slowly) and completion of partial views
  const bool stable =
      from_motion || (t->hits >= 5 && std::abs(wrapAngle(det->yaw - t->yaw)) < 0.17f);
  for (int axis = 0; axis < 2; ++axis) {
    const float d = det->size[axis] - t->extent[axis];
    t->extent[axis] += (d > 0.f ? kExtentGrow : kExtentShrink) * d;
  }
  t->extent.x() = std::min(t->extent.x(), kMaxCompletedLength);
  t->extent.y() = std::min(t->extent.y(), kMaxCompletedWidth);
  // The filter tracks the measured centre (what association compares against);
  // completion only changes the reported box
  const Eigen::Vector2f measured = det->center.head<2>();
  if (cfg_.shape_completion && stable && !learned && t->hits + 1 >= cfg_.min_hits)
    completeShape(*t, sensor_xy, det);

  // Kalman update
  const float r = cfg_.meas_noise * cfg_.meas_noise;
  const Eigen::Vector2f innovation = measured - t->x.head<2>();
  const Eigen::Matrix2f S = t->P.topLeftCorner<2, 2>() + r * Eigen::Matrix2f::Identity();
  const Eigen::Matrix<float, 4, 2> K = t->P.leftCols<2>() * S.inverse();
  t->x += K * innovation;
  t->P = (Eigen::Matrix4f::Identity() - K * Eigen::Matrix<float, 2, 4>::Identity()) * t->P;

  const float a = cfg_.size_smoothing;
  if (!learned)
    t->yaw = from_motion ? heading : wrapAngle(t->yaw + a * wrapAngle(det->yaw - t->yaw));
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
  t.extent = det.size.head<2>();
  t.yaw = det.yaw;
  t.motion_heading = 0.f;
  t.heading_streak = 0;
  t.label = det.label;
  t.z = det.center.z();
  t.hits = 1;
  t.last_update = stamp;
  return t;
}

void MultiObjectTracker::update(double stamp, std::vector<Box> *boxes,
                                const Eigen::Vector2f &sensor_xy) {
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
    Track *t = nullptr;
    if (det_to_track[j] >= 0) {
      Track &track = tracks_[det_to_track[j]];
      matched[det_to_track[j]] = true;
      correct(&track, &det, stamp, sensor_xy);
      t = &track;
    } else {
      born.push_back(spawn(det, stamp));
      t = &born.back();
    }
    // The class sticks to the track (e.g. a car later seen only as a cluster)
    if (det.label != ObjectClass::kUnknown)
      t->label = det.label;
    else
      det.label = t->label;
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
