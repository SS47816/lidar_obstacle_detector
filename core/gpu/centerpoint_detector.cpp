/* centerpoint_detector.cpp

 * Copyright (C) 2021-2026 SS47816

 * LearnedDetector backed by NVIDIA CUDA-CenterPoint (third_party/cuda_centerpoint):
 * sparse-conv backbone (libspconv) + TensorRT head, trained on nuScenes.

**/

#include <cuda_runtime.h>

#include <sys/stat.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "NvInferVersion.h"
#include "centerpoint.h"
#include "lidar_obstacle_detector/learned_detector.hpp"

namespace lidar_obstacle_detector {

namespace {

// nuScenes LIDAR_TOP is mounted 1.84 m above the ground; the model expects that
constexpr float kModelSensorHeight = 1.84f;
constexpr unsigned int kFeatures = 5;  // x, y, z, intensity, time lag

ObjectClass toObjectClass(int nusc_class) {
  // Params::class_name order of CUDA-CenterPoint
  static const ObjectClass table[] = {
      ObjectClass::kCar,        ObjectClass::kTruck,   ObjectClass::kConstructionVehicle,
      ObjectClass::kBus,        ObjectClass::kTrailer, ObjectClass::kBarrier,
      ObjectClass::kMotorcycle, ObjectClass::kBicycle, ObjectClass::kPedestrian,
      ObjectClass::kTrafficCone};
  return nusc_class >= 0 && nusc_class < 10 ? table[nusc_class] : ObjectClass::kUnknown;
}

std::string defaultCacheDir() {
  if (const char *xdg = std::getenv("XDG_CACHE_HOME"))
    return std::string(xdg) + "/lidar_obstacle_detector";
  if (const char *home = std::getenv("HOME"))
    return std::string(home) + "/.cache/lidar_obstacle_detector";
  return "/tmp/lidar_obstacle_detector";
}

void makeDirs(const std::string &path) {
  for (size_t pos = path.find('/', 1); pos != std::string::npos; pos = path.find('/', pos + 1))
    mkdir(path.substr(0, pos).c_str(), 0755);
  mkdir(path.c_str(), 0755);
}

class CenterPointDetector : public LearnedDetector {
 public:
  CenterPointDetector(const DetectorConfig &cfg, float sensor_height)
      : score_threshold_(cfg.score_threshold), z_offset_(sensor_height - kModelSensorHeight) {
    // Installed models first, then the copy in the build tree (devel / plain builds)
    std::string model_dir = cfg.model_dir;
    if (model_dir.empty()) {
      struct stat st;
      model_dir = stat(LOD_CENTERPOINT_MODEL_DIR "/rpn_centerhead_sim.onnx", &st) == 0
                      ? LOD_CENTERPOINT_MODEL_DIR
                      : LOD_CENTERPOINT_MODEL_BUILD_DIR;
    }
    // The engine is specific to the GPU architecture and the TensorRT version
    cudaDeviceProp prop;
    if (cudaGetDeviceProperties(&prop, 0) != cudaSuccess)
      throw std::runtime_error("CenterPoint: no CUDA device available");
    const std::string cache = cfg.engine_cache.empty() ? defaultCacheDir() : cfg.engine_cache;
    makeDirs(cache);
    const std::string plan = cache + "/centerpoint_rpn_sm" + std::to_string(prop.major) +
                             std::to_string(prop.minor) + "_trt" +
                             std::to_string(NV_TENSORRT_MAJOR) + "." +
                             std::to_string(NV_TENSORRT_MINOR) + ".plan";
    if (cudaStreamCreate(&stream_) != cudaSuccess)
      throw std::runtime_error("CenterPoint: no CUDA stream");
    net_.reset(new CenterPoint(model_dir, plan, false));
    net_->prepare();
    if (cudaMalloc(&d_points_, MAX_POINTS_NUM * kFeatures * sizeof(float)) != cudaSuccess)
      throw std::runtime_error("CenterPoint: cudaMalloc failed");
  }

  ~CenterPointDetector() override {
    net_.reset();
    cudaFree(d_points_);
    cudaStreamDestroy(stream_);
  }

  std::string name() const override { return "centerpoint"; }

  std::vector<Box> detect(const std::vector<SweepPoint> &points) override {
    // Sensor frame (x forward, y left) -> nuScenes lidar frame (x right, y forward).
    // Points come newest first, so the cap drops the oldest scans.
    const size_t n = std::min<size_t>(points.size(), MAX_POINTS_NUM);
    host_.resize(n * kFeatures);
    for (size_t i = 0; i < n; ++i) {
      const auto &p = points[i];
      float *h = &host_[i * kFeatures];
      h[0] = -p.y;
      h[1] = p.x;
      h[2] = p.z + z_offset_;
      h[3] = p.intensity;
      h[4] = p.time_lag;
    }
    cudaMemcpyAsync(d_points_, host_.data(), host_.size() * sizeof(float), cudaMemcpyHostToDevice,
                    stream_);
    net_->doinfer(d_points_, static_cast<unsigned int>(n), stream_);

    std::vector<Box> boxes;
    boxes.reserve(net_->nms_pred_.size());
    for (const auto &b : net_->nms_pred_) {
      if (b.score < score_threshold_) continue;
      Box box;
      box.center = Eigen::Vector3f(b.y, -b.x, b.z - z_offset_);
      box.size = Eigen::Vector3f(b.l, b.w, b.h);  // length along the heading
      box.yaw = std::remainder(-b.rt - static_cast<float>(M_PI), 2.f * static_cast<float>(M_PI));
      box.heading_known = true;
      box.label = toObjectClass(b.id);
      box.score = b.score;
      boxes.push_back(box);
    }
    return boxes;
  }

 private:
  float score_threshold_, z_offset_;
  cudaStream_t stream_ = nullptr;
  std::unique_ptr<CenterPoint> net_;
  float *d_points_ = nullptr;
  std::vector<float> host_;
};

}  // namespace

std::unique_ptr<LearnedDetector> makeCenterPointDetector(const DetectorConfig &cfg,
                                                         float sensor_height) {
  return std::make_unique<CenterPointDetector>(cfg, sensor_height);
}

}  // namespace lidar_obstacle_detector
