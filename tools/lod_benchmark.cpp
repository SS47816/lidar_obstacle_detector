/* lod_benchmark.cpp

 * Copyright (C) 2021-2026 SS47816

 * Offline benchmark runner: feeds a sequence in the benchmark format (see
 * tools/python/lod_bench/common.py) through a pipeline and writes
 * results.jsonl (boxes, track ids, per-stage timings) for evaluate.py.

**/

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "benchmark_types.hpp"
#include "core_adapter.hpp"

namespace lod = lidar_obstacle_detector::bench;
using json = nlohmann::json;

namespace {

void usage() {
  std::cerr << "Usage: lod_benchmark --seq DIR --out DIR [options]\n"
               "  --preset NAME          preset: legacy|fast|accurate (default fast)\n"
               "  --config FILE          key=value parameter file (# comments)\n"
               "  --set key=value        parameter override (repeatable)\n"
               "  --dump-every N         dump processed clouds every N-th annotated "
               "frame (0=off)\n"
               "  --max-frames N         stop after N frames\n";
}

void parseKeyValue(const std::string &kv, lod::ParamMap *params) {
  const auto pos = kv.find('=');
  if (pos == std::string::npos) throw std::invalid_argument("expected key=value: " + kv);
  auto trim = [](std::string s) {
    const auto b = s.find_first_not_of(" \t");
    const auto e = s.find_last_not_of(" \t\r");
    return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
  };
  (*params)[trim(kv.substr(0, pos))] = trim(kv.substr(pos + 1));
}

void readConfigFile(const std::string &path, lod::ParamMap *params) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open config " + path);
  std::string line;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    parseKeyValue(line, params);
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadBin(const std::string &path) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) throw std::runtime_error("cannot open " + path);
  const std::streamsize bytes = in.tellg();
  in.seekg(0);
  std::vector<float> buf(bytes / sizeof(float));
  in.read(reinterpret_cast<char *>(buf.data()), bytes);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  const size_t n = buf.size() / 4;
  cloud->resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto &p = cloud->points[i];
    p.x = buf[4 * i];
    p.y = buf[4 * i + 1];
    p.z = buf[4 * i + 2];
    p.intensity = buf[4 * i + 3];
  }
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void dumpCloud(const std::string &path, const std::vector<Eigen::Vector3f> &pts) {
  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<const char *>(pts.data()),
            static_cast<std::streamsize>(pts.size() * sizeof(Eigen::Vector3f)));
}

}  // namespace

int main(int argc, char **argv) {
  std::string seq_dir, out_dir, preset = "fast";
  int dump_every = 0;
  long max_frames = -1;
  lod::ParamMap params;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw std::invalid_argument("missing value for " + a);
        return argv[++i];
      };
      if (a == "--seq")
        seq_dir = next();
      else if (a == "--out")
        out_dir = next();
      else if (a == "--preset")
        preset = next();
      else if (a == "--config")
        readConfigFile(next(), &params);
      else if (a == "--set")
        parseKeyValue(next(), &params);
      else if (a == "--dump-every")
        dump_every = std::stoi(next());
      else if (a == "--max-frames")
        max_frames = std::stol(next());
      else if (a == "-h" || a == "--help") {
        usage();
        return 0;
      } else
        throw std::invalid_argument("unknown argument " + a);
    }
    if (seq_dir.empty() || out_dir.empty()) {
      usage();
      return 2;
    }

    auto adapter = std::make_unique<lod::CoreAdapter>(preset, params);

    if (std::system(("mkdir -p '" + out_dir + "/clouds'").c_str()) != 0)
      throw std::runtime_error("cannot create " + out_dir);
    std::ifstream frames(seq_dir + "/frames.jsonl");
    if (!frames) throw std::runtime_error("cannot open frames.jsonl in " + seq_dir);
    std::ofstream results(out_dir + "/results.jsonl");
    results << std::setprecision(6);

    std::string line;
    long n = 0, annotated = 0;
    while (std::getline(frames, line) && (max_frames < 0 || n < max_frames)) {
      const json f = json::parse(line);
      const int idx = f["idx"];
      std::ostringstream name;
      name << std::setw(6) << std::setfill('0') << idx;
      const auto cloud = loadBin(seq_dir + "/points/" + name.str() + ".bin");

      Eigen::Matrix4d m;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) m(r, c) = f["pose"][r * 4 + c];
      Eigen::Isometry3d pose(m);

      const auto res = adapter->process(cloud, f["t"].get<double>(), pose);

      json rec;
      rec["idx"] = idx;
      rec["n_points"] = cloud->size();
      json boxes = json::array();
      for (const auto &b : res.boxes) {
        json jb = {{"id", b.id},
                   {"center", {b.center.x(), b.center.y(), b.center.z()}},
                   {"size", {b.size.x(), b.size.y(), b.size.z()}},
                   {"yaw", b.yaw}};
        if (b.has_velocity) jb["velocity"] = {b.velocity.x(), b.velocity.y()};
        jb["label"] = b.label;
        jb["score"] = b.score;
        if (!b.path.empty()) {
          json path = json::array();
          for (const auto &p : b.path) path.push_back({p.x(), p.y()});
          jb["path"] = path;
        }
        boxes.push_back(jb);
      }
      rec["boxes"] = boxes;
      json structures = json::array();
      for (const auto &b : res.structures)
        structures.push_back({{"center", {b.center.x(), b.center.y(), b.center.z()}},
                              {"size", {b.size.x(), b.size.y(), b.size.z()}},
                              {"yaw", b.yaw}});
      rec["structures"] = structures;
      json timing = json::object();
      for (const auto &t : res.timing_ms) timing[t.first] = t.second;
      rec["timing_ms"] = timing;

      if (f["annotated"].get<bool>()) {
        // Pillars of every annotated frame (int16: x, y, z_min, z_max [cm], observed)
        if (!res.pillars.empty()) {
          std::vector<int16_t> buf;
          buf.reserve(res.pillars.size() * 5);
          for (const auto &p : res.pillars)
            for (int k = 0; k < 5; ++k)
              buf.push_back(static_cast<int16_t>(std::lround(k < 4 ? 100.f * p[k] : p[k])));
          std::ofstream pf(out_dir + "/clouds/" + name.str() + "_pillars.bin", std::ios::binary);
          pf.write(reinterpret_cast<const char *>(buf.data()),
                   static_cast<std::streamsize>(buf.size() * sizeof(int16_t)));
          rec["pillars"] = res.pillars.size();
        }
        if (dump_every > 0 && annotated % dump_every == 0) {
          dumpCloud(out_dir + "/clouds/" + name.str() + "_ground.bin", res.ground);
          dumpCloud(out_dir + "/clouds/" + name.str() + "_obstacle.bin", res.obstacles);
          rec["dumped"] = true;
          if (!res.occupancy.empty()) {
            std::ofstream occ(out_dir + "/clouds/" + name.str() + "_occupancy.bin",
                              std::ios::binary);
            occ.write(reinterpret_cast<const char *>(res.occupancy.data()),
                      static_cast<std::streamsize>(res.occupancy.size()));
            rec["occupancy"] = {{"resolution", res.occ_resolution},
                                {"origin", {res.occ_origin_x, res.occ_origin_y}},
                                {"width", res.occ_width},
                                {"height", res.occ_height}};
          }
        }
        ++annotated;
      }
      results << rec.dump() << "\n";
      ++n;
    }
    std::cerr << "processed " << n << " frames of " << seq_dir << "\n";
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
