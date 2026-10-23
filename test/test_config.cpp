#include <gtest/gtest.h>

#include "lidar_obstacle_detector/config.hpp"

using namespace lidar_obstacle_detector;

TEST(Config, Presets) {
  EXPECT_EQ(PipelineConfig::preset("fast").cluster.method, ClusterMethod::kBevGrid);
  EXPECT_EQ(PipelineConfig::preset("fast").ground.method, GroundMethod::kPatchworkpp);
  EXPECT_EQ(PipelineConfig::preset("accurate").ground.method, GroundMethod::kPatchworkpp);
  EXPECT_EQ(PipelineConfig::preset("legacy").box.method, BoxMethod::kAabb);
  EXPECT_EQ(PipelineConfig::preset("fast").tracker.cost, TrackCost::kGIoU);
  EXPECT_THROW(PipelineConfig::preset("nope"), std::invalid_argument);
}

TEST(Config, SetAndGetByName) {
  PipelineConfig c;
  setParameter(&c, "cluster_threshold", "0.8");
  setParameter(&c, "box_method", "lshape");
  setParameter(&c, "use_tracking", "false");
  setParameter(&c, "cluster_min_size", "4");
  EXPECT_FLOAT_EQ(c.cluster.tolerance, 0.8f);
  EXPECT_EQ(c.box.method, BoxMethod::kLShape);
  EXPECT_FALSE(c.tracker.enabled);
  EXPECT_EQ(c.cluster.min_size, 4);
  EXPECT_EQ(getParameter(c, "box_method"), "lshape");
  EXPECT_EQ(getParameter(c, "cluster_threshold"), "0.8");
}

TEST(Config, RejectsBadValues) {
  PipelineConfig c;
  EXPECT_THROW(setParameter(&c, "no_such_param", "1"), std::invalid_argument);
  EXPECT_THROW(setParameter(&c, "cluster_threshold", "abc"), std::invalid_argument);
  EXPECT_THROW(setParameter(&c, "cluster_threshold", "100"), std::invalid_argument);
  EXPECT_THROW(setParameter(&c, "cluster_min_size", "2.5"), std::invalid_argument);
  EXPECT_THROW(setParameter(&c, "box_method", "obb"), std::invalid_argument);
  EXPECT_THROW(setParameter(&c, "use_tracking", "yes"), std::invalid_argument);
}

TEST(Config, EveryParameterRoundTrips) {
  for (const auto &preset : {"legacy", "fast", "accurate"}) {
    const PipelineConfig src = PipelineConfig::preset(preset);
    PipelineConfig dst;
    for (const auto &p : parameterTable()) {
      const std::string v = getParameter(src, p.name);
      ASSERT_NO_THROW(setParameter(&dst, p.name, v)) << p.name << "=" << v;
      EXPECT_EQ(getParameter(dst, p.name), v) << p.name;
    }
  }
}

TEST(Config, DetectorModeAliases) {
  PipelineConfig cfg;
  setParameter(&cfg, "detector_method", "centerpoint");
  EXPECT_EQ(cfg.detector.method, DetectorMethod::kLearning);
  EXPECT_EQ(getParameter(cfg, "detector_method"), "learning");
  setParameter(&cfg, "detector_method", "hybrid");
  EXPECT_EQ(getParameter(cfg, "detector_method"), "fused");
  const auto &table = parameterTable();
  const auto it = std::find_if(table.begin(), table.end(),
                               [](const ParamInfo &p) { return p.name == "detector_method"; });
  ASSERT_NE(it, table.end());
  EXPECT_EQ(it->choices, (std::vector<std::string>{"clustering", "learning", "fused"}));
}
