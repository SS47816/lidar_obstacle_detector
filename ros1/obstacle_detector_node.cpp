/* obstacle_detector_node.cpp (ROS 1)

 * Copyright (C) 2021-2026 SS47816

 * ROS 1 node wrapping the ROS-agnostic obstacle detection pipeline. Topics
 * are set by parameters; all algorithm parameters are exposed through
 * dynamic_reconfigure.

**/

#include <dynamic_reconfigure/server.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <lidar_obstacle_detector/obstacle_detectorConfig.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <boost/any.hpp>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#ifdef LOD_HAVE_AUTOWARE
#include <autoware_msgs/DetectedObjectArray.h>
#endif

#include "lidar_obstacle_detector/obstacle_detector.hpp"

namespace lidar_obstacle_detector {

namespace {

using PointT = pcl::PointXYZI;
using Config = lidar_obstacle_detector::obstacle_detectorConfig;

std::string anyToString(const boost::any &v) {
  std::ostringstream ss;
  ss.precision(10);
  if (v.type() == typeid(bool)) return boost::any_cast<bool>(v) ? "true" : "false";
  if (v.type() == typeid(int)) return std::to_string(boost::any_cast<int>(v));
  if (v.type() == typeid(double)) {
    ss << boost::any_cast<double>(v);
    return ss.str();
  }
  if (v.type() == typeid(std::string)) return boost::any_cast<std::string>(v);
  throw std::invalid_argument("unsupported dynamic_reconfigure type");
}

geometry_msgs::Pose toPose(const Eigen::Vector3f &center, float yaw) {
  geometry_msgs::Pose pose;
  pose.position.x = center.x();
  pose.position.y = center.y();
  pose.position.z = center.z();
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

std_msgs::ColorRGBA rgba(float r, float g, float b, float a) {
  std_msgs::ColorRGBA c;
  c.r = r, c.g = g, c.b = b, c.a = a;
  return c;
}

std_msgs::ColorRGBA classColor(ObjectClass c) {
  switch (c) {
    case ObjectClass::kCar:
      return rgba(0.25f, 0.62f, 1.f, 0.9f);
    case ObjectClass::kTruck:
    case ObjectClass::kConstructionVehicle:
    case ObjectClass::kTrailer:
      return rgba(1.f, 0.6f, 0.15f, 0.9f);
    case ObjectClass::kBus:
      return rgba(1.f, 0.82f, 0.2f, 0.9f);
    case ObjectClass::kPedestrian:
      return rgba(1.f, 0.35f, 0.45f, 0.9f);
    case ObjectClass::kMotorcycle:
    case ObjectClass::kBicycle:
      return rgba(0.3f, 0.9f, 0.5f, 0.9f);
    default:
      return rgba(0.85f, 0.85f, 0.85f, 0.9f);
  }
}

geometry_msgs::Point toPoint(const Eigen::Vector3f &p) {
  geometry_msgs::Point q;
  q.x = p.x(), q.y = p.y(), q.z = p.z();
  return q;
}

}  // namespace

class ObstacleDetectorNode {
 public:
  ObstacleDetectorNode() : private_nh_("~"), tf_listener_(tf_buffer_), reconfigure_(private_nh_) {
    // Topic parameters (names kept from the original ROS 1 node)
    std::string lidar_points_topic, cloud_ground_topic, cloud_clusters_topic, jsk_bboxes_topic,
        autoware_objects_topic, markers_topic;
    private_nh_.param<std::string>("lidar_points_topic", lidar_points_topic, "/points");
    private_nh_.param<std::string>("cloud_ground_topic", cloud_ground_topic,
                                   "obstacle_detector/cloud_ground");
    private_nh_.param<std::string>("cloud_clusters_topic", cloud_clusters_topic,
                                   "obstacle_detector/cloud_clusters");
    private_nh_.param<std::string>("jsk_bboxes_topic", jsk_bboxes_topic,
                                   "obstacle_detector/jsk_bboxes");
    private_nh_.param<std::string>("autoware_objects_topic", autoware_objects_topic,
                                   "obstacle_detector/objects");
    private_nh_.param<std::string>("markers_topic", markers_topic, "obstacle_detector/markers");
    std::string occupancy_topic, voxels_topic, pillars_topic;
    private_nh_.param<std::string>("pillars_topic", pillars_topic, "obstacle_detector/pillars");
    private_nh_.param<std::string>("occupancy_topic", occupancy_topic,
                                   "obstacle_detector/occupancy");
    private_nh_.param<std::string>("occupancy_voxels_topic", voxels_topic,
                                   "obstacle_detector/occupancy_voxels");
    private_nh_.param<std::string>("bbox_target_frame", target_frame_, "");
    private_nh_.param<std::string>("tracking_frame", tracking_frame_, "");
    private_nh_.param<double>("tf_timeout", tf_timeout_, 0.1);
    private_nh_.param<std::string>("mesh_uri", mesh_uri_,
                                   "package://lidar_obstacle_detector/meshes/");
    std::string preset;
    private_nh_.param<std::string>("preset", preset, "fast");

    // Seed dynamic_reconfigure with the preset for every parameter that the
    // user did not set explicitly (e.g. in a launch file)
    const PipelineConfig config = PipelineConfig::preset(preset);
    for (const auto &p : parameterTable()) {
      if (private_nh_.hasParam(p.name)) continue;
      const std::string v = getParameter(config, p.name);
      switch (p.type) {
        case ParamInfo::Type::kBool:
          private_nh_.setParam(p.name, v == "true");
          break;
        case ParamInfo::Type::kInt:
          private_nh_.setParam(p.name, std::stoi(v));
          break;
        case ParamInfo::Type::kDouble:
          private_nh_.setParam(p.name, std::stod(v));
          break;
        case ParamInfo::Type::kString:
          private_nh_.setParam(p.name, v);
          break;
      }
    }
    detector_ = std::make_unique<ObstacleDetector<PointT>>(config);
    reconfigure_.setCallback([this](const Config &c, uint32_t) { onReconfigure(c); });

    pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_ground_topic, 1);
    pub_obstacles_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_clusters_topic, 1);
    pub_occupancy_ = nh_.advertise<nav_msgs::OccupancyGrid>(occupancy_topic, 1);
    pub_voxels_ = nh_.advertise<sensor_msgs::PointCloud2>(voxels_topic, 1);
    pub_pillars_ = nh_.advertise<sensor_msgs::PointCloud2>(pillars_topic, 1);
    pub_jsk_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(jsk_bboxes_topic, 1);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(markers_topic, 1);
#ifdef LOD_HAVE_AUTOWARE
    pub_autoware_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(autoware_objects_topic, 1);
#else
    if (!autoware_objects_topic.empty())
      ROS_INFO("Built without autoware_msgs: '%s' is not published",
               autoware_objects_topic.c_str());
#endif
    sub_points_ = nh_.subscribe(lidar_points_topic, 1, &ObstacleDetectorNode::onPoints, this);
    ROS_INFO("obstacle_detector started with preset '%s', listening on '%s'", preset.c_str(),
             sub_points_.getTopic().c_str());
  }

 private:
  void onReconfigure(const Config &c) {
    std::lock_guard<std::mutex> lock(mutex_);
    PipelineConfig config = detector_->config();
    try {
      for (const auto &desc : Config::__getParamDescriptions__()) {
        boost::any value;
        desc->getValue(c, value);
        setParameter(&config, desc->name, anyToString(value));
      }
      detector_->setConfig(config);  // may load a network, or fail if it is not built in
    } catch (const std::exception &e) {
      ROS_ERROR("Rejected parameter update: %s", e.what());
      return;
    }
  }

  bool lookup(const std::string &target, const std::string &source, const ros::Time &stamp,
              Eigen::Isometry3f *tf) {
    if (target.empty() || target == source) {
      tf->setIdentity();
      return true;
    }
    try {
      *tf = tf2::transformToEigen(
                tf_buffer_.lookupTransform(target, source, stamp, ros::Duration(tf_timeout_)))
                .cast<float>();
      return true;
    } catch (const tf2::TransformException &) {
      try {
        *tf = tf2::transformToEigen(tf_buffer_.lookupTransform(target, source, ros::Time(0)))
                  .cast<float>();
        ROS_WARN_THROTTLE(5.0, "TF %s <- %s not available at the cloud stamp, using the latest",
                          target.c_str(), source.c_str());
        return true;
      } catch (const tf2::TransformException &e) {
        ROS_WARN_THROTTLE(5.0, "TF %s <- %s unavailable: %s", target.c_str(), source.c_str(),
                          e.what());
        return false;
      }
    }
  }

  void onPoints(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    bool has_intensity = false;
    for (const auto &f : msg->fields) has_intensity |= f.name == "intensity";
    if (has_intensity) {
      pcl::fromROSMsg(*msg, *cloud);
    } else {
      pcl::PointCloud<pcl::PointXYZ> xyz;
      pcl::fromROSMsg(*msg, xyz);
      pcl::copyPointCloud(xyz, *cloud);
    }

    const std::string &sensor_frame = msg->header.frame_id;
    Eigen::Isometry3f tracking_T_sensor;
    if (!lookup(tracking_frame_, sensor_frame, msg->header.stamp, &tracking_T_sensor)) {
      tracking_T_sensor.setIdentity();
      detector_->reset();
    }
    const auto res = detector_->process(cloud, msg->header.stamp.toSec(), tracking_T_sensor);

    // Clouds stay in the lidar frame
    sensor_msgs::PointCloud2 ground, obstacles;
    pcl::toROSMsg(*res.ground, ground);
    pcl::toROSMsg(*res.obstacles, obstacles);
    ground.header = obstacles.header = msg->header;
    pub_ground_.publish(ground);
    pub_obstacles_.publish(obstacles);
    if (pub_occupancy_.getNumSubscribers() > 0 && res.occupancy.width > 0) {
      const auto &g = res.occupancy;
      nav_msgs::OccupancyGrid grid;
      grid.header = msg->header;
      grid.info.map_load_time = msg->header.stamp;
      grid.info.resolution = g.resolution;
      grid.info.width = static_cast<uint32_t>(g.width);
      grid.info.height = static_cast<uint32_t>(g.height);
      grid.info.origin.position.x = g.origin.x();
      grid.info.origin.position.y = g.origin.y();
      grid.info.origin.orientation.w = 1.0;
      grid.data.assign(g.data.begin(), g.data.end());
      pub_occupancy_.publish(grid);
    }
    if (pub_voxels_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 voxels;
      pcl::toROSMsg(*res.occupied_voxels, voxels);
      voxels.header = msg->header;
      pub_voxels_.publish(voxels);
    }
    // 2.5D pillars: cell centre x, y, top z, ground z_min, observed in this scan
    if (pub_pillars_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 pillars;
      pillars.header = msg->header;
      sensor_msgs::PointCloud2Modifier mod(pillars);
      mod.setPointCloud2Fields(
          5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
          "z", 1, sensor_msgs::PointField::FLOAT32, "z_min", 1, sensor_msgs::PointField::FLOAT32,
          "observed", 1, sensor_msgs::PointField::FLOAT32);
      mod.resize(res.pillars.size());
      sensor_msgs::PointCloud2Iterator<float> x(pillars, "x"), y(pillars, "y"), z(pillars, "z"),
          z_min(pillars, "z_min"), observed(pillars, "observed");
      for (const auto &p : res.pillars) {
        *x = p.center.x(), *y = p.center.y(), *z = p.z_max, *z_min = p.z_min;
        *observed = p.observed ? 1.f : 0.f;
        ++x, ++y, ++z, ++z_min, ++observed;
      }
      pub_pillars_.publish(pillars);
    }

    // Boxes in bbox_target_frame (falls back to the lidar frame)
    std_msgs::Header header = msg->header;
    Eigen::Isometry3f target_T_sensor;
    if (lookup(target_frame_, sensor_frame, msg->header.stamp, &target_T_sensor)) {
      if (!target_frame_.empty()) header.frame_id = target_frame_;
    } else {
      target_T_sensor.setIdentity();
    }
    publishObjects(res, header, target_T_sensor, tracking_T_sensor);
    ROS_INFO_THROTTLE(5.0, "%zu obstacles, %.1f ms/frame", res.boxes.size(), res.timings.total);
  }

  // Pillars as cube lists, one per height bin (compact for RViz and recordings)
  static std::vector<visualization_msgs::Marker> pillarMarkers(
      const ObstacleDetector<PointT>::Result &res, const std_msgs::Header &header,
      const Eigen::Isometry3f &target_T_sensor) {
    constexpr float kBin = 0.25f;  // [m]
    constexpr int kBins = 12;
    std::vector<visualization_msgs::Marker> bins(kBins);
    for (int k = 0; k < kBins; ++k) {
      auto &m = bins[k];
      m.header = header;
      m.ns = "pillars";
      m.id = k;
      m.type = visualization_msgs::Marker::CUBE_LIST;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = res.pillar_resolution;
      m.scale.z = (k + 1) * kBin;
      const float shade = 0.55f + 0.35f * k / (kBins - 1);  // taller = lighter
      m.color = rgba(0.62f * shade, 0.66f * shade, 0.76f * shade, 0.9f);
    }
    for (const auto &p : res.pillars) {
      const int k =
          std::clamp(static_cast<int>(std::ceil((p.z_max - p.z_min) / kBin)) - 1, 0, kBins - 1);
      const Eigen::Vector3f c = target_T_sensor * Eigen::Vector3f(p.center.x(), p.center.y(),
                                                                  p.z_min + 0.5f * (k + 1) * kBin);
      geometry_msgs::Point q;
      q.x = c.x(), q.y = c.y(), q.z = c.z();
      bins[k].points.push_back(q);
    }
    std::vector<visualization_msgs::Marker> out;
    for (auto &m : bins)
      if (!m.points.empty()) out.push_back(std::move(m));
    return out;
  }

  void publishObjects(const ObstacleDetector<PointT>::Result &res, const std_msgs::Header &header,
                      const Eigen::Isometry3f &target_T_sensor,
                      const Eigen::Isometry3f &tracking_T_sensor) {
    const std::vector<Box> &boxes = res.boxes;
    const Eigen::Matrix3f r_ts = target_T_sensor.rotation();
    const float target_dyaw = std::atan2(r_ts(1, 0), r_ts(0, 0));
    const Eigen::Matrix3f r_target_tracking = r_ts * tracking_T_sensor.rotation().transpose();

    jsk_recognition_msgs::BoundingBoxArray jsk;
    jsk.header = header;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    // Everything that is not an object (walls, vegetation, unrecognised things)
    for (auto &m : pillarMarkers(res, header, target_T_sensor)) markers.markers.push_back(m);
#ifdef LOD_HAVE_AUTOWARE
    autoware_msgs::DetectedObjectArray objects;
    objects.header = header;
#endif
    for (const auto &b : boxes) {
      const Eigen::Vector3f center = target_T_sensor * b.center;
      const float yaw = wrapAngle(b.yaw + target_dyaw);
      const geometry_msgs::Pose pose = toPose(center, yaw);
      const Eigen::Vector3f vel =
          r_target_tracking * Eigen::Vector3f(b.velocity.x(), b.velocity.y(), 0.f);

      jsk_recognition_msgs::BoundingBox jb;
      jb.header = header;
      jb.pose = pose;
      jb.dimensions.x = b.size.x();
      jb.dimensions.y = b.size.y();
      jb.dimensions.z = b.size.z();
      jb.value = b.score;
      jb.label = b.id;
      jsk.boxes.push_back(jb);

      visualization_msgs::Marker text;
      text.header = header;
      text.ns = "ids";
      text.id = static_cast<int>(b.id);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.pose = pose;
      text.pose.position.z += 0.5 * b.size.z() + 0.4;
      text.scale.z = 0.6;
      text.color.r = text.color.g = text.color.b = text.color.a = 1.f;
      const bool known = b.label != ObjectClass::kUnknown;
      text.text = known ? std::string(objectClassName(b.label)) + " " + std::to_string(b.id)
                        : std::to_string(b.id);
      markers.markers.push_back(text);
      // Recognised classes as 3D models stretched to the box
      if (known) {
        visualization_msgs::Marker model;
        model.header = header;
        model.ns = "models";
        model.id = static_cast<int>(b.id);
        model.type = visualization_msgs::Marker::MESH_RESOURCE;
        model.mesh_resource = mesh_uri_ + objectClassName(b.label) + ".stl";
        model.pose = pose;
        model.scale.x = std::max(b.size.x(), 0.05f);
        model.scale.y = std::max(b.size.y(), 0.05f);
        model.scale.z = std::max(b.size.z(), 0.05f);
        model.color = classColor(b.label);
        if (!b.tracked) model.color.a = 0.35f;
        markers.markers.push_back(model);
      }
      if (!b.predicted_path.empty()) {
        visualization_msgs::Marker path;
        path.header = header;
        path.ns = "predictions";
        path.id = static_cast<int>(b.id);
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.pose.orientation.w = 1.0;
        path.scale.x = 0.15;
        path.color = classColor(b.label);
        const float base = b.center.z() - 0.5f * b.size.z() + 0.1f;  // just above the ground
        path.points.push_back(
            toPoint(target_T_sensor * Eigen::Vector3f(b.center.x(), b.center.y(), base)));
        for (const auto &pp : b.predicted_path)
          path.points.push_back(
              toPoint(target_T_sensor * Eigen::Vector3f(pp.position.x(), pp.position.y(), base)));
        markers.markers.push_back(path);
      }
      if (b.has_velocity && vel.head<2>().norm() > 1.5f) {
        visualization_msgs::Marker arrow = text;
        arrow.ns = "velocity";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.pose = geometry_msgs::Pose();
        arrow.pose.orientation.w = 1.0;
        geometry_msgs::Point p0, p1;
        p0.x = center.x(), p0.y = center.y(), p0.z = center.z();
        p1.x = p0.x + vel.x(), p1.y = p0.y + vel.y(), p1.z = p0.z;
        arrow.points = {p0, p1};
        arrow.scale.x = 0.15, arrow.scale.y = 0.3, arrow.scale.z = 0.3;
        markers.markers.push_back(arrow);
      }

#ifdef LOD_HAVE_AUTOWARE
      autoware_msgs::DetectedObject obj;
      obj.header = header;
      obj.id = b.id;
      obj.label = objectClassName(b.label);
      obj.score = b.score;
      obj.valid = true;
      obj.pose = pose;
      obj.pose_reliable = b.tracked;
      obj.dimensions = jb.dimensions;
      obj.velocity.linear.x = vel.x();
      obj.velocity.linear.y = vel.y();
      obj.velocity_reliable = b.has_velocity;
      objects.objects.push_back(obj);
#endif
    }
    pub_jsk_.publish(jsk);
    pub_markers_.publish(markers);
#ifdef LOD_HAVE_AUTOWARE
    pub_autoware_.publish(objects);
#endif
  }

  ros::NodeHandle nh_, private_nh_;
  std::mutex mutex_;
  std::unique_ptr<ObstacleDetector<PointT>> detector_;
  std::string target_frame_, tracking_frame_, mesh_uri_;
  double tf_timeout_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  dynamic_reconfigure::Server<Config> reconfigure_;
  ros::Subscriber sub_points_;
  ros::Publisher pub_ground_, pub_obstacles_, pub_jsk_, pub_markers_, pub_occupancy_, pub_voxels_,
      pub_pillars_;
#ifdef LOD_HAVE_AUTOWARE
  ros::Publisher pub_autoware_;
#endif
};

}  // namespace lidar_obstacle_detector

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_detector_node");
  lidar_obstacle_detector::ObstacleDetectorNode node;
  ros::spin();
  return 0;
}
