/* obstacle_detector_node.cpp (ROS 2)

 * Copyright (C) 2021-2026 SS47816

 * ROS 2 component wrapping the ROS-agnostic obstacle detection pipeline

**/

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#ifdef LOD_HAVE_AUTOWARE
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#endif

#include "lidar_obstacle_detector/obstacle_detector.hpp"

namespace lidar_obstacle_detector {

namespace {

using PointT = pcl::PointXYZI;

std::string toString(const rclcpp::Parameter &p) {
  std::ostringstream ss;
  ss.precision(10);
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return p.as_bool() ? "true" : "false";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return std::to_string(p.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      ss << p.as_double();
      return ss.str();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return p.as_string();
    default:
      throw std::invalid_argument(p.get_name() + ": unsupported parameter type");
  }
}

geometry_msgs::msg::Pose toPose(const Eigen::Vector3f &center, float yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = center.x();
  pose.position.y = center.y();
  pose.position.z = center.z();
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

std_msgs::msg::ColorRGBA idColor(uint32_t id, float alpha) {
  // Golden-ratio hue walk: neighbouring ids get clearly different colours
  const float h = std::fmod(id * 0.618034f, 1.f) * 6.f;
  const float x = 1.f - std::abs(std::fmod(h, 2.f) - 1.f);
  float r = 0, g = 0, b = 0;
  switch (static_cast<int>(h)) {
    case 0:
      r = 1, g = x;
      break;
    case 1:
      r = x, g = 1;
      break;
    case 2:
      g = 1, b = x;
      break;
    case 3:
      g = x, b = 1;
      break;
    case 4:
      r = x, b = 1;
      break;
    default:
      r = 1, b = x;
      break;
  }
  std_msgs::msg::ColorRGBA c;
  c.r = r, c.g = g, c.b = b, c.a = alpha;
  return c;
}

}  // namespace

class ObstacleDetectorNode : public rclcpp::Node {
 public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions &options)
      : Node("obstacle_detector", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Base preset (legacy | fast | accurate); other parameters override it";
    const std::string preset = declare_parameter<std::string>("preset", "fast", read_only);
    target_frame_ = declare_parameter<std::string>("target_frame", "");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "");
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);

    PipelineConfig config = PipelineConfig::preset(preset);
    declarePipelineParameters(&config);
    detector_ = std::make_unique<ObstacleDetector<PointT>>(config);
    param_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) { return onParameters(params); });

    pub_ground_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud_ground", 1);
    pub_obstacles_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud_obstacles", 1);
    pub_detections_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/detections", 1);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);
    pub_occupancy_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy", 1);
    pub_voxels_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/occupancy_voxels", 1);
#ifdef LOD_HAVE_AUTOWARE
    pub_autoware_ =
        create_publisher<autoware_perception_msgs::msg::DetectedObjects>("~/objects", 1);
#endif
    sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onPoints(msg); });

    RCLCPP_INFO(get_logger(), "Started with preset '%s', listening on '%s'", preset.c_str(),
                sub_points_->get_topic_name());
  }

 private:
  void declarePipelineParameters(PipelineConfig *config) {
    for (const auto &p : parameterTable()) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = p.description;
      const std::string value = getParameter(*config, p.name);
      rclcpp::Parameter declared;
      switch (p.type) {
        case ParamInfo::Type::kBool:
          declared = rclcpp::Parameter(p.name, declare_parameter<bool>(p.name, value == "true", d));
          break;
        case ParamInfo::Type::kInt: {
          rcl_interfaces::msg::IntegerRange r;
          r.from_value = static_cast<int64_t>(p.min);
          r.to_value = static_cast<int64_t>(p.max);
          r.step = 1;
          d.integer_range.push_back(r);
          declared =
              rclcpp::Parameter(p.name, declare_parameter<int64_t>(p.name, std::stoll(value), d));
          break;
        }
        case ParamInfo::Type::kDouble: {
          rcl_interfaces::msg::FloatingPointRange r;
          r.from_value = p.min;
          r.to_value = p.max;
          d.floating_point_range.push_back(r);
          declared =
              rclcpp::Parameter(p.name, declare_parameter<double>(p.name, std::stod(value), d));
          break;
        }
        case ParamInfo::Type::kString: {
          std::string choices;
          for (const auto &c : p.choices) choices += (choices.empty() ? "" : ", ") + c;
          d.additional_constraints = "one of: " + choices;
          declared = rclcpp::Parameter(p.name, declare_parameter<std::string>(p.name, value, d));
          break;
        }
      }
      // Values from YAML / launch overrides
      setParameter(config, p.name, toString(declared));
    }
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
      const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::lock_guard<std::mutex> lock(mutex_);
    PipelineConfig config = detector_->config();
    std::string target = target_frame_, tracking = tracking_frame_;
    double timeout = tf_timeout_;
    try {
      for (const auto &p : params) {
        if (p.get_name() == "target_frame")
          target = p.as_string();
        else if (p.get_name() == "tracking_frame")
          tracking = p.as_string();
        else if (p.get_name() == "tf_timeout")
          timeout = p.as_double();
        else
          setParameter(&config, p.get_name(), toString(p));
      }
    } catch (const std::exception &e) {
      result.successful = false;
      result.reason = e.what();
      return result;
    }
    if (tracking != tracking_frame_) detector_->reset();  // tracks live in that frame
    target_frame_ = target;
    tracking_frame_ = tracking;
    tf_timeout_ = timeout;
    detector_->setConfig(config);
    return result;
  }

  bool lookup(const std::string &target, const std::string &source, const rclcpp::Time &stamp,
              Eigen::Isometry3f *tf) {
    if (target.empty() || target == source) {
      tf->setIdentity();
      return true;
    }
    try {
      const auto msg = tf_buffer_.lookupTransform(target, source, stamp,
                                                  rclcpp::Duration::from_seconds(tf_timeout_));
      *tf = tf2::transformToEigen(msg).cast<float>();
      return true;
    } catch (const tf2::TransformException &e) {
      // Fall back to the latest transform rather than dropping the frame
      try {
        const auto msg = tf_buffer_.lookupTransform(target, source, tf2::TimePointZero);
        *tf = tf2::transformToEigen(msg).cast<float>();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "TF %s <- %s not available at the cloud stamp, using the latest "
                             "one (%s)",
                             target.c_str(), source.c_str(), e.what());
        return true;
      } catch (const tf2::TransformException &e2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s <- %s unavailable: %s",
                             target.c_str(), source.c_str(), e2.what());
        return false;
      }
    }
  }

  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    const bool has_intensity = std::any_of(msg->fields.begin(), msg->fields.end(),
                                           [](const auto &f) { return f.name == "intensity"; });
    if (has_intensity) {
      pcl::fromROSMsg(*msg, *cloud);
    } else {
      pcl::PointCloud<pcl::PointXYZ> xyz;
      pcl::fromROSMsg(*msg, xyz);
      pcl::copyPointCloud(xyz, *cloud);
    }

    const std::string &sensor_frame = msg->header.frame_id;
    const rclcpp::Time stamp(msg->header.stamp);
    Eigen::Isometry3f tracking_T_sensor;
    if (!lookup(tracking_frame_, sensor_frame, stamp, &tracking_T_sensor)) {
      // Tracking in a frame we cannot reach would corrupt the tracks
      tracking_T_sensor.setIdentity();
      detector_->reset();
    }
    const auto res = detector_->process(cloud, stamp.seconds(), tracking_T_sensor);

    Eigen::Isometry3f target_T_sensor;
    std_msgs::msg::Header out_header = msg->header;
    if (lookup(target_frame_, sensor_frame, stamp, &target_T_sensor)) {
      if (!target_frame_.empty()) out_header.frame_id = target_frame_;
    } else {
      target_T_sensor.setIdentity();
    }
    publishClouds(res, msg->header);
    publishObjects(res.boxes, res.structure_boxes, out_header, target_T_sensor, tracking_T_sensor);

    RCLCPP_DEBUG(get_logger(),
                 "%zu obstacles in %.1f ms (pre %.1f, ground %.1f, cluster %.1f, box %.1f, "
                 "track %.1f)",
                 res.boxes.size(), res.timings.total, res.timings.preprocess, res.timings.ground,
                 res.timings.cluster, res.timings.box, res.timings.track);
    ++frames_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "%zu obstacles, %.1f ms/frame (%zu frames processed)", res.boxes.size(),
                         res.timings.total, frames_);
  }

  void publishClouds(const ObstacleDetector<PointT>::Result &res,
                     const std_msgs::msg::Header &header) {
    if (pub_ground_->get_subscription_count() > 0) {
      auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*res.ground, *out);
      out->header = header;
      pub_ground_->publish(std::move(out));
    }
    if (pub_obstacles_->get_subscription_count() > 0) {
      auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*res.obstacles, *out);
      out->header = header;
      pub_obstacles_->publish(std::move(out));
    }
    // Occupancy outputs, in the lidar frame
    if (pub_occupancy_->get_subscription_count() > 0 && res.occupancy.width > 0) {
      const auto &g = res.occupancy;
      auto out = std::make_unique<nav_msgs::msg::OccupancyGrid>();
      out->header = header;
      out->info.map_load_time = header.stamp;
      out->info.resolution = g.resolution;
      out->info.width = static_cast<uint32_t>(g.width);
      out->info.height = static_cast<uint32_t>(g.height);
      out->info.origin.position.x = g.origin.x();
      out->info.origin.position.y = g.origin.y();
      out->info.origin.orientation.w = 1.0;
      out->data.assign(g.data.begin(), g.data.end());
      pub_occupancy_->publish(std::move(out));
    }
    if (pub_voxels_->get_subscription_count() > 0) {
      auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*res.occupied_voxels, *out);
      out->header = header;
      pub_voxels_->publish(std::move(out));
    }
  }

  void publishObjects(const std::vector<Box> &boxes, const std::vector<Box> &structures,
                      const std_msgs::msg::Header &header, const Eigen::Isometry3f &target_T_sensor,
                      const Eigen::Isometry3f &tracking_T_sensor) {
    const Eigen::Matrix3f r_ts = target_T_sensor.rotation();
    const float target_dyaw = std::atan2(r_ts(1, 0), r_ts(0, 0));
    // Velocities come in the tracking frame; express them in the target frame
    const Eigen::Matrix3f r_target_tracking = r_ts * tracking_T_sensor.rotation().transpose();

    vision_msgs::msg::Detection3DArray detections;
    detections.header = header;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    // Structures (walls, vegetation, ...): grey, not tracked, not in the object outputs
    for (size_t k = 0; k < structures.size(); ++k) {
      const Box &b = structures[k];
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns = "structures";
      m.id = static_cast<int>(k);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.pose = toPose(target_T_sensor * b.center, wrapAngle(b.yaw + target_dyaw));
      m.scale.x = std::max(b.size.x(), 0.05f);
      m.scale.y = std::max(b.size.y(), 0.05f);
      m.scale.z = std::max(b.size.z(), 0.05f);
      m.color.r = m.color.g = m.color.b = 0.6f;
      m.color.a = 0.15f;
      markers.markers.push_back(m);
    }
#ifdef LOD_HAVE_AUTOWARE
    autoware_perception_msgs::msg::DetectedObjects objects;
    objects.header = header;
#endif

    for (const auto &b : boxes) {
      const Eigen::Vector3f center = target_T_sensor * b.center;
      const float yaw = wrapAngle(b.yaw + target_dyaw);
      const geometry_msgs::msg::Pose pose = toPose(center, yaw);
      const Eigen::Vector3f vel =
          r_target_tracking * Eigen::Vector3f(b.velocity.x(), b.velocity.y(), 0.f);

      vision_msgs::msg::Detection3D det;
      det.header = header;
      det.id = std::to_string(b.id);
      det.bbox.center = pose;
      det.bbox.size.x = b.size.x();
      det.bbox.size.y = b.size.y();
      det.bbox.size.z = b.size.z();
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = "unknown";
      hyp.hypothesis.score = 1.0;
      hyp.pose.pose = pose;
      det.results.push_back(hyp);
      detections.detections.push_back(det);

      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "boxes";
      box.id = static_cast<int>(b.id);
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.pose = pose;
      box.scale.x = std::max(b.size.x(), 0.05f);
      box.scale.y = std::max(b.size.y(), 0.05f);
      box.scale.z = std::max(b.size.z(), 0.05f);
      box.color = idColor(b.id, b.tracked ? 0.5f : 0.2f);
      markers.markers.push_back(box);

      visualization_msgs::msg::Marker text = box;
      text.ns = "ids";
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.z += 0.5 * b.size.z() + 0.4;
      text.scale.x = text.scale.y = 0.0;
      text.scale.z = 0.6;
      text.color.a = 1.0;
      text.text = std::to_string(b.id);
      markers.markers.push_back(text);

      if (b.has_velocity && vel.head<2>().norm() > 1.5f) {
        visualization_msgs::msg::Marker arrow = box;
        arrow.ns = "velocity";
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.pose = geometry_msgs::msg::Pose();
        geometry_msgs::msg::Point p0, p1;
        p0.x = center.x(), p0.y = center.y(), p0.z = center.z();
        p1.x = p0.x + vel.x(), p1.y = p0.y + vel.y(), p1.z = p0.z;
        arrow.points = {p0, p1};
        arrow.scale.x = 0.15, arrow.scale.y = 0.3, arrow.scale.z = 0.3;
        arrow.color.a = 1.0;
        markers.markers.push_back(arrow);
      }

#ifdef LOD_HAVE_AUTOWARE
      autoware_perception_msgs::msg::DetectedObject obj;
      obj.existence_probability = 1.f;
      autoware_perception_msgs::msg::ObjectClassification cls;
      cls.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      cls.probability = 1.f;
      obj.classification.push_back(cls);
      obj.kinematics.pose_with_covariance.pose = pose;
      obj.kinematics.orientation_availability =
          b.heading_known ? autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE
                          : autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
      if (b.has_velocity) {
        // Autoware expresses the twist in the object frame
        const float c = std::cos(yaw), s = std::sin(yaw);
        obj.kinematics.has_twist = true;
        obj.kinematics.twist_with_covariance.twist.linear.x = c * vel.x() + s * vel.y();
        obj.kinematics.twist_with_covariance.twist.linear.y = -s * vel.x() + c * vel.y();
      }
      obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      obj.shape.dimensions.x = b.size.x();
      obj.shape.dimensions.y = b.size.y();
      obj.shape.dimensions.z = b.size.z();
      objects.objects.push_back(obj);
#endif
    }
    pub_detections_->publish(detections);
    pub_markers_->publish(markers);
#ifdef LOD_HAVE_AUTOWARE
    pub_autoware_->publish(objects);
#endif
  }

  std::mutex mutex_;
  std::unique_ptr<ObstacleDetector<PointT>> detector_;
  std::string target_frame_, tracking_frame_;
  double tf_timeout_;
  size_t frames_ = 0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_, pub_obstacles_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxels_;
#ifdef LOD_HAVE_AUTOWARE
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_autoware_;
#endif
};

}  // namespace lidar_obstacle_detector

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_obstacle_detector::ObstacleDetectorNode)
