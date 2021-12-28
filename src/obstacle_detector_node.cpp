/* obstacle_detector_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_obstacle_detector/obstacle_detector_Config.h>

#include "lidar_obstacle_detector/obstacle_detector.hpp"

namespace lidar_obstacle_detector 
{

// Pointcloud Filtering Parameters
float VOXEL_GRID_SIZE;
Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;

class ObstacleDetectorNode
{
 public:
  ObstacleDetectorNode();
  virtual ~ObstacleDetectorNode() {};

 private:
  
  // Filter the inputCloud
  // float max_length = 30.0;
  // float max_width = 10.0;
  // float pos_height = 1.0;
  // float neg_height = 2.0;
  
  std::string bbox_target_frame_;

  std::vector<Box> prev_boxes_;
  std::shared_ptr<ObstacleDetector<pcl::PointXYZ>> obstacle_detector;

  ros::NodeHandle nh;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  dynamic_reconfigure::Server<lidar_obstacle_detector::obstacle_detector_Config> server;
  dynamic_reconfigure::Server<lidar_obstacle_detector::obstacle_detector_Config>::CallbackType f;

  ros::Subscriber sub_lidar_points;
  ros::Publisher pub_cloud_ground;
  ros::Publisher pub_cloud_clusters;
  ros::Publisher pub_jsk_bboxes;
  ros::Publisher pub_autoware_objects;

  void lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points);
  jsk_recognition_msgs::BoundingBox transformJskBbox(const Box& box, const geometry_msgs::Pose& pose_transformed);
  autoware_msgs::DetectedObject transformAutowareObject(const Box& box, const geometry_msgs::Pose& pose_transformed);
};

// Dynamic parameter server callback function
void dynamicParamCallback(lidar_obstacle_detector::obstacle_detector_Config& config, uint32_t level)
{
  // Pointcloud Filtering Parameters
  VOXEL_GRID_SIZE = config.voxel_grid_size;
  ROI_MAX_POINT = Eigen::Vector4f(config.roi_max_x, config.roi_max_y, config.roi_max_z, 1);
  ROI_MIN_POINT = Eigen::Vector4f(config.roi_min_x, config.roi_min_y, config.roi_min_z, 1);
}

ObstacleDetectorNode::ObstacleDetectorNode() : tf2_listener(tf2_buffer)
{
  ros::NodeHandle private_nh("~");
  
  std::string lidar_points_topic;
  std::string cloud_ground_topic;
  std::string cloud_clusters_topic;
  std::string jsk_bboxes_topic;
  std::string autoware_objects_topic;
  
  ROS_ASSERT(private_nh.getParam("lidar_points_topic", lidar_points_topic));
  ROS_ASSERT(private_nh.getParam("cloud_ground_topic", cloud_ground_topic));
  ROS_ASSERT(private_nh.getParam("cloud_clusters_topic", cloud_clusters_topic));
  ROS_ASSERT(private_nh.getParam("jsk_bboxes_topic", jsk_bboxes_topic));
  ROS_ASSERT(private_nh.getParam("autoware_objects_topic", autoware_objects_topic));
  ROS_ASSERT(private_nh.getParam("bbox_target_frame", bbox_target_frame_));

  sub_lidar_points = nh.subscribe(lidar_points_topic, 1, &ObstacleDetectorNode::lidarPointsCallback, this);
  pub_cloud_ground = nh.advertise<sensor_msgs::PointCloud2>(cloud_ground_topic, 1);
  pub_cloud_clusters = nh.advertise<sensor_msgs::PointCloud2>(cloud_clusters_topic, 1);
  pub_jsk_bboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(jsk_bboxes_topic, 1);
  pub_autoware_objects = nh.advertise<autoware_msgs::DetectedObjectArray>(autoware_objects_topic, 1);

  // Dynamic Parameter Server & Function
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  // Create point processor
  obstacle_detector = std::make_shared<ObstacleDetector<pcl::PointXYZ>>();

}

void ObstacleDetectorNode::lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points)
{
  ROS_INFO("lidar points recieved");
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_points, *raw_cloud);

  // Downsampleing, ROI, and removing the car roof
  // auto filtered_cloud = obstacle_detector->FilterCloud(raw_cloud, 0.2, Eigen::Vector4f(-max_length, -max_width, -neg_height, 1), Eigen::Vector4f(max_length, max_width, pos_height, 1));
  auto filtered_cloud = obstacle_detector->FilterCloud(raw_cloud, VOXEL_GRID_SIZE, ROI_MIN_POINT, ROI_MAX_POINT);

  // Segment the groud plane and obstacles
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = obstacle_detector->SegmentPlane(raw_cloud, 100, 0.2);

  // Cluster objects
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = obstacle_detector->Clustering(segmentCloud.first, 1.0, 3, 30);
  
  // Construct Bounding Boxes from the clusters
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf2_buffer.lookupTransform(bbox_target_frame_, lidar_points->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
  jsk_bboxes.header = lidar_points->header;
  jsk_bboxes.header.frame_id = bbox_target_frame_;
  autoware_msgs::DetectedObjectArray autoware_objects;
  autoware_objects.header = lidar_points->header;
  autoware_objects.header.frame_id = bbox_target_frame_;

  int cluster_id = 0;
  std::vector<Box> curr_boxes;
  for (auto& cluster : cloudClusters)
  {
    // Create Bounding Boxes
    auto box = obstacle_detector->BoundingBox(cluster, cluster_id);
    // Box box = obstacle_detector->MinimumBoundingBox(cluster);

    geometry_msgs::Pose pose, pose_transformed;
    pose.position.x = box.position(0);
    pose.position.y = box.position(1);
    pose.position.z = box.position(2);
    pose.orientation.w = box.quaternion.w();
    pose.orientation.x = box.quaternion.x();
    pose.orientation.y = box.quaternion.y();
    pose.orientation.z = box.quaternion.z();
    tf2::doTransform(pose, pose_transformed, transform_stamped);

    jsk_bboxes.boxes.emplace_back(transformJskBbox(box, pose_transformed));
    autoware_objects.objects.emplace_back(transformAutowareObject(box, pose_transformed));

    curr_boxes.emplace_back(box);
    ++cluster_id;
  }
  pub_jsk_bboxes.publish(std::move(jsk_bboxes));
  pub_autoware_objects.publish(std::move(autoware_objects));

  sensor_msgs::PointCloud2::Ptr ground_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(segmentCloud.second), *ground_cloud);
  ground_cloud->header = lidar_points->header;

  sensor_msgs::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(segmentCloud.first), *obstacle_cloud);
  obstacle_cloud->header = lidar_points->header;

  pub_cloud_ground.publish(std::move(ground_cloud));
  pub_cloud_clusters.publish(std::move(obstacle_cloud));

  // Update previous bounding boxes
  prev_boxes_ = std::move(curr_boxes);
}


jsk_recognition_msgs::BoundingBox ObstacleDetectorNode::transformJskBbox(const Box& box, const geometry_msgs::Pose& pose_transformed)
{
  jsk_recognition_msgs::BoundingBox jsk_bbox;
  // jsk_bbox.header = box->header;
  jsk_bbox.header.frame_id = bbox_target_frame_;
  jsk_bbox.pose = pose_transformed;
  jsk_bbox.dimensions.x = box.dimension(0);
  jsk_bbox.dimensions.y = box.dimension(1);
  jsk_bbox.dimensions.z = box.dimension(2);
  jsk_bbox.value = 1.0f;
  jsk_bbox.label = box.id;
  // jsk_bbox.value = box->score;
  // jsk_bbox.label = 0;

  return std::move(jsk_bbox);
}


autoware_msgs::DetectedObject ObstacleDetectorNode::transformAutowareObject(const Box& box, const geometry_msgs::Pose& pose_transformed)
{
  autoware_msgs::DetectedObject autoware_object;
  // autoware_object.header = box->header;
  autoware_object.header.frame_id = bbox_target_frame_;
  autoware_object.id = box.id;
  autoware_object.label = "unknown";
  autoware_object.score = 1.0f;
  autoware_object.pose = pose_transformed;
  autoware_object.pose.position.z += box.dimension(2)/2;
  autoware_object.pose_reliable = true;
  autoware_object.dimensions.x = box.dimension(0);
  autoware_object.dimensions.y = box.dimension(1);
  autoware_object.dimensions.z = box.dimension(2);

  // autoware_object.velocity = box->velocity;
  // autoware_object.velocity_reliable = true;
  autoware_object.valid = true;

  return std::move(autoware_object);
}


} // namespace lidar_obstacle_detector

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector_node");
  lidar_obstacle_detector::ObstacleDetectorNode obstacle_detector_node;
  ros::spin();
  return 0;
}