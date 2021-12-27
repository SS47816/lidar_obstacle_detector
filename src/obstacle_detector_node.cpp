/* obstacle_detector_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#include <iostream>
#include <string>
#include <vector>
#include <memory>

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

#include "lidar_obstacle_detector/obstacle_detector.hpp"

namespace lidar_obstacle_detector 
{

class ObstacleDetectorNode
{
 public:
  ObstacleDetectorNode();
  virtual ~ObstacleDetectorNode() {};

 private:
  
  // Filter the inputCloud
  float max_length = 30.0;
  float max_width = 10.0;
  float pos_height = 1.0;
  float neg_height = 2.0;

  std::string bbox_target_frame_;

  std::shared_ptr<ObstacleDetector<pcl::PointXYZ>> obstacle_detector;

  ros::NodeHandle nh;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;

  ros::Subscriber sub_lidar_points;
  ros::Publisher pub_cloud_ground;
  ros::Publisher pub_cloud_clusters;
  ros::Publisher pub_jsk_bboxes;
  // ros::Publisher pub_autoware_objects;

  void lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points);
  jsk_recognition_msgs::BoundingBox transformJskBbox(const BoxQ& box, const geometry_msgs::Pose& pose_transformed);
  // autoware_msgs::DetectedObject transformAutowareObject(const lidar_obstacle_detector::Detection3D::ConstPtr& box, const geometry_msgs::Pose& pose_transformed);

  pcl::PointCloud<pcl::PointXYZ>::Ptr rosPointCloud2toPCL(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2);
  
};

ObstacleDetectorNode::ObstacleDetectorNode() : tf2_listener(tf2_buffer)
{
  ros::NodeHandle private_nh("~");
  
  std::string lidar_points_topic;
  std::string cloud_ground_topic;
  std::string cloud_clusters_topic;
  std::string jsk_bboxes_topic;
  // std::string autoware_objects_topic;
  
  ROS_ASSERT(private_nh.getParam("lidar_points_topic", lidar_points_topic));
  ROS_ASSERT(private_nh.getParam("cloud_ground_topic", cloud_ground_topic));
  ROS_ASSERT(private_nh.getParam("cloud_clusters_topic", cloud_clusters_topic));
  ROS_ASSERT(private_nh.getParam("jsk_bboxes_topic", jsk_bboxes_topic));
  // ROS_ASSERT(private_nh.getParam("autoware_objects_topic", autoware_objects_topic));
  ROS_ASSERT(private_nh.getParam("bbox_target_frame", bbox_target_frame_));

  sub_lidar_points = nh.subscribe(lidar_points_topic, 1, &ObstacleDetectorNode::lidarPointsCallback, this);
  pub_cloud_ground = nh.advertise<sensor_msgs::PointCloud2>(cloud_ground_topic, 1);
  pub_cloud_clusters = nh.advertise<sensor_msgs::PointCloud2>(cloud_clusters_topic, 1);
  pub_jsk_bboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(jsk_bboxes_topic, 1);
  // autoware_objects_pub = nh.advertise<autoware_msgs::DetectedObjectArray>(autoware_objects_topic, 1);

  // Create point processor
  obstacle_detector = std::make_shared<ObstacleDetector<pcl::PointXYZ>>();
}

void ObstacleDetectorNode::lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points)
{
  ROS_INFO("lidar points recieved");
  // auto raw_cloud = rosPointCloud2toPCL(lidar_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_points, *raw_cloud);

  // Downsampleing, ROI, and removing the car roof
  auto filtered_cloud = obstacle_detector->FilterCloud(raw_cloud, 0.2, Eigen::Vector4f(-max_length, -max_width, -neg_height, 1), Eigen::Vector4f(max_length, max_width, pos_height, 1));

  // Segment the groud plane and obstacles
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = obstacle_detector->SegmentPlane(raw_cloud, 100, 0.2);

  // Cluster objects
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = obstacle_detector->Clustering(segmentCloud.first, 1.0, 3, 30);
  int clusterId = 0;
  // std::vector<Color> colors = {Color(1, 0, 1), Color(1, 1, 0), Color(0, 0, 1)};

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
  // autoware_msgs::DetectedObjectArray autoware_objects;
  // autoware_objects.header = lidar_points->header;
  // autoware_objects.header.frame_id = bbox_target_frame_;

  for (auto cluster : cloudClusters)
  {
    // Render Bounding Boxes
    // Box box = obstacle_detector->BoundingBox(cluster);
    // renderBox(viewer, box, clusterId);
    BoxQ box = obstacle_detector->MinimumBoundingBox(cluster);
    // renderBox(viewer, box, clusterId);

    geometry_msgs::Pose pose;
    pose.position.x = box.bboxTransform(0);
    pose.position.y = box.bboxTransform(1);
    pose.position.z = box.bboxTransform(2);
    pose.orientation.x = box.bboxQuaternion.x();
    pose.orientation.y = box.bboxQuaternion.y();
    pose.orientation.z = box.bboxQuaternion.z();
    pose.orientation.w = box.bboxQuaternion.w();

    geometry_msgs::Pose pose_transformed;
    tf2::doTransform(pose, pose_transformed, transform_stamped);

    jsk_bboxes.boxes.emplace_back(transformJskBbox(box, pose_transformed));
    // autoware_objects.objects.emplace_back(transformAutowareObject(box, pose_transformed));

    ++clusterId;
  }
  pub_jsk_bboxes.publish(std::move(jsk_bboxes));
  // autoware_objects_pub.publish(std::move(autoware_objects));

  sensor_msgs::PointCloud2::Ptr ground_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(segmentCloud.second), *ground_cloud);
  ground_cloud->header = lidar_points->header;

  sensor_msgs::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(segmentCloud.first), *obstacle_cloud);
  obstacle_cloud->header = lidar_points->header;

  pub_cloud_ground.publish(std::move(ground_cloud));
  pub_cloud_clusters.publish(std::move(obstacle_cloud));

}



jsk_recognition_msgs::BoundingBox ObstacleDetectorNode::transformJskBbox(const BoxQ& box, const geometry_msgs::Pose& pose_transformed)
{
  jsk_recognition_msgs::BoundingBox jsk_bbox;
  // jsk_bbox.header = box->header;
  jsk_bbox.header.frame_id = bbox_target_frame_;
  jsk_bbox.pose = pose_transformed;
  jsk_bbox.dimensions.x = box.cube_length;
  jsk_bbox.dimensions.y = box.cube_width;
  jsk_bbox.dimensions.z = box.cube_height;
  // jsk_bbox.value = box->score;
  jsk_bbox.label = 0;

  return std::move(jsk_bbox);
}


/**
 
autoware_msgs::DetectedObject ObstacleDetectorNode::transformAutowareObject(const lidar_obstacle_detector::Detection3D::ConstPtr& box, const geometry_msgs::Pose& pose_transformed)
{
  autoware_msgs::DetectedObject autoware_object;
  autoware_object.header = box->header;
  autoware_object.header.frame_id = bbox_target_frame_;
  autoware_object.id = box->id;
  autoware_object.label = box->label;
  autoware_object.score = box->score;
  autoware_object.pose = pose_transformed;
  autoware_object.pose.position.z += box->bbox.size.z/2;
  autoware_object.pose_reliable = true;
  autoware_object.dimensions = box->bbox.size;
  autoware_object.velocity = box->velocity;
  autoware_object.velocity_reliable = true;
  autoware_object.valid = true;
  if (box->label == "Pedestrian")
  { 
    autoware_object.label = "person";
  }
  else if (box->label == "Bicyclist")
  { 
    autoware_object.label = "bicycle";
  }
  else if (box->label == "SchoolBus")
  { 
    autoware_object.label = "bus";
  }
  else if (box->label == "BoxTruck")
  { 
    autoware_object.label = "truck";
  }
  else 
  { 
    autoware_object.label = "car";
  }

  return std::move(autoware_object);
}

**/

} // namespace lidar_obstacle_detector

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector_node");
  lidar_obstacle_detector::ObstacleDetectorNode obstacle_detector_node;
  ros::spin();
  return 0;
}