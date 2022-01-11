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
bool USE_PCA_BOX;
bool USE_TRACKING;
float VOXEL_GRID_SIZE;
Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;
float GROUND_THRESH;
float CLUSTER_THRESH;
int CLUSTER_MAX_SIZE, CLUSTER_MIN_SIZE;
float DISPLACEMENT_THRESH, IOU_THRESH;

class ObstacleDetectorNode
{
 public:
  ObstacleDetectorNode();
  virtual ~ObstacleDetectorNode() {};

 private:
  size_t obstacle_id_;
  std::string bbox_target_frame_, bbox_source_frame_;
  std::vector<Box> prev_boxes_, curr_boxes_;
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
  void publishClouds(const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>&& segmented_clouds, const std_msgs::Header& header);
  jsk_recognition_msgs::BoundingBox transformJskBbox(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed);
  autoware_msgs::DetectedObject transformAutowareObject(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed);
  void publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::Header& header);
};

// Dynamic parameter server callback function
void dynamicParamCallback(lidar_obstacle_detector::obstacle_detector_Config& config, uint32_t level)
{
  // Pointcloud Filtering Parameters
  USE_PCA_BOX = config.use_pca_box;
  USE_TRACKING = config.use_tracking;
  VOXEL_GRID_SIZE = config.voxel_grid_size;
  ROI_MAX_POINT = Eigen::Vector4f(config.roi_max_x, config.roi_max_y, config.roi_max_z, 1);
  ROI_MIN_POINT = Eigen::Vector4f(config.roi_min_x, config.roi_min_y, config.roi_min_z, 1);
  GROUND_THRESH = config.ground_threshold;
  CLUSTER_THRESH = config.cluster_threshold;
  CLUSTER_MAX_SIZE = config.cluster_max_size;
  CLUSTER_MIN_SIZE = config.cluster_min_size;
  DISPLACEMENT_THRESH = config.displacement_threshold;
  IOU_THRESH = config.iou_threshold;
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
  obstacle_id_ = 0;
}

void ObstacleDetectorNode::lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points)
{
  ROS_DEBUG("lidar points recieved");
  // Time the whole process
  const auto start_time = std::chrono::steady_clock::now();
  const auto pointcloud_header = lidar_points->header;
  bbox_source_frame_ = lidar_points->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_points, *raw_cloud);

  // Downsampleing, ROI, and removing the car roof
  auto filtered_cloud = obstacle_detector->filterCloud(raw_cloud, VOXEL_GRID_SIZE, ROI_MIN_POINT, ROI_MAX_POINT);

  // Segment the groud plane and obstacles
  auto segmented_clouds = obstacle_detector->segmentPlane(filtered_cloud, 30, GROUND_THRESH);

  // Cluster objects
  auto cloud_clusters = obstacle_detector->clustering(segmented_clouds.first, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
  
  // Publish ground cloud and obstacle cloud
  publishClouds(std::move(segmented_clouds), pointcloud_header);
  // Publish Obstacles
  publishDetectedObjects(std::move(cloud_clusters), pointcloud_header);

  // Time the whole process
  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  ROS_INFO("The obstacle_detector_node found %d obstacles in %.3f second", int(prev_boxes_.size()), float(elapsed_time.count()/1000.0));
}

void ObstacleDetectorNode::publishClouds(const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>&& segmented_clouds, const std_msgs::Header& header)
{
  sensor_msgs::PointCloud2::Ptr ground_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(segmented_clouds.second), *ground_cloud);
  ground_cloud->header = header;

  sensor_msgs::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(segmented_clouds.first), *obstacle_cloud);
  obstacle_cloud->header = header;

  pub_cloud_ground.publish(std::move(ground_cloud));
  pub_cloud_clusters.publish(std::move(obstacle_cloud));
}

jsk_recognition_msgs::BoundingBox ObstacleDetectorNode::transformJskBbox(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed)
{
  jsk_recognition_msgs::BoundingBox jsk_bbox;
  jsk_bbox.header = header;
  jsk_bbox.pose = pose_transformed;
  jsk_bbox.dimensions.x = box.dimension(0);
  jsk_bbox.dimensions.y = box.dimension(1);
  jsk_bbox.dimensions.z = box.dimension(2);
  jsk_bbox.value = 1.0f;
  jsk_bbox.label = box.id;

  return std::move(jsk_bbox);
}


autoware_msgs::DetectedObject ObstacleDetectorNode::transformAutowareObject(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed)
{
  autoware_msgs::DetectedObject autoware_object;
  autoware_object.header = header;
  autoware_object.id = box.id;
  autoware_object.label = "unknown";
  autoware_object.score = 1.0f;
  autoware_object.pose = pose_transformed;
  autoware_object.pose_reliable = true;
  autoware_object.dimensions.x = box.dimension(0);
  autoware_object.dimensions.y = box.dimension(1);
  autoware_object.dimensions.z = box.dimension(2);
  autoware_object.valid = true;

  return std::move(autoware_object);
}

void ObstacleDetectorNode::publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::Header& header)
{
  for (auto& cluster : cloud_clusters)
  {
    // Create Bounding Boxes
    Box box = USE_PCA_BOX? 
      obstacle_detector->pcaBoundingBox(cluster, obstacle_id_) : 
      obstacle_detector->axisAlignedBoundingBox(cluster, obstacle_id_);
    
    obstacle_id_ = (obstacle_id_ < SIZE_MAX)? ++obstacle_id_ : 0;
    curr_boxes_.emplace_back(box);
  }

  // Re-assign Box ids based on tracking result
  if (USE_TRACKING)
    obstacle_detector->obstacleTracking(prev_boxes_, curr_boxes_, DISPLACEMENT_THRESH, IOU_THRESH);
  
  // Lookup for frame transform between the lidar frame and the target frame
  auto bbox_header = header;
  bbox_header.frame_id = bbox_target_frame_;
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf2_buffer.lookupTransform(bbox_target_frame_, bbox_source_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("Frame Transform Given Up! Outputing obstacles in the original LiDAR frame %s instead...", bbox_source_frame_.c_str());
    bbox_header.frame_id = bbox_source_frame_;
    try
    {
      transform_stamped = tf2_buffer.lookupTransform(bbox_source_frame_, bbox_source_frame_, ros::Time(0));
    }
    catch (tf2::TransformException& ex2)
    {
      ROS_ERROR("%s", ex2.what());
      return;
    }
  }

  // Construct Bounding Boxes from the clusters
  jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
  jsk_bboxes.header = bbox_header;
  autoware_msgs::DetectedObjectArray autoware_objects;
  autoware_objects.header = bbox_header;

  // Transform boxes from lidar frame to base_link frame, and convert to jsk and autoware msg formats
  for (auto& box : curr_boxes_)
  {
    geometry_msgs::Pose pose, pose_transformed;
    pose.position.x = box.position(0);
    pose.position.y = box.position(1);
    pose.position.z = box.position(2);
    pose.orientation.w = box.quaternion.w();
    pose.orientation.x = box.quaternion.x();
    pose.orientation.y = box.quaternion.y();
    pose.orientation.z = box.quaternion.z();
    tf2::doTransform(pose, pose_transformed, transform_stamped);

    jsk_bboxes.boxes.emplace_back(transformJskBbox(box, bbox_header, pose_transformed));
    autoware_objects.objects.emplace_back(transformAutowareObject(box, bbox_header, pose_transformed));
  }
  pub_jsk_bboxes.publish(std::move(jsk_bboxes));
  pub_autoware_objects.publish(std::move(autoware_objects));

  // Update previous bounding boxes
  prev_boxes_.swap(curr_boxes_);
  curr_boxes_.clear();
}


} // namespace lidar_obstacle_detector

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector_node");
  lidar_obstacle_detector::ObstacleDetectorNode obstacle_detector_node;
  ros::spin();
  return 0;
}