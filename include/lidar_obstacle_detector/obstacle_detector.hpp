/* obstacle_detector.hpp

 * Copyright (C) 2021 SS47816

 * Implementation of 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "box.hpp"

namespace lidar_obstacle_detector
{
template <typename PointT>
class ObstacleDetector
{
 public:
  ObstacleDetector();
  virtual ~ObstacleDetector();

  // ****************** Detection ***********************

  typename pcl::PointCloud<PointT>::Ptr filterCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float filter_res, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt);
  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentPlane(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const int max_iterations, const float distance_thresh);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clustering(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size);

  Box axisAlignedBoundingBox(const typename pcl::PointCloud<PointT>::ConstPtr& cluster, const int id);

  Box pcaBoundingBox(typename pcl::PointCloud<PointT>::Ptr& cluster, const int id);

  // ****************** Tracking ***********************
  void obstacleTracking(const std::vector<Box>& prev_boxes, std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh);

 private:
  // ****************** Detection ***********************
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateClouds(const pcl::PointIndices::ConstPtr& inliers, const typename pcl::PointCloud<PointT>::ConstPtr& cloud);

  // ****************** Tracking ***********************
  bool compareBoxes(const Box& a, const Box& b, const float displacement_thresh, const float iou_thresh);

  // Link nearby bounding boxes between the previous and previous frame
  std::vector<std::vector<int>> associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh);

  // Connection Matrix
  std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right);

  // Helper function for Hungarian Algorithm
  bool hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);

  // Customized Hungarian Algorithm
  std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix);

  // Helper function for searching the box index in boxes given an id
  int searchBoxIndex(const std::vector<Box>& Boxes, const int id);
};

// constructor:
template <typename PointT>
ObstacleDetector<PointT>::ObstacleDetector() {}

// de-constructor:
template <typename PointT>
ObstacleDetector<PointT>::~ObstacleDetector() {}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ObstacleDetector<PointT>::filterCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float filter_res, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt)
{
  // Time segmentation process
  // const auto start_time = std::chrono::steady_clock::now();

  // Create the filtering object: downsample the dataset using a leaf size
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filter_res, filter_res, filter_res);
  vg.filter(*cloud_filtered);

  // Cropping the ROI
  typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(min_pt);
  region.setMax(max_pt);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_roi);

  // Removing the car roof region
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_roi);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto& point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_roi);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_roi);

  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;

  return cloud_roi;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::separateClouds(const pcl::PointIndices::ConstPtr& inliers, const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());

  // Pushback all the inliers into the ground_cloud
  for (int index : inliers->indices)
  {
    ground_cloud->points.push_back(cloud->points[index]);
  }

  // Extract the points that are not in the inliers to obstacle_cloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstacle_cloud, ground_cloud);
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::segmentPlane(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const int max_iterations, const float distance_thresh)
{
  // Time segmentation process
  // const auto start_time = std::chrono::steady_clock::now();

  // Find inliers for the cloud.
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(distance_thresh);

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty())
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // std::cout << "plane segmentation took " << elapsed_time.count() << " milliseconds" << std::endl;

  return separateClouds(inliers, cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::clustering(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size)
{
  // Time clustering process
  // const auto start_time = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Perform euclidean clustering to group detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (auto& getIndices : cluster_indices)
  {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    for (auto& index : getIndices.indices)
      cluster->points.push_back(cloud->points[index]);

    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    clusters.push_back(cluster);
  }

  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // std::cout << "clustering took " << elapsed_time.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT>
Box ObstacleDetector<PointT>::axisAlignedBoundingBox(const typename pcl::PointCloud<PointT>::ConstPtr& cluster, const int id)
{
  // Find bounding box for one of the clusters
  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  
  const Eigen::Vector3f position((max_pt.x + min_pt.x)/2, (max_pt.y + min_pt.y)/2, (max_pt.z + min_pt.z)/2);
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), (max_pt.z - min_pt.z));

  return Box(id, position, dimension);
}

template <typename PointT>
Box ObstacleDetector<PointT>::pcaBoundingBox(typename pcl::PointCloud<PointT>::Ptr& cluster, const int id)
{
  // Compute the bounding box height (to be used later for recreating the box)
  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  const float box_height = max_pt.z - min_pt.z;
  const float box_z = (max_pt.z + min_pt.z)/2;

  // Compute the cluster centroid 
  Eigen::Vector4f pca_centroid;
  pcl::compute3DCentroid(*cluster, pca_centroid);

  // Squash the cluster to x-y plane with z = centroid z 
  for (size_t i = 0; i < cluster->size(); ++i)
  {
    cluster->points[i].z = pca_centroid(2);
  }

  // Compute principal directions & Transform the original cloud to PCA coordinates
  pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster);
  pca.project(*cluster, *pca_projected_cloud);
  
  const auto eigen_vectors = pca.getEigenVectors();

  // Get the minimum and maximum points of the transformed cloud.
  pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
  const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf quaternion(eigen_vectors); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid.head<3>();
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), box_height);

  return Box(id, position, dimension, quaternion);
}

// ************************* Tracking ***************************
template <typename PointT>
void ObstacleDetector<PointT>::obstacleTracking(const std::vector<Box>& prev_boxes, std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  // Tracking (based on the change in size and displacement between frames)
  
  if (curr_boxes.empty() || prev_boxes.empty())
  {
    return;
  }
  else
  {
    // vectors containing the id of boxes in left and right sets
    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    // Associate Boxes that are similar in two frames
    auto connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) return;

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);

    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);

    for (int j = 0; j < matches.size(); ++j)
    {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);
      
      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j]; // right and matches has the same size
      const auto cur_index = searchBoxIndex(curr_boxes, cur_id);
      
      if (pre_index > -1 && cur_index > -1)
      {
        // change the id of the current box to the same as the previous box
        curr_boxes[cur_index].id = prev_boxes[pre_index].id;
      }
    }
  }
}

template <typename PointT>
bool ObstacleDetector<PointT>::compareBoxes(const Box& a, const Box& b, const float displacement_thresh, const float iou_thresh)
{
  // Percetage Displacements ranging between [0.0, +oo]
  const float dis = sqrt((a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) + (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) + (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

  const float a_max_dim = std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
  const float b_max_dim = std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
  const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

  // Dimension similiarity values between [0.0, 1.0]
  const float x_dim = 2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]);
  const float y_dim = 2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]);
  const float z_dim = 2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]);

  if (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  std::vector<std::vector<int>> connection_pairs;

  for (auto& prev_box : prev_boxes)
  {
    for (auto& curBox : curr_boxes)
    {
      // Add the indecies of a pair of similiar boxes to the matrix
      if (this->compareBoxes(curBox, prev_box, displacement_thresh, iou_thresh))
      {
        connection_pairs.push_back({prev_box.id, curBox.id});
      }
    }
  }

  return connection_pairs;
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right)
{
  // Hash the box ids in the connection_pairs to two vectors(sets), left and right
  for (auto& pair : connection_pairs)
  {
    bool left_found = false;
    for (auto i : left)
    {
      if (i == pair[0])
        left_found = true;
    }
    if (!left_found)
      left.push_back(pair[0]);

    bool right_found = false;
    for (auto j : right)
    {
      if (j == pair[1])
        right_found = true;
    }
    if (!right_found)
      right.push_back(pair[1]);
  }

  std::vector<std::vector<int>> connection_matrix(left.size(), std::vector<int>(right.size(), 0));

  for (auto& pair : connection_pairs)
  {
    int left_index = -1;
    for (int i = 0; i < left.size(); ++i)
    {
      if (pair[0] == left[i])
        left_index = i;
    }

    int right_index = -1;
    for (int i = 0; i < right.size(); ++i)
    {
      if (pair[1] == right[i])
        right_index = i;
    }

    if (left_index != -1 && right_index != -1)
      connection_matrix[left_index][right_index] = 1;
  }

  return connection_matrix;
}

template <typename PointT>
bool ObstacleDetector<PointT>::hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair)
{
  for (int j = 0; j < connection_matrix[0].size(); ++j)
  {
    if (connection_matrix[i][j] == 1 && right_connected[j] == false)
    {
      right_connected[j] = true;

      if (right_pair[j] == -1 || hungarianFind(right_pair[j], connection_matrix, right_connected, right_pair))
      {
        right_pair[j] = i;
        return true;
      }
    }
  }
}

template <typename PointT>
std::vector<int> ObstacleDetector<PointT>::hungarian(const std::vector<std::vector<int>>& connection_matrix)
{
  std::vector<bool> right_connected(connection_matrix[0].size(), false);
  std::vector<int> right_pair(connection_matrix[0].size(), -1);

  int count = 0;
  for (int i = 0; i < connection_matrix.size(); ++i)
  {
    if (hungarianFind(i, connection_matrix, right_connected, right_pair))
      count++;
  }

  std::cout << "For: " << right_pair.size() << " current frame bounding boxes, found: " << count << " matches in previous frame! " << std::endl;

  return right_pair;
}

template <typename PointT>
int ObstacleDetector<PointT>::searchBoxIndex(const std::vector<Box>& boxes, const int id)
{
  for (int i = 0; i < boxes.size(); i++)
  {
    if (boxes[i].id == id)
    return i;
  }

  return -1;
}

} // namespace lidar_obstacle_detector