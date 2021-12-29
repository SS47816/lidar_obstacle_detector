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

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

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

  typename pcl::PointCloud<PointT>::Ptr FilterCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint);
  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const int maxIterations, const float distanceThreshold);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float clusterTolerance, const int minSize, const int maxSize);

  Box BoundingBox(const typename pcl::PointCloud<PointT>::ConstPtr& cluster, const int id);

  Box MinimumBoundingBox(const typename pcl::PointCloud<PointT>::ConstPtr& cluster, const int id);

  // ****************** Tracking ***********************

  void obstacleTracking(const std::vector<Box>& prev_boxes, std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh);

 private:

  // ****************** Detection ***********************
  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(const pcl::PointIndices::ConstPtr& inliers, const typename pcl::PointCloud<PointT>::ConstPtr& cloud);

  // ****************** Tracking ***********************

  bool compareBoxes(const Box& a, const Box& b, const float displacement_thresh, const float iou_thresh);

  // Link nearby bounding boxes between the previous and previous frame
  std::vector<std::vector<int>> associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curBoxes, const float displacement_thresh, const float iou_thresh);

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
typename pcl::PointCloud<PointT>::Ptr ObstacleDetector<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint)
{
  // Time segmentation process
  const auto startTime = std::chrono::steady_clock::now();

  // Create the filtering object: downsample the dataset using a leaf size
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  // Cropping the ROI
  typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  // Removing the car roof region
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto& point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  const auto endTime = std::chrono::steady_clock::now();
  const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::SeparateClouds(const pcl::PointIndices::ConstPtr& inliers, const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

  // Pushback all the inliers into the planeCloud
  for (int index : inliers->indices)
  {
    planeCloud->points.push_back(cloud->points[index]);
  }

  // Extract the points that are not in the inliers to obstCloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstCloud, planeCloud);
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const int maxIterations, const float distanceThreshold)
{
  // Time segmentation process
  const auto startTime = std::chrono::steady_clock::now();

  // Find inliers for the cloud.
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty())
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  const auto endTime = std::chrono::steady_clock::now();
  const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  return SeparateClouds(inliers, cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::Clustering(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float clusterTolerance, const int minSize, const int maxSize)
{

  // Time clustering process
  const auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Perform euclidean clustering to group detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  for (auto& getIndices : clusterIndices)
  {
    typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

    for (auto& index : getIndices.indices)
      cloudCluster->points.push_back(cloud->points[index]);

    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    clusters.push_back(cloudCluster);
  }

  const auto endTime = std::chrono::steady_clock::now();
  const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT>
Box ObstacleDetector<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::ConstPtr& cluster, const int id)
{
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);
  
  const Eigen::Vector3f position((maxPoint.x + minPoint.x)/2, (maxPoint.y + minPoint.y)/2, (maxPoint.z + minPoint.z)/2);
  const Eigen::Vector3f dimension((maxPoint.x - minPoint.x), (maxPoint.y - minPoint.y), (maxPoint.z - minPoint.z));

  return Box(id, position, dimension);
}

template <typename PointT>
Box ObstacleDetector<PointT>::MinimumBoundingBox(const typename pcl::PointCloud<PointT>::ConstPtr& cluster, const int id)
{
  // Find bounding box for one of the clusters

  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cluster, pcaCentroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                  ///    the signs are different and the box doesn't get correctly oriented in some cases.
  // TODO: Limit the PCA to the grond plane
  // eigenVectorsPCA.row(0).col(2) << 1.0f;
  // eigenVectorsPCA.row(2).col(0) << 1.0f;
  // // eigenVectorsPCA.row(2).col(2) << 0.0f;
  // eigenVectorsPCA.row(2).col(1) << 1.0f;
  // eigenVectorsPCA.row(1).col(2) << 1.0f;

  // Print Matrix
  std::cout << "eigenVectorsPCA: " << std::endl;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      std::cout << eigenVectorsPCA.row(i).col(j) << ", ";
    }
    std::cout << std::endl;
  }

  /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster);
  pca.project(*cluster, *cloudPCAprojection);
  std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
  std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
  // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
  */

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf quaternion(eigenVectorsPCA); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f position = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
  const Eigen::Vector3f dimension((maxPoint.x - minPoint.x), (maxPoint.y - minPoint.y), (maxPoint.z - minPoint.z));

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
std::vector<std::vector<int>> ObstacleDetector<PointT>::associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curBoxes, const float displacement_thresh, const float iou_thresh)
{
  std::vector<std::vector<int>> connection_pairs;

  for (auto& prev_box : prev_boxes)
  {
    for (auto& curBox : curBoxes)
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

  std::cout << "For: " << right_pair.size() << " current-frame bounding boxes, found: " << count << " matches in previous frame! " << std::endl;

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