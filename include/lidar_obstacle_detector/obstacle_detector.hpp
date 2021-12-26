/* ObstacleDetector.hpp

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
template<typename PointT>
class ObstacleDetector
{
 public:
  ObstacleDetector();
  virtual ~ObstacleDetector();
  
 private:
  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster, int id, int colorId);

  BoxQ MinimumBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
  
  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
  // ###################### Start of Project Code ##########################

  // Customized 3D Ransac Algoeithm
  std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

  // Customized ground plane segmentation
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomizedSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

  // Helper function for clustering
  void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol);

  // Helper function for clustering
  std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize);

  // Cluster Obstacles
  std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomizedClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
  
  // ****************** Tracking ***********************
  std::vector<float> getCentroid(const Box& a);

  std::vector<float> getDimension(const Box& a);

  bool compareBoxes(const Box& a, const Box& b, float displacementTol, float dimensionTol);

  // Link nearby bounding boxes between the previous and previous frame
  std::vector<std::vector<int>> associateBoxes(const std::vector<Box>& preBoxes, const std::vector<Box>& curBoxes, float displacementTol, float dimensionTol);

  // connectionMatrix
  std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connectionPairs, std::vector<int>& left, std::vector<int>& right);

  // Helper function for Hungarian Algorithm
  bool hungarianFind(const int i, const std::vector<std::vector<int>>& connectionMatrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);
  
  // Customized Hungarian Algorithm
  std::vector<int> hungarian(const std::vector<std::vector<int>>& connectionMatrix);

  // Helper function for searching the box index in boxes given an id
  int searchBoxIndex(const std::vector<Box>& Boxes, int id);
};

} // namespace lidar_obstacle_detector