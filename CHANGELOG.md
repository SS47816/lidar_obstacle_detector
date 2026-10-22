# Changelog

## 2.0.0 (unreleased)

A rewrite around a ROS-agnostic core library, with ROS 1 and ROS 2 support, new
algorithms for every stage and a reproducible benchmark. The v1 code is preserved at the
`v1-ros1-legacy` tag.

### Added

- **ROS 2** (Humble, Jazzy) composable node `lidar_obstacle_detector::ObstacleDetectorNode`
  publishing `vision_msgs/Detection3DArray`, RViz markers and (when available)
  `autoware_perception_msgs/DetectedObjects`. All parameters are live-tunable.
- **Patchwork++** ground segmentation (vendored, BSD-2-Clause), plus a local elevation-map
  refinement that repairs ground patches rejected as a whole.
- **Clustering**: range-adaptive Euclidean clustering and an O(n) BEV-grid
  connected-components clusterer (20-40x faster than KD-tree clustering).
- **L-shape box fitting** (Zhang et al., IV 2017).
- **Multi-object tracking**: constant-velocity Kalman filter, optimal (Hungarian)
  assignment on centre distance or BEV GIoU, track confirmation and coasting, velocity
  output. Optional fixed `tracking_frame` (e.g. `odom`) so velocities exclude ego motion.
- **Heading from motion**: moving, stably tracked objects take their yaw from the tracked velocity
  (front/back resolved; Autoware `orientation_availability = AVAILABLE`); other tracks keep the
  L-shape yaw. **Shape completion**: partially visible objects keep the size seen earlier in their
  track, anchored at the face towards the sensor.
- **Height-aware BEV clustering** (columns split at vertical gaps, e.g. tree canopy over a car) and
  **recursive splitting** of clusters larger than an object. What is still too large is reported as
  a **structure**: not tracked, not in the object outputs, but in the occupancy outputs and markers.
- **Occupancy outputs**: `nav_msgs/OccupancyGrid` with ray-cast free space (cells occupied by
  obstacle points 0.2-2.5 m above the local ground) and 3D occupied voxels, in ROS 1 and ROS 2.
- GIoU is the default association cost (about 10 % fewer ID switches than centre distance).
- `tools/python/lod_bench/render_demo.py`: chase-camera and side-by-side demo videos from benchmark
  runs.
- Presets `legacy`, `fast` and `accurate`; one parameter table shared by ROS 1, ROS 2 and the
  benchmark CLI.
- Benchmark on Argoverse 2 and nuScenes (`tools/`), GoogleTest unit tests, pixi environments,
  CI for Noetic / Humble / Jazzy plus sanitizer builds.

### Fixed (v1 bugs)

- Out-of-bounds read in `obstacleTracking()` when a box had no match.
- The "Hungarian" matcher never reset its visited set and ignored distances, so it missed
  matches and could pair the wrong boxes.
- `compareBoxes()` used a signed size difference, so every box smaller than its
  predecessor passed the "IoU" gate; the default threshold of 1.0 disabled it anyway.
- PCA boxes modified the input cloud, could produce reflections (invalid rotations), used
  the centroid instead of the extent centre for z, and only compiled for `pcl::PointXYZ`.
- Topic parameters were read inside `ROS_ASSERT`, which is compiled out in release builds.
- The voxel grid ran before the ROI crop; the ego-vehicle box was hard-coded.
- Box poses were transformed with the latest TF instead of the one at the cloud stamp.

### Changed

- ROS 1: `use_pca_box`, `displacement_threshold` and `iou_threshold` are replaced by
  `box_method` and the `track_*` parameters. Topic parameter names are unchanged and now
  have defaults. New optional parameters: `preset`, `tracking_frame`, `markers_topic`.
- With the `fast` and `accurate` presets, oversized clusters (walls, vegetation) are no longer
  boxes in the object outputs but structures. Clusters above `cluster_max_size` points used to be
  dropped silently; they are now split or reported as structures. Set `separate_structures: false`
  to keep them as objects.
- C++17; CMake >= 3.16. The package builds with `catkin_make`, `catkin build` (ROS 1),
  `colcon build` (ROS 2) or plain CMake.
