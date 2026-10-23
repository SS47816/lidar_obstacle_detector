# Changelog

## 2.0.0

A rewrite around a ROS-agnostic C++ library, with ROS 1 and ROS 2 nodes, new algorithms for every
stage, an optional GPU detector and a reproducible benchmark. The previous ROS 1-only release is
kept at the `v1-ros1-legacy` tag.

### Detection

- **Three detector modes** (`detector_method`):
  - `clustering` (default, CPU only): every obstacle becomes a tracked box. Everything is also
    reported as 2.5D pillars.
  - `learning`: NVIDIA CUDA-CenterPoint objects only, with class labels, scores and full
    headings for 10 classes. Objects the network does not know are not reported.
  - `fused`: CenterPoint objects become tracked boxes, absorbing the clusters that belong to
    them. Everything else becomes untracked 2.5D pillars, so nothing unrecognised is dropped.
  - `centerpoint` and `hybrid` are accepted as aliases of `learning` and `fused`.
- **CUDA-CenterPoint** (TensorRT 10, behind `-DLOD_WITH_TENSORRT=ON`).
  - **Multi-sweep input**: up to 10 ego-motion-compensated scans of the last 0.5 s
    (`detector_sweeps`), as in training.
  - NVIDIA's MIT sources are vendored and ported to TensorRT 10 and the current libspconv API. The
    prebuilt libspconv and the models are downloaded with SHA-256 checks.
  - `ObstacleDetector::setLearnedDetector()` plugs in any other network.
- **Ground segmentation**: Patchwork++ (vendored, BSD-2-Clause), with a local elevation-map
  refinement that repairs ground patches rejected as a whole. A tilt-constrained RANSAC is
  available too.
- **Clustering**:
  - an O(n) bird's-eye-view grid that splits columns at vertical gaps (a tree canopy over a car)
  - range-adaptive Euclidean clustering
  - recursive splitting of clusters larger than an object; what is still too large becomes a
    structure, reported as pillars instead of an object box
  - optional track-guided clustering: the predicted footprints of tracked vehicles re-join
    fragments of long trucks (`cluster_track_guided`)
- **Boxes**: L-shape fitting (Zhang et al., IV 2017), PCA or axis-aligned.

### Tracking and prediction

- Constant-velocity Kalman filter with optimal (Hungarian) association on BEV GIoU (default) or
  centre distance. Velocities are reported in a fixed `tracking_frame` (e.g. `odom`).
- **Heading from motion**: moving, stably tracked objects take their yaw from their velocity, with
  front and back resolved. **Shape completion**: partially visible objects keep the size seen
  earlier in their track.
- **Class-aware tracking**: tracks keep their class and never match across class groups.
- **Trajectory prediction**: every moving track has a 3 s path from a constant turn rate and
  velocity model (`prediction_horizon`, `prediction_step`). It is published as markers and as
  `autoware_perception_msgs/PredictedObjects`.

### Occupancy

- **2.5D pillars** (`~/pillars`): columns from the local ground to the top of every obstacle that
  is not a recognised object. They are fused over time in the tracking frame (`pillar_memory`,
  log-odds with free-space clearing), so static structures stay stable from scan to scan.
- `nav_msgs/OccupancyGrid` with ray-cast free space, and 3D occupied voxels.

### ROS

- ROS 2 (Humble, Jazzy) composable node and ROS 1 (Noetic) node on the same library. All
  parameters can be changed at runtime.
- **ROS 2 outputs**: `vision_msgs/Detection3DArray`, `autoware_perception_msgs/DetectedObjects`
  and `PredictedObjects`, RViz markers, occupancy and pillars.
- **ROS 1 outputs**: jsk boxes, `autoware_msgs` objects, markers, occupancy and pillars. Topic
  parameter names are the original ones and now have defaults.
- **Markers**: recognised classes are drawn as low-poly 3D models (`meshes/`, set `mesh_uri` to
  load them over https in Foxglove), plus predicted paths, pillars and velocity arrows.
- A Foxglove layout (`config/foxglove_layout.json`).

### Tooling

- Presets `legacy` (classic RANSAC + Euclidean + axis-aligned boxes), `fast` and `accurate`. One
  parameter table is shared by ROS 1, ROS 2 and the benchmark CLI.
- A benchmark on Argoverse 2 and nuScenes (`tools/`) with detection, segmentation, tracking,
  coverage and stability metrics.
- Demo renderers: a showcase with map, camera inset and class models, a chase camera and a
  bird's-eye comparison.
- GoogleTest unit tests, CI for Noetic, Humble and Jazzy plus sanitizer builds, and pixi
  environments (`core`, `jazzy`, `noetic`, `gpu`) that need no system ROS install.

### Fixed (compared with the previous release)

- An out-of-bounds read in the tracker when a box had no match.
- The matcher never reset its visited set and ignored distances, so it missed matches and paired
  the wrong boxes.
- The box comparison used a signed size difference, so its IoU gate let almost everything
  through.
- PCA boxes modified the input cloud, could produce reflections, and used the centroid for z.
- Topic parameters were read inside `ROS_ASSERT`, which is compiled out in release builds.
- The voxel grid ran before the ROI crop. The ego-vehicle box was hard-coded.
- Box poses were transformed with the latest TF instead of the one at the cloud stamp.

### Changed

- ROS 1: `use_pca_box`, `displacement_threshold` and `iou_threshold` are replaced by `box_method`
  and the `track_*` parameters.
- C++17 and CMake ≥ 3.16. The package builds with `catkin_make`, `catkin build`, `colcon build`
  or plain CMake.
