cmake_minimum_required(VERSION 3.16)
project(lidar_obstacle_detector VERSION 2.0.0 LANGUAGES C CXX)  # C: FindMPI (via PCL -> VTK) needs it

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Build mode: catkin (ROS 1), ament (ROS 2) or plain CMake (no ROS)
set(LOD_ROS_VERSION "$ENV{ROS_VERSION}" CACHE STRING "1, 2 or empty (plain CMake)")

option(LOD_BUILD_TESTS "Build unit tests" OFF)
option(LOD_BUILD_TOOLS "Build the offline benchmark CLI" OFF)
option(LOD_WITH_OPENMP "Parallelise per-cluster work with OpenMP" ON)
option(LOD_SANITIZE "Build with address & undefined-behaviour sanitizers" OFF)

if(LOD_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search kdtree sample_consensus segmentation)
if(LOD_WITH_OPENMP)
  find_package(OpenMP)
endif()

# ------------------------------------------------------------------------------
# Vendored Patchwork++ (BSD-2-Clause), see third_party/patchworkpp/VENDORED.md
# ------------------------------------------------------------------------------
add_library(lod_patchworkpp STATIC
  third_party/patchworkpp/src/patchworkpp.cpp
  third_party/patchworkpp/src/plane_fit.cpp
)
set_target_properties(lod_patchworkpp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lod_patchworkpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/patchworkpp/include>)
target_link_libraries(lod_patchworkpp PUBLIC Eigen3::Eigen)
target_compile_options(lod_patchworkpp PRIVATE -w)  # third-party code, keep our build quiet

# ------------------------------------------------------------------------------
# ROS-agnostic core library
# ------------------------------------------------------------------------------
add_library(lidar_obstacle_detector_core
  core/src/config.cpp
  core/src/hungarian.cpp
  core/src/obstacle_detector.cpp
  core/src/occupancy.cpp
  core/src/patchworkpp_segmenter.cpp
  core/src/tracker.cpp
  core/src/types.cpp
)
set_target_properties(lidar_obstacle_detector_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lidar_obstacle_detector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(lidar_obstacle_detector_core PUBLIC ${PCL_DEFINITIONS})
target_compile_options(lidar_obstacle_detector_core PRIVATE -Wall -Wextra)
target_link_libraries(lidar_obstacle_detector_core
  PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen
  PRIVATE $<BUILD_INTERFACE:lod_patchworkpp>
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(lidar_obstacle_detector_core PUBLIC OpenMP::OpenMP_CXX)
endif()

set(LOD_TEST_SOURCES
  test/test_box_fitting.cpp
  test/test_clustering.cpp
  test/test_config.cpp
  test/test_geometry.cpp
  test/test_hungarian.cpp
  test/test_occupancy.cpp
  test/test_pipeline.cpp
  test/test_tracker.cpp
)

# ------------------------------------------------------------------------------
# ROS 2 (ament): composable node + standalone executable
# ------------------------------------------------------------------------------
if(LOD_ROS_VERSION STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(vision_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(autoware_perception_msgs QUIET)

  add_library(obstacle_detector_component SHARED ros2/obstacle_detector_node.cpp)
  target_link_libraries(obstacle_detector_component
    lidar_obstacle_detector_core
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    tf2_ros::tf2_ros
    tf2_eigen::tf2_eigen
    ${pcl_conversions_LIBRARIES}
  )
  # pcl_conversions exports no CMake target on all distros
  target_include_directories(obstacle_detector_component PRIVATE ${pcl_conversions_INCLUDE_DIRS})
  if(autoware_perception_msgs_FOUND)
    target_compile_definitions(obstacle_detector_component PRIVATE LOD_HAVE_AUTOWARE)
    target_link_libraries(obstacle_detector_component ${autoware_perception_msgs_TARGETS})
    message(STATUS "lidar_obstacle_detector: publishing autoware_perception_msgs/DetectedObjects")
  endif()
  rclcpp_components_register_node(obstacle_detector_component
    PLUGIN "lidar_obstacle_detector::ObstacleDetectorNode"
    EXECUTABLE obstacle_detector_node)

  install(TARGETS obstacle_detector_component lidar_obstacle_detector_core
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY core/include/ DESTINATION include)
  install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}
    PATTERN "*.launch" EXCLUDE PATTERN "lidar_obstacle_detector.rviz" EXCLUDE PATTERN "mai_city.rviz" EXCLUDE)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(lod_tests ${LOD_TEST_SOURCES})
    target_link_libraries(lod_tests lidar_obstacle_detector_core)
  endif()
  ament_package()

# ------------------------------------------------------------------------------
# ROS 1 (catkin): node with dynamic_reconfigure
# ------------------------------------------------------------------------------
elseif(LOD_ROS_VERSION STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs visualization_msgs nav_msgs pcl_conversions tf2_ros tf2_eigen
    dynamic_reconfigure jsk_recognition_msgs)
  find_package(autoware_msgs QUIET)
  generate_dynamic_reconfigure_options(cfg/obstacle_detector.cfg)
  catkin_package(
    INCLUDE_DIRS core/include
    LIBRARIES lidar_obstacle_detector_core
    CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs nav_msgs jsk_recognition_msgs
  )
  add_executable(obstacle_detector_node ros1/obstacle_detector_node.cpp)
  target_include_directories(obstacle_detector_node PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(obstacle_detector_node lidar_obstacle_detector_core ${catkin_LIBRARIES})
  add_dependencies(obstacle_detector_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
  if(autoware_msgs_FOUND)
    target_compile_definitions(obstacle_detector_node PRIVATE LOD_HAVE_AUTOWARE)
    target_include_directories(obstacle_detector_node PRIVATE ${autoware_msgs_INCLUDE_DIRS})
    add_dependencies(obstacle_detector_node ${autoware_msgs_EXPORTED_TARGETS})
    message(STATUS "lidar_obstacle_detector: publishing autoware_msgs/DetectedObjectArray")
  endif()

  install(TARGETS obstacle_detector_node lidar_obstacle_detector_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY core/include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*.launch.py" EXCLUDE PATTERN "*ros2*" EXCLUDE)

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(lod_tests ${LOD_TEST_SOURCES} test/gtest_main.cpp)
    target_link_libraries(lod_tests lidar_obstacle_detector_core)
  endif()
endif()

# ------------------------------------------------------------------------------
# Unit tests & benchmark CLI (plain CMake)
# ------------------------------------------------------------------------------
if(LOD_BUILD_TESTS AND NOT LOD_ROS_VERSION)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(lod_tests ${LOD_TEST_SOURCES})
  target_link_libraries(lod_tests PRIVATE lidar_obstacle_detector_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(lod_tests)
endif()

if(LOD_BUILD_TOOLS)
  find_package(nlohmann_json REQUIRED)
  add_executable(lod_benchmark tools/lod_benchmark.cpp)
  target_include_directories(lod_benchmark PRIVATE tools legacy/include)
  target_compile_definitions(lod_benchmark PRIVATE LOD_HAVE_CORE)
  target_link_libraries(lod_benchmark PRIVATE lidar_obstacle_detector_core nlohmann_json::nlohmann_json)
endif()
