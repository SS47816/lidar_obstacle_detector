cmake_minimum_required(VERSION 3.16)
project(lidar_obstacle_detector VERSION 2.0.0 LANGUAGES CXX)

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

# ------------------------------------------------------------------------------
# Benchmark CLI (plain CMake only)
# ------------------------------------------------------------------------------
if(LOD_BUILD_TOOLS)
  find_package(nlohmann_json REQUIRED)
  add_executable(lod_benchmark tools/lod_benchmark.cpp)
  target_include_directories(lod_benchmark PRIVATE tools legacy/include ${PCL_INCLUDE_DIRS})
  target_link_libraries(lod_benchmark PRIVATE ${PCL_LIBRARIES} Eigen3::Eigen nlohmann_json::nlohmann_json)
endif()
