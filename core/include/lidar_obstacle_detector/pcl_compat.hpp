/* pcl_compat.hpp

 * Copyright (C) 2021-2026 SS47816

 * Compatibility with PCL < 1.11 (e.g. ROS Noetic on Ubuntu 20.04 ships 1.10)

**/

#pragma once

#include <pcl/pcl_config.h>

#include <vector>

#if PCL_VERSION_COMPARE(<, 1, 11, 0)
namespace pcl {
using Indices = std::vector<int>;
}  // namespace pcl
#endif
