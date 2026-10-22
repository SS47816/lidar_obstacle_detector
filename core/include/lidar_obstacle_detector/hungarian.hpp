/* hungarian.hpp

 * Copyright (C) 2021-2026 SS47816

 * Minimum-cost rectangular assignment (Hungarian / Kuhn-Munkres algorithm)

**/

#pragma once

#include <Eigen/Core>
#include <vector>

namespace lidar_obstacle_detector {

// Solves the rectangular linear assignment problem in O(n^2 m).
// Returns, for every row, the assigned column or -1 (only possible when there
// are more rows than columns). Every column is used at most once and the sum
// of the assigned costs is minimal. Callers apply their own gating afterwards.
std::vector<int> solveAssignment(const Eigen::MatrixXd &cost);

}  // namespace lidar_obstacle_detector
