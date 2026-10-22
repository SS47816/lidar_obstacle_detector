/* hungarian.cpp

 * Copyright (C) 2021-2026 SS47816

 * Shortest augmenting path Hungarian algorithm with row/column potentials

**/

#include "lidar_obstacle_detector/hungarian.hpp"

#include <limits>

namespace lidar_obstacle_detector {

std::vector<int> solveAssignment(const Eigen::MatrixXd &cost) {
  const bool transposed = cost.rows() > cost.cols();
  const Eigen::MatrixXd a = transposed ? Eigen::MatrixXd(cost.transpose()) : cost;
  const int n = static_cast<int>(a.rows());  // n <= m
  const int m = static_cast<int>(a.cols());
  std::vector<int> result(cost.rows(), -1);
  if (n == 0 || m == 0) return result;

  constexpr double kInf = std::numeric_limits<double>::infinity();
  // 1-based arrays; p[j] = row matched to column j, way[j] = previous column
  std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(m + 1, kInf);
    std::vector<bool> used(m + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = kInf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) continue;
        const double cur = a(i0 - 1, j - 1) - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  for (int j = 1; j <= m; ++j) {
    if (p[j] == 0) continue;
    if (transposed)
      result[j - 1] = p[j] - 1;
    else
      result[p[j] - 1] = j - 1;
  }
  return result;
}

}  // namespace lidar_obstacle_detector
