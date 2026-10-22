#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>
#include <random>

#include "lidar_obstacle_detector/hungarian.hpp"

using lidar_obstacle_detector::solveAssignment;

namespace {

// Brute force over all injective maps of the smaller side into the larger one
double bruteForce(const Eigen::MatrixXd &c) {
  const bool t = c.rows() > c.cols();
  const Eigen::MatrixXd a = t ? Eigen::MatrixXd(c.transpose()) : c;
  std::vector<int> cols(a.cols());
  std::iota(cols.begin(), cols.end(), 0);
  double best = std::numeric_limits<double>::max();
  do {
    double s = 0.0;
    for (int i = 0; i < a.rows(); ++i) s += a(i, cols[i]);
    best = std::min(best, s);
  } while (std::next_permutation(cols.begin(), cols.end()));
  return best;
}

double assignedCost(const Eigen::MatrixXd &c, const std::vector<int> &a) {
  double s = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    if (a[i] >= 0) s += c(i, a[i]);
  return s;
}

}  // namespace

TEST(Hungarian, EmptyInputs) {
  EXPECT_TRUE(solveAssignment(Eigen::MatrixXd(0, 3)).empty());
  const auto a = solveAssignment(Eigen::MatrixXd(2, 0));
  ASSERT_EQ(a.size(), 2u);
  EXPECT_EQ(a[0], -1);
  EXPECT_EQ(a[1], -1);
}

TEST(Hungarian, PrefersGlobalOptimumOverGreedy) {
  // Greedy would take (0,0)=1 and then be forced into (1,1)=100
  Eigen::MatrixXd c(2, 2);
  c << 1, 2, 3, 100;
  const auto a = solveAssignment(c);
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 0);
}

TEST(Hungarian, MatchesBruteForceOnRandomRectangularMatrices) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(0.0, 10.0);
  for (int trial = 0; trial < 300; ++trial) {
    const int rows = 1 + trial % 7, cols = 1 + (trial / 7) % 7;
    Eigen::MatrixXd c(rows, cols);
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j) c(i, j) = dist(rng);
    const auto a = solveAssignment(c);
    ASSERT_EQ(static_cast<int>(a.size()), rows);
    // valid: each column used at most once, min(rows, cols) assignments
    std::vector<int> used;
    for (int j : a)
      if (j >= 0) used.push_back(j);
    std::sort(used.begin(), used.end());
    EXPECT_EQ(std::adjacent_find(used.begin(), used.end()), used.end());
    EXPECT_EQ(static_cast<int>(used.size()), std::min(rows, cols));
    EXPECT_NEAR(assignedCost(c, a), bruteForce(c), 1e-9) << rows << "x" << cols;
  }
}
