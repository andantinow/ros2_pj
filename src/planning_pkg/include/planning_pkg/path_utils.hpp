#ifndef PLANNING_PKG__PATH_UTILS_HPP_
#define PLANNING_PKG__PATH_UTILS_HPP_

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>

namespace planning_pkg
{

/**
 * @brief Calculate distance between two 2D points
 */
inline double distance2d(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Find the closest point on the path to a given position
 * @return Index of the closest point, or -1 if path is empty
 */
int find_closest_point(
  const nav_msgs::msg::Path & path,
  double x, double y,
  int start_idx = 0);

/**
 * @brief Interpolate a point along the path at a given distance from start
 * @param path Input path
 * @param distance Distance along path from first point
 * @return Interpolated pose, or empty pose if path is invalid
 */
geometry_msgs::msg::PoseStamped interpolate_path(
  const nav_msgs::msg::Path & path,
  double distance);

/**
 * @brief Smooth a path using moving average
 * @param path Input path (will be modified)
 * @param window_size Size of smoothing window (must be odd, >= 3)
 */
void smooth_path(nav_msgs::msg::Path & path, int window_size = 5);

/**
 * @brief Resample path to have uniform spacing
 * @param path Input path
 * @param spacing Desired spacing between points (meters)
 * @return Resampled path
 */
nav_msgs::msg::Path resample_path(
  const nav_msgs::msg::Path & path,
  double spacing);

/**
 * @brief Calculate path curvature at a given point
 * @param path Input path
 * @param idx Index of point
 * @return Curvature (1/radius), positive for left turns
 */
double calculate_curvature(
  const nav_msgs::msg::Path & path,
  size_t idx);

/**
 * @brief Validate path (check for NaN, Inf, etc.)
 * @return true if path is valid
 */
bool validate_path(const nav_msgs::msg::Path & path);

/**
 * @brief Calculate total path length
 */
double path_length(const nav_msgs::msg::Path & path);

}  // namespace planning_pkg

#endif  // PLANNING_PKG__PATH_UTILS_HPP_

