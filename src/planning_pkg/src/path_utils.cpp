#include "planning_pkg/path_utils.hpp"
#include <tf2/utils.h>
#include <limits>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace planning_pkg
{

int find_closest_point(
  const nav_msgs::msg::Path & path,
  double x, double y,
  int start_idx)
{
  if (path.poses.empty()) {
    return -1;
  }

  int closest_idx = start_idx;
  double min_dist_sq = std::numeric_limits<double>::max();

  // Search from start_idx forward
  for (size_t i = start_idx; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - x;
    double dy = path.poses[i].pose.position.y - y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = static_cast<int>(i);
    }
  }

  // Also search backward from start_idx
  for (int i = start_idx - 1; i >= 0; --i) {
    double dx = path.poses[i].pose.position.x - x;
    double dy = path.poses[i].pose.position.y - y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  return closest_idx;
}

geometry_msgs::msg::PoseStamped interpolate_path(
  const nav_msgs::msg::Path & path,
  double distance)
{
  geometry_msgs::msg::PoseStamped result;
  if (path.poses.empty()) {
    return result;
  }

  if (distance <= 0.0) {
    return path.poses[0];
  }

  // Accumulate distance along path
  double accumulated = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    double segment_length = std::sqrt(dx * dx + dy * dy);

    if (accumulated + segment_length >= distance) {
      // Interpolate between path.poses[i-1] and path.poses[i]
      double t = (distance - accumulated) / segment_length;
      result.header = path.header;
      result.pose.position.x = (1.0 - t) * path.poses[i - 1].pose.position.x +
        t * path.poses[i].pose.position.x;
      result.pose.position.y = (1.0 - t) * path.poses[i - 1].pose.position.y +
        t * path.poses[i].pose.position.y;
      result.pose.position.z = (1.0 - t) * path.poses[i - 1].pose.position.z +
        t * path.poses[i].pose.position.z;

      // Interpolate orientation (yaw)
      double yaw1 = tf2::getYaw(path.poses[i - 1].pose.orientation);
      double yaw2 = tf2::getYaw(path.poses[i].pose.orientation);
      double yaw_diff = yaw2 - yaw1;
      // Normalize to [-pi, pi]
      while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
      while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
      double yaw_interp = yaw1 + t * yaw_diff;
      result.pose.orientation.z = std::sin(yaw_interp / 2.0);
      result.pose.orientation.w = std::cos(yaw_interp / 2.0);
      return result;
    }
    accumulated += segment_length;
  }

  // Distance exceeds path length, return last point
  return path.poses.back();
}

void smooth_path(nav_msgs::msg::Path & path, int window_size)
{
  if (path.poses.size() < 3 || window_size < 3) {
    return;
  }

  // Ensure window_size is odd
  if (window_size % 2 == 0) {
    window_size++;
  }

  int half = window_size / 2;
  std::vector<geometry_msgs::msg::PoseStamped> smoothed = path.poses;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    int start = std::max(0, static_cast<int>(i) - half);
    int end = std::min(static_cast<int>(path.poses.size() - 1), static_cast<int>(i) + half);

    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;
    for (int j = start; j <= end; ++j) {
      sum_x += path.poses[j].pose.position.x;
      sum_y += path.poses[j].pose.position.y;
      count++;
    }
    smoothed[i].pose.position.x = sum_x / count;
    smoothed[i].pose.position.y = sum_y / count;
  }

  path.poses = smoothed;
}

nav_msgs::msg::Path resample_path(
  const nav_msgs::msg::Path & path,
  double spacing)
{
  nav_msgs::msg::Path resampled;
  resampled.header = path.header;

  if (path.poses.empty() || spacing <= 0.0) {
    return resampled;
  }

  double total_length = path_length(path);
  for (double d = 0.0; d <= total_length; d += spacing) {
    resampled.poses.push_back(interpolate_path(path, d));
  }

  // Ensure last point is included
  if (!path.poses.empty()) {
    double last_dist = distance2d(
      resampled.poses.back().pose.position.x,
      resampled.poses.back().pose.position.y,
      path.poses.back().pose.position.x,
      path.poses.back().pose.position.y);
    if (last_dist > spacing * 0.5) {
      resampled.poses.push_back(path.poses.back());
    }
  }

  return resampled;
}

double calculate_curvature(
  const nav_msgs::msg::Path & path,
  size_t idx)
{
  if (path.poses.size() < 3 || idx == 0 || idx >= path.poses.size() - 1) {
    return 0.0;
  }

  const auto & p0 = path.poses[idx - 1].pose.position;
  const auto & p1 = path.poses[idx].pose.position;
  const auto & p2 = path.poses[idx + 1].pose.position;

  double dx1 = p1.x - p0.x;
  double dy1 = p1.y - p0.y;
  double dx2 = p2.x - p1.x;
  double dy2 = p2.y - p1.y;

  double cross = dx1 * dy2 - dy1 * dx2;
  double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double ds2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

  if (ds1 < 1e-6 || ds2 < 1e-6) {
    return 0.0;
  }

  double avg_ds = (ds1 + ds2) / 2.0;
  return cross / (ds1 * ds2 * avg_ds);
}

bool validate_path(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return false;
  }

  for (const auto & pose : path.poses) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return false;
    }
  }

  return true;
}

double path_length(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    length += distance2d(
      path.poses[i - 1].pose.position.x,
      path.poses[i - 1].pose.position.y,
      path.poses[i].pose.position.x,
      path.poses[i].pose.position.y);
  }
  return length;
}

}  // namespace planning_pkg

