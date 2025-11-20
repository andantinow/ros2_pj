#pragma once
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>

namespace path_utils {

inline double normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline size_t nearest_index(const nav_msgs::msg::Path& path, double x, double y) {
  size_t best_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - x;
    double dy = path.poses[i].pose.position.y - y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = i;
    }
  }
  return best_idx;
}

class RateLimiter {
public:
  explicit RateLimiter(double rate_limit) : rate_limit_(rate_limit), last_value_(0.0) {}
  
  double apply(double target, double dt) {
    double delta = target - last_value_;
    double max_delta = rate_limit_ * dt;
    delta = std::clamp(delta, -max_delta, max_delta);
    last_value_ += delta;
    return last_value_;
  }
  
  void reset(double value = 0.0) { last_value_ = value; }

private:
  double rate_limit_;
  double last_value_;
};

inline double clamp(double value, double min_val, double max_val) {
  return std::max(min_val, std::min(value, max_val));
}

inline double compute_curvature(const nav_msgs::msg::Path& path, size_t idx) {
  if (idx == 0 || idx >= path.poses.size() - 1) return 0.0;
  
  const auto& p0 = path.poses[idx - 1].pose.position;
  const auto& p1 = path.poses[idx].pose.position;
  const auto& p2 = path.poses[idx + 1].pose.position;
  
  double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
  double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
  double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
  
  if (a < 1e-6 || b < 1e-6 || c < 1e-6) return 0.0;
  
  double s = (a + b + c) / 2.0;
  double area_sq = s * (s - a) * (s - b) * (s - c);
  if (area_sq < 0.0) return 0.0;
  
  double area = std::sqrt(area_sq);
  return (4.0 * area) / (a * b * c + 1e-9);
}

}

