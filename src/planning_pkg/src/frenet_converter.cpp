/**
 * @file frenet_converter.cpp
 * @brief Implementation of FrenetConverter class for Frenet coordinate conversions
 */

#include "planning_pkg/opponent_detector.hpp"
#include <algorithm>

namespace planning_pkg
{

// Constants for numerical stability
constexpr double MIN_SEGMENT_LENGTH = 1e-6;

void FrenetConverter::setReferencePath(const nav_msgs::msg::Path& path)
{
    if (path.poses.empty()) {
        path_initialized_ = false;
        return;
    }
    
    reference_path_ = path;
    cumulative_distances_.clear();
    cumulative_distances_.reserve(path.poses.size());
    
    // Calculate cumulative distances (s values)
    cumulative_distances_.push_back(0.0);
    for (size_t i = 1; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        double ds = std::sqrt(dx * dx + dy * dy);  // Use direct calculation for performance
        cumulative_distances_.push_back(cumulative_distances_.back() + ds);
    }
    
    total_path_length_ = cumulative_distances_.back();
    path_initialized_ = true;
}

int FrenetConverter::findClosestPoint(double x, double y, int hint_idx) const
{
    if (!path_initialized_ || reference_path_.poses.empty()) {
        return 0;
    }
    
    int n = static_cast<int>(reference_path_.poses.size());
    hint_idx = std::clamp(hint_idx, 0, n - 1);
    
    // Local search around hint
    int search_range = 50;
    int start = std::max(0, hint_idx - search_range);
    int end = std::min(n, hint_idx + search_range);
    
    double min_dist_sq = std::numeric_limits<double>::max();
    int closest_idx = hint_idx;
    
    for (int i = start; i < end; ++i) {
        double dx = reference_path_.poses[i].pose.position.x - x;
        double dy = reference_path_.poses[i].pose.position.y - y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }
    
    // If local search didn't find a close match, do global search
    if (min_dist_sq > 4.0) {  // > 2m from hint
        for (int i = 0; i < n; ++i) {
            double dx = reference_path_.poses[i].pose.position.x - x;
            double dy = reference_path_.poses[i].pose.position.y - y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
    }
    
    return closest_idx;
}

FrenetCoord FrenetConverter::toFrenet(double x, double y, int hint_idx) const
{
    FrenetCoord result;
    
    if (!path_initialized_ || reference_path_.poses.size() < 2) {
        return result;
    }
    
    // Find closest point on path
    int closest_idx = findClosestPoint(x, y, hint_idx);
    result.path_idx = closest_idx;
    
    // Get s value at closest point
    result.s = cumulative_distances_[closest_idx];
    
    // Calculate path heading at this point
    int n = static_cast<int>(reference_path_.poses.size());
    int next_idx = (closest_idx + 1) % n;
    
    double path_dx = reference_path_.poses[next_idx].pose.position.x - 
                     reference_path_.poses[closest_idx].pose.position.x;
    double path_dy = reference_path_.poses[next_idx].pose.position.y - 
                     reference_path_.poses[closest_idx].pose.position.y;
    result.psi = std::atan2(path_dy, path_dx);
    
    // Calculate lateral deviation d
    // d is positive to the left of the path (perpendicular distance)
    double dx = x - reference_path_.poses[closest_idx].pose.position.x;
    double dy = y - reference_path_.poses[closest_idx].pose.position.y;
    
    // Project onto normal vector (perpendicular to path heading)
    // Normal vector: (-sin(psi), cos(psi))
    result.d = -std::sin(result.psi) * dx + std::cos(result.psi) * dy;
    
    return result;
}

geometry_msgs::msg::Point FrenetConverter::toCartesian(double s, double d) const
{
    geometry_msgs::msg::Point result;
    
    if (!path_initialized_ || reference_path_.poses.empty()) {
        return result;
    }
    
    // Handle s wrapping for closed tracks
    double s_mod = std::fmod(s, total_path_length_);
    if (s_mod < 0) s_mod += total_path_length_;
    
    // Find the path segment containing s
    int idx = 0;
    for (size_t i = 1; i < cumulative_distances_.size(); ++i) {
        if (cumulative_distances_[i] >= s_mod) {
            idx = static_cast<int>(i) - 1;
            break;
        }
    }
    
    // Interpolation factor within segment
    double seg_start = cumulative_distances_[idx];
    double seg_length = cumulative_distances_[idx + 1] - seg_start;
    double t = (seg_length > MIN_SEGMENT_LENGTH) ? (s_mod - seg_start) / seg_length : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    
    // Interpolate centerline position
    double cx = reference_path_.poses[idx].pose.position.x * (1.0 - t) +
                reference_path_.poses[idx + 1].pose.position.x * t;
    double cy = reference_path_.poses[idx].pose.position.y * (1.0 - t) +
                reference_path_.poses[idx + 1].pose.position.y * t;
    
    // Get path heading
    double psi = getHeadingAtS(s_mod);
    
    // Apply lateral offset (d) perpendicular to path
    result.x = cx - d * std::sin(psi);
    result.y = cy + d * std::cos(psi);
    result.z = 0.0;
    
    return result;
}

double FrenetConverter::getHeadingAtS(double s) const
{
    if (!path_initialized_ || reference_path_.poses.size() < 2) {
        return 0.0;
    }
    
    double s_mod = std::fmod(s, total_path_length_);
    if (s_mod < 0) s_mod += total_path_length_;
    
    // Find segment
    int idx = 0;
    for (size_t i = 1; i < cumulative_distances_.size(); ++i) {
        if (cumulative_distances_[i] >= s_mod) {
            idx = static_cast<int>(i) - 1;
            break;
        }
    }
    
    int n = static_cast<int>(reference_path_.poses.size());
    int next_idx = (idx + 1) % n;
    
    double dx = reference_path_.poses[next_idx].pose.position.x - 
                reference_path_.poses[idx].pose.position.x;
    double dy = reference_path_.poses[next_idx].pose.position.y - 
                reference_path_.poses[idx].pose.position.y;
    
    return std::atan2(dy, dx);
}

}  // namespace planning_pkg
