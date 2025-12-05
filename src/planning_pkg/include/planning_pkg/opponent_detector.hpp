/**
 * @file opponent_detector.hpp
 * @brief Opponent/Leading vehicle detection using LiDAR + odometry + raceline
 * 
 * This header defines structures and utilities for detecting and tracking
 * the leading vehicle (opponent) in front of the ego vehicle, using:
 *   - LiDAR scan data (for obstacle detection)
 *   - Odometry (for ego vehicle pose)
 *   - Global raceline (for Frenet coordinate conversion)
 * 
 * Output: Opponent position in both Cartesian (x, y) and Frenet (s, d) coordinates
 * 
 * Coordinate System:
 *   s: Longitudinal distance along raceline
 *   d: Lateral deviation from raceline (+left, -right in vehicle frame)
 */

#ifndef PLANNING_PKG__OPPONENT_DETECTOR_HPP_
#define PLANNING_PKG__OPPONENT_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <optional>

namespace planning_pkg
{

/**
 * @brief Frenet coordinate representation
 */
struct FrenetCoord
{
    double s;       // Longitudinal position along path
    double d;       // Lateral deviation from path (+left, -right)
    double psi;     // Heading angle at this s position
    int path_idx;   // Index on the reference path
    
    FrenetCoord() : s(0.0), d(0.0), psi(0.0), path_idx(0) {}
    FrenetCoord(double s_, double d_, double psi_ = 0.0, int idx = 0)
        : s(s_), d(d_), psi(psi_), path_idx(idx) {}
};

/**
 * @brief Detected opponent information
 */
struct OpponentInfo
{
    // Cartesian coordinates (map frame)
    double x;
    double y;
    
    // Frenet coordinates (relative to raceline)
    double s;
    double d;
    
    // Distance and angle from ego vehicle
    double distance;        // Euclidean distance to ego
    double angle;           // Angle from ego heading (rad)
    
    // Velocity estimation (optional)
    double estimated_speed;
    
    // Validity flag
    bool valid;
    
    // Timestamp
    rclcpp::Time timestamp;
    
    OpponentInfo() 
        : x(0.0), y(0.0), s(0.0), d(0.0), distance(0.0), angle(0.0),
          estimated_speed(0.0), valid(false), timestamp(0, 0, RCL_ROS_TIME) {}
};

/**
 * @brief Ego vehicle state in Frenet coordinates
 */
struct EgoState
{
    double x, y;        // Cartesian position
    double yaw;         // Heading
    double speed;       // Current speed
    double s, d;        // Frenet coordinates
    int path_idx;       // Closest path index
    
    EgoState() : x(0.0), y(0.0), yaw(0.0), speed(0.0), s(0.0), d(0.0), path_idx(0) {}
};

/**
 * @brief Utility class for Frenet coordinate conversions
 */
class FrenetConverter
{
public:
    FrenetConverter() : path_initialized_(false), total_path_length_(0.0) {}
    
    /**
     * @brief Set the reference path (raceline)
     */
    void setReferencePath(const nav_msgs::msg::Path& path);
    
    /**
     * @brief Convert Cartesian (x, y) to Frenet (s, d)
     * @param x Cartesian x coordinate
     * @param y Cartesian y coordinate
     * @param hint_idx Starting index hint for search (optional, for efficiency)
     * @return Frenet coordinates
     */
    FrenetCoord toFrenet(double x, double y, int hint_idx = 0) const;
    
    /**
     * @brief Convert Frenet (s, d) to Cartesian (x, y)
     * @param s Longitudinal position
     * @param d Lateral deviation
     * @return Point in Cartesian coordinates
     */
    geometry_msgs::msg::Point toCartesian(double s, double d) const;
    
    /**
     * @brief Get path heading (psi) at given s position
     */
    double getHeadingAtS(double s) const;
    
    /**
     * @brief Check if path is initialized
     */
    bool isInitialized() const { return path_initialized_; }
    
    /**
     * @brief Get total path length
     */
    double getTotalPathLength() const { return total_path_length_; }
    
    /**
     * @brief Get reference path
     */
    const nav_msgs::msg::Path& getPath() const { return reference_path_; }

private:
    nav_msgs::msg::Path reference_path_;
    std::vector<double> cumulative_distances_;  // s values at each path point
    bool path_initialized_;
    double total_path_length_;
    
    int findClosestPoint(double x, double y, int hint_idx = 0) const;
};

/**
 * @brief Opponent detection parameters
 */
struct OpponentDetectorParams
{
    // LiDAR scan filtering
    double front_angle_min = -0.26;     // -15 degrees in radians
    double front_angle_max = 0.26;      // +15 degrees in radians
    double min_valid_range = 0.05;      // Minimum valid LiDAR range (m)
    double max_detection_range = 10.0;  // Maximum detection range (m)
    
    // Opponent validation
    double lane_half_width = 0.6;       // Half lane width for opponent validation (m)
    double min_ahead_margin = 0.3;      // Minimum s difference to consider as "ahead" (m)
    double max_ahead_distance = 5.0;    // Maximum s difference to track opponent (m)
    
    // Visualization
    bool enable_visualization = true;
    std::string frame_id = "map";
};

}  // namespace planning_pkg

#endif  // PLANNING_PKG__OPPONENT_DETECTOR_HPP_
