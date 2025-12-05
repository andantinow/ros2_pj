/**
 * @file opponent_detector_node.cpp
 * @brief ROS2 Node for opponent/leading vehicle detection
 * 
 * This node implements a LiDAR-based opponent detection system that:
 * 1. Subscribes to /scan, /odom, and /global_raceline
 * 2. Detects the closest obstacle in the front narrow angle sector
 * 3. Converts obstacle position to Frenet coordinates (s, d)
 * 4. Publishes opponent info and visualization markers
 * 
 * Key features:
 * - Frenet coordinate conversion for raceline-relative positioning
 * - Opponent validation (must be ahead on raceline, within lane bounds)
 * - RViz visualization with markers showing opponent position
 * 
 * Topics:
 *   Input:
 *     /scan (sensor_msgs/LaserScan)
 *     /odom (nav_msgs/Odometry)
 *     /global_raceline (nav_msgs/Path)
 *   Output:
 *     /opponent_info (std_msgs/Float64MultiArray) - [x, y, s, d, distance, angle, valid]
 *     /opponent_marker (visualization_msgs/MarkerArray) - RViz visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>

#include "planning_pkg/opponent_detector.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace planning_pkg
{

// ============================================================================
// FrenetConverter Implementation
// ============================================================================

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

// ============================================================================
// Opponent Detector Node
// ============================================================================

class OpponentDetectorNode : public rclcpp::Node
{
public:
    OpponentDetectorNode()
    : Node("opponent_detector")
    {
        // Parameters
        this->declare_parameter<double>("front_angle_min", -0.26);  // -15 deg
        this->declare_parameter<double>("front_angle_max", 0.26);   // +15 deg
        this->declare_parameter<double>("min_valid_range", 0.05);
        this->declare_parameter<double>("max_detection_range", 10.0);
        this->declare_parameter<double>("lane_half_width", 0.6);
        this->declare_parameter<double>("min_ahead_margin", 0.3);
        this->declare_parameter<double>("max_ahead_distance", 5.0);
        this->declare_parameter<bool>("enable_visualization", true);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<double>("detection_rate", 20.0);
        
        loadParameters();
        
        // Publishers
        opponent_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/opponent_info", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/opponent_marker", 10);
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&OpponentDetectorNode::scanCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10).best_effort(),
            std::bind(&OpponentDetectorNode::odomCallback, this, std::placeholders::_1));
        
        // Path subscription with transient_local for latched message
        rclcpp::QoS path_qos(rclcpp::QoS(10).transient_local().reliable());
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_raceline", path_qos,
            std::bind(&OpponentDetectorNode::pathCallback, this, std::placeholders::_1));
        
        // Detection timer
        double rate = this->get_parameter("detection_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&OpponentDetectorNode::detectionLoop, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Opponent Detector initialized. Front angle: [%.2f, %.2f] deg",
            params_.front_angle_min * 180.0 / M_PI,
            params_.front_angle_max * 180.0 / M_PI);
    }

private:
    // Data
    FrenetConverter frenet_converter_;
    OpponentDetectorParams params_;
    EgoState ego_state_;
    OpponentInfo current_opponent_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan current_scan_;
    nav_msgs::msg::Odometry current_odom_;
    bool scan_received_ = false;
    bool odom_received_ = false;
    bool path_received_ = false;
    
    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr opponent_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void loadParameters()
    {
        params_.front_angle_min = this->get_parameter("front_angle_min").as_double();
        params_.front_angle_max = this->get_parameter("front_angle_max").as_double();
        params_.min_valid_range = this->get_parameter("min_valid_range").as_double();
        params_.max_detection_range = this->get_parameter("max_detection_range").as_double();
        params_.lane_half_width = this->get_parameter("lane_half_width").as_double();
        params_.min_ahead_margin = this->get_parameter("min_ahead_margin").as_double();
        params_.max_ahead_distance = this->get_parameter("max_ahead_distance").as_double();
        params_.enable_visualization = this->get_parameter("enable_visualization").as_bool();
        params_.frame_id = this->get_parameter("frame_id").as_string();
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_scan_ = *msg;
        scan_received_ = true;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        odom_received_ = true;
        
        // Update ego state
        ego_state_.x = msg->pose.pose.position.x;
        ego_state_.y = msg->pose.pose.position.y;
        ego_state_.yaw = tf2::getYaw(msg->pose.pose.orientation);
        ego_state_.speed = std::hypot(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y);
        
        // Convert ego to Frenet if path is available
        if (frenet_converter_.isInitialized()) {
            FrenetCoord ego_frenet = frenet_converter_.toFrenet(
                ego_state_.x, ego_state_.y, ego_state_.path_idx);
            ego_state_.s = ego_frenet.s;
            ego_state_.d = ego_frenet.d;
            ego_state_.path_idx = ego_frenet.path_idx;
        }
    }
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty raceline path");
            return;
        }
        
        frenet_converter_.setReferencePath(*msg);
        path_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
            "Raceline received: %zu points, total length: %.2f m",
            msg->poses.size(), frenet_converter_.getTotalPathLength());
    }
    
    void detectionLoop()
    {
        if (!scan_received_ || !odom_received_ || !path_received_) {
            return;
        }
        
        // Detect opponent from LiDAR
        OpponentInfo opponent = detectOpponent();
        
        // Publish opponent info
        publishOpponentInfo(opponent);
        
        // Publish visualization
        if (params_.enable_visualization) {
            publishVisualization(opponent);
        }
        
        current_opponent_ = opponent;
    }
    
    OpponentInfo detectOpponent()
    {
        OpponentInfo opponent;
        opponent.timestamp = this->now();
        opponent.valid = false;
        
        if (current_scan_.ranges.empty()) {
            return opponent;
        }
        
        // Find closest point in front sector
        double min_range = std::numeric_limits<double>::max();
        double best_angle = 0.0;
        
        double angle_min = current_scan_.angle_min;
        double angle_inc = current_scan_.angle_increment;
        
        for (size_t i = 0; i < current_scan_.ranges.size(); ++i) {
            double angle = angle_min + i * angle_inc;
            
            // Only consider front sector
            if (angle < params_.front_angle_min || angle > params_.front_angle_max) {
                continue;
            }
            
            double range = current_scan_.ranges[i];
            
            // Validate range
            if (std::isnan(range) || std::isinf(range)) {
                continue;
            }
            if (range < params_.min_valid_range || range > params_.max_detection_range) {
                continue;
            }
            
            // Find minimum
            if (range < min_range) {
                min_range = range;
                best_angle = angle;
            }
        }
        
        // No valid detection
        if (min_range == std::numeric_limits<double>::max()) {
            return opponent;
        }
        
        // Convert to map frame
        // LiDAR point in ego frame: (range * cos(angle), range * sin(angle))
        double obs_x_ego = min_range * std::cos(best_angle);
        double obs_y_ego = min_range * std::sin(best_angle);
        
        // Transform to map frame
        double cos_yaw = std::cos(ego_state_.yaw);
        double sin_yaw = std::sin(ego_state_.yaw);
        opponent.x = ego_state_.x + cos_yaw * obs_x_ego - sin_yaw * obs_y_ego;
        opponent.y = ego_state_.y + sin_yaw * obs_x_ego + cos_yaw * obs_y_ego;
        
        opponent.distance = min_range;
        opponent.angle = best_angle;
        
        // Convert to Frenet coordinates
        FrenetCoord opp_frenet = frenet_converter_.toFrenet(
            opponent.x, opponent.y, ego_state_.path_idx);
        opponent.s = opp_frenet.s;
        opponent.d = opp_frenet.d;
        
        // Validate opponent (must be ahead on raceline, within lane)
        double s_diff = opponent.s - ego_state_.s;
        
        // Handle wraparound for closed tracks
        double total_len = frenet_converter_.getTotalPathLength();
        if (s_diff < -total_len / 2.0) {
            s_diff += total_len;
        } else if (s_diff > total_len / 2.0) {
            s_diff -= total_len;
        }
        
        // Must be ahead
        if (s_diff < params_.min_ahead_margin || s_diff > params_.max_ahead_distance) {
            return opponent;  // Not ahead or too far
        }
        
        // Must be within lane bounds
        if (std::abs(opponent.d) > params_.lane_half_width) {
            return opponent;  // Outside lane
        }
        
        // Valid opponent detected!
        opponent.valid = true;
        
        RCLCPP_DEBUG(this->get_logger(),
            "Opponent detected: (%.2f, %.2f) -> Frenet(s=%.2f, d=%.2f), dist=%.2f",
            opponent.x, opponent.y, opponent.s, opponent.d, opponent.distance);
        
        return opponent;
    }
    
    void publishOpponentInfo(const OpponentInfo& opponent)
    {
        std_msgs::msg::Float64MultiArray msg;
        // Layout: [x, y, s, d, distance, angle, valid, ego_s, ego_d]
        msg.data = {
            opponent.x,
            opponent.y,
            opponent.s,
            opponent.d,
            opponent.distance,
            opponent.angle,
            opponent.valid ? 1.0 : 0.0,
            ego_state_.s,
            ego_state_.d
        };
        
        opponent_pub_->publish(msg);
    }
    
    void publishVisualization(const OpponentInfo& opponent)
    {
        visualization_msgs::msg::MarkerArray markers;
        
        // Opponent position marker (sphere)
        visualization_msgs::msg::Marker opp_marker;
        opp_marker.header.frame_id = params_.frame_id;
        opp_marker.header.stamp = this->now();
        opp_marker.ns = "opponent";
        opp_marker.id = 0;
        opp_marker.type = visualization_msgs::msg::Marker::SPHERE;
        
        if (opponent.valid) {
            opp_marker.action = visualization_msgs::msg::Marker::ADD;
            opp_marker.pose.position.x = opponent.x;
            opp_marker.pose.position.y = opponent.y;
            opp_marker.pose.position.z = 0.3;
            opp_marker.pose.orientation.w = 1.0;
            
            opp_marker.scale.x = 0.4;
            opp_marker.scale.y = 0.4;
            opp_marker.scale.z = 0.4;
            
            // Orange color for opponent
            opp_marker.color.r = 1.0f;
            opp_marker.color.g = 0.5f;
            opp_marker.color.b = 0.0f;
            opp_marker.color.a = 0.9f;
        } else {
            opp_marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        
        opp_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
        markers.markers.push_back(opp_marker);
        
        // Ego position marker (small green sphere)
        visualization_msgs::msg::Marker ego_marker;
        ego_marker.header.frame_id = params_.frame_id;
        ego_marker.header.stamp = this->now();
        ego_marker.ns = "ego_frenet";
        ego_marker.id = 1;
        ego_marker.type = visualization_msgs::msg::Marker::SPHERE;
        ego_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Show ego position projected onto raceline
        geometry_msgs::msg::Point ego_on_path = frenet_converter_.toCartesian(ego_state_.s, 0.0);
        ego_marker.pose.position.x = ego_on_path.x;
        ego_marker.pose.position.y = ego_on_path.y;
        ego_marker.pose.position.z = 0.1;
        ego_marker.pose.orientation.w = 1.0;
        
        ego_marker.scale.x = 0.2;
        ego_marker.scale.y = 0.2;
        ego_marker.scale.z = 0.2;
        
        ego_marker.color.r = 0.0f;
        ego_marker.color.g = 1.0f;
        ego_marker.color.b = 0.0f;
        ego_marker.color.a = 0.7f;
        
        ego_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
        markers.markers.push_back(ego_marker);
        
        // Text marker showing Frenet coordinates
        if (opponent.valid) {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = params_.frame_id;
            text_marker.header.stamp = this->now();
            text_marker.ns = "opponent_info";
            text_marker.id = 2;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose.position.x = opponent.x;
            text_marker.pose.position.y = opponent.y;
            text_marker.pose.position.z = 0.8;
            text_marker.pose.orientation.w = 1.0;
            
            std::ostringstream oss;
            oss << "OPP\ns=" << std::fixed << std::setprecision(1) << opponent.s
                << " d=" << std::setprecision(2) << opponent.d
                << "\ndist=" << opponent.distance;
            text_marker.text = oss.str();
            
            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;
            
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
            markers.markers.push_back(text_marker);
        }
        
        marker_pub_->publish(markers);
    }
};

}  // namespace planning_pkg

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planning_pkg::OpponentDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
