/**
 * @file racing_agent.cpp
 * @brief Implementation of Racing Agent - Upper Level Strategy Controller
 * 
 * This implements the racing agent architecture as specified:
 * - Five-mode state machine (CRUISE, FOLLOW, OVERTAKE_CANDIDATE, OVERTAKE, OBSTACLE_STOP)
 * - Pre-defined OVERTAKE ZONEs on global raceline
 * - Pre-computed overtake trajectory selection
 * - Safe stop strategy for obstacles (no repulsion/bouncing)
 */

#include "planning_pkg/racing_agent.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <limits>

namespace planning_pkg
{

// === Constants ===
static constexpr double VEHICLE_WIDTH = 0.35;       // F1TENTH vehicle width (m)
static constexpr double SAFETY_MARGIN = 0.15;       // Safety margin (m)
static constexpr double OVERTAKE_LATERAL_OFFSET = 0.8;  // Default lateral offset for overtake (m)
static constexpr double LOOKAHEAD_DISTANCE = 3.0;   // Default lookahead for reference path (m)
static constexpr int REFERENCE_PATH_POINTS = 30;    // Number of points in reference path

// === Corner Exit Safety ===
static constexpr double CORNER_EXIT_WALL_MARGIN = 0.35;  // Minimum distance from wall on corner exit (m)
static constexpr double CORNER_EXIT_LATERAL_SOFTEN = 0.85;  // Factor to reduce lateral OUT on corner exit (0-1), balanced value

// === Inside/Outside Overtake Factors ===
static constexpr double INSIDE_OVERTAKE_FACTOR = 0.7;   // Tighter offset for inside-line overtake (apex side)
static constexpr double OUTSIDE_OVERTAKE_FACTOR = 1.1;  // Wider offset for outside-line overtake

// === Overtake Feasibility Constants (defaults, overridden by parameters) ===
// NOTE: OPPONENT_WIDTH default equals VEHICLE_WIDTH for F1TENTH races where vehicles
// are similar. The parameter "opponent_width" can be configured differently if needed.
static constexpr double OPPONENT_WIDTH = 0.35;          // Default opponent vehicle width (m)
static constexpr double OVERTAKE_WIDTH_FACTOR = 1.1;    // Factor multiplied by opponent width for clearance (reduced for more overtakes)
static constexpr double OVERTAKE_LATERAL_MARGIN = 0.20; // Additional safety margin for overtake (m)
static constexpr double MIN_LONGITUDINAL_WINDOW = 4.0;  // Minimum longitudinal distance to complete overtake (m)

RacingAgent::RacingAgent()
: Node("racing_agent")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Racing Agent - Upper Level Strategy Controller");
    
    // === Declare Parameters ===
    this->declare_parameter("safe_follow_distance", 3.0);  // Increased for more relaxed following
    this->declare_parameter("min_stop_distance", 0.3);
    this->declare_parameter("cruise_speed", 5.0);
    this->declare_parameter("decision_rate", 20.0);
    this->declare_parameter("zones_config_file", "");
    this->declare_parameter("trajectories_config_file", "");
    
    // === Corner Exit Smoothing Parameters ===
    this->declare_parameter("corner_exit_wall_margin", CORNER_EXIT_WALL_MARGIN);
    this->declare_parameter("corner_exit_lateral_soften", CORNER_EXIT_LATERAL_SOFTEN);
    
    // === Corner Handling Parameters ===
    this->declare_parameter("corner_curvature_threshold", 0.15);
    this->declare_parameter("corner_follow_distance_factor", 1.3);
    this->declare_parameter("corner_speed_reduction", 0.8);
    
    // === Overtake Feasibility Parameters ===
    this->declare_parameter("opponent_width", OPPONENT_WIDTH);
    this->declare_parameter("overtake_width_factor", OVERTAKE_WIDTH_FACTOR);
    this->declare_parameter("overtake_lateral_margin", OVERTAKE_LATERAL_MARGIN);
    this->declare_parameter("min_longitudinal_window", MIN_LONGITUDINAL_WINDOW);
    
    // Load parameters
    safe_follow_distance_ = this->get_parameter("safe_follow_distance").as_double();
    min_stop_distance_ = this->get_parameter("min_stop_distance").as_double();
    cruise_speed_ = this->get_parameter("cruise_speed").as_double();
    decision_rate_ = this->get_parameter("decision_rate").as_double();
    
    // Load corner exit parameters
    corner_exit_wall_margin_ = this->get_parameter("corner_exit_wall_margin").as_double();
    corner_exit_lateral_soften_ = this->get_parameter("corner_exit_lateral_soften").as_double();
    
    // Load corner handling parameters
    corner_curvature_threshold_ = this->get_parameter("corner_curvature_threshold").as_double();
    corner_follow_distance_factor_ = this->get_parameter("corner_follow_distance_factor").as_double();
    corner_speed_reduction_ = this->get_parameter("corner_speed_reduction").as_double();
    
    // Load overtake feasibility parameters
    opponent_width_ = this->get_parameter("opponent_width").as_double();
    overtake_width_factor_ = this->get_parameter("overtake_width_factor").as_double();
    overtake_lateral_margin_ = this->get_parameter("overtake_lateral_margin").as_double();
    min_longitudinal_window_ = this->get_parameter("min_longitudinal_window").as_double();
    
    // === Subscriptions ===
    global_raceline_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_raceline", rclcpp::QoS(10).transient_local().reliable(),
        std::bind(&RacingAgent::raceline_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10).best_effort(),
        std::bind(&RacingAgent::odom_callback, this, std::placeholders::_1));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&RacingAgent::scan_callback, this, std::placeholders::_1));
    
    // === Publishers ===
    reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/racing_agent/reference_path", 10);
    
    mode_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/racing_agent/mode", 10);
    
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/racing_agent/visualization", 10);
    
    // === Load Configuration ===
    std::string zones_config = this->get_parameter("zones_config_file").as_string();
    if (!zones_config.empty()) {
        load_zones_from_config(zones_config);
    } else {
        // Default: Generate some sample zones for testing
        RCLCPP_INFO(this->get_logger(), "No zones config file specified, using default zones");
    }
    
    std::string traj_config = this->get_parameter("trajectories_config_file").as_string();
    if (!traj_config.empty()) {
        load_overtake_trajectories(traj_config);
    } else {
        RCLCPP_INFO(this->get_logger(), "No trajectory config file specified, will generate defaults");
    }
    
    // === Decision Timer ===
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / decision_rate_));
    decision_timer_ = this->create_wall_timer(period,
        std::bind(&RacingAgent::decision_loop, this));
    
    RCLCPP_INFO(this->get_logger(), 
        "Racing Agent initialized. Mode: %s, Decision rate: %.1f Hz",
        racing_mode_to_string(current_mode_).c_str(), decision_rate_);
}

// === Callbacks ===

void RacingAgent::raceline_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty raceline");
        return;
    }
    
    global_raceline_ = *msg;
    raceline_received_ = true;
    
    // Compute arc length for each waypoint
    raceline_s_.clear();
    raceline_s_.push_back(0.0);
    
    double total_length = 0.0;
    for (size_t i = 1; i < msg->poses.size(); ++i) {
        double dx = msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x;
        double dy = msg->poses[i].pose.position.y - msg->poses[i-1].pose.position.y;
        total_length += std::hypot(dx, dy);
        raceline_s_.push_back(total_length);
    }
    track_length_ = total_length;
    
    // Generate default zones if not loaded
    if (zones_.empty()) {
        // Example: Create some overtake zones at regular intervals
        // In production, these would come from track analysis
        double zone_length = track_length_ / 4.0;
        for (int i = 0; i < 2; ++i) {
            double start = zone_length * (2 * i);
            double end = start + zone_length * 0.3;  // 30% of each quarter is overtake zone
            zones_.emplace_back(start, end, ZoneType::OVERTAKE_ZONE, 
                               "OvertakeZone_" + std::to_string(i));
        }
        RCLCPP_INFO(this->get_logger(), 
            "Generated %zu default OVERTAKE ZONEs for track length %.1fm",
            zones_.size(), track_length_);
    }
    
    // Generate default overtake trajectories
    generate_default_overtake_trajectories();
    
    RCLCPP_INFO(this->get_logger(), 
        "Received global raceline: %zu points, length: %.1fm",
        msg->poses.size(), track_length_);
}

void RacingAgent::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Update ego state
    env_state_.ego_speed = std::hypot(
        msg->twist.twist.linear.x, 
        msg->twist.twist.linear.y);
    
    // Compute arc length position on raceline
    if (raceline_received_) {
        env_state_.ego_s = compute_arc_length_position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y);
        
        // Compute lateral deviation (simplified - perpendicular distance to raceline)
        size_t closest_idx = find_closest_raceline_index(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y);
        
        if (closest_idx < global_raceline_.poses.size()) {
            double dx = msg->pose.pose.position.x - 
                       global_raceline_.poses[closest_idx].pose.position.x;
            double dy = msg->pose.pose.position.y - 
                       global_raceline_.poses[closest_idx].pose.position.y;
            
            // Get raceline heading at this point
            double raceline_yaw = tf2::getYaw(
                global_raceline_.poses[closest_idx].pose.orientation);
            
            // Lateral deviation = perpendicular distance (positive = left of raceline)
            env_state_.ego_d = -dx * std::sin(raceline_yaw) + dy * std::cos(raceline_yaw);
        }
    }
    
    env_state_.ego_heading = tf2::getYaw(msg->pose.pose.orientation);
}

void RacingAgent::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Process LiDAR scan for obstacle detection
    constexpr double FRONT_SECTOR_HALF_ANGLE = M_PI / 6.0;  // ±30 degrees
    constexpr double SIDE_SECTOR_START = M_PI / 6.0;         // 30 degrees
    constexpr double SIDE_SECTOR_END = M_PI / 2.0;           // 90 degrees
    
    double min_front = 10.0;
    double min_left = 10.0;
    double min_right = 10.0;
    
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];
        
        if (std::isnan(range) || std::isinf(range) || range < 0.03) {
            continue;
        }
        
        // Front sector
        if (std::abs(angle) <= FRONT_SECTOR_HALF_ANGLE) {
            min_front = std::min(min_front, range);
        }
        // Left sector
        else if (angle > SIDE_SECTOR_START && angle < SIDE_SECTOR_END) {
            min_left = std::min(min_left, range);
        }
        // Right sector
        else if (angle < -SIDE_SECTOR_START && angle > -SIDE_SECTOR_END) {
            min_right = std::min(min_right, range);
        }
    }
    
    env_state_.front_obstacle_distance = min_front;
    env_state_.left_clearance = min_left;
    env_state_.right_clearance = min_right;
    env_state_.front_obstacle_detected = min_front < safe_follow_distance_;
    
    // Simple preceding vehicle detection (if front obstacle within follow distance)
    env_state_.has_preceding_vehicle = env_state_.front_obstacle_detected;
    env_state_.preceding_distance = min_front;
}

// === Main Decision Loop ===

void RacingAgent::decision_loop()
{
    if (!raceline_received_) {
        return;  // Wait for raceline
    }
    
    // Update environment state
    update_environment_state();
    
    // Compute next mode
    RacingMode next_mode = compute_next_mode();
    
    // Transition if needed
    if (next_mode != current_mode_) {
        transition_to_mode(next_mode);
    }
    
    // Execute current mode logic
    switch (current_mode_) {
        case RacingMode::CRUISE:
            execute_cruise_mode();
            break;
        case RacingMode::FOLLOW:
            execute_follow_mode();
            break;
        case RacingMode::OVERTAKE_CANDIDATE:
            execute_overtake_candidate_mode();
            break;
        case RacingMode::OVERTAKE:
            execute_overtake_mode();
            break;
        case RacingMode::OBSTACLE_STOP:
            execute_obstacle_stop_mode();
            break;
    }
    
    // Publish outputs
    reference_path_pub_->publish(current_command_.reference_path);
    
    std_msgs::msg::String mode_msg;
    mode_msg.data = racing_mode_to_string(current_mode_);
    mode_pub_->publish(mode_msg);
    
    // Publish visualization
    publish_visualization();
}

// === Mode Transition Logic ===

RacingMode RacingAgent::compute_next_mode()
{
    // Priority 1: Emergency stop for immediate collision risk
    if (should_stop_for_obstacle()) {
        return RacingMode::OBSTACLE_STOP;
    }
    
    // Mode-specific transitions
    switch (current_mode_) {
        case RacingMode::CRUISE:
            // If preceding vehicle detected, start following
            if (should_start_following()) {
                return RacingMode::FOLLOW;
            }
            return RacingMode::CRUISE;
            
        case RacingMode::FOLLOW:
            // If no longer following, return to cruise
            if (!env_state_.has_preceding_vehicle) {
                return RacingMode::CRUISE;
            }
            // If in overtake zone and can consider overtake
            if (can_consider_overtake()) {
                return RacingMode::OVERTAKE_CANDIDATE;
            }
            return RacingMode::FOLLOW;
            
        case RacingMode::OVERTAKE_CANDIDATE:
            // If no longer in overtake zone, return to follow/cruise
            if (get_current_zone_type() != ZoneType::OVERTAKE_ZONE) {
                return env_state_.has_preceding_vehicle ? 
                       RacingMode::FOLLOW : RacingMode::CRUISE;
            }
            // If overtake trajectory selected and safe, start overtake
            if (active_trajectory_idx_.has_value()) {
                return RacingMode::OVERTAKE;
            }
            // Stay in candidate mode
            return RacingMode::OVERTAKE_CANDIDATE;
            
        case RacingMode::OVERTAKE:
            // If overtake complete (progress >= 1.0)
            if (overtake_progress_ >= 1.0) {
                active_trajectory_idx_ = std::nullopt;
                return RacingMode::CRUISE;
            }
            // If collision risk during overtake, abort and stop
            if (should_stop_for_obstacle()) {
                active_trajectory_idx_ = std::nullopt;
                return RacingMode::OBSTACLE_STOP;
            }
            // Check if overtake conditions become invalid (clearance too narrow, unsafe)
            // If so, abort overtake and return to safe FOLLOW mode
            if (should_abort_overtake()) {
                RCLCPP_WARN(this->get_logger(), 
                    "Aborting overtake: clearance or conditions became unsafe");
                active_trajectory_idx_ = std::nullopt;
                return RacingMode::FOLLOW;
            }
            return RacingMode::OVERTAKE;
            
        case RacingMode::OBSTACLE_STOP:
            // If obstacle cleared, return to cruise
            if (env_state_.front_obstacle_distance > safe_follow_distance_) {
                return RacingMode::CRUISE;
            }
            return RacingMode::OBSTACLE_STOP;
    }
    
    return RacingMode::CRUISE;  // Default fallback
}

void RacingAgent::transition_to_mode(RacingMode new_mode)
{
    RCLCPP_INFO(this->get_logger(), 
        "Mode transition: %s -> %s",
        racing_mode_to_string(current_mode_).c_str(),
        racing_mode_to_string(new_mode).c_str());
    
    previous_mode_ = current_mode_;
    current_mode_ = new_mode;
    
    // Reset mode-specific state
    switch (new_mode) {
        case RacingMode::OVERTAKE:
            overtake_progress_ = 0.0;
            break;
        case RacingMode::OBSTACLE_STOP:
            current_command_.should_stop = true;
            break;
        case RacingMode::CRUISE:
        case RacingMode::FOLLOW:
        case RacingMode::OVERTAKE_CANDIDATE:
            current_command_.should_stop = false;
            active_trajectory_idx_ = std::nullopt;
            break;
    }
    
    current_command_.mode = new_mode;
}

// === Mode Execution ===

void RacingAgent::execute_cruise_mode()
{
    current_command_.reference_path = generate_cruise_reference_path();
    current_command_.speed_limit = cruise_speed_;
    current_command_.should_stop = false;
}

void RacingAgent::execute_follow_mode()
{
    current_command_.reference_path = generate_follow_reference_path();
    
    // Use corner-aware following distance
    double effective_follow_distance = get_corner_adjusted_follow_distance();
    
    // Speed control: follow at safe distance using simple PD control
    double gap_error = env_state_.preceding_distance - effective_follow_distance;
    double target_speed = env_state_.preceding_speed + 0.5 * gap_error;
    
    // In corners, reduce speed further for safety
    if (is_in_corner()) {
        target_speed *= corner_speed_reduction_;
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "FOLLOW in corner: reduced speed to %.2f, follow_dist=%.2fm",
            target_speed, effective_follow_distance);
    }
    
    target_speed = std::clamp(target_speed, 0.0, cruise_speed_);
    
    current_command_.speed_limit = target_speed;
    current_command_.should_stop = false;
    
    // Pre-prepare overtake candidates based on distance to front car
    prepare_overtake_candidates();
}

void RacingAgent::execute_overtake_candidate_mode()
{
    // Keep following while evaluating
    current_command_.reference_path = generate_follow_reference_path();
    current_command_.speed_limit = std::min(cruise_speed_, env_state_.preceding_speed + 0.5);
    
    // Evaluate available trajectories
    auto valid_trajectories = get_valid_trajectories_for_current_zone();
    
    if (!valid_trajectories.empty()) {
        auto best_idx = select_best_trajectory(valid_trajectories);
        if (best_idx.has_value()) {
            active_trajectory_idx_ = best_idx;
            RCLCPP_INFO(this->get_logger(), 
                "Selected overtake trajectory: %s",
                overtake_trajectories_[*best_idx].id.c_str());
        }
    }
}

void RacingAgent::execute_overtake_mode()
{
    if (!active_trajectory_idx_.has_value()) {
        RCLCPP_WARN(this->get_logger(), "No active trajectory in OVERTAKE mode");
        return;
    }
    
    current_command_.reference_path = get_current_overtake_path();
    current_command_.speed_limit = cruise_speed_ * 1.2;  // Slight speed boost during overtake
    current_command_.should_stop = false;
    
    // Update progress (simplified - based on arc length)
    const auto& traj = overtake_trajectories_[*active_trajectory_idx_];
    double traj_length = traj.s_end - traj.s_start;
    if (traj_length > 0 && track_length_ > 0) {
        double progress_s = env_state_.ego_s - traj.s_start;
        // Handle track wrap-around only if track_length is valid
        if (progress_s < 0) progress_s += track_length_;
        overtake_progress_ = std::clamp(progress_s / traj_length, 0.0, 1.0);
    } else {
        // Invalid trajectory or track length - mark as complete
        overtake_progress_ = 1.0;
    }
}

void RacingAgent::execute_obstacle_stop_mode()
{
    // Generate stop path: decelerate to zero
    current_command_.reference_path = generate_stop_path();
    current_command_.speed_limit = 0.0;
    current_command_.should_stop = true;
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "OBSTACLE_STOP: Stopped safely. Front distance: %.2fm",
        env_state_.front_obstacle_distance);
}

// === Zone Management ===

void RacingAgent::load_zones_from_config(const std::string& config_path)
{
    // In a real implementation, this would parse a YAML or JSON file
    // For now, just log that we tried
    RCLCPP_INFO(this->get_logger(), "Loading zones from: %s", config_path.c_str());
    
    std::ifstream file(config_path);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open zones config file: %s", config_path.c_str());
        return;
    }
    
    // NOTE: YAML/JSON config parsing is not yet implemented.
    // Currently, the system relies on default zone generation after raceline is received.
    // For production use, implement parsing here or use the racing_agent_params.yaml
    // via ROS2 parameters instead of file-based configuration.
    // TODO: Implement YAML parsing using yaml-cpp or similar library
    RCLCPP_WARN(this->get_logger(), 
        "Zone config file parsing not implemented. Using default zone generation.");
    file.close();
}

ZoneType RacingAgent::get_current_zone_type() const
{
    // Guard against division by zero when track length is not yet set
    if (track_length_ <= 0.0) {
        return ZoneType::NORMAL;
    }
    
    for (const auto& zone : zones_) {
        // Handle track wrap-around
        double s = std::fmod(env_state_.ego_s, track_length_);
        if (zone.contains(s)) {
            return zone.type;
        }
    }
    return ZoneType::NORMAL;
}

const RacelineZone* RacingAgent::get_current_zone() const
{
    // Guard against division by zero when track length is not yet set
    if (track_length_ <= 0.0) {
        return nullptr;
    }
    
    for (const auto& zone : zones_) {
        double s = std::fmod(env_state_.ego_s, track_length_);
        if (zone.contains(s)) {
            return &zone;
        }
    }
    return nullptr;
}

// === Overtake Trajectory Management ===

void RacingAgent::load_overtake_trajectories(const std::string& config_path)
{
    RCLCPP_INFO(this->get_logger(), "Loading trajectories from: %s", config_path.c_str());
    // TODO: Implement trajectory loading from file
}

void RacingAgent::generate_default_overtake_trajectories()
{
    overtake_trajectories_.clear();
    
    for (const auto& zone : zones_) {
        if (zone.type != ZoneType::OVERTAKE_ZONE) continue;
        
        // Generate waypoints indices for this zone
        size_t start_idx = std::numeric_limits<size_t>::max();
        size_t end_idx = 0;
        for (size_t i = 0; i < raceline_s_.size(); ++i) {
            if (raceline_s_[i] >= zone.s_start && start_idx == std::numeric_limits<size_t>::max()) {
                start_idx = i;
            }
            if (raceline_s_[i] >= zone.s_end) { 
                end_idx = i; 
                break; 
            }
        }
        
        // Skip if no valid waypoints found in zone
        if (start_idx == std::numeric_limits<size_t>::max()) {
            continue;
        }
        
        // Handle single waypoint case explicitly
        size_t waypoint_count = (end_idx >= start_idx) ? (end_idx - start_idx + 1) : 1;
        
        // Generate 4 trajectories per zone:
        // 1. Left side outside (standard offset)
        // 2. Left side inside (reduced offset for inside-line overtake)
        // 3. Right side outside (standard offset)
        // 4. Right side inside (reduced offset for inside-line overtake)
        
        struct TrajectoryConfig {
            std::string suffix;
            bool is_left_side;
            bool is_inside;
            double offset_factor;
        };
        
        std::vector<TrajectoryConfig> configs = {
            {"_left_outside", true, false, OUTSIDE_OVERTAKE_FACTOR},   // Outside: wider offset
            {"_left_inside", true, true, INSIDE_OVERTAKE_FACTOR},      // Inside: tighter offset
            {"_right_outside", false, false, OUTSIDE_OVERTAKE_FACTOR}, // Outside: wider offset
            {"_right_inside", false, true, INSIDE_OVERTAKE_FACTOR}     // Inside: tighter offset
        };
        
        for (const auto& config : configs) {
            OvertakeTrajectory traj;
            traj.id = zone.name + config.suffix;
            traj.zone_name = zone.name;
            traj.s_start = zone.s_start;
            traj.s_end = zone.s_end;
            traj.is_left_side = config.is_left_side;
            traj.lateral_offset_max = (config.is_left_side ? 1.0 : -1.0) * 
                                      OVERTAKE_LATERAL_OFFSET * config.offset_factor;
            traj.estimated_duration = (zone.s_end - zone.s_start) / cruise_speed_;
            
            for (size_t i = start_idx; i <= end_idx && i < global_raceline_.poses.size(); ++i) {
                double progress = (waypoint_count > 1) ? 
                    static_cast<double>(i - start_idx) / static_cast<double>(waypoint_count - 1) : 0.0;
                
                // Apply asymmetric S-curve for corner exit safety
                double lateral_scale = 1.0;
                if (progress > 0.5) {
                    double exit_progress = (progress - 0.5) * 2.0;
                    lateral_scale = 1.0 - exit_progress * (1.0 - corner_exit_lateral_soften_);
                }
                
                double base_offset = OVERTAKE_LATERAL_OFFSET * config.offset_factor;
                double lateral_offset = base_offset * std::sin(progress * M_PI) * lateral_scale;
                if (!config.is_left_side) {
                    lateral_offset = -lateral_offset;  // Flip for right side
                }
                
                geometry_msgs::msg::PoseStamped pose = global_raceline_.poses[i];
                double yaw = tf2::getYaw(pose.pose.orientation);
                pose.pose.position.x -= lateral_offset * std::sin(yaw);
                pose.pose.position.y += lateral_offset * std::cos(yaw);
                
                traj.waypoints.push_back(pose);
                traj.speeds.push_back(cruise_speed_);
            }
            
            overtake_trajectories_.push_back(traj);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "Generated %zu overtake trajectories (inside/outside) for %zu zones",
        overtake_trajectories_.size(), zones_.size());
}

std::vector<size_t> RacingAgent::get_valid_trajectories_for_current_zone() const
{
    std::vector<size_t> valid;
    const RacelineZone* current_zone = get_current_zone();
    
    if (!current_zone || current_zone->type != ZoneType::OVERTAKE_ZONE) {
        return valid;
    }
    
    for (size_t i = 0; i < overtake_trajectories_.size(); ++i) {
        const auto& traj = overtake_trajectories_[i];
        if (traj.zone_name == current_zone->name && is_overtake_safe(traj)) {
            valid.push_back(i);
        }
    }
    
    return valid;
}

std::optional<size_t> RacingAgent::select_best_trajectory(const std::vector<size_t>& candidates)
{
    if (candidates.empty()) return std::nullopt;
    
    // Determine the preferred overtake side (inside vs outside)
    OvertakeSide preferred_side = determine_best_overtake_side();
    
    // Score-based selection considering:
    // 1. Inside vs outside preference based on track geometry
    // 2. Available clearance on each side
    // 3. Trajectory offset (smaller = safer)
    
    size_t best_idx = candidates[0];
    double best_score = -std::numeric_limits<double>::max();
    
    for (size_t idx : candidates) {
        const auto& traj = overtake_trajectories_[idx];
        double clearance = traj.is_left_side ? env_state_.left_clearance : env_state_.right_clearance;
        
        // Base score from clearance
        double score = clearance * 1.0;
        
        // Bonus for matching preferred side (inside/outside)
        bool is_inside_traj = (traj.id.find("inside") != std::string::npos);
        if (preferred_side == OvertakeSide::INSIDE && is_inside_traj) {
            score += 1.5;  // Bonus for inside trajectory when inside is preferred
        } else if (preferred_side == OvertakeSide::OUTSIDE && !is_inside_traj) {
            score += 1.0;  // Smaller bonus for outside trajectory
        }
        
        // Penalty for large lateral offsets (safer to use smaller offsets)
        score -= std::abs(traj.lateral_offset_max) * 0.5;
        
        if (score > best_score) {
            best_score = score;
            best_idx = idx;
        }
    }
    
    // Only select if clearance is sufficient
    const auto& selected_traj = overtake_trajectories_[best_idx];
    double clearance = selected_traj.is_left_side ? env_state_.left_clearance : env_state_.right_clearance;
    double min_required = VEHICLE_WIDTH * 2 + SAFETY_MARGIN;
    
    if (clearance > min_required) {
        RCLCPP_INFO(this->get_logger(), 
            "Selected trajectory: %s (clearance: %.2fm, score: %.2f)",
            selected_traj.id.c_str(), clearance, best_score);
        return best_idx;
    }
    
    return std::nullopt;
}

nav_msgs::msg::Path RacingAgent::get_current_overtake_path() const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    
    if (!active_trajectory_idx_.has_value()) {
        return path;
    }
    
    const auto& traj = overtake_trajectories_[*active_trajectory_idx_];
    
    // Return remaining portion of trajectory based on progress
    size_t start_wp = static_cast<size_t>(overtake_progress_ * traj.waypoints.size());
    for (size_t i = start_wp; i < traj.waypoints.size(); ++i) {
        path.poses.push_back(traj.waypoints[i]);
    }
    
    return path;
}

// === Reference Path Generation ===

nav_msgs::msg::Path RacingAgent::generate_cruise_reference_path() const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    
    if (!raceline_received_ || global_raceline_.poses.empty()) {
        return path;
    }
    
    // Find current position on raceline
    size_t current_idx = 0;
    for (size_t i = 0; i < raceline_s_.size(); ++i) {
        if (raceline_s_[i] >= env_state_.ego_s) {
            current_idx = i;
            break;
        }
    }
    
    // Add lookahead points from raceline
    for (int i = 0; i < REFERENCE_PATH_POINTS; ++i) {
        size_t idx = (current_idx + i) % global_raceline_.poses.size();
        path.poses.push_back(global_raceline_.poses[idx]);
    }
    
    return path;
}

nav_msgs::msg::Path RacingAgent::generate_follow_reference_path() const
{
    // For following, use the same raceline but the speed will be adjusted
    return generate_cruise_reference_path();
}

nav_msgs::msg::Path RacingAgent::generate_stop_path() const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    
    // Minimal path - just current direction, speed will be zero
    // This tells the controller to stop in place
    if (raceline_received_ && !global_raceline_.poses.empty()) {
        size_t current_idx = 0;
        for (size_t i = 0; i < raceline_s_.size(); ++i) {
            if (raceline_s_[i] >= env_state_.ego_s) {
                current_idx = i;
                break;
            }
        }
        
        // Just a few points ahead for stable stopping
        for (int i = 0; i < 5; ++i) {
            size_t idx = (current_idx + i) % global_raceline_.poses.size();
            path.poses.push_back(global_raceline_.poses[idx]);
        }
    }
    
    return path;
}

// === Visualization ===

void RacingAgent::publish_visualization()
{
    visualization_msgs::msg::MarkerArray markers;
    
    // Add raceline visualization
    auto raceline_markers = create_raceline_markers();
    for (auto& m : raceline_markers.markers) markers.markers.push_back(m);
    
    // Add zone visualization
    auto zone_markers = create_zone_markers();
    for (auto& m : zone_markers.markers) markers.markers.push_back(m);
    
    // Add overtake trajectory visualization
    auto traj_markers = create_overtake_trajectory_markers();
    for (auto& m : traj_markers.markers) markers.markers.push_back(m);
    
    // Add mode indicator
    auto mode_markers = create_mode_marker();
    for (auto& m : mode_markers.markers) markers.markers.push_back(m);
    
    viz_pub_->publish(markers);
}

visualization_msgs::msg::MarkerArray RacingAgent::create_raceline_markers() const
{
    visualization_msgs::msg::MarkerArray markers;
    
    if (!raceline_received_) return markers;
    
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "global_raceline";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.05;  // Line width
    line_marker.color.r = 0.3f;
    line_marker.color.g = 0.3f;
    line_marker.color.b = 1.0f;
    line_marker.color.a = 0.8f;
    
    for (const auto& pose : global_raceline_.poses) {
        geometry_msgs::msg::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = 0.05;
        line_marker.points.push_back(p);
    }
    
    markers.markers.push_back(line_marker);
    return markers;
}

visualization_msgs::msg::MarkerArray RacingAgent::create_zone_markers() const
{
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    
    for (const auto& zone : zones_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "overtake_zones";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15;  // Thicker line for zones
        
        if (zone.type == ZoneType::OVERTAKE_ZONE) {
            // Green for overtake zones
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.9f;
        } else {
            continue;  // Only visualize overtake zones
        }
        
        // Find waypoints in this zone
        for (size_t i = 0; i < raceline_s_.size(); ++i) {
            if (raceline_s_[i] >= zone.s_start && raceline_s_[i] <= zone.s_end) {
                geometry_msgs::msg::Point p;
                p.x = global_raceline_.poses[i].pose.position.x;
                p.y = global_raceline_.poses[i].pose.position.y;
                p.z = 0.1;
                marker.points.push_back(p);
            }
        }
        
        if (!marker.points.empty()) {
            markers.markers.push_back(marker);
        }
    }
    
    return markers;
}

visualization_msgs::msg::MarkerArray RacingAgent::create_overtake_trajectory_markers() const
{
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    
    for (size_t traj_idx = 0; traj_idx < overtake_trajectories_.size(); ++traj_idx) {
        const auto& traj = overtake_trajectories_[traj_idx];
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "overtake_trajectories";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        
        // Color based on side and whether active
        bool is_active = active_trajectory_idx_.has_value() && 
                        *active_trajectory_idx_ == traj_idx;
        
        if (is_active) {
            marker.scale.x = 0.12;  // Thick for active
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;  // Yellow for active
            marker.color.a = 1.0f;
        } else if (traj.is_left_side) {
            marker.scale.x = 0.05;
            marker.color.r = 0.0f;
            marker.color.g = 0.5f;
            marker.color.b = 1.0f;  // Blue for left
            marker.color.a = 0.5f;
        } else {
            marker.scale.x = 0.05;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.5f;  // Cyan for right
            marker.color.a = 0.5f;
        }
        
        for (const auto& pose : traj.waypoints) {
            geometry_msgs::msg::Point p;
            p.x = pose.pose.position.x;
            p.y = pose.pose.position.y;
            p.z = 0.15;
            marker.points.push_back(p);
        }
        
        if (!marker.points.empty()) {
            markers.markers.push_back(marker);
        }
    }
    
    return markers;
}

visualization_msgs::msg::MarkerArray RacingAgent::create_mode_marker() const
{
    visualization_msgs::msg::MarkerArray markers;
    
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = this->now();
    text_marker.ns = "mode_indicator";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position above ego vehicle (approximation)
    if (!global_raceline_.poses.empty()) {
        size_t idx = find_closest_raceline_index(0, 0);  // Simplified
        text_marker.pose.position.x = global_raceline_.poses[idx].pose.position.x;
        text_marker.pose.position.y = global_raceline_.poses[idx].pose.position.y;
        text_marker.pose.position.z = 1.5;
    }
    
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.4;
    text_marker.text = racing_mode_to_string(current_mode_);
    
    // Color based on mode
    switch (current_mode_) {
        case RacingMode::CRUISE:
            text_marker.color.r = 0.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 0.0f;
            text_marker.color.a = 1.0f;  // Green
            break;
        case RacingMode::FOLLOW:
            text_marker.color.r = 0.0f;
            text_marker.color.g = 0.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;  // Blue
            break;
        case RacingMode::OVERTAKE_CANDIDATE:
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 0.0f;
            text_marker.color.a = 1.0f;  // Yellow
            break;
        case RacingMode::OVERTAKE:
            text_marker.color.r = 1.0f;
            text_marker.color.g = 0.5f;
            text_marker.color.b = 0.0f;
            text_marker.color.a = 1.0f;  // Orange
            break;
        case RacingMode::OBSTACLE_STOP:
            text_marker.color.r = 1.0f;
            text_marker.color.g = 0.0f;
            text_marker.color.b = 0.0f;
            text_marker.color.a = 1.0f;  // Red
            break;
    }
    
    markers.markers.push_back(text_marker);
    return markers;
}

// === Utilities ===

double RacingAgent::compute_arc_length_position(double x, double y) const
{
    size_t closest_idx = find_closest_raceline_index(x, y);
    if (closest_idx < raceline_s_.size()) {
        return raceline_s_[closest_idx];
    }
    return 0.0;
}

size_t RacingAgent::find_closest_raceline_index(double x, double y) const
{
    if (global_raceline_.poses.empty()) return 0;
    
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    for (size_t i = 0; i < global_raceline_.poses.size(); ++i) {
        double dx = x - global_raceline_.poses[i].pose.position.x;
        double dy = y - global_raceline_.poses[i].pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

bool RacingAgent::is_overtake_safe(const OvertakeTrajectory& trajectory) const
{
    // Use the new stricter lateral clearance check that considers opponent width
    if (!check_lateral_clearance_for_overtake(trajectory.is_left_side)) {
        return false;
    }
    
    // Also check longitudinal window
    if (!check_longitudinal_window_for_overtake()) {
        return false;
    }
    
    return true;
}

bool RacingAgent::should_stop_for_obstacle() const
{
    return env_state_.front_obstacle_distance < min_stop_distance_;
}

bool RacingAgent::should_start_following() const
{
    return env_state_.has_preceding_vehicle && 
           env_state_.preceding_distance < safe_follow_distance_;
}

bool RacingAgent::can_consider_overtake() const
{
    // Must be in OVERTAKE ZONE
    if (get_current_zone_type() != ZoneType::OVERTAKE_ZONE) {
        return false;
    }
    
    // Must have preceding vehicle
    if (!env_state_.has_preceding_vehicle) {
        return false;
    }
    
    // === IMPORTANT: Being in OVERTAKE ZONE is NOT enough ===
    // Must also check lateral clearance and longitudinal window
    
    // Check if overtake is actually feasible given opponent width and position
    if (!is_overtake_feasible()) {
        return false;
    }
    
    return true;
}

bool RacingAgent::check_lateral_clearance_for_overtake(bool is_left_side) const
{
    // Get clearance on the specified side
    // Note: clearance is measured from the vehicle center to the wall/obstacle
    double clearance = is_left_side ? env_state_.left_clearance : env_state_.right_clearance;
    
    // Calculate minimum required clearance considering opponent width
    // Required clearance = opponent_width * factor + safety_margin + lateral_offset
    // The lateral_offset (OVERTAKE_LATERAL_OFFSET) represents how far the overtake
    // path deviates from the raceline, which accounts for the space our vehicle needs
    double min_required = opponent_width_ * overtake_width_factor_ + 
                          overtake_lateral_margin_ + OVERTAKE_LATERAL_OFFSET;
    
    return clearance > min_required;
}

bool RacingAgent::check_longitudinal_window_for_overtake() const
{
    // Check if there is enough longitudinal distance to complete the overtake
    // before the next corner or narrow section
    
    // Get current zone
    const RacelineZone* current_zone = get_current_zone();
    if (!current_zone) {
        return false;
    }
    
    // Calculate remaining distance in the overtake zone
    double remaining_zone_length = current_zone->s_end - env_state_.ego_s;
    if (remaining_zone_length < 0) {
        // Handle track wrap-around
        remaining_zone_length += track_length_;
    }
    
    // Must have enough longitudinal distance to complete the overtake
    // This prevents starting overtakes too close to the end of an overtake zone
    // or near a tight corner
    return remaining_zone_length >= min_longitudinal_window_;
}

bool RacingAgent::is_overtake_feasible() const
{
    // Overtake is feasible only if BOTH lateral clearance AND longitudinal window are sufficient
    
    // Check lateral clearance on at least one side
    bool left_clearance_ok = check_lateral_clearance_for_overtake(true);
    bool right_clearance_ok = check_lateral_clearance_for_overtake(false);
    
    if (!left_clearance_ok && !right_clearance_ok) {
        // Neither side has sufficient clearance
        return false;
    }
    
    // Check longitudinal window
    if (!check_longitudinal_window_for_overtake()) {
        return false;
    }
    
    return true;
}

bool RacingAgent::should_abort_overtake() const
{
    // During OVERTAKE mode, check if conditions become unsafe
    // If so, should abort overtake and return to FOLLOW
    
    if (!active_trajectory_idx_.has_value()) {
        return true;  // No active trajectory, abort
    }
    
    const auto& traj = overtake_trajectories_[*active_trajectory_idx_];
    
    // Check if clearance on the overtake side is still sufficient
    if (!check_lateral_clearance_for_overtake(traj.is_left_side)) {
        // Clearance has become insufficient - abort
        return true;
    }
    
    // Check for immediate collision risk
    if (should_stop_for_obstacle()) {
        return true;
    }
    
    return false;
}

void RacingAgent::update_environment_state()
{
    // Environment state is updated in callbacks
    // This function can be used for additional processing if needed
}

// === Corner Handling Implementation ===

double RacingAgent::compute_local_curvature(size_t raceline_idx) const
{
    if (global_raceline_.poses.size() < 3 || raceline_idx < 1 || 
        raceline_idx >= global_raceline_.poses.size() - 1) {
        return 0.0;
    }
    
    const auto& p0 = global_raceline_.poses[raceline_idx - 1].pose.position;
    const auto& p1 = global_raceline_.poses[raceline_idx].pose.position;
    const auto& p2 = global_raceline_.poses[raceline_idx + 1].pose.position;
    
    // Compute curvature using three points: kappa = 2 * |cross| / (|a||b||c|)
    double dx1 = p1.x - p0.x;
    double dy1 = p1.y - p0.y;
    double dx2 = p2.x - p1.x;
    double dy2 = p2.y - p1.y;
    
    double cross = dx1 * dy2 - dy1 * dx2;
    double ds1 = std::hypot(dx1, dy1);
    double ds2 = std::hypot(dx2, dy2);
    
    if (ds1 < 1e-6 || ds2 < 1e-6) {
        return 0.0;
    }
    
    double avg_ds = (ds1 + ds2) / 2.0;
    return std::abs(cross) / (ds1 * ds2 * avg_ds);
}

bool RacingAgent::is_in_corner() const
{
    if (!raceline_received_ || global_raceline_.poses.empty() || raceline_s_.empty()) {
        return false;
    }
    
    // Find current position on raceline using binary search (O(log n))
    auto it = std::lower_bound(raceline_s_.begin(), raceline_s_.end(), env_state_.ego_s);
    size_t current_idx = (it != raceline_s_.end()) ? 
        std::distance(raceline_s_.begin(), it) : raceline_s_.size() - 1;
    
    // Check curvature at current position and ahead
    double max_curvature = 0.0;
    size_t search_limit = std::min(current_idx + 10, global_raceline_.poses.size() - 1);
    for (size_t i = current_idx; i < search_limit; ++i) {
        double curv = compute_local_curvature(i);
        max_curvature = std::max(max_curvature, curv);
    }
    
    return max_curvature > corner_curvature_threshold_;
}

double RacingAgent::get_corner_adjusted_follow_distance() const
{
    // In corners, increase the follow distance for safety
    if (is_in_corner()) {
        return safe_follow_distance_ * corner_follow_distance_factor_;
    }
    return safe_follow_distance_;
}

// === Inside/Outside Overtake Path Selection ===

RacingAgent::OvertakeSide RacingAgent::determine_best_overtake_side() const
{
    // Determine whether to overtake on inside (apex side) or outside
    // Based on:
    // 1. Track geometry (corner direction)
    // 2. Opponent lateral position
    // 3. Available clearance on each side
    
    if (!raceline_received_ || global_raceline_.poses.empty() || raceline_s_.empty()) {
        return OvertakeSide::NONE;
    }
    
    // Find current position on raceline using binary search (O(log n))
    auto it = std::lower_bound(raceline_s_.begin(), raceline_s_.end(), env_state_.ego_s);
    size_t current_idx = (it != raceline_s_.end()) ? 
        std::distance(raceline_s_.begin(), it) : raceline_s_.size() - 1;
    
    // Determine corner direction (positive = left turn, negative = right turn)
    double corner_direction = 0.0;
    size_t search_limit = std::min(current_idx + 15, global_raceline_.poses.size());
    for (size_t i = current_idx; i < search_limit; ++i) {
        double curv = compute_local_curvature(i);
        if (curv > corner_curvature_threshold_) {
            // Get sign of curvature to determine corner direction
            // Bounds checking: compute_local_curvature already checks i > 0 and i < size - 1
            if (i > 0 && i < global_raceline_.poses.size() - 1) {
                const auto& p0 = global_raceline_.poses[i - 1].pose.position;
                const auto& p1 = global_raceline_.poses[i].pose.position;
                const auto& p2 = global_raceline_.poses[i + 1].pose.position;
                double dx1 = p1.x - p0.x;
                double dy1 = p1.y - p0.y;
                double dx2 = p2.x - p1.x;
                double dy2 = p2.y - p1.y;
                double cross = dx1 * dy2 - dy1 * dx2;
                corner_direction = (cross > 0) ? 1.0 : -1.0;
            }
            break;
        }
    }
    
    // Check clearance on each side
    bool left_clear = check_lateral_clearance_for_overtake(true);
    bool right_clear = check_lateral_clearance_for_overtake(false);
    
    if (!left_clear && !right_clear) {
        return OvertakeSide::NONE;
    }
    
    // Inside overtake preference:
    // - For left turn (corner_direction > 0): inside is RIGHT side
    // - For right turn (corner_direction < 0): inside is LEFT side
    // - On straight: prefer side with more clearance
    
    if (corner_direction > 0.1) {
        // Left turn: inside = right
        if (right_clear) {
            return OvertakeSide::INSIDE;
        } else if (left_clear) {
            return OvertakeSide::OUTSIDE;
        }
    } else if (corner_direction < -0.1) {
        // Right turn: inside = left
        if (left_clear) {
            return OvertakeSide::INSIDE;
        } else if (right_clear) {
            return OvertakeSide::OUTSIDE;
        }
    } else {
        // Straight: prefer side with more clearance
        if (env_state_.left_clearance > env_state_.right_clearance && left_clear) {
            return OvertakeSide::OUTSIDE;  // Arbitrary - could also be INSIDE
        } else if (right_clear) {
            return OvertakeSide::INSIDE;
        } else if (left_clear) {
            return OvertakeSide::OUTSIDE;
        }
    }
    
    return OvertakeSide::NONE;
}

double RacingAgent::compute_inside_overtake_offset() const
{
    // Inside overtake: use a smaller offset as we're cutting the corner
    // This is typically the apex side with less lateral deviation needed
    return OVERTAKE_LATERAL_OFFSET * INSIDE_OVERTAKE_FACTOR;  // Tighter inside line
}

double RacingAgent::compute_outside_overtake_offset() const
{
    // Outside overtake: use the full or slightly larger offset
    // Need more room when going around the outside
    return OVERTAKE_LATERAL_OFFSET * OUTSIDE_OVERTAKE_FACTOR;  // Wider outside line
}

// === Distance-based Overtake Preparation ===

void RacingAgent::prepare_overtake_candidates()
{
    // Pre-select candidate overtake trajectories based on distance to front car
    // This is called during FOLLOW mode to prepare for possible overtaking
    
    if (!env_state_.has_preceding_vehicle) {
        return;
    }
    
    double dist_to_opponent = env_state_.preceding_distance;
    
    // Distance bands for overtake preparation:
    // Band 1: Far (> 4m) - No preparation needed, just follow
    // Band 2: Medium (2.5m - 4m) - Start evaluating trajectories
    // Band 3: Close (1.5m - 2.5m) - Have trajectories ready, evaluate conditions
    // Band 4: Very close (< 1.5m) - Execute if safe, otherwise maintain distance
    
    if (dist_to_opponent > 4.0) {
        // Far: No overtake preparation
        return;
    }
    
    if (dist_to_opponent > 2.5) {
        // Medium distance: Start evaluating which side is better
        OvertakeSide best_side = determine_best_overtake_side();
        if (best_side != OvertakeSide::NONE) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Overtake prep (medium): Evaluating %s side overtake at distance %.2fm",
                best_side == OvertakeSide::INSIDE ? "INSIDE" : "OUTSIDE",
                dist_to_opponent);
        }
    } else if (dist_to_opponent > 1.5) {
        // Close distance: Have trajectory ready
        OvertakeSide best_side = determine_best_overtake_side();
        if (best_side != OvertakeSide::NONE) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Overtake ready: %s side, distance %.2fm, waiting for zone",
                best_side == OvertakeSide::INSIDE ? "INSIDE" : "OUTSIDE",
                dist_to_opponent);
        }
    }
    // Very close: Decision is made in execute_overtake_candidate_mode
}

}  // namespace planning_pkg

// Main entry point
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning_pkg::RacingAgent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
