/**
 * @file racing_agent.hpp
 * @brief Racing Agent Architecture for Path Planning + Overtaking Decision Making + Obstacle Response
 * 
 * Core Philosophy:
 * 1. Overtaking is NOT done ad-hoc, but only in pre-defined OVERTAKE ZONEs on the global raceline
 * 2. Pre-computed overtake trajectory candidates are selected when overtaking
 * 3. For obstacles/preceding vehicles: prioritize "safe following + stop if needed" 
 * 4. NMPC/local controller only follows the reference path/speed given by upper level
 * 
 * Mode Structure:
 * - CRUISE: Normal driving along global raceline at target speed
 * - FOLLOW: Safely follow preceding vehicle within safe distance
 * - OVERTAKE_CANDIDATE: In OVERTAKE ZONE, evaluating overtake path candidates
 * - OVERTAKE: Executing selected pre-computed overtake trajectory
 * - OBSTACLE_STOP: Stopped safely due to obstacle/collision risk
 */

#ifndef PLANNING_PKG_RACING_AGENT_HPP_
#define PLANNING_PKG_RACING_AGENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>
#include <string>
#include <memory>
#include <optional>

namespace planning_pkg
{

/**
 * @brief Racing Agent Mode enumeration
 * 
 * The agent operates in one of these distinct modes at any time.
 * Mode transitions are explicit and controlled by the upper-level strategy.
 */
enum class RacingMode
{
    CRUISE,              ///< Normal driving along global raceline
    FOLLOW,              ///< Safe following of preceding vehicle
    OVERTAKE_CANDIDATE,  ///< Evaluating overtake candidates in OVERTAKE ZONE
    OVERTAKE,            ///< Executing pre-computed overtake trajectory
    OBSTACLE_STOP        ///< Safely stopped due to obstacle/collision
};

/**
 * @brief Convert RacingMode to string for logging/visualization
 */
inline std::string racing_mode_to_string(RacingMode mode)
{
    switch (mode) {
        case RacingMode::CRUISE: return "CRUISE";
        case RacingMode::FOLLOW: return "FOLLOW";
        case RacingMode::OVERTAKE_CANDIDATE: return "OVERTAKE_CANDIDATE";
        case RacingMode::OVERTAKE: return "OVERTAKE";
        case RacingMode::OBSTACLE_STOP: return "OBSTACLE_STOP";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Zone type on the global raceline
 */
enum class ZoneType
{
    NORMAL,        ///< Normal driving zone - no overtaking allowed
    OVERTAKE_ZONE  ///< Pre-defined zone where overtaking is allowed
};

/**
 * @brief Structure representing a zone segment on the raceline
 * 
 * The raceline is parameterized by s (arc length).
 * Zones are defined as [s_start, s_end) segments.
 */
struct RacelineZone
{
    double s_start;    ///< Start position on raceline (arc length, meters)
    double s_end;      ///< End position on raceline (arc length, meters)
    ZoneType type;     ///< Type of zone (NORMAL or OVERTAKE_ZONE)
    std::string name;  ///< Optional name for debugging
    
    RacelineZone(double start = 0.0, double end = 0.0, 
                 ZoneType t = ZoneType::NORMAL, const std::string& n = "")
        : s_start(start), s_end(end), type(t), name(n) {}
    
    /// Check if a given s position is within this zone
    bool contains(double s) const {
        return s >= s_start && s < s_end;
    }
};

/**
 * @brief Pre-computed overtake trajectory
 * 
 * Each OVERTAKE ZONE has multiple pre-computed trajectory candidates.
 * These are generated offline and stored for runtime selection.
 */
struct OvertakeTrajectory
{
    std::string id;           ///< Unique identifier (e.g., "left_lane_aggressive")
    std::string zone_name;    ///< Associated OVERTAKE ZONE name
    double s_start;           ///< Start s position on raceline
    double s_end;             ///< End s position (return to raceline)
    
    /// Waypoints: (s, d_lateral, heading, speed) or (x, y, heading, speed)
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::vector<double> speeds;  ///< Reference speed at each waypoint
    
    /// Metadata for selection
    double estimated_duration;  ///< Estimated time to complete (seconds)
    double lateral_offset_max;  ///< Maximum lateral offset from raceline
    bool is_left_side;          ///< True if overtaking on left side
    
    OvertakeTrajectory() 
        : estimated_duration(0.0), lateral_offset_max(0.0), is_left_side(true) {}
};

/**
 * @brief Upper-level command sent to NMPC/local controller
 * 
 * This structure contains everything the local controller needs.
 * The local controller should NOT make strategic decisions on its own.
 */
struct ControllerCommand
{
    RacingMode mode;                       ///< Current operating mode
    nav_msgs::msg::Path reference_path;    ///< Reference path to follow
    std::vector<double> reference_speeds;  ///< Reference speeds along path
    double speed_limit;                    ///< Current speed limit
    
    /// For OBSTACLE_STOP mode
    bool should_stop;                      ///< True if should decelerate to stop
    
    /// Timestamp
    rclcpp::Time stamp;
    
    ControllerCommand() : mode(RacingMode::CRUISE), speed_limit(0.0), should_stop(false) {}
};

/**
 * @brief Sensor data summary for decision making
 */
struct EnvironmentState
{
    /// Ego vehicle state
    double ego_s;           ///< Ego position on raceline (arc length)
    double ego_d;           ///< Ego lateral deviation from raceline
    double ego_speed;       ///< Ego current speed
    double ego_heading;     ///< Ego heading
    
    /// Preceding vehicle (if detected)
    bool has_preceding_vehicle;
    double preceding_distance;  ///< Distance to preceding vehicle
    double preceding_speed;     ///< Estimated speed of preceding vehicle
    double preceding_d;         ///< Lateral position of preceding vehicle
    
    /// Obstacle detection
    bool front_obstacle_detected;
    double front_obstacle_distance;
    double left_clearance;      ///< Clearance to left (wall/obstacle)
    double right_clearance;     ///< Clearance to right (wall/obstacle)
    
    EnvironmentState() 
        : ego_s(0.0), ego_d(0.0), ego_speed(0.0), ego_heading(0.0),
          has_preceding_vehicle(false), preceding_distance(10.0), 
          preceding_speed(0.0), preceding_d(0.0),
          front_obstacle_detected(false), front_obstacle_distance(10.0),
          left_clearance(10.0), right_clearance(10.0) {}
};

/**
 * @brief Racing Agent - Upper Level Strategy Controller
 * 
 * This class implements the high-level decision making for racing:
 * - Mode transitions (CRUISE, FOLLOW, OVERTAKE_CANDIDATE, OVERTAKE, OBSTACLE_STOP)
 * - OVERTAKE ZONE management
 * - Pre-computed overtake trajectory selection
 * - Reference path/speed generation for NMPC
 */
class RacingAgent : public rclcpp::Node
{
public:
    RacingAgent();
    
    /// Get current operating mode
    RacingMode get_current_mode() const { return current_mode_; }
    
    /// Get command for local controller
    ControllerCommand get_controller_command() const { return current_command_; }

private:
    // === ROS Communication ===
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_raceline_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    
    // Publishers for global overtaking lanes
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr inside_overtake_lane_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr outside_overtake_lane_pub_;
    
    rclcpp::TimerBase::SharedPtr decision_timer_;
    
    // === State ===
    RacingMode current_mode_ = RacingMode::CRUISE;
    RacingMode previous_mode_ = RacingMode::CRUISE;
    ControllerCommand current_command_;
    EnvironmentState env_state_;
    
    // === Global Raceline ===
    nav_msgs::msg::Path global_raceline_;
    std::vector<double> raceline_s_;      ///< Arc length at each waypoint
    std::vector<double> raceline_speeds_; ///< Reference speed at each waypoint
    double track_length_ = 0.0;           ///< Total track length
    bool raceline_received_ = false;
    
    // === Global Overtaking Lanes ===
    nav_msgs::msg::Path global_inside_overtake_lane_;   ///< Inside overtaking lane (full global path)
    nav_msgs::msg::Path global_outside_overtake_lane_;  ///< Outside overtaking lane (full global path)
    std::vector<double> inside_lane_s_;                  ///< Arc length for inside lane
    std::vector<double> outside_lane_s_;                 ///< Arc length for outside lane
    
    // === Zone Management ===
    std::vector<RacelineZone> zones_;     ///< List of zones (NORMAL, OVERTAKE_ZONE)
    
    // === Pre-computed Overtake Trajectories ===
    std::vector<OvertakeTrajectory> overtake_trajectories_;
    std::optional<size_t> active_trajectory_idx_;  ///< Currently executing trajectory
    double overtake_progress_ = 0.0;               ///< Progress along overtake trajectory [0, 1]
    
    // === Parameters ===
    double safe_follow_distance_ = 3.0;   ///< Target following distance (m) - increased for relaxed following
    double min_stop_distance_ = 0.3;      ///< Minimum distance before emergency stop
    double cruise_speed_ = 5.0;           ///< Default cruise speed (m/s)
    double decision_rate_ = 20.0;         ///< Decision loop rate (Hz)
    
    // === Corner Exit Smoothing Parameters ===
    double corner_exit_wall_margin_ = 0.4;    ///< Minimum distance from wall on corner exit (m)
    double corner_exit_lateral_soften_ = 0.7; ///< Factor to reduce lateral OUT on corner exit (0-1)
    
    // === Corner Handling Parameters ===
    double corner_curvature_threshold_ = 0.15; ///< Curvature threshold for corner detection (1/m)
    double corner_follow_distance_factor_ = 1.3; ///< Factor to increase follow distance in corners
    double corner_speed_reduction_ = 0.8;      ///< Speed reduction factor in corners (0-1)
    
    // === Overtake Feasibility Parameters ===
    double opponent_width_ = 0.35;            ///< Assumed opponent vehicle width (m)
    double overtake_width_factor_ = 1.2;      ///< Factor multiplied by opponent width for clearance
    double overtake_lateral_margin_ = 0.25;   ///< Additional safety margin for overtake (m)
    double min_longitudinal_window_ = 5.0;    ///< Minimum distance to complete overtake (m)
    
    // === Callbacks ===
    void raceline_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // === Main Decision Loop ===
    void decision_loop();
    
    // === Mode Transition Logic ===
    RacingMode compute_next_mode();
    void transition_to_mode(RacingMode new_mode);
    
    // === Mode-specific Logic ===
    void execute_cruise_mode();
    void execute_follow_mode();
    void execute_overtake_candidate_mode();
    void execute_overtake_mode();
    void execute_obstacle_stop_mode();
    
    // === Zone Management ===
    void load_zones_from_config(const std::string& config_path);
    ZoneType get_current_zone_type() const;
    const RacelineZone* get_current_zone() const;
    
    // === Overtake Trajectory Management ===
    void load_overtake_trajectories(const std::string& config_path);
    void generate_default_overtake_trajectories();
    std::vector<size_t> get_valid_trajectories_for_current_zone() const;
    std::optional<size_t> select_best_trajectory(const std::vector<size_t>& candidates);
    nav_msgs::msg::Path get_current_overtake_path() const;
    
    // === Global Overtaking Lanes ===
    void generate_global_overtaking_lanes();
    nav_msgs::msg::Path create_overtaking_lane(double lateral_offset, const std::string& lane_name);
    void publish_global_overtaking_lanes();
    visualization_msgs::msg::MarkerArray create_overtaking_lane_markers() const;
    
    // === Reference Path Generation ===
    nav_msgs::msg::Path generate_cruise_reference_path() const;
    nav_msgs::msg::Path generate_follow_reference_path() const;
    nav_msgs::msg::Path generate_stop_path() const;
    
    // === Visualization ===
    void publish_visualization();
    visualization_msgs::msg::MarkerArray create_raceline_markers() const;
    visualization_msgs::msg::MarkerArray create_zone_markers() const;
    visualization_msgs::msg::MarkerArray create_overtake_trajectory_markers() const;
    visualization_msgs::msg::MarkerArray create_mode_marker() const;
    
    // === Utilities ===
    double compute_arc_length_position(double x, double y) const;
    size_t find_closest_raceline_index(double x, double y) const;
    bool is_overtake_safe(const OvertakeTrajectory& trajectory) const;
    bool should_stop_for_obstacle() const;
    bool should_start_following() const;
    bool can_consider_overtake() const;
    void update_environment_state();
    
    // === Corner Handling ===
    double compute_local_curvature(size_t raceline_idx) const;
    bool is_in_corner() const;
    double get_corner_adjusted_follow_distance() const;
    
    // === Inside/Outside Overtake Path Selection ===
    enum class OvertakeSide { INSIDE, OUTSIDE, NONE };
    OvertakeSide determine_best_overtake_side() const;
    double compute_inside_overtake_offset() const;
    double compute_outside_overtake_offset() const;
    
    // === Distance-based Overtake Preparation ===
    void prepare_overtake_candidates();
    
    // === Overtake Feasibility Checks ===
    bool check_lateral_clearance_for_overtake(bool is_left_side) const;
    bool check_longitudinal_window_for_overtake() const;
    bool is_overtake_feasible() const;
    bool should_abort_overtake() const;
};

}  // namespace planning_pkg

#endif  // PLANNING_PKG_RACING_AGENT_HPP_
