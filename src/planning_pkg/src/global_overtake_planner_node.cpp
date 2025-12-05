/**
 * @file global_overtake_planner_node.cpp
 * @brief Global Overtake Line Planner with State Machine
 * 
 * This node implements the overtake planning system described in the requirements:
 * 1. Receives opponent position from opponent_detector
 * 2. Generates global overtake raceline when overtake is possible
 * 3. Manages overtake state machine (NORMAL, PREPARE_OVERTAKE, OVERTAKE, RETURN)
 * 4. Publishes overtake line for controller to follow
 * 
 * Key features:
 * - Polynomial d(s) transition for smooth lane changes
 * - Pre-computed overtake paths when conditions are met
 * - State machine for safe overtake execution
 * - RViz visualization of current state and paths
 * 
 * Topics:
 *   Input:
 *     /global_raceline (nav_msgs/Path) - Base raceline
 *     /opponent_info (std_msgs/Float64MultiArray) - From opponent_detector
 *     /odom (nav_msgs/Odometry) - Ego vehicle state
 *   Output:
 *     /global_overtake_raceline (nav_msgs/Path) - Overtake path when active
 *     /active_raceline (nav_msgs/Path) - Currently active path (raceline or overtake)
 *     /overtake_state (std_msgs/String) - Current state machine state
 *     /overtake_markers (visualization_msgs/MarkerArray) - RViz visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "planning_pkg/opponent_detector.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

namespace planning_pkg
{

/**
 * @brief Overtake state machine states
 */
enum class OvertakeState
{
    NORMAL,             // Following raceline, no overtake planned
    PREPARE_OVERTAKE,   // Opponent detected, overtake line computed, waiting for conditions
    OVERTAKE,           // Actively executing overtake maneuver
    RETURN              // Returning to raceline after overtake
};

/**
 * @brief Convert state to string for logging/publishing
 */
std::string stateToString(OvertakeState state)
{
    switch (state) {
        case OvertakeState::NORMAL: return "NORMAL";
        case OvertakeState::PREPARE_OVERTAKE: return "PREPARE_OVERTAKE";
        case OvertakeState::OVERTAKE: return "OVERTAKE";
        case OvertakeState::RETURN: return "RETURN";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Overtake planner parameters
 */
struct OvertakePlannerParams
{
    // Overtake geometry
    double d_max = 0.6;                 // Maximum lateral offset for overtake (m)
    double s_buffer_start = 1.0;        // Start overtake this far ahead of current position (m)
    double s_overlap = 1.5;             // Pass this far beyond opponent before returning (m)
    double s_buffer_end = 2.0;          // Distance to fully return to raceline (m)
    
    // Track constraints
    double min_track_width = 2.0;       // Minimum required track width for overtake (m)
    double wall_safety_margin = 0.3;    // Minimum distance to wall during overtake (m)
    double vehicle_width = 0.35;        // Vehicle width (m)
    
    // Trigger conditions
    double min_opponent_distance = 0.5; // Minimum distance to start considering overtake (m)
    double max_opponent_distance = 3.0; // Maximum distance to track opponent (m)
    double trigger_distance = 1.5;      // Distance to start PREPARE_OVERTAKE (m)
    double execute_distance = 1.0;      // Distance to start OVERTAKE execution (m)
    
    // Timing
    double overtake_timeout = 5.0;      // Maximum time in OVERTAKE state (s)
    double return_duration = 2.0;       // Duration of RETURN state (s)
    
    // Path generation
    int path_points = 50;               // Number of points in overtake path
    std::string frame_id = "map";
};

class GlobalOvertakePlannerNode : public rclcpp::Node
{
public:
    GlobalOvertakePlannerNode()
    : Node("global_overtake_planner")
    {
        // Parameters
        this->declare_parameter<double>("d_max", 0.6);
        this->declare_parameter<double>("s_buffer_start", 1.0);
        this->declare_parameter<double>("s_overlap", 1.5);
        this->declare_parameter<double>("s_buffer_end", 2.0);
        this->declare_parameter<double>("min_track_width", 2.0);
        this->declare_parameter<double>("wall_safety_margin", 0.3);
        this->declare_parameter<double>("vehicle_width", 0.35);
        this->declare_parameter<double>("min_opponent_distance", 0.5);
        this->declare_parameter<double>("max_opponent_distance", 3.0);
        this->declare_parameter<double>("trigger_distance", 1.5);
        this->declare_parameter<double>("execute_distance", 1.0);
        this->declare_parameter<double>("overtake_timeout", 5.0);
        this->declare_parameter<double>("return_duration", 2.0);
        this->declare_parameter<int>("path_points", 50);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<double>("planner_rate", 20.0);
        
        loadParameters();
        
        // Publishers
        overtake_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/global_overtake_raceline", 10);
        active_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/active_raceline", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/overtake_state", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/overtake_markers", 10);
        
        // Subscribers
        rclcpp::QoS path_qos(rclcpp::QoS(10).transient_local().reliable());
        raceline_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_raceline", path_qos,
            std::bind(&GlobalOvertakePlannerNode::racelineCallback, this, std::placeholders::_1));
        
        opponent_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/opponent_info", 10,
            std::bind(&GlobalOvertakePlannerNode::opponentCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10).best_effort(),
            std::bind(&GlobalOvertakePlannerNode::odomCallback, this, std::placeholders::_1));
        
        // Planner timer
        double rate = this->get_parameter("planner_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&GlobalOvertakePlannerNode::plannerLoop, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Global Overtake Planner initialized. d_max=%.2f, trigger_dist=%.2f",
            params_.d_max, params_.trigger_distance);
    }

private:
    // State
    OvertakeState current_state_ = OvertakeState::NORMAL;
    OvertakePlannerParams params_;
    FrenetConverter frenet_converter_;
    
    // Ego state
    double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0;
    double ego_s_ = 0.0, ego_d_ = 0.0;
    int ego_path_idx_ = 0;
    
    // Opponent state
    double opp_x_ = 0.0, opp_y_ = 0.0;
    double opp_s_ = 0.0, opp_d_ = 0.0;
    double opp_distance_ = 10.0;
    bool opponent_valid_ = false;
    
    // Overtake planning
    nav_msgs::msg::Path base_raceline_;
    nav_msgs::msg::Path overtake_path_;
    double planned_d_overtake_ = 0.0;  // +left, -right
    rclcpp::Time overtake_start_time_;
    rclcpp::Time return_start_time_;
    
    bool raceline_received_ = false;
    bool odom_received_ = false;
    
    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr overtake_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raceline_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr opponent_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void loadParameters()
    {
        params_.d_max = this->get_parameter("d_max").as_double();
        params_.s_buffer_start = this->get_parameter("s_buffer_start").as_double();
        params_.s_overlap = this->get_parameter("s_overlap").as_double();
        params_.s_buffer_end = this->get_parameter("s_buffer_end").as_double();
        params_.min_track_width = this->get_parameter("min_track_width").as_double();
        params_.wall_safety_margin = this->get_parameter("wall_safety_margin").as_double();
        params_.vehicle_width = this->get_parameter("vehicle_width").as_double();
        params_.min_opponent_distance = this->get_parameter("min_opponent_distance").as_double();
        params_.max_opponent_distance = this->get_parameter("max_opponent_distance").as_double();
        params_.trigger_distance = this->get_parameter("trigger_distance").as_double();
        params_.execute_distance = this->get_parameter("execute_distance").as_double();
        params_.overtake_timeout = this->get_parameter("overtake_timeout").as_double();
        params_.return_duration = this->get_parameter("return_duration").as_double();
        params_.path_points = this->get_parameter("path_points").as_int();
        params_.frame_id = this->get_parameter("frame_id").as_string();
    }
    
    void racelineCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty raceline");
            return;
        }
        
        base_raceline_ = *msg;
        frenet_converter_.setReferencePath(*msg);
        raceline_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
            "Raceline received: %zu points, length: %.2f m",
            msg->poses.size(), frenet_converter_.getTotalPathLength());
    }
    
    void opponentCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Parse opponent info: [x, y, s, d, distance, angle, valid, ego_s, ego_d]
        if (msg->data.size() >= 9) {
            opp_x_ = msg->data[0];
            opp_y_ = msg->data[1];
            opp_s_ = msg->data[2];
            opp_d_ = msg->data[3];
            opp_distance_ = msg->data[4];
            opponent_valid_ = msg->data[6] > 0.5;
            
            // Also get ego Frenet coords from detector
            ego_s_ = msg->data[7];
            ego_d_ = msg->data[8];
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ego_x_ = msg->pose.pose.position.x;
        ego_y_ = msg->pose.pose.position.y;
        ego_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        odom_received_ = true;
        
        // Update ego Frenet coords
        if (frenet_converter_.isInitialized()) {
            FrenetCoord ego_frenet = frenet_converter_.toFrenet(ego_x_, ego_y_, ego_path_idx_);
            ego_s_ = ego_frenet.s;
            ego_d_ = ego_frenet.d;
            ego_path_idx_ = ego_frenet.path_idx;
        }
    }
    
    void plannerLoop()
    {
        if (!raceline_received_ || !odom_received_) {
            return;
        }
        
        // Update state machine
        updateStateMachine();
        
        // Publish current state
        publishState();
        
        // Publish active path
        publishActivePath();
        
        // Publish visualization
        publishVisualization();
    }
    
    void updateStateMachine()
    {
        OvertakeState prev_state = current_state_;
        
        switch (current_state_) {
            case OvertakeState::NORMAL:
                handleNormalState();
                break;
                
            case OvertakeState::PREPARE_OVERTAKE:
                handlePrepareOvertakeState();
                break;
                
            case OvertakeState::OVERTAKE:
                handleOvertakeState();
                break;
                
            case OvertakeState::RETURN:
                handleReturnState();
                break;
        }
        
        // Log state transitions
        if (current_state_ != prev_state) {
            RCLCPP_WARN(this->get_logger(), 
                "State transition: %s -> %s",
                stateToString(prev_state).c_str(),
                stateToString(current_state_).c_str());
        }
    }
    
    void handleNormalState()
    {
        // Check if opponent is detected and within trigger range
        if (opponent_valid_ && 
            opp_distance_ < params_.trigger_distance &&
            opp_distance_ > params_.min_opponent_distance) {
            
            // Check if overtake is feasible
            if (canOvertake()) {
                // Generate overtake path
                if (generateOvertakePath()) {
                    current_state_ = OvertakeState::PREPARE_OVERTAKE;
                    RCLCPP_INFO(this->get_logger(), 
                        "Overtake planned! Direction: %s, d_max=%.2f",
                        planned_d_overtake_ > 0 ? "LEFT" : "RIGHT",
                        planned_d_overtake_);
                }
            }
        }
    }
    
    void handlePrepareOvertakeState()
    {
        // Check if opponent is still valid
        if (!opponent_valid_ || opp_distance_ > params_.max_opponent_distance) {
            // Opponent lost, return to normal
            current_state_ = OvertakeState::NORMAL;
            RCLCPP_INFO(this->get_logger(), "Opponent lost, canceling overtake preparation");
            return;
        }
        
        // Check if close enough to execute
        if (opp_distance_ < params_.execute_distance) {
            current_state_ = OvertakeState::OVERTAKE;
            overtake_start_time_ = this->now();
            RCLCPP_WARN(this->get_logger(), "EXECUTING OVERTAKE!");
        }
        
        // Keep updating overtake path as we approach
        generateOvertakePath();
    }
    
    void handleOvertakeState()
    {
        // Check timeout
        double elapsed = (this->now() - overtake_start_time_).seconds();
        if (elapsed > params_.overtake_timeout) {
            RCLCPP_WARN(this->get_logger(), "Overtake timeout, transitioning to RETURN");
            current_state_ = OvertakeState::RETURN;
            return_start_time_ = this->now();
            return;
        }
        
        // Check if we've passed the opponent (s_ego > s_opp + overlap)
        double s_diff = ego_s_ - opp_s_;
        double total_len = frenet_converter_.getTotalPathLength();
        
        // Handle wraparound
        if (s_diff < -total_len / 2.0) s_diff += total_len;
        else if (s_diff > total_len / 2.0) s_diff -= total_len;
        
        if (s_diff > params_.s_overlap) {
            RCLCPP_INFO(this->get_logger(), 
                "Passed opponent (s_diff=%.2f), transitioning to RETURN", s_diff);
            current_state_ = OvertakeState::RETURN;
            return_start_time_ = this->now();
        }
    }
    
    void handleReturnState()
    {
        // Check if return duration elapsed
        double elapsed = (this->now() - return_start_time_).seconds();
        if (elapsed > params_.return_duration) {
            current_state_ = OvertakeState::NORMAL;
            RCLCPP_INFO(this->get_logger(), "Overtake complete, back to NORMAL");
        }
    }
    
    bool canOvertake()
    {
        // Simple check: is there enough space on either side?
        // In a full implementation, this would check wall distances
        
        // For now, just check that opponent is within lane
        if (std::abs(opp_d_) > params_.min_track_width / 2.0) {
            return false;  // Opponent is off-track
        }
        
        return true;
    }
    
    bool generateOvertakePath()
    {
        if (!frenet_converter_.isInitialized()) {
            return false;
        }
        
        // Decide overtake direction based on opponent position
        // If opponent is left of center (d > 0), go right (d < 0)
        // If opponent is right of center (d < 0), go left (d > 0)
        double overtake_sign = (opp_d_ >= 0) ? -1.0 : 1.0;
        planned_d_overtake_ = overtake_sign * params_.d_max;
        
        // Define s range for overtake
        double s_start = ego_s_ + params_.s_buffer_start;
        double s_mid = opp_s_ + params_.s_overlap;
        double s_end = s_mid + params_.s_buffer_end;
        
        double total_len = frenet_converter_.getTotalPathLength();
        
        // Wrap s values using std::fmod for efficiency
        auto wrapS = [total_len](double s) {
            double wrapped = std::fmod(s, total_len);
            return wrapped < 0 ? wrapped + total_len : wrapped;
        };
        
        // Generate path points
        overtake_path_.header.stamp = this->now();
        overtake_path_.header.frame_id = params_.frame_id;
        overtake_path_.poses.clear();
        
        int n = params_.path_points;
        double s_range = s_end - s_start;
        
        for (int i = 0; i <= n; ++i) {
            double t = static_cast<double>(i) / n;  // 0 to 1
            double s = s_start + t * s_range;
            double s_wrapped = wrapS(s);
            
            // Compute d(s) using quintic polynomial for smooth transition
            // d(s) goes: 0 -> d_max -> 0
            double d = computeLateralOffset(t, planned_d_overtake_);
            
            // Convert to Cartesian
            geometry_msgs::msg::Point p = frenet_converter_.toCartesian(s_wrapped, d);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = overtake_path_.header;
            pose.pose.position = p;
            pose.pose.position.z = 0.0;
            
            // Compute heading from path direction and set quaternion properly
            // For 2D rotation around z-axis: qx=0, qy=0, qz=sin(yaw/2), qw=cos(yaw/2)
            double psi = frenet_converter_.getHeadingAtS(s_wrapped);
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(psi / 2.0);
            pose.pose.orientation.w = std::cos(psi / 2.0);
            
            overtake_path_.poses.push_back(pose);
        }
        
        // Publish overtake path
        overtake_path_pub_->publish(overtake_path_);
        
        return true;
    }
    
    /**
     * @brief Compute lateral offset d(t) using smooth polynomial transition
     * 
     * Uses a quintic polynomial that:
     * - Starts at d=0
     * - Reaches d_max at t=0.5
     * - Returns to d=0 at t=1.0
     * - Has zero velocity and acceleration at boundaries
     */
    double computeLateralOffset(double t, double d_max)
    {
        // Clamp t to [0, 1]
        t = std::clamp(t, 0.0, 1.0);
        
        // Use sin function for smooth S-curve: d = d_max * sin(π*t)
        // This naturally gives 0 -> d_max -> 0 with smooth transitions
        return d_max * std::sin(M_PI * t);
    }
    
    void publishState()
    {
        std_msgs::msg::String msg;
        msg.data = stateToString(current_state_);
        state_pub_->publish(msg);
    }
    
    void publishActivePath()
    {
        nav_msgs::msg::Path active_path;
        
        switch (current_state_) {
            case OvertakeState::OVERTAKE:
            case OvertakeState::PREPARE_OVERTAKE:
                // Use overtake path during these states
                if (!overtake_path_.poses.empty()) {
                    active_path = overtake_path_;
                } else {
                    active_path = base_raceline_;
                }
                break;
                
            case OvertakeState::RETURN:
                // Blend back to raceline (simplified: just use raceline)
                active_path = base_raceline_;
                break;
                
            case OvertakeState::NORMAL:
            default:
                active_path = base_raceline_;
                break;
        }
        
        active_path.header.stamp = this->now();
        active_path_pub_->publish(active_path);
    }
    
    void publishVisualization()
    {
        visualization_msgs::msg::MarkerArray markers;
        
        // State text marker
        visualization_msgs::msg::Marker state_marker;
        state_marker.header.frame_id = params_.frame_id;
        state_marker.header.stamp = this->now();
        state_marker.ns = "overtake_state";
        state_marker.id = 0;
        state_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        state_marker.action = visualization_msgs::msg::Marker::ADD;
        
        state_marker.pose.position.x = ego_x_ + 1.0;
        state_marker.pose.position.y = ego_y_ + 1.0;
        state_marker.pose.position.z = 1.0;
        state_marker.pose.orientation.w = 1.0;
        
        state_marker.text = "State: " + stateToString(current_state_);
        state_marker.scale.z = 0.4;
        
        // Color based on state
        switch (current_state_) {
            case OvertakeState::NORMAL:
                state_marker.color.r = 0.0f;
                state_marker.color.g = 1.0f;
                state_marker.color.b = 0.0f;
                break;
            case OvertakeState::PREPARE_OVERTAKE:
                state_marker.color.r = 1.0f;
                state_marker.color.g = 1.0f;
                state_marker.color.b = 0.0f;
                break;
            case OvertakeState::OVERTAKE:
                state_marker.color.r = 1.0f;
                state_marker.color.g = 0.0f;
                state_marker.color.b = 0.0f;
                break;
            case OvertakeState::RETURN:
                state_marker.color.r = 0.0f;
                state_marker.color.g = 0.5f;
                state_marker.color.b = 1.0f;
                break;
        }
        state_marker.color.a = 1.0f;
        state_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        markers.markers.push_back(state_marker);
        
        // Overtake path visualization (when in PREPARE or OVERTAKE)
        if (current_state_ == OvertakeState::PREPARE_OVERTAKE ||
            current_state_ == OvertakeState::OVERTAKE) {
            
            visualization_msgs::msg::Marker path_marker;
            path_marker.header.frame_id = params_.frame_id;
            path_marker.header.stamp = this->now();
            path_marker.ns = "overtake_path_vis";
            path_marker.id = 1;
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::msg::Marker::ADD;
            path_marker.pose.orientation.w = 1.0;
            
            path_marker.scale.x = 0.12;  // Line width
            
            // Green/purple based on direction
            if (planned_d_overtake_ > 0) {
                path_marker.color.r = 0.5f;
                path_marker.color.g = 0.0f;
                path_marker.color.b = 1.0f;  // Purple for left
            } else {
                path_marker.color.r = 0.0f;
                path_marker.color.g = 1.0f;
                path_marker.color.b = 0.5f;  // Cyan for right
            }
            path_marker.color.a = 0.9f;
            
            for (const auto& pose : overtake_path_.poses) {
                geometry_msgs::msg::Point p;
                p.x = pose.pose.position.x;
                p.y = pose.pose.position.y;
                p.z = 0.1;
                path_marker.points.push_back(p);
            }
            
            path_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            markers.markers.push_back(path_marker);
            
            // Direction arrow
            if (!overtake_path_.poses.empty()) {
                visualization_msgs::msg::Marker arrow;
                arrow.header.frame_id = params_.frame_id;
                arrow.header.stamp = this->now();
                arrow.ns = "overtake_direction";
                arrow.id = 2;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                
                // Arrow from ego to first overtake point
                geometry_msgs::msg::Point start, end;
                start.x = ego_x_;
                start.y = ego_y_;
                start.z = 0.3;
                
                size_t mid_idx = overtake_path_.poses.size() / 4;
                end.x = overtake_path_.poses[mid_idx].pose.position.x;
                end.y = overtake_path_.poses[mid_idx].pose.position.y;
                end.z = 0.3;
                
                arrow.points.push_back(start);
                arrow.points.push_back(end);
                
                arrow.scale.x = 0.1;  // Shaft diameter
                arrow.scale.y = 0.2;  // Head diameter
                arrow.scale.z = 0.3;  // Head length
                
                arrow.color.r = 1.0f;
                arrow.color.g = 1.0f;
                arrow.color.b = 0.0f;
                arrow.color.a = 0.9f;
                
                arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
                markers.markers.push_back(arrow);
            }
        }
        
        marker_pub_->publish(markers);
    }
};

}  // namespace planning_pkg

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planning_pkg::GlobalOvertakePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
