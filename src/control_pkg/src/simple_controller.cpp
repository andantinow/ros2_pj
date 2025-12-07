#include "simple_controller.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <rclcpp/qos.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

SimpleController::SimpleController() : Node("simple_controller")
{
  RCLCPP_INFO(this->get_logger(), "Simple Controller Initialized");

  declare_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter("min_lookahead", min_lookahead_);
  declare_parameter("max_lookahead", max_lookahead_);
  declare_parameter("lookahead_speed_gain", lookahead_speed_gain_);
  declare_parameter("lookahead_error_gain", lookahead_error_gain_);
  declare_parameter("target_speed", target_speed_);
  declare_parameter("max_speed", max_speed_);
  declare_parameter("wheelbase", wheelbase_);
  declare_parameter("max_steer_angle", max_steer_angle_);
  declare_parameter("max_steer_rate", max_steer_rate_);
  declare_parameter("lateral_error_gain", lateral_error_gain_);
  declare_parameter("heading_error_gain", heading_error_gain_);
  declare_parameter("curvature_feedforward_gain", curvature_feedforward_gain_);
  declare_parameter("smoothing_factor", smoothing_factor_);
  declare_parameter("steering_sign", steering_sign_);
  declare_parameter("direct_correction_gain", direct_correction_gain_);
  declare_parameter("direct_correction_limit", direct_correction_limit_);
  declare_parameter("pid_kp", pid_kp_);
  declare_parameter("pid_ki", pid_ki_);
  declare_parameter("pid_kd", pid_kd_);
  declare_parameter("pid_integral_limit", 0.0);  
  declare_parameter("use_path_interpolation", use_path_interpolation_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("path_topic", path_topic_);
  declare_parameter<std::string>("drive_topic", drive_topic_);
  declare_parameter<std::string>("scan_topic", scan_topic_);
  declare_parameter("a1_threshold", a1_threshold_);
  declare_parameter("a2_threshold", a2_threshold_);
  declare_parameter("a2_urgent_threshold", a2_urgent_threshold_);
  declare_parameter("a1_side_factor", a1_side_factor_);
  declare_parameter("a2_max_steer_ratio", a2_max_steer_ratio_);
  declare_parameter("a2_urgent_steer_ratio", a2_urgent_steer_ratio_);
  declare_parameter("reverse_speed", reverse_speed_);
  declare_parameter("reverse_duration", reverse_duration_);
  declare_parameter("a1_steer_gain", a1_steer_gain_);
  declare_parameter("a2_steer_gain", a2_steer_gain_);
  declare_parameter("a2_urgent_steer_gain", a2_urgent_steer_gain_);
  declare_parameter("reverse_pause_duration", reverse_pause_duration_);
  declare_parameter("wall_repulsion_gain", wall_repulsion_gain_);
  declare_parameter("wall_repulsion_threshold", wall_repulsion_threshold_);
  declare_parameter("wall_repulsion_max_steer", wall_repulsion_max_steer_);
  declare_parameter("follow_distance_threshold", follow_distance_threshold_);
  declare_parameter("follow_min_distance", follow_min_distance_);
  declare_parameter("follow_speed_factor", follow_speed_factor_);
  declare_parameter("enable_collision_avoidance", enable_collision_avoidance_);
  declare_parameter("corner_curvature_threshold", corner_curvature_threshold_);
  declare_parameter("corner_speed_factor", corner_speed_factor_);
  declare_parameter("corner_steer_amplify", corner_steer_amplify_);
  declare_parameter("min_corner_speed_factor", min_corner_speed_factor_);
  declare_parameter("corner_approach_distance", corner_approach_distance_);
  declare_parameter("out_in_out_offset", out_in_out_offset_);
  declare_parameter("enable_out_in_out", enable_out_in_out_);
  declare_parameter("overtake_max_duration", overtake_max_duration_);
  declare_parameter("overtake_lateral_offset", overtake_lateral_offset_);
  declare_parameter("return_to_line_distance", return_to_line_distance_);
  declare_parameter("min_overtake_gap", min_overtake_gap_);
  declare_parameter("overtake_speed_boost", overtake_speed_boost_);
  declare_parameter("post_overtake_speed_factor", post_overtake_speed_factor_);
  declare_parameter("post_overtake_duration", post_overtake_duration_);
  declare_parameter("overtake_steer_max", overtake_steer_max_);
  declare_parameter("overtake_steer_decay", overtake_steer_decay_);
  declare_parameter("overtake_wall_caution_dist", overtake_wall_caution_dist_);
  declare_parameter("speed_smooth_factor", speed_smooth_factor_);
  declare_parameter("follow_min_speed_ratio", follow_min_speed_ratio_);
  declare_parameter("narrow_road_threshold", narrow_road_threshold_);
  declare_parameter("min_visibility_angle", min_visibility_angle_);
  declare_parameter("vehicle_width", vehicle_width_);
  declare_parameter("vehicle_length", vehicle_length_);
  declare_parameter<std::string>("wall_data_file", "");
  declare_parameter("reverse_steering_mode", reverse_steering_mode_);  
  declare_parameter<std::string>("imu_topic", "/imu");
  declare_parameter("acc_kp", acc_kp_);
  declare_parameter("acc_kd", acc_kd_);
  declare_parameter("target_follow_gap", target_follow_gap_);

  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  min_lookahead_ = get_parameter("min_lookahead").as_double();
  max_lookahead_ = get_parameter("max_lookahead").as_double();
  lookahead_speed_gain_ = get_parameter("lookahead_speed_gain").as_double();
  lookahead_error_gain_ = get_parameter("lookahead_error_gain").as_double();
  target_speed_ = get_parameter("target_speed").as_double();
  wheelbase_ = get_parameter("wheelbase").as_double();
  max_steer_angle_ = get_parameter("max_steer_angle").as_double();
  max_speed_ = get_parameter("max_speed").as_double();
  max_steer_rate_ = get_parameter("max_steer_rate").as_double();
  lateral_error_gain_ = get_parameter("lateral_error_gain").as_double();
  heading_error_gain_ = get_parameter("heading_error_gain").as_double();
  curvature_feedforward_gain_ = get_parameter("curvature_feedforward_gain").as_double();
  smoothing_factor_ = get_parameter("smoothing_factor").as_double();
  steering_sign_ = get_parameter("steering_sign").as_double();
  direct_correction_gain_ = get_parameter("direct_correction_gain").as_double();
  direct_correction_limit_ = get_parameter("direct_correction_limit").as_double();
  pid_kp_ = get_parameter("pid_kp").as_double();
  pid_ki_ = get_parameter("pid_ki").as_double();
  pid_kd_ = get_parameter("pid_kd").as_double();
  pid_integral_limit_ = get_parameter("pid_integral_limit").as_double();
  use_path_interpolation_ = get_parameter("use_path_interpolation").as_bool();
  odom_topic_ = get_parameter("odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  drive_topic_ = get_parameter("drive_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  a1_threshold_ = get_parameter("a1_threshold").as_double();
  a2_threshold_ = get_parameter("a2_threshold").as_double();
  a2_urgent_threshold_ = get_parameter("a2_urgent_threshold").as_double();
  a1_side_factor_ = get_parameter("a1_side_factor").as_double();
  a2_max_steer_ratio_ = get_parameter("a2_max_steer_ratio").as_double();
  a2_urgent_steer_ratio_ = get_parameter("a2_urgent_steer_ratio").as_double();
  reverse_speed_ = get_parameter("reverse_speed").as_double();
  reverse_duration_ = get_parameter("reverse_duration").as_double();
  a1_steer_gain_ = get_parameter("a1_steer_gain").as_double();
  a2_steer_gain_ = get_parameter("a2_steer_gain").as_double();
  a2_urgent_steer_gain_ = get_parameter("a2_urgent_steer_gain").as_double();
  reverse_pause_duration_ = get_parameter("reverse_pause_duration").as_double();
  wall_repulsion_gain_ = get_parameter("wall_repulsion_gain").as_double();
  wall_repulsion_threshold_ = get_parameter("wall_repulsion_threshold").as_double();
  wall_repulsion_max_steer_ = get_parameter("wall_repulsion_max_steer").as_double();
  follow_distance_threshold_ = get_parameter("follow_distance_threshold").as_double();
  follow_min_distance_ = get_parameter("follow_min_distance").as_double();
  follow_speed_factor_ = get_parameter("follow_speed_factor").as_double();
  enable_collision_avoidance_ = get_parameter("enable_collision_avoidance").as_bool();
  corner_curvature_threshold_ = get_parameter("corner_curvature_threshold").as_double();
  corner_speed_factor_ = get_parameter("corner_speed_factor").as_double();
  corner_steer_amplify_ = get_parameter("corner_steer_amplify").as_double();
  min_corner_speed_factor_ = get_parameter("min_corner_speed_factor").as_double();
  corner_approach_distance_ = get_parameter("corner_approach_distance").as_double();
  out_in_out_offset_ = get_parameter("out_in_out_offset").as_double();
  enable_out_in_out_ = get_parameter("enable_out_in_out").as_bool();
  overtake_max_duration_ = get_parameter("overtake_max_duration").as_double();
  overtake_lateral_offset_ = get_parameter("overtake_lateral_offset").as_double();
  return_to_line_distance_ = get_parameter("return_to_line_distance").as_double();
  min_overtake_gap_ = get_parameter("min_overtake_gap").as_double();
  overtake_speed_boost_ = get_parameter("overtake_speed_boost").as_double();
  post_overtake_speed_factor_ = get_parameter("post_overtake_speed_factor").as_double();
  post_overtake_duration_ = get_parameter("post_overtake_duration").as_double();
  overtake_steer_max_ = get_parameter("overtake_steer_max").as_double();
  overtake_steer_decay_ = get_parameter("overtake_steer_decay").as_double();
  overtake_wall_caution_dist_ = get_parameter("overtake_wall_caution_dist").as_double();
  speed_smooth_factor_ = get_parameter("speed_smooth_factor").as_double();
  follow_min_speed_ratio_ = get_parameter("follow_min_speed_ratio").as_double();
  narrow_road_threshold_ = get_parameter("narrow_road_threshold").as_double();
  min_visibility_angle_ = get_parameter("min_visibility_angle").as_double();
  vehicle_width_ = get_parameter("vehicle_width").as_double();
  vehicle_length_ = get_parameter("vehicle_length").as_double();
  
  reverse_steering_mode_ = get_parameter("reverse_steering_mode").as_int();
  std::string imu_topic = get_parameter("imu_topic").as_string();
  
  acc_kp_ = get_parameter("acc_kp").as_double();
  acc_kd_ = get_parameter("acc_kd").as_double();
  target_follow_gap_ = get_parameter("target_follow_gap").as_double();
  
  opponent_width_with_margin_ = vehicle_width_ + 2.0 * SAFETY_MARGIN;  
  
  min_overtake_clearance_ = vehicle_width_ * 2.0 + SAFETY_MARGIN;  
  RCLCPP_INFO(this->get_logger(), 
              "Vehicle: %.2fm x %.2fm, overtake_clearance: %.2fm, D_forbidden: %.2fm (lateral exclusion zone)",
              vehicle_width_, vehicle_length_, min_overtake_clearance_, opponent_width_with_margin_);
  RCLCPP_INFO(this->get_logger(), "CRSM enabled: reverse_steering_mode=%d (0=neutral, 1=invert, 2=maintain)",
              reverse_steering_mode_);
  RCLCPP_INFO(this->get_logger(), "ACC enabled: Kp=%.2f, Kd=%.2f, target_gap=%.2fm",
              acc_kp_, acc_kd_, target_follow_gap_);
  
  std::string wall_data_file = get_parameter("wall_data_file").as_string();
  if (!wall_data_file.empty()) {
    if (load_wall_data(wall_data_file)) {
      RCLCPP_INFO(this->get_logger(), "Wall data loaded: %zu points", wall_data_.size());
    }
  }
  
  prev_time_ = this->get_clock()->now();
  reverse_start_time_ = this->get_clock()->now();
  pause_start_time_ = this->get_clock()->now();
  planning_start_time_ = this->get_clock()->now();
  overtake_start_time_ros_ = this->get_clock()->now();
  overtake_end_time_ros_ = this->get_clock()->now();
  prev_opponent_time_ = this->get_clock()->now();  

  rclcpp::QoS path_qos(rclcpp::QoS(10).transient_local().reliable());
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      path_qos,
      std::bind(&SimpleController::path_callback, this, std::placeholders::_1));

  rclcpp::QoS odom_qos(rclcpp::QoS(10).best_effort());
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      odom_qos,
      std::bind(&SimpleController::odom_callback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SimpleController::scan_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SimpleController::imu_callback, this, std::placeholders::_1));

  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/racing_agent/mode",
      rclcpp::QoS(10),
      std::bind(&SimpleController::mode_callback, this, std::placeholders::_1));

  // Subscribe to v_ref topic (published by raceline_server)
  rclcpp::QoS vref_qos(rclcpp::QoS(10).transient_local().reliable());
  vref_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/global_vref",
      vref_qos,
      std::bind(&SimpleController::vref_callback, this, std::placeholders::_1));

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_,
      10);
  
  lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/pid_lookahead_point",
      rclcpp::QoS(1));
  
  overtake_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/overtake_path",
      rclcpp::QoS(1));
  
  wall_collision_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/wall_collision_indicator",
      rclcpp::QoS(1));
  
  state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/crsm_state",
      rclcpp::QoS(1));
  
  safety_zone_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/safety_zones",
      rclcpp::QoS(1));
  
  RCLCPP_INFO(this->get_logger(), "Subscribing to odom: %s, path: %s, scan: %s, imu: %s, publishing to drive: %s", 
              odom_topic_.c_str(), path_topic_.c_str(), scan_topic_.c_str(), imu_topic.c_str(), drive_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "A1/A2 Collision avoidance: A1=%.2fm (reverse), A2=%.2fm (steer), reverse_speed=%.2fm/s",
              a1_threshold_, a2_threshold_, reverse_speed_);
  RCLCPP_INFO(this->get_logger(), "Lookahead point visualization: /pid_lookahead_point (RED sphere)");

  timer_ = this->create_wall_timer(
      20ms, std::bind(&SimpleController::control_loop, this));
}

void SimpleController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
  if (!odom_received_) {
    RCLCPP_INFO(this->get_logger(), "Received first odometry message. Frame: %s, Position: (%.2f, %.2f, %.2f)",
                msg->header.frame_id.c_str(),
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);
    odom_received_ = true;
  }
}

void SimpleController::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = *msg;
  if (!path_received_) {
    RCLCPP_INFO(this->get_logger(), "Received first path message with %zu points, frame_id: %s", 
                msg->poses.size(), msg->header.frame_id.c_str());
    if (!msg->poses.empty()) {
      RCLCPP_INFO(this->get_logger(), "First path point: (%.2f, %.2f), Last: (%.2f, %.2f)",
                  msg->poses[0].pose.position.x, msg->poses[0].pose.position.y,
                  msg->poses.back().pose.position.x, msg->poses.back().pose.position.y);
    }
    path_received_ = true;
  }
}

void SimpleController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  current_scan_ = *msg;
  if (!scan_received_) {
    RCLCPP_INFO(this->get_logger(), "Received first scan message. Ranges: %zu, angle: [%.2f, %.2f]",
                msg->ranges.size(), msg->angle_min, msg->angle_max);
    scan_received_ = true;
  }
}

void SimpleController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_imu_ = *msg;
  
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  imu_accel_magnitude_ = std::sqrt(ax * ax + ay * ay);
  
  if (!imu_received_) {
    RCLCPP_INFO(this->get_logger(), "Received first IMU message. Accel: (%.2f, %.2f) m/s^2",
                ax, ay);
    imu_received_ = true;
  }
}

void SimpleController::mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (current_racing_mode_ != msg->data) {
    RCLCPP_INFO(this->get_logger(), "Racing mode changed: %s -> %s",
                current_racing_mode_.c_str(), msg->data.c_str());
    current_racing_mode_ = msg->data;
    
    if (current_racing_mode_ == "OBSTACLE_STOP") {
      RCLCPP_WARN(this->get_logger(), 
                  "OBSTACLE_STOP mode: Initiating safe stop (no repulsion/bouncing)");
    }
  }
}

void SimpleController::vref_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  current_vref_ = msg->data;
  if (!vref_received_) {
    RCLCPP_INFO(this->get_logger(), "Received v_ref array: %zu speeds", current_vref_.size());
    if (!current_vref_.empty()) {
      float vref_min = *std::min_element(current_vref_.begin(), current_vref_.end());
      float vref_max = *std::max_element(current_vref_.begin(), current_vref_.end());
      RCLCPP_INFO(this->get_logger(), "v_ref range: %.2f - %.2f m/s", vref_min, vref_max);
    }
    vref_received_ = true;
  }
}

bool SimpleController::is_obstacle_stop_mode() const
{
  return current_racing_mode_ == "OBSTACLE_STOP";
}

void SimpleController::execute_obstacle_stop()
{
  static constexpr double OBSTACLE_STOP_STEERING_DECAY = 0.95;
  
  rclcpp::Time current_time = this->get_clock()->now();
  
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = current_time;
  drive_msg.header.frame_id = current_odom_.child_frame_id;
  
  drive_msg.drive.speed = 0.0;
  
  double stable_steer = prev_steering_angle_ * OBSTACLE_STOP_STEERING_DECAY;
  drive_msg.drive.steering_angle = stable_steer;
  
  drive_pub_->publish(drive_msg);
  last_cmd_velocity_ = 0.0;
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "OBSTACLE_STOP: v_ref=0, steering=%.3f (stable, no repulsion)",
                       stable_steer);
}

void SimpleController::update_obstacle_distances()
{
  if (!scan_received_ || current_scan_.ranges.empty()) {
    last_obstacle_front_dist_ = 10.0;
    last_obstacle_left_dist_ = 10.0;
    last_obstacle_right_dist_ = 10.0;
    last_obstacle_angle_ = 0.0;
    return;
  }
  
  constexpr double FRONT_SECTOR_HALF_ANGLE = M_PI / 3.0;   
  constexpr double SIDE_INNER = M_PI / 6.0;               
  constexpr double SIDE_OUTER = M_PI / 2.0;               
  constexpr double MIN_VALID_RANGE = 0.03;
  
  double angle_min = current_scan_.angle_min;
  double angle_inc = current_scan_.angle_increment;
  int num_ranges = current_scan_.ranges.size();
  
  double min_front = std::numeric_limits<double>::max();
  double min_left = std::numeric_limits<double>::max();
  double min_right = std::numeric_limits<double>::max();
  double front_angle = 0.0;
  
  for (int i = 0; i < num_ranges; ++i) {
    double angle = angle_min + i * angle_inc;
    double range = current_scan_.ranges[i];
    
    if (std::isnan(range) || std::isinf(range) || range < MIN_VALID_RANGE) {
      continue;
    }
    
    if (angle >= -FRONT_SECTOR_HALF_ANGLE && angle <= FRONT_SECTOR_HALF_ANGLE) {
      if (range < min_front) {
        min_front = range;
        front_angle = angle;
      }
    }
    
    if (angle >= SIDE_INNER && angle <= SIDE_OUTER) {
      if (range < min_left) {
        min_left = range;
      }
    }
    
    if (angle >= -SIDE_OUTER && angle <= -SIDE_INNER) {
      if (range < min_right) {
        min_right = range;
      }
    }
  }
  
  last_obstacle_front_dist_ = min_front;
  last_obstacle_left_dist_ = min_left;
  last_obstacle_right_dist_ = min_right;
  last_obstacle_angle_ = front_angle;
}

bool SimpleController::check_lidar_box_occupancy(double front_dist, double lateral_width, double angle_range)
{
  if (!scan_received_ || current_scan_.ranges.empty()) {
    return false;
  }
  
  double angle_min = current_scan_.angle_min;
  double angle_inc = current_scan_.angle_increment;
  int num_ranges = current_scan_.ranges.size();
  
  constexpr double MIN_VALID_RANGE = 0.03;
  
  // Check if any LiDAR point falls within the virtual box
  for (int i = 0; i < num_ranges; ++i) {
    double angle = angle_min + i * angle_inc;
    double range = current_scan_.ranges[i];
    
    if (std::isnan(range) || std::isinf(range) || range < MIN_VALID_RANGE) {
      continue;
    }
    
    // Check if point is within the box
    // Front distance check
    if (range > front_dist) {
      continue;
    }
    
    // Lateral width check (angle range)
    if (std::abs(angle) > angle_range) {
      continue;
    }
    
    // Calculate lateral distance (y in vehicle frame)
    double lateral_dist = std::abs(range * std::sin(angle));
    
    if (lateral_dist < lateral_width / 2.0) {
      return true;  // Found a point in the box
    }
  }
  
  return false;  // No points found in the box
}

void SimpleController::update_virtual_boxes()
{
  if (!scan_received_) {
    opponent_in_safety_box_ = false;
    opponent_in_follow_box_ = false;
    overtake_path_clear_ = true;
    return;
  }
  
  // Check safety box (Box 1) - very close, emergency stop zone
  opponent_in_safety_box_ = check_lidar_box_occupancy(
    LIDAR_BOX_SAFETY_FRONT,
    LIDAR_BOX_SAFETY_WIDTH,
    M_PI / 6.0  // ±30 degrees
  );
  
  // Check follow box (Box 2) - maintain distance zone
  opponent_in_follow_box_ = check_lidar_box_occupancy(
    LIDAR_BOX_FOLLOW_FRONT,
    LIDAR_BOX_FOLLOW_WIDTH,
    M_PI / 4.0  // ±45 degrees
  );
  
  // Check overtake path clearance (Box 3) - wider check for obstacles in overtake zone
  // This checks if there are any obstacles in the wider overtake assessment area
  // The actual left/right overtake decision is made based on last_obstacle_left_dist_ 
  // and last_obstacle_right_dist_ in the can_overtake_safely() function
  overtake_path_clear_ = !check_lidar_box_occupancy(
    LIDAR_BOX_OVERTAKE_FRONT,
    LIDAR_BOX_OVERTAKE_WIDTH,
    M_PI / 3.0  // ±60 degrees for wider check
  );
  
  // Log FSM state changes
  static bool prev_safety = false;
  static bool prev_follow = false;
  static bool prev_overtake_clear = true;
  
  if (opponent_in_safety_box_ != prev_safety) {
    RCLCPP_INFO(this->get_logger(), 
                "FSM: Safety box %s", 
                opponent_in_safety_box_ ? "OCCUPIED (STOP/REVERSE)" : "CLEAR");
    prev_safety = opponent_in_safety_box_;
  }
  
  if (opponent_in_follow_box_ != prev_follow) {
    RCLCPP_INFO(this->get_logger(), 
                "FSM: Follow box %s", 
                opponent_in_follow_box_ ? "OCCUPIED (FOLLOW)" : "CLEAR");
    prev_follow = opponent_in_follow_box_;
  }
  
  if (overtake_path_clear_ != prev_overtake_clear) {
    RCLCPP_INFO(this->get_logger(), 
                "FSM: Overtake path %s", 
                overtake_path_clear_ ? "CLEAR (ASSESS_OVERTAKE)" : "BLOCKED");
    prev_overtake_clear = overtake_path_clear_;
  }
}

bool SimpleController::check_a1_zone()
{
  if (!enable_collision_avoidance_ || !scan_received_) {
    is_in_a1_zone_ = false;
    return false;
  }
  
  double side_threshold = a1_threshold_ * a1_side_factor_;
  
  is_in_a1_zone_ = (last_obstacle_front_dist_ < a1_threshold_) ||
                   (last_obstacle_left_dist_ < side_threshold) ||
                   (last_obstacle_right_dist_ < side_threshold);
  
  if (is_in_a1_zone_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "A1 Zone! Front: %.2fm, L: %.2fm, R: %.2fm (threshold: %.2fm, side: %.2fm)",
                         last_obstacle_front_dist_, last_obstacle_left_dist_, 
                         last_obstacle_right_dist_, a1_threshold_, side_threshold);
  }
  
  return is_in_a1_zone_;
}

bool SimpleController::check_a2_zone()
{
  if (!enable_collision_avoidance_ || !scan_received_) {
    return false;
  }
  
  
  bool in_a2 = (last_obstacle_front_dist_ < a2_threshold_) ||
               (last_obstacle_left_dist_ < a2_threshold_) ||
               (last_obstacle_right_dist_ < a2_threshold_);
  
  return in_a2;
}

double SimpleController::compute_reverse_steering()
{
  double reverse_steer = 0.0;
  
  double left_dist = std::min(last_obstacle_left_dist_, 5.0);
  double right_dist = std::min(last_obstacle_right_dist_, 5.0);
  
  double dist_diff = right_dist - left_dist;
  
  if (std::abs(dist_diff) > 0.05) {
    reverse_steer = -std::copysign(1.0, dist_diff) * max_steer_angle_ * a1_steer_gain_;
  } else if (std::abs(last_steering_before_reverse_) > 0.05) {
    reverse_steer = -last_steering_before_reverse_ * a1_steer_gain_;
  } else if (std::abs(last_obstacle_angle_) > 0.1) {
    reverse_steer = -last_obstacle_angle_ * a1_steer_gain_;
  }
  
  reverse_steer = std::clamp(reverse_steer, -max_steer_angle_, max_steer_angle_);
  
  RCLCPP_DEBUG(this->get_logger(), 
               "A1 Reverse steer: %.3f (L:%.2f, R:%.2f, angle:%.2f)",
               reverse_steer, left_dist, right_dist, last_obstacle_angle_);
  
  return reverse_steer;
}

double SimpleController::find_gap_center_angle(double lookahead_angle)
{
  if (!scan_received_ || current_scan_.ranges.empty()) {
    return lookahead_angle;  
  }
  
  double angle_min = current_scan_.angle_min;
  double angle_inc = current_scan_.angle_increment;
  int num_ranges = current_scan_.ranges.size();
  
  constexpr double SEARCH_HALF_ANGLE = M_PI / 3.0;
  constexpr double MIN_VALID_RANGE = 0.05;
  
  double search_min = lookahead_angle - SEARCH_HALF_ANGLE;
  double search_max = lookahead_angle + SEARCH_HALF_ANGLE;
  
  struct RayInfo {
    double angle;
    double range;
    bool is_obstacle;
  };
  std::vector<RayInfo> rays;
  
  for (int i = 0; i < num_ranges; ++i) {
    double angle = angle_min + i * angle_inc;
    
    if (angle < search_min || angle > search_max) {
      continue;
    }
    
    double range = current_scan_.ranges[i];
    bool valid = !std::isnan(range) && !std::isinf(range) && range > MIN_VALID_RANGE;
    bool is_close_obstacle = valid && range < a2_threshold_;
    
    rays.push_back({angle, valid ? range : 10.0, is_close_obstacle});
  }
  
  if (rays.empty()) {
    return lookahead_angle;
  }
  
  double best_gap_center = lookahead_angle;
  double best_gap_score = -1.0;  
  
  ssize_t gap_start_idx = -1;
  
  for (size_t i = 0; i < rays.size(); ++i) {
    if (!rays[i].is_obstacle) {
      if (gap_start_idx < 0) {
        gap_start_idx = static_cast<ssize_t>(i);
      }
    } else {
      if (gap_start_idx >= 0) {
        size_t gap_end_idx = i - 1;
        size_t gap_start = static_cast<size_t>(gap_start_idx);
        double gap_start_angle = rays[gap_start].angle;
        double gap_end_angle = rays[gap_end_idx].angle;
        double gap_width = gap_end_angle - gap_start_angle;
        double gap_center = (gap_start_angle + gap_end_angle) / 2.0;
        
        double avg_range = 0.0;
        size_t count = 0;
        for (size_t j = gap_start; j <= gap_end_idx; ++j) {
          avg_range += rays[j].range;
          count++;
        }
        avg_range = count > 0 ? avg_range / static_cast<double>(count) : 0.0;
        
        double direction_bonus = 1.0 - std::abs(gap_center - lookahead_angle) / SEARCH_HALF_ANGLE;
        double score = gap_width * avg_range * (0.5 + 0.5 * direction_bonus);
        
        if (score > best_gap_score) {
          best_gap_score = score;
          best_gap_center = gap_center;
        }
        
        gap_start_idx = -1;
      }
    }
  }
  
  if (gap_start_idx >= 0) {
    size_t gap_end_idx = rays.size() - 1;
    size_t gap_start = static_cast<size_t>(gap_start_idx);
    double gap_start_angle = rays[gap_start].angle;
    double gap_end_angle = rays[gap_end_idx].angle;
    double gap_width = gap_end_angle - gap_start_angle;
    double gap_center = (gap_start_angle + gap_end_angle) / 2.0;
    
    double avg_range = 0.0;
    size_t count = 0;
    for (size_t j = gap_start; j <= gap_end_idx; ++j) {
      avg_range += rays[j].range;
      count++;
    }
    avg_range = count > 0 ? avg_range / static_cast<double>(count) : 0.0;
    
    double direction_bonus = 1.0 - std::abs(gap_center - lookahead_angle) / SEARCH_HALF_ANGLE;
    double score = gap_width * avg_range * (0.5 + 0.5 * direction_bonus);
    
    if (score > best_gap_score) {
      best_gap_score = score;
      best_gap_center = gap_center;
    }
  }
  
  if (best_gap_score < 0.1) {
    return lookahead_angle;
  }
  
  return best_gap_center;
}

[[deprecated("A2 avoidance disabled per user request - use hard collision stop only")]]
double SimpleController::compute_avoidance_steering(double )
{
  return 0.0;
}

[[deprecated("Wall repulsion disabled per user request - use curvature-based speed only")]]
double SimpleController::compute_wall_repulsion_steering()
{
  return 0.0;
}

double SimpleController::compute_opponent_following_speed(double base_speed)
{
  double front_dist = last_obstacle_front_dist_;
  
  if (front_dist < follow_distance_threshold_ && front_dist > a1_threshold_) {
    bool can_overtake = can_overtake_safely(front_dist, last_obstacle_angle_);
    
    if (!can_overtake || !has_valid_overtake_plan_) {
      if (!is_following_opponent_) {
        is_following_opponent_ = true;
        RCLCPP_INFO(this->get_logger(), 
                    "FOLLOWING START (ACC): No safe overtake path. front=%.2fm, target_gap=%.2fm",
                    front_dist, target_follow_gap_);
      }
      
      double acc_speed = compute_acc_following_speed(base_speed);
      
      publish_safety_zone_markers();
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "FOLLOWING (ACC): gap=%.2fm, target=%.2fm, speed=%.2f [checking overtake...]",
                            front_dist, target_follow_gap_, acc_speed);
      
      return acc_speed;
    } else {
      if (is_following_opponent_) {
        is_following_opponent_ = false;
        RCLCPP_WARN(this->get_logger(), 
                    "OVERTAKE OPPORTUNITY! Valid path found while FOLLOWING. Direction: %s",
                    planned_overtake_direction_ > 0 ? "LEFT" : "RIGHT");
      }
      publish_overtake_path(planned_overtake_direction_);
    }
  } else {
    if (is_following_opponent_) {
      is_following_opponent_ = false;
      RCLCPP_INFO(this->get_logger(), "Clear path, exiting FOLLOWING mode");
    }
  }
  
  return base_speed;
}

double SimpleController::smooth_speed_change(double target_speed, double current_adjusted_speed)
{
  if (is_overtaking_ || is_post_overtake_) {
    return target_speed;
  }
  
  double speed_diff = target_speed - current_adjusted_speed;
  double max_speed_change = speed_smooth_factor_ * max_speed_;
  
  if (std::abs(speed_diff) > max_speed_change) {
    return std::clamp(target_speed, 
                      current_adjusted_speed - max_speed_change, 
                      current_adjusted_speed + max_speed_change);
  }
  
  return target_speed;
}

void SimpleController::check_and_start_overtake()
{
  if (is_overtaking_ || is_post_overtake_) {
    return;  
  }
  
  if (is_reversing_ || is_pausing_after_reverse_ || is_planning_after_pause_) {
    return;
  }
  
  double front_dist = last_obstacle_front_dist_;
  
  if (front_dist < follow_distance_threshold_ && front_dist > a1_threshold_) {
    bool can_overtake = can_overtake_safely(front_dist, last_obstacle_angle_);
    
    if (can_overtake && has_valid_overtake_plan_) {
      is_overtaking_ = true;
      is_following_opponent_ = false;
      overtake_start_time_ros_ = this->get_clock()->now();
      
      double overtake_direction = planned_overtake_direction_;
      
      RCLCPP_WARN(this->get_logger(), 
                  "OVERTAKE START! VALIDATED path (vehicle_width=%.2fm). front=%.2fm, direction=%s, L=%.2fm, R=%.2fm",
                  vehicle_width_, front_dist, 
                  overtake_direction > 0 ? "LEFT" : "RIGHT",
                  last_obstacle_left_dist_, last_obstacle_right_dist_);
      
      publish_overtake_path(overtake_direction);
    }
  }
}

void SimpleController::update_overtake_state()
{
  rclcpp::Time current_time = this->get_clock()->now();
  
  if (is_overtaking_) {
    if (is_in_a1_zone_) {
      is_overtaking_ = false;
      is_post_overtake_ = false;
      has_valid_overtake_plan_ = false;
      planned_overtake_direction_ = 0.0;  
      RCLCPP_WARN(this->get_logger(), "OVERTAKE ABORTED: Collision detected! Initiating reverse...");
      return;
    }
    
    if (should_return_to_line()) {
      is_overtaking_ = false;
      is_post_overtake_ = true;
      overtake_end_time_ros_ = current_time;
      RCLCPP_INFO(this->get_logger(), "OVERTAKE COMPLETE, entering high-speed phase");
    }
  }
  
  if (is_post_overtake_) {
    double elapsed = (current_time - overtake_end_time_ros_).seconds();
    if (elapsed > post_overtake_duration_) {
      is_post_overtake_ = false;
      RCLCPP_INFO(this->get_logger(), "Post-overtake phase ended, returning to normal speed");
    }
  }
}

double SimpleController::compute_overtake_speed(double base_speed)
{
  if (is_overtaking_) {
    double overtake_speed = base_speed * overtake_speed_boost_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "OVERTAKING: speed boost %.2f -> %.2f (%.1fx)",
                          base_speed, overtake_speed, overtake_speed_boost_);
    return overtake_speed;
  }
  
  if (is_post_overtake_) {
    double post_speed = base_speed * post_overtake_speed_factor_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "POST-OVERTAKE: maintaining speed %.2f -> %.2f",
                          base_speed, post_speed);
    return post_speed;
  }
  
  return base_speed;
}

int SimpleController::find_target_point_index(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Path& path, double adaptive_lookahead)
{
    if (path.poses.empty()) return -1;

    static int last_target_idx = 0;
    
    double current_x = odom.pose.pose.position.x;
    double current_y = odom.pose.pose.position.y;
    double current_yaw = tf2::getYaw(odom.pose.pose.orientation);

    const double search_radius = 3.0;
    const double search_radius_sq = search_radius * search_radius;
    
    int search_start = std::max(0, last_target_idx - 5);
    int search_end = std::min(static_cast<int>(path.poses.size()), last_target_idx + 30);
    
    double min_dist_sq = std::numeric_limits<double>::max();
    int closest_idx = last_target_idx;
    
    for (int i = search_start; i < search_end; ++i) {
        double dx = path.poses[i].pose.position.x - current_x;
        double dy = path.poses[i].pose.position.y - current_y;
        double dist_sq = dx * dx + dy * dy;
        
        if (dist_sq > search_radius_sq) continue;
        
        double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
        if (dot_product <= 0.0) continue;
        
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }
    
    if (min_dist_sq == std::numeric_limits<double>::max()) {
        return last_target_idx;
    }

    int target_idx = closest_idx;
    double accum_dist = 0.0;
    double min_lookahead_dist = adaptive_lookahead * 0.8;
    double max_lookahead_dist = adaptive_lookahead * 1.2;
    
    for (int i = closest_idx; i < static_cast<int>(path.poses.size() - 1); ++i) {
        double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
        double seg_dist = std::hypot(dx, dy);
        accum_dist += seg_dist;
        
        if (accum_dist >= min_lookahead_dist) {
            target_idx = i + 1;
            if (accum_dist > max_lookahead_dist) {
                break;
            }
        }
    }
    
    double dx = path.poses[target_idx].pose.position.x - current_x;
    double dy = path.poses[target_idx].pose.position.y - current_y;
    double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
    
    if (dot_product <= 0.0 && target_idx < static_cast<int>(path.poses.size() - 1)) {
        for (int i = target_idx + 1; i < static_cast<int>(path.poses.size()); ++i) {
            dx = path.poses[i].pose.position.x - current_x;
            dy = path.poses[i].pose.position.y - current_y;
            dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
            if (dot_product > 0.0) {
                target_idx = i;
                break;
            }
        }
    }
    
    target_idx = std::max(0, std::min(target_idx, static_cast<int>(path.poses.size() - 1)));
    
    if (std::abs(target_idx - last_target_idx) > 50) {
        target_idx = last_target_idx;
    } else {
        last_target_idx = target_idx;
    }
    
    return target_idx;
}

void SimpleController::control_loop()
{
  static bool warned_odom = false;
  static bool warned_path = false;
  static bool warned_empty = false;
  
  if (!odom_received_) {
    if (!warned_odom) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                          "Waiting for odometry on topic: %s", odom_topic_.c_str());
      warned_odom = true;
    }
    return;
  }
  if (!path_received_) {
    if (!warned_path) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                          "Waiting for path on topic: %s", path_topic_.c_str());
      warned_path = true;
    }
    return;
  }
  if (current_path_.poses.empty()) {
    if (!warned_empty) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                          "Path received but empty");
      warned_empty = true;
    }
    return;
  }

  rclcpp::Time current_time = this->get_clock()->now();
  
  if (is_obstacle_stop_mode()) {
    execute_obstacle_stop();
    return;  
  }
  
  update_obstacle_distances();
  update_virtual_boxes();  // Update LiDAR virtual boxes for FSM
  
  if (is_planning_after_pause_) {
    double planning_elapsed = (current_time - planning_start_time_).seconds();
    if (planning_elapsed < planning_duration_) {
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = current_time;
      drive_msg.header.frame_id = current_odom_.child_frame_id;
      drive_msg.drive.speed = 0.0;
      drive_msg.drive.steering_angle = 0.0;  
      
      drive_pub_->publish(drive_msg);
      
      static int planning_log_count = 0;
      if (planning_log_count++ % 25 == 0) {
        RCLCPP_INFO(this->get_logger(), "PLANNING: %.2fs/%.2fs (computing next action...)",
                    planning_elapsed, planning_duration_);
      }
      return;
    } else {
      is_planning_after_pause_ = false;
      just_finished_reverse_ = true;
      reverse_cooldown_counter_ = REVERSE_COOLDOWN_CYCLES;
      RCLCPP_INFO(this->get_logger(), "Planning complete, starting cooldown before allowing another reverse");
    }
  }
  
  if (is_pausing_after_reverse_) {
    double pause_elapsed = (current_time - pause_start_time_).seconds();
    if (pause_elapsed < reverse_pause_duration_) {
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = current_time;
      drive_msg.header.frame_id = current_odom_.child_frame_id;
      drive_msg.drive.speed = 0.0;
      drive_msg.drive.steering_angle = 0.0;  
      
      drive_pub_->publish(drive_msg);
      
      static int pause_log_count = 0;
      if (pause_log_count++ % 25 == 0) {
        RCLCPP_INFO(this->get_logger(), "PAUSING: %.2fs/%.2fs (stabilizing...)",
                    pause_elapsed, reverse_pause_duration_);
      }
      return;
    } else {
      is_pausing_after_reverse_ = false;
      is_planning_after_pause_ = true;
      planning_start_time_ = current_time;
      prev_steering_angle_ = 0.0;
      lateral_error_integral_ = 0.0;  
      RCLCPP_INFO(this->get_logger(), "Pause complete, entering planning phase...");
      return;
    }
  }
  
  if (reverse_cooldown_counter_ > 0) {
    reverse_cooldown_counter_--;
    if (reverse_cooldown_counter_ == 0) {
      just_finished_reverse_ = false;
      RCLCPP_DEBUG(this->get_logger(), "Reverse cooldown complete, reverse allowed again");
    }
  }
  
  bool imu_crash_detected = check_imu_collision();
  bool stall_detected = check_stall_condition();
  
  bool collision_trigger = check_a1_zone() || imu_crash_detected || stall_detected;
  
  if (collision_trigger && !just_finished_reverse_) {
    if (!is_reversing_ && !is_pausing_after_reverse_ && !is_planning_after_pause_) {
      is_reversing_ = true;
      crsm_state_ = CRSMState::ST_CRASH_DETECTED;
      reverse_start_time_ = current_time;
      last_steering_before_reverse_ = prev_steering_angle_;  
      
      const char* trigger_type = imu_crash_detected ? "IMU CRASH" : 
                                 (stall_detected ? "STALL" : "A1 ZONE");
      RCLCPP_WARN(this->get_logger(), 
                  "CRSM [%s] COLLISION! mode=%d, saved_steer=%.3f", 
                  trigger_type, reverse_steering_mode_, last_steering_before_reverse_);
    }
    
    double elapsed = (current_time - reverse_start_time_).seconds();
    if (elapsed < reverse_duration_) {
      crsm_state_ = CRSMState::ST_RECOVERY_REVERSE;
      
      double reverse_steer = compute_inverted_reverse_steering();
      
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = current_time;
      drive_msg.header.frame_id = current_odom_.child_frame_id;
      drive_msg.drive.speed = -reverse_speed_;
      drive_msg.drive.steering_angle = reverse_steer;
      
      drive_pub_->publish(drive_msg);
      last_cmd_velocity_ = -reverse_speed_;  
      
      publish_crsm_state();
      
      static int reverse_log_count = 0;
      if (reverse_log_count++ % 25 == 0) {
        const char* mode_str = (reverse_steering_mode_ == 0) ? "NEUTRAL" :
                              ((reverse_steering_mode_ == 1) ? "INVERTED" : "MAINTAIN");
        RCLCPP_INFO(this->get_logger(), 
                    "CRSM REVERSING [%s]: speed=%.2f, steer=%.3f (orig=%.3f), elapsed=%.2fs/%.2fs",
                    mode_str, -reverse_speed_, reverse_steer, last_steering_before_reverse_, 
                    elapsed, reverse_duration_);
      }
      return;
    } else {
      is_reversing_ = false;
      is_pausing_after_reverse_ = true;
      crsm_state_ = CRSMState::ST_RECOVERY_REALIGN;
      pause_start_time_ = current_time;
      RCLCPP_INFO(this->get_logger(), "CRSM Reverse complete, entering REALIGN phase...");
      publish_crsm_state();
      return;
    }
  } else if (check_a1_zone() && just_finished_reverse_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "A1 Zone during cooldown! Slow forward escape (cooldown: %d)",
                          reverse_cooldown_counter_);
  } else {
    if (is_reversing_) {
      is_reversing_ = false;
      is_pausing_after_reverse_ = true;
      pause_start_time_ = current_time;
      RCLCPP_INFO(this->get_logger(), "Left A1 zone, stopping and stabilizing...");
      return;
    }
  }

  double current_speed = std::sqrt(
    current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
    current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
  );

  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  static int last_closest_idx = 0;
  int closest_idx = find_closest_point_along_path(
      current_x, current_y, current_yaw, current_path_, last_closest_idx);
  last_closest_idx = closest_idx;
  
  double lateral_error = compute_lateral_error(
      current_x, current_y, current_yaw, current_path_, closest_idx);
  double heading_error = compute_heading_error(current_yaw, current_path_, closest_idx);
  
  double adaptive_lookahead = compute_adaptive_lookahead(current_speed);

  int target_idx = find_target_point_index(current_odom_, current_path_, adaptive_lookahead);
  if (target_idx < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Failed to find target point. Current pos: (%.2f, %.2f), Path size: %zu",
                        current_odom_.pose.pose.position.x, 
                        current_odom_.pose.pose.position.y,
                        current_path_.poses.size());
    return;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  if (use_path_interpolation_ && target_idx < static_cast<int>(current_path_.poses.size() - 1)) {
    double dist_to_target = std::hypot(
      current_path_.poses[target_idx].pose.position.x - current_x,
      current_path_.poses[target_idx].pose.position.y - current_y
    );
    double dist_to_next = std::hypot(
      current_path_.poses[target_idx + 1].pose.position.x - current_path_.poses[target_idx].pose.position.x,
      current_path_.poses[target_idx + 1].pose.position.y - current_path_.poses[target_idx].pose.position.y
    );
    
    if (dist_to_next > 1e-6 && dist_to_target < adaptive_lookahead) {
      double fraction = (adaptive_lookahead - dist_to_target) / dist_to_next;
      fraction = std::max(0.0, std::min(1.0, fraction));
      target_pose = interpolate_path_point(current_path_, target_idx, fraction);
    } else {
      target_pose = current_path_.poses[target_idx];
    }
  } else {
    target_pose = current_path_.poses[target_idx];
  }
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "Target idx: %d, Current: (%.2f, %.2f), Target: (%.2f, %.2f), Lookahead: %.2f",
                        target_idx,
                        current_odom_.pose.pose.position.x,
                        current_odom_.pose.pose.position.y,
                        target_pose.pose.position.x,
                        target_pose.pose.position.y,
                        adaptive_lookahead);

  double target_x_global = target_pose.pose.position.x;
  double target_y_global = target_pose.pose.position.y;
  
  publish_lookahead_marker(target_x_global, target_y_global, 0.15);

  double dx_global = target_x_global - current_x;
  double dy_global = target_y_global - current_y;
  double cos_neg_yaw = std::cos(-current_yaw);
  double sin_neg_yaw = std::sin(-current_yaw);
  
  double target_x_vehicle = dx_global * cos_neg_yaw - dy_global * sin_neg_yaw;
  double target_y_vehicle = dx_global * sin_neg_yaw + dy_global * cos_neg_yaw;
  
  static int coord_debug_count = 0;
  if (coord_debug_count++ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(), 
                "Coord: global_dx=%.3f, global_dy=%.3f, veh_x=%.3f, veh_y=%.3f, yaw=%.3f",
                dx_global, dy_global, target_x_vehicle, target_y_vehicle, current_yaw);
  }

  double dt = (current_time - prev_time_).seconds();
  if (dt > 0.1) dt = 0.1;
  if (dt <= 0.0) dt = 0.02;
  
  double alpha = std::atan2(target_y_vehicle, target_x_vehicle);
  double delta_pp = 0.0;
  if (adaptive_lookahead > 1e-6) {
    double sin_alpha_clipped = std::max(-1.0, std::min(1.0, std::sin(alpha)));
    delta_pp = std::atan2(2.0 * wheelbase_ * sin_alpha_clipped, adaptive_lookahead);
  }

  double delta_pid = compute_pid_control(lateral_error, dt);

  double path_curvature = compute_path_curvature(current_path_, target_idx);
  double delta_ff = curvature_feedforward_gain_ * std::atan(wheelbase_ * path_curvature);

  double stanley_den = current_speed + 1.0;  
  double delta_stanley = std::atan(heading_error_gain_ * heading_error / stanley_den);

  double steering_angle = delta_pp + delta_pid + delta_stanley + delta_ff;
  
  bool is_corner = std::abs(path_curvature) > corner_curvature_threshold_;
  
  double corner_direction = 0.0;
  double out_in_out_steer_offset = 0.0;
  if (enable_out_in_out_ && detect_upcoming_corner(closest_idx, corner_direction)) {
    double offset = compute_out_in_out_offset(closest_idx, corner_direction);
    out_in_out_steer_offset = -offset * 0.5;  
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "OUT-IN-OUT: corner_dir=%.1f, offset=%.3f, steer_offset=%.3f",
                          corner_direction, offset, out_in_out_steer_offset);
  }
  steering_angle += out_in_out_steer_offset;
  
  if (is_corner) {
    steering_angle *= corner_steer_amplify_;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "CORNER detected: curvature=%.3f, steer amplified by %.2fx",
                          path_curvature, corner_steer_amplify_);
  }
  
  
  bool is_wall_collision = is_in_a1_zone_ || 
                           last_obstacle_front_dist_ < 0.15 ||
                           last_obstacle_left_dist_ < 0.1 ||
                           last_obstacle_right_dist_ < 0.1;
  publish_wall_collision_indicator(is_wall_collision, current_x, current_y);

  steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);

  steering_angle *= steering_sign_;
  
  steering_angle = limit_steering_rate(steering_angle, dt);
  
  steering_angle = smooth_steering(steering_angle, prev_steering_angle_);
  
  prev_steering_angle_ = steering_angle;
  prev_time_ = current_time;

  static int debug_count = 0;
  if (debug_count++ % 25 == 0) {  
    RCLCPP_INFO(this->get_logger(), 
                "Hybrid: lat_err=%.3f, hdg_err=%.3f, PP=%.3f, PID=%.3f, Stanley=%.3f, FF=%.3f, steer=%.3f",
                lateral_error, heading_error, delta_pp, delta_pid, delta_stanley, delta_ff, steering_angle);
  }

  check_and_start_overtake();
  update_overtake_state();
  
  if (is_overtaking_) {
    double overtake_direction;
    if (has_valid_overtake_plan_ && std::abs(planned_overtake_direction_) > 0.5) {
      overtake_direction = planned_overtake_direction_;
    } else {
      double left_space = last_obstacle_left_dist_;
      double right_space = last_obstacle_right_dist_;
      overtake_direction = (left_space > right_space) ? 1.0 : -1.0;
    }
    publish_overtake_path(overtake_direction);
    
    steering_angle = compute_overtake_steering(steering_angle);
  }

  // ===================================================================
  // Speed determination: Use v_ref from raceline (curvature-based)
  // ===================================================================
  // Strategy:
  // 1. If v_ref is available from raceline, use it directly (already accounts for curvature)
  // 2. Otherwise fall back to target_speed with corner speed factor
  // 
  // This ensures:
  // - Straights: v_ref → high (near max_speed_)
  // - Corners: v_ref → low (proportional to safe cornering speed)
  // - No additional speed penalties from steering angle or side sensors
  double base_adjusted_speed = target_speed_;  // Fallback default
  
  if (vref_received_ && !current_vref_.empty() && target_idx >= 0 && 
      static_cast<size_t>(target_idx) < current_vref_.size()) {
    // Use v_ref from raceline (curvature-based speed)
    base_adjusted_speed = current_vref_[target_idx];
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "Using v_ref from raceline: idx=%d, v_ref=%.2f m/s",
                          target_idx, base_adjusted_speed);
  } else {
    // Fallback: use old corner speed factor method
    double speed_factor = 1.0;
    
    if (is_corner && !is_overtaking_) {
      // Only reduce speed in actual corners (curvature > threshold)
      double curvature_speed_reduction = std::max(min_corner_speed_factor_, 1.0 / (1.0 + std::abs(path_curvature)));
      speed_factor = std::min(corner_speed_factor_, curvature_speed_reduction);
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                            "FALLBACK corner speed reduction: curv=%.3f, factor=%.2f",
                            path_curvature, speed_factor);
    }
    
    base_adjusted_speed = target_speed_ * speed_factor;
  }
  
  double adjusted_speed = base_adjusted_speed;
  
  if (is_overtaking_ || is_post_overtake_) {
    adjusted_speed = compute_overtake_speed(base_adjusted_speed);
  } else {
    adjusted_speed = compute_opponent_following_speed(base_adjusted_speed);
  }
  
  adjusted_speed = smooth_speed_change(adjusted_speed, last_adjusted_speed_);
  last_adjusted_speed_ = adjusted_speed;
  
  adjusted_speed = std::clamp(adjusted_speed, 0.0, max_speed_);
  
  if (just_finished_reverse_ && is_in_a1_zone_) {
    adjusted_speed = std::min(adjusted_speed, 0.3);  
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "Cooldown escape mode: limiting speed to %.2f", adjusted_speed);
  }

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = current_time;
  drive_msg.header.frame_id = current_odom_.child_frame_id;
  drive_msg.drive.speed = adjusted_speed;
  drive_msg.drive.steering_angle = steering_angle;

  drive_pub_->publish(drive_msg);
  
  last_cmd_velocity_ = adjusted_speed;
  
  if (!is_reversing_ && !is_pausing_after_reverse_ && !is_planning_after_pause_) {
    crsm_state_ = CRSMState::ST_NORMAL;
  }
  
  static int publish_count = 0;
  if (publish_count++ % 50 == 0) {  
    RCLCPP_INFO(this->get_logger(), "Drive: speed=%.2f, steer=%.3f, idx=%d, LA=%.2f%s%s%s%s",
                drive_msg.drive.speed, drive_msg.drive.steering_angle, target_idx, adaptive_lookahead,
                is_overtaking_ ? " [OVERTAKING]" : "",
                is_post_overtake_ ? " [POST-OT]" : "",
                is_following_opponent_ ? " [FOLLOWING]" : "",
                just_finished_reverse_ ? " [COOLDOWN]" : "");
  }
}

double SimpleController::compute_adaptive_lookahead(double speed)
{
  double base_lookahead = std::max(min_lookahead_, lookahead_distance_);
  double adaptive = base_lookahead + lookahead_speed_gain_ * speed;
  double result = std::max(min_lookahead_, std::min(max_lookahead_, adaptive));
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "Lookahead: base=%.2f, speed=%.2f, adaptive=%.2f (min=%.2f, max=%.2f, gain=%.2f)",
              base_lookahead, speed, result, min_lookahead_, max_lookahead_, lookahead_speed_gain_);
  
  return result;
}

double SimpleController::compute_lateral_error(double current_x, double current_y, double current_yaw, 
                                               const nav_msgs::msg::Path& path, int closest_idx)
{
  if (closest_idx < 0 || closest_idx >= static_cast<int>(path.poses.size())) {
    return 0.0;
  }

  double path_x = path.poses[closest_idx].pose.position.x;
  double path_y = path.poses[closest_idx].pose.position.y;
  
  double dx = path_x - current_x;
  double dy = path_y - current_y;
  
  double cos_yaw = std::cos(current_yaw);
  double sin_yaw = std::sin(current_yaw);
  double dy_vehicle = -dx * sin_yaw + dy * cos_yaw;
  
  double lateral_error = dy_vehicle;
  
  return lateral_error;
}

double SimpleController::limit_steering_rate(double desired_steering, double dt)
{
  if (dt <= 0.0) return prev_steering_angle_;
  
  double max_change = max_steer_rate_ * dt;
  double steering_diff = desired_steering - prev_steering_angle_;
  
  if (std::abs(steering_diff) > max_change) {
    double sign = (steering_diff > 0) ? 1.0 : -1.0;
    return prev_steering_angle_ + sign * max_change;
  }
  
  return desired_steering;
}

double SimpleController::compute_heading_error(double current_yaw, const nav_msgs::msg::Path& path, int closest_idx)
{
  if (closest_idx < 0 || closest_idx >= static_cast<int>(path.poses.size())) {
    return 0.0;
  }
  
  double path_yaw = tf2::getYaw(path.poses[closest_idx].pose.orientation);
  double error = path_yaw - current_yaw;
  
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;
  
  return error;
}

geometry_msgs::msg::PoseStamped SimpleController::interpolate_path_point(const nav_msgs::msg::Path& path, int idx, double fraction)
{
  geometry_msgs::msg::PoseStamped result;
  result.header = path.header;
  
  if (idx < 0 || idx >= static_cast<int>(path.poses.size() - 1)) {
    if (idx >= 0 && idx < static_cast<int>(path.poses.size())) {
      return path.poses[idx];
    }
    return result;
  }
  
  const auto& p1 = path.poses[idx].pose.position;
  const auto& p2 = path.poses[idx + 1].pose.position;
  
  result.pose.position.x = p1.x + fraction * (p2.x - p1.x);
  result.pose.position.y = p1.y + fraction * (p2.y - p1.y);
  result.pose.position.z = p1.z + fraction * (p2.z - p1.z);
  
  double yaw1 = tf2::getYaw(path.poses[idx].pose.orientation);
  double yaw2 = tf2::getYaw(path.poses[idx + 1].pose.orientation);
  
  double yaw_diff = yaw2 - yaw1;
  while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
  
  double yaw_interp = yaw1 + fraction * yaw_diff;
  result.pose.orientation.z = std::sin(yaw_interp / 2.0);
  result.pose.orientation.w = std::cos(yaw_interp / 2.0);
  
  return result;
}

double SimpleController::compute_pid_control(double error, double dt)
{
  if (dt <= 0.0) return 0.0;
  
  double p_term = pid_kp_ * error;
  
  lateral_error_integral_ += error * dt;
  double max_integral;
  if (pid_integral_limit_ > 0.0) {
    max_integral = pid_integral_limit_;
  } else {
    max_integral = 1.0 / std::max(pid_ki_, 1e-6);
  }
  lateral_error_integral_ = std::max(-max_integral, std::min(max_integral, lateral_error_integral_));
  double i_term = pid_ki_ * lateral_error_integral_;
  
  double d_term = pid_kd_ * (error - prev_lateral_error_) / dt;
  prev_lateral_error_ = error;
  
  return p_term + i_term + d_term;
}

double SimpleController::compute_path_curvature(const nav_msgs::msg::Path& path, int idx)
{
  if (idx < 1 || idx >= static_cast<int>(path.poses.size() - 1)) {
    return 0.0;
  }
  
  const auto& p0 = path.poses[idx - 1].pose.position;
  const auto& p1 = path.poses[idx].pose.position;
  const auto& p2 = path.poses[idx + 1].pose.position;
  
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
  if (avg_ds < 1e-6) {
    return 0.0;
  }
  
  return cross / (ds1 * ds2 * avg_ds);
}

int SimpleController::find_closest_point_along_path(double current_x, double current_y, double current_yaw, 
                                                     const nav_msgs::msg::Path& path, int start_idx)
{
  if (path.poses.empty()) return 0;
  
  int path_size = static_cast<int>(path.poses.size());
  start_idx = std::clamp(start_idx, 0, path_size - 1);
  
  int search_range_forward = 20;
  int search_range_backward = 5;
  int min_idx = std::max(0, start_idx - search_range_backward);
  int max_idx = std::min(path_size - 1, start_idx + search_range_forward);
  
  double min_dist_sq = std::numeric_limits<double>::max();
  int closest_idx = start_idx;
  
  for (int i = min_idx; i <= max_idx; ++i) {
    double dx = current_x - path.poses[i].pose.position.x;
    double dy = current_y - path.poses[i].pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    
    double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
    
    double weight = dist_sq;
    if (dot_product > 0) {
      weight *= 0.3;  
    } else if (dot_product < -0.1) {
      weight *= 10.0;  
    }
    
    if (weight < min_dist_sq) {
      min_dist_sq = weight;
      closest_idx = i;
    }
  }
  
  return closest_idx;
}

double SimpleController::smooth_steering(double new_steering, double prev_steering)
{
  return smoothing_factor_ * new_steering + (1.0 - smoothing_factor_) * prev_steering;
}

void SimpleController::publish_lookahead_marker(double x, double y, double z)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "pid_lookahead";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);  
  
  lookahead_marker_pub_->publish(marker);
}

bool SimpleController::load_wall_data(const std::string& csv_file)
{
  std::ifstream file(csv_file);
  if (!file.is_open()) {
    RCLCPP_WARN(this->get_logger(), "Failed to open wall data file: %s", csv_file.c_str());
    return false;
  }
  
  wall_data_.clear();
  std::string line;
  int line_num = 0;
  int parse_errors = 0;
  
  std::getline(file, line);
  line_num++;
  
  while (std::getline(file, line)) {
    line_num++;
    if (line.empty()) continue;
    
    std::stringstream ss(line);
    WallPoint point;
    char comma;
    
    if (ss >> point.x >> comma >> point.y >> comma >> point.d_left >> comma >> point.d_right >> comma >> point.psi) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.d_left) && std::isfinite(point.d_right) &&
          std::isfinite(point.psi) &&
          point.d_left >= 0.0 && point.d_right >= 0.0) {
        wall_data_.push_back(point);
      } else {
        parse_errors++;
        if (parse_errors <= 3) {
          RCLCPP_WARN(this->get_logger(), "Invalid wall data at line %d: values out of range", line_num);
        }
      }
    } else {
      parse_errors++;
      if (parse_errors <= 3) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse wall data at line %d", line_num);
      }
    }
  }
  
  if (parse_errors > 3) {
    RCLCPP_WARN(this->get_logger(), "... and %d more parsing errors", parse_errors - 3);
  }
  
  wall_data_loaded_ = !wall_data_.empty();
  if (wall_data_loaded_) {
    RCLCPP_INFO(this->get_logger(), "Wall data loaded: %zu valid points from %s", 
                wall_data_.size(), csv_file.c_str());
  }
  return wall_data_loaded_;
}

size_t SimpleController::find_closest_wall_point(double x, double y)
{
  if (wall_data_.empty()) {
    return 0;
  }
  
  size_t search_range = 50;  
  size_t start_idx = (last_wall_search_idx_ > search_range) ? 
                      last_wall_search_idx_ - search_range : 0;
  size_t end_idx = std::min(last_wall_search_idx_ + search_range, wall_data_.size());
  
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = last_wall_search_idx_;
  
  for (size_t i = start_idx; i < end_idx; ++i) {
    double dx = wall_data_[i].x - x;
    double dy = wall_data_[i].y - y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }
  
  if (min_dist_sq > 4.0) {  
    for (size_t i = 0; i < wall_data_.size(); ++i) {
      double dx = wall_data_[i].x - x;
      double dy = wall_data_[i].y - y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_idx = i;
      }
    }
  }
  
  last_wall_search_idx_ = closest_idx;
  return closest_idx;
}

double SimpleController::get_wall_distance_left(double x, double y)
{
  if (!wall_data_loaded_ || wall_data_.empty()) {
    return 10.0;  
  }
  
  size_t closest_idx = find_closest_wall_point(x, y);
  return wall_data_[closest_idx].d_left;
}

double SimpleController::get_wall_distance_right(double x, double y)
{
  if (!wall_data_loaded_ || wall_data_.empty()) {
    return 10.0;
  }
  
  size_t closest_idx = find_closest_wall_point(x, y);
  return wall_data_[closest_idx].d_right;
}

bool SimpleController::detect_upcoming_corner(int current_idx, double& corner_direction)
{
  if (current_path_.poses.empty() || current_idx < 0) {
    return false;
  }
  
  if (current_idx >= static_cast<int>(current_path_.poses.size())) {
    return false;
  }
  
  int look_ahead_points = static_cast<int>(corner_approach_distance_ / 0.1);  
  int remaining_points = static_cast<int>(current_path_.poses.size()) - current_idx - 2;
  if (remaining_points < 3) {
    return false;
  }
  look_ahead_points = std::min(look_ahead_points, remaining_points);
  
  double max_curvature = 0.0;
  double curvature_sign = 0.0;
  
  for (int i = current_idx; i < current_idx + look_ahead_points - 1; ++i) {
    double curvature = compute_path_curvature(current_path_, i);
    if (std::abs(curvature) > std::abs(max_curvature)) {
      max_curvature = curvature;
      curvature_sign = (curvature > 0) ? 1.0 : -1.0;
    }
  }
  
  if (std::abs(max_curvature) > corner_curvature_threshold_) {
    corner_direction = curvature_sign;  
    return true;
  }
  
  return false;
}

double SimpleController::compute_out_in_out_offset([[maybe_unused]] int current_idx, double corner_direction)
{
  if (!enable_out_in_out_) {
    return 0.0;
  }
  
  double offset = -corner_direction * out_in_out_offset_;
  
  if (wall_data_loaded_) {
    double current_x = current_odom_.pose.pose.position.x;
    double current_y = current_odom_.pose.pose.position.y;
    
    double wall_left = get_wall_distance_left(current_x, current_y);
    double wall_right = get_wall_distance_right(current_x, current_y);
    
    double safety_margin = 0.3;
    if (offset > 0 && offset > wall_left - safety_margin) {
      offset = std::max(0.0, wall_left - safety_margin);
    } else if (offset < 0 && -offset > wall_right - safety_margin) {
      offset = -std::max(0.0, wall_right - safety_margin);
    }
  }
  
  return offset;
}

bool SimpleController::can_overtake_safely(double opponent_dist, double opponent_angle)
{
  has_valid_overtake_plan_ = false;
  planned_overtake_direction_ = 0.0;
  
  if (!enable_collision_avoidance_) {
    return false;
  }
  
  if (current_path_.poses.empty()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "NO OVERTAKE: No path available");
    return false;
  }
  
  bool opponent_in_front = std::abs(opponent_angle) < M_PI / 6.0;  
  bool opponent_close = opponent_dist < follow_distance_threshold_ && opponent_dist > MIN_OVERTAKE_DISTANCE;
  
  if (!opponent_in_front || !opponent_close) {
    return false;
  }
  
  double left_space = last_obstacle_left_dist_;
  double right_space = last_obstacle_right_dist_;
  double total_space = left_space + right_space;
  
  double required_clearance = min_overtake_clearance_;  
  
  if (total_space < narrow_road_threshold_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Narrow road (L:%.2f + R:%.2f = %.2f < %.2f)",
                          left_space, right_space, total_space, narrow_road_threshold_);
    return false;
  }
  
  if (std::abs(opponent_angle) > min_visibility_angle_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Poor visibility (angle: %.2f > %.2f)",
                          std::abs(opponent_angle), min_visibility_angle_);
    return false;
  }
  
  double corner_direction = 0.0;
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  int closest_idx = find_closest_point_along_path(current_x, current_y, current_yaw, 
                                                   current_path_, last_closest_idx_overtake_);
  last_closest_idx_overtake_ = closest_idx;
  
  if (detect_upcoming_corner(closest_idx, corner_direction)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Corner detected ahead (dir: %.1f)", corner_direction);
    return false;
  }
  
  bool can_overtake_left = left_space > (required_clearance + SAFETY_MARGIN);
  bool can_overtake_right = right_space > (required_clearance + SAFETY_MARGIN);
  
  if (wall_data_loaded_) {
    double wall_left = get_wall_distance_left(current_x, current_y);
    double wall_right = get_wall_distance_right(current_x, current_y);
    
    double required_wall_clearance = overtake_lateral_offset_ + (vehicle_width_ / 2.0) + SAFETY_MARGIN;
    
    can_overtake_left = can_overtake_left && (wall_left > required_wall_clearance);
    can_overtake_right = can_overtake_right && (wall_right > required_wall_clearance);
  }
  
  if (!can_overtake_left && !can_overtake_right) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Insufficient gap (L:%.2f, R:%.2f) for vehicle width %.2fm",
                          left_space, right_space, vehicle_width_);
    return false;
  }
  
  
  double preferred_direction = (left_space > right_space) ? 1.0 : -1.0;
  double alternate_direction = -preferred_direction;
  
  bool preferred_valid = false;
  bool alternate_valid = false;
  
  if ((preferred_direction > 0 && can_overtake_left) || 
      (preferred_direction < 0 && can_overtake_right)) {
    preferred_valid = validate_overtake_path(preferred_direction);
    if (preferred_valid) {
      RCLCPP_INFO(this->get_logger(), 
                  "OVERTAKE PATH CREATED: %s direction validated (vehicle_width=%.2fm)",
                  preferred_direction > 0 ? "LEFT" : "RIGHT", vehicle_width_);
      planned_overtake_direction_ = preferred_direction;
      has_valid_overtake_plan_ = true;
      return true;
    }
  }
  
  if ((alternate_direction > 0 && can_overtake_left) || 
      (alternate_direction < 0 && can_overtake_right)) {
    alternate_valid = validate_overtake_path(alternate_direction);
    if (alternate_valid) {
      RCLCPP_INFO(this->get_logger(), 
                  "OVERTAKE PLAN: Alternate %s path validated successfully",
                  alternate_direction > 0 ? "LEFT" : "RIGHT");
      planned_overtake_direction_ = alternate_direction;
      has_valid_overtake_plan_ = true;
      return true;
    }
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "FOLLOWING ONLY: No safe overtake path found (L_valid:%d, R_valid:%d)",
                        (preferred_direction > 0) ? preferred_valid : alternate_valid,
                        (preferred_direction < 0) ? preferred_valid : alternate_valid);
  
  return false;
}

double SimpleController::compute_overtake_steering(double base_steering)
{
  if (!is_overtaking_) {
    return base_steering;
  }
  
  double overtake_direction;
  if (has_valid_overtake_plan_ && std::abs(planned_overtake_direction_) > 0.5) {
    overtake_direction = planned_overtake_direction_;
  } else {
    double left_space = last_obstacle_left_dist_;
    double right_space = last_obstacle_right_dist_;
    overtake_direction = (left_space > right_space) ? 1.0 : -1.0;
  }
  
  rclcpp::Time current_time = this->get_clock()->now();
  double elapsed = (current_time - overtake_start_time_ros_).seconds();
  double overtake_progress = std::min(1.0, elapsed / overtake_max_duration_);
  
  double steer_intensity = overtake_steer_max_ * (1.0 - overtake_progress * overtake_steer_decay_);
  
  double wall_dist = (overtake_direction > 0) ? last_obstacle_left_dist_ : last_obstacle_right_dist_;
  if (wall_dist < overtake_wall_caution_dist_) {
    steer_intensity *= (wall_dist / overtake_wall_caution_dist_);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "OVERTAKE CAUTION: wall nearby %.2fm, reducing steer intensity",
                          wall_dist);
  }
  
  double overtake_steer = base_steering + overtake_direction * steer_intensity;
  
  double safe_max_steer = max_steer_angle_ * 0.8;
  overtake_steer = std::clamp(overtake_steer, -safe_max_steer, safe_max_steer);
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                        "OVERTAKE steer: base=%.3f, dir=%.0f, intensity=%.3f, final=%.3f",
                        base_steering, overtake_direction, steer_intensity, overtake_steer);
  
  return overtake_steer;
}

bool SimpleController::should_return_to_line()
{
  if (!is_overtaking_) {
    return false;
  }
  
  rclcpp::Time current_time = this->get_clock()->now();
  double elapsed = (current_time - overtake_start_time_ros_).seconds();
  
  if (elapsed > overtake_max_duration_) {
    RCLCPP_INFO(this->get_logger(), "Overtake timeout, returning to line");
    return true;
  }
  
  if (last_obstacle_front_dist_ > return_to_line_distance_) {
    RCLCPP_INFO(this->get_logger(), "Overtake complete, returning to line");
    return true;
  }
  
  return false;
}

double SimpleController::compute_return_to_line_steering()
{
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  static int last_closest_idx = 0;
  int closest_idx = find_closest_point_along_path(current_x, current_y, current_yaw, 
                                                   current_path_, last_closest_idx);
  last_closest_idx = closest_idx;
  
  double lateral_error = compute_lateral_error(current_x, current_y, current_yaw, 
                                                current_path_, closest_idx);
  
  double return_steer = -lateral_error * 0.5;  
  
  return std::clamp(return_steer, -max_steer_angle_ * 0.5, max_steer_angle_ * 0.5);
}

void SimpleController::publish_overtake_path(double offset_direction)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "overtake_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  
  if (!has_valid_overtake_plan_) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    overtake_path_marker_pub_->publish(marker);
    
    visualization_msgs::msg::Marker arrow_delete;
    arrow_delete.header = marker.header;
    arrow_delete.ns = "overtake_arrow";
    arrow_delete.id = 1;
    arrow_delete.action = visualization_msgs::msg::Marker::DELETE;
    overtake_path_marker_pub_->publish(arrow_delete);
    return;
  }
  
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.scale.x = 0.15;  
  
  marker.color.r = 0.2f;
  marker.color.g = 1.0f;
  marker.color.b = 0.2f;
  marker.color.a = 0.9f;
  
  marker.pose.orientation.w = 1.0;
  
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  double safe_lateral_offset = overtake_lateral_offset_ + vehicle_width_;
  
  for (double dist = 0.0; dist < OVERTAKE_VIS_LENGTH; dist += OVERTAKE_VIS_STEP) {
    geometry_msgs::msg::Point p;
    double t = dist / OVERTAKE_VIS_LENGTH;  
    double lateral_offset = offset_direction * safe_lateral_offset * 
                           std::sin(t * M_PI);  
    
    p.x = current_x + dist * std::cos(current_yaw) - lateral_offset * std::sin(current_yaw);
    p.y = current_y + dist * std::sin(current_yaw) + lateral_offset * std::cos(current_yaw);
    p.z = 0.15;
    marker.points.push_back(p);
  }
  
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);  
  overtake_path_marker_pub_->publish(marker);
  
  visualization_msgs::msg::Marker arrow_marker;
  arrow_marker.header.frame_id = "map";
  arrow_marker.header.stamp = this->get_clock()->now();
  arrow_marker.ns = "overtake_arrow";
  arrow_marker.id = 1;
  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.action = visualization_msgs::msg::Marker::ADD;
  
  arrow_marker.scale.x = 0.8;  
  arrow_marker.scale.y = 0.2;  
  arrow_marker.scale.z = 0.2;
  
  arrow_marker.color.r = 1.0f;
  arrow_marker.color.g = 1.0f;
  arrow_marker.color.b = 0.0f;
  arrow_marker.color.a = 0.9f;
  
  double arrow_offset = offset_direction * 0.4;
  arrow_marker.pose.position.x = current_x + 1.0 * std::cos(current_yaw) - arrow_offset * std::sin(current_yaw);
  arrow_marker.pose.position.y = current_y + 1.0 * std::sin(current_yaw) + arrow_offset * std::cos(current_yaw);
  arrow_marker.pose.position.z = 0.3;
  
  double arrow_yaw = current_yaw + offset_direction * 0.3;  
  arrow_marker.pose.orientation.z = std::sin(arrow_yaw / 2.0);
  arrow_marker.pose.orientation.w = std::cos(arrow_yaw / 2.0);
  
  arrow_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  overtake_path_marker_pub_->publish(arrow_marker);
}

void SimpleController::publish_wall_collision_indicator(bool is_colliding, double x, double y)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "wall_collision";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.5;
  marker.pose.orientation.w = 1.0;
  
  if (is_colliding) {
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "WALL COLLISION DETECTED! pos=(%.2f, %.2f), F=%.3fm, L=%.3fm, R=%.3fm",
                          x, y, last_obstacle_front_dist_, 
                          last_obstacle_left_dist_, last_obstacle_right_dist_);
  } else {
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.4f;
  }
  
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  wall_collision_marker_pub_->publish(marker);
}

std::vector<geometry_msgs::msg::Point> SimpleController::generate_overtake_trajectory(double overtake_direction)
{
  std::vector<geometry_msgs::msg::Point> trajectory;
  
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  constexpr double OVERTAKE_PATH_LENGTH = 5.0;
  constexpr double PATH_RESOLUTION = 0.2;  
  
  for (double dist = 0.0; dist < OVERTAKE_PATH_LENGTH; dist += PATH_RESOLUTION) {
    geometry_msgs::msg::Point p;
    
    double t = dist / OVERTAKE_PATH_LENGTH;
    
    double lateral_offset = overtake_direction * overtake_lateral_offset_ * std::sin(t * M_PI);
    
    p.x = current_x + dist * std::cos(current_yaw) - lateral_offset * std::sin(current_yaw);
    p.y = current_y + dist * std::sin(current_yaw) + lateral_offset * std::cos(current_yaw);
    p.z = 0.0;
    
    trajectory.push_back(p);
  }
  
  return trajectory;
}

bool SimpleController::validate_overtake_path(double overtake_direction)
{
  std::vector<geometry_msgs::msg::Point> trajectory = generate_overtake_trajectory(overtake_direction);
  
  if (trajectory.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "OVERTAKE PATH FAILED: Cannot generate trajectory");
    return false;
  }
  
  double vehicle_half_width = vehicle_width_ / 2.0;
  double min_wall_clearance = vehicle_half_width + WALL_SAFETY_MARGIN;  
  double min_track_width = vehicle_width_ * 2.0 + SAFETY_MARGIN;        
  
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto& point = trajectory[i];
    
    if (wall_data_loaded_) {
      double wall_left = get_wall_distance_left(point.x, point.y);
      double wall_right = get_wall_distance_right(point.x, point.y);
      
      double wall_clearance = (overtake_direction > 0) ? wall_left : wall_right;
      
      if (wall_clearance < min_wall_clearance) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "OVERTAKE PATH INVALID: Wall too close (%.2fm < %.2fm, vehicle_width=%.2fm)",
                              wall_clearance, min_wall_clearance, vehicle_width_);
        return false;
      }
      
      double track_width = wall_left + wall_right;
      if (track_width < min_track_width) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "OVERTAKE PATH INVALID: Track too narrow (%.2fm < %.2fm for 2 vehicles)",
                              track_width, min_track_width);
        return false;
      }
    }
    
    if (!current_path_.poses.empty()) {
      double min_dist_to_raceline = std::numeric_limits<double>::max();
      
      for (const auto& pose : current_path_.poses) {
        double dx = point.x - pose.pose.position.x;
        double dy = point.y - pose.pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist_to_raceline) {
          min_dist_to_raceline = dist;
        }
      }
      
      double max_deviation = overtake_lateral_offset_ * 1.5;  
      if (min_dist_to_raceline > max_deviation) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "OVERTAKE PATH INVALID: Too far from raceline (%.2fm > %.2fm)",
                              min_dist_to_raceline, max_deviation);
        return false;
      }
    }
  }
  
  if (!trajectory.empty()) {
    const auto& end_point = trajectory.back();
    
    if (!current_path_.poses.empty()) {
      double end_yaw = current_yaw;  
      int end_idx = find_closest_point_along_path(end_point.x, end_point.y, end_yaw, 
                                                    current_path_, last_closest_idx_overtake_);
      double corner_direction = 0.0;
      if (detect_upcoming_corner(end_idx, corner_direction)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "OVERTAKE PATH INVALID: Corner ahead after overtake (dir: %.1f)",
                              corner_direction);
        return false;
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), 
              "OVERTAKE PATH VALID: Direction=%s, %zu points verified",
              overtake_direction > 0 ? "LEFT" : "RIGHT", trajectory.size());
  
  return true;
}

bool SimpleController::check_imu_collision()
{
  if (!imu_received_) {
    return false;
  }
  
  if (imu_accel_magnitude_ > IMU_CRASH_ACCEL_THRESHOLD) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                          "CRSM: IMU crash detected! accel=%.2f m/s^2 > %.2f threshold",
                          imu_accel_magnitude_, IMU_CRASH_ACCEL_THRESHOLD);
    return true;
  }
  
  return false;
}

bool SimpleController::check_stall_condition()
{
  if (!odom_received_ || !scan_received_) {
    return false;
  }
  
  double actual_speed = std::sqrt(
    current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
    current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
  );
  
  bool cmd_forward = last_cmd_velocity_ > STALL_CMD_VELOCITY_THRESHOLD;
  bool speed_stalled = actual_speed < STALL_VELOCITY_THRESHOLD;
  bool front_blocked = last_obstacle_front_dist_ < 0.2;
  
  if (cmd_forward && speed_stalled && front_blocked) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "CRSM: Stall detected! cmd=%.2f, actual=%.2f, front=%.2fm",
                          last_cmd_velocity_, actual_speed, last_obstacle_front_dist_);
    return true;
  }
  
  return false;
}

double SimpleController::compute_inverted_reverse_steering()
{
  double reverse_steer = 0.0;
  
  switch (reverse_steering_mode_) {
    case 0:  
      reverse_steer = 0.0;
      break;
      
    case 1:  
      reverse_steer = -last_steering_before_reverse_;
      reverse_steer = std::clamp(reverse_steer, -max_steer_angle_, max_steer_angle_);
      break;
      
    case 2:  
    default:
      reverse_steer = last_steering_before_reverse_;
      break;
  }
  
  return reverse_steer;
}

void SimpleController::publish_crsm_state()
{
  std_msgs::msg::String state_msg;
  
  switch (crsm_state_) {
    case CRSMState::ST_NORMAL:
      state_msg.data = "ST_NORMAL";
      break;
    case CRSMState::ST_CRASH_DETECTED:
      state_msg.data = "ST_CRASH_DETECTED";
      break;
    case CRSMState::ST_RECOVERY_REVERSE:
      state_msg.data = "ST_RECOVERY_REVERSE";
      break;
    case CRSMState::ST_RECOVERY_REALIGN:
      state_msg.data = "ST_RECOVERY_REALIGN";
      break;
  }
  
  state_pub_->publish(state_msg);
}

void SimpleController::publish_safety_zone_markers()
{
  if (!is_following_opponent_ && !is_overtaking_) {
    return;  
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  
  visualization_msgs::msg::Marker opponent_zone;
  opponent_zone.header.frame_id = "map";
  opponent_zone.header.stamp = this->get_clock()->now();
  opponent_zone.ns = "opponent_forbidden_zone";
  opponent_zone.id = 0;
  opponent_zone.type = visualization_msgs::msg::Marker::CUBE;
  opponent_zone.action = visualization_msgs::msg::Marker::ADD;
  
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  double opp_dist = last_obstacle_front_dist_;
  opponent_zone.pose.position.x = current_x + opp_dist * std::cos(current_yaw);
  opponent_zone.pose.position.y = current_y + opp_dist * std::sin(current_yaw);
  opponent_zone.pose.position.z = 0.25;
  opponent_zone.pose.orientation.w = 1.0;
  
  opponent_zone.scale.x = vehicle_length_;
  opponent_zone.scale.y = opponent_width_with_margin_;  
  opponent_zone.scale.z = 0.5;
  
  opponent_zone.color.r = 1.0f;
  opponent_zone.color.g = 0.0f;
  opponent_zone.color.b = 0.0f;
  opponent_zone.color.a = 0.4f;
  
  opponent_zone.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_array.markers.push_back(opponent_zone);
  
  if (is_following_opponent_) {
    visualization_msgs::msg::Marker follow_anchor;
    follow_anchor.header = opponent_zone.header;
    follow_anchor.ns = "follow_anchor";
    follow_anchor.id = 1;
    follow_anchor.type = visualization_msgs::msg::Marker::SPHERE;
    follow_anchor.action = visualization_msgs::msg::Marker::ADD;
    
    double anchor_dist = opp_dist - target_follow_gap_;
    if (anchor_dist > 0) {
      follow_anchor.pose.position.x = current_x + anchor_dist * std::cos(current_yaw);
      follow_anchor.pose.position.y = current_y + anchor_dist * std::sin(current_yaw);
    } else {
      follow_anchor.pose.position.x = current_x;
      follow_anchor.pose.position.y = current_y;
    }
    follow_anchor.pose.position.z = 0.3;
    follow_anchor.pose.orientation.w = 1.0;
    
    follow_anchor.scale.x = 0.3;
    follow_anchor.scale.y = 0.3;
    follow_anchor.scale.z = 0.3;
    
    follow_anchor.color.r = 0.0f;
    follow_anchor.color.g = 0.0f;
    follow_anchor.color.b = 1.0f;
    follow_anchor.color.a = 0.9f;
    
    follow_anchor.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(follow_anchor);
  }
  
  safety_zone_pub_->publish(marker_array);
}

void SimpleController::update_opponent_velocity_estimate()
{
  if (!scan_received_) {
    return;
  }
  
  rclcpp::Time current_time = this->get_clock()->now();
  double dt = (current_time - prev_opponent_time_).seconds();
  
  if (dt > 0.01 && dt < 1.0) {  
    double current_dist = last_obstacle_front_dist_;
    double dist_change = current_dist - prev_opponent_distance_;
    
    double ego_speed = std::sqrt(
      current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
      current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
    );
    
    double relative_vel = dist_change / dt;
    double raw_opponent_vel = ego_speed + relative_vel;
    
    raw_opponent_vel = std::max(0.0, raw_opponent_vel);
    
    estimated_opponent_velocity_ = 0.7 * estimated_opponent_velocity_ + 
                                   0.3 * raw_opponent_vel;
  }
  
  prev_opponent_distance_ = last_obstacle_front_dist_;
  prev_opponent_time_ = current_time;
}

double SimpleController::compute_acc_following_speed(double base_speed)
{
  double front_dist = last_obstacle_front_dist_;
  
  if (front_dist >= follow_distance_threshold_ || front_dist <= a1_threshold_) {
    return base_speed;
  }
  
  update_opponent_velocity_estimate();
  
  double ego_speed = std::sqrt(
    current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
    current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
  );
  
  double gap_error = front_dist - target_follow_gap_;
  
  double velocity_error = estimated_opponent_velocity_ - ego_speed;
  
  double acc_speed = estimated_opponent_velocity_ + 
                     acc_kp_ * gap_error + 
                     acc_kd_ * velocity_error;
  
  acc_speed = std::clamp(acc_speed, 0.0, base_speed);
  
  double min_speed = base_speed * follow_min_speed_ratio_;
  acc_speed = std::max(acc_speed, min_speed);
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                        "ACC: gap=%.2fm, v_opp=%.2f, v_ego=%.2f, gap_err=%.2f, cmd=%.2f",
                        front_dist, estimated_opponent_velocity_, ego_speed, gap_error, acc_speed);
  
  return acc_speed;
}
