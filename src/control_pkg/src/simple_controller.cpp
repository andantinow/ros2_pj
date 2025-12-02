#include "simple_controller.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <rclcpp/qos.hpp>

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
  declare_parameter("pid_integral_limit", 0.0);  // 0.0 means auto-limit based on Ki
  declare_parameter("use_path_interpolation", use_path_interpolation_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("path_topic", path_topic_);
  declare_parameter<std::string>("drive_topic", drive_topic_);

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
  
  prev_time_ = this->get_clock()->now();

  // 1. Path Subscription (Reliable mode with queue depth 10)
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      10,
      std::bind(&SimpleController::path_callback, this, std::placeholders::_1));

  // 2. Odom Subscription (Best Effort mode for simulator compatibility)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SimpleController::odom_callback, this, std::placeholders::_1));

  // 3. Drive Publisher
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_,
      10);
  
  RCLCPP_INFO(this->get_logger(), "Subscribing to odom: %s, path: %s, publishing to drive: %s", 
              odom_topic_.c_str(), path_topic_.c_str(), drive_topic_.c_str());

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

int SimpleController::find_target_point_index(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Path& path, double adaptive_lookahead)
{
    if (path.poses.empty()) return -1;

    static int last_target_idx = 0;  // Track last target for continuity
    
    double current_x = odom.pose.pose.position.x;
    double current_y = odom.pose.pose.position.y;
    double current_yaw = tf2::getYaw(odom.pose.pose.orientation);

    // Find closest point along path (restricted search for stability)
    // Only search forward from last known position to prevent wrong direction
    int search_start = std::max(0, last_target_idx - 5);
    int search_end = std::min(static_cast<int>(path.poses.size()), last_target_idx + 30);
    
    double min_dist_sq = std::numeric_limits<double>::max();
    int closest_idx = last_target_idx;
    
    for (int i = search_start; i < search_end; ++i) {
        double dx = current_x - path.poses[i].pose.position.x;
        double dy = current_y - path.poses[i].pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        
        // Check if point is ahead of vehicle (strong preference)
        double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
        
        // Strongly prefer points ahead, heavily penalize points behind
        double weight = dist_sq;
        if (dot_product > 0.1) {  // Point is clearly ahead
            weight *= 0.3;  // Strong preference for points ahead
        } else if (dot_product < -0.1) {  // Point is behind
            weight *= 10.0;  // Heavy penalty for points behind
        }
        
        if (weight < min_dist_sq) {
            min_dist_sq = weight;
            closest_idx = i;
        }
    }
    
    // Only do global search if no reasonable point found in restricted range
    if (min_dist_sq > 25.0) {  // 5m threshold
        closest_idx = last_target_idx;
        min_dist_sq = std::numeric_limits<double>::max();
        // Still restrict to forward direction
        int global_end = std::min(static_cast<int>(path.poses.size() - 1), last_target_idx + 50);
        for (int i = last_target_idx; i <= global_end; ++i) {
            double dx = current_x - path.poses[i].pose.position.x;
            double dy = current_y - path.poses[i].pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
    }

    // Find target point ahead with lookahead distance (Pure Pursuit core)
    int target_idx = closest_idx;
    double accum_dist = 0.0;
    double min_lookahead_dist = adaptive_lookahead * 0.8;  // Minimum acceptable distance
    double max_lookahead_dist = adaptive_lookahead * 1.2;  // Maximum acceptable distance
    
    // Search forward from closest point to find point at lookahead distance
    for (int i = closest_idx; i < static_cast<int>(path.poses.size() - 1); ++i) {
        double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
        double seg_dist = std::hypot(dx, dy);
        accum_dist += seg_dist;
        
        // Check if we've reached the lookahead distance
        if (accum_dist >= min_lookahead_dist) {
            target_idx = i + 1;
            // Prefer point closest to exact lookahead distance
            if (accum_dist > max_lookahead_dist) {
                break;  // Too far, use previous point
            }
        }
    }
    
    // Ensure target is ahead of vehicle (critical for Pure Pursuit)
    double dx = path.poses[target_idx].pose.position.x - current_x;
    double dy = path.poses[target_idx].pose.position.y - current_y;
    double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
    
    // If target is behind, find next point ahead
    if (dot_product < 0.1 && target_idx < static_cast<int>(path.poses.size() - 1)) {
        for (int i = target_idx + 1; i < static_cast<int>(path.poses.size()); ++i) {
            dx = path.poses[i].pose.position.x - current_x;
            dy = path.poses[i].pose.position.y - current_y;
            dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
            if (dot_product > 0.1) {  // Point is clearly ahead
                target_idx = i;
                break;
            }
        }
    }
    
    // Clamp to valid range
    target_idx = std::max(0, std::min(target_idx, static_cast<int>(path.poses.size() - 1)));
    last_target_idx = target_idx;
    
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

  // Compute current speed
  double current_speed = std::sqrt(
    current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
    current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
  );

  // Get current vehicle state
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  // Adaptive lookahead distance based on speed (Pure Pursuit only)
  // Range: 0.8 (low speed) ~ 2.0 (high speed)
  double adaptive_lookahead = compute_adaptive_lookahead(current_speed);

  // Find target point with adaptive lookahead
  int target_idx = find_target_point_index(current_odom_, current_path_, adaptive_lookahead);
  if (target_idx < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Failed to find target point. Current pos: (%.2f, %.2f), Path size: %zu",
                        current_odom_.pose.pose.position.x, 
                        current_odom_.pose.pose.position.y,
                        current_path_.poses.size());
    return;
  }

  // Path interpolation for smoother following
  geometry_msgs::msg::PoseStamped target_pose;
  if (use_path_interpolation_ && target_idx < static_cast<int>(current_path_.poses.size() - 1)) {
    // Interpolate between current and next point based on lookahead distance
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

  // Transform target to vehicle frame
  // Vehicle frame: x forward, y left, z up
  // Global to vehicle: rotate by -current_yaw
  // [x_veh]   [cos(θ)  sin(θ)] [x_global - x_curr]
  // [y_veh] = [-sin(θ) cos(θ)] [y_global - y_curr]
  double dx_global = target_x_global - current_x;
  double dy_global = target_y_global - current_y;
  double cos_yaw = std::cos(current_yaw);
  double sin_yaw = std::sin(current_yaw);
  
  double target_x_vehicle = dx_global * cos_yaw + dy_global * sin_yaw;
  double target_y_vehicle = -dx_global * sin_yaw + dy_global * cos_yaw;
  
  // Debug: log coordinate transformation
  static int coord_debug_count = 0;
  if (coord_debug_count++ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(), 
                "Coord: global_dx=%.3f, global_dy=%.3f, veh_x=%.3f, veh_y=%.3f, yaw=%.3f",
                dx_global, dy_global, target_x_vehicle, target_y_vehicle, current_yaw);
  }

  // Pure Pursuit: Calculate steering angle
  // Formula: steering = atan2(2.0 * wheelbase * sin(alpha), lookahead_distance)
  double alpha = std::atan2(target_y_vehicle, target_x_vehicle);
  double steering_angle = 0.0;
  
  if (adaptive_lookahead > 1e-6) {
    double sin_alpha_clipped = std::max(-1.0, std::min(1.0, std::sin(alpha)));
    steering_angle = std::atan2(2.0 * wheelbase_ * sin_alpha_clipped, adaptive_lookahead);
  }

  // Clamp to physical steering limits [-max_steer_angle_, max_steer_angle_]
  steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);

  // Apply steering sign correction (for coordinate system mismatch)
  steering_angle *= steering_sign_;
  
  // Apply steering rate limiting for smooth control
  rclcpp::Time current_time = this->get_clock()->now();
  double dt = (current_time - prev_time_).seconds();
  if (dt > 0.1) dt = 0.1;
  if (dt <= 0.0) dt = 0.02;
  steering_angle = limit_steering_rate(steering_angle, dt);
  
  // Low-pass filter for smooth steering
  steering_angle = smooth_steering(steering_angle, prev_steering_angle_);
  
  prev_steering_angle_ = steering_angle;
  prev_time_ = current_time;

  // Debug logging
  static int debug_count = 0;
  if (debug_count++ % 25 == 0) {  // Every 0.5 seconds
    RCLCPP_INFO(this->get_logger(), 
                "Pure Pursuit: alpha=%.3f, lookahead=%.2f, steer=%.3f, target_idx=%d",
                alpha, adaptive_lookahead, steering_angle, target_idx);
  }

  // Speed control based on curvature (simplified) with max speed clamp
  double curvature = std::abs(steering_angle) / wheelbase_;
  double speed_factor = 1.0 / (1.0 + 2.0 * curvature);
  double adjusted_speed = target_speed_ * speed_factor;
  adjusted_speed = std::clamp(adjusted_speed, 0.0, max_speed_);

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = current_time;
  drive_msg.header.frame_id = current_odom_.child_frame_id;
  drive_msg.drive.speed = adjusted_speed;
  drive_msg.drive.steering_angle = steering_angle;

  drive_pub_->publish(drive_msg);
  
  static int publish_count = 0;
  if (publish_count++ % 50 == 0) {  // Every 1 second at 20ms timer
    RCLCPP_INFO(this->get_logger(), "Published drive command: speed=%.2f, steer=%.3f, target_idx=%d, lateral_err=%.3f",
                drive_msg.drive.speed, drive_msg.drive.steering_angle, target_idx, lateral_error);
  }
}

double SimpleController::compute_adaptive_lookahead(double speed)
{
  // Adaptive lookahead: increases with speed (Pure Pursuit optimization)
  // Formula: L = clamp(L_min + speed_gain * v, L_min, L_max)
  // Range: 0.8m (low speed) ~ 2.0m (high speed)
  // - Low speed: close lookahead for precise cornering, prevents cutting corners
  // - High speed: far lookahead for smooth stability
  double adaptive = min_lookahead_ + lookahead_speed_gain_ * speed;
  return std::max(min_lookahead_, std::min(max_lookahead_, adaptive));
}

double SimpleController::compute_lateral_error(double current_x, double current_y, double current_yaw, 
                                               const nav_msgs::msg::Path& path, int closest_idx)
{
  if (closest_idx < 0 || closest_idx >= static_cast<int>(path.poses.size())) {
    return 0.0;
  }

  // Get the path point
  double path_x = path.poses[closest_idx].pose.position.x;
  double path_y = path.poses[closest_idx].pose.position.y;
  
  // Compute path heading from path orientation
  double path_yaw = tf2::getYaw(path.poses[closest_idx].pose.orientation);
  
  // Compute vector from vehicle to path point (in global frame)
  double dx = path_x - current_x;
  double dy = path_y - current_y;
  
  // Transform to vehicle frame
  // Vehicle frame: x forward, y left
  double cos_yaw = std::cos(current_yaw);
  double sin_yaw = std::sin(current_yaw);
  double dx_vehicle = dx * cos_yaw + dy * sin_yaw;
  double dy_vehicle = -dx * sin_yaw + dy * cos_yaw;
  
  // Lateral error: y-component in vehicle frame
  // Positive dy_vehicle = path is to the left of vehicle = need left turn = positive steering
  // Negative dy_vehicle = path is to the right of vehicle = need right turn = negative steering
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
  
  // Normalize to [-pi, pi]
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
  
  // Interpolate orientation (simple linear interpolation of yaw)
  double yaw1 = tf2::getYaw(path.poses[idx].pose.orientation);
  double yaw2 = tf2::getYaw(path.poses[idx + 1].pose.orientation);
  
  // Handle wrap-around
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
  
  // Proportional term
  double p_term = pid_kp_ * error;
  
  // Integral term
  lateral_error_integral_ += error * dt;
  // Anti-windup: limit integral with configurable bound
  double max_integral;
  if (pid_integral_limit_ > 0.0) {
    max_integral = pid_integral_limit_;
  } else {
    // Fallback: auto-limit based on Ki (keeps I-term roughly bounded)
    max_integral = 1.0 / std::max(pid_ki_, 1e-6);
  }
  lateral_error_integral_ = std::max(-max_integral, std::min(max_integral, lateral_error_integral_));
  double i_term = pid_ki_ * lateral_error_integral_;
  
  // Derivative term
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
  
  // Compute curvature using three points
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
  
  // Curvature = cross product / (ds1 * ds2 * average_ds)
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
  
  // Restricted search: only look forward from current position to prevent wrong direction
  // This prevents the vehicle from jumping to waypoints behind or far away at startup
  int search_range_forward = 20;  // Only search 20 points ahead
  int search_range_backward = 5;  // Minimal backward search for safety
  int min_idx = std::max(0, start_idx - search_range_backward);
  int max_idx = std::min(static_cast<int>(path.poses.size() - 1), start_idx + search_range_forward);
  
  double min_dist_sq = std::numeric_limits<double>::max();
  int closest_idx = start_idx;
  
  for (int i = min_idx; i <= max_idx; ++i) {
    double dx = current_x - path.poses[i].pose.position.x;
    double dy = current_y - path.poses[i].pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    
    // Check if point is ahead of vehicle
    double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
    
    // Weight: strongly prefer points ahead and closer
    double weight = dist_sq;
    if (dot_product > 0) {
      weight *= 0.3;  // Even stronger preference for points ahead
    } else if (dot_product < -0.1) {
      weight *= 10.0;  // Heavily penalize points behind
    }
    
    if (weight < min_dist_sq) {
      min_dist_sq = weight;
      closest_idx = i;
    }
  }
  
  // Only do global search if no reasonable point found in restricted range
  // This prevents jumping to wrong waypoints at startup
  if (min_dist_sq > 25.0) {  // 5m threshold (increased from 3.16m)
    // Still restrict global search to forward direction only
    closest_idx = start_idx;
    min_dist_sq = std::numeric_limits<double>::max();
    int global_search_end = std::min(static_cast<int>(path.poses.size() - 1), start_idx + 50);
    for (int i = start_idx; i <= global_search_end; ++i) {
      double dx = current_x - path.poses[i].pose.position.x;
      double dy = current_y - path.poses[i].pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_idx = i;
      }
    }
  }
  
  return closest_idx;
}

double SimpleController::smooth_steering(double new_steering, double prev_steering)
{
  // Exponential moving average filter
  return smoothing_factor_ * new_steering + (1.0 - smoothing_factor_) * prev_steering;
}
