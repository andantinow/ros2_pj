#include "simple_controller.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

using namespace std::chrono_literals;

SimpleController::SimpleController() : Node("simple_controller")
{
  RCLCPP_INFO(this->get_logger(), "[Kang Donghyeon] Simple Pure Pursuit Controller (ROSCPP) initializing...");

  declare_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter("min_lookahead", min_lookahead_);
  declare_parameter("max_lookahead", max_lookahead_);
  declare_parameter("lookahead_speed_gain", lookahead_speed_gain_);
  declare_parameter("target_speed", target_speed_);
  declare_parameter("wheelbase", wheelbase_);
  declare_parameter("max_steer_angle", max_steer_angle_);
  declare_parameter("max_steer_rate", max_steer_rate_);
  declare_parameter("lateral_error_gain", lateral_error_gain_);
  declare_parameter("smoothing_factor", smoothing_factor_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("path_topic", path_topic_);
  declare_parameter<std::string>("drive_topic", drive_topic_);

  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  min_lookahead_ = get_parameter("min_lookahead").as_double();
  max_lookahead_ = get_parameter("max_lookahead").as_double();
  lookahead_speed_gain_ = get_parameter("lookahead_speed_gain").as_double();
  target_speed_ = get_parameter("target_speed").as_double();
  wheelbase_ = get_parameter("wheelbase").as_double();
  max_steer_angle_ = get_parameter("max_steer_angle").as_double();
  max_steer_rate_ = get_parameter("max_steer_rate").as_double();
  lateral_error_gain_ = get_parameter("lateral_error_gain").as_double();
  smoothing_factor_ = get_parameter("smoothing_factor").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  drive_topic_ = get_parameter("drive_topic").as_string();
  
  prev_time_ = this->get_clock()->now();

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&SimpleController::odom_callback, this, std::placeholders::_1));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10).transient_local(),
      std::bind(&SimpleController::path_callback, this, std::placeholders::_1));

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
  
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

    double current_x = odom.pose.pose.position.x;
    double current_y = odom.pose.pose.position.y;
    double current_yaw = tf2::getYaw(odom.pose.pose.orientation);

    // Find the closest point on the path
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dx = current_x - path.poses[i].pose.position.x;
        double dy = current_y - path.poses[i].pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    // Find the target point that is ahead of the vehicle
    // Project the vehicle's heading direction to find points ahead
    double lookahead_sq = adaptive_lookahead * adaptive_lookahead;
    
    // Start searching from the closest point, but prefer points ahead
    size_t start_idx = closest_idx;
    for (size_t i = start_idx; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - current_x;
        double dy = path.poses[i].pose.position.y - current_y;
        
        // Check if this point is ahead of the vehicle (in the direction of travel)
        double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
        
        // Only consider points ahead
        if (dot_product > 0) {
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq >= lookahead_sq) {
                return static_cast<int>(i);
            }
        }
    }

    // If no point found ahead with lookahead distance, use the last point
    // But only if it's ahead of the vehicle
    if (!path.poses.empty()) {
        size_t last_idx = path.poses.size() - 1;
        double dx = path.poses[last_idx].pose.position.x - current_x;
        double dy = path.poses[last_idx].pose.position.y - current_y;
        double dot_product = dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
        if (dot_product > 0) {
            return static_cast<int>(last_idx);
        }
    }

    // Fallback: return the closest point if no point ahead is found
    return static_cast<int>(closest_idx);
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

  // Adaptive lookahead distance based on speed
  double adaptive_lookahead = compute_adaptive_lookahead(current_speed);

  // Find closest point for lateral error calculation
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  double min_dist_sq = std::numeric_limits<double>::max();
  int closest_idx = 0;
  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    double dx = current_x - current_path_.poses[i].pose.position.x;
    double dy = current_y - current_path_.poses[i].pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = static_cast<int>(i);
    }
  }

  // Compute lateral error
  double lateral_error = compute_lateral_error(current_x, current_y, current_yaw, current_path_, closest_idx);

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

  geometry_msgs::msg::PoseStamped target_pose = current_path_.poses[target_idx];
  
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
  double target_x_vehicle = (target_x_global - current_x) * std::cos(-current_yaw) - (target_y_global - current_y) * std::sin(-current_yaw);
  double target_y_vehicle = (target_x_global - current_x) * std::sin(-current_yaw) + (target_y_global - current_y) * std::cos(-current_yaw);

  double alpha = std::atan2(target_y_vehicle, target_x_vehicle);
  double steering_angle = 0.0;

  // Pure pursuit steering calculation
  if (adaptive_lookahead > 1e-6) {
    double sin_alpha_clipped = std::max(-1.0, std::min(1.0, std::sin(alpha)));
    steering_angle = std::atan2(2.0 * wheelbase_ * sin_alpha_clipped, adaptive_lookahead);
  }

  // Add lateral error compensation
  steering_angle += lateral_error_gain_ * lateral_error;

  // Limit steering angle
  steering_angle = std::max(-max_steer_angle_, std::min(max_steer_angle_, steering_angle));

  // Compute time delta for rate limiting
  rclcpp::Time current_time = this->get_clock()->now();
  double dt = (current_time - prev_time_).seconds();
  if (dt > 0.1) dt = 0.1;  // Cap dt to prevent large jumps
  prev_time_ = current_time;

  // Apply steering rate limiting
  steering_angle = limit_steering_rate(steering_angle, dt);

  // Apply smoothing filter
  steering_angle = smooth_steering(steering_angle, prev_steering_angle_);
  prev_steering_angle_ = steering_angle;

  // Speed control based on curvature (simplified)
  double curvature = std::abs(steering_angle) / wheelbase_;
  double speed_factor = 1.0 / (1.0 + 2.0 * curvature);
  double adjusted_speed = target_speed_ * speed_factor;

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
  // Adaptive lookahead: increases with speed
  // L = L_min + k * v
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
  
  // Compute vector from vehicle to path point
  double dx = path_x - current_x;
  double dy = path_y - current_y;
  
  // Lateral error: perpendicular distance to path
  // Project onto perpendicular to path direction
  double lateral_error = -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);
  
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

double SimpleController::smooth_steering(double new_steering, double prev_steering)
{
  // Exponential moving average filter
  return smoothing_factor_ * new_steering + (1.0 - smoothing_factor_) * prev_steering;
}
