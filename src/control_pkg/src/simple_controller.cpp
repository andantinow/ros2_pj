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
  declare_parameter("target_speed", target_speed_);
  declare_parameter("wheelbase", wheelbase_);
  declare_parameter("max_steer_angle", max_steer_angle_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("path_topic", path_topic_);
  declare_parameter<std::string>("drive_topic", drive_topic_);

  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  target_speed_ = get_parameter("target_speed").as_double();
  wheelbase_ = get_parameter("wheelbase").as_double();
  max_steer_angle_ = get_parameter("max_steer_angle").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  drive_topic_ = get_parameter("drive_topic").as_string();

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
    RCLCPP_INFO(this->get_logger(), "Received first odometry message");
    odom_received_ = true;
  }
}

void SimpleController::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = *msg;
  if (!path_received_) {
    RCLCPP_INFO(this->get_logger(), "Received first path message with %zu points", msg->poses.size());
    path_received_ = true;
  }
}

int SimpleController::find_target_point_index(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Path& path)
{
    if (path.poses.empty()) return -1;

    double current_x = odom.pose.pose.position.x;
    double current_y = odom.pose.pose.position.y;

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

    double lookahead_sq = lookahead_distance_ * lookahead_distance_;
    for (size_t i = closest_idx; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - current_x;
        double dy = path.poses[i].pose.position.y - current_y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq >= lookahead_sq) {
            return static_cast<int>(i);
        }
    }

    return static_cast<int>(path.poses.size() - 1);
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

  int target_idx = find_target_point_index(current_odom_, current_path_);
  if (target_idx < 0) return;

  geometry_msgs::msg::PoseStamped target_pose = current_path_.poses[target_idx];

  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);

  double target_x_global = target_pose.pose.position.x;
  double target_y_global = target_pose.pose.position.y;

  double target_x_vehicle = (target_x_global - current_x) * std::cos(-current_yaw) - (target_y_global - current_y) * std::sin(-current_yaw);
  double target_y_vehicle = (target_x_global - current_x) * std::sin(-current_yaw) + (target_y_global - current_y) * std::cos(-current_yaw);

  double alpha = std::atan2(target_y_vehicle, target_x_vehicle);
  double steering_angle = 0.0;

  if (lookahead_distance_ > 1e-6) {
     double sin_alpha_clipped = std::max(-1.0, std::min(1.0, std::sin(alpha)));
     steering_angle = std::atan2(2.0 * wheelbase_ * sin_alpha_clipped, lookahead_distance_);
  }

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->get_clock()->now();
  drive_msg.header.frame_id = current_odom_.child_frame_id;
  drive_msg.drive.speed = target_speed_;
  drive_msg.drive.steering_angle = std::max(-max_steer_angle_, std::min(max_steer_angle_, steering_angle));

  drive_pub_->publish(drive_msg);
}
