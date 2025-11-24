#ifndef SIMPLE_CONTROLLER_HPP_
#define SIMPLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/utils.h>
#include <vector>
#include <string>

class SimpleController : public rclcpp::Node
{
public:
  SimpleController();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Path current_path_;
  bool odom_received_ = false;
  bool path_received_ = false;

  double lookahead_distance_ = 1.0;
  double min_lookahead_ = 0.5;
  double max_lookahead_ = 2.0;
  double lookahead_speed_gain_ = 0.5;
  double target_speed_ = 1.5;
  double wheelbase_ = 0.33;
  double max_steer_angle_ = 0.418;
  double max_steer_rate_ = 2.0;  // rad/s
  double lateral_error_gain_ = 0.5;  // Lateral error compensation gain
  double smoothing_factor_ = 0.3;  // Steering angle smoothing (0-1, lower = more smoothing)
  std::string odom_topic_{"/odom"};
  std::string path_topic_{"/global_raceline"};
  std::string drive_topic_{"/drive"};
  
  double prev_steering_angle_ = 0.0;
  rclcpp::Time prev_time_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void control_loop();
  int find_target_point_index(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Path& path, double adaptive_lookahead);
  double compute_adaptive_lookahead(double speed);
  double compute_lateral_error(double current_x, double current_y, double current_yaw, const nav_msgs::msg::Path& path, int closest_idx);
  double limit_steering_rate(double desired_steering, double dt);
  double smooth_steering(double new_steering, double prev_steering);
};
#endif
