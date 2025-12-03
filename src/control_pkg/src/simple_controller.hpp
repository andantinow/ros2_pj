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

  double lookahead_distance_ = 0.0;
  double min_lookahead_ = 0.8;
  double max_lookahead_ = 2.0;
  double lookahead_speed_gain_ = 0.8;
  double lookahead_error_gain_ = 0.0;
  double target_speed_ = 2.0;
  double max_speed_ = 2.0;
  double wheelbase_ = 0.33;
  double max_steer_angle_ = 0.52;  // Physical steering limit [rad]
  double max_steer_rate_ = 1.0;  // rad/s
  double lateral_error_gain_ = 0.5;  // Lateral error compensation gain
  double heading_error_gain_ = 1.0;  // Heading error compensation gain (Stanley term)
  double curvature_feedforward_gain_ = 1.2;  // Curvature feedforward gain
  double smoothing_factor_ = 0.3;  // Steering angle smoothing (0-1, lower = more smoothing)
  double steering_sign_ = 1.0;  // Steering angle sign (1.0 or -1.0, for coordinate system correction)
  double direct_correction_gain_ = 0.0;
  double direct_correction_limit_ = 0.0;
  
  // PID control for lateral error
  double pid_kp_ = 1.5;
  double pid_ki_ = 0.05;
  double pid_kd_ = 0.5;
  double pid_integral_limit_ = 0.0;
  double lateral_error_integral_ = 0.0;
  double prev_lateral_error_ = 0.0;
  
  // Path interpolation
  bool use_path_interpolation_ = true;
  
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
  double compute_heading_error(double current_yaw, const nav_msgs::msg::Path& path, int closest_idx);
  double compute_path_curvature(const nav_msgs::msg::Path& path, int idx);
  geometry_msgs::msg::PoseStamped interpolate_path_point(const nav_msgs::msg::Path& path, int idx, double fraction);
  double compute_pid_control(double error, double dt);
  double limit_steering_rate(double desired_steering, double dt);
  double smooth_steering(double new_steering, double prev_steering);
  int find_closest_point_along_path(double current_x, double current_y, double current_yaw, const nav_msgs::msg::Path& path, int start_idx);
};
#endif
