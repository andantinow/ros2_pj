#ifndef SIMPLE_CONTROLLER_HPP_
#define SIMPLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Path current_path_;
  sensor_msgs::msg::LaserScan current_scan_;
  bool odom_received_ = false;
  bool path_received_ = false;
  bool scan_received_ = false;

  double lookahead_distance_ = 0.0;
  double min_lookahead_ = 1.5;         // Increased to look further ahead (1.0 -> 1.5)
  double max_lookahead_ = 4.0;         // Increased for more look-ahead on straights (3.0 -> 4.0)
  double lookahead_speed_gain_ = 0.6;  // Decreased for smoother look-ahead (0.8 -> 0.6)
  double lookahead_error_gain_ = 0.0;
  double target_speed_ = 2.0;
  double max_speed_ = 2.0;
  double wheelbase_ = 0.33;
  double max_steer_angle_ = 0.40;  // Reduced max steering limit (0.52 -> 0.40) [rad]
  double max_steer_rate_ = 4.0;  // Increased for faster steering response (1.0 -> 4.0) rad/s
  double lateral_error_gain_ = 1.5;  // Increased lateral error compensation gain (0.5 -> 1.5)
  double heading_error_gain_ = 0.8;  // Reduced heading error compensation gain (1.0 -> 0.8)
  double curvature_feedforward_gain_ = 1.2;  // Reduced curvature feedforward gain
  double smoothing_factor_ = 0.6;  // Increased for smoother response (0.3 -> 0.6)
  double steering_sign_ = 1.0;  // Steering angle sign (1.0 or -1.0, for coordinate system correction)
  double direct_correction_gain_ = 0.0;
  double direct_correction_limit_ = 0.0;
  
  // PID control for lateral error (reduced gains for smoother control)
  double pid_kp_ = 0.7;    // Reduced (1.5 -> 0.7)
  double pid_ki_ = 0.03;   // Reduced (0.05 -> 0.03)
  double pid_kd_ = 0.2;    // Reduced (0.5 -> 0.2)
  double pid_integral_limit_ = 0.0;
  double lateral_error_integral_ = 0.0;
  double prev_lateral_error_ = 0.0;
  
  // Path interpolation
  bool use_path_interpolation_ = true;
  
  // Collision avoidance and reverse parameters
  // 센서 범위 더 축소 - 더 가까운 거리에서만 반응
  double collision_threshold_ = 0.4;      // Distance to trigger reverse (meters) - front collision (reduced)
  double reverse_speed_ = 0.5;            // Gentle reverse speed (m/s)
  double reverse_duration_ = 1.0;         // How long to reverse (seconds)
  double side_collision_threshold_ = 0.3; // Side obstacle threshold (meters) - reduced range (0.4 -> 0.3)
  bool is_reversing_ = false;             // Current reverse state
  rclcpp::Time reverse_start_time_;       // When reverse started
  double last_steering_before_reverse_ = 0.0;  // Steering angle before reversing
  
  // 장애물 위치 정보 (후진 방향 결정용)
  double last_obstacle_left_dist_ = 10.0;   // 왼쪽 장애물 거리
  double last_obstacle_right_dist_ = 10.0;  // 오른쪽 장애물 거리
  double last_obstacle_front_dist_ = 10.0;  // 전방 장애물 거리
  double last_obstacle_angle_ = 0.0;        // 가장 가까운 전방 장애물 각도
  
  // 측면 센서 기반 회피 조향 (벽 회피용)
  double side_avoidance_gain_ = 0.3;        // 측면 회피 강도 감소 (0.5 -> 0.3)
  bool enable_side_avoidance_ = true;       // 측면 회피 활성화
  
  std::string odom_topic_{"/odom"};
  std::string path_topic_{"/global_raceline"};
  std::string drive_topic_{"/drive"};
  std::string scan_topic_{"/scan"};
  
  double prev_steering_angle_ = 0.0;
  rclcpp::Time prev_time_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
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
  bool check_collision_imminent();
  double compute_reverse_steering();
  double compute_side_avoidance_steering();  // 측면 센서 기반 벽 회피 조향
};
#endif
