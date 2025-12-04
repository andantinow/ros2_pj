#ifndef SIMPLE_CONTROLLER_HPP_
#define SIMPLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Path current_path_;
  sensor_msgs::msg::LaserScan current_scan_;
  bool odom_received_ = false;
  bool path_received_ = false;
  bool scan_received_ = false;

  double lookahead_distance_ = 0.0;
  double min_lookahead_ = 0.8;         // Reduced for closer lookahead at low speed (1.5 -> 0.8)
  double max_lookahead_ = 2.5;         // Reduced for tighter corner tracking (4.0 -> 2.5)
  double lookahead_speed_gain_ = 0.5;  // Reduced for less aggressive speed-based lookahead
  double lookahead_error_gain_ = 0.0;
  double target_speed_ = 2.0;
  double max_speed_ = 2.0;
  double wheelbase_ = 0.33;
  double max_steer_angle_ = 0.436;  // Max steering limit: 25 degrees [rad]
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
  
  // === A1/A2 범위 기반 충돌 회피 시스템 ===
  // A1 범위: 좁은 범위 - 이 범위 안에 들어오면 후진 + 반대방향 조향
  // A2 범위: 넓은 범위 - A1보다 넓지만 이 범위 안에 들어오면 조향만 반대방향으로
  double a1_threshold_ = 0.15;            // A1 범위: 후진 트리거 거리 (meters) - 0.15m 벽에 가까움
  double a2_threshold_ = 0.6;             // A2 범위: 조향 회피 거리 (meters) - 0.6m
  double a1_side_factor_ = 0.8;           // A1 측면 거리 팩터 (a1_threshold * 이 값)
  double a2_max_steer_ratio_ = 0.8;       // A2 최대 조향 비율 (max_steer_angle * 이 값) - 더 강한 회피
  double reverse_speed_ = 0.5;            // 후진 속도 (m/s)
  double reverse_duration_ = 0.8;         // 후진 지속 시간 (seconds)
  double a1_steer_gain_ = 0.8;            // A1 범위에서 후진 시 조향 강도
  double a2_steer_gain_ = 0.8;            // A2 범위에서 회피 조향 강도 - 더 강한 회피
  bool is_reversing_ = false;             // 현재 후진 중인지
  bool is_in_a1_zone_ = false;            // 현재 A1 범위에 있는지 (중복 체크 방지)
  rclcpp::Time reverse_start_time_;       // 후진 시작 시간
  double last_steering_before_reverse_ = 0.0;  // 후진 전 조향각
  
  // 장애물 위치 정보 (회피 방향 결정용)
  double last_obstacle_left_dist_ = 10.0;   // 왼쪽 장애물 거리
  double last_obstacle_right_dist_ = 10.0;  // 오른쪽 장애물 거리
  double last_obstacle_front_dist_ = 10.0;  // 전방 장애물 거리
  double last_obstacle_angle_ = 0.0;        // 가장 가까운 전방 장애물 각도
  
  // 활성화 플래그
  bool enable_collision_avoidance_ = true;  // 충돌 회피 활성화
  
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
  bool check_a1_zone();                     // A1 범위 체크 (후진 필요)
  bool check_a2_zone();                     // A2 범위 체크 (조향 회피 필요)
  double compute_reverse_steering();        // 후진 시 조향 계산 (A1)
  double compute_avoidance_steering();      // 회피 조향 계산 (A2)
  void update_obstacle_distances();         // 장애물 거리 업데이트
  void publish_lookahead_marker(double x, double y, double z);  // Lookahead 시각화
};
#endif
