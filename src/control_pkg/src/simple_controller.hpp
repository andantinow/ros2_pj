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
  double min_lookahead_ = 0.4;         // Much reduced for tight path following (0.8 -> 0.4)
  double max_lookahead_ = 1.2;         // Much reduced for precise corner tracking (2.5 -> 1.2)
  double lookahead_speed_gain_ = 0.3;  // Reduced for tighter speed-based lookahead (0.5 -> 0.3)
  double lookahead_error_gain_ = 0.0;
  double target_speed_ = 2.0;
  double max_speed_ = 2.0;
  double wheelbase_ = 0.33;
  double max_steer_angle_ = 0.6458;  // Max steering limit: 37 degrees [rad]
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
  
  // Corner handling parameters (코너링 설정)
  double corner_curvature_threshold_ = 0.5;  // 곡률이 이 값 이상이면 코너로 판단
  double corner_speed_factor_ = 0.5;         // 코너에서 속도 감소 비율 (0.5 = 50% 속도)
  double corner_steer_amplify_ = 1.5;        // 코너에서 조향각 증폭 비율 (더 큰 각도로 돌기)
  
  // === Out-In-Out 코너링 (레이싱 라인) ===
  double corner_approach_distance_ = 1.5;    // 코너 접근 감지 거리 (m)
  double out_in_out_offset_ = 0.3;           // 아웃-인-아웃 오프셋 (m) - 코너 전 바깥으로 이동
  bool enable_out_in_out_ = true;            // 아웃-인-아웃 활성화
  
  // === 추월 시스템 (Opponent Overtaking) ===
  bool is_overtaking_ = false;               // 현재 추월 중인지
  double overtake_start_time_ = 0.0;         // 추월 시작 시간
  double overtake_max_duration_ = 3.0;       // 최대 추월 지속 시간 (seconds)
  double overtake_lateral_offset_ = 0.5;     // 추월 시 횡방향 오프셋 (m)
  double return_to_line_distance_ = 2.0;     // 추월 후 라인 복귀 거리 (m)
  double min_overtake_gap_ = 1.0;            // 최소 추월 가능 간격 (m)
  rclcpp::Time overtake_start_time_ros_;     // 추월 시작 시간 (ROS Time)
  
  // 추월 경로 시각화
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr overtake_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_collision_marker_pub_;
  
  // 벽 데이터 (CSV 로드)
  struct WallPoint {
    double x, y;
    double d_left, d_right;
    double psi;
  };
  std::vector<WallPoint> wall_data_;
  bool wall_data_loaded_ = false;
  // Path interpolation
  bool use_path_interpolation_ = true;
  
  // === A1/A2 범위 기반 충돌 회피 시스템 ===
  // A1 범위: 좁은 범위 - 이 범위 안에 들어오면 후진 + 반대방향 조향
  // A2 범위: 넓은 범위 - A1보다 넓지만 이 범위 안에 들어오면 조향만 반대방향으로
  double a1_threshold_ = 0.1;              // A1 범위: 후진 트리거 거리 (meters) - 0.1m
  double a2_threshold_ = 0.2;              // A2 범위: 조향 회피 거리 (meters) - 0.2m (Changed from 0.4m)
  double a2_urgent_threshold_ = 0.15;      // A2 긴급 범위: 0.15m 이내시 급격한 조향 (Changed from 0.25m)
  double a1_side_factor_ = 0.8;           // A1 측면 거리 팩터 (a1_threshold * 이 값)
  double a2_max_steer_ratio_ = 1.0;        // A2 최대 조향 비율 - 완전 반대 조향
  double a2_urgent_steer_ratio_ = 1.0;     // A2 긴급시 최대 조향 비율 - 완전 반대 조향
  double reverse_speed_ = 1.0;             // 후진 속도 (m/s) - 더 시원하게
  double reverse_duration_ = 0.8;         // 후진 지속 시간 (seconds)
  double a1_steer_gain_ = 0.8;            // A1 범위에서 후진 시 조향 강도
  double a2_steer_gain_ = 1.0;             // A2 범위에서 회피 조향 강도 - 완전 반대 조향
  double a2_urgent_steer_gain_ = 1.5;     // A2 긴급시 조향 강도 - 0.4m 이내시 1.5배 강화 (NEW)
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
  double compute_avoidance_steering(double lookahead_angle);  // 회피 조향 계산 (A2) - 완전 반대 조향
  void update_obstacle_distances();         // 장애물 거리 업데이트
  double find_gap_center_angle(double lookahead_angle);  // lookahead 방향 기준 장애물 사이 중간점 각도 계산
  void publish_lookahead_marker(double x, double y, double z);  // Lookahead 시각화
  
  // === 새로운 기능들 ===
  bool load_wall_data(const std::string& csv_file);  // 벽 데이터 CSV 로드
  double get_wall_distance_left(double x, double y);  // 현재 위치에서 왼쪽 벽까지 거리
  double get_wall_distance_right(double x, double y); // 현재 위치에서 오른쪽 벽까지 거리
  bool detect_upcoming_corner(int current_idx, double& corner_direction);  // 전방 코너 감지
  double compute_out_in_out_offset(int current_idx, double corner_direction);  // 아웃-인-아웃 오프셋 계산
  bool can_overtake_safely(double opponent_dist, double opponent_angle);  // 안전 추월 가능 여부
  double compute_overtake_steering(double base_steering);  // 추월 조향 계산
  bool should_return_to_line();  // 라인 복귀 필요 여부
  double compute_return_to_line_steering();  // 라인 복귀 조향 계산
  void publish_overtake_path(double offset_direction);  // 추월 경로 시각화
  void publish_wall_collision_indicator(bool is_colliding, double x, double y);  // 벽 충돌 시각화
};
#endif  // SIMPLE_CONTROLLER_HPP_
