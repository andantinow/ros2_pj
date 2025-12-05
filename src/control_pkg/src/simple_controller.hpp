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

  // === Constants for collision avoidance and overtake ===
  static constexpr double SAFETY_MARGIN = 0.3;              // General safety margin (m)
  static constexpr double WALL_SAFETY_MARGIN = 0.15;        // Wall clearance safety margin (m)
  static constexpr double MIN_OVERTAKE_DISTANCE = 0.6;      // Minimum distance for overtake consideration (m)
  static constexpr double OVERTAKE_VIS_LENGTH = 5.0;        // Overtake visualization path length (m)
  static constexpr double OVERTAKE_VIS_STEP = 0.15;         // Overtake visualization step size (m)

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
  double max_steer_angle_ = 0.6981;  // Max steering limit: 40 degrees [rad]
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
  double min_overtake_gap_ = 1.2;            // 최소 추월 가능 간격 (m) - 더 엄격하게 (1.0 -> 1.2)
  double overtake_speed_boost_ = 1.5;        // 추월 시 속도 증가 배율 (1.5x)
  double post_overtake_speed_factor_ = 1.2;  // 추월 후 속도 유지 비율 (1.2x)
  double post_overtake_duration_ = 2.0;      // 추월 후 고속 유지 시간 (seconds)
  double overtake_steer_max_ = 0.25;         // 추월 시 최대 조향 강도 (rad)
  double overtake_steer_decay_ = 0.6;        // 추월 조향 감쇠율 (완료시점에 40% 유지)
  double overtake_wall_caution_dist_ = 0.6;  // 벽 주의 거리 (이보다 가까우면 조향 강도 감소) - 더 조심스럽게 (0.5 -> 0.6)
  rclcpp::Time overtake_start_time_ros_;     // 추월 시작 시간 (ROS Time)
  rclcpp::Time overtake_end_time_ros_;       // 추월 종료 시간 (ROS Time)
  bool is_post_overtake_ = false;            // 추월 직후 고속 유지 상태
  
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
  // A1 범위: 좁은 범위 - 이 범위 안에 들어오면 후진 + 조향각 유지
  // A2 범위: 약간 넓은 범위 - 벽 반발 조향 적용
  // wall_repulsion_threshold: 벽 반발 시작 거리 (기본값 0.25m)
  // 참고: 충돌 회피 우선순위: A1(후진) < A2(급조향) < wall_repulsion(반발조향)
  //       예: 18cm < 50cm (a1 < a2)
  double a1_threshold_ = 0.18;             // A1 범위: 후진 트리거 거리 (meters) - 18cm (벽 충돌 더 일찍 감지)
  double a2_threshold_ = 0.50;             // A2 범위: 조향 회피 거리 (meters) - 50cm (a1보다 커야 함)
  double a2_urgent_threshold_ = 0.30;      // A2 긴급 범위: 30cm 이내시 강한 조향
  double a1_side_factor_ = 0.9;            // A1 측면 거리 팩터 (a1_threshold * 이 값) - 측면도 민감하게
  double a2_max_steer_ratio_ = 1.0;        // A2 최대 조향 비율
  double a2_urgent_steer_ratio_ = 1.0;     // A2 긴급시 최대 조향 비율
  double reverse_speed_ = 0.8;             // 후진 속도 (m/s) - 조금 느리게 후진
  double reverse_duration_ = 1.2;          // 후진 지속 시간 (seconds) - 조금 더 오래 후진
  double reverse_pause_duration_ = 0.5;    // 후진 후 정지 시간 (seconds) - 생각 시간
  double a1_steer_gain_ = 0.8;             // A1 범위에서 후진 시 조향 강도
  double a2_steer_gain_ = 1.0;             // A2 범위에서 회피 조향 강도
  double a2_urgent_steer_gain_ = 1.5;      // A2 긴급시 조향 강도
  bool is_reversing_ = false;              // 현재 후진 중인지
  bool is_pausing_after_reverse_ = false;  // 후진 후 정지 중인지 (생각 시간)
  bool is_planning_after_pause_ = false;   // 정지 후 경로 계획 중인지 (새 기능)
  bool is_in_a1_zone_ = false;             // 현재 A1 범위에 있는지 (중복 체크 방지)
  bool just_finished_reverse_ = false;     // 후진 직후 상태 (재후진 방지용)
  int reverse_cooldown_counter_ = 0;       // 후진 쿨다운 카운터 (무한 루프 방지)
  static constexpr int REVERSE_COOLDOWN_CYCLES = 50;  // 약 1초 (20ms * 50) - 더 길게
  rclcpp::Time reverse_start_time_;        // 후진 시작 시간
  rclcpp::Time pause_start_time_;          // 정지 시작 시간
  rclcpp::Time planning_start_time_;       // 경로 계획 시작 시간 (새 기능)
  double last_steering_before_reverse_ = 0.0;  // 충돌 시 조향각 (유지용)
  double planning_duration_ = 0.3;         // 경로 계획에 필요한 시간 (seconds)
  
  // === 벽 반발 조향 시스템 (수식 기반) ===
  // 벽에 가까워지면 반발력으로 조향 (속도 감소 없이)
  double wall_repulsion_gain_ = 0.8;       // 벽 반발 조향 게인
  double wall_repulsion_threshold_ = 0.20; // 벽 반발 시작 거리 (20cm)
  double wall_repulsion_max_steer_ = 0.3;  // 최대 반발 조향각 (rad, ~17도)
  
  // === 상대 차량 Following 시스템 (거리 비례 속도 조절) ===
  bool is_following_opponent_ = false;     // 현재 상대 차량 following 중인지
  double follow_distance_threshold_ = 1.5; // following 시작 거리 (m) - 사용자 요청에 따라 1.5m로 조정
  double follow_min_distance_ = 0.4;       // 최소 유지 거리 (m) - 이보다 가까우면 정지에 가까움
  double follow_speed_factor_ = 0.3;       // following 시 속도 비율 (앞 차 있을 때 적절하게 느리게)
  double follow_min_speed_ratio_ = 0.1;    // 최소 속도 비율 (10%) - 완전 정지 방지
  double speed_smooth_factor_ = 0.1;       // 속도 변화 스무딩 (급격한 속도 변화 방지)
  double last_adjusted_speed_ = 0.0;       // 마지막 조정된 속도 (스무딩용)
  
  // === 차량 치수 (추월 경로 계산용) ===
  double vehicle_width_ = 0.3;             // 차량 넓이 (m) - 약 30cm (F1TENTH 기준)
  double vehicle_length_ = 0.5;            // 차량 길이 (m) - 약 50cm (F1TENTH 기준)
  
  // === 추월 조건 체크용 파라미터 ===
  double narrow_road_threshold_ = 2.0;     // 좁은 도로 판단 기준 (좌우 합계 m) - 더 엄격하게 (1.5 -> 2.0)
  double min_visibility_angle_ = 0.35;     // 최소 시야 각도 (rad, 약 20도) - 더 엄격하게 (0.5 -> 0.35)
  double min_overtake_clearance_ = 0.0;    // 최소 추월 간격 (계산됨: vehicle_width * 2 + safety_margin)
  
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
  double compute_wall_repulsion_steering(); // 벽 반발 조향 계산 (수식 기반)
  double compute_opponent_following_speed(double base_speed);  // 상대 차량 following 속도 계산
  double smooth_speed_change(double target_speed, double current_speed);  // 속도 변화 스무딩
  double compute_overtake_speed(double base_speed);  // 추월 시 속도 계산
  void check_and_start_overtake();  // 추월 시작 조건 체크 및 시작
  void update_overtake_state();  // 추월 상태 업데이트
  
  // === 새로운 기능들 ===
  bool load_wall_data(const std::string& csv_file);  // 벽 데이터 CSV 로드
  size_t find_closest_wall_point(double x, double y);  // 헬퍼: 가장 가까운 벽 포인트 인덱스
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
  bool validate_overtake_path(double overtake_direction);  // 추월 경로 시뮬레이션 및 안전성 검증
  std::vector<geometry_msgs::msg::Point> generate_overtake_trajectory(double overtake_direction);  // 추월 경로 생성
  
  // 벽 검색 캐싱
  mutable size_t last_wall_search_idx_ = 0;
  
  // 추월 안전 체크용 인덱스 캐싱
  mutable int last_closest_idx_overtake_ = 0;
  
  // 추월 경로 계획용 멤버 변수
  double planned_overtake_direction_ = 0.0;  // 계획된 추월 방향 (좌:1.0, 우:-1.0)
  bool has_valid_overtake_plan_ = false;     // 유효한 추월 계획이 있는지
};
#endif  // SIMPLE_CONTROLLER_HPP_
