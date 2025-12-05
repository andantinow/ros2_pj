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
  declare_parameter<std::string>("scan_topic", scan_topic_);
  // A1/A2 범위 기반 충돌 회피 파라미터
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
  // Corner handling parameters (코너링 설정)
  declare_parameter("corner_curvature_threshold", corner_curvature_threshold_);
  declare_parameter("corner_speed_factor", corner_speed_factor_);
  declare_parameter("corner_steer_amplify", corner_steer_amplify_);
  // Out-In-Out 코너링 파라미터
  declare_parameter("corner_approach_distance", corner_approach_distance_);
  declare_parameter("out_in_out_offset", out_in_out_offset_);
  declare_parameter("enable_out_in_out", enable_out_in_out_);
  // 추월 시스템 파라미터
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
  // 추월 조건 체크 파라미터
  declare_parameter("narrow_road_threshold", narrow_road_threshold_);
  declare_parameter("min_visibility_angle", min_visibility_angle_);
  // 차량 치수 파라미터
  declare_parameter("vehicle_width", vehicle_width_);
  declare_parameter("vehicle_length", vehicle_length_);
  // 벽 데이터 파일 경로
  declare_parameter<std::string>("wall_data_file", "");
  // CRSM (Collision Recovery State Machine) 파라미터
  declare_parameter("reverse_steering_mode", reverse_steering_mode_);  // 0=neutral, 1=invert, 2=maintain
  declare_parameter<std::string>("imu_topic", "/imu");
  // ACC (Adaptive Cruise Control) 파라미터
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
  // Corner handling parameters (코너링 설정)
  corner_curvature_threshold_ = get_parameter("corner_curvature_threshold").as_double();
  corner_speed_factor_ = get_parameter("corner_speed_factor").as_double();
  corner_steer_amplify_ = get_parameter("corner_steer_amplify").as_double();
  // Out-In-Out 코너링 파라미터
  corner_approach_distance_ = get_parameter("corner_approach_distance").as_double();
  out_in_out_offset_ = get_parameter("out_in_out_offset").as_double();
  enable_out_in_out_ = get_parameter("enable_out_in_out").as_bool();
  // 추월 시스템 파라미터
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
  
  // CRSM 파라미터 로드
  reverse_steering_mode_ = get_parameter("reverse_steering_mode").as_int();
  std::string imu_topic = get_parameter("imu_topic").as_string();
  
  // ACC 파라미터 로드
  acc_kp_ = get_parameter("acc_kp").as_double();
  acc_kd_ = get_parameter("acc_kd").as_double();
  target_follow_gap_ = get_parameter("target_follow_gap").as_double();
  
  // 상대 차량 폭 + 안전 마진 계산 (Frenet 좌표계 기반 D_forbidden 금지 구역)
  // D_forbidden: 상대 차량 주변의 추월 금지 횡방향 구간
  // D_forbidden = [d_opp - W_car/2 - W_margin, d_opp + W_car/2 + W_margin]
  // 여기서 d_opp는 상대 차량의 횡방향 위치, W_car는 차량 폭, W_margin은 안전 마진
  // Example: W_car = 0.35m, W_margin = 0.15m -> D_forbidden width = 0.65m
  opponent_width_with_margin_ = vehicle_width_ + 2.0 * SAFETY_MARGIN;  // W_car + 2*W_margin
  
  // 차량 넓이 기반으로 최소 추월 간격 계산 (자차 + 상대 차량 + 안전 마진)
  // 약 0.6~0.7m 폭의 구간이 "주행 불가 영역"으로 설정됨
  min_overtake_clearance_ = vehicle_width_ * 2.0 + SAFETY_MARGIN;  // 두 차량 넓이 + 안전마진
  RCLCPP_INFO(this->get_logger(), 
              "Vehicle: %.2fm x %.2fm, overtake_clearance: %.2fm, D_forbidden: %.2fm (lateral exclusion zone)",
              vehicle_width_, vehicle_length_, min_overtake_clearance_, opponent_width_with_margin_);
  RCLCPP_INFO(this->get_logger(), "CRSM enabled: reverse_steering_mode=%d (0=neutral, 1=invert, 2=maintain)",
              reverse_steering_mode_);
  RCLCPP_INFO(this->get_logger(), "ACC enabled: Kp=%.2f, Kd=%.2f, target_gap=%.2fm",
              acc_kp_, acc_kd_, target_follow_gap_);
  
  // 벽 데이터 로드
  std::string wall_data_file = get_parameter("wall_data_file").as_string();
  if (!wall_data_file.empty()) {
    if (load_wall_data(wall_data_file)) {
      RCLCPP_INFO(this->get_logger(), "Wall data loaded: %zu points", wall_data_.size());
    }
  }
  
  // ROS Time 변수들 초기화
  prev_time_ = this->get_clock()->now();
  reverse_start_time_ = this->get_clock()->now();
  pause_start_time_ = this->get_clock()->now();
  planning_start_time_ = this->get_clock()->now();
  overtake_start_time_ros_ = this->get_clock()->now();
  overtake_end_time_ros_ = this->get_clock()->now();
  prev_opponent_time_ = this->get_clock()->now();  // ACC용 시간 초기화

  // Path Subscription - Use transient_local QoS to receive latched path from raceline_server
  rclcpp::QoS path_qos(rclcpp::QoS(10).transient_local().reliable());
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      path_qos,
      std::bind(&SimpleController::path_callback, this, std::placeholders::_1));

  // Odom Subscription (Best Effort mode for simulator compatibility)
  rclcpp::QoS odom_qos(rclcpp::QoS(10).best_effort());
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      odom_qos,
      std::bind(&SimpleController::odom_callback, this, std::placeholders::_1));

  // Scan Subscription for collision detection
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SimpleController::scan_callback, this, std::placeholders::_1));

  // IMU Subscription for crash detection (CRSM)
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SimpleController::imu_callback, this, std::placeholders::_1));

  // Drive Publisher
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_,
      10);
  
  // Lookahead point visualization marker publisher (빨간 점으로 표시)
  lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/pid_lookahead_point",
      rclcpp::QoS(1));
  
  // 추월 경로 시각화 publisher (녹색 라인)
  overtake_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/overtake_path",
      rclcpp::QoS(1));
  
  // 벽 충돌 시각화 publisher (충돌시 빨간색)
  wall_collision_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/wall_collision_indicator",
      rclcpp::QoS(1));
  
  // CRSM 상태 퍼블리셔 (텍스트로 상태 표시)
  state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/crsm_state",
      rclcpp::QoS(1));
  
  // 안전 구역 시각화 (상대 차량 금지 구역)
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

/**
 * @brief IMU 콜백 - 충돌 감지를 위한 가속도 데이터 수집
 * 
 * CRSM 보고서 기반: 가속도 크기가 임계값(9.5 m/s^2)을 초과하면 충돌로 판정
 * |a_total| = sqrt(a_x^2 + a_y^2) > A_thresh
 */
void SimpleController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_imu_ = *msg;
  
  // 가속도 크기 계산
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  imu_accel_magnitude_ = std::sqrt(ax * ax + ay * ay);
  
  if (!imu_received_) {
    RCLCPP_INFO(this->get_logger(), "Received first IMU message. Accel: (%.2f, %.2f) m/s^2",
                ax, ay);
    imu_received_ = true;
  }
}

/**
 * @brief 장애물 거리 업데이트
 * 센서 데이터에서 전방, 좌측, 우측 장애물까지의 최소 거리를 계산
 */
void SimpleController::update_obstacle_distances()
{
  if (!scan_received_ || current_scan_.ranges.empty()) {
    last_obstacle_front_dist_ = 10.0;
    last_obstacle_left_dist_ = 10.0;
    last_obstacle_right_dist_ = 10.0;
    last_obstacle_angle_ = 0.0;
    return;
  }
  
  constexpr double FRONT_SECTOR_HALF_ANGLE = M_PI / 3.0;   // 60도
  constexpr double SIDE_INNER = M_PI / 6.0;               // 30도
  constexpr double SIDE_OUTER = M_PI / 2.0;               // 90도
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
    
    // 전방 섹터 (-60° ~ +60°)
    if (angle >= -FRONT_SECTOR_HALF_ANGLE && angle <= FRONT_SECTOR_HALF_ANGLE) {
      if (range < min_front) {
        min_front = range;
        front_angle = angle;
      }
    }
    
    // 좌측 섹터 (30° ~ 90°)
    if (angle >= SIDE_INNER && angle <= SIDE_OUTER) {
      if (range < min_left) {
        min_left = range;
      }
    }
    
    // 우측 섹터 (-90° ~ -30°)
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

/**
 * @brief A1 범위 체크 - 후진이 필요한지 확인
 * A1 범위: 좁은 범위, 이 안에 장애물이 있으면 후진 필요
 */
bool SimpleController::check_a1_zone()
{
  if (!enable_collision_avoidance_ || !scan_received_) {
    is_in_a1_zone_ = false;
    return false;
  }
  
  // 장애물 거리는 control_loop에서 미리 업데이트됨
  double side_threshold = a1_threshold_ * a1_side_factor_;
  
  // A1 범위 내에 장애물이 있는지 확인
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

/**
 * @brief A2 범위 체크 - 조향 회피가 필요한지 확인
 * A2 범위: A1보다 넓은 범위, 이 안에 장애물이 있으면 (A1 밖) 조향으로 회피
 * 주의: check_a1_zone()이 true면 이 함수는 호출하지 않아야 함
 */
bool SimpleController::check_a2_zone()
{
  if (!enable_collision_avoidance_ || !scan_received_) {
    return false;
  }
  
  // A1 범위 체크는 호출자에서 미리 수행되어야 함
  // 여기서는 A2 범위만 체크
  
  // A2 범위 내에 장애물이 있는지 확인
  bool in_a2 = (last_obstacle_front_dist_ < a2_threshold_) ||
               (last_obstacle_left_dist_ < a2_threshold_) ||
               (last_obstacle_right_dist_ < a2_threshold_);
  
  return in_a2;
}

/**
 * @brief A1 범위에서 후진 시 조향각 계산
 * 원래 가던 방향의 반대 방향으로 조향
 */
double SimpleController::compute_reverse_steering()
{
  double reverse_steer = 0.0;
  
  // 좌우 장애물 거리 비교
  double left_dist = std::min(last_obstacle_left_dist_, 5.0);
  double right_dist = std::min(last_obstacle_right_dist_, 5.0);
  
  // 장애물이 있는 방향의 반대로 조향
  // 왼쪽에 장애물이 가까우면 -> 오른쪽으로 조향 (음수)
  // 오른쪽에 장애물이 가까우면 -> 왼쪽으로 조향 (양수)
  double dist_diff = right_dist - left_dist;
  
  if (std::abs(dist_diff) > 0.05) {
    // dist_diff > 0: 오른쪽이 더 멀다 -> 후진하면서 차가 오른쪽으로 가도록
    reverse_steer = -std::copysign(1.0, dist_diff) * max_steer_angle_ * a1_steer_gain_;
  } else if (std::abs(last_steering_before_reverse_) > 0.05) {
    // 충돌 전 조향 방향의 반대로 조향
    reverse_steer = -last_steering_before_reverse_ * a1_steer_gain_;
  } else if (std::abs(last_obstacle_angle_) > 0.1) {
    // 장애물이 왼쪽(양수)이면 오른쪽으로(음수), 오른쪽(음수)이면 왼쪽으로(양수)
    reverse_steer = -last_obstacle_angle_ * a1_steer_gain_;
  }
  
  // 조향 각도 제한
  reverse_steer = std::clamp(reverse_steer, -max_steer_angle_, max_steer_angle_);
  
  RCLCPP_DEBUG(this->get_logger(), 
               "A1 Reverse steer: %.3f (L:%.2f, R:%.2f, angle:%.2f)",
               reverse_steer, left_dist, right_dist, last_obstacle_angle_);
  
  return reverse_steer;
}

/**
 * @brief lookahead 방향 기준으로 라이다 스캔에서 장애물 사이 중간점(gap center) 각도 계산
 * @param lookahead_angle 차량 프레임 기준 lookahead point 방향 각도 (rad)
 * @return gap center 방향 각도 (rad), 장애물이 없으면 lookahead_angle 반환
 * 
 * 로직:
 * 1. lookahead 방향을 중심으로 좌우 일정 범위 스캔
 * 2. 그 범위 내에서 가장 가까운 장애물의 좌측/우측 경계 찾기
 * 3. 장애물들 사이의 가장 넓은 gap의 중심으로 조향
 */
double SimpleController::find_gap_center_angle(double lookahead_angle)
{
  if (!scan_received_ || current_scan_.ranges.empty()) {
    return lookahead_angle;  // 스캔 없으면 원래 방향 유지
  }
  
  double angle_min = current_scan_.angle_min;
  double angle_inc = current_scan_.angle_increment;
  int num_ranges = current_scan_.ranges.size();
  
  // lookahead 방향 기준 ±60도 범위에서 gap 찾기
  constexpr double SEARCH_HALF_ANGLE = M_PI / 3.0;  // 60도
  constexpr double MIN_VALID_RANGE = 0.05;
  constexpr double GAP_THRESHOLD = 0.5;  // 0.5m 이상 떨어지면 gap으로 인식
  
  double search_min = lookahead_angle - SEARCH_HALF_ANGLE;
  double search_max = lookahead_angle + SEARCH_HALF_ANGLE;
  
  // 각 방향별 장애물 거리 저장 (gap 분석용)
  struct RayInfo {
    double angle;
    double range;
    bool is_obstacle;
  };
  std::vector<RayInfo> rays;
  
  for (int i = 0; i < num_ranges; ++i) {
    double angle = angle_min + i * angle_inc;
    
    // lookahead 방향 중심으로 범위 제한
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
  
  // Gap 찾기: 장애물이 없거나 멀리 있는 구간들 분석
  double best_gap_center = lookahead_angle;
  double best_gap_score = -1.0;  // gap 크기 * 거리 점수
  
  // Use ssize_t for gap_start_idx to allow -1 value
  ssize_t gap_start_idx = -1;
  
  for (size_t i = 0; i < rays.size(); ++i) {
    if (!rays[i].is_obstacle) {
      // Gap 시작 또는 계속
      if (gap_start_idx < 0) {
        gap_start_idx = static_cast<ssize_t>(i);
      }
    } else {
      // Gap 끝 - 평가
      if (gap_start_idx >= 0) {
        size_t gap_end_idx = i - 1;
        size_t gap_start = static_cast<size_t>(gap_start_idx);
        double gap_start_angle = rays[gap_start].angle;
        double gap_end_angle = rays[gap_end_idx].angle;
        double gap_width = gap_end_angle - gap_start_angle;
        double gap_center = (gap_start_angle + gap_end_angle) / 2.0;
        
        // Gap 내 평균 거리
        double avg_range = 0.0;
        size_t count = 0;
        for (size_t j = gap_start; j <= gap_end_idx; ++j) {
          avg_range += rays[j].range;
          count++;
        }
        avg_range = count > 0 ? avg_range / static_cast<double>(count) : 0.0;
        
        // 점수: gap 폭 * 거리 * lookahead 방향과의 일치도
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
  
  // 마지막 gap 처리
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
  
  // Gap이 너무 작으면 원래 방향 유지
  if (best_gap_score < 0.1) {
    return lookahead_angle;
  }
  
  return best_gap_center;
}

/**
 * @brief A2 범위에서 회피 조향각 계산 - 완전 반대 방향으로 조향
 * @param lookahead_angle 차량 프레임 기준 lookahead point 방향 각도 (rad)
 * 
 * 새로운 로직:
 * 1. 장애물이 왼쪽에 있으면 -> 오른쪽으로 완전 반대 조향
 * 2. 장애물이 오른쪽에 있으면 -> 왼쪽으로 완전 반대 조향
 * 3. 느려지지 않고 반대 조향으로 회피하며 주행 계속
 */
double SimpleController::compute_avoidance_steering(double lookahead_angle)
{
  (void)lookahead_angle;  // 더 이상 gap following 사용 안함
  
  double avoidance_steer = 0.0;
  
  // 장애물 거리 정보
  double left_dist = std::min(last_obstacle_left_dist_, 5.0);
  double right_dist = std::min(last_obstacle_right_dist_, 5.0);
  double front_dist = std::min(last_obstacle_front_dist_, 5.0);
  double min_side_dist = std::min(left_dist, right_dist);
  double min_dist = std::min(min_side_dist, front_dist);
  
  // 장애물이 충분히 멀면 회피 불필요
  if (min_dist > a2_threshold_) {
    return 0.0;
  }
  
  // 긴급 모드 판단 (a2_urgent_threshold 이내)
  bool is_urgent = min_dist < a2_urgent_threshold_;
  
  // 장애물 방향 반대로 완전 조향
  // 왼쪽에 장애물이 더 가까우면 -> 오른쪽으로 조향 (음수)
  // 오른쪽에 장애물이 더 가까우면 -> 왼쪽으로 조향 (양수)
  double steer_direction = 0.0;
  if (left_dist < right_dist) {
    // 왼쪽 장애물 -> 오른쪽으로 회피 (음수 조향)
    steer_direction = -1.0;
  } else if (right_dist < left_dist) {
    // 오른쪽 장애물 -> 왼쪽으로 회피 (양수 조향)
    steer_direction = 1.0;
  } else if (front_dist < a2_threshold_) {
    // 전방 장애물이면 더 넓은 쪽으로
    steer_direction = (left_dist > right_dist) ? 1.0 : -1.0;
  }
  
  // 긴급도에 따른 조향 강도 결정
  double steer_ratio = is_urgent ? a2_urgent_steer_ratio_ : a2_max_steer_ratio_;
  double steer_gain = is_urgent ? a2_urgent_steer_gain_ : a2_steer_gain_;
  
  // 거리에 반비례하여 조향 강도 증가 (가까울수록 더 강하게)
  double distance_factor = 1.0 - (min_dist / a2_threshold_);
  distance_factor = std::clamp(distance_factor, 0.3, 1.0);
  
  // 최종 회피 조향각 계산 - 완전 반대 방향으로
  avoidance_steer = steer_direction * max_steer_angle_ * steer_ratio * steer_gain * distance_factor;
  
  // 최대 조향 각도 제한
  avoidance_steer = std::clamp(avoidance_steer, -max_steer_angle_, max_steer_angle_);
  
  // 로그 출력
  int throttle_ms = is_urgent ? 200 : 500;
  if (std::abs(avoidance_steer) > 0.01) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ms,
                 "[%s OPPOSITE] steer=%.3f (dir=%.0f) | dist: L%.2f R%.2f F%.2f",
                 is_urgent ? "URGENT" : "AVOID",
                 avoidance_steer, steer_direction,
                 left_dist, right_dist, front_dist);
  }
  
  return avoidance_steer;
}

/**
 * @brief 벽 반발 조향 계산 (수식 기반)
 * 
 * 벽에 가까워지면 반발력 공식으로 조향각 계산:
 *   steer = gain * (1/d - 1/d_max)  where d < d_max
 * 
 * 속도 감소 없이 조향만으로 벽에서 멀어지도록 함
 */
double SimpleController::compute_wall_repulsion_steering()
{
  // 최소 유효 거리 (센서 노이즈 방지)
  constexpr double MIN_VALID_REPULSION_DIST = 0.03;
  
  // 측면 장애물 거리 (벽)
  double left_dist = std::min(last_obstacle_left_dist_, 10.0);
  double right_dist = std::min(last_obstacle_right_dist_, 10.0);
  
  double repulsion_steer = 0.0;
  
  // 왼쪽 벽이 가까우면 -> 오른쪽으로 조향 (음수)
  if (left_dist < wall_repulsion_threshold_ && left_dist > MIN_VALID_REPULSION_DIST) {
    // 반발력 공식: F = k * (1/d - 1/d_max)
    double inv_d = 1.0 / left_dist;
    double inv_d_max = 1.0 / wall_repulsion_threshold_;
    double repulsion_force = wall_repulsion_gain_ * (inv_d - inv_d_max);
    repulsion_steer -= repulsion_force;  // 오른쪽으로 (음수)
  }
  
  // 오른쪽 벽이 가까우면 -> 왼쪽으로 조향 (양수)
  if (right_dist < wall_repulsion_threshold_ && right_dist > MIN_VALID_REPULSION_DIST) {
    double inv_d = 1.0 / right_dist;
    double inv_d_max = 1.0 / wall_repulsion_threshold_;
    double repulsion_force = wall_repulsion_gain_ * (inv_d - inv_d_max);
    repulsion_steer += repulsion_force;  // 왼쪽으로 (양수)
  }
  
  // 최대 반발 조향각 제한
  repulsion_steer = std::clamp(repulsion_steer, -wall_repulsion_max_steer_, wall_repulsion_max_steer_);
  
  // 로그 출력 (활성화시)
  if (std::abs(repulsion_steer) > 0.01) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "Wall repulsion: steer=%.3f, L=%.2fm, R=%.2fm",
                          repulsion_steer, left_dist, right_dist);
  }
  
  return repulsion_steer;
}

/**
 * @brief 상대 차량 Following 속도 계산 (ACC 스타일 PD 제어)
 * 
 * 전방에 상대 차량이 있을 때:
 * 1. 추월 불가능: ACC PD 제어로 1.5m 간격 유지 (보고서 기반)
 * 2. 추월 가능: 추월 시스템 활성화
 * 3. Following 중에도 계속 추월 경로를 체크하여 가능해지면 즉시 추월
 * 
 * ACC 제어 법칙: v_cmd = v_opp + Kp*(gap - 1.5) + Kd*(v_opp - v_ego)
 */
double SimpleController::compute_opponent_following_speed(double base_speed)
{
  double front_dist = last_obstacle_front_dist_;
  
  // 전방 장애물이 following 거리 (1.5m) 이내이고, A1 zone 아닌 경우
  if (front_dist < follow_distance_threshold_ && front_dist > a1_threshold_) {
    // 추월 가능 여부 확인 - Following 중에도 매 제어 주기마다 체크!
    // 보고서: "별도의 재시도 타이머 없이 기하학적 조건이 만족되는 즉시 반응"
    bool can_overtake = can_overtake_safely(front_dist, last_obstacle_angle_);
    
    if (!can_overtake || !has_valid_overtake_plan_) {
      // 추월 불가 (유효한 추월 경로가 없음) -> Following 모드
      // Note: Following 상태는 is_following_opponent_ 플래그로 별도 추적
      // CRSM 상태는 충돌 복구용이므로 Following과 독립적
      if (!is_following_opponent_) {
        is_following_opponent_ = true;
        RCLCPP_INFO(this->get_logger(), 
                    "FOLLOWING START (ACC): No safe overtake path. front=%.2fm, target_gap=%.2fm",
                    front_dist, target_follow_gap_);
      }
      
      // === ACC 스타일 PD 제어 적용 (보고서 기반) ===
      double acc_speed = compute_acc_following_speed(base_speed);
      
      // 안전 구역 시각화 (상대 차량 금지 구역 표시)
      publish_safety_zone_markers();
      
      // 주기적으로 상태 로그 출력 (추월 경로 계속 탐색 중임을 표시)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "FOLLOWING (ACC): gap=%.2fm, target=%.2fm, speed=%.2f [checking overtake...]",
                            front_dist, target_follow_gap_, acc_speed);
      
      return acc_speed;
    } else {
      // 추월 가능! 유효한 추월 경로가 생성됨 -> 추월 시작 준비
      // 보고서: "다음 제어 주기에서 valid_overtake_paths가 비어있지 않게 되므로 즉시 추월 모드로 전환"
      if (is_following_opponent_) {
        is_following_opponent_ = false;
        RCLCPP_WARN(this->get_logger(), 
                    "OVERTAKE OPPORTUNITY! Valid path found while FOLLOWING. Direction: %s",
                    planned_overtake_direction_ > 0 ? "LEFT" : "RIGHT");
      }
      // 추월 시작은 check_and_start_overtake()에서 처리됨
      // 추월 경로 시각화 (경로가 유효할 때만)
      publish_overtake_path(planned_overtake_direction_);
    }
  } else {
    // Following 범위 밖 (전방이 clear하거나 A1 zone 내부)
    if (is_following_opponent_) {
      is_following_opponent_ = false;
      RCLCPP_INFO(this->get_logger(), "Clear path, exiting FOLLOWING mode");
    }
  }
  
  return base_speed;
}

/**
 * @brief 속도 변화 스무딩 (급격한 속도 변화 방지)
 * 
 * 추월 시가 아닐 때 속도 변화를 부드럽게 만들어 안정적인 주행 제공
 */
double SimpleController::smooth_speed_change(double target_speed, double current_adjusted_speed)
{
  if (is_overtaking_ || is_post_overtake_) {
    // 추월 중이면 스무딩 없이 빠르게 속도 변경
    return target_speed;
  }
  
  // 속도 변화 제한 (speed_smooth_factor_가 변화율을 결정)
  // max_speed_change = max_speed_ * factor (예: 2.0 * 0.1 = 0.2 m/s per cycle)
  double speed_diff = target_speed - current_adjusted_speed;
  double max_speed_change = speed_smooth_factor_ * max_speed_;
  
  if (std::abs(speed_diff) > max_speed_change) {
    // 급격한 속도 변화 방지 - clamp로 간결하게 처리
    return std::clamp(target_speed, 
                      current_adjusted_speed - max_speed_change, 
                      current_adjusted_speed + max_speed_change);
  }
  
  return target_speed;
}

/**
 * @brief 추월 시작 조건 체크 및 시작
 * 
 * 추월 가능한 경로가 있으면 추월 시작
 * 추월 불가능하면 Following 유지
 */
void SimpleController::check_and_start_overtake()
{
  if (is_overtaking_ || is_post_overtake_) {
    return;  // 이미 추월 중
  }
  
  // 후진/정지/계획 상태에서는 추월 시작 불가
  if (is_reversing_ || is_pausing_after_reverse_ || is_planning_after_pause_) {
    return;
  }
  
  double front_dist = last_obstacle_front_dist_;
  
  // 추월 조건: 전방에 장애물이 있고, 추월 가능한 경로가 있음
  // Following 거리 (1.5m) 이내에 장애물이 있어야 추월 시도
  if (front_dist < follow_distance_threshold_ && front_dist > a1_threshold_) {
    // can_overtake_safely가 성공하면 planned_overtake_direction_에 검증된 방향이 저장됨
    // 이 함수 내에서 추월 경로 시뮬레이션 및 검증이 수행됨
    bool can_overtake = can_overtake_safely(front_dist, last_obstacle_angle_);
    
    // 핵심: has_valid_overtake_plan_이 true일 때만 추월 시작!
    // 경로가 생성되지 않으면 추월 불가
    if (can_overtake && has_valid_overtake_plan_) {
      is_overtaking_ = true;
      is_following_opponent_ = false;
      overtake_start_time_ros_ = this->get_clock()->now();
      
      // 검증된 추월 방향 사용
      double overtake_direction = planned_overtake_direction_;
      
      RCLCPP_WARN(this->get_logger(), 
                  "OVERTAKE START! VALIDATED path (vehicle_width=%.2fm). front=%.2fm, direction=%s, L=%.2fm, R=%.2fm",
                  vehicle_width_, front_dist, 
                  overtake_direction > 0 ? "LEFT" : "RIGHT",
                  last_obstacle_left_dist_, last_obstacle_right_dist_);
      
      // 추월 경로 시각화 (검증된 경로만 표시)
      publish_overtake_path(overtake_direction);
    }
    // else: 추월 불가 -> Following 유지 (compute_opponent_following_speed에서 처리)
  }
}

/**
 * @brief 추월 상태 업데이트
 * 
 * 추월 완료 조건 체크 및 상태 전환
 * 추월 중 충돌 감지 시 추월 중단 및 후진
 */
void SimpleController::update_overtake_state()
{
  rclcpp::Time current_time = this->get_clock()->now();
  
  if (is_overtaking_) {
    // 추월 중 충돌 감지 시 추월 중단
    if (is_in_a1_zone_) {
      is_overtaking_ = false;
      is_post_overtake_ = false;
      has_valid_overtake_plan_ = false;
      planned_overtake_direction_ = 0.0;  // 추월 방향도 초기화
      RCLCPP_WARN(this->get_logger(), "OVERTAKE ABORTED: Collision detected! Initiating reverse...");
      // 후진 로직은 control_loop의 A1 zone 처리에서 자동으로 수행됨
      return;
    }
    
    // 추월 완료 또는 타임아웃 체크
    if (should_return_to_line()) {
      is_overtaking_ = false;
      is_post_overtake_ = true;
      overtake_end_time_ros_ = current_time;
      RCLCPP_INFO(this->get_logger(), "OVERTAKE COMPLETE, entering high-speed phase");
    }
  }
  
  if (is_post_overtake_) {
    // 추월 후 고속 유지 시간 체크
    double elapsed = (current_time - overtake_end_time_ros_).seconds();
    if (elapsed > post_overtake_duration_) {
      is_post_overtake_ = false;
      RCLCPP_INFO(this->get_logger(), "Post-overtake phase ended, returning to normal speed");
    }
  }
}

/**
 * @brief 추월 시 속도 계산
 * 
 * 추월 중: 속도 크게 증가 (overtake_speed_boost_)
 * 추월 직후: 높은 속도 유지 (post_overtake_speed_factor_)
 */
double SimpleController::compute_overtake_speed(double base_speed)
{
  if (is_overtaking_) {
    // 추월 중: 속도 크게 증가
    double overtake_speed = base_speed * overtake_speed_boost_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "OVERTAKING: speed boost %.2f -> %.2f (%.1fx)",
                          base_speed, overtake_speed, overtake_speed_boost_);
    return overtake_speed;
  }
  
  if (is_post_overtake_) {
    // 추월 직후: 높은 속도 유지
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

  // Check for collision and handle reverse mode
  rclcpp::Time current_time = this->get_clock()->now();
  
  // === A1/A2 범위 기반 충돌 회피 ===
  // 먼저 장애물 거리 업데이트
  update_obstacle_distances();
  
  // === 경로 계획 상태 (정지 후 다음 행동 계획) ===
  if (is_planning_after_pause_) {
    double planning_elapsed = (current_time - planning_start_time_).seconds();
    if (planning_elapsed < planning_duration_) {
      // 계획 중: 정지 상태 유지하며 추월 경로 재계산
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = current_time;
      drive_msg.header.frame_id = current_odom_.child_frame_id;
      drive_msg.drive.speed = 0.0;
      drive_msg.drive.steering_angle = 0.0;  // 조향 중립
      
      drive_pub_->publish(drive_msg);
      
      static int planning_log_count = 0;
      if (planning_log_count++ % 25 == 0) {
        RCLCPP_INFO(this->get_logger(), "PLANNING: %.2fs/%.2fs (computing next action...)",
                    planning_elapsed, planning_duration_);
      }
      return;
    } else {
      // 계획 완료, 출발 준비
      is_planning_after_pause_ = false;
      just_finished_reverse_ = true;
      reverse_cooldown_counter_ = REVERSE_COOLDOWN_CYCLES;
      RCLCPP_INFO(this->get_logger(), "Planning complete, starting cooldown before allowing another reverse");
    }
  }
  
  // === 후진 후 정지 상태 (생각 시간) ===
  if (is_pausing_after_reverse_) {
    double pause_elapsed = (current_time - pause_start_time_).seconds();
    if (pause_elapsed < reverse_pause_duration_) {
      // 정지 상태 유지
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = current_time;
      drive_msg.header.frame_id = current_odom_.child_frame_id;
      drive_msg.drive.speed = 0.0;
      drive_msg.drive.steering_angle = 0.0;  // 조향 중립으로 리셋
      
      drive_pub_->publish(drive_msg);
      
      static int pause_log_count = 0;
      if (pause_log_count++ % 25 == 0) {
        RCLCPP_INFO(this->get_logger(), "PAUSING: %.2fs/%.2fs (stabilizing...)",
                    pause_elapsed, reverse_pause_duration_);
      }
      return;
    } else {
      // 정지 완료, 경로 계획 상태로 전환
      is_pausing_after_reverse_ = false;
      is_planning_after_pause_ = true;
      planning_start_time_ = current_time;
      // 조향 상태 초기화
      prev_steering_angle_ = 0.0;
      lateral_error_integral_ = 0.0;  // PID 적분 리셋
      RCLCPP_INFO(this->get_logger(), "Pause complete, entering planning phase...");
      return;
    }
  }
  
  // 후진 쿨다운 카운터 감소 (무한 후진 루프 방지)
  if (reverse_cooldown_counter_ > 0) {
    reverse_cooldown_counter_--;
    if (reverse_cooldown_counter_ == 0) {
      just_finished_reverse_ = false;
      RCLCPP_DEBUG(this->get_logger(), "Reverse cooldown complete, reverse allowed again");
    }
  }
  
  // === CRSM: IMU 기반 충돌 감지 추가 ===
  bool imu_crash_detected = check_imu_collision();
  bool stall_detected = check_stall_condition();
  
  // A1 범위 체크: 후진 필요 (쿨다운 중에는 후진하지 않음)
  // CRSM 확장: IMU 충돌 또는 스톨 감지도 후진 트리거
  bool collision_trigger = check_a1_zone() || imu_crash_detected || stall_detected;
  
  if (collision_trigger && !just_finished_reverse_) {
    // Start or continue reversing
    if (!is_reversing_ && !is_pausing_after_reverse_ && !is_planning_after_pause_) {
      is_reversing_ = true;
      crsm_state_ = CRSMState::ST_CRASH_DETECTED;
      reverse_start_time_ = current_time;
      last_steering_before_reverse_ = prev_steering_angle_;  // 충돌 시 조향각 저장
      
      const char* trigger_type = imu_crash_detected ? "IMU CRASH" : 
                                 (stall_detected ? "STALL" : "A1 ZONE");
      RCLCPP_WARN(this->get_logger(), 
                  "CRSM [%s] COLLISION! mode=%d, saved_steer=%.3f", 
                  trigger_type, reverse_steering_mode_, last_steering_before_reverse_);
    }
    
    double elapsed = (current_time - reverse_start_time_).seconds();
    if (elapsed < reverse_duration_) {
      // === CRSM: 조향각 역보정 (Steering Inversion) 적용 ===
      // 보고서 기반: 충돌 시 조향을 중립(0) 또는 반대(-δ_last)로 설정하여 벽에서 탈출
      crsm_state_ = CRSMState::ST_RECOVERY_REVERSE;
      
      double reverse_steer = compute_inverted_reverse_steering();
      
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = current_time;
      drive_msg.header.frame_id = current_odom_.child_frame_id;
      drive_msg.drive.speed = -reverse_speed_;
      drive_msg.drive.steering_angle = reverse_steer;
      
      drive_pub_->publish(drive_msg);
      last_cmd_velocity_ = -reverse_speed_;  // 스톨 감지용
      
      // CRSM 상태 발행
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
      // Reverse complete, 이제 정지 시작 (생각 시간)
      is_reversing_ = false;
      is_pausing_after_reverse_ = true;
      crsm_state_ = CRSMState::ST_RECOVERY_REALIGN;
      pause_start_time_ = current_time;
      RCLCPP_INFO(this->get_logger(), "CRSM Reverse complete, entering REALIGN phase...");
      publish_crsm_state();
      return;
    }
  } else if (check_a1_zone() && just_finished_reverse_) {
    // 쿨다운 중인데 아직 A1 zone - 매우 느리게 전진하며 탈출 시도
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "A1 Zone during cooldown! Slow forward escape (cooldown: %d)",
                          reverse_cooldown_counter_);
    // 일반 주행 로직으로 넘어감 (속도 제한은 아래에서 적용됨)
  } else {
    // Not in A1 zone, stop reversing if we were
    if (is_reversing_) {
      is_reversing_ = false;
      is_pausing_after_reverse_ = true;
      pause_start_time_ = current_time;
      RCLCPP_INFO(this->get_logger(), "Left A1 zone, stopping and stabilizing...");
      return;
    }
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
  
  // Find closest point for error calculations (restricted forward window)
  static int last_closest_idx = 0;
  int closest_idx = find_closest_point_along_path(
      current_x, current_y, current_yaw, current_path_, last_closest_idx);
  last_closest_idx = closest_idx;
  
  // Compute errors for Hybrid Controller
  double lateral_error = compute_lateral_error(
      current_x, current_y, current_yaw, current_path_, closest_idx);
  double heading_error = compute_heading_error(current_yaw, current_path_, closest_idx);
  
  // Adaptive lookahead distance based on speed
  // Range: 0.8 (low speed) ~ 2.0 (high speed)
  double adaptive_lookahead = compute_adaptive_lookahead(current_speed);

  // Find target point with adaptive lookahead (restricted search within 3.0m)
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
  
  // Publish lookahead point visualization (빨간 점)
  publish_lookahead_marker(target_x_global, target_y_global, 0.15);

  // Transform target to vehicle frame
  // Vehicle frame: x forward, y left, z up
  // Global to vehicle: rotate by -current_yaw
  // [x_veh]   [cos(θ)  sin(θ)] [x_global - x_curr]
  // [y_veh] = [-sin(θ) cos(θ)] [y_global - y_curr]
  double dx_global = target_x_global - current_x;
  double dy_global = target_y_global - current_y;
  double cos_neg_yaw = std::cos(-current_yaw);
  double sin_neg_yaw = std::sin(-current_yaw);
  
  double target_x_vehicle = dx_global * cos_neg_yaw - dy_global * sin_neg_yaw;
  double target_y_vehicle = dx_global * sin_neg_yaw + dy_global * cos_neg_yaw;
  
  // Debug: log coordinate transformation
  static int coord_debug_count = 0;
  if (coord_debug_count++ % 50 == 0) {
    RCLCPP_INFO(this->get_logger(), 
                "Coord: global_dx=%.3f, global_dy=%.3f, veh_x=%.3f, veh_y=%.3f, yaw=%.3f",
                dx_global, dy_global, target_x_vehicle, target_y_vehicle, current_yaw);
  }

  // === Hybrid Controller (Slide 13-14) ===
  // current_time already defined at start of control_loop
  double dt = (current_time - prev_time_).seconds();
  if (dt > 0.1) dt = 0.1;
  if (dt <= 0.0) dt = 0.02;
  
  // 1) Pure Pursuit: delta_pp = atan(2 * L * sin(alpha) / l_d)
  double alpha = std::atan2(target_y_vehicle, target_x_vehicle);
  double delta_pp = 0.0;
  if (adaptive_lookahead > 1e-6) {
    double sin_alpha_clipped = std::max(-1.0, std::min(1.0, std::sin(alpha)));
    delta_pp = std::atan2(2.0 * wheelbase_ * sin_alpha_clipped, adaptive_lookahead);
  }

  // 2) PID for lateral (cross-track) error: delta_pid = Kp*e + Ki*∫e + Kd*de/dt
  double delta_pid = compute_pid_control(lateral_error, dt);

  // 3) Curvature feedforward: delta_ff = K_ff * atan(L * kappa)
  double path_curvature = compute_path_curvature(current_path_, target_idx);
  double delta_ff = curvature_feedforward_gain_ * std::atan(wheelbase_ * path_curvature);

  // 4) Stanley-style heading correction: delta_stanley = atan(K_h * e_heading / (v + 1.0))
  double stanley_den = current_speed + 1.0;  // Singularity protection
  double delta_stanley = std::atan(heading_error_gain_ * heading_error / stanley_den);

  // 5) Raw sum: delta_raw = delta_pp + delta_pid + delta_stanley + delta_ff
  double steering_angle = delta_pp + delta_pid + delta_stanley + delta_ff;
  
  // 6) 코너 감지 및 조향각 증폭 - 코너에서 더 큰 각도로 회전 (A2 회피 전에 적용)
  // 경로 곡률이 임계값 이상이면 코너로 판단하고 경로 추종 조향각만 증폭
  bool is_corner = std::abs(path_curvature) > corner_curvature_threshold_;
  
  // === 아웃-인-아웃 (Out-In-Out) 코너링 ===
  double corner_direction = 0.0;
  double out_in_out_steer_offset = 0.0;
  if (enable_out_in_out_ && detect_upcoming_corner(closest_idx, corner_direction)) {
    // 코너 진입 전: 반대 방향으로 살짝 조향 (아웃)
    double offset = compute_out_in_out_offset(closest_idx, corner_direction);
    // 오프셋을 조향각으로 변환 (횡방향 이동을 위한 조향)
    out_in_out_steer_offset = -offset * 0.5;  // gain 0.5
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "OUT-IN-OUT: corner_dir=%.1f, offset=%.3f, steer_offset=%.3f",
                          corner_direction, offset, out_in_out_steer_offset);
  }
  steering_angle += out_in_out_steer_offset;
  
  if (is_corner) {
    // 코너에서 조향각 증폭 (corner_steer_amplify_ 배) - 경로 추종 조향에만 적용
    steering_angle *= corner_steer_amplify_;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "CORNER detected: curvature=%.3f, steer amplified by %.2fx",
                          path_curvature, corner_steer_amplify_);
  }
  
  // 7) A2 범위 기반 회피 조향 - 비활성화됨
  // 측면 센서 A2 zone 기능이 제거됨. 장애물/벽 충돌 시 A1 zone 후진만 동작
  // 2024-12: 사용자 요청에 따라 A2 zone 조향 및 속도 감소 기능 제거
  
  // === 벽 충돌 시각화 ===
  // A1 zone (매우 가까움) 또는 벽 데이터 기반 충돌 감지
  bool is_wall_collision = is_in_a1_zone_ || 
                           last_obstacle_front_dist_ < 0.15 ||
                           last_obstacle_left_dist_ < 0.1 ||
                           last_obstacle_right_dist_ < 0.1;
  publish_wall_collision_indicator(is_wall_collision, current_x, current_y);

  // Clamp to physical steering limits
  steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);

  // Apply steering sign correction (for coordinate system mismatch)
  steering_angle *= steering_sign_;
  
  // Apply steering rate limiting for smooth control
  steering_angle = limit_steering_rate(steering_angle, dt);
  
  // Low-pass filter (LPF): delta_final = alpha * delta + (1 - alpha) * delta_prev
  steering_angle = smooth_steering(steering_angle, prev_steering_angle_);
  
  prev_steering_angle_ = steering_angle;
  prev_time_ = current_time;

  // Debug logging
  static int debug_count = 0;
  if (debug_count++ % 25 == 0) {  // Every 0.5 seconds
    RCLCPP_INFO(this->get_logger(), 
                "Hybrid: lat_err=%.3f, hdg_err=%.3f, PP=%.3f, PID=%.3f, Stanley=%.3f, FF=%.3f, steer=%.3f",
                lateral_error, heading_error, delta_pp, delta_pid, delta_stanley, delta_ff, steering_angle);
  }

  // === 추월 시스템 업데이트 ===
  // 추월 시작 조건 체크
  check_and_start_overtake();
  // 추월 상태 업데이트 (완료 체크)
  update_overtake_state();
  
  // 추월 중이면 추월 경로 시각화 (계속 업데이트)
  if (is_overtaking_) {
    // 검증된 추월 방향 사용 (계획된 경로)
    double overtake_direction;
    if (has_valid_overtake_plan_ && std::abs(planned_overtake_direction_) > 0.5) {
      overtake_direction = planned_overtake_direction_;
    } else {
      // Fallback: 더 넓은 쪽으로
      double left_space = last_obstacle_left_dist_;
      double right_space = last_obstacle_right_dist_;
      overtake_direction = (left_space > right_space) ? 1.0 : -1.0;
    }
    publish_overtake_path(overtake_direction);
    
    // 추월 중 조향각 조정 (검증된 경로 기반)
    steering_angle = compute_overtake_steering(steering_angle);
  }

  // Speed control: Reduce speed based on path curvature (not amplified steering angle)
  // path_curvature 를 직접 사용하여 증폭된 조향각으로 인한 과도한 속도 감소 방지
  double speed_factor = 1.0 / (1.0 + 2.0 * std::abs(path_curvature));
  
  // 코너에서 추가 속도 감소 적용 (추월 중이 아닐 때만)
  if (is_corner && !is_overtaking_) {
    speed_factor *= corner_speed_factor_;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "CORNER speed reduction (not overtaking): factor=%.2f, corner_factor=%.2f",
                          speed_factor, corner_speed_factor_);
  }
  
  double base_adjusted_speed = target_speed_ * speed_factor;
  
  // === 추월 또는 Following에 따른 속도 조절 ===
  double adjusted_speed = base_adjusted_speed;
  
  if (is_overtaking_ || is_post_overtake_) {
    // 추월 중 또는 추월 직후: 속도 증가 (신중하게 하되 빠르게 추월)
    adjusted_speed = compute_overtake_speed(base_adjusted_speed);
  } else {
    // 일반 모드: 전방 장애물에 따른 Following 속도 계산
    adjusted_speed = compute_opponent_following_speed(base_adjusted_speed);
  }
  
  // 속도 스무딩 (급격한 속도 변화 방지) - 추월 시에는 스무딩 안함
  adjusted_speed = smooth_speed_change(adjusted_speed, last_adjusted_speed_);
  last_adjusted_speed_ = adjusted_speed;
  
  adjusted_speed = std::clamp(adjusted_speed, 0.0, max_speed_);
  
  // 후진 쿨다운 중 A1 zone에 있으면 매우 느리게 (탈출 시도)
  if (just_finished_reverse_ && is_in_a1_zone_) {
    adjusted_speed = std::min(adjusted_speed, 0.3);  // 최대 0.3 m/s
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "Cooldown escape mode: limiting speed to %.2f", adjusted_speed);
  }

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = current_time;
  drive_msg.header.frame_id = current_odom_.child_frame_id;
  drive_msg.drive.speed = adjusted_speed;
  drive_msg.drive.steering_angle = steering_angle;

  drive_pub_->publish(drive_msg);
  
  // CRSM: 명령된 속도 저장 (스톨 감지용)
  last_cmd_velocity_ = adjusted_speed;
  
  // CRSM 상태가 정상일 때만 NORMAL로 설정
  if (!is_reversing_ && !is_pausing_after_reverse_ && !is_planning_after_pause_) {
    crsm_state_ = CRSMState::ST_NORMAL;
  }
  
  static int publish_count = 0;
  if (publish_count++ % 50 == 0) {  // Every 1 second at 20ms timer
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
  // Adaptive lookahead: uses lookahead_distance_ as the base value
  // Formula: L = clamp(base + speed_gain * v, L_min, L_max)
  // - lookahead_distance_: base lookahead distance (user-configurable via parameter)
  // - lookahead_speed_gain_: how much lookahead increases with speed
  // - Low speed: lookahead close to lookahead_distance_ for precise cornering
  // - High speed: lookahead increases for smooth stability
  double base_lookahead = std::max(min_lookahead_, lookahead_distance_);
  double adaptive = base_lookahead + lookahead_speed_gain_ * speed;
  double result = std::max(min_lookahead_, std::min(max_lookahead_, adaptive));
  
  // Log lookahead changes for debugging using ROS2 throttled logging (~1 second interval)
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
  
  // start_idx 범위 보정
  int path_size = static_cast<int>(path.poses.size());
  start_idx = std::clamp(start_idx, 0, path_size - 1);
  
  // Restricted search window: limit to previous few points and next 20 points
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
  
  return closest_idx;
}

double SimpleController::smooth_steering(double new_steering, double prev_steering)
{
  // Exponential moving average filter
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
  
  // 빨간 점으로 표시 (크기 0.3m)
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // 200ms lifetime for stable visualization
  
  lookahead_marker_pub_->publish(marker);
}

// ============================================================================
// 새로운 기능들: 벽 인식, 아웃-인-아웃, 추월, 시각화
// ============================================================================

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
  
  // 헤더 스킵
  std::getline(file, line);
  line_num++;
  
  while (std::getline(file, line)) {
    line_num++;
    if (line.empty()) continue;
    
    std::stringstream ss(line);
    WallPoint point;
    char comma;
    
    // CSV 파싱 with validation
    if (ss >> point.x >> comma >> point.y >> comma >> point.d_left >> comma >> point.d_right >> comma >> point.psi) {
      // 값 범위 검증
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
  
  // 지역 검색: 마지막 검색 위치 근처에서 먼저 검색 (O(1) 평균)
  size_t search_range = 50;  // 주변 50개 포인트 검색
  size_t start_idx = (last_wall_search_idx_ > search_range) ? 
                      last_wall_search_idx_ - search_range : 0;
  size_t end_idx = std::min(last_wall_search_idx_ + search_range, wall_data_.size());
  
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = last_wall_search_idx_;
  
  // 지역 검색
  for (size_t i = start_idx; i < end_idx; ++i) {
    double dx = wall_data_[i].x - x;
    double dy = wall_data_[i].y - y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }
  
  // 지역 검색에서 너무 멀면 전체 검색
  if (min_dist_sq > 4.0) {  // 2m 이상 떨어져 있으면
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
    return 10.0;  // 기본값: 충분히 멀다고 가정
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
  
  // 인덱스 범위 체크
  if (current_idx >= static_cast<int>(current_path_.poses.size())) {
    return false;
  }
  
  // 전방 몇 포인트를 확인하여 코너 감지
  int look_ahead_points = static_cast<int>(corner_approach_distance_ / 0.1);  // 0.1m spacing 가정
  int remaining_points = static_cast<int>(current_path_.poses.size()) - current_idx - 2;
  if (remaining_points < 3) {
    return false;
  }
  look_ahead_points = std::min(look_ahead_points, remaining_points);
  // look_ahead_points는 remaining_points 이하이므로 별도 체크 불필요
  
  // 곡률 계산
  double max_curvature = 0.0;
  double curvature_sign = 0.0;
  
  for (int i = current_idx; i < current_idx + look_ahead_points - 1; ++i) {
    double curvature = compute_path_curvature(current_path_, i);
    if (std::abs(curvature) > std::abs(max_curvature)) {
      max_curvature = curvature;
      curvature_sign = (curvature > 0) ? 1.0 : -1.0;
    }
  }
  
  // 코너 감지 (곡률이 임계값 이상)
  if (std::abs(max_curvature) > corner_curvature_threshold_) {
    corner_direction = curvature_sign;  // 양수: 왼쪽, 음수: 오른쪽
    return true;
  }
  
  return false;
}

double SimpleController::compute_out_in_out_offset(int current_idx, double corner_direction)
{
  if (!enable_out_in_out_) {
    return 0.0;
  }
  
  // 코너 방향의 반대쪽으로 오프셋 (아웃-인-아웃)
  // corner_direction > 0: 왼쪽 코너 -> 오른쪽으로 아웃 (음수 오프셋)
  // corner_direction < 0: 오른쪽 코너 -> 왼쪽으로 아웃 (양수 오프셋)
  double offset = -corner_direction * out_in_out_offset_;
  
  // 벽 데이터가 있으면 벽까지 거리 확인
  if (wall_data_loaded_) {
    double current_x = current_odom_.pose.pose.position.x;
    double current_y = current_odom_.pose.pose.position.y;
    
    double wall_left = get_wall_distance_left(current_x, current_y);
    double wall_right = get_wall_distance_right(current_x, current_y);
    
    // 벽에 너무 가까워지지 않도록 오프셋 제한
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
  // 추월 계획 초기화 (중요: 경로가 생성되지 않으면 추월 불가!)
  has_valid_overtake_plan_ = false;
  planned_overtake_direction_ = 0.0;
  
  if (!enable_collision_avoidance_) {
    return false;
  }
  
  // 경로가 비어있으면 추월 불가
  if (current_path_.poses.empty()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "NO OVERTAKE: No path available");
    return false;
  }
  
  // 상대방이 일정 거리 이내이고, 전방에 있는 경우
  // 더 엄격한 전방 체크: ±30도 이내
  bool opponent_in_front = std::abs(opponent_angle) < M_PI / 6.0;  // ±30도 이내
  // Following 거리 (1.5m) 내에서만 추월 시도, 최소 간격 이상에서만
  bool opponent_close = opponent_dist < follow_distance_threshold_ && opponent_dist > MIN_OVERTAKE_DISTANCE;
  
  if (!opponent_in_front || !opponent_close) {
    return false;
  }
  
  // 추월 가능한 간격이 있는지 확인
  double left_space = last_obstacle_left_dist_;
  double right_space = last_obstacle_right_dist_;
  double total_space = left_space + right_space;
  
  // === 차량 치수를 고려한 최소 간격 계산 ===
  // 자차 넓이(vehicle_width_) + 상대 차량 넓이(동일 가정) + 안전 마진
  double required_clearance = min_overtake_clearance_;  // 생성자에서 계산됨
  
  // === 추월 금지 조건 체크 (1단계: 기본 조건) ===
  // 1. 좁은 도로 체크 (좌우 공간 합계가 threshold 미만이면 추월 금지)
  if (total_space < narrow_road_threshold_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Narrow road (L:%.2f + R:%.2f = %.2f < %.2f)",
                          left_space, right_space, total_space, narrow_road_threshold_);
    return false;
  }
  
  // 2. 시야/각도 체크 (상대방 각도가 일정 이상이면 시야 확보 불가로 판단)
  if (std::abs(opponent_angle) > min_visibility_angle_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Poor visibility (angle: %.2f > %.2f)",
                          std::abs(opponent_angle), min_visibility_angle_);
    return false;
  }
  
  // 3. 코너 진입 중이면 추월 금지
  double corner_direction = 0.0;
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  // 현재 위치에서 가장 가까운 경로 인덱스 찾기
  int closest_idx = find_closest_point_along_path(current_x, current_y, current_yaw, 
                                                   current_path_, last_closest_idx_overtake_);
  last_closest_idx_overtake_ = closest_idx;
  
  if (detect_upcoming_corner(closest_idx, corner_direction)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Corner detected ahead (dir: %.1f)", corner_direction);
    return false;
  }
  
  // === 추월 방향 결정 및 경로 검증 (2단계: 경로 시뮬레이션) ===
  // 차량 넓이를 고려한 최소 간격 체크
  bool can_overtake_left = left_space > (required_clearance + SAFETY_MARGIN);
  bool can_overtake_right = right_space > (required_clearance + SAFETY_MARGIN);
  
  // 벽 데이터가 있으면 추가 확인
  if (wall_data_loaded_) {
    double wall_left = get_wall_distance_left(current_x, current_y);
    double wall_right = get_wall_distance_right(current_x, current_y);
    
    // 추월 시 필요한 횡방향 오프셋 + 차량 넓이 절반 + 안전 마진
    double required_wall_clearance = overtake_lateral_offset_ + (vehicle_width_ / 2.0) + SAFETY_MARGIN;
    
    can_overtake_left = can_overtake_left && (wall_left > required_wall_clearance);
    can_overtake_right = can_overtake_right && (wall_right > required_wall_clearance);
  }
  
  // 초기 공간 체크로 추월 불가능하면 종료
  if (!can_overtake_left && !can_overtake_right) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "NO OVERTAKE: Insufficient gap (L:%.2f, R:%.2f) for vehicle width %.2fm",
                          left_space, right_space, vehicle_width_);
    return false;
  }
  
  // === 3단계: 추월 경로 시뮬레이션 및 검증 (핵심!) ===
  // 경로가 만들어지지 않으면 추월 금지
  
  // 더 넓은 쪽을 우선 시도, 그 다음 다른 쪽 시도
  double preferred_direction = (left_space > right_space) ? 1.0 : -1.0;
  double alternate_direction = -preferred_direction;
  
  bool preferred_valid = false;
  bool alternate_valid = false;
  
  // 우선 방향 경로 검증
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
  
  // 우선 방향이 안전하지 않으면 대안 방향 시도
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
  
  // 모든 경로가 안전하지 않음
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
  
  // 검증된 추월 방향 사용 (can_overtake_safely에서 계획됨)
  // 유효한 계획이 없으면 현재 공간 기반으로 판단 (fallback)
  double overtake_direction;
  if (has_valid_overtake_plan_ && std::abs(planned_overtake_direction_) > 0.5) {
    overtake_direction = planned_overtake_direction_;
  } else {
    // Fallback: 더 넓은 쪽으로
    double left_space = last_obstacle_left_dist_;
    double right_space = last_obstacle_right_dist_;
    overtake_direction = (left_space > right_space) ? 1.0 : -1.0;
  }
  
  // 추월 진행 시간에 따른 조향 강도 조절 (초반에 강하게, 후반에 약하게)
  rclcpp::Time current_time = this->get_clock()->now();
  double elapsed = (current_time - overtake_start_time_ros_).seconds();
  double overtake_progress = std::min(1.0, elapsed / overtake_max_duration_);
  
  // 추월 조향 강도: 초반에 강하게 (overtake_steer_max_), 후반에 decay만큼 유지
  // 예: 0.25rad에서 시작, 완료시점에 0.25 * (1 - 0.6) = 0.1rad
  double steer_intensity = overtake_steer_max_ * (1.0 - overtake_progress * overtake_steer_decay_);
  
  // 벽까지 거리에 따른 안전 제어
  double wall_dist = (overtake_direction > 0) ? last_obstacle_left_dist_ : last_obstacle_right_dist_;
  if (wall_dist < overtake_wall_caution_dist_) {
    // 벽에 너무 가까우면 조향 강도 줄임 (신중한 행동)
    steer_intensity *= (wall_dist / overtake_wall_caution_dist_);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                          "OVERTAKE CAUTION: wall nearby %.2fm, reducing steer intensity",
                          wall_dist);
  }
  
  // 추월 조향각 = 기본 조향 + 추월 오프셋
  double overtake_steer = base_steering + overtake_direction * steer_intensity;
  
  // 최대 조향각 제한 (안전을 위해 80%로 제한)
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
  
  // 최대 추월 시간 초과
  if (elapsed > overtake_max_duration_) {
    RCLCPP_INFO(this->get_logger(), "Overtake timeout, returning to line");
    return true;
  }
  
  // 전방에 장애물 없음 (추월 완료)
  if (last_obstacle_front_dist_ > return_to_line_distance_) {
    RCLCPP_INFO(this->get_logger(), "Overtake complete, returning to line");
    return true;
  }
  
  return false;
}

double SimpleController::compute_return_to_line_steering()
{
  // 레이싱 라인으로 부드럽게 복귀
  // 현재 횡방향 오차를 기반으로 복귀 조향 계산
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  // 경로상 가장 가까운 점 찾기
  static int last_closest_idx = 0;
  int closest_idx = find_closest_point_along_path(current_x, current_y, current_yaw, 
                                                   current_path_, last_closest_idx);
  last_closest_idx = closest_idx;
  
  // 횡방향 오차 계산
  double lateral_error = compute_lateral_error(current_x, current_y, current_yaw, 
                                                current_path_, closest_idx);
  
  // 복귀 조향 = 횡방향 오차에 비례하는 조향 (부드럽게)
  double return_steer = -lateral_error * 0.5;  // gain 0.5
  
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
  
  // 유효한 추월 계획이 있을 때만 경로 표시
  if (!has_valid_overtake_plan_) {
    // 추월 경로가 없으면 마커 삭제
    marker.action = visualization_msgs::msg::Marker::DELETE;
    overtake_path_marker_pub_->publish(marker);
    
    // 화살표도 삭제
    visualization_msgs::msg::Marker arrow_delete;
    arrow_delete.header = marker.header;
    arrow_delete.ns = "overtake_arrow";
    arrow_delete.id = 1;
    arrow_delete.action = visualization_msgs::msg::Marker::DELETE;
    overtake_path_marker_pub_->publish(arrow_delete);
    return;
  }
  
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.scale.x = 0.15;  // 라인 두께 (더 두껍게)
  
  // 밝은 녹색 라인 (유효한 추월 경로 시각화)
  marker.color.r = 0.2f;
  marker.color.g = 1.0f;
  marker.color.b = 0.2f;
  marker.color.a = 0.9f;
  
  marker.pose.orientation.w = 1.0;
  
  // 현재 위치에서 전방 추월 경로 표시
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  // 추월 경로 생성 및 시각화 (차량 넓이 고려)
  // 횡방향 오프셋: 자차 넓이 + 상대차 넓이 + 안전 마진
  double safe_lateral_offset = overtake_lateral_offset_ + vehicle_width_;
  
  for (double dist = 0.0; dist < OVERTAKE_VIS_LENGTH; dist += OVERTAKE_VIS_STEP) {
    geometry_msgs::msg::Point p;
    // 추월 방향으로 오프셋된 경로 (부드러운 S자 곡선)
    // 진입 -> 최대 오프셋 -> 복귀
    double t = dist / OVERTAKE_VIS_LENGTH;  // 0 ~ 1
    double lateral_offset = offset_direction * safe_lateral_offset * 
                           std::sin(t * M_PI);  // 부드러운 곡선
    
    p.x = current_x + dist * std::cos(current_yaw) - lateral_offset * std::sin(current_yaw);
    p.y = current_y + dist * std::sin(current_yaw) + lateral_offset * std::cos(current_yaw);
    p.z = 0.15;
    marker.points.push_back(p);
  }
  
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // 더 짧은 lifetime으로 실시간 업데이트
  overtake_path_marker_pub_->publish(marker);
  
  // 추월 방향 화살표 표시 (경로가 유효할 때만)
  visualization_msgs::msg::Marker arrow_marker;
  arrow_marker.header.frame_id = "map";
  arrow_marker.header.stamp = this->get_clock()->now();
  arrow_marker.ns = "overtake_arrow";
  arrow_marker.id = 1;
  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.action = visualization_msgs::msg::Marker::ADD;
  
  arrow_marker.scale.x = 0.8;  // 화살표 길이
  arrow_marker.scale.y = 0.2;  // 화살표 너비
  arrow_marker.scale.z = 0.2;
  
  // 노란색 화살표 (유효한 추월 경로 표시)
  arrow_marker.color.r = 1.0f;
  arrow_marker.color.g = 1.0f;
  arrow_marker.color.b = 0.0f;
  arrow_marker.color.a = 0.9f;
  
  // 화살표 위치: 차량 전방 1m
  double arrow_offset = offset_direction * 0.4;
  arrow_marker.pose.position.x = current_x + 1.0 * std::cos(current_yaw) - arrow_offset * std::sin(current_yaw);
  arrow_marker.pose.position.y = current_y + 1.0 * std::sin(current_yaw) + arrow_offset * std::cos(current_yaw);
  arrow_marker.pose.position.z = 0.3;
  
  // 추월 방향으로 화살표 회전
  double arrow_yaw = current_yaw + offset_direction * 0.3;  // 약간 추월 방향으로
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
  
  // 충돌 시 빨간색 (크게), 아니면 녹색 (작게)
  if (is_colliding) {
    // 충돌 상태: 큰 빨간 구 표시
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    
    // 추가 로그 (벽 충돌 시 정보 표시)
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "WALL COLLISION DETECTED! pos=(%.2f, %.2f), F=%.3fm, L=%.3fm, R=%.3fm",
                          x, y, last_obstacle_front_dist_, 
                          last_obstacle_left_dist_, last_obstacle_right_dist_);
  } else {
    // 정상 상태: 작은 녹색 구 표시
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

/**
 * @brief 추월 경로를 생성하여 벡터로 반환
 * 
 * 현재 위치에서 추월 방향으로 S자 곡선 경로를 생성합니다.
 * 이 경로는 추월 안전성 검증과 시각화에 사용됩니다.
 * 
 * @param overtake_direction 추월 방향 (1.0: 왼쪽, -1.0: 오른쪽)
 * @return 생성된 추월 경로 포인트들
 */
std::vector<geometry_msgs::msg::Point> SimpleController::generate_overtake_trajectory(double overtake_direction)
{
  std::vector<geometry_msgs::msg::Point> trajectory;
  
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  // 추월 경로 길이: 5m (진입 -> 추월 -> 복귀)
  constexpr double OVERTAKE_PATH_LENGTH = 5.0;
  constexpr double PATH_RESOLUTION = 0.2;  // 20cm 간격
  
  for (double dist = 0.0; dist < OVERTAKE_PATH_LENGTH; dist += PATH_RESOLUTION) {
    geometry_msgs::msg::Point p;
    
    // S자 곡선으로 추월 경로 생성
    // t: 0 -> 1 (진입부터 복귀까지)
    double t = dist / OVERTAKE_PATH_LENGTH;
    
    // 횡방향 오프셋: sin(π*t)로 부드러운 S자 곡선
    // 최대 오프셋은 overtake_lateral_offset_
    double lateral_offset = overtake_direction * overtake_lateral_offset_ * std::sin(t * M_PI);
    
    // 전방 거리와 횡방향 오프셋을 글로벌 좌표로 변환
    p.x = current_x + dist * std::cos(current_yaw) - lateral_offset * std::sin(current_yaw);
    p.y = current_y + dist * std::sin(current_yaw) + lateral_offset * std::cos(current_yaw);
    p.z = 0.0;
    
    trajectory.push_back(p);
  }
  
  return trajectory;
}

/**
 * @brief 추월 경로가 안전한지 시뮬레이션하여 검증
 * 
 * 추월 경로의 각 포인트에서:
 * 1. 벽까지 거리가 충분한지 확인 (차량 넓이 고려)
 * 2. 레이싱 라인으로 복귀 가능한지 확인
 * 3. 경로 전체가 안전한지 종합적으로 판단
 * 
 * @param overtake_direction 추월 방향 (1.0: 왼쪽, -1.0: 오른쪽)
 * @return 추월 경로가 안전하면 true, 안전하지 않으면 false (이 경우 추월 금지!)
 */
bool SimpleController::validate_overtake_path(double overtake_direction)
{
  // 추월 경로 생성
  std::vector<geometry_msgs::msg::Point> trajectory = generate_overtake_trajectory(overtake_direction);
  
  if (trajectory.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "OVERTAKE PATH FAILED: Cannot generate trajectory");
    return false;
  }
  
  // 차량 넓이를 고려한 안전 마진 설정
  // 차량 넓이 절반 + 여유 공간
  double vehicle_half_width = vehicle_width_ / 2.0;
  double min_wall_clearance = vehicle_half_width + WALL_SAFETY_MARGIN;  // 차량 넓이 고려
  double min_track_width = vehicle_width_ * 2.0 + SAFETY_MARGIN;        // 두 차량 + 안전 마진
  
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  // 경로의 각 포인트에서 안전성 검증
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto& point = trajectory[i];
    
    // 1. 벽 데이터가 있으면 벽까지 거리 확인
    if (wall_data_loaded_) {
      double wall_left = get_wall_distance_left(point.x, point.y);
      double wall_right = get_wall_distance_right(point.x, point.y);
      
      // 추월 방향의 벽 거리 확인 (차량 넓이 고려)
      double wall_clearance = (overtake_direction > 0) ? wall_left : wall_right;
      
      if (wall_clearance < min_wall_clearance) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "OVERTAKE PATH INVALID: Wall too close (%.2fm < %.2fm, vehicle_width=%.2fm)",
                              wall_clearance, min_wall_clearance, vehicle_width_);
        return false;
      }
      
      // 트랙 너비 확인 (양쪽 벽 사이 공간 - 두 차량이 지나갈 수 있어야 함)
      double track_width = wall_left + wall_right;
      if (track_width < min_track_width) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "OVERTAKE PATH INVALID: Track too narrow (%.2fm < %.2fm for 2 vehicles)",
                              track_width, min_track_width);
        return false;
      }
    }
    
    // 2. 경로 상의 레이싱 라인과의 거리 확인 (복귀 가능성)
    if (!current_path_.poses.empty()) {
      // 이 포인트에서 가장 가까운 레이싱 라인 포인트 찾기
      double min_dist_to_raceline = std::numeric_limits<double>::max();
      
      for (const auto& pose : current_path_.poses) {
        double dx = point.x - pose.pose.position.x;
        double dy = point.y - pose.pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist_to_raceline) {
          min_dist_to_raceline = dist;
        }
      }
      
      // 레이싱 라인에서 너무 멀리 벗어나면 추월 불가
      double max_deviation = overtake_lateral_offset_ * 1.5;  // 최대 편차
      if (min_dist_to_raceline > max_deviation) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "OVERTAKE PATH INVALID: Too far from raceline (%.2fm > %.2fm)",
                              min_dist_to_raceline, max_deviation);
        return false;
      }
    }
  }
  
  // 3. 코너 진입 여부 확인 (경로 끝 부분)
  if (!trajectory.empty()) {
    const auto& end_point = trajectory.back();
    
    // 경로 끝 지점에서 코너가 있는지 확인
    if (!current_path_.poses.empty()) {
      double end_yaw = current_yaw;  // 대략적인 yaw 추정
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
  
  // 모든 검증 통과
  RCLCPP_INFO(this->get_logger(), 
              "OVERTAKE PATH VALID: Direction=%s, %zu points verified",
              overtake_direction > 0 ? "LEFT" : "RIGHT", trajectory.size());
  
  return true;
}

// ============================================================================
// CRSM (Collision Recovery State Machine) 기능 구현
// 보고서 기반: IMU 충돌 감지, 스톨 감지, 조향 역보정
// ============================================================================

/**
 * @brief IMU 기반 충돌 감지
 * 
 * 보고서 기반: |a_total| = sqrt(a_x^2 + a_y^2) > A_thresh (약 9.5 m/s^2)
 * 급격한 감속이 발생하면 충돌로 판정
 */
bool SimpleController::check_imu_collision()
{
  if (!imu_received_) {
    return false;
  }
  
  // 가속도 크기가 임계값 초과시 충돌로 판정
  if (imu_accel_magnitude_ > IMU_CRASH_ACCEL_THRESHOLD) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                          "CRSM: IMU crash detected! accel=%.2f m/s^2 > %.2f threshold",
                          imu_accel_magnitude_, IMU_CRASH_ACCEL_THRESHOLD);
    return true;
  }
  
  return false;
}

/**
 * @brief 스톨 감지 (명령 vs 실제 속도)
 * 
 * 보고서 기반: (v_cmd > 0.5 m/s) AND (|v_act| < 0.1 m/s) AND (min(Lidar_front) < 0.2m)
 * 전진 명령이 있으나 차량이 움직이지 않고 전방에 장애물이 있는 경우
 */
bool SimpleController::check_stall_condition()
{
  if (!odom_received_ || !scan_received_) {
    return false;
  }
  
  // 현재 실제 속도 계산
  double actual_speed = std::sqrt(
    current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
    current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
  );
  
  // 스톨 조건 체크
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

/**
 * @brief 반전된 후진 조향 계산 (CRSM 보고서 기반)
 * 
 * 조향각 역보정(Steering Inversion) 알고리즘:
 * - mode 0 (NEUTRAL): δ_rec = 0 (중립으로 후진) - 가장 안전
 * - mode 1 (INVERT): δ_rec = -δ_last (반대 방향으로 후진) - 벽에서 빠르게 탈출
 * - mode 2 (MAINTAIN): δ_rec = δ_last (기존 방식, 레거시 호환)
 * 
 * 원리: 충돌 시 바퀴가 벽 쪽으로 꺾여 있는 상태에서 조향을 중립으로 풀거나 
 * 반대로 꺾으면서 후진해야 차체가 벽에서 떨어져 나오는 궤적(Unwinding Trajectory)을 그릴 수 있음
 */
double SimpleController::compute_inverted_reverse_steering()
{
  double reverse_steer = 0.0;
  
  switch (reverse_steering_mode_) {
    case 0:  // NEUTRAL - 중립으로 후진
      reverse_steer = 0.0;
      break;
      
    case 1:  // INVERT - 반대 방향으로 후진
      reverse_steer = -last_steering_before_reverse_;
      // 최대 조향각 제한
      reverse_steer = std::clamp(reverse_steer, -max_steer_angle_, max_steer_angle_);
      break;
      
    case 2:  // MAINTAIN - 기존 방식 (레거시)
    default:
      reverse_steer = last_steering_before_reverse_;
      break;
  }
  
  return reverse_steer;
}

/**
 * @brief CRSM 상태 발행 (시각화용)
 */
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

/**
 * @brief 안전 구역 마커 발행 (상대 차량 금지 구역 시각화)
 * 
 * 보고서 기반: D_forbidden = [d_opp - W_car/2 - W_margin, d_opp + W_car/2 + W_margin]
 * 상대 차량 주변에 빨간 반투명 박스로 금지 구역 표시
 */
void SimpleController::publish_safety_zone_markers()
{
  if (!is_following_opponent_ && !is_overtaking_) {
    return;  // Following 또는 추월 중이 아니면 표시 안함
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  
  // 상대 차량 금지 구역 (빨간 반투명 박스)
  visualization_msgs::msg::Marker opponent_zone;
  opponent_zone.header.frame_id = "map";
  opponent_zone.header.stamp = this->get_clock()->now();
  opponent_zone.ns = "opponent_forbidden_zone";
  opponent_zone.id = 0;
  opponent_zone.type = visualization_msgs::msg::Marker::CUBE;
  opponent_zone.action = visualization_msgs::msg::Marker::ADD;
  
  // 현재 위치 기준으로 전방에 표시
  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;
  double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
  
  // 상대 차량 위치 (전방 장애물 거리 사용)
  double opp_dist = last_obstacle_front_dist_;
  opponent_zone.pose.position.x = current_x + opp_dist * std::cos(current_yaw);
  opponent_zone.pose.position.y = current_y + opp_dist * std::sin(current_yaw);
  opponent_zone.pose.position.z = 0.25;
  opponent_zone.pose.orientation.w = 1.0;
  
  // 금지 구역 크기: 차량 길이 x (차량 폭 + 2*안전마진)
  opponent_zone.scale.x = vehicle_length_;
  opponent_zone.scale.y = opponent_width_with_margin_;  // W_car + 2*W_margin
  opponent_zone.scale.z = 0.5;
  
  // 빨간 반투명
  opponent_zone.color.r = 1.0f;
  opponent_zone.color.g = 0.0f;
  opponent_zone.color.b = 0.0f;
  opponent_zone.color.a = 0.4f;
  
  opponent_zone.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_array.markers.push_back(opponent_zone);
  
  // Following 앵커 (파란 구 - 1.5m 뒤 목표 지점)
  if (is_following_opponent_) {
    visualization_msgs::msg::Marker follow_anchor;
    follow_anchor.header = opponent_zone.header;
    follow_anchor.ns = "follow_anchor";
    follow_anchor.id = 1;
    follow_anchor.type = visualization_msgs::msg::Marker::SPHERE;
    follow_anchor.action = visualization_msgs::msg::Marker::ADD;
    
    // 상대 차량 뒤 1.5m 지점
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
    
    // 파란색
    follow_anchor.color.r = 0.0f;
    follow_anchor.color.g = 0.0f;
    follow_anchor.color.b = 1.0f;
    follow_anchor.color.a = 0.9f;
    
    follow_anchor.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(follow_anchor);
  }
  
  safety_zone_pub_->publish(marker_array);
}

// ============================================================================
// ACC (Adaptive Cruise Control) 스타일 Following 기능 구현
// 보고서 기반: v_cmd = v_opp + Kp*(gap - 1.5) + Kd*(v_opp - v_ego)
// ============================================================================

/**
 * @brief 상대 차량 속도 추정 업데이트
 * 
 * 전방 장애물 거리 변화율로 상대 차량 속도 추정
 */
void SimpleController::update_opponent_velocity_estimate()
{
  if (!scan_received_) {
    return;
  }
  
  rclcpp::Time current_time = this->get_clock()->now();
  double dt = (current_time - prev_opponent_time_).seconds();
  
  if (dt > 0.01 && dt < 1.0) {  // 유효한 시간 간격
    double current_dist = last_obstacle_front_dist_;
    double dist_change = current_dist - prev_opponent_distance_;
    
    // 자차 속도 고려한 상대 속도 계산
    double ego_speed = std::sqrt(
      current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
      current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
    );
    
    // 거리 변화율 = 상대 속도 - 자차 속도
    // 상대 속도 = 거리 변화율 + 자차 속도
    double relative_vel = dist_change / dt;
    double raw_opponent_vel = ego_speed + relative_vel;
    
    // 음수 속도는 0으로 클램프 (상대방이 후진하지 않는다고 가정)
    raw_opponent_vel = std::max(0.0, raw_opponent_vel);
    
    // 스무딩 적용 (이전 값 70% + 새 값 30%)
    estimated_opponent_velocity_ = 0.7 * estimated_opponent_velocity_ + 
                                   0.3 * raw_opponent_vel;
  }
  
  prev_opponent_distance_ = last_obstacle_front_dist_;
  prev_opponent_time_ = current_time;
}

/**
 * @brief ACC 스타일 Following 속도 계산
 * 
 * 보고서 기반 제어 법칙:
 * v_cmd = v_opp + Kp * ((s_opp - s_ego) - 1.5) + Kd * (v_opp - v_ego)
 * 
 * - 거리가 1.5m보다 멀어지면 가속하여 붙음
 * - 거리가 1.5m보다 가까우면 감속하여 간격 유지
 * - 상대방 속도에 동기화하여 안정적인 Following
 */
double SimpleController::compute_acc_following_speed(double base_speed)
{
  double front_dist = last_obstacle_front_dist_;
  
  // Following 범위 체크
  if (front_dist >= follow_distance_threshold_ || front_dist <= a1_threshold_) {
    // Following 범위 밖 - ACC 비활성
    return base_speed;
  }
  
  // 상대 차량 속도 추정 업데이트
  update_opponent_velocity_estimate();
  
  // 자차 속도
  double ego_speed = std::sqrt(
    current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
    current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y
  );
  
  // === ACC PD 제어 법칙 ===
  // gap_error = 현재 거리 - 목표 거리 (양수면 더 멀다, 음수면 더 가깝다)
  double gap_error = front_dist - target_follow_gap_;
  
  // 상대 속도 오차 = 상대 속도 - 자차 속도
  double velocity_error = estimated_opponent_velocity_ - ego_speed;
  
  // ACC 속도 명령
  // v_cmd = v_opp + Kp * gap_error + Kd * velocity_error
  double acc_speed = estimated_opponent_velocity_ + 
                     acc_kp_ * gap_error + 
                     acc_kd_ * velocity_error;
  
  // 속도 제한 적용
  acc_speed = std::clamp(acc_speed, 0.0, base_speed);
  
  // 최소 속도 보장 (완전 정지 방지)
  double min_speed = base_speed * follow_min_speed_ratio_;
  acc_speed = std::max(acc_speed, min_speed);
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                        "ACC: gap=%.2fm, v_opp=%.2f, v_ego=%.2f, gap_err=%.2f, cmd=%.2f",
                        front_dist, estimated_opponent_velocity_, ego_speed, gap_error, acc_speed);
  
  return acc_speed;
}
