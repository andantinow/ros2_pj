#include "simple_controller.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <rclcpp/qos.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
  declare_parameter("a1_side_factor", a1_side_factor_);
  declare_parameter("a2_max_steer_ratio", a2_max_steer_ratio_);
  declare_parameter("reverse_speed", reverse_speed_);
  declare_parameter("reverse_duration", reverse_duration_);
  declare_parameter("a1_steer_gain", a1_steer_gain_);
  declare_parameter("a2_steer_gain", a2_steer_gain_);
  declare_parameter("enable_collision_avoidance", enable_collision_avoidance_);

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
  a1_side_factor_ = get_parameter("a1_side_factor").as_double();
  a2_max_steer_ratio_ = get_parameter("a2_max_steer_ratio").as_double();
  reverse_speed_ = get_parameter("reverse_speed").as_double();
  reverse_duration_ = get_parameter("reverse_duration").as_double();
  a1_steer_gain_ = get_parameter("a1_steer_gain").as_double();
  a2_steer_gain_ = get_parameter("a2_steer_gain").as_double();
  enable_collision_avoidance_ = get_parameter("enable_collision_avoidance").as_bool();
  
  prev_time_ = this->get_clock()->now();
  reverse_start_time_ = this->get_clock()->now();

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

  // Drive Publisher
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_,
      10);
  
  // Lookahead point visualization marker publisher (빨간 점으로 표시)
  lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/pid_lookahead_point",
      rclcpp::QoS(1));
  
  RCLCPP_INFO(this->get_logger(), "Subscribing to odom: %s, path: %s, scan: %s, publishing to drive: %s", 
              odom_topic_.c_str(), path_topic_.c_str(), scan_topic_.c_str(), drive_topic_.c_str());
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
 * @brief A2 범위에서 회피 조향각 계산
 * 후진 없이 조향만으로 장애물 회피 - 측면 장애물 0.4m 이내시 급격한 반대방향 조향
 */
double SimpleController::compute_avoidance_steering()
{
  double avoidance_steer = 0.0;
  
  // 장애물이 있는 방향의 반대로 조향
  double left_dist = std::min(last_obstacle_left_dist_, 5.0);
  double right_dist = std::min(last_obstacle_right_dist_, 5.0);
  double front_dist = std::min(last_obstacle_front_dist_, 5.0);
  
  // 방향별 긴급도 계산 - 각 방향의 거리에 따라 다름
  constexpr double URGENCY_BOOST = 0.5;  // Boost factor for urgency-based steering
  
  std::string avoid_direction = "NONE";
  double urgency = 0.0;
  
  if (left_dist < right_dist) {
    // 왼쪽에 장애물이 더 가까움 -> 오른쪽으로 급격히 조향 (음수)
    double left_urgency = 1.0 - (left_dist / a2_threshold_);
    left_urgency = std::clamp(left_urgency, 0.0, 1.0);
    urgency = left_urgency;
    // 긴급도가 높을수록 조향 강도 증가
    double effective_gain = a2_steer_gain_ * (1.0 + left_urgency * URGENCY_BOOST);
    avoidance_steer = -max_steer_angle_ * effective_gain * left_urgency;
    avoid_direction = "RIGHT (wall on LEFT)";
  } else if (right_dist < left_dist) {
    // 오른쪽에 장애물이 더 가까움 -> 왼쪽으로 급격히 조향 (양수)
    double right_urgency = 1.0 - (right_dist / a2_threshold_);
    right_urgency = std::clamp(right_urgency, 0.0, 1.0);
    urgency = right_urgency;
    // 긴급도가 높을수록 조향 강도 증가
    double effective_gain = a2_steer_gain_ * (1.0 + right_urgency * URGENCY_BOOST);
    avoidance_steer = max_steer_angle_ * effective_gain * right_urgency;
    avoid_direction = "LEFT (wall on RIGHT)";
  } else {
    // 전방 장애물 각도 기반
    double front_urgency = 1.0 - (front_dist / a2_threshold_);
    front_urgency = std::clamp(front_urgency, 0.0, 1.0);
    urgency = front_urgency;
    if (std::abs(last_obstacle_angle_) > 0.05) {
      double effective_gain = a2_steer_gain_ * (1.0 + front_urgency * URGENCY_BOOST);
      avoidance_steer = -last_obstacle_angle_ * effective_gain;
      avoid_direction = last_obstacle_angle_ > 0 ? "RIGHT (front-left wall)" : "LEFT (front-right wall)";
    }
  }
  
  // 가장 가까운 장애물 기준으로 최대 조향 각도 제한 조정
  double min_dist = std::min(left_dist, std::min(right_dist, front_dist));
  double min_urgency = 1.0 - (min_dist / a2_threshold_);
  min_urgency = std::clamp(min_urgency, 0.0, 1.0);
  
  // 급격한 회피 허용 - 긴급시 최대 조향까지
  double max_avoidance = max_steer_angle_ * std::min(1.0, a2_max_steer_ratio_ + min_urgency * URGENCY_BOOST);
  avoidance_steer = std::clamp(avoidance_steer, -max_avoidance, max_avoidance);
  
  // 벽 감지 시 로그 출력 (INFO level for wall avoidance debugging - throttled to 500ms)
  if (std::abs(avoidance_steer) > 0.01) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                 "[WALL AVOID] %s | steer=%.3f | L:%.2fm R:%.2fm F:%.2fm | urgency:%.2f",
                 avoid_direction.c_str(), avoidance_steer, left_dist, right_dist, front_dist, urgency);
  }
  
  return avoidance_steer;
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
  
  // A1 범위 체크: 후진 필요
  if (check_a1_zone()) {
    // Start or continue reversing
    if (!is_reversing_) {
      is_reversing_ = true;
      reverse_start_time_ = current_time;
      last_steering_before_reverse_ = prev_steering_angle_;
      RCLCPP_WARN(this->get_logger(), "A1 Zone! Starting reverse with opposite steering");
    }
    
    double elapsed = (current_time - reverse_start_time_).seconds();
    if (elapsed < reverse_duration_) {
      // Continue reversing
      double reverse_steer = compute_reverse_steering();
      
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = current_time;
      drive_msg.header.frame_id = current_odom_.child_frame_id;
      drive_msg.drive.speed = -reverse_speed_;
      drive_msg.drive.steering_angle = reverse_steer;
      
      drive_pub_->publish(drive_msg);
      
      static int reverse_log_count = 0;
      if (reverse_log_count++ % 25 == 0) {
        RCLCPP_INFO(this->get_logger(), "A1 REVERSING: speed=%.2f, steer=%.3f, elapsed=%.2fs/%.2fs",
                    -reverse_speed_, reverse_steer, elapsed, reverse_duration_);
      }
      return;
    } else {
      // Reverse complete
      is_reversing_ = false;
      RCLCPP_INFO(this->get_logger(), "A1 Reverse complete, resuming forward");
    }
  } else {
    // Not in A1 zone, stop reversing if we were
    if (is_reversing_) {
      is_reversing_ = false;
      RCLCPP_INFO(this->get_logger(), "Left A1 zone, stopping reverse");
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
  
  // 6) A2 범위 기반 회피 조향 추가 (후진 없이 조향만)
  // is_in_a1_zone_는 check_a1_zone()에서 업데이트됨 (중복 체크 방지)
  double delta_a2_avoidance = 0.0;
  double a2_slowdown_factor = 1.0;
  if (!is_in_a1_zone_ && check_a2_zone()) {
    delta_a2_avoidance = compute_avoidance_steering();
    steering_angle += delta_a2_avoidance;
    
    // 측면 장애물 거리에 따른 속도 감소 - 0.4m 이내면 점점 느려짐
    double min_side_dist = std::min(last_obstacle_left_dist_, last_obstacle_right_dist_);
    a2_slowdown_factor = std::max(0.3, min_side_dist / a2_threshold_);  // 30% ~ 100% 속도
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                          "A2 Zone: steer=%.3f, slowdown=%.1f%%", 
                          delta_a2_avoidance, a2_slowdown_factor * 100.0);
  }

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
                "Hybrid: lat_err=%.3f, hdg_err=%.3f, PP=%.3f, PID=%.3f, Stanley=%.3f, FF=%.3f, A2Avoid=%.3f, steer=%.3f",
                lateral_error, heading_error, delta_pp, delta_pid, delta_stanley, delta_ff, delta_a2_avoidance, steering_angle);
  }

  // Speed control: Reduce speed based on curvature
  // A2 범위에 있으면 속도도 줄임
  double curvature = std::abs(steering_angle) / wheelbase_;
  double speed_factor = 1.0 / (1.0 + 2.0 * curvature);
  
  // A2 회피 중이면 속도 추가 감소 (거리 기반)
  speed_factor *= a2_slowdown_factor;
  
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
    RCLCPP_INFO(this->get_logger(), "Published drive command: speed=%.2f, steer=%.3f, target_idx=%d, lookahead=%.2f",
                drive_msg.drive.speed, drive_msg.drive.steering_angle, target_idx, adaptive_lookahead);
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
  
  // Restricted search window: limit to previous few points and next 20 points
  int search_range_forward = 20;
  int search_range_backward = 5;
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
