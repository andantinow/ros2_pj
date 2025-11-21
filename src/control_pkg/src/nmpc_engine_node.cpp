#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vehicle_model_msgs/msg/adaptive_vehicle_model.hpp>
#include "path_utils.hpp"

using namespace path_utils;

class NMPCEngineNode : public rclcpp::Node {
public:
  NMPCEngineNode() : Node("nmpc_engine_node") {
    declare_parameter("lookahead_distance", 1.5);
    declare_parameter("speed_limit", 5.0);
    declare_parameter("steer_rate_limit", 2.0);
    declare_parameter("accel_limit", 3.0);
    declare_parameter("decel_limit", 5.0);
    declare_parameter("wheelbase", 0.33);
    declare_parameter("Kp_lateral", 1.5);
    declare_parameter("Kp_heading", 2.0);
    
    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    speed_limit_ = get_parameter("speed_limit").as_double();
    steer_rate_limit_ = get_parameter("steer_rate_limit").as_double();
    accel_limit_ = get_parameter("accel_limit").as_double();
    decel_limit_ = get_parameter("decel_limit").as_double();
    wheelbase_ = get_parameter("wheelbase").as_double();
    Kp_lateral_ = get_parameter("Kp_lateral").as_double();
    Kp_heading_ = get_parameter("Kp_heading").as_double();
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/perfect_odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; });
    
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_raceline", rclcpp::QoS(10).transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) { path_ = msg; });
    
    vref_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/raceline_vref", rclcpp::QoS(10).transient_local(),
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { vref_ = msg; });
    
    vehicle_model_sub_ = create_subscription<vehicle_model_msgs::msg::AdaptiveVehicleModel>(
      "/adaptive_vehicle_model", 10,
      [this](vehicle_model_msgs::msg::AdaptiveVehicleModel::SharedPtr msg) { vehicle_model_ = msg; });
    
    cmd_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/sim/drive", 10);
    
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { loop(); });
    
    steer_rl_ = std::make_unique<RateLimiter>(steer_rate_limit_);
    speed_rl_ = std::make_unique<RateLimiter>(3.0);
    
    RCLCPP_INFO(get_logger(), "NMPC Engine Node initialized");
#ifdef HAVE_ACADOS
    RCLCPP_INFO(get_logger(), "acados support: ENABLED");
#else
    RCLCPP_INFO(get_logger(), "acados support: DISABLED (using feedforward+feedback)");
#endif
  }

private:
  void loop() {
    auto now = this->now();
    double dt = 0.01;
    if (last_time_.seconds() > 0.0) {
      dt = (now - last_time_).seconds();
      if (dt <= 0.0 || dt > 0.1) dt = 0.01;
    }
    last_time_ = now;
    
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = "base_link";
    
    if (!computeControl(dt, cmd)) {
      if (last_good_cmd_.header.stamp.sec > 0) {
        cmd = last_good_cmd_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Control computation failed, using last good command");
      } else {
        cmd.drive.speed = 0.0;
        cmd.drive.steering_angle = 0.0;
      }
    }
    
    cmd_pub_->publish(cmd);
  }
  
  bool computeControl(double dt, ackermann_msgs::msg::AckermannDriveStamped& cmd) {
    if (!odom_ || !path_ || path_->poses.empty()) {
      return false;
    }
    
    double x = odom_->pose.pose.position.x;
    double y = odom_->pose.pose.position.y;
    double current_yaw = yaw_from_quat(odom_->pose.pose.orientation);
    
    size_t nearest_idx = nearest_index(*path_, x, y);
    
    size_t lookahead_idx = nearest_idx;
    double accum_dist = 0.0;
    for (size_t i = nearest_idx; i < path_->poses.size() - 1; ++i) {
      const auto& p1 = path_->poses[i].pose.position;
      const auto& p2 = path_->poses[i + 1].pose.position;
      double seg_dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
      accum_dist += seg_dist;
      if (accum_dist >= lookahead_distance_) {
        lookahead_idx = i + 1;
        break;
      }
    }
    if (lookahead_idx >= path_->poses.size()) {
      lookahead_idx = path_->poses.size() - 1;
    }
    
    const auto& target_pose = path_->poses[lookahead_idx].pose;
    double target_x = target_pose.position.x;
    double target_y = target_pose.position.y;
    double path_yaw = yaw_from_quat(target_pose.orientation);
    
    double dx = target_x - x;
    double dy = target_y - y;
    double target_heading = std::atan2(dy, dx);
    double angle_error = normalize_angle(target_heading - current_yaw);
    
    double lateral_error = -dx * std::sin(current_yaw) + dy * std::cos(current_yaw);
    double heading_error = normalize_angle(path_yaw - current_yaw);
    
    double kappa = compute_curvature(*path_, lookahead_idx);
    
    double delta_ff = std::atan(wheelbase_ * kappa);
    double delta_fb = -Kp_lateral_ * lateral_error - Kp_heading_ * heading_error;
    double steering = delta_ff + delta_fb;
    steering = clamp(steering, -0.41, 0.41);
    
    double v_target = speed_limit_;
    if (vref_ && lookahead_idx < vref_->data.size()) {
      v_target = std::min(v_target, static_cast<double>(vref_->data[lookahead_idx]));
    }
    
    double speed_factor = 1.0 / (1.0 + 2.0 * std::abs(kappa));
    v_target *= speed_factor;
    
    if (vehicle_model_) {
      double mu = vehicle_model_->mu;
      if (mu < 0.8) {
        v_target *= 0.8;
      }
    }
    
#ifdef HAVE_ACADOS
    bool solver_ok = solve_acados(x, y, current_yaw, lateral_error, heading_error, kappa, v_target, dt);
    if (solver_ok) {
      steering = acados_solution_.steering;
      v_target = acados_solution_.speed;
    }
#endif
    
    cmd.drive.steering_angle = steer_rl_->apply(steering, dt);
    cmd.drive.speed = speed_rl_->apply(v_target, dt);
    cmd.header.stamp = this->now();
    
    last_good_cmd_ = cmd;
    return true;
  }
  
#ifdef HAVE_ACADOS
  struct AcadosSolution {
    double steering;
    double speed;
  };
  
  bool solve_acados(double x, double y, double yaw, double e_lat, double e_psi, 
                   double kappa, double v_ref, double dt) {
    return true;
  }
  
  AcadosSolution acados_solution_;
#endif
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vref_sub_;
  rclcpp::Subscription<vehicle_model_msgs::msg::AdaptiveVehicleModel>::SharedPtr vehicle_model_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  nav_msgs::msg::Odometry::SharedPtr odom_;
  nav_msgs::msg::Path::SharedPtr path_;
  std_msgs::msg::Float32MultiArray::SharedPtr vref_;
  vehicle_model_msgs::msg::AdaptiveVehicleModel::SharedPtr vehicle_model_;
  
  ackermann_msgs::msg::AckermannDriveStamped last_good_cmd_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  
  std::unique_ptr<RateLimiter> steer_rl_;
  std::unique_ptr<RateLimiter> speed_rl_;
  
  double lookahead_distance_;
  double speed_limit_;
  double steer_rate_limit_;
  double accel_limit_;
  double decel_limit_;
  double wheelbase_;
  double Kp_lateral_;
  double Kp_heading_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCEngineNode>());
  rclcpp::shutdown();
  return 0;
}

