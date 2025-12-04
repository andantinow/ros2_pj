/**
 * @file nmpc_engine_node.cpp
 * @brief Nonlinear Model Predictive Controller (NMPC) for autonomous racing
 * 
 * This implementation uses a bicycle kinematic model for prediction and
 * solves the optimization problem using iterative linearization (SQP-like approach).
 * 
 * State: [x, y, yaw, v]
 * Control: [steering_angle, acceleration]
 */

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nmpc
{

// MPC Configuration
struct MPCConfig
{
  double horizon_sec = 1.0;       // Prediction horizon in seconds
  int prediction_steps = 10;      // Number of prediction steps (N)
  double dt = 0.1;                // Time step for prediction
  double wheelbase = 0.33;        // Vehicle wheelbase [m]
  
  // Weights for cost function
  double w_pos = 10.0;            // Position tracking weight
  double w_yaw = 5.0;             // Heading tracking weight
  double w_vel = 2.0;             // Velocity tracking weight
  double w_steer = 1.0;           // Steering effort weight
  double w_accel = 0.5;           // Acceleration effort weight
  double w_steer_rate = 10.0;     // Steering rate weight (smoothness)
  double w_accel_rate = 5.0;      // Acceleration rate weight (smoothness)
  
  // Constraints
  double max_steer = 0.5;         // Maximum steering angle [rad]
  double max_steer_rate = 1.0;    // Maximum steering rate [rad/s]
  double max_accel = 3.0;         // Maximum acceleration [m/s^2]
  double min_accel = -5.0;        // Maximum deceleration [m/s^2]
  double max_speed = 5.0;         // Maximum speed [m/s]
  double min_speed = 0.0;         // Minimum speed [m/s]
  
  // Solver settings
  int max_iterations = 10;        // Maximum SQP iterations (increased from 5)
  double convergence_tol = 1e-3;  // Convergence tolerance
};

// Vehicle state
struct VehicleState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
};

// Control input
struct ControlInput
{
  double steering = 0.0;
  double acceleration = 0.0;
};

// Reference trajectory point
struct ReferencePoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
};

// MPC Solution
struct MPCSolution
{
  double steering = 0.0;
  double speed = 0.0;
  bool feasible = false;
  double cost = 0.0;
  int iterations = 0;
};

/**
 * @brief Bicycle Kinematic Model based NMPC Solver
 * 
 * Uses iterative linearization to solve the nonlinear optimization problem.
 * The bicycle model equations are:
 *   x_dot = v * cos(yaw)
 *   y_dot = v * sin(yaw)
 *   yaw_dot = v * tan(delta) / L
 *   v_dot = a
 */
class BicycleMPCSolver
{
public:
  BicycleMPCSolver() = default;
  
  bool initialize(const MPCConfig& config, const rclcpp::Logger& logger)
  {
    config_ = config;
    config_.dt = config_.horizon_sec / std::max(1, config_.prediction_steps);
    
    // Initialize control sequence with moderate acceleration to get moving
    u_prev_.resize(config_.prediction_steps);
    for (auto& u : u_prev_) {
      u.steering = 0.0;
      u.acceleration = 1.0;  // Start with positive acceleration to get vehicle moving
    }
    
    RCLCPP_INFO(logger, 
      "NMPC Solver initialized | horizon: %.2fs, steps: %d, dt: %.3fs, wheelbase: %.2fm",
      config_.horizon_sec, config_.prediction_steps, config_.dt, config_.wheelbase);
    RCLCPP_INFO(logger,
      "Weights | pos: %.1f, yaw: %.1f, vel: %.1f, steer: %.1f, accel: %.1f",
      config_.w_pos, config_.w_yaw, config_.w_vel, config_.w_steer, config_.w_accel);
    
    return true;
  }
  
  MPCSolution solve(
    const VehicleState& current_state,
    const std::vector<ReferencePoint>& reference,
    const rclcpp::Logger& logger)
  {
    MPCSolution solution;
    solution.feasible = false;
    
    if (reference.empty()) {
      RCLCPP_WARN_THROTTLE(logger, *rclcpp::Clock::make_shared(), 2000,
        "NMPC: Empty reference trajectory");
      return solution;
    }
    
    // Ensure reference has enough points
    std::vector<ReferencePoint> ref = reference;
    while (ref.size() < static_cast<size_t>(config_.prediction_steps + 1)) {
      ref.push_back(ref.back());
    }
    
    // Initialize control sequence (warm start from previous solution)
    std::vector<ControlInput> u = u_prev_;
    
    // Iterative optimization (simplified SQP)
    double prev_cost = std::numeric_limits<double>::max();
    
    for (int iter = 0; iter < config_.max_iterations; ++iter) {
      // Forward simulate with current control sequence
      std::vector<VehicleState> predicted_states = forwardSimulate(current_state, u);
      
      // Compute cost
      double cost = computeCost(predicted_states, u, ref);
      
      // Check convergence
      if (std::abs(prev_cost - cost) < config_.convergence_tol) {
        solution.iterations = iter + 1;
        break;
      }
      prev_cost = cost;
      solution.iterations = iter + 1;
      
      // Compute gradients and update controls
      updateControls(current_state, predicted_states, u, ref);
      
      solution.cost = cost;
    }
    
    // Apply constraints
    applyConstraints(u);
    
    // Store for warm start
    u_prev_ = u;
    // Shift for next iteration
    if (u_prev_.size() > 1) {
      u_prev_.erase(u_prev_.begin());
      u_prev_.push_back(u_prev_.back());
    }
    
    // Extract first control input
    if (!u.empty()) {
      solution.steering = u[0].steering;
      
      // Compute speed command with minimum speed boost for initial acceleration
      double computed_speed = current_state.v + u[0].acceleration * config_.dt;
      
      // If vehicle is nearly stopped and we have a valid reference, apply minimum speed boost
      constexpr double MIN_OPERATING_SPEED = 0.5;  // Minimum speed to get moving
      constexpr double LOW_SPEED_THRESHOLD = 0.3;  // Below this, apply boost
      
      if (current_state.v < LOW_SPEED_THRESHOLD && computed_speed < MIN_OPERATING_SPEED) {
        // Apply initial speed boost to overcome static friction / get vehicle moving
        computed_speed = MIN_OPERATING_SPEED;
      }
      
      solution.speed = std::clamp(computed_speed, config_.min_speed, config_.max_speed);
      solution.feasible = true;
    }
    
    return solution;
  }

private:
  MPCConfig config_;
  std::vector<ControlInput> u_prev_;
  
  /**
   * @brief Forward simulate vehicle trajectory using bicycle model
   */
  std::vector<VehicleState> forwardSimulate(
    const VehicleState& initial_state,
    const std::vector<ControlInput>& controls) const
  {
    std::vector<VehicleState> states;
    states.reserve(controls.size() + 1);
    states.push_back(initial_state);
    
    VehicleState state = initial_state;
    
    for (const auto& u : controls) {
      // Bicycle kinematic model integration (Euler method)
      double cos_yaw = std::cos(state.yaw);
      double sin_yaw = std::sin(state.yaw);
      double tan_steer = std::tan(u.steering);
      
      // State update
      state.x += state.v * cos_yaw * config_.dt;
      state.y += state.v * sin_yaw * config_.dt;
      state.yaw += (state.v / config_.wheelbase) * tan_steer * config_.dt;
      state.v += u.acceleration * config_.dt;
      
      // Normalize yaw to [-pi, pi]
      state.yaw = normalizeAngle(state.yaw);
      
      // Enforce speed limits
      state.v = std::clamp(state.v, config_.min_speed, config_.max_speed);
      
      states.push_back(state);
    }
    
    return states;
  }
  
  /**
   * @brief Compute total cost for the predicted trajectory
   */
  double computeCost(
    const std::vector<VehicleState>& states,
    const std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference) const
  {
    double cost = 0.0;
    
    // State tracking cost
    for (size_t i = 0; i < states.size() && i < reference.size(); ++i) {
      double dx = states[i].x - reference[i].x;
      double dy = states[i].y - reference[i].y;
      double dyaw = normalizeAngle(states[i].yaw - reference[i].yaw);
      double dv = states[i].v - reference[i].v;
      
      cost += config_.w_pos * (dx * dx + dy * dy);
      cost += config_.w_yaw * dyaw * dyaw;
      cost += config_.w_vel * dv * dv;
    }
    
    // Control effort cost
    for (size_t i = 0; i < controls.size(); ++i) {
      cost += config_.w_steer * controls[i].steering * controls[i].steering;
      cost += config_.w_accel * controls[i].acceleration * controls[i].acceleration;
      
      // Control rate cost (smoothness)
      if (i > 0) {
        double d_steer = controls[i].steering - controls[i-1].steering;
        double d_accel = controls[i].acceleration - controls[i-1].acceleration;
        cost += config_.w_steer_rate * d_steer * d_steer;
        cost += config_.w_accel_rate * d_accel * d_accel;
      }
    }
    
    return cost;
  }
  
  /**
   * @brief Update controls using gradient descent
   */
  void updateControls(
    const VehicleState& current_state,
    const std::vector<VehicleState>& predicted_states,
    std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference)
  {
    // Increased learning rates for faster convergence
    const double learning_rate_steer = 0.2;   // Increased from 0.1
    const double learning_rate_accel = 0.15;  // Increased from 0.05
    const double epsilon = 1e-4;
    
    for (size_t i = 0; i < controls.size(); ++i) {
      // Numerical gradient for steering
      std::vector<ControlInput> u_plus = controls;
      std::vector<ControlInput> u_minus = controls;
      u_plus[i].steering += epsilon;
      u_minus[i].steering -= epsilon;
      
      auto states_plus = forwardSimulate(current_state, u_plus);
      auto states_minus = forwardSimulate(current_state, u_minus);
      
      double cost_plus = computeCost(states_plus, u_plus, reference);
      double cost_minus = computeCost(states_minus, u_minus, reference);
      double grad_steer = (cost_plus - cost_minus) / (2.0 * epsilon);
      
      // Numerical gradient for acceleration
      u_plus = controls;
      u_minus = controls;
      u_plus[i].acceleration += epsilon;
      u_minus[i].acceleration -= epsilon;
      
      states_plus = forwardSimulate(current_state, u_plus);
      states_minus = forwardSimulate(current_state, u_minus);
      
      cost_plus = computeCost(states_plus, u_plus, reference);
      cost_minus = computeCost(states_minus, u_minus, reference);
      double grad_accel = (cost_plus - cost_minus) / (2.0 * epsilon);
      
      // Gradient descent update
      controls[i].steering -= learning_rate_steer * grad_steer;
      controls[i].acceleration -= learning_rate_accel * grad_accel;
    }
  }
  
  /**
   * @brief Apply constraints to control sequence
   */
  void applyConstraints(std::vector<ControlInput>& controls) const
  {
    for (size_t i = 0; i < controls.size(); ++i) {
      // Steering limits
      controls[i].steering = std::clamp(controls[i].steering, 
        -config_.max_steer, config_.max_steer);
      
      // Acceleration limits
      controls[i].acceleration = std::clamp(controls[i].acceleration,
        config_.min_accel, config_.max_accel);
      
      // Steering rate limit
      if (i > 0) {
        double max_d_steer = config_.max_steer_rate * config_.dt;
        double d_steer = controls[i].steering - controls[i-1].steering;
        if (std::abs(d_steer) > max_d_steer) {
          controls[i].steering = controls[i-1].steering + 
            std::copysign(max_d_steer, d_steer);
        }
      }
    }
  }
  
  /**
   * @brief Normalize angle to [-pi, pi]
   */
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
};

}  // namespace nmpc

/**
 * @brief NMPC Engine Node - ROS2 interface for the MPC controller
 * 
 * Subscribes to odometry and path, runs MPC optimization, publishes drive commands.
 * Can use either Dual EKF output or ground truth odometry.
 * Includes A1/A2 range-based collision avoidance using LiDAR.
 */
class NMPCEngineNode : public rclcpp::Node
{
public:
  NMPCEngineNode()
  : Node("nmpc_engine_node")
  {
    // Declare parameters
    declare_parameter("prediction_horizon", 1.0);
    declare_parameter("prediction_steps", 10);
    declare_parameter("control_rate_hz", 50.0);
    declare_parameter("nominal_speed", 2.0);
    declare_parameter("solver_wheelbase", 0.33);
    
    // MPC weights
    declare_parameter("w_pos", 10.0);
    declare_parameter("w_yaw", 5.0);
    declare_parameter("w_vel", 2.0);
    declare_parameter("w_steer", 1.0);
    declare_parameter("w_accel", 0.5);
    
    // Constraints
    declare_parameter("max_steer", 0.5);
    declare_parameter("max_speed", 5.0);
    declare_parameter("max_accel", 3.0);
    declare_parameter("min_accel", -5.0);
    
    // Topic parameters
    declare_parameter<std::string>("odom_topic", "/dual_ekf/global_odom");  // Use Dual EKF output
    declare_parameter<std::string>("path_topic", "/global_raceline");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("scan_topic", "/scan");  // LiDAR topic
    declare_parameter<bool>("use_dual_ekf", true);  // Toggle between Dual EKF and ground truth
    
    // A1/A2 collision avoidance parameters
    declare_parameter("a1_threshold", 0.3);           // A1 range: reverse trigger distance (meters)
    declare_parameter("a2_threshold", 0.8);           // A2 range: steering avoidance distance (meters)
    declare_parameter("a1_side_factor", 0.8);         // A1 side distance factor
    declare_parameter("a2_max_steer_ratio", 0.5);     // A2 max steering ratio
    declare_parameter("reverse_speed", 0.5);          // Reverse speed (m/s)
    declare_parameter("reverse_duration", 0.8);       // Reverse duration (seconds)
    declare_parameter("a1_steer_gain", 0.8);          // A1 steering gain during reverse
    declare_parameter("a2_steer_gain", 0.4);          // A2 avoidance steering gain
    declare_parameter("enable_collision_avoidance", true);  // Enable collision avoidance

    // Get parameters
    double prediction_horizon = get_parameter("prediction_horizon").as_double();
    int prediction_steps = get_parameter("prediction_steps").as_int();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    nominal_speed_ = get_parameter("nominal_speed").as_double();
    max_steer_ = get_parameter("max_steer").as_double();
    
    std::string odom_topic = get_parameter("odom_topic").as_string();
    std::string path_topic = get_parameter("path_topic").as_string();
    std::string drive_topic = get_parameter("drive_topic").as_string();
    std::string scan_topic = get_parameter("scan_topic").as_string();
    use_dual_ekf_ = get_parameter("use_dual_ekf").as_bool();
    
    // Get collision avoidance parameters
    a1_threshold_ = get_parameter("a1_threshold").as_double();
    a2_threshold_ = get_parameter("a2_threshold").as_double();
    a1_side_factor_ = get_parameter("a1_side_factor").as_double();
    a2_max_steer_ratio_ = get_parameter("a2_max_steer_ratio").as_double();
    reverse_speed_ = get_parameter("reverse_speed").as_double();
    reverse_duration_ = get_parameter("reverse_duration").as_double();
    a1_steer_gain_ = get_parameter("a1_steer_gain").as_double();
    a2_steer_gain_ = get_parameter("a2_steer_gain").as_double();
    enable_collision_avoidance_ = get_parameter("enable_collision_avoidance").as_bool();
    
    // If not using Dual EKF, use ground truth
    if (!use_dual_ekf_) {
      odom_topic = "/car_state/odom_GT";
    }

    RCLCPP_INFO(get_logger(), "=== NMPC Engine Node Initialized ===");
    RCLCPP_INFO(get_logger(), "Prediction horizon: %.2fs, steps: %d", prediction_horizon, prediction_steps);
    RCLCPP_INFO(get_logger(), "Using %s for state estimation", use_dual_ekf_ ? "Dual EKF" : "Ground Truth");
    RCLCPP_INFO(get_logger(), "Topics - odom: %s, path: %s, drive: %s", 
                odom_topic.c_str(), path_topic.c_str(), drive_topic.c_str());
    RCLCPP_INFO(get_logger(), "A1/A2 Collision avoidance: A1=%.2fm (reverse), A2=%.2fm (steer), enabled=%s",
                a1_threshold_, a2_threshold_, enable_collision_avoidance_ ? "true" : "false");

    // Configure MPC solver
    nmpc::MPCConfig cfg;
    cfg.horizon_sec = prediction_horizon;
    cfg.prediction_steps = prediction_steps;
    cfg.wheelbase = get_parameter("solver_wheelbase").as_double();
    cfg.w_pos = get_parameter("w_pos").as_double();
    cfg.w_yaw = get_parameter("w_yaw").as_double();
    cfg.w_vel = get_parameter("w_vel").as_double();
    cfg.w_steer = get_parameter("w_steer").as_double();
    cfg.w_accel = get_parameter("w_accel").as_double();
    cfg.max_steer = get_parameter("max_steer").as_double();
    cfg.max_speed = get_parameter("max_speed").as_double();
    cfg.max_accel = get_parameter("max_accel").as_double();
    cfg.min_accel = get_parameter("min_accel").as_double();
    
    solver_.initialize(cfg, get_logger());

    // Setup subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(20).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(10).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = msg;
        if (msg && !msg->poses.empty()) {
          RCLCPP_INFO_ONCE(get_logger(), "Received path with %zu points", msg->poses.size());
        }
      });
    
    // LiDAR subscription for collision avoidance
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        if (!scan_received_) {
          RCLCPP_INFO(get_logger(), "Received first scan message. Ranges: %zu, angle: [%.2f, %.2f]",
                      msg->ranges.size(), msg->angle_min, msg->angle_max);
          scan_received_ = true;
        }
      });

    // Setup publisher
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic, rclcpp::QoS(10));

    // Setup control timer
    const double period_s = 1.0 / std::max(1.0, control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s),
      std::bind(&NMPCEngineNode::controlCycle, this));
    
    reverse_start_time_ = now();
    
    RCLCPP_INFO(get_logger(), "NMPC control loop running at %.1f Hz", control_rate_hz_);
  }

private:
  /**
   * @brief Main control cycle - called at control_rate_hz
   */
  void controlCycle()
  {
    if (!latest_odom_) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, 
        "Waiting for odometry...");
      return;
    }
    if (!latest_path_ || latest_path_->poses.empty()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, 
        "Waiting for reference path...");
      return;
    }
    
    rclcpp::Time current_time = now();
    
    // === A1/A2 Range-based Collision Avoidance ===
    // Update obstacle distances from LiDAR
    updateObstacleDistances();
    
    // A1 zone check: need to reverse
    if (checkA1Zone()) {
      // Start or continue reversing
      if (!is_reversing_) {
        is_reversing_ = true;
        reverse_start_time_ = current_time;
        RCLCPP_WARN(get_logger(), "A1 Zone! Starting reverse with opposite steering");
      }
      
      double elapsed = (current_time - reverse_start_time_).seconds();
      if (elapsed < reverse_duration_) {
        // Continue reversing
        double reverse_steer = computeReverseSteering();
        
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = current_time;
        cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
        cmd.drive.speed = -reverse_speed_;
        cmd.drive.steering_angle = reverse_steer;
        
        drive_pub_->publish(cmd);
        
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "A1 REVERSING: speed=%.2f, steer=%.3f, elapsed=%.2fs/%.2fs",
          -reverse_speed_, reverse_steer, elapsed, reverse_duration_);
        return;
      } else {
        // Reverse complete
        is_reversing_ = false;
        RCLCPP_INFO(get_logger(), "A1 Reverse complete, resuming forward");
      }
    } else {
      // Not in A1 zone, stop reversing if we were
      if (is_reversing_) {
        is_reversing_ = false;
        RCLCPP_INFO(get_logger(), "Left A1 zone, stopping reverse");
      }
    }

    // Extract current state from odometry
    nmpc::VehicleState current_state = extractState(latest_odom_);
    
    // Build reference trajectory
    std::vector<nmpc::ReferencePoint> reference = buildReference(current_state);
    
    if (reference.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Empty reference trajectory");
      return;
    }
    
    // Solve MPC
    auto solution = solver_.solve(current_state, reference, get_logger());
    
    // A2 zone: apply steering avoidance (without reversing)
    double delta_a2_avoidance = 0.0;
    if (!is_in_a1_zone_ && checkA2Zone()) {
      delta_a2_avoidance = computeAvoidanceSteering();
      solution.steering += delta_a2_avoidance;
      solution.steering = std::clamp(solution.steering, -max_steer_, max_steer_);
      // Also reduce speed in A2 zone
      solution.speed *= 0.7;
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
        "A2 Zone: adding avoidance steer=%.3f", delta_a2_avoidance);
    }
    
    // Publish result
    publishDriveCommand(solution);
    
    // Debug logging using RCLCPP_INFO_THROTTLE for thread safety
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,  // Every 1 second
      "MPC: state(%.2f, %.2f, %.2f°, %.2f m/s) -> cmd(steer=%.3f, speed=%.2f) [%d iters, cost=%.2f]",
      current_state.x, current_state.y, current_state.yaw * 180.0 / M_PI, current_state.v,
      solution.steering, solution.speed, solution.iterations, solution.cost);
  }
  
  /**
   * @brief Update obstacle distances from LiDAR scan
   */
  void updateObstacleDistances()
  {
    if (!enable_collision_avoidance_ || !scan_received_ || !latest_scan_) {
      last_obstacle_front_dist_ = 10.0;
      last_obstacle_left_dist_ = 10.0;
      last_obstacle_right_dist_ = 10.0;
      last_obstacle_angle_ = 0.0;
      return;
    }
    
    constexpr double FRONT_SECTOR_HALF_ANGLE = M_PI / 3.0;   // 60 degrees
    constexpr double SIDE_INNER = M_PI / 6.0;               // 30 degrees
    constexpr double SIDE_OUTER = M_PI / 2.0;               // 90 degrees
    constexpr double MIN_VALID_RANGE = 0.03;
    
    double angle_min = latest_scan_->angle_min;
    double angle_inc = latest_scan_->angle_increment;
    int num_ranges = latest_scan_->ranges.size();
    
    double min_front = std::numeric_limits<double>::max();
    double min_left = std::numeric_limits<double>::max();
    double min_right = std::numeric_limits<double>::max();
    double front_angle = 0.0;
    
    for (int i = 0; i < num_ranges; ++i) {
      double angle = angle_min + i * angle_inc;
      double range = latest_scan_->ranges[i];
      
      if (std::isnan(range) || std::isinf(range) || range < MIN_VALID_RANGE) {
        continue;
      }
      
      // Front sector (-60° ~ +60°)
      if (angle >= -FRONT_SECTOR_HALF_ANGLE && angle <= FRONT_SECTOR_HALF_ANGLE) {
        if (range < min_front) {
          min_front = range;
          front_angle = angle;
        }
      }
      
      // Left sector (30° ~ 90°)
      if (angle >= SIDE_INNER && angle <= SIDE_OUTER) {
        if (range < min_left) {
          min_left = range;
        }
      }
      
      // Right sector (-90° ~ -30°)
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
   * @brief Check if in A1 zone (need to reverse)
   */
  bool checkA1Zone()
  {
    if (!enable_collision_avoidance_ || !scan_received_) {
      is_in_a1_zone_ = false;
      return false;
    }
    
    double side_threshold = a1_threshold_ * a1_side_factor_;
    
    is_in_a1_zone_ = (last_obstacle_front_dist_ < a1_threshold_) ||
                     (last_obstacle_left_dist_ < side_threshold) ||
                     (last_obstacle_right_dist_ < side_threshold);
    
    if (is_in_a1_zone_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "A1 Zone! Front: %.2fm, L: %.2fm, R: %.2fm (threshold: %.2fm, side: %.2fm)",
        last_obstacle_front_dist_, last_obstacle_left_dist_, 
        last_obstacle_right_dist_, a1_threshold_, side_threshold);
    }
    
    return is_in_a1_zone_;
  }
  
  /**
   * @brief Check if in A2 zone (steering avoidance needed)
   */
  bool checkA2Zone()
  {
    if (!enable_collision_avoidance_ || !scan_received_) {
      return false;
    }
    
    bool in_a2 = (last_obstacle_front_dist_ < a2_threshold_) ||
                 (last_obstacle_left_dist_ < a2_threshold_) ||
                 (last_obstacle_right_dist_ < a2_threshold_);
    
    return in_a2;
  }
  
  /**
   * @brief Compute steering angle for reverse (A1 zone)
   */
  double computeReverseSteering()
  {
    double reverse_steer = 0.0;
    
    double left_dist = std::min(last_obstacle_left_dist_, 5.0);
    double right_dist = std::min(last_obstacle_right_dist_, 5.0);
    
    double dist_diff = right_dist - left_dist;
    
    if (std::abs(dist_diff) > 0.05) {
      reverse_steer = -std::copysign(1.0, dist_diff) * max_steer_ * a1_steer_gain_;
    } else {
      if (std::abs(last_obstacle_angle_) > 0.1) {
        reverse_steer = -last_obstacle_angle_ * a1_steer_gain_;
      }
    }
    
    reverse_steer = std::clamp(reverse_steer, -max_steer_, max_steer_);
    
    return reverse_steer;
  }
  
  /**
   * @brief Compute avoidance steering (A2 zone)
   */
  double computeAvoidanceSteering()
  {
    double avoidance_steer = 0.0;
    
    double left_dist = std::min(last_obstacle_left_dist_, 5.0);
    double right_dist = std::min(last_obstacle_right_dist_, 5.0);
    double front_dist = std::min(last_obstacle_front_dist_, 5.0);
    
    double urgency = 1.0 - (front_dist / a2_threshold_);
    urgency = std::clamp(urgency, 0.0, 1.0);
    
    if (left_dist < right_dist) {
      double left_urgency = 1.0 - (left_dist / a2_threshold_);
      left_urgency = std::clamp(left_urgency, 0.0, 1.0);
      avoidance_steer = -max_steer_ * a2_steer_gain_ * left_urgency;
    } else if (right_dist < left_dist) {
      double right_urgency = 1.0 - (right_dist / a2_threshold_);
      right_urgency = std::clamp(right_urgency, 0.0, 1.0);
      avoidance_steer = max_steer_ * a2_steer_gain_ * right_urgency;
    } else {
      if (std::abs(last_obstacle_angle_) > 0.05) {
        avoidance_steer = -last_obstacle_angle_ * a2_steer_gain_ * urgency;
      }
    }
    
    double max_avoidance = max_steer_ * a2_max_steer_ratio_;
    avoidance_steer = std::clamp(avoidance_steer, -max_avoidance, max_avoidance);
    
    return avoidance_steer;
  }
  
  /**
   * @brief Extract vehicle state from odometry message
   */
  nmpc::VehicleState extractState(const nav_msgs::msg::Odometry::SharedPtr& odom) const
  {
    nmpc::VehicleState state;
    state.x = odom->pose.pose.position.x;
    state.y = odom->pose.pose.position.y;
    
    // Extract yaw from quaternion
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    state.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    // Get velocity (use linear velocity magnitude)
    state.v = std::sqrt(
      odom->twist.twist.linear.x * odom->twist.twist.linear.x +
      odom->twist.twist.linear.y * odom->twist.twist.linear.y);
    
    return state;
  }
  
  /**
   * @brief Build reference trajectory from path
   */
  std::vector<nmpc::ReferencePoint> buildReference(const nmpc::VehicleState& current_state) const
  {
    std::vector<nmpc::ReferencePoint> reference;
    
    if (!latest_path_ || latest_path_->poses.empty()) {
      return reference;
    }
    
    const auto& poses = latest_path_->poses;
    const size_t num_poses = poses.size();
    
    // Find closest point on path
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < num_poses; ++i) {
      double dx = poses[i].pose.position.x - current_state.x;
      double dy = poses[i].pose.position.y - current_state.y;
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }
    
    // Build reference from closest point forward
    // Constants for reference trajectory building
    constexpr int NUM_REF_POINTS = 15;      // Number of reference points for MPC horizon
    constexpr size_t PATH_STRIDE_DIVISOR = 50;  // Divisor for computing sampling stride
    
    const size_t stride = std::max<size_t>(1, num_poses / PATH_STRIDE_DIVISOR);
    
    for (int i = 0; i < NUM_REF_POINTS; ++i) {
      size_t idx = (closest_idx + i * stride) % num_poses;
      
      nmpc::ReferencePoint ref;
      ref.x = poses[idx].pose.position.x;
      ref.y = poses[idx].pose.position.y;
      
      // Extract yaw from pose orientation
      double qx = poses[idx].pose.orientation.x;
      double qy = poses[idx].pose.orientation.y;
      double qz = poses[idx].pose.orientation.z;
      double qw = poses[idx].pose.orientation.w;
      ref.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      
      ref.v = nominal_speed_;
      
      reference.push_back(ref);
    }
    
    return reference;
  }
  
  /**
   * @brief Publish drive command from MPC solution
   */
  void publishDriveCommand(const nmpc::MPCSolution& solution)
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
    
    if (solution.feasible) {
      cmd.drive.steering_angle = solution.steering;
      cmd.drive.speed = solution.speed;
    } else {
      // Stop if solution is not feasible
      cmd.drive.steering_angle = 0.0;
      cmd.drive.speed = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "MPC solution not feasible, stopping vehicle");
    }
    
    drive_pub_->publish(cmd);
  }

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Path::SharedPtr latest_path_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  bool scan_received_{false};

  // Parameters
  double control_rate_hz_{50.0};
  double nominal_speed_{2.0};
  bool use_dual_ekf_{true};
  double max_steer_{0.5};
  
  // A1/A2 Collision avoidance parameters
  double a1_threshold_{0.3};
  double a2_threshold_{0.8};
  double a1_side_factor_{0.8};
  double a2_max_steer_ratio_{0.5};
  double reverse_speed_{0.5};
  double reverse_duration_{0.8};
  double a1_steer_gain_{0.8};
  double a2_steer_gain_{0.4};
  bool enable_collision_avoidance_{true};
  
  // Collision avoidance state
  bool is_reversing_{false};
  bool is_in_a1_zone_{false};
  rclcpp::Time reverse_start_time_;
  double last_obstacle_left_dist_{10.0};
  double last_obstacle_right_dist_{10.0};
  double last_obstacle_front_dist_{10.0};
  double last_obstacle_angle_{0.0};

  // MPC Solver
  nmpc::BicycleMPCSolver solver_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCEngineNode>());
  rclcpp::shutdown();
  return 0;
}

