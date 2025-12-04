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
#include <visualization_msgs/msg/marker.hpp>
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
  // Issue 3.1: Strengthen control rate (slew rate) penalty to prevent oscillation
  // The steering rate weight is critical for stability - must be significantly higher
  // than position/heading weights to prevent Bang-Bang control behavior
  double w_pos = 10.0;            // Position tracking weight (lateral error)
  double w_yaw = 8.0;             // Heading tracking weight (increased for stability)
  double w_vel = 2.0;             // Velocity tracking weight
  double w_steer = 0.5;           // Steering effort weight (reduced to allow necessary steering)
  double w_accel = 0.3;           // Acceleration effort weight
  double w_steer_rate = 500.0;    // Steering rate weight - CRITICAL for oscillation suppression
  double w_accel_rate = 50.0;     // Acceleration rate weight (smoothness)
  
  // Issue 5.1: Lateral error tube - allow deviation from centerline within bounds
  // This enables NMPC to find optimal racing lines within the tube
  double lateral_tolerance = 0.3; // Allow ±0.3m deviation from reference (Frenet tube)
  double w_terminal = 20.0;       // Terminal cost weight for better convergence
  
  // Constraints
  double max_steer = 0.436;       // Maximum steering angle [rad] (25 degrees)
  double max_steer_rate = 1.5;    // Maximum steering rate [rad/s] (slightly increased)
  double max_accel = 3.0;         // Maximum acceleration [m/s^2]
  double min_accel = -5.0;        // Maximum deceleration [m/s^2]
  double max_speed = 5.0;         // Maximum speed [m/s]
  double min_speed = 0.0;         // Minimum speed [m/s]
  
  // Solver settings
  int max_iterations = 15;        // Maximum SQP iterations (increased for better convergence)
  double convergence_tol = 1e-4;  // Tighter convergence tolerance
  
  // Issue 3.3: Latency compensation
  double latency_compensation_sec = 0.02;  // Default 20ms computation delay for forward simulation
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
  std::vector<VehicleState> predicted_trajectory;  // NMPC 예측 궤적 (시각화용)
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
      "Weights | pos: %.1f, yaw: %.1f, vel: %.1f, steer_rate: %.1f (CRITICAL)",
      config_.w_pos, config_.w_yaw, config_.w_vel, config_.w_steer_rate);
    RCLCPP_INFO(logger,
      "Lateral tolerance tube: %.2fm, Terminal weight: %.1f",
      config_.lateral_tolerance, config_.w_terminal);
    
    return true;
  }
  
  /**
   * @brief Issue 3.3 Fix: Forward simulate state to compensate for computation latency
   * 
   * When NMPC computation takes time (e.g., 50ms), the vehicle has moved.
   * This function predicts where the vehicle will be after the latency period.
   */
  VehicleState compensateLatency(
    const VehicleState& current_state,
    double latency_sec) const
  {
    if (latency_sec <= 0.0 || u_prev_.empty()) {
      return current_state;
    }
    
    VehicleState predicted = current_state;
    
    // Use previous control input to predict state after latency
    const auto& last_control = u_prev_[0];
    
    // Simple forward integration using bicycle model
    double cos_yaw = std::cos(predicted.yaw);
    double sin_yaw = std::sin(predicted.yaw);
    double tan_steer = std::tan(last_control.steering);
    
    predicted.x += predicted.v * cos_yaw * latency_sec;
    predicted.y += predicted.v * sin_yaw * latency_sec;
    predicted.yaw += (predicted.v / config_.wheelbase) * tan_steer * latency_sec;
    predicted.v += last_control.acceleration * latency_sec;
    
    predicted.yaw = normalizeAngle(predicted.yaw);
    predicted.v = std::clamp(predicted.v, config_.min_speed, config_.max_speed);
    
    return predicted;
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
    
    // Issue 3.3: Apply latency compensation - predict where vehicle will be
    VehicleState compensated_state = compensateLatency(current_state, config_.latency_compensation_sec);
    
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
      // Forward simulate with current control sequence (using compensated state)
      std::vector<VehicleState> predicted_states = forwardSimulate(compensated_state, u);
      
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
      updateControls(compensated_state, predicted_states, u, ref);
      
      solution.cost = cost;
    }
    
    // Apply constraints
    applyConstraints(u);
    
    // Store predicted trajectory for visualization (NMPC 예측 궤적)
    // Note: Re-simulation required after applyConstraints() modifies controls
    solution.predicted_trajectory = forwardSimulate(compensated_state, u);
    
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
   * 
   * Issue 3.1 & 5.1 Fix: Improved cost function with:
   * - Lateral tolerance tube (soft constraint) instead of hard position tracking
   * - Strong steering rate penalty to prevent oscillation
   * - Coupled heading-lateral error for better straight-line stability
   * - Terminal cost for improved convergence
   */
  double computeCost(
    const std::vector<VehicleState>& states,
    const std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference) const
  {
    double cost = 0.0;
    
    // State tracking cost with lateral tolerance tube (Issue 5.1)
    for (size_t i = 0; i < states.size() && i < reference.size(); ++i) {
      double dx = states[i].x - reference[i].x;
      double dy = states[i].y - reference[i].y;
      double dyaw = normalizeAngle(states[i].yaw - reference[i].yaw);
      double dv = states[i].v - reference[i].v;
      
      // Compute lateral error in path frame (Frenet-like)
      // Approximate: project position error onto perpendicular to reference heading
      double ref_cos = std::cos(reference[i].yaw);
      double ref_sin = std::sin(reference[i].yaw);
      double lateral_error = -dx * ref_sin + dy * ref_cos;
      double longitudinal_error = dx * ref_cos + dy * ref_sin;
      
      // Issue 5.1: Lateral tolerance tube - soft constraint with dead zone
      // Only penalize lateral error beyond the tolerance (allows optimal line within tube)
      double lateral_excess = std::max(0.0, std::abs(lateral_error) - config_.lateral_tolerance);
      
      // Issue 3.2: Coupled heading-lateral error for straight-line stability
      // When lateral error is small, heading error should dominate to prevent crabbing
      double lateral_weight_factor = 1.0;
      double heading_weight_factor = 1.0;
      if (std::abs(lateral_error) < 0.1) {
        // Small lateral error: increase heading weight to maintain direction
        heading_weight_factor = 2.0;
        lateral_weight_factor = 0.5;
      }
      
      // Position cost: penalize lateral excess (tube violation) and longitudinal error
      cost += config_.w_pos * lateral_weight_factor * lateral_excess * lateral_excess;
      cost += config_.w_pos * 0.3 * longitudinal_error * longitudinal_error;  // Lower weight for along-path error
      
      // Heading cost with adaptive weighting
      cost += config_.w_yaw * heading_weight_factor * dyaw * dyaw;
      
      // Velocity tracking cost
      cost += config_.w_vel * dv * dv;
    }
    
    // Terminal cost for better convergence (Issue 7.2)
    if (!states.empty() && !reference.empty()) {
      size_t last_idx = std::min(states.size() - 1, reference.size() - 1);
      double dx_term = states[last_idx].x - reference[last_idx].x;
      double dy_term = states[last_idx].y - reference[last_idx].y;
      double dyaw_term = normalizeAngle(states[last_idx].yaw - reference[last_idx].yaw);
      cost += config_.w_terminal * (dx_term * dx_term + dy_term * dy_term);
      cost += config_.w_terminal * 0.5 * dyaw_term * dyaw_term;
    }
    
    // Control effort cost
    for (size_t i = 0; i < controls.size(); ++i) {
      cost += config_.w_steer * controls[i].steering * controls[i].steering;
      cost += config_.w_accel * controls[i].acceleration * controls[i].acceleration;
      
      // Issue 3.1: Strong control rate cost (slew rate penalty) - CRITICAL for stability
      // This prevents Bang-Bang control and oscillation on straight sections
      if (i > 0) {
        double d_steer = controls[i].steering - controls[i-1].steering;
        double d_accel = controls[i].acceleration - controls[i-1].acceleration;
        cost += config_.w_steer_rate * d_steer * d_steer;
        cost += config_.w_accel_rate * d_accel * d_accel;
      }
    }
    
    // Add first control change cost (from previous command)
    if (!controls.empty() && !u_prev_.empty()) {
      double d_steer_init = controls[0].steering - u_prev_[0].steering;
      double d_accel_init = controls[0].acceleration - u_prev_[0].acceleration;
      cost += config_.w_steer_rate * d_steer_init * d_steer_init;
      cost += config_.w_accel_rate * d_accel_init * d_accel_init;
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
 * 
 * Issue Fixes Applied:
 * - 3.1: Strong steering rate penalty for oscillation suppression
 * - 3.2: Heading-lateral error coupling for straight-line stability
 * - 3.3: Latency compensation for high-speed control
 * - 4.1/4.2: Improved reference trajectory building (path-based, not Ld-based)
 * - 5.1: Lateral tolerance tube for optimal racing lines
 * - 6.2: Soft constraints for obstacle avoidance (via cost weighting)
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
    
    // MPC weights - Issue 3.1: steering rate is critical
    declare_parameter("w_pos", 10.0);
    declare_parameter("w_yaw", 8.0);           // Increased for stability
    declare_parameter("w_vel", 2.0);
    declare_parameter("w_steer", 0.5);         // Reduced to allow necessary steering
    declare_parameter("w_accel", 0.3);
    declare_parameter("w_steer_rate", 500.0);  // CRITICAL: High value prevents oscillation
    declare_parameter("w_accel_rate", 50.0);
    declare_parameter("w_terminal", 20.0);     // Terminal cost for convergence
    
    // Issue 5.1: Lateral tolerance tube
    declare_parameter("lateral_tolerance", 0.3);  // Allow ±0.3m deviation from centerline
    
    // Issue 3.3: Latency compensation
    declare_parameter("latency_compensation_sec", 0.02);  // 20ms default computation latency
    
    // Constraints
    declare_parameter("max_steer", 0.436);  // 25 degrees in radians
    declare_parameter("max_steer_rate", 1.5);  // rad/s
    declare_parameter("max_speed", 5.0);
    declare_parameter("max_accel", 3.0);
    declare_parameter("min_accel", -5.0);
    
    // Topic parameters
    declare_parameter<std::string>("odom_topic", "/dual_ekf/global_odom");  // Use Dual EKF output
    declare_parameter<std::string>("path_topic", "/global_raceline");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("scan_topic", "/scan");  // LiDAR topic
    declare_parameter<bool>("use_dual_ekf", true);  // Toggle between Dual EKF and ground truth
    
    // A1/A2 collision avoidance parameters (Issue 6: soft constraints via cost)
    declare_parameter("a1_threshold", 0.3);            // A1 range: reverse trigger distance (meters) - 30cm for safety
    declare_parameter("a2_threshold", 0.8);            // A2 range: side steering avoidance distance (meters) - 80cm
    declare_parameter("a2_urgent_threshold", 0.4);     // A2 urgent range: sharp steering when < 0.4m
    declare_parameter("a1_side_factor", 0.8);         // A1 side distance factor
    declare_parameter("a2_max_steer_ratio", 0.7);     // A2 max steering ratio - increased for sharper avoidance
    declare_parameter("a2_urgent_steer_ratio", 1.0);  // A2 urgent: full steering allowed when < 0.4m
    declare_parameter("reverse_speed", 0.5);          // Reverse speed (m/s)
    declare_parameter("reverse_duration", 0.8);       // Reverse duration (seconds)
    declare_parameter("a1_steer_gain", 0.8);          // A1 steering gain during reverse
    declare_parameter("a2_steer_gain", 0.6);          // A2 avoidance steering gain - increased for stronger response
    declare_parameter("a2_urgent_steer_gain", 1.5);   // A2 urgent steering gain - 1.5x when < 0.4m
    declare_parameter("enable_collision_avoidance", true);  // Enable collision avoidance

    // Get parameters
    double prediction_horizon = get_parameter("prediction_horizon").as_double();
    prediction_horizon_ = prediction_horizon;  // Store for reference trajectory building
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
    a2_urgent_threshold_ = get_parameter("a2_urgent_threshold").as_double();
    a1_side_factor_ = get_parameter("a1_side_factor").as_double();
    a2_max_steer_ratio_ = get_parameter("a2_max_steer_ratio").as_double();
    a2_urgent_steer_ratio_ = get_parameter("a2_urgent_steer_ratio").as_double();
    reverse_speed_ = get_parameter("reverse_speed").as_double();
    reverse_duration_ = get_parameter("reverse_duration").as_double();
    a1_steer_gain_ = get_parameter("a1_steer_gain").as_double();
    a2_steer_gain_ = get_parameter("a2_steer_gain").as_double();
    a2_urgent_steer_gain_ = get_parameter("a2_urgent_steer_gain").as_double();
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

    // Configure MPC solver with all Issue fixes
    nmpc::MPCConfig cfg;
    cfg.horizon_sec = prediction_horizon;
    cfg.prediction_steps = prediction_steps;
    cfg.wheelbase = get_parameter("solver_wheelbase").as_double();
    
    // Issue 3.1: Weights - steering rate is critical for oscillation suppression
    cfg.w_pos = get_parameter("w_pos").as_double();
    cfg.w_yaw = get_parameter("w_yaw").as_double();
    cfg.w_vel = get_parameter("w_vel").as_double();
    cfg.w_steer = get_parameter("w_steer").as_double();
    cfg.w_accel = get_parameter("w_accel").as_double();
    cfg.w_steer_rate = get_parameter("w_steer_rate").as_double();
    cfg.w_accel_rate = get_parameter("w_accel_rate").as_double();
    cfg.w_terminal = get_parameter("w_terminal").as_double();
    
    // Issue 5.1: Lateral tolerance tube
    cfg.lateral_tolerance = get_parameter("lateral_tolerance").as_double();
    
    // Issue 3.3: Latency compensation
    cfg.latency_compensation_sec = get_parameter("latency_compensation_sec").as_double();
    
    // Constraints
    cfg.max_steer = get_parameter("max_steer").as_double();
    cfg.max_steer_rate = get_parameter("max_steer_rate").as_double();
    cfg.max_speed = get_parameter("max_speed").as_double();
    cfg.max_accel = get_parameter("max_accel").as_double();
    cfg.min_accel = get_parameter("min_accel").as_double();
    
    solver_.initialize(cfg, get_logger());
    
    RCLCPP_INFO(get_logger(), "NMPC Issue Fixes Applied:");
    RCLCPP_INFO(get_logger(), "  - Steering rate weight: %.1f (Issue 3.1: oscillation suppression)", cfg.w_steer_rate);
    RCLCPP_INFO(get_logger(), "  - Lateral tolerance: %.2fm (Issue 5.1: racing line tube)", cfg.lateral_tolerance);
    RCLCPP_INFO(get_logger(), "  - Latency compensation: %.3fs (Issue 3.3: delay handling)", cfg.latency_compensation_sec);

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
    
    // NMPC 예측 궤적 시각화 publisher (녹색 라인으로 표시)
    nmpc_trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nmpc_predicted_trajectory", rclcpp::QoS(1));
    
    // NMPC 레퍼런스 포인트 시각화 publisher (파란 점으로 표시)
    nmpc_reference_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nmpc_reference_points", rclcpp::QoS(1));
    
    RCLCPP_INFO(get_logger(), "NMPC visualization: /nmpc_predicted_trajectory (GREEN), /nmpc_reference_points (BLUE)");

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
    
    // Track steering for collision recovery (before A2 modifications)
    if (!is_reversing_) {
      last_steering_before_collision_ = solution.steering;
    }
    
    // A2 zone: apply steering avoidance - lookahead 방향 기준 gap following
    double delta_a2_avoidance = 0.0;
    if (!is_in_a1_zone_ && checkA2Zone()) {
      // lookahead 방향 = 현재 위치에서 첫 번째 reference point 방향
      double lookahead_angle = 0.0;
      if (!reference.empty()) {
        double dx = reference[0].x - current_state.x;
        double dy = reference[0].y - current_state.y;
        // 차량 프레임으로 변환
        double cos_yaw = std::cos(-current_state.yaw);
        double sin_yaw = std::sin(-current_state.yaw);
        double dx_vehicle = dx * cos_yaw - dy * sin_yaw;
        double dy_vehicle = dx * sin_yaw + dy * cos_yaw;
        lookahead_angle = std::atan2(dy_vehicle, dx_vehicle);
      }
      
      delta_a2_avoidance = computeAvoidanceSteering(lookahead_angle);
      solution.steering += delta_a2_avoidance;
      solution.steering = std::clamp(solution.steering, -max_steer_, max_steer_);
      // Reduce speed in A2 zone - more aggressive slowdown for closer obstacles
      double min_side_dist = std::min(last_obstacle_left_dist_, last_obstacle_right_dist_);
      double slowdown_factor = std::max(0.3, min_side_dist / a2_threshold_);  // 30% to 100% speed
      solution.speed *= slowdown_factor;
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
        "A2 Zone: adding avoidance steer=%.3f, slowdown=%.1f%%", 
        delta_a2_avoidance, slowdown_factor * 100.0);
    }
    
    // Publish NMPC visualization (예측 궤적 + 레퍼런스)
    publishNMPCVisualization(solution.predicted_trajectory, reference);
    
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
   * When hitting obstacle, reverse and steer opposite to the direction that caused collision
   */
  double computeReverseSteering()
  {
    double reverse_steer = 0.0;
    
    double left_dist = std::min(last_obstacle_left_dist_, 5.0);
    double right_dist = std::min(last_obstacle_right_dist_, 5.0);
    
    // If we have a clear direction from obstacle distances
    double dist_diff = right_dist - left_dist;
    
    if (std::abs(dist_diff) > 0.05) {
      // Obstacle is closer on one side - steer opposite direction when reversing
      // If right side closer (dist_diff < 0), steer left (positive) when reversing
      // If left side closer (dist_diff > 0), steer right (negative) when reversing
      reverse_steer = -std::copysign(1.0, dist_diff) * max_steer_ * a1_steer_gain_;
    } else if (std::abs(last_steering_before_collision_) > 0.05) {
      // Steer opposite to the direction we were going before collision
      reverse_steer = -last_steering_before_collision_ * a1_steer_gain_;
    } else if (std::abs(last_obstacle_angle_) > 0.1) {
      // Fall back to obstacle angle based steering
      reverse_steer = -last_obstacle_angle_ * a1_steer_gain_;
    }
    
    reverse_steer = std::clamp(reverse_steer, -max_steer_, max_steer_);
    
    return reverse_steer;
  }
  
  /**
   * @brief lookahead 방향 기준으로 라이다 스캔에서 장애물 사이 중간점(gap center) 각도 계산
   * @param lookahead_angle 차량 프레임 기준 lookahead point 방향 각도 (rad)
   * @return gap center 방향 각도 (rad), 장애물이 없으면 lookahead_angle 반환
   */
  double findGapCenterAngle(double lookahead_angle)
  {
    if (!scan_received_ || !latest_scan_ || latest_scan_->ranges.empty()) {
      return lookahead_angle;
    }
    
    double angle_min = latest_scan_->angle_min;
    double angle_inc = latest_scan_->angle_increment;
    int num_ranges = latest_scan_->ranges.size();
    
    constexpr double SEARCH_HALF_ANGLE = M_PI / 3.0;  // 60 degrees
    constexpr double MIN_VALID_RANGE = 0.05;
    
    double search_min = lookahead_angle - SEARCH_HALF_ANGLE;
    double search_max = lookahead_angle + SEARCH_HALF_ANGLE;
    
    struct RayInfo {
      double angle;
      double range;
      bool is_obstacle;
    };
    std::vector<RayInfo> rays;
    
    for (int i = 0; i < num_ranges; ++i) {
      double angle = angle_min + i * angle_inc;
      
      if (angle < search_min || angle > search_max) {
        continue;
      }
      
      double range = latest_scan_->ranges[i];
      bool valid = !std::isnan(range) && !std::isinf(range) && range > MIN_VALID_RANGE;
      bool is_close_obstacle = valid && range < a2_threshold_;
      
      rays.push_back({angle, valid ? range : 10.0, is_close_obstacle});
    }
    
    if (rays.empty()) {
      return lookahead_angle;
    }
    
    double best_gap_center = lookahead_angle;
    double best_gap_score = -1.0;
    ssize_t gap_start_idx = -1;
    
    for (size_t i = 0; i < rays.size(); ++i) {
      if (!rays[i].is_obstacle) {
        if (gap_start_idx < 0) {
          gap_start_idx = static_cast<ssize_t>(i);
        }
      } else {
        if (gap_start_idx >= 0) {
          size_t gap_end_idx = i - 1;
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
          
          gap_start_idx = -1;
        }
      }
    }
    
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
    
    if (best_gap_score < 0.1) {
      return lookahead_angle;
    }
    
    return best_gap_center;
  }
  
  /**
   * @brief Compute avoidance steering (A2 zone) - lookahead 방향 기준 gap following
   * @param lookahead_angle 차량 프레임 기준 lookahead point 방향 각도 (rad)
   * 
   * When obstacles are detected, find gap center relative to lookahead direction
   * and steer toward the gap. Urgent mode (< 0.4m): maximum steering with 1.5x gain
   */
  double computeAvoidanceSteering(double lookahead_angle)
  {
    double avoidance_steer = 0.0;
    
    double left_dist = std::min(last_obstacle_left_dist_, 5.0);
    double right_dist = std::min(last_obstacle_right_dist_, 5.0);
    double front_dist = std::min(last_obstacle_front_dist_, 5.0);
    double min_side_dist = std::min(left_dist, right_dist);
    double min_dist = std::min(min_side_dist, front_dist);
    
    if (min_dist > a2_threshold_) {
      return 0.0;
    }
    
    // Find gap center relative to lookahead direction
    double gap_center_angle = findGapCenterAngle(lookahead_angle);
    double angle_diff = gap_center_angle - lookahead_angle;
    
    // Check if in urgent zone (< 0.4m)
    bool is_urgent = min_dist < a2_urgent_threshold_;
    
    // Calculate urgency and effective gain
    double urgency = 0.0;
    double effective_gain = a2_steer_gain_;
    double steer_ratio = a2_max_steer_ratio_;
    
    if (is_urgent) {
      urgency = 1.0 - (min_dist / a2_urgent_threshold_);
      urgency = std::clamp(urgency, 0.5, 1.0);
      effective_gain = a2_urgent_steer_gain_;
      steer_ratio = a2_urgent_steer_ratio_;
    } else {
      urgency = 1.0 - (min_dist / a2_threshold_);
      urgency = std::clamp(urgency, 0.0, 1.0);
      effective_gain = a2_steer_gain_ * (1.0 + urgency * 0.5);
      steer_ratio = a2_max_steer_ratio_ + urgency * 0.2;
    }
    
    // Steer toward gap center
    avoidance_steer = angle_diff * effective_gain * (0.5 + 0.5 * urgency);
    
    // Apply steering limit
    double max_avoidance = max_steer_ * std::min(1.0, steer_ratio);
    avoidance_steer = std::clamp(avoidance_steer, -max_avoidance, max_avoidance);
    
    // Log avoidance
    if (std::abs(avoidance_steer) > 0.01) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), is_urgent ? 200 : 500,
        "[%s GAP] lookahead=%.1f° gap=%.1f° | steer=%.3f | L:%.2f R:%.2f F:%.2f",
        is_urgent ? "URGENT" : "AVOID",
        lookahead_angle * 180.0 / M_PI, gap_center_angle * 180.0 / M_PI,
        avoidance_steer, left_dist, right_dist, front_dist);
    }
    
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
   * @brief Build reference trajectory from path for NMPC horizon
   * 
   * Issue 4.1/4.2 Fix: Build time-based reference trajectory for NMPC
   * - Uses prediction horizon time (Tp), NOT Pure Pursuit lookahead distance (Ld)
   * - Provides full trajectory for NMPC optimization, not just a single point
   * - Spacing is based on time steps (dt), not geometric lookahead
   * 
   * This decouples NMPC from Pure Pursuit logic and allows NMPC to see
   * the full path ahead for proper trajectory optimization.
   */
  std::vector<nmpc::ReferencePoint> buildReference(const nmpc::VehicleState& current_state) const
  {
    std::vector<nmpc::ReferencePoint> reference;
    
    if (!latest_path_ || latest_path_->poses.empty()) {
      return reference;
    }
    
    const auto& poses = latest_path_->poses;
    const size_t num_poses = poses.size();
    
    // Issue 4.2: Use prediction horizon parameters, NOT Pure Pursuit Ld
    // The MPC solver config determines the horizon, which should match reference
    constexpr int NUM_REF_POINTS = 15;              // Match or exceed prediction_steps
    constexpr double BEHIND_PENALTY = 3.0;          // Strong penalty for points behind
    constexpr size_t MAX_SEARCH_ITERATIONS = 20;    // Increased for better path following
    constexpr double MIN_AHEAD_DISTANCE = 0.05;     // Closer start point for precision
    
    double cos_yaw = std::cos(current_state.yaw);
    double sin_yaw = std::sin(current_state.yaw);
    
    // Find closest point on path, strongly preferring points ahead
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    // Search in a local window for efficiency (wrap around for closed loops)
    // Using member variable last_closest_idx_ instead of static for thread safety
    size_t search_start = (last_closest_idx_ > 50) ? last_closest_idx_ - 50 : 0;
    size_t search_end = std::min(num_poses, last_closest_idx_ + 100);
    
    // First pass: search near last known position
    for (size_t i = search_start; i < search_end; ++i) {
      double dx = poses[i].pose.position.x - current_state.x;
      double dy = poses[i].pose.position.y - current_state.y;
      double dist_sq = dx * dx + dy * dy;
      
      // Check if point is ahead of vehicle
      double ahead = dx * cos_yaw + dy * sin_yaw;
      
      // Strong preference for points ahead
      double weighted_dist = dist_sq;
      if (ahead < 0.0) {
        weighted_dist *= BEHIND_PENALTY;
      }
      
      if (weighted_dist < min_dist) {
        min_dist = weighted_dist;
        closest_idx = i;
      }
    }
    
    // If search window didn't find a good match, search globally
    if (min_dist > 4.0) {  // More than 2m away
      for (size_t i = 0; i < num_poses; ++i) {
        double dx = poses[i].pose.position.x - current_state.x;
        double dy = poses[i].pose.position.y - current_state.y;
        double dist_sq = dx * dx + dy * dy;
        double ahead = dx * cos_yaw + dy * sin_yaw;
        double weighted_dist = (ahead < 0.0) ? dist_sq * BEHIND_PENALTY : dist_sq;
        
        if (weighted_dist < min_dist) {
          min_dist = weighted_dist;
          closest_idx = i;
        }
      }
    }
    
    last_closest_idx_ = closest_idx;
    
    // Advance to first point that is clearly ahead
    size_t start_idx = closest_idx;
    for (size_t i = 0; i < MAX_SEARCH_ITERATIONS && start_idx < num_poses; ++i) {
      double dx = poses[start_idx].pose.position.x - current_state.x;
      double dy = poses[start_idx].pose.position.y - current_state.y;
      double ahead = dx * cos_yaw + dy * sin_yaw;
      
      if (ahead > MIN_AHEAD_DISTANCE) {
        break;
      }
      start_idx = (start_idx + 1) % num_poses;
    }
    
    // Issue 4.1: Time-based reference spacing (NOT Pure Pursuit Ld)
    // Compute spacing based on prediction horizon and vehicle speed
    // This ensures NMPC sees the full trajectory it needs for optimization
    // Using member variable prediction_horizon_ for consistency with solver config
    double speed = std::max(0.5, current_state.v);
    double total_lookahead_dist = speed * prediction_horizon_;
    
    // Calculate path resolution
    double total_path_length = 0.0;
    for (size_t i = 0; i < num_poses - 1; ++i) {
      double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
      double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
      total_path_length += std::sqrt(dx * dx + dy * dy);
    }
    double avg_point_spacing = std::max(0.05, total_path_length / std::max<size_t>(1, num_poses - 1));
    
    // Stride to cover prediction horizon evenly
    double dist_per_ref_point = total_lookahead_dist / NUM_REF_POINTS;
    size_t stride = std::max<size_t>(1, static_cast<size_t>(dist_per_ref_point / avg_point_spacing));
    
    // Bound stride to prevent issues with very dense or sparse paths
    stride = std::min(stride, std::max<size_t>(1, num_poses / 5));
    
    // Build reference trajectory covering full prediction horizon
    for (int i = 0; i < NUM_REF_POINTS; ++i) {
      size_t idx = (start_idx + i * stride) % num_poses;
      
      nmpc::ReferencePoint ref;
      ref.x = poses[idx].pose.position.x;
      ref.y = poses[idx].pose.position.y;
      
      // Extract yaw from pose orientation
      double qx = poses[idx].pose.orientation.x;
      double qy = poses[idx].pose.orientation.y;
      double qz = poses[idx].pose.orientation.z;
      double qw = poses[idx].pose.orientation.w;
      ref.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      
      // Reference speed - can be modulated based on path curvature
      ref.v = nominal_speed_;
      
      // Optional: reduce reference speed for high curvature sections
      if (i < NUM_REF_POINTS - 1) {
        size_t next_idx = (start_idx + (i + 1) * stride) % num_poses;
        double dx = poses[next_idx].pose.position.x - poses[idx].pose.position.x;
        double dy = poses[next_idx].pose.position.y - poses[idx].pose.position.y;
        double next_qz = poses[next_idx].pose.orientation.z;
        double next_qw = poses[next_idx].pose.orientation.w;
        double next_yaw = std::atan2(2.0 * next_qw * next_qz, 1.0 - 2.0 * next_qz * next_qz);
        
        // Proper angle normalization for dyaw
        double dyaw = next_yaw - ref.yaw;
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        dyaw = std::abs(dyaw);
        
        double seg_dist = std::sqrt(dx * dx + dy * dy);
        if (seg_dist > 0.01) {
          double curvature = dyaw / seg_dist;
          // Slow down for sharp curves
          if (curvature > 0.5) {
            ref.v = nominal_speed_ * std::max(0.3, 1.0 - curvature * 0.5);
          }
        }
      }
      
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
  
  /**
   * @brief Publish NMPC visualization (예측 궤적 + 레퍼런스 포인트)
   * 예측 궤적: 녹색 LINE_STRIP
   * 레퍼런스 포인트: 파란 SPHERE_LIST
   */
  void publishNMPCVisualization(
    const std::vector<nmpc::VehicleState>& predicted_trajectory,
    const std::vector<nmpc::ReferencePoint>& reference)
  {
    rclcpp::Time current_time = now();
    
    // 1. NMPC 예측 궤적 (녹색 라인)
    if (!predicted_trajectory.empty()) {
      visualization_msgs::msg::Marker traj_marker;
      traj_marker.header.frame_id = "map";
      traj_marker.header.stamp = current_time;
      traj_marker.ns = "nmpc_prediction";
      traj_marker.id = 0;
      traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      traj_marker.action = visualization_msgs::msg::Marker::ADD;
      
      traj_marker.scale.x = 0.08;  // 라인 두께
      
      traj_marker.color.r = 0.0f;
      traj_marker.color.g = 1.0f;
      traj_marker.color.b = 0.0f;
      traj_marker.color.a = 1.0f;
      
      traj_marker.pose.orientation.w = 1.0;
      
      for (const auto& state : predicted_trajectory) {
        geometry_msgs::msg::Point p;
        p.x = state.x;
        p.y = state.y;
        p.z = 0.1;
        traj_marker.points.push_back(p);
      }
      
      traj_marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // 200ms lifetime for stable visualization
      nmpc_trajectory_pub_->publish(traj_marker);
    }
    
    // 2. NMPC 레퍼런스 포인트 (파란 점들)
    if (!reference.empty()) {
      visualization_msgs::msg::Marker ref_marker;
      ref_marker.header.frame_id = "map";
      ref_marker.header.stamp = current_time;
      ref_marker.ns = "nmpc_reference";
      ref_marker.id = 0;
      ref_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      ref_marker.action = visualization_msgs::msg::Marker::ADD;
      
      ref_marker.scale.x = 0.15;
      ref_marker.scale.y = 0.15;
      ref_marker.scale.z = 0.15;
      
      ref_marker.color.r = 0.0f;
      ref_marker.color.g = 0.0f;
      ref_marker.color.b = 1.0f;
      ref_marker.color.a = 0.7f;
      
      ref_marker.pose.orientation.w = 1.0;
      
      for (const auto& ref_pt : reference) {
        geometry_msgs::msg::Point p;
        p.x = ref_pt.x;
        p.y = ref_pt.y;
        p.z = 0.05;
        ref_marker.points.push_back(p);
      }
      
      ref_marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // 200ms lifetime for stable visualization
      nmpc_reference_pub_->publish(ref_marker);
    }
  }

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nmpc_trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nmpc_reference_pub_;
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
  double max_steer_{0.436};  // 25 degrees in radians
  double prediction_horizon_{1.0};  // Store prediction horizon for reference building
  
  // A1/A2 Collision avoidance parameters
  double a1_threshold_{0.3};    // Reverse trigger at 30cm for safety
  double a2_threshold_{0.8};    // Side avoidance threshold (80cm)
  double a2_urgent_threshold_{0.4};  // Urgent threshold: sharp steering when < 0.4m
  double a1_side_factor_{0.8};
  double a2_max_steer_ratio_{0.7};
  double a2_urgent_steer_ratio_{1.0};  // Full steering in urgent mode
  double reverse_speed_{0.5};
  double reverse_duration_{0.8};
  double a1_steer_gain_{0.8};
  double a2_steer_gain_{0.6};
  double a2_urgent_steer_gain_{1.5};  // 1.5x gain when < 0.4m
  bool enable_collision_avoidance_{true};
  
  // Collision avoidance state
  bool is_reversing_{false};
  bool is_in_a1_zone_{false};
  rclcpp::Time reverse_start_time_;
  double last_obstacle_left_dist_{10.0};
  double last_obstacle_right_dist_{10.0};
  double last_obstacle_front_dist_{10.0};
  double last_obstacle_angle_{0.0};
  double last_steering_before_collision_{0.0};  // Track last steering direction before collision
  
  // Reference trajectory state (moved from static to member)
  mutable size_t last_closest_idx_{0};

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

