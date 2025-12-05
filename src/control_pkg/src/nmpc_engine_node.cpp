/**
 * @file nmpc_engine_node.cpp
 * @brief Nonlinear Model Predictive Controller (NMPC) for autonomous racing
 * 
 * This implementation uses a bicycle kinematic/dynamic model for prediction and
 * solves the optimization problem using iterative linearization (SQP-like approach).
 * 
 * Key Features (v2.0):
 * - Levenberg-Marquardt regularization for numerical stability (Status 3 prevention)
 * - Soft constraints with slack variables for infeasibility prevention (Status 4)
 * - Dynamic bicycle model with Pacejka tire model for high-speed operation
 * - Latency compensation for control delay handling
 * - Lateral tolerance tube for optimal racing lines
 * 
 * State: [x, y, yaw, v]
 * Control: [steering_angle, acceleration]
 * 
 * Reference: ForzaETH autonomous racing stack architecture
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
  double horizon_sec = 1.5;       // Prediction horizon in seconds (increased from 1.0 for better planning)
  int prediction_steps = 15;      // Number of prediction steps (N) (increased for finer resolution)
  double dt = 0.1;                // Time step for prediction (NOTE: computed as horizon_sec/prediction_steps in initialize)
  double wheelbase = 0.33;        // Vehicle wheelbase [m]
  
  // Vehicle dynamics parameters (for Dynamic Bicycle Model at high speed)
  double vehicle_mass = 3.5;      // Vehicle mass [kg] (F1TENTH typical)
  double vehicle_inertia = 0.04;  // Yaw moment of inertia [kg*m^2]
  double lf = 0.17;               // Front axle to CG distance [m]
  double lr = 0.16;               // Rear axle to CG distance [m]
  double tire_Bf = 2.5;           // Pacejka tire coefficient B (front)
  double tire_Cf = 1.3;           // Pacejka tire coefficient C (front)
  double tire_Df = 4.5;           // Pacejka tire coefficient D (front) - max lateral force
  double tire_Br = 2.5;           // Pacejka tire coefficient B (rear)
  double tire_Cr = 1.3;           // Pacejka tire coefficient C (rear)
  double tire_Dr = 4.5;           // Pacejka tire coefficient D (rear)
  double dynamic_model_threshold = 2.5;  // Speed threshold for switching to dynamic model [m/s]
  
  // Weights for cost function
  // Issue 3.1: Strengthen control rate (slew rate) penalty to prevent oscillation
  // The steering rate weight is critical for stability - must be significantly higher
  // than position/heading weights to prevent Bang-Bang control behavior
  double w_pos = 10.0;            // Position tracking weight (lateral error)
  double w_yaw = 10.0;            // Heading tracking weight (increased for stability)
  double w_vel = 3.0;             // Velocity tracking weight (increased)
  double w_steer = 0.5;           // Steering effort weight (reduced to allow necessary steering)
  double w_accel = 0.3;           // Acceleration effort weight
  double w_steer_rate = 600.0;    // Steering rate weight - CRITICAL for oscillation suppression (increased)
  double w_accel_rate = 60.0;     // Acceleration rate weight (smoothness)
  
  // Issue 5.1: Lateral error tube - allow deviation from centerline within bounds
  // This enables NMPC to find optimal racing lines within the tube
  double lateral_tolerance = 0.25; // Allow ±0.25m deviation from reference (Frenet tube)
  double w_terminal = 30.0;       // Terminal cost weight for better convergence (increased)
  
  // Soft Constraint settings (Issue 6.1: Prevent Status 4 - Infeasibility)
  // Slack variable weights for constraint violations
  double w_slack_track = 1000.0;  // High penalty for track boundary violations
  double w_slack_speed = 500.0;   // Penalty for speed constraint violations
  double track_boundary_margin = 0.05;  // Additional margin for track boundaries [m]
  
  // Constraints
  double max_steer = 0.6458;      // Maximum steering angle [rad] (37 degrees - increased from 25)
  double max_steer_rate = 1.8;    // Maximum steering rate [rad/s] (increased for responsiveness)
  double max_accel = 4.0;         // Maximum acceleration [m/s^2] (increased)
  double min_accel = -6.0;        // Maximum deceleration [m/s^2] (increased for safety)
  double max_speed = 6.0;         // Maximum speed [m/s] (increased)
  double min_speed = 0.0;         // Minimum speed [m/s]
  
  // Solver settings (Issue 3.1: Levenberg-Marquardt regularization)
  int max_iterations = 20;        // Maximum SQP iterations (increased for better convergence)
  double convergence_tol = 1e-5;  // Tighter convergence tolerance
  double levenberg_marquardt = 1e-2;  // L-M regularization for Hessian stability (Status 3 fix)
  double min_step_size = 1e-6;    // Minimum step size before declaring convergence
  
  // Issue 3.3: Latency compensation
  double latency_compensation_sec = 0.05;  // Default 50ms total system delay (increased for realistic compensation)
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

// Solver status codes (similar to acados)
enum class SolverStatus
{
  SUCCESS = 0,           // Optimal solution found
  MAX_ITER = 1,          // Maximum iterations reached
  INFEASIBLE = 2,        // Infeasible problem (Status 4 in acados)
  NUMERICAL_ERROR = 3,   // Numerical error (Status 3 in acados - NaN/singularity)
  MIN_STEP = 4           // Minimum step size reached
};

// MPC Solution
struct MPCSolution
{
  double steering = 0.0;
  double speed = 0.0;
  bool feasible = false;
  double cost = 0.0;
  int iterations = 0;
  SolverStatus status = SolverStatus::SUCCESS;
  std::vector<VehicleState> predicted_trajectory;  // NMPC 예측 궤적 (시각화용)
  double slack_violation = 0.0;  // Total soft constraint violation (for monitoring)
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
    
    // Reset solver state
    consecutive_failures_ = 0;
    last_solve_status_ = SolverStatus::SUCCESS;
    
    RCLCPP_INFO(logger, 
      "NMPC Solver initialized | horizon: %.2fs, steps: %d, dt: %.3fs, wheelbase: %.2fm",
      config_.horizon_sec, config_.prediction_steps, config_.dt, config_.wheelbase);
    RCLCPP_INFO(logger,
      "Weights | pos: %.1f, yaw: %.1f, vel: %.1f, steer_rate: %.1f (CRITICAL)",
      config_.w_pos, config_.w_yaw, config_.w_vel, config_.w_steer_rate);
    RCLCPP_INFO(logger,
      "Lateral tolerance tube: %.2fm, Terminal weight: %.1f",
      config_.lateral_tolerance, config_.w_terminal);
    RCLCPP_INFO(logger,
      "Solver settings | L-M regularization: %.1e, max_iter: %d, convergence_tol: %.1e",
      config_.levenberg_marquardt, config_.max_iterations, config_.convergence_tol);
    RCLCPP_INFO(logger,
      "Dynamic model threshold: %.2f m/s, Latency compensation: %.3fs",
      config_.dynamic_model_threshold, config_.latency_compensation_sec);
    
    return true;
  }
  
  /**
   * @brief Reset the solver state (call when solver fails repeatedly)
   */
  void resetSolver()
  {
    for (auto& u : u_prev_) {
      u.steering = 0.0;
      u.acceleration = 0.5;
    }
    consecutive_failures_ = 0;
    last_solve_status_ = SolverStatus::SUCCESS;
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
    solution.status = SolverStatus::SUCCESS;
    
    if (reference.empty()) {
      RCLCPP_WARN_THROTTLE(logger, *rclcpp::Clock::make_shared(), 2000,
        "NMPC: Empty reference trajectory");
      solution.status = SolverStatus::INFEASIBLE;
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
    
    // Resize if needed
    while (u.size() < static_cast<size_t>(config_.prediction_steps)) {
      u.push_back(ControlInput{0.0, 0.5});
    }
    while (u.size() > static_cast<size_t>(config_.prediction_steps)) {
      u.pop_back();
    }
    
    // Issue 3.1 Fix: Levenberg-Marquardt regularization and improved SQP
    // Iterative optimization with adaptive step size and regularization
    double prev_cost = std::numeric_limits<double>::max();
    double lambda = config_.levenberg_marquardt;  // Initial regularization
    double step_size = 1.0;
    bool numerical_error = false;
    
    for (int iter = 0; iter < config_.max_iterations; ++iter) {
      // Forward simulate with current control sequence (using compensated state)
      // Use dynamic model for high speed, kinematic for low speed
      std::vector<VehicleState> predicted_states = 
        (compensated_state.v > config_.dynamic_model_threshold) ?
        forwardSimulateDynamic(compensated_state, u) :
        forwardSimulate(compensated_state, u);
      
      // Check for NaN in predicted states (Status 3 prevention)
      for (const auto& state : predicted_states) {
        if (std::isnan(state.x) || std::isnan(state.y) || 
            std::isnan(state.yaw) || std::isnan(state.v)) {
          numerical_error = true;
          break;
        }
      }
      
      if (numerical_error) {
        RCLCPP_WARN_THROTTLE(logger, *rclcpp::Clock::make_shared(), 500,
          "NMPC: NaN detected in prediction, resetting solver");
        solution.status = SolverStatus::NUMERICAL_ERROR;
        resetSolver();
        consecutive_failures_++;
        break;
      }
      
      // Compute cost with soft constraints
      double cost = computeCostWithSoftConstraints(predicted_states, u, ref, solution.slack_violation);
      
      // Check for NaN cost
      if (std::isnan(cost) || std::isinf(cost)) {
        numerical_error = true;
        solution.status = SolverStatus::NUMERICAL_ERROR;
        lambda *= 10.0;  // Increase regularization
        continue;
      }
      
      // Check convergence
      double cost_change = std::abs(prev_cost - cost);
      if (cost_change < config_.convergence_tol) {
        solution.iterations = iter + 1;
        solution.status = SolverStatus::SUCCESS;
        break;
      }
      
      // Check minimum step size (Status 3/4 prevention)
      if (step_size < config_.min_step_size) {
        solution.iterations = iter + 1;
        solution.status = SolverStatus::MIN_STEP;
        RCLCPP_DEBUG_THROTTLE(logger, *rclcpp::Clock::make_shared(), 1000,
          "NMPC: Minimum step size reached at iteration %d", iter);
        break;
      }
      
      prev_cost = cost;
      solution.iterations = iter + 1;
      
      // Compute gradients and update controls with L-M regularization
      step_size = updateControlsWithRegularization(
        compensated_state, predicted_states, u, ref, lambda);
      
      // Adaptive regularization: increase if cost not decreasing well (use relative change)
      double relative_change = (prev_cost > 1e-6) ? cost_change / prev_cost : 1.0;
      if (relative_change < 0.1 && lambda < 1.0) {
        lambda *= 2.0;  // Slow convergence: increase regularization
      } else if (relative_change > 0.3 && lambda > 1e-4) {
        lambda *= 0.5;  // Good convergence: decrease regularization
      }
      
      solution.cost = cost;
    }
    
    // Handle solver failures
    if (numerical_error || solution.iterations >= config_.max_iterations) {
      if (solution.iterations >= config_.max_iterations) {
        solution.status = SolverStatus::MAX_ITER;
      }
      consecutive_failures_++;
      
      // If too many consecutive failures, reset the solver
      if (consecutive_failures_ > 5) {
        RCLCPP_WARN_THROTTLE(logger, *rclcpp::Clock::make_shared(), 1000,
          "NMPC: %d consecutive failures, resetting solver", consecutive_failures_);
        resetSolver();
      }
    } else {
      consecutive_failures_ = 0;
    }
    
    // Apply constraints (with soft constraint handling)
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
      solution.feasible = (solution.status == SolverStatus::SUCCESS || 
                           solution.status == SolverStatus::MAX_ITER ||
                           solution.status == SolverStatus::MIN_STEP);
    }
    
    last_solve_status_ = solution.status;
    return solution;
  }

private:
  MPCConfig config_;
  std::vector<ControlInput> u_prev_;
  int consecutive_failures_ = 0;
  SolverStatus last_solve_status_ = SolverStatus::SUCCESS;
  
  /**
   * @brief Forward simulate vehicle trajectory using kinematic bicycle model
   * Used for low-speed operation (v < dynamic_model_threshold)
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
      
      // Clamp steering to prevent numerical issues with tan()
      double clamped_steer = std::clamp(u.steering, -config_.max_steer * 0.99, config_.max_steer * 0.99);
      double tan_steer = std::tan(clamped_steer);
      
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
   * @brief Forward simulate using Dynamic Bicycle Model (for high-speed operation)
   * 
   * Accounts for tire slip angles using simplified Pacejka tire model.
   * Used when v > dynamic_model_threshold to capture tire slip effects.
   * 
   * State: [x, y, yaw, v, yaw_rate, slip_angle]
   * This simplified version integrates yaw_rate into the kinematic state.
   */
  std::vector<VehicleState> forwardSimulateDynamic(
    const VehicleState& initial_state,
    const std::vector<ControlInput>& controls) const
  {
    std::vector<VehicleState> states;
    states.reserve(controls.size() + 1);
    states.push_back(initial_state);
    
    VehicleState state = initial_state;
    double yaw_rate = 0.0;  // Initial yaw rate
    double slip_angle = 0.0;  // Vehicle slip angle (beta)
    
    const double& m = config_.vehicle_mass;
    const double& Iz = config_.vehicle_inertia;
    const double& lf = config_.lf;
    const double& lr = config_.lr;
    
    for (const auto& u : controls) {
      double v = std::max(state.v, 0.1);  // Avoid division by zero
      
      // Clamp steering for numerical stability
      double delta = std::clamp(u.steering, -config_.max_steer * 0.99, config_.max_steer * 0.99);
      
      // Compute tire slip angles
      // Front slip angle: alpha_f = delta - atan((v_y + lf * r) / v_x)
      // Rear slip angle: alpha_r = -atan((v_y - lr * r) / v_x)
      // Simplified: assume v_x ≈ v, v_y ≈ v * beta
      double alpha_f = delta - std::atan2(v * slip_angle + lf * yaw_rate, v);
      double alpha_r = -std::atan2(v * slip_angle - lr * yaw_rate, v);
      
      // Clamp slip angles to prevent extreme values
      alpha_f = std::clamp(alpha_f, -0.5, 0.5);
      alpha_r = std::clamp(alpha_r, -0.5, 0.5);
      
      // Pacejka Magic Formula for tire lateral forces
      // F_y = D * sin(C * atan(B * alpha))
      // This is the simplified (no E parameter) Magic Formula that captures
      // tire nonlinear behavior including saturation at large slip angles.
      double Fy_f = config_.tire_Df * std::sin(config_.tire_Cf * std::atan(config_.tire_Bf * alpha_f));
      double Fy_r = config_.tire_Dr * std::sin(config_.tire_Cr * std::atan(config_.tire_Br * alpha_r));
      
      // Dynamic equations (simplified, assuming constant v_x for this step)
      // m * a_y = F_yf + F_yr  (lateral acceleration)
      // Iz * r_dot = lf * F_yf - lr * F_yr  (yaw moment)
      
      double a_y = (Fy_f + Fy_r) / m;
      double r_dot = (lf * Fy_f - lr * Fy_r) / Iz;
      
      // Update slip angle (beta_dot = a_y/v - r)
      double beta_dot = a_y / v - yaw_rate;
      
      // Integrate states
      double cos_yaw_beta = std::cos(state.yaw + slip_angle);
      double sin_yaw_beta = std::sin(state.yaw + slip_angle);
      
      state.x += v * cos_yaw_beta * config_.dt;
      state.y += v * sin_yaw_beta * config_.dt;
      state.yaw += yaw_rate * config_.dt;
      state.v += u.acceleration * config_.dt;
      yaw_rate += r_dot * config_.dt;
      slip_angle += beta_dot * config_.dt;
      
      // Clamp yaw rate and slip angle
      yaw_rate = std::clamp(yaw_rate, -3.0, 3.0);
      slip_angle = std::clamp(slip_angle, -0.3, 0.3);
      
      // Normalize yaw
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
   * @brief Compute cost with soft constraints (Issue 6.1: Status 4 prevention)
   * 
   * Instead of hard constraints that cause infeasibility, we use soft constraints
   * with slack variables. Constraint violations are allowed but heavily penalized.
   * This ensures the solver always finds a solution, even if the solution
   * slightly violates constraints.
   * 
   * @param slack_violation Output: total slack violation for monitoring
   */
  double computeCostWithSoftConstraints(
    const std::vector<VehicleState>& states,
    const std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference,
    double& slack_violation) const
  {
    // Start with base cost
    double cost = computeCost(states, controls, reference);
    slack_violation = 0.0;
    
    // Add soft constraint violations (slack penalties)
    for (size_t i = 0; i < states.size(); ++i) {
      // Speed constraint violations (soft)
      double speed_excess_high = std::max(0.0, states[i].v - config_.max_speed);
      double speed_excess_low = std::max(0.0, config_.min_speed - states[i].v);
      double speed_slack = speed_excess_high + speed_excess_low;
      
      cost += config_.w_slack_speed * (speed_slack * speed_slack + 0.1 * speed_slack);
      slack_violation += speed_slack;
      
      // Track boundary violations would go here if we had track boundary info
      // For now, we use lateral tolerance as a soft constraint (already in base cost)
    }
    
    // Steering constraint violations (soft)
    for (size_t i = 0; i < controls.size(); ++i) {
      double steer_excess = std::max(0.0, std::abs(controls[i].steering) - config_.max_steer);
      cost += config_.w_slack_track * steer_excess * steer_excess;
      slack_violation += steer_excess;
      
      // Acceleration constraint violations
      double accel_excess_high = std::max(0.0, controls[i].acceleration - config_.max_accel);
      double accel_excess_low = std::max(0.0, config_.min_accel - controls[i].acceleration);
      double accel_slack = accel_excess_high + accel_excess_low;
      cost += config_.w_slack_speed * accel_slack * accel_slack;
      slack_violation += accel_slack;
      
      // Steering rate constraint violations (soft)
      if (i > 0) {
        double d_steer = std::abs(controls[i].steering - controls[i-1].steering);
        double max_d_steer = config_.max_steer_rate * config_.dt;
        double rate_excess = std::max(0.0, d_steer - max_d_steer);
        cost += config_.w_slack_track * rate_excess * rate_excess;
        slack_violation += rate_excess;
      }
    }
    
    return cost;
  }
  
  /**
   * @brief Update controls using gradient descent with Levenberg-Marquardt regularization
   * 
   * Issue 3.1 Fix: Levenberg-Marquardt regularization prevents numerical issues
   * when the Hessian becomes ill-conditioned (near singularity).
   * 
   * H_reg = H + lambda * I
   * 
   * This ensures the update step is always well-defined.
   * 
   * @return Actual step size taken (for adaptive regularization)
   */
  double updateControlsWithRegularization(
    const VehicleState& current_state,
    const std::vector<VehicleState>& /* predicted_states */,
    std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference,
    double lambda)
  {
    // Base learning rates (can be tuned for different scenarios)
    constexpr double BASE_LEARNING_RATE_STEER = 0.25;
    constexpr double BASE_LEARNING_RATE_ACCEL = 0.20;
    constexpr double GRADIENT_EPSILON = 1e-4;
    constexpr double MAX_STEP_STEER = 0.1;
    constexpr double MAX_STEP_ACCEL = 0.5;
    
    // Learning rates with L-M regularization factor
    // As lambda increases, steps become smaller (more conservative)
    double reg_factor = 1.0 / (1.0 + lambda);
    double learning_rate_steer = BASE_LEARNING_RATE_STEER * reg_factor;
    double learning_rate_accel = BASE_LEARNING_RATE_ACCEL * reg_factor;
    
    double total_step = 0.0;
    
    // Use batched gradient update for efficiency
    std::vector<double> grad_steer(controls.size(), 0.0);
    std::vector<double> grad_accel(controls.size(), 0.0);
    
    // Compute all gradients first
    double slack_dummy = 0.0;
    for (size_t i = 0; i < controls.size(); ++i) {
      // Numerical gradient for steering
      std::vector<ControlInput> u_plus = controls;
      std::vector<ControlInput> u_minus = controls;
      u_plus[i].steering += GRADIENT_EPSILON;
      u_minus[i].steering -= GRADIENT_EPSILON;
      
      // Use dynamic model for high speed
      std::vector<VehicleState> states_plus, states_minus;
      if (current_state.v > config_.dynamic_model_threshold) {
        states_plus = forwardSimulateDynamic(current_state, u_plus);
        states_minus = forwardSimulateDynamic(current_state, u_minus);
      } else {
        states_plus = forwardSimulate(current_state, u_plus);
        states_minus = forwardSimulate(current_state, u_minus);
      }
      
      double cost_plus = computeCostWithSoftConstraints(states_plus, u_plus, reference, slack_dummy);
      double cost_minus = computeCostWithSoftConstraints(states_minus, u_minus, reference, slack_dummy);
      grad_steer[i] = (cost_plus - cost_minus) / (2.0 * GRADIENT_EPSILON);
      
      // Check for NaN gradient (numerical issue)
      if (std::isnan(grad_steer[i]) || std::isinf(grad_steer[i])) {
        grad_steer[i] = 0.0;
      }
      
      // Numerical gradient for acceleration
      u_plus = controls;
      u_minus = controls;
      u_plus[i].acceleration += GRADIENT_EPSILON;
      u_minus[i].acceleration -= GRADIENT_EPSILON;
      
      if (current_state.v > config_.dynamic_model_threshold) {
        states_plus = forwardSimulateDynamic(current_state, u_plus);
        states_minus = forwardSimulateDynamic(current_state, u_minus);
      } else {
        states_plus = forwardSimulate(current_state, u_plus);
        states_minus = forwardSimulate(current_state, u_minus);
      }
      
      cost_plus = computeCostWithSoftConstraints(states_plus, u_plus, reference, slack_dummy);
      cost_minus = computeCostWithSoftConstraints(states_minus, u_minus, reference, slack_dummy);
      grad_accel[i] = (cost_plus - cost_minus) / (2.0 * GRADIENT_EPSILON);
      
      // Check for NaN gradient
      if (std::isnan(grad_accel[i]) || std::isinf(grad_accel[i])) {
        grad_accel[i] = 0.0;
      }
    }
    
    // Apply gradient descent updates
    for (size_t i = 0; i < controls.size(); ++i) {
      double step_steer = learning_rate_steer * grad_steer[i];
      double step_accel = learning_rate_accel * grad_accel[i];
      
      // Gradient clipping for stability
      step_steer = std::clamp(step_steer, -MAX_STEP_STEER, MAX_STEP_STEER);
      step_accel = std::clamp(step_accel, -MAX_STEP_ACCEL, MAX_STEP_ACCEL);
      
      controls[i].steering -= step_steer;
      controls[i].acceleration -= step_accel;
      
      total_step += std::abs(step_steer) + std::abs(step_accel);
    }
    
    return total_step;
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
    declare_parameter("prediction_horizon", 1.5);   // Increased for better planning
    declare_parameter("prediction_steps", 15);      // Increased for finer resolution
    declare_parameter("control_rate_hz", 50.0);
    declare_parameter("nominal_speed", 2.5);        // Slightly higher default
    declare_parameter("solver_wheelbase", 0.33);
    
    // MPC weights - Issue 3.1: steering rate is critical
    declare_parameter("w_pos", 10.0);
    declare_parameter("w_yaw", 10.0);          // Increased for stability
    declare_parameter("w_vel", 3.0);           // Increased for better velocity tracking
    declare_parameter("w_steer", 0.5);         // Reduced to allow necessary steering
    declare_parameter("w_accel", 0.3);
    declare_parameter("w_steer_rate", 600.0);  // CRITICAL: Higher value prevents oscillation
    declare_parameter("w_accel_rate", 60.0);
    declare_parameter("w_terminal", 30.0);     // Terminal cost for convergence
    
    // Issue 5.1: Lateral tolerance tube
    declare_parameter("lateral_tolerance", 0.25);  // Allow ±0.25m deviation from centerline
    
    // Issue 3.3: Latency compensation
    declare_parameter("latency_compensation_sec", 0.05);  // 50ms total system delay
    
    // Solver parameters (Issue 3.1: Levenberg-Marquardt)
    declare_parameter("levenberg_marquardt", 0.01);       // L-M regularization for Hessian stability
    declare_parameter("max_solver_iterations", 20);       // Maximum iterations
    
    // Dynamic model parameters
    declare_parameter("dynamic_model_threshold", 2.5);    // Speed for switching to dynamic model
    declare_parameter("vehicle_mass", 3.5);               // Vehicle mass [kg]
    declare_parameter("vehicle_inertia", 0.04);           // Yaw moment of inertia [kg*m^2]
    
    // Constraints
    declare_parameter("max_steer", 0.6458);  // 37 degrees in radians (Changed from 25)
    declare_parameter("max_steer_rate", 1.8);  // rad/s (increased)
    declare_parameter("max_speed", 6.0);       // Increased max speed
    declare_parameter("max_accel", 4.0);       // Increased acceleration
    declare_parameter("min_accel", -6.0);      // Increased deceleration
    
    // Topic parameters
    declare_parameter<std::string>("odom_topic", "/dual_ekf/global_odom");  // Use Dual EKF output
    declare_parameter<std::string>("path_topic", "/global_raceline");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("scan_topic", "/scan");  // LiDAR topic
    declare_parameter<bool>("use_dual_ekf", true);  // Toggle between Dual EKF and ground truth
    
    // A1/A2 collision avoidance parameters
    // A1: STRONG SHORT BURST reverse when hitting front obstacle
    // A2: Steering avoidance only (NO SLOWDOWN per user request)
    declare_parameter("a1_threshold", 0.3);            // [m] Front obstacle distance to trigger reverse
    declare_parameter("a2_threshold", 0.4);            // [m] Distance to trigger steering avoidance
    declare_parameter("a2_urgent_threshold", 0.25);    // [m] Distance for urgent avoidance
    declare_parameter("a2_max_steer_ratio", 1.0);      // Full steering allowed
    declare_parameter("a2_urgent_steer_ratio", 1.0);   // Full steering in urgent mode
    declare_parameter("reverse_speed", 2.5);           // [m/s] STRONG reverse speed
    declare_parameter("reverse_duration", 0.15);       // [s] VERY SHORT burst duration
    declare_parameter("a1_steer_gain", 1.0);           // Full steering during reverse
    declare_parameter("a2_steer_gain", 1.5);           // High repulsive force gain
    declare_parameter("a2_urgent_steer_gain", 2.0);    // Very high urgent gain
    declare_parameter("enable_collision_avoidance", true);

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
    RCLCPP_INFO(get_logger(), "A1/A2 Collision avoidance: A1=%.2fm (reverse), A2=%.2fm (repulsive force), enabled=%s",
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
    
    // Solver settings (Issue 3.1: Levenberg-Marquardt)
    cfg.levenberg_marquardt = get_parameter("levenberg_marquardt").as_double();
    cfg.max_iterations = get_parameter("max_solver_iterations").as_int();
    
    // Dynamic model parameters
    cfg.dynamic_model_threshold = get_parameter("dynamic_model_threshold").as_double();
    cfg.vehicle_mass = get_parameter("vehicle_mass").as_double();
    cfg.vehicle_inertia = get_parameter("vehicle_inertia").as_double();
    
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
    RCLCPP_INFO(get_logger(), "  - L-M regularization: %.1e (Status 3 prevention)", cfg.levenberg_marquardt);
    RCLCPP_INFO(get_logger(), "  - Dynamic model threshold: %.2f m/s", cfg.dynamic_model_threshold);

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
    
    // A1 zone check: STRONG SHORT BURST reverse when hitting front obstacle
    if (checkA1Zone()) {
      // Start or continue reversing
      if (!is_reversing_) {
        is_reversing_ = true;
        reverse_start_time_ = current_time;
        RCLCPP_WARN(get_logger(), 
          "A1 COLLISION! Front=%.2fm < %.2fm -> BURST REVERSE (speed=%.1f, duration=%.2fs)",
          last_obstacle_front_dist_, a1_threshold_, reverse_speed_, reverse_duration_);
      }
      
      double elapsed = (current_time - reverse_start_time_).seconds();
      if (elapsed < reverse_duration_) {
        // STRONG SHORT BURST: Apply maximum reverse force
        double reverse_steer = computeReverseSteering();
        
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = current_time;
        cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
        cmd.drive.speed = -reverse_speed_;  // Strong negative speed
        cmd.drive.steering_angle = reverse_steer;
        
        drive_pub_->publish(cmd);
        
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 50,  // Log every 50ms for short burst
          "BURST REVERSE: speed=%.2f, steer=%.3f, %.0f%%",
          -reverse_speed_, reverse_steer, (elapsed / reverse_duration_) * 100.0);
        return;
      } else {
        // Burst complete
        is_reversing_ = false;
        RCLCPP_INFO(get_logger(), "Burst reverse complete, resuming forward");
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
    
    // Log solver status for debugging (Status 3/4 monitoring)
    if (solution.status != nmpc::SolverStatus::SUCCESS) {
      const char* status_str = "UNKNOWN";
      switch (solution.status) {
        case nmpc::SolverStatus::MAX_ITER: status_str = "MAX_ITER"; break;
        case nmpc::SolverStatus::INFEASIBLE: status_str = "INFEASIBLE (Status 4)"; break;
        case nmpc::SolverStatus::NUMERICAL_ERROR: status_str = "NUMERICAL_ERROR (Status 3)"; break;
        case nmpc::SolverStatus::MIN_STEP: status_str = "MIN_STEP"; break;
        default: break;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "NMPC Solver: %s after %d iterations, slack_violation=%.4f",
        status_str, solution.iterations, solution.slack_violation);
    }
    
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
      // Note: Speed slowdown in A2 zone removed per user request
      // Side obstacles should not cause speed reduction
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
        "A2 Zone: adding avoidance steer=%.3f (no slowdown)", 
        delta_a2_avoidance);
    }
    
    // Publish NMPC visualization (예측 궤적 + 레퍼런스)
    publishNMPCVisualization(solution.predicted_trajectory, reference);
    
    // Publish result
    publishDriveCommand(solution);
    
    // Debug logging using RCLCPP_INFO_THROTTLE for thread safety
    const char* model_type = (current_state.v > 2.5) ? "DYN" : "KIN";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,  // Every 1 second
      "MPC[%s]: state(%.2f, %.2f, %.1f°, %.2f m/s) -> cmd(steer=%.3f, speed=%.2f) [%d iters, cost=%.1f]",
      model_type, current_state.x, current_state.y, current_state.yaw * 180.0 / M_PI, current_state.v,
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
    
    // Only check front obstacle for A1 zone (reverse trigger)
    // Side obstacles should not trigger reverse - per user request
    is_in_a1_zone_ = (last_obstacle_front_dist_ < a1_threshold_);
    
    if (is_in_a1_zone_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "A1 Zone! Front: %.2fm (threshold: %.2fm) - triggering reverse",
        last_obstacle_front_dist_, a1_threshold_);
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
   * @brief Compute avoidance steering (A2 zone) - Repulsive Force based
   * @param lookahead_angle 차량 프레임 기준 lookahead point 방향 각도 (rad) - used for logging
   * 
   * Uses repulsive force approach: obstacles push the car away in the opposite direction.
   * - Left obstacle closer → steer right (negative steering)
   * - Right obstacle closer → steer left (positive steering)
   * - Force magnitude inversely proportional to distance (closer = stronger push)
   * 
   * This is additive to NMPC steering, creating a smooth blend:
   * final_steer = nmpc_steer + repulsive_force
   */
  double computeAvoidanceSteering(double lookahead_angle)
  {
    double left_dist = std::min(last_obstacle_left_dist_, 5.0);
    double right_dist = std::min(last_obstacle_right_dist_, 5.0);
    double front_dist = std::min(last_obstacle_front_dist_, 5.0);
    double min_side_dist = std::min(left_dist, right_dist);
    double min_dist = std::min(min_side_dist, front_dist);
    
    // No avoidance needed if no obstacles within A2 threshold
    if (min_dist > a2_threshold_) {
      return 0.0;
    }
    
    // === Repulsive Force Calculation ===
    // Force from each side: F = gain * (1/dist - 1/threshold) when dist < threshold
    // This creates smooth force that increases as obstacle gets closer
    
    double repulsive_steer = 0.0;
    
    // Left obstacle repulsive force (pushes right = negative steering)
    if (left_dist < a2_threshold_) {
      // Repulsive force: stronger when closer (inverse relationship)
      double left_force = std::max(0.0, (a2_threshold_ - left_dist) / a2_threshold_);  // 0 at threshold, 1 at 0
      left_force = left_force * left_force;  // Quadratic for stronger response at close range
      repulsive_steer -= left_force * a2_steer_gain_ * max_steer_;  // Push right (negative)
    }
    
    // Right obstacle repulsive force (pushes left = positive steering)
    if (right_dist < a2_threshold_) {
      double right_force = std::max(0.0, (a2_threshold_ - right_dist) / a2_threshold_);
      right_force = right_force * right_force;  // Quadratic
      repulsive_steer += right_force * a2_steer_gain_ * max_steer_;  // Push left (positive)
    }
    
    // Front obstacle: steer toward more open side
    if (front_dist < a2_threshold_) {
      double front_force = std::max(0.0, (a2_threshold_ - front_dist) / a2_threshold_);
      front_force = front_force * front_force;
      // Steer toward the more open side
      if (left_dist > right_dist) {
        repulsive_steer += front_force * a2_steer_gain_ * max_steer_ * 0.5;  // Turn left
      } else {
        repulsive_steer -= front_force * a2_steer_gain_ * max_steer_ * 0.5;  // Turn right
      }
    }
    
    // Urgent zone: amplify repulsive force
    bool is_urgent = min_dist < a2_urgent_threshold_;
    if (is_urgent) {
      repulsive_steer *= (a2_urgent_steer_gain_ / a2_steer_gain_);  // Apply urgent multiplier
    }
    
    // Clamp to maximum allowed steering
    double max_avoidance = max_steer_ * (is_urgent ? a2_urgent_steer_ratio_ : a2_max_steer_ratio_);
    repulsive_steer = std::clamp(repulsive_steer, -max_avoidance, max_avoidance);
    
    // Log repulsive force avoidance
    if (std::abs(repulsive_steer) > 0.01) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), is_urgent ? 200 : 500,
        "[%s REPULSE] steer=%.3f | L:%.2f R:%.2f F:%.2f (threshold:%.2f)",
        is_urgent ? "URGENT" : "AVOID",
        repulsive_steer, left_dist, right_dist, front_dist, a2_threshold_);
    }
    
    // Suppress unused parameter warning
    (void)lookahead_angle;
    
    return repulsive_steer;
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
  double a1_threshold_{0.3};          // [m] Front obstacle reverse trigger
  double a2_threshold_{0.4};          // [m] Steering avoidance threshold
  double a2_urgent_threshold_{0.25};  // [m] Urgent avoidance threshold
  double a2_max_steer_ratio_{1.0};    // Full steering allowed
  double a2_urgent_steer_ratio_{1.0}; // Full steering in urgent mode
  double reverse_speed_{2.5};         // [m/s] Strong burst reverse
  double reverse_duration_{0.15};     // [s] Short burst duration
  double a1_steer_gain_{1.0};         // Full steering during reverse
  double a2_steer_gain_{1.5};         // High repulsive force gain
  double a2_urgent_steer_gain_{2.0};  // Very high urgent gain
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

