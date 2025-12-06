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
 * Key Features (v3.0) - Overtaking and Collision System:
 * - Simplified collision handling: stop -> wait -> straight reverse (no spinning)
 * - Opponent detection and following with speed limiting
 * - Global overtaking path generation with visualization
 * - Overtaking decision and commitment system
 * 
 * Key Features (v4.0) - Refined Driving Philosophy:
 * - Curvature-based speed control: slow down for corners based on raceline curvature
 * - Explicit OVERTAKE ZONEs with s-intervals for controlled overtaking
 * - Improved FOLLOW behavior with stable speed matching (v_follow = opponent - margin)
 * - Committed OVERTAKE episodes with clear speed boost
 * - Simplified collision: just stop stably (no aggressive reverse)
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
#include <visualization_msgs/msg/marker_array.hpp>
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
  double w_vel = 5.0;             // Velocity tracking weight (v4.0: increased for strong speed control)
  double w_steer = 0.5;           // Steering effort weight (reduced to allow necessary steering)
  double w_accel = 0.3;           // Acceleration effort weight
  double w_steer_rate = 600.0;    // Steering rate weight - CRITICAL for oscillation suppression (increased)
  double w_accel_rate = 60.0;     // Acceleration rate weight (smoothness)
  
  // Issue 5.1: Lateral error tube - allow deviation from centerline within bounds
  // This enables NMPC to find optimal racing lines within the tube
  double lateral_tolerance = 0.25; // Allow ±0.25m deviation from reference (Frenet tube)
  double w_terminal = 30.0;       // Terminal cost weight for better convergence (increased)
  
  // Soft Constraint settings (v4.0: Tuned for better convergence during overtaking)
  // Slack variable weights for constraint violations
  // Slightly reduced penalties allow solver to find feasible solutions more easily
  double w_slack_track = 800.0;   // Penalty for track boundary violations (reduced from 1000)
  double w_slack_speed = 400.0;   // Penalty for speed constraint violations (reduced from 500)
  double track_boundary_margin = 0.05;  // Additional margin for track boundaries [m]
  
  // Constraints
  double max_steer = 0.6458;      // Maximum steering angle [rad] (37 degrees - increased from 25)
  double max_steer_rate = 1.8;    // Maximum steering rate [rad/s] (increased for responsiveness)
  double max_accel = 4.0;         // Maximum acceleration [m/s^2] (increased)
  double min_accel = -6.0;        // Maximum deceleration [m/s^2] (increased for safety)
  double max_speed = 6.0;         // Maximum speed [m/s] (increased)
  double min_speed = 0.0;         // Minimum speed [m/s]
  
  // Solver settings (v4.0: Improved convergence for overtaking maneuvers)
  int max_iterations = 25;        // Maximum SQP iterations (increased for complex maneuvers)
  double convergence_tol = 5e-5;  // Slightly relaxed for faster convergence during overtake
  double levenberg_marquardt = 2e-2;  // L-M regularization (increased for stability)
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
    
    // Handle solver failures (v4.0: more graceful handling during complex maneuvers)
    if (numerical_error || solution.iterations >= config_.max_iterations) {
      if (solution.iterations >= config_.max_iterations) {
        solution.status = SolverStatus::MAX_ITER;
        // MAX_ITER is not a critical failure during overtaking - still use the solution
      }
      consecutive_failures_++;
      
      // If too many consecutive failures, reset the solver
      // Increased threshold from 5 to 8 for better stability during complex maneuvers
      if (consecutive_failures_ > 8) {
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
  // Named constants for magic numbers (code review feedback)
  static constexpr double MIN_CREEP_SPEED = 0.3;              // [m/s] Minimum speed when following
  static constexpr double BRIEF_STOP_DURATION = 0.1;          // [s] Brief stop before transitioning states
  static constexpr double OVERTAKE_COMPLETION_TIMEOUT = 2.0;  // [s] Time to complete overtake after passing
  static constexpr double OVERTAKE_SAFETY_MARGIN = 0.3;       // [m] Extra clearance margin for overtaking
  static constexpr double DEFAULT_OBSTACLE_SPEED = 0.5;       // [m/s] Assumed speed for LiDAR-detected obstacles
  
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
    
    // Simplified Collision Avoidance System (v3.0)
    // When hitting wall/obstacle:
    // 1. STOP immediately
    // 2. Wait briefly
    // 3. Reverse straight back (no complex steering - fixes spinning issue)
    declare_parameter("a1_threshold", 0.3);            // [m] Front obstacle distance to trigger collision
    declare_parameter("a2_threshold", 0.4);            // [m] Distance to trigger avoidance steering
    declare_parameter("a2_urgent_threshold", 0.25);    // [m] Distance for urgent avoidance
    declare_parameter("a2_max_steer_ratio", 1.0);      // Full steering allowed
    declare_parameter("a2_urgent_steer_ratio", 1.0);   // Full steering in urgent mode
    declare_parameter("reverse_speed", 1.5);           // [m/s] Reverse speed (reduced for safety)
    declare_parameter("reverse_duration", 0.5);        // [s] Reverse duration
    declare_parameter("stop_duration", 0.3);           // [s] Stop duration before reversing
    declare_parameter("a1_steer_gain", 1.0);           // Steering during reverse (unused in simplified mode)
    declare_parameter("a2_steer_gain", 1.5);           // High repulsive force gain
    declare_parameter("a2_urgent_steer_gain", 2.0);    // Very high urgent gain
    declare_parameter("enable_collision_avoidance", true);
    
    // Opponent Following and Overtaking System (v4.0 - Refined)
    declare_parameter("opponent_detection_range", 3.0);    // [m] Range to detect opponent ahead
    declare_parameter("opponent_following_distance", 1.0); // [m] Safe following distance
    declare_parameter("overtake_path_width", 0.8);         // [m] Lateral offset for overtaking
    declare_parameter("overtake_decision_distance", 2.5);  // [m] Distance to start considering overtake
    declare_parameter("enable_overtaking", true);          // Enable/disable overtaking system
    declare_parameter("opponent_speed_tracking_gain", 0.8); // Speed limiting when following (0.8 = 80% of opponent speed)
    
    // FOLLOW behavior parameters (v4.0 - Stable following)
    declare_parameter("follow_margin", 0.05);              // [m/s] Speed margin below opponent when following
    declare_parameter("v_min_follow", 0.3);                // [m/s] Minimum follow speed (avoid crawling)
    
    // OVERTAKE parameters (v4.0 - Committed overtaking)
    declare_parameter("overtake_boost", 0.25);             // [m/s] Speed boost above opponent during overtake
    declare_parameter("overtake_min_commit_time", 2.0);    // [s] Min time to commit to overtake before giving up
    declare_parameter("overtake_completion_timeout", 3.0); // [s] Time after passing to confirm overtake complete
    
    // Overtake feasibility parameters (v5.0 - Stricter overtake decision)
    // These ensure overtakes only start when clearly feasible given opponent geometry
    declare_parameter("overtake_opponent_width", 0.35);    // [m] Assumed opponent vehicle width
    declare_parameter("overtake_width_factor", 1.2);       // Factor multiplied by opponent width for clearance
    declare_parameter("overtake_safety_margin", 0.25);     // [m] Additional safety margin for overtake
    declare_parameter("overtake_min_longitudinal_window", 5.0); // [m] Min distance to complete overtake
    
    // Curvature-based speed control (v4.0 - Corner behavior)
    declare_parameter("curvature_k1", 0.2);                // [1/m] Curvature threshold: below = straight
    declare_parameter("curvature_k2", 0.8);                // [1/m] Curvature threshold: above = tight corner
    declare_parameter("v_max_straight", 3.0);              // [m/s] Speed on straights
    declare_parameter("v_min_corner", 1.2);                // [m/s] Speed in tight corners
    declare_parameter("curvature_lookahead", 5);           // [points] Lookahead for curvature calculation
    
    // Corner exit smoothing parameters (v5.0)
    // These control how the car behaves when exiting a corner and rejoining the straight
    declare_parameter("corner_exit_wall_margin", 0.4);     // [m] Min distance from wall on corner exit
    declare_parameter("corner_exit_lateral_limit", 0.15);  // [m] Max lateral deviation during exit transition
    declare_parameter("corner_exit_steer_rate_limit", 0.8);// [ratio] Steering rate limit factor (0-1) on exit
    declare_parameter("corner_exit_transition_time", 1.0); // [s] Time to transition from corner to straight
    declare_parameter("enable_corner_exit_smoothing", true);// Enable/disable corner exit smoothing
    
    // Overtake zones (s-intervals on raceline where overtaking is allowed)
    // Format: comma-separated pairs of start,end s-values
    declare_parameter<std::vector<double>>("overtake_zone_starts", std::vector<double>{0.0});
    declare_parameter<std::vector<double>>("overtake_zone_ends", std::vector<double>{1000.0}); // Full track by default

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
    stop_duration_ = get_parameter("stop_duration").as_double();
    a1_steer_gain_ = get_parameter("a1_steer_gain").as_double();
    a2_steer_gain_ = get_parameter("a2_steer_gain").as_double();
    a2_urgent_steer_gain_ = get_parameter("a2_urgent_steer_gain").as_double();
    enable_collision_avoidance_ = get_parameter("enable_collision_avoidance").as_bool();
    
    // Get opponent following and overtaking parameters
    opponent_detection_range_ = get_parameter("opponent_detection_range").as_double();
    opponent_following_distance_ = get_parameter("opponent_following_distance").as_double();
    overtake_path_width_ = get_parameter("overtake_path_width").as_double();
    overtake_decision_distance_ = get_parameter("overtake_decision_distance").as_double();
    enable_overtaking_ = get_parameter("enable_overtaking").as_bool();
    opponent_speed_tracking_gain_ = get_parameter("opponent_speed_tracking_gain").as_double();
    
    // Get v4.0 FOLLOW and OVERTAKE parameters
    follow_margin_ = get_parameter("follow_margin").as_double();
    v_min_follow_ = get_parameter("v_min_follow").as_double();
    overtake_boost_ = get_parameter("overtake_boost").as_double();
    overtake_min_commit_time_ = get_parameter("overtake_min_commit_time").as_double();
    overtake_completion_timeout_ = get_parameter("overtake_completion_timeout").as_double();
    
    // Get v5.0 overtake feasibility parameters
    overtake_opponent_width_ = get_parameter("overtake_opponent_width").as_double();
    overtake_width_factor_ = get_parameter("overtake_width_factor").as_double();
    overtake_safety_margin_ = get_parameter("overtake_safety_margin").as_double();
    overtake_min_longitudinal_window_ = get_parameter("overtake_min_longitudinal_window").as_double();
    
    // Get v4.0 curvature-based speed control parameters
    curvature_k1_ = get_parameter("curvature_k1").as_double();
    curvature_k2_ = get_parameter("curvature_k2").as_double();
    v_max_straight_ = get_parameter("v_max_straight").as_double();
    v_min_corner_ = get_parameter("v_min_corner").as_double();
    curvature_lookahead_ = get_parameter("curvature_lookahead").as_int();
    
    // Get v5.0 corner exit smoothing parameters
    corner_exit_wall_margin_ = get_parameter("corner_exit_wall_margin").as_double();
    corner_exit_lateral_limit_ = get_parameter("corner_exit_lateral_limit").as_double();
    corner_exit_steer_rate_limit_ = get_parameter("corner_exit_steer_rate_limit").as_double();
    corner_exit_transition_time_ = get_parameter("corner_exit_transition_time").as_double();
    enable_corner_exit_smoothing_ = get_parameter("enable_corner_exit_smoothing").as_bool();
    
    // Get overtake zone definitions
    overtake_zone_starts_ = get_parameter("overtake_zone_starts").as_double_array();
    overtake_zone_ends_ = get_parameter("overtake_zone_ends").as_double_array();
    
    // Calculate recovery cooldown as 2x the reverse duration
    // This prevents immediate re-triggering of reverse after completion
    recovery_cooldown_duration_ = reverse_duration_ * 2.0;
    
    // If not using Dual EKF, use ground truth
    if (!use_dual_ekf_) {
      odom_topic = "/car_state/odom_GT";
    }

    RCLCPP_INFO(get_logger(), "=== NMPC Engine Node v4.0 Initialized ===");
    RCLCPP_INFO(get_logger(), "Prediction horizon: %.2fs, steps: %d", prediction_horizon, prediction_steps);
    RCLCPP_INFO(get_logger(), "Using %s for state estimation", use_dual_ekf_ ? "Dual EKF" : "Ground Truth");
    RCLCPP_INFO(get_logger(), "Topics - odom: %s, path: %s, drive: %s", 
                odom_topic.c_str(), path_topic.c_str(), drive_topic.c_str());
    RCLCPP_INFO(get_logger(), "Collision System: A1=%.2fm (stop), stop=%.2fs, reverse=%.2fs",
                a1_threshold_, stop_duration_, reverse_duration_);
    RCLCPP_INFO(get_logger(), "Curvature Speed: k1=%.2f, k2=%.2f, v_straight=%.1fm/s, v_corner=%.1fm/s",
                curvature_k1_, curvature_k2_, v_max_straight_, v_min_corner_);
    RCLCPP_INFO(get_logger(), "FOLLOW: margin=%.2fm/s, v_min=%.2fm/s",
                follow_margin_, v_min_follow_);
    RCLCPP_INFO(get_logger(), "OVERTAKE: boost=%.2fm/s, commit_time=%.1fs, zones=%zu",
                overtake_boost_, overtake_min_commit_time_, overtake_zone_starts_.size());

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
    
    // Overtaking paths visualization publisher (MAGENTA for left, CYAN for right)
    overtake_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/overtake_paths", rclcpp::QoS(1).transient_local());
    
    // Opponent detection marker publisher (RED sphere)
    opponent_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/detected_opponent", rclcpp::QoS(1));
    
    // Subscribe to opponent odometry for speed tracking
    opponent_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/opp_racecar/odom", rclcpp::QoS(10).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_opponent_odom_ = msg;
      });
    
    RCLCPP_INFO(get_logger(), "NMPC visualization: /nmpc_predicted_trajectory (GREEN), /nmpc_reference_points (BLUE)");
    RCLCPP_INFO(get_logger(), "Overtaking visualization: /overtake_paths (MAGENTA/CYAN), /detected_opponent (RED)");

    // Setup control timer
    const double period_s = 1.0 / std::max(1.0, control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s),
      std::bind(&NMPCEngineNode::controlCycle, this));
    
    // Initialize time stamps
    collision_state_start_time_ = now();
    overtake_start_time_ = now();
    corner_exit_start_time_ = now();
    
    RCLCPP_INFO(get_logger(), "NMPC control loop running at %.1f Hz", control_rate_hz_);
    RCLCPP_INFO(get_logger(), "Corner exit smoothing: %s (margin=%.2fm, lateral_limit=%.2fm, steer_limit=%.1f%%)",
                enable_corner_exit_smoothing_ ? "ENABLED" : "DISABLED",
                corner_exit_wall_margin_, corner_exit_lateral_limit_, corner_exit_steer_rate_limit_ * 100);
  }

private:
  /**
   * @brief Collision state machine states
   */
  enum class CollisionState {
    NORMAL,           // Normal driving
    STOPPING,         // Detected collision, stopping
    WAITING,          // Stopped, waiting before reverse
    REVERSING,        // Reversing straight back
    COOLDOWN          // Recovery cooldown period
  };
  
  /**
   * @brief Driving mode for high-level behavior (v4.0)
   * 
   * CRUISE: Normal driving on global raceline at target speed
   * FOLLOW: Safe following behind opponent with similar speed
   * OVERTAKE_CANDIDATE: In overtake zone, checking if safe to overtake
   * OVERTAKE: Committed overtake using precomputed trajectory
   * OBSTACLE_STOP: Safe stop due to collision / extremely close obstacle
   */
  enum class DrivingMode {
    CRUISE,
    FOLLOW,
    OVERTAKE_CANDIDATE,
    OVERTAKE,
    OBSTACLE_STOP
  };
  
  /**
   * @brief Main control cycle - called at control_rate_hz
   * 
   * Implements simplified collision handling:
   * 1. NORMAL: Regular NMPC control
   * 2. STOPPING: Stop when hitting wall (speed = 0)
   * 3. WAITING: Wait briefly (no movement)
   * 4. REVERSING: Reverse straight back (steer = 0, speed < 0)
   * 5. COOLDOWN: Brief cooldown before returning to normal
   * 
   * Also implements opponent following and overtaking logic.
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
    
    // Update obstacle distances from LiDAR
    updateObstacleDistances();
    
    // Detect opponent ahead and get opponent info
    OpponentInfo opponent = detectOpponent();
    
    // Publish opponent visualization
    publishOpponentVisualization(opponent);
    
    // Generate and publish overtaking paths if enabled
    if (enable_overtaking_ && latest_path_) {
      generateAndPublishOvertakePaths();
    }
    
    // ========================================
    // SIMPLIFIED COLLISION STATE MACHINE
    // ========================================
    // States: NORMAL -> STOPPING -> WAITING -> REVERSING -> COOLDOWN -> NORMAL
    // This removes all complex steering during collision, fixing the spinning issue
    
    double elapsed = (current_time - collision_state_start_time_).seconds();
    
    switch (collision_state_) {
      case CollisionState::NORMAL:
        // v4.0: Use smaller collision threshold (0.20-0.25m) for very close detection
        // Only trigger on front collision, not side walls
        if (enable_collision_avoidance_ && last_obstacle_front_dist_ < a1_threshold_) {
          collision_state_ = CollisionState::STOPPING;
          collision_state_start_time_ = current_time;
          RCLCPP_WARN(get_logger(), 
            "COLLISION DETECTED! Front=%.2fm < %.2fm -> SIMPLE STOP",
            last_obstacle_front_dist_, a1_threshold_);
        }
        break;
        
      case CollisionState::STOPPING:
        {
          // v4.0: Simple STOP - speed=0, steer=0 (straight)
          // No bouncing, no spinning, no overshooting
          ackermann_msgs::msg::AckermannDriveStamped cmd;
          cmd.header.stamp = current_time;
          cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
          cmd.drive.speed = 0.0;
          cmd.drive.steering_angle = 0.0;
          drive_pub_->publish(cmd);
          
          // After brief stop, transition to waiting
          if (elapsed > BRIEF_STOP_DURATION) {
            collision_state_ = CollisionState::WAITING;
            collision_state_start_time_ = current_time;
            RCLCPP_INFO(get_logger(), "Stopped. Holding for %.2fs...", stop_duration_);
          }
          return;
        }
        
      case CollisionState::WAITING:
        {
          // v4.0: Hold stopped state for stability
          ackermann_msgs::msg::AckermannDriveStamped cmd;
          cmd.header.stamp = current_time;
          cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
          cmd.drive.speed = 0.0;
          cmd.drive.steering_angle = 0.0;
          drive_pub_->publish(cmd);
          
          // v4.0: After stop_duration, check if we need to reverse or can resume
          if (elapsed > stop_duration_) {
            // Check if still blocked - only reverse if absolutely necessary
            if (last_obstacle_front_dist_ < a1_threshold_ * 0.8) {
              // Still very close - gentle reverse
              collision_state_ = CollisionState::REVERSING;
              collision_state_start_time_ = current_time;
              RCLCPP_INFO(get_logger(), "Still blocked. Gentle reverse for %.2fs...", reverse_duration_);
            } else {
              // Space cleared - go to cooldown
              collision_state_ = CollisionState::COOLDOWN;
              collision_state_start_time_ = current_time;
              RCLCPP_INFO(get_logger(), "Space cleared. Resuming after cooldown.");
            }
          }
          return;
        }
        
      case CollisionState::REVERSING:
        {
          // v4.0: Gentle straight reverse - reduced speed for stability
          ackermann_msgs::msg::AckermannDriveStamped cmd;
          cmd.header.stamp = current_time;
          cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
          // Use reduced reverse speed (half the configured value) for gentler recovery
          cmd.drive.speed = -reverse_speed_ * 0.5;
          cmd.drive.steering_angle = 0.0;  // STRAIGHT BACK - no steering
          drive_pub_->publish(cmd);
          
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
            "GENTLE REVERSE: speed=%.2f, steer=0.0", cmd.drive.speed);
          
          // Short reverse duration
          if (elapsed > reverse_duration_ * 0.5) {
            collision_state_ = CollisionState::COOLDOWN;
            collision_state_start_time_ = current_time;
            RCLCPP_INFO(get_logger(), "Reverse complete. Cooldown...");
          }
          return;
        }
        
      case CollisionState::COOLDOWN:
        {
          if (elapsed > recovery_cooldown_duration_) {
            collision_state_ = CollisionState::NORMAL;
            RCLCPP_INFO(get_logger(), "Cooldown complete. Resuming normal operation.");
          } else {
            // During cooldown, allow normal driving but don't re-trigger collision
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
              "Recovery cooldown: %.1fs remaining",
              recovery_cooldown_duration_ - elapsed);
          }
        }
        break;
    }

    // Extract current state from odometry
    nmpc::VehicleState current_state = extractState(latest_odom_);
    
    // v4.0: Calculate ego s-position for overtake zone checks
    updateEgoSPosition(current_state);
    
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
    
    // ========================================
    // v5.0 CORNER EXIT SMOOTHING
    // ========================================
    // Detects when exiting a corner and applies:
    // 1. Wall margin constraint to avoid pushing to the outer wall
    // 2. Steering rate limiting for smoother transition to straight
    // 3. Lateral deviation limiting to prevent overshooting
    
    if (enable_corner_exit_smoothing_) {
      // Calculate current curvature from reference trajectory
      double current_curvature = calculateCurrentCurvature(reference);
      
      // Detect corner exit transition
      bool is_corner_exit = detectCornerExit(current_curvature);
      
      if (is_corner_exit && !in_corner_exit_transition_) {
        // Just started exiting a corner
        in_corner_exit_transition_ = true;
        corner_exit_start_time_ = current_time;
        RCLCPP_INFO(get_logger(), 
          "CORNER EXIT detected: curvature %.3f -> %.3f, applying smoothing for %.1fs",
          last_curvature_, current_curvature, corner_exit_transition_time_);
      }
      
      if (in_corner_exit_transition_) {
        // Calculate transition progress
        double elapsed = (current_time - corner_exit_start_time_).seconds();
        double transition_factor = std::min(1.0, elapsed / corner_exit_transition_time_);
        
        // Apply corner exit smoothing to reduce lateral movement and steering corrections
        applyCornerExitSmoothing(solution, reference, current_state, transition_factor);
        
        // Check if transition is complete
        if (elapsed >= corner_exit_transition_time_) {
          in_corner_exit_transition_ = false;
          RCLCPP_INFO(get_logger(), "Corner exit transition complete");
        }
      }
      
      // Update curvature state for next cycle's exit detection
      last_curvature_ = current_curvature;
    }
    
    // ========================================
    // v4.0 OPPONENT FOLLOWING AND OVERTAKING LOGIC
    // ========================================
    // Key philosophy:
    // - FOLLOW: politely track opponent at safe distance with similar speed
    // - OVERTAKE: only in allowed zones, commit and accelerate clearly
    // - No rapid mode switching
    
    // Check if we are in an overtake zone (based on s-position)
    bool in_overtake_zone = isInOvertakeZone();
    
    // Update driving mode state machine
    updateDrivingMode(opponent, in_overtake_zone, current_time);
    
    // Apply mode-specific behavior
    switch (driving_mode_) {
      case DrivingMode::CRUISE:
        // Normal driving on raceline - speed from curvature-based reference
        // (already set in buildReference)
        break;
        
      case DrivingMode::FOLLOW:
        {
          // v4.0: Stable following with v_follow = opponent_speed - follow_margin
          double v_max_from_raceline = solution.speed;  // From curvature-based calculation
          double v_follow = std::clamp(
            opponent.speed - follow_margin_,
            v_min_follow_,
            v_max_from_raceline
          );
          solution.speed = v_follow;
          
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "FOLLOW: distance=%.2fm, v_follow=%.2f (opponent=%.2f, margin=%.2f)",
            opponent.distance, solution.speed, opponent.speed, follow_margin_);
        }
        break;
        
      case DrivingMode::OVERTAKE_CANDIDATE:
        // In overtake zone, preparing to overtake
        // Still follow for now but may transition to OVERTAKE
        {
          double v_follow = std::clamp(
            opponent.speed - follow_margin_,
            v_min_follow_,
            solution.speed
          );
          solution.speed = v_follow;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "OVERTAKE_CANDIDATE: distance=%.2fm, evaluating...", opponent.distance);
        }
        break;
        
      case DrivingMode::OVERTAKE:
        {
          // v4.0: Committed overtake with clear speed boost
          // Apply lateral offset for overtaking path
          double lateral_offset = overtake_left_side_ ? overtake_path_width_ : -overtake_path_width_;
          
          for (auto& ref : reference) {
            double cos_yaw = std::cos(ref.yaw);
            double sin_yaw = std::sin(ref.yaw);
            ref.x += lateral_offset * (-sin_yaw);
            ref.y += lateral_offset * cos_yaw;
          }
          
          // v4.0: Clear speed boost above opponent
          double v_overtake = std::min(
            opponent.speed + overtake_boost_,
            v_max_straight_  // Respect max speed
          );
          solution.speed = std::max(solution.speed, v_overtake);
          
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
            "OVERTAKE %s: offset=%.2fm, speed=%.2f (boost=%.2f above opponent)",
            overtake_left_side_ ? "LEFT" : "RIGHT", lateral_offset, 
            solution.speed, solution.speed - opponent.speed);
        }
        break;
        
      case DrivingMode::OBSTACLE_STOP:
        // Emergency stop - handled by collision state machine
        solution.speed = 0.0;
        solution.steering = 0.0;
        break;
    }
    
    // A2 zone: apply steering avoidance (collision avoidance state must be NORMAL or COOLDOWN)
    if (collision_state_ == CollisionState::NORMAL || collision_state_ == CollisionState::COOLDOWN) {
      double delta_a2_avoidance = 0.0;
      if (checkA2Zone()) {
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
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
          "A2 Zone: adding avoidance steer=%.3f", delta_a2_avoidance);
      }
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
   * @brief Information about detected opponent vehicle
   */
  struct OpponentInfo {
    bool is_detected = false;
    bool is_ahead = false;
    double distance = 10.0;    // Distance to opponent [m]
    double speed = 0.0;        // Opponent speed [m/s]
    double x = 0.0;            // Opponent position x
    double y = 0.0;            // Opponent position y
    double relative_angle = 0.0; // Angle to opponent from vehicle heading
  };
  
  /**
   * @brief Detect opponent vehicle using LiDAR and opponent odometry
   * 
   * Combines:
   * 1. LiDAR-based obstacle detection in front
   * 2. Known opponent position from odometry (if available)
   */
  OpponentInfo detectOpponent()
  {
    OpponentInfo opponent;
    
    // Method 1: Use opponent odometry if available (most accurate)
    if (latest_opponent_odom_ && latest_odom_) {
      double ego_x = latest_odom_->pose.pose.position.x;
      double ego_y = latest_odom_->pose.pose.position.y;
      double ego_qz = latest_odom_->pose.pose.orientation.z;
      double ego_qw = latest_odom_->pose.pose.orientation.w;
      double ego_yaw = std::atan2(2.0 * ego_qw * ego_qz, 1.0 - 2.0 * ego_qz * ego_qz);
      
      opponent.x = latest_opponent_odom_->pose.pose.position.x;
      opponent.y = latest_opponent_odom_->pose.pose.position.y;
      
      double dx = opponent.x - ego_x;
      double dy = opponent.y - ego_y;
      opponent.distance = std::sqrt(dx * dx + dy * dy);
      
      // Check if within detection range
      if (opponent.distance < opponent_detection_range_) {
        opponent.is_detected = true;
        
        // Calculate relative angle (in vehicle frame)
        double cos_yaw = std::cos(-ego_yaw);
        double sin_yaw = std::sin(-ego_yaw);
        double dx_vehicle = dx * cos_yaw - dy * sin_yaw;
        double dy_vehicle = dx * sin_yaw + dy * cos_yaw;
        opponent.relative_angle = std::atan2(dy_vehicle, dx_vehicle);
        
        // Is opponent ahead? (within ±45 degrees of front)
        opponent.is_ahead = (dx_vehicle > 0 && std::abs(opponent.relative_angle) < M_PI / 4.0);
        
        // Get opponent speed
        double vx = latest_opponent_odom_->twist.twist.linear.x;
        double vy = latest_opponent_odom_->twist.twist.linear.y;
        opponent.speed = std::sqrt(vx * vx + vy * vy);
      }
    }
    
    // Method 2: If no opponent odometry, use LiDAR-based detection
    // Detect moving obstacles in front (simple heuristic)
    if (!opponent.is_detected && scan_received_ && latest_scan_) {
      // Check for large obstacle in front (could be opponent)
      if (last_obstacle_front_dist_ < opponent_detection_range_ && 
          last_obstacle_front_dist_ > a1_threshold_) {
        opponent.is_detected = true;
        opponent.is_ahead = true;
        opponent.distance = last_obstacle_front_dist_;
        opponent.relative_angle = last_obstacle_angle_;
        opponent.speed = DEFAULT_OBSTACLE_SPEED;
      }
    }
    
    return opponent;
  }
  
  /**
   * @brief Publish opponent visualization marker
   */
  void publishOpponentVisualization(const OpponentInfo& opponent)
  {
    if (!opponent.is_detected) {
      return;
    }
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "opponent";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = opponent.x;
    marker.pose.position.y = opponent.y;
    marker.pose.position.z = 0.3;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    
    // RED for opponent
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    opponent_marker_pub_->publish(marker);
  }
  
  /**
   * @brief Generate and publish overtaking paths
   * Creates left and right overtaking paths based on current raceline
   * v4.0: Also shows overtake zones (GREEN) and active path (YELLOW)
   */
  void generateAndPublishOvertakePaths()
  {
    if (!latest_path_ || latest_path_->poses.empty()) {
      return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // v4.0: Calculate s-position along path for zone highlighting
    double cumulative_s = 0.0;
    std::vector<double> s_positions;
    s_positions.push_back(0.0);
    for (size_t i = 1; i < latest_path_->poses.size(); ++i) {
      double dx = latest_path_->poses[i].pose.position.x - latest_path_->poses[i-1].pose.position.x;
      double dy = latest_path_->poses[i].pose.position.y - latest_path_->poses[i-1].pose.position.y;
      cumulative_s += std::sqrt(dx * dx + dy * dy);
      s_positions.push_back(cumulative_s);
    }
    
    // v4.0: Generate overtake zone visualization (GREEN thick line)
    visualization_msgs::msg::Marker zone_marker;
    zone_marker.header.frame_id = "map";
    zone_marker.header.stamp = now();
    zone_marker.ns = "overtake_zones";
    zone_marker.id = 10;
    zone_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    zone_marker.action = visualization_msgs::msg::Marker::ADD;
    zone_marker.scale.x = 0.1;  // Thick line
    zone_marker.color.r = 0.0f;
    zone_marker.color.g = 0.8f;  // GREEN
    zone_marker.color.b = 0.2f;
    zone_marker.color.a = 0.8f;
    zone_marker.pose.orientation.w = 1.0;
    
    // Add points that are within overtake zones
    for (size_t i = 0; i < latest_path_->poses.size(); ++i) {
      double s = s_positions[i];
      bool in_zone = false;
      size_t num_zones = std::min(overtake_zone_starts_.size(), overtake_zone_ends_.size());
      for (size_t z = 0; z < num_zones; ++z) {
        if (s >= overtake_zone_starts_[z] && s <= overtake_zone_ends_[z]) {
          in_zone = true;
          break;
        }
      }
      if (in_zone) {
        geometry_msgs::msg::Point p;
        p.x = latest_path_->poses[i].pose.position.x;
        p.y = latest_path_->poses[i].pose.position.y;
        p.z = 0.15;  // Slightly elevated
        zone_marker.points.push_back(p);
      }
    }
    zone_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    marker_array.markers.push_back(zone_marker);
    
    // Generate left overtaking path (MAGENTA)
    visualization_msgs::msg::Marker left_path;
    left_path.header.frame_id = "map";
    left_path.header.stamp = now();
    left_path.ns = "overtake_left";
    left_path.id = 0;
    left_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    left_path.action = visualization_msgs::msg::Marker::ADD;
    left_path.scale.x = 0.05;
    left_path.color.r = 1.0f;
    left_path.color.g = 0.0f;
    left_path.color.b = 1.0f;  // Magenta
    left_path.color.a = 0.7f;
    left_path.pose.orientation.w = 1.0;
    
    // Generate right overtaking path (CYAN)
    visualization_msgs::msg::Marker right_path;
    right_path.header.frame_id = "map";
    right_path.header.stamp = now();
    right_path.ns = "overtake_right";
    right_path.id = 1;
    right_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    right_path.action = visualization_msgs::msg::Marker::ADD;
    right_path.scale.x = 0.05;
    right_path.color.r = 0.0f;
    right_path.color.g = 1.0f;
    right_path.color.b = 1.0f;  // Cyan
    right_path.color.a = 0.7f;
    right_path.pose.orientation.w = 1.0;
    
    overtake_left_path_.clear();
    overtake_right_path_.clear();
    
    for (const auto& pose : latest_path_->poses) {
      double x = pose.pose.position.x;
      double y = pose.pose.position.y;
      double qz = pose.pose.orientation.z;
      double qw = pose.pose.orientation.w;
      double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
      
      // Calculate perpendicular offset
      double cos_yaw = std::cos(yaw);
      double sin_yaw = std::sin(yaw);
      
      // Left path (positive lateral offset)
      geometry_msgs::msg::Point left_pt;
      left_pt.x = x + overtake_path_width_ * (-sin_yaw);
      left_pt.y = y + overtake_path_width_ * cos_yaw;
      left_pt.z = 0.05;
      left_path.points.push_back(left_pt);
      overtake_left_path_.push_back(left_pt);
      
      // Right path (negative lateral offset)
      geometry_msgs::msg::Point right_pt;
      right_pt.x = x - overtake_path_width_ * (-sin_yaw);
      right_pt.y = y - overtake_path_width_ * cos_yaw;
      right_pt.z = 0.05;
      right_path.points.push_back(right_pt);
      overtake_right_path_.push_back(right_pt);
    }
    
    left_path.lifetime = rclcpp::Duration::from_seconds(2.0);
    right_path.lifetime = rclcpp::Duration::from_seconds(2.0);
    
    marker_array.markers.push_back(left_path);
    marker_array.markers.push_back(right_path);
    
    // v4.0: Add active overtake path visualization (YELLOW thick line)
    if (driving_mode_ == DrivingMode::OVERTAKE && overtake_committed_) {
      visualization_msgs::msg::Marker active_path;
      active_path.header.frame_id = "map";
      active_path.header.stamp = now();
      active_path.ns = "active_overtake_path";
      active_path.id = 20;
      active_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
      active_path.action = visualization_msgs::msg::Marker::ADD;
      active_path.scale.x = 0.12;  // Thick line
      active_path.color.r = 1.0f;
      active_path.color.g = 1.0f;  // YELLOW
      active_path.color.b = 0.0f;
      active_path.color.a = 1.0f;
      active_path.pose.orientation.w = 1.0;
      
      // Copy the active overtake path (left or right based on committed side)
      const auto& active_pts = overtake_left_side_ ? overtake_left_path_ : overtake_right_path_;
      for (const auto& pt : active_pts) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.2;  // Elevated to be clearly visible
        active_path.points.push_back(p);
      }
      active_path.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(active_path);
    }
    
    overtake_path_pub_->publish(marker_array);
  }
  
  /**
   * @brief Check if overtaking can be started
   * 
   * v5.0: Stricter feasibility check that considers:
   * - Opponent vehicle width
   * - Required lateral clearance = opponent_width * width_factor + safety_margin
   * - Longitudinal window to complete overtake
   */
  bool canStartOvertake(const OpponentInfo& opponent)
  {
    if (!enable_overtaking_ || !opponent.is_detected || !opponent.is_ahead) {
      return false;
    }
    
    // Check distance - must be within decision range
    if (opponent.distance > overtake_decision_distance_) {
      return false;
    }
    
    // === IMPORTANT: Being in OVERTAKE ZONE is NOT enough ===
    // Must also verify lateral clearance and longitudinal feasibility
    
    // Check if we have room on at least one side with stricter criteria
    // Required clearance considers opponent width, not just our path width
    bool left_clear = checkOvertakeLateralClearance(true, opponent);
    bool right_clear = checkOvertakeLateralClearance(false, opponent);
    
    if (!left_clear && !right_clear) {
      return false;
    }
    
    // Check longitudinal window - ensure enough distance to complete overtake
    if (!checkOvertakeLongitudinalWindow()) {
      return false;
    }
    
    return true;
  }
  
  /**
   * @brief Check lateral clearance for overtake considering opponent width
   * 
   * Lateral clearance requirement:
   *   clearance >= opponent_width * factor + safety_margin
   * 
   * This ensures the gap between the overtake path and opponent's footprint
   * is safely larger than the opponent width plus margin.
   */
  bool checkOvertakeLateralClearance(bool left_side, const OpponentInfo& /* opponent */) const
  {
    double clearance = left_side ? last_obstacle_left_dist_ : last_obstacle_right_dist_;
    
    // Calculate minimum required clearance considering opponent geometry
    // Required = opponent_width * factor (e.g. 1.2) + safety_margin + our_path_width
    double min_required = overtake_opponent_width_ * overtake_width_factor_ + 
                          overtake_safety_margin_ + overtake_path_width_;
    
    return clearance > min_required;
  }
  
  /**
   * @brief Check longitudinal window for overtake completion
   * 
   * Ensures there's enough distance to complete the overtake before
   * the next corner or narrow section.
   */
  bool checkOvertakeLongitudinalWindow() const
  {
    // Find the current overtake zone
    size_t num_zones = std::min(overtake_zone_starts_.size(), overtake_zone_ends_.size());
    for (size_t i = 0; i < num_zones; ++i) {
      if (ego_s_position_ >= overtake_zone_starts_[i] && 
          ego_s_position_ <= overtake_zone_ends_[i]) {
        // Calculate remaining distance in this zone
        double remaining = overtake_zone_ends_[i] - ego_s_position_;
        return remaining >= overtake_min_longitudinal_window_;
      }
    }
    return false;
  }
  
  /**
   * @brief Check if we can overtake on specified side (simplified check)
   */
  bool canOvertakeOnSide(bool left_side)
  {
    double clearance = left_side ? last_obstacle_left_dist_ : last_obstacle_right_dist_;
    // Use stricter requirement with opponent width consideration
    double min_required = overtake_opponent_width_ * overtake_width_factor_ + 
                          overtake_safety_margin_ + overtake_path_width_;
    return clearance > min_required;
  }
  
  /**
   * @brief Check if current overtake should be aborted due to unsafe conditions
   */
  bool shouldAbortOvertake() const
  {
    if (!overtake_committed_) {
      return false;
    }
    
    // Check if clearance on the committed side is still sufficient
    double clearance = overtake_left_side_ ? last_obstacle_left_dist_ : last_obstacle_right_dist_;
    double min_required = overtake_opponent_width_ * overtake_width_factor_ + 
                          overtake_safety_margin_ + overtake_path_width_;
    
    if (clearance < min_required) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "Overtake abort check: clearance=%.2fm < required=%.2fm", 
        clearance, min_required);
      return true;
    }
    
    return false;
  }
  
  /**
   * @brief v4.0: Check if ego is currently in an overtake zone
   * 
   * Overtake zones are defined by s-intervals on the raceline.
   * Only allow OVERTAKE mode when inside these zones.
   */
  bool isInOvertakeZone() const
  {
    if (overtake_zone_starts_.empty() || overtake_zone_ends_.empty()) {
      return false;
    }
    
    // Check each zone
    size_t num_zones = std::min(overtake_zone_starts_.size(), overtake_zone_ends_.size());
    for (size_t i = 0; i < num_zones; ++i) {
      if (ego_s_position_ >= overtake_zone_starts_[i] && 
          ego_s_position_ <= overtake_zone_ends_[i]) {
        return true;
      }
    }
    return false;
  }
  
  /**
   * @brief v4.0: Update driving mode state machine
   * 
   * Implements the philosophy:
   * - CRUISE: default, no opponent ahead
   * - FOLLOW: opponent ahead, outside overtake zone or conditions not met
   * - OVERTAKE_CANDIDATE: in overtake zone, evaluating
   * - OVERTAKE: committed to overtake, stay until complete
   * - OBSTACLE_STOP: collision detected
   */
  void updateDrivingMode(const OpponentInfo& opponent, bool in_overtake_zone, 
                         const rclcpp::Time& current_time)
  {
    // Check for collision first (highest priority)
    if (collision_state_ != CollisionState::NORMAL && 
        collision_state_ != CollisionState::COOLDOWN) {
      driving_mode_ = DrivingMode::OBSTACLE_STOP;
      return;
    }
    
    double overtake_elapsed = (current_time - overtake_start_time_).seconds();
    
    // If we're committed to overtaking, stay committed until complete
    if (overtake_committed_) {
      // Check if overtake conditions become unsafe - abort and return to FOLLOW
      if (shouldAbortOvertake()) {
        RCLCPP_WARN(get_logger(), "OVERTAKE ABORTED: clearance became unsafe");
        overtake_committed_ = false;
        is_overtaking_ = false;
        driving_mode_ = DrivingMode::FOLLOW;
        return;
      }
      
      // Check if overtake is complete
      if (!opponent.is_ahead || !opponent.is_detected) {
        // No opponent ahead - possible overtake complete
        if (overtake_elapsed > overtake_completion_timeout_) {
          // Overtake complete!
          overtake_committed_ = false;
          is_overtaking_ = false;
          driving_mode_ = DrivingMode::CRUISE;
          RCLCPP_INFO(get_logger(), "OVERTAKE COMPLETE (passed opponent for %.1fs)", 
                      overtake_elapsed);
          return;
        }
      }
      
      // Still overtaking
      driving_mode_ = DrivingMode::OVERTAKE;
      return;
    }
    
    // No opponent ahead -> CRUISE
    if (!opponent.is_detected || !opponent.is_ahead) {
      driving_mode_ = DrivingMode::CRUISE;
      is_overtaking_ = false;
      return;
    }
    
    // Opponent ahead
    double distance = opponent.distance;
    
    // Check if in following range
    if (distance > opponent_following_distance_ && distance > overtake_decision_distance_) {
      // Far from opponent, continue cruising
      driving_mode_ = DrivingMode::CRUISE;
      return;
    }
    
    // Within following/overtake range
    if (in_overtake_zone && enable_overtaking_) {
      // In overtake zone - check if we can start overtake
      // v5.0: Use stricter feasibility checks
      bool left_clear = canOvertakeOnSide(true);
      bool right_clear = canOvertakeOnSide(false);
      bool longitudinal_ok = checkOvertakeLongitudinalWindow();
      
      // Must have BOTH lateral clearance AND longitudinal window
      if ((left_clear || right_clear) && longitudinal_ok) {
        // Clearance OK - transition to OVERTAKE_CANDIDATE first
        if (driving_mode_ != DrivingMode::OVERTAKE_CANDIDATE) {
          driving_mode_ = DrivingMode::OVERTAKE_CANDIDATE;
          overtake_start_time_ = current_time;  // Start timing for commit
          RCLCPP_INFO(get_logger(), "OVERTAKE_CANDIDATE: distance=%.2fm, zone=true, left=%d, right=%d", 
                      distance, left_clear, right_clear);
        }
        
        // Check if we should commit to overtake
        double candidate_time = (current_time - overtake_start_time_).seconds();
        if (candidate_time > 0.5 && distance < overtake_decision_distance_) {
          // Commit to overtake!
          overtake_committed_ = true;
          is_overtaking_ = true;
          overtake_left_side_ = left_clear;  // Prefer left if both clear
          overtake_start_time_ = current_time;
          driving_mode_ = DrivingMode::OVERTAKE;
          RCLCPP_WARN(get_logger(), "OVERTAKE COMMITTED! side=%s, distance=%.2fm",
                      overtake_left_side_ ? "LEFT" : "RIGHT", distance);
        }
        return;
      } else if (!longitudinal_ok) {
        // In zone but not enough longitudinal window - stay in FOLLOW
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
          "In overtake zone but longitudinal window insufficient, staying in FOLLOW");
      }
    }
    
    // Not in overtake zone or can't overtake -> FOLLOW
    driving_mode_ = DrivingMode::FOLLOW;
  }
  
  /**
   * @brief v4.0: Update ego s-position along raceline
   * 
   * Calculates cumulative distance along path from start to closest point.
   * Used for overtake zone checks.
   */
  void updateEgoSPosition(const nmpc::VehicleState& current_state)
  {
    if (!latest_path_ || latest_path_->poses.empty()) {
      ego_s_position_ = 0.0;
      return;
    }
    
    const auto& poses = latest_path_->poses;
    const size_t num_poses = poses.size();
    
    // Find closest point on path
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < num_poses; ++i) {
      double dx = poses[i].pose.position.x - current_state.x;
      double dy = poses[i].pose.position.y - current_state.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_idx = i;
      }
    }
    
    // Calculate s-position as cumulative distance from start
    double s = 0.0;
    for (size_t i = 1; i <= closest_idx && i < num_poses; ++i) {
      double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
      double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
      s += std::sqrt(dx * dx + dy * dy);
    }
    
    ego_s_position_ = s;
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
   * @brief Calculate current curvature from reference trajectory
   * 
   * Uses the first few reference points to estimate the local curvature.
   * This helps detect when we're in a corner vs on a straight section.
   * 
   * @param reference The reference trajectory
   * @return The estimated curvature at the current position [1/m]
   */
  double calculateCurrentCurvature(const std::vector<nmpc::ReferencePoint>& reference) const
  {
    if (reference.size() < 3) {
      return 0.0;
    }
    
    // Use the first few points to estimate curvature
    double total_curvature = 0.0;
    int count = 0;
    int lookahead = std::min(static_cast<int>(reference.size()) - 1, curvature_lookahead_);
    
    // Minimum distance threshold for valid curvature calculation
    // Points closer than this are considered coincident and skipped to avoid division by near-zero
    constexpr double MIN_SEGMENT_DISTANCE = 0.01;  // [m]
    
    for (int i = 0; i < lookahead; ++i) {
      double dx = reference[i + 1].x - reference[i].x;
      double dy = reference[i + 1].y - reference[i].y;
      double dist = std::sqrt(dx * dx + dy * dy);
      
      if (dist > MIN_SEGMENT_DISTANCE) {
        double dyaw = reference[i + 1].yaw - reference[i].yaw;
        // Normalize angle
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        
        double curvature = std::abs(dyaw) / dist;
        total_curvature += curvature;
        count++;
      }
    }
    
    return (count > 0) ? total_curvature / count : 0.0;
  }
  
  /**
   * @brief Detect if currently exiting a corner
   * 
   * Corner exit is detected when:
   * 1. We were recently in a corner (high curvature, > curvature_k1)
   * 2. Current curvature is now low (straight section, < curvature_k1)
   * 
   * Uses last_curvature_ member variable to track previous cycle's curvature.
   * 
   * @param current_curvature Current path curvature [1/m]
   * @return true if transitioning from corner to straight
   */
  bool detectCornerExit(double current_curvature) const
  {
    // Exiting corner = was in corner (high curvature) and now on straight (low curvature)
    // Use curvature_k1_ as the threshold between corner and straight sections
    bool was_in_corner = last_curvature_ > curvature_k1_;
    bool now_on_straight = current_curvature < curvature_k1_;
    
    return was_in_corner && now_on_straight;
  }
  
  /**
   * @brief Apply corner exit smoothing to NMPC solution
   * 
   * When exiting a corner, this function:
   * 1. Limits lateral deviation to stay away from the wall
   * 2. Applies steering rate limiting for smoother transitions
   * 
   * @param solution The NMPC solution to modify
   * @param reference The reference trajectory
   * @param current_state The current vehicle state
   * @param transition_factor How far along the transition we are (0=start, 1=end)
   */
  void applyCornerExitSmoothing(
    nmpc::MPCSolution& solution,
    const std::vector<nmpc::ReferencePoint>& reference,
    const nmpc::VehicleState& current_state,
    double transition_factor)
  {
    if (reference.empty() || !enable_corner_exit_smoothing_) {
      return;
    }
    
    // Design constants for corner exit smoothing behavior
    // These are fixed values that define the smoothing algorithm characteristics
    // Lateral limit scaling: starts at INITIAL_LIMIT_SCALE * config, decreases to 1.0 * config
    constexpr double INITIAL_LIMIT_SCALE = 2.0;
    // Maximum steering correction factor when overshooting (50% reduction at transition start)
    constexpr double MAX_CORRECTION_FACTOR = 0.5;
    // Maximum wall avoidance reduction factor (50% maximum reduction when very close to wall)
    constexpr double MAX_WALL_AVOIDANCE_FACTOR = 0.5;
    
    // 1. Apply steering rate limiting during corner exit transition
    // This prevents sudden large steering corrections when rejoining the straight
    // transition_factor: 0 at start (full limiting), 1 at end (no limiting)
    double steer_rate_factor = corner_exit_steer_rate_limit_ + 
                               (1.0 - corner_exit_steer_rate_limit_) * transition_factor;
    
    // Apply the limit by blending toward center (reduce magnitude)
    solution.steering *= steer_rate_factor;
    
    // 2. Limit lateral position to maintain wall margin
    // Calculate current lateral error from reference
    if (!reference.empty()) {
      double dx = current_state.x - reference[0].x;
      double dy = current_state.y - reference[0].y;
      
      // Project to lateral direction (perpendicular to reference heading)
      double ref_cos = std::cos(reference[0].yaw);
      double ref_sin = std::sin(reference[0].yaw);
      double lateral_error = -dx * ref_sin + dy * ref_cos;
      
      // Check if we're pushing too far to one side (toward the wall)
      // During corner exit, the car tends to push outward
      // Lateral limit is more restrictive at the start of transition (INITIAL_LIMIT_SCALE)
      // and relaxes to the configured value (1.0) as transition completes
      double max_lateral = corner_exit_lateral_limit_ * (INITIAL_LIMIT_SCALE - transition_factor);
      
      if (std::abs(lateral_error) > max_lateral) {
        // We're too far from the ideal line - apply correction
        // This helps prevent the car from hugging the outer wall
        // Correction is strongest at start (MAX_CORRECTION_FACTOR) and decreases over transition
        double correction_factor = MAX_CORRECTION_FACTOR * (1.0 - transition_factor);
        
        // Steer back toward the reference (reduce magnitude if steering away from reference)
        if ((lateral_error > 0 && solution.steering < 0) ||
            (lateral_error < 0 && solution.steering > 0)) {
          // Steering is already correcting - no change needed
        } else {
          // Steering is making it worse - reduce steering magnitude
          solution.steering *= (1.0 - correction_factor);
        }
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
          "Corner exit: lateral=%.2fm > limit=%.2fm, reducing steer by %.0f%%",
          std::abs(lateral_error), max_lateral, correction_factor * 100);
      }
    }
    
    // 3. Check wall distance and apply margin constraint
    // Use the side obstacle distances to enforce wall margin
    double min_side_dist = std::min(last_obstacle_left_dist_, last_obstacle_right_dist_);
    if (min_side_dist < corner_exit_wall_margin_) {
      // Too close to wall during corner exit - reduce steering toward wall
      // Urgency increases as we get closer to the wall (0 = at margin, 1 = touching wall)
      double wall_urgency = 1.0 - (min_side_dist / corner_exit_wall_margin_);
      
      // Determine which side is closer
      bool wall_on_left = (last_obstacle_left_dist_ < last_obstacle_right_dist_);
      
      // If steering toward the wall, reduce steering
      // Reduction combines wall urgency with transition factor (stronger at start of transition)
      if ((wall_on_left && solution.steering > 0) ||
          (!wall_on_left && solution.steering < 0)) {
        double reduction = wall_urgency * MAX_WALL_AVOIDANCE_FACTOR * (1.0 - transition_factor);
        solution.steering *= (1.0 - reduction);
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
          "Corner exit: wall dist=%.2fm < margin=%.2fm, reducing steer toward wall by %.0f%%",
          min_side_dist, corner_exit_wall_margin_, reduction * 100);
      }
    }
    
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 300,
      "Corner exit smoothing: transition=%.1f%%, steer_factor=%.2f, final_steer=%.3f",
      transition_factor * 100, steer_rate_factor, solution.steering);
  }
  
  /**
   * @brief Compute avoidance steering (A2 zone) - DISABLED
   * 
   * 2024-12: 사용자 요청에 따라 비활성화됨
   * 장애물/벽 충돌 시 갑작스런 반대 방향 조향이 차량을 불안정하게 만들어 제거
   * NMPC 경로 추종 + A1 zone 후진만 동작
   */
  double computeAvoidanceSteering(double /* lookahead_angle */)
  {
    return 0.0;
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
      
      // v4.0: Curvature-based speed control using configurable thresholds
      // Speed is determined by raceline curvature, NOT wall proximity
      ref.v = v_max_straight_;  // Default to straight speed
      
      // Calculate curvature for this point using lookahead
      if (i < NUM_REF_POINTS - 1) {
        size_t next_idx = (start_idx + (i + 1) * stride) % num_poses;
        double dx = poses[next_idx].pose.position.x - poses[idx].pose.position.x;
        double dy = poses[next_idx].pose.position.y - poses[idx].pose.position.y;
        
        // Full quaternion to yaw conversion for correctness
        double next_qx = poses[next_idx].pose.orientation.x;
        double next_qy = poses[next_idx].pose.orientation.y;
        double next_qz = poses[next_idx].pose.orientation.z;
        double next_qw = poses[next_idx].pose.orientation.w;
        double next_yaw = std::atan2(2.0 * (next_qw * next_qz + next_qx * next_qy), 
                                      1.0 - 2.0 * (next_qy * next_qy + next_qz * next_qz));
        
        // Proper angle normalization for dyaw
        double dyaw = next_yaw - ref.yaw;
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        dyaw = std::abs(dyaw);
        
        double seg_dist = std::sqrt(dx * dx + dy * dy);
        if (seg_dist > 0.01) {
          double curvature = dyaw / seg_dist;
          
          // v4.0: Apply curvature-based speed profile using k1, k2 thresholds
          // |κ| < k1 (straight): v_ref = v_max_straight
          // k1 <= |κ| <= k2: linear interpolation
          // |κ| > k2 (tight corner): v_ref = v_min_corner
          if (curvature < curvature_k1_) {
            ref.v = v_max_straight_;
          } else if (curvature > curvature_k2_) {
            ref.v = v_min_corner_;
          } else {
            // Linear interpolation between k1 and k2
            double t = (curvature - curvature_k1_) / (curvature_k2_ - curvature_k1_);
            ref.v = v_max_straight_ + t * (v_min_corner_ - v_max_straight_);
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opponent_odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nmpc_trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nmpc_reference_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr overtake_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr opponent_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Odometry::SharedPtr latest_opponent_odom_;
  nav_msgs::msg::Path::SharedPtr latest_path_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  bool scan_received_{false};

  // Parameters
  double control_rate_hz_{50.0};
  double nominal_speed_{2.0};
  bool use_dual_ekf_{true};
  double max_steer_{0.436};  // 25 degrees in radians
  double prediction_horizon_{1.0};  // Store prediction horizon for reference building
  
  // Simplified Collision parameters
  double a1_threshold_{0.3};          // [m] Front obstacle collision trigger
  double a2_threshold_{0.4};          // [m] Steering avoidance threshold
  double a2_urgent_threshold_{0.25};  // [m] Urgent avoidance threshold
  double a2_max_steer_ratio_{1.0};    // Full steering allowed
  double a2_urgent_steer_ratio_{1.0}; // Full steering in urgent mode
  double reverse_speed_{1.5};         // [m/s] Reverse speed
  double reverse_duration_{0.5};      // [s] Reverse duration
  double stop_duration_{0.3};         // [s] Stop duration before reversing
  double a1_steer_gain_{1.0};         // Not used in simplified mode
  double a2_steer_gain_{1.5};         // High repulsive force gain
  double a2_urgent_steer_gain_{2.0};  // Very high urgent gain
  bool enable_collision_avoidance_{true};
  
  // Opponent following and overtaking parameters
  double opponent_detection_range_{3.0};
  double opponent_following_distance_{1.0};
  double overtake_path_width_{0.8};
  double overtake_decision_distance_{2.5};
  bool enable_overtaking_{true};
  double opponent_speed_tracking_gain_{0.8};
  
  // v4.0 FOLLOW parameters
  double follow_margin_{0.05};              // [m/s] Speed margin below opponent
  double v_min_follow_{0.3};                // [m/s] Minimum follow speed
  
  // v4.0 OVERTAKE parameters  
  double overtake_boost_{0.25};             // [m/s] Speed boost above opponent
  double overtake_min_commit_time_{2.0};    // [s] Min time to commit to overtake
  double overtake_completion_timeout_{3.0}; // [s] Time after passing to confirm complete
  
  // v5.0 Overtake feasibility parameters (stricter decision logic)
  double overtake_opponent_width_{0.35};    // [m] Assumed opponent vehicle width
  double overtake_width_factor_{1.2};       // Factor for clearance calculation
  double overtake_safety_margin_{0.25};     // [m] Additional safety margin
  double overtake_min_longitudinal_window_{5.0}; // [m] Min distance to complete overtake
  
  // v4.0 Curvature-based speed control
  double curvature_k1_{0.2};                // [1/m] Threshold for straight
  double curvature_k2_{0.8};                // [1/m] Threshold for tight corner
  double v_max_straight_{3.0};              // [m/s] Speed on straights
  double v_min_corner_{1.2};                // [m/s] Speed in tight corners
  int curvature_lookahead_{5};              // [points] Lookahead for curvature calc
  
  // v5.0 Corner exit smoothing parameters
  double corner_exit_wall_margin_{0.4};     // [m] Min distance from wall on corner exit
  double corner_exit_lateral_limit_{0.15};  // [m] Max lateral deviation during exit transition
  double corner_exit_steer_rate_limit_{0.8};// [ratio] Steering rate limit factor (0-1) on exit
  double corner_exit_transition_time_{1.0}; // [s] Time to transition from corner to straight
  bool enable_corner_exit_smoothing_{true}; // Enable/disable corner exit smoothing
  
  // Corner exit detection state
  rclcpp::Time corner_exit_start_time_;     // When corner exit started
  bool in_corner_exit_transition_{false};   // Currently transitioning from corner to straight
  double last_curvature_{0.0};              // Curvature from last cycle (for exit detection)
  
  // v4.0 Overtake zones
  std::vector<double> overtake_zone_starts_;
  std::vector<double> overtake_zone_ends_;
  
  // Collision state machine
  CollisionState collision_state_{CollisionState::NORMAL};
  rclcpp::Time collision_state_start_time_;
  
  // Obstacle distances (updated each cycle)
  bool is_in_a1_zone_{false};
  double last_obstacle_left_dist_{10.0};
  double last_obstacle_right_dist_{10.0};
  double last_obstacle_front_dist_{10.0};
  double last_obstacle_angle_{0.0};
  double recovery_cooldown_duration_{0.3};
  
  // v4.0 Driving mode and overtaking state
  DrivingMode driving_mode_{DrivingMode::CRUISE};
  bool is_overtaking_{false};
  bool overtake_committed_{false};           // Once true, stay in OVERTAKE until complete
  bool overtake_left_side_{true};            // Which side we're overtaking on
  rclcpp::Time overtake_start_time_;
  double ego_s_position_{0.0};               // Ego s-position on raceline (for zone checks)
  std::vector<geometry_msgs::msg::Point> overtake_left_path_;
  std::vector<geometry_msgs::msg::Point> overtake_right_path_;
  
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

