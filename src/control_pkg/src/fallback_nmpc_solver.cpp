/**
 * @file fallback_nmpc_solver.cpp
 * @brief Fallback NMPC solver implementation using hand-written SQP
 * 
 * This is the fallback implementation when acados is not available.
 * It provides the same interface as AcadosNMPCSolver but uses a simple
 * numerical optimization approach similar to the original nmpc_engine_node.
 */

#include "control_pkg/acados_nmpc_interface.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace acados_interface
{

/**
 * @brief Implementation class (PIMPL pattern)
 */
class FallbackNMPCSolver::Impl
{
public:
  NMPCConfig config_;
  bool initialized_{false};
  
  // Current state and references
  VehicleState initial_state_;
  std::vector<ReferencePoint> reference_;
  ControlInput prev_control_;
  
  // Optimization state (warm start)
  std::vector<ControlInput> u_sequence_;
  int consecutive_failures_{0};
  
  /**
   * @brief Normalize angle to [-pi, pi]
   */
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
  
  /**
   * @brief Forward simulate using kinematic bicycle model
   */
  std::vector<VehicleState> forwardSimulate(
    const VehicleState& initial,
    const std::vector<ControlInput>& controls) const
  {
    std::vector<VehicleState> states;
    states.reserve(controls.size() + 1);
    states.push_back(initial);
    
    VehicleState state = initial;
    
    for (const auto& u : controls) {
      double cos_yaw = std::cos(state.yaw);
      double sin_yaw = std::sin(state.yaw);
      double clamped_steer = std::clamp(u.steering, 
        -config_.max_steering * 0.99, config_.max_steering * 0.99);
      double tan_steer = std::tan(clamped_steer);
      
      state.x += state.v * cos_yaw * config_.dt;
      state.y += state.v * sin_yaw * config_.dt;
      state.yaw += (state.v / config_.wheelbase) * tan_steer * config_.dt;
      state.v += u.acceleration * config_.dt;
      
      state.yaw = normalizeAngle(state.yaw);
      state.v = std::clamp(state.v, config_.min_speed, config_.max_speed);
      
      states.push_back(state);
    }
    
    return states;
  }
  
  /**
   * @brief Compute cost function
   */
  double computeCost(
    const std::vector<VehicleState>& states,
    const std::vector<ControlInput>& controls) const
  {
    double cost = 0.0;
    
    // State tracking cost with lateral tolerance tube
    for (size_t i = 0; i < states.size() && i < reference_.size(); ++i) {
      double dx = states[i].x - reference_[i].x;
      double dy = states[i].y - reference_[i].y;
      double dyaw = normalizeAngle(states[i].yaw - reference_[i].yaw);
      double dv = states[i].v - reference_[i].v;
      
      // Frenet-like lateral error
      double ref_cos = std::cos(reference_[i].yaw);
      double ref_sin = std::sin(reference_[i].yaw);
      double lateral_error = -dx * ref_sin + dy * ref_cos;
      double longitudinal_error = dx * ref_cos + dy * ref_sin;
      
      // Lateral tolerance tube (soft constraint)
      double lateral_excess = std::max(0.0, 
        std::abs(lateral_error) - config_.lateral_tolerance);
      
      cost += config_.w_y * lateral_excess * lateral_excess;
      cost += config_.w_x * 0.3 * longitudinal_error * longitudinal_error;
      cost += config_.w_yaw * dyaw * dyaw;
      cost += config_.w_v * dv * dv;
    }
    
    // Terminal cost
    if (!states.empty() && !reference_.empty()) {
      size_t last_idx = std::min(states.size() - 1, reference_.size() - 1);
      double dx = states[last_idx].x - reference_[last_idx].x;
      double dy = states[last_idx].y - reference_[last_idx].y;
      double dyaw = normalizeAngle(states[last_idx].yaw - reference_[last_idx].yaw);
      cost += config_.w_terminal * (dx * dx + dy * dy);
      cost += config_.w_terminal * 0.5 * dyaw * dyaw;
    }
    
    // Control effort cost
    for (size_t i = 0; i < controls.size(); ++i) {
      cost += config_.w_steering * controls[i].steering * controls[i].steering;
      cost += config_.w_acceleration * controls[i].acceleration * controls[i].acceleration;
      
      // Control rate cost (CRITICAL for stability)
      if (i > 0) {
        double d_steer = controls[i].steering - controls[i-1].steering;
        double d_accel = controls[i].acceleration - controls[i-1].acceleration;
        cost += config_.w_steering_rate * d_steer * d_steer;
        cost += config_.w_acceleration_rate * d_accel * d_accel;
      }
    }
    
    // First control change from previous
    if (!controls.empty()) {
      double d_steer = controls[0].steering - prev_control_.steering;
      double d_accel = controls[0].acceleration - prev_control_.acceleration;
      cost += config_.w_steering_rate * d_steer * d_steer;
      cost += config_.w_acceleration_rate * d_accel * d_accel;
    }
    
    return cost;
  }
  
  /**
   * @brief Gradient descent update with regularization
   */
  double updateControls(std::vector<ControlInput>& controls)
  {
    constexpr double EPSILON = 1e-4;
    constexpr double LEARNING_RATE_STEER = 0.25;
    constexpr double LEARNING_RATE_ACCEL = 0.20;
    
    double total_step = 0.0;
    
    for (size_t i = 0; i < controls.size(); ++i) {
      // Numerical gradient for steering
      std::vector<ControlInput> u_plus = controls;
      std::vector<ControlInput> u_minus = controls;
      u_plus[i].steering += EPSILON;
      u_minus[i].steering -= EPSILON;
      
      auto states_plus = forwardSimulate(initial_state_, u_plus);
      auto states_minus = forwardSimulate(initial_state_, u_minus);
      double grad_steer = (computeCost(states_plus, u_plus) - 
                          computeCost(states_minus, u_minus)) / (2.0 * EPSILON);
      
      // Numerical gradient for acceleration
      u_plus = controls;
      u_minus = controls;
      u_plus[i].acceleration += EPSILON;
      u_minus[i].acceleration -= EPSILON;
      
      states_plus = forwardSimulate(initial_state_, u_plus);
      states_minus = forwardSimulate(initial_state_, u_minus);
      double grad_accel = (computeCost(states_plus, u_plus) - 
                          computeCost(states_minus, u_minus)) / (2.0 * EPSILON);
      
      // Handle NaN gradients
      if (std::isnan(grad_steer) || std::isinf(grad_steer)) grad_steer = 0.0;
      if (std::isnan(grad_accel) || std::isinf(grad_accel)) grad_accel = 0.0;
      
      // Gradient descent with clipping
      double step_steer = std::clamp(LEARNING_RATE_STEER * grad_steer, -0.1, 0.1);
      double step_accel = std::clamp(LEARNING_RATE_ACCEL * grad_accel, -0.5, 0.5);
      
      controls[i].steering -= step_steer;
      controls[i].acceleration -= step_accel;
      
      total_step += std::abs(step_steer) + std::abs(step_accel);
    }
    
    return total_step;
  }
  
  /**
   * @brief Apply constraints
   */
  void applyConstraints(std::vector<ControlInput>& controls) const
  {
    for (size_t i = 0; i < controls.size(); ++i) {
      controls[i].steering = std::clamp(controls[i].steering,
        -config_.max_steering, config_.max_steering);
      controls[i].acceleration = std::clamp(controls[i].acceleration,
        config_.min_acceleration, config_.max_acceleration);
      
      if (i > 0) {
        double max_d_steer = config_.max_steering_rate * config_.dt;
        double d_steer = controls[i].steering - controls[i-1].steering;
        if (std::abs(d_steer) > max_d_steer) {
          controls[i].steering = controls[i-1].steering + 
            std::copysign(max_d_steer, d_steer);
        }
      }
    }
  }
  
  /**
   * @brief Compensate for latency
   */
  VehicleState compensateLatency(const VehicleState& state) const
  {
    if (config_.latency_compensation_sec <= 0.0 || u_sequence_.empty()) {
      return state;
    }
    
    VehicleState predicted = state;
    const auto& last_control = u_sequence_[0];
    double latency = config_.latency_compensation_sec;
    
    double cos_yaw = std::cos(predicted.yaw);
    double sin_yaw = std::sin(predicted.yaw);
    double tan_steer = std::tan(last_control.steering);
    
    predicted.x += predicted.v * cos_yaw * latency;
    predicted.y += predicted.v * sin_yaw * latency;
    predicted.yaw += (predicted.v / config_.wheelbase) * tan_steer * latency;
    predicted.v += last_control.acceleration * latency;
    
    predicted.yaw = normalizeAngle(predicted.yaw);
    predicted.v = std::clamp(predicted.v, config_.min_speed, config_.max_speed);
    
    return predicted;
  }
};

FallbackNMPCSolver::FallbackNMPCSolver()
  : impl_(std::make_unique<Impl>())
{
}

FallbackNMPCSolver::~FallbackNMPCSolver() = default;

bool FallbackNMPCSolver::initialize(const NMPCConfig& config)
{
  impl_->config_ = config;
  impl_->config_.dt = config.prediction_horizon_sec / std::max(1, config.N);
  
  // Initialize control sequence
  impl_->u_sequence_.resize(config.N);
  for (auto& u : impl_->u_sequence_) {
    u.steering = 0.0;
    u.acceleration = 1.0;  // Initial acceleration to get moving
  }
  
  impl_->consecutive_failures_ = 0;
  impl_->initialized_ = true;
  
  return true;
}

void FallbackNMPCSolver::cleanup()
{
  impl_->initialized_ = false;
  impl_->u_sequence_.clear();
}

void FallbackNMPCSolver::setInitialState(const VehicleState& state)
{
  impl_->initial_state_ = impl_->compensateLatency(state);
}

void FallbackNMPCSolver::setReference(const std::vector<ReferencePoint>& reference)
{
  impl_->reference_ = reference;
  // Ensure enough reference points
  while (impl_->reference_.size() < static_cast<size_t>(impl_->config_.N + 1)) {
    impl_->reference_.push_back(impl_->reference_.back());
  }
}

void FallbackNMPCSolver::setPreviousControl(const ControlInput& prev_control)
{
  impl_->prev_control_ = prev_control;
}

NMPCSolution FallbackNMPCSolver::solve()
{
  NMPCSolution solution;
  solution.feasible = false;
  solution.stats.status = SolverStatus::NOT_INITIALIZED;
  
  if (!impl_->initialized_) {
    return solution;
  }
  
  if (impl_->reference_.empty()) {
    solution.stats.status = SolverStatus::QP_FAILURE;
    return solution;
  }
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Use warm started control sequence
  std::vector<ControlInput> u = impl_->u_sequence_;
  
  // Resize if needed
  while (u.size() < static_cast<size_t>(impl_->config_.N)) {
    u.push_back(ControlInput{0.0, 0.5});
  }
  while (u.size() > static_cast<size_t>(impl_->config_.N)) {
    u.pop_back();
  }
  
  // Optimization loop
  double prev_cost = std::numeric_limits<double>::max();
  bool converged = false;
  
  for (int iter = 0; iter < impl_->config_.max_sqp_iterations; ++iter) {
    auto states = impl_->forwardSimulate(impl_->initial_state_, u);
    
    // Check for NaN
    bool has_nan = false;
    for (const auto& state : states) {
      if (std::isnan(state.x) || std::isnan(state.y) || 
          std::isnan(state.yaw) || std::isnan(state.v)) {
        has_nan = true;
        break;
      }
    }
    
    if (has_nan) {
      solution.stats.status = SolverStatus::NAN_SOLUTION;
      impl_->consecutive_failures_++;
      if (impl_->consecutive_failures_ > 5) {
        reset();
      }
      break;
    }
    
    double cost = impl_->computeCost(states, u);
    
    if (std::isnan(cost) || std::isinf(cost)) {
      solution.stats.status = SolverStatus::NAN_SOLUTION;
      break;
    }
    
    // Check convergence
    double cost_change = std::abs(prev_cost - cost);
    if (cost_change < impl_->config_.convergence_tolerance) {
      converged = true;
      solution.stats.sqp_iterations = iter + 1;
      break;
    }
    
    prev_cost = cost;
    solution.stats.sqp_iterations = iter + 1;
    solution.stats.cost = cost;
    
    // Update controls
    impl_->updateControls(u);
  }
  
  // Apply constraints
  impl_->applyConstraints(u);
  
  // Store for warm start
  impl_->u_sequence_ = u;
  // Shift for next iteration
  if (impl_->u_sequence_.size() > 1) {
    impl_->u_sequence_.erase(impl_->u_sequence_.begin());
    impl_->u_sequence_.push_back(impl_->u_sequence_.back());
  }
  
  // Build solution
  solution.predicted_trajectory = impl_->forwardSimulate(impl_->initial_state_, u);
  
  if (!u.empty()) {
    solution.control = u[0];
    
    // Minimum speed boost for initial movement
    double computed_speed = impl_->initial_state_.v + u[0].acceleration * impl_->config_.dt;
    if (impl_->initial_state_.v < 0.3 && computed_speed < 0.5) {
      computed_speed = 0.5;
    }
    // Store computed speed in control for caller to use
    // Note: ControlInput doesn't have speed field, caller will compute it
    
    solution.feasible = converged || 
      solution.stats.sqp_iterations >= impl_->config_.max_sqp_iterations;
    
    if (solution.feasible) {
      solution.stats.status = converged ? SolverStatus::SUCCESS : SolverStatus::MAX_SQP_ITER;
      impl_->consecutive_failures_ = 0;
    }
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  solution.stats.total_time_ms = std::chrono::duration<double, std::milli>(
    end_time - start_time).count();
  
  return solution;
}

void FallbackNMPCSolver::preparationPhase()
{
  // No-op for fallback solver (preparation is part of solve)
}

NMPCSolution FallbackNMPCSolver::feedbackPhase(const VehicleState& state)
{
  setInitialState(state);
  return solve();
}

void FallbackNMPCSolver::warmStart()
{
  // Already using warm start in solve()
}

void FallbackNMPCSolver::reset()
{
  for (auto& u : impl_->u_sequence_) {
    u.steering = 0.0;
    u.acceleration = 0.5;
  }
  impl_->consecutive_failures_ = 0;
}

bool FallbackNMPCSolver::isInitialized() const
{
  return impl_->initialized_;
}

const NMPCConfig& FallbackNMPCSolver::getConfig() const
{
  return impl_->config_;
}

}  // namespace acados_interface
