
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

struct MPCConfig
{
  double horizon_sec = 1.5;       
  int prediction_steps = 15;      
  double dt = 0.1;                
  double wheelbase = 0.33;        
  
  double vehicle_mass = 3.5;      
  double vehicle_inertia = 0.04;  
  double lf = 0.17;               
  double lr = 0.16;               
  double tire_Bf = 2.5;           
  double tire_Cf = 1.3;           
  double tire_Df = 4.5;           
  double tire_Br = 2.5;           
  double tire_Cr = 1.3;           
  double tire_Dr = 4.5;           
  double dynamic_model_threshold = 2.5;  
  
  double w_pos = 10.0;            
  double w_yaw = 10.0;            
  double w_vel = 5.0;             
  double w_steer = 0.5;           
  double w_accel = 0.3;           
  double w_steer_rate = 600.0;    
  double w_accel_rate = 60.0;     
  
  double lateral_tolerance = 0.25; 
  double w_terminal = 30.0;       
  
  double w_slack_track = 800.0;   
  double w_slack_speed = 400.0;   
  double track_boundary_margin = 0.05;  
  
  double max_steer = 0.6458;      
  double max_steer_rate = 1.8;    
  double max_accel = 4.0;         
  double min_accel = -6.0;        
  double max_speed = 6.0;         
  double min_speed = 0.0;         
  
  int max_iterations = 25;        
  double convergence_tol = 5e-5;  
  double levenberg_marquardt = 2e-2;  
  double min_step_size = 1e-6;    
  
  double latency_compensation_sec = 0.05;  
};

struct VehicleState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
};

struct ControlInput
{
  double steering = 0.0;
  double acceleration = 0.0;
};

struct ReferencePoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
};

enum class SolverStatus
{
  SUCCESS = 0,           
  MAX_ITER = 1,          
  INFEASIBLE = 2,        
  NUMERICAL_ERROR = 3,   
  MIN_STEP = 4           
};

struct MPCSolution
{
  double steering = 0.0;
  double speed = 0.0;
  bool feasible = false;
  double cost = 0.0;
  int iterations = 0;
  SolverStatus status = SolverStatus::SUCCESS;
  std::vector<VehicleState> predicted_trajectory;  
  double slack_violation = 0.0;  
};

class BicycleMPCSolver
{
public:
  BicycleMPCSolver() = default;
  
  bool initialize(const MPCConfig& config, const rclcpp::Logger& logger)
  {
    config_ = config;
    config_.dt = config_.horizon_sec / std::max(1, config_.prediction_steps);
    
    u_prev_.resize(config_.prediction_steps);
    for (auto& u : u_prev_) {
      u.steering = 0.0;
      u.acceleration = 1.0;  
    }
    
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
  
    void resetSolver()
  {
    for (auto& u : u_prev_) {
      u.steering = 0.0;
      u.acceleration = 0.5;
    }
    consecutive_failures_ = 0;
    last_solve_status_ = SolverStatus::SUCCESS;
  }
  
    VehicleState compensateLatency(
    const VehicleState& current_state,
    double latency_sec) const
  {
    if (latency_sec <= 0.0 || u_prev_.empty()) {
      return current_state;
    }
    
    VehicleState predicted = current_state;
    
    const auto& last_control = u_prev_[0];
    
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
    
    VehicleState compensated_state = compensateLatency(current_state, config_.latency_compensation_sec);
    
    std::vector<ReferencePoint> ref = reference;
    while (ref.size() < static_cast<size_t>(config_.prediction_steps + 1)) {
      ref.push_back(ref.back());
    }
    
    std::vector<ControlInput> u = u_prev_;
    
    while (u.size() < static_cast<size_t>(config_.prediction_steps)) {
      u.push_back(ControlInput{0.0, 0.5});
    }
    while (u.size() > static_cast<size_t>(config_.prediction_steps)) {
      u.pop_back();
    }
    
    double prev_cost = std::numeric_limits<double>::max();
    double lambda = config_.levenberg_marquardt;  
    double step_size = 1.0;
    bool numerical_error = false;
    
    for (int iter = 0; iter < config_.max_iterations; ++iter) {
      std::vector<VehicleState> predicted_states = 
        (compensated_state.v > config_.dynamic_model_threshold) ?
        forwardSimulateDynamic(compensated_state, u) :
        forwardSimulate(compensated_state, u);
      
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
      
      double cost = computeCostWithSoftConstraints(predicted_states, u, ref, solution.slack_violation);
      
      if (std::isnan(cost) || std::isinf(cost)) {
        numerical_error = true;
        solution.status = SolverStatus::NUMERICAL_ERROR;
        lambda *= 10.0;  
        continue;
      }
      
      double cost_change = std::abs(prev_cost - cost);
      if (cost_change < config_.convergence_tol) {
        solution.iterations = iter + 1;
        solution.status = SolverStatus::SUCCESS;
        break;
      }
      
      if (step_size < config_.min_step_size) {
        solution.iterations = iter + 1;
        solution.status = SolverStatus::MIN_STEP;
        RCLCPP_DEBUG_THROTTLE(logger, *rclcpp::Clock::make_shared(), 1000,
          "NMPC: Minimum step size reached at iteration %d", iter);
        break;
      }
      
      prev_cost = cost;
      solution.iterations = iter + 1;
      
      step_size = updateControlsWithRegularization(
        compensated_state, predicted_states, u, ref, lambda);
      
      double relative_change = (prev_cost > 1e-6) ? cost_change / prev_cost : 1.0;
      if (relative_change < 0.1 && lambda < 1.0) {
        lambda *= 2.0;  
      } else if (relative_change > 0.3 && lambda > 1e-4) {
        lambda *= 0.5;  
      }
      
      solution.cost = cost;
    }
    
    if (numerical_error || solution.iterations >= config_.max_iterations) {
      if (solution.iterations >= config_.max_iterations) {
        solution.status = SolverStatus::MAX_ITER;
      }
      consecutive_failures_++;
      
      if (consecutive_failures_ > 8) {
        RCLCPP_WARN_THROTTLE(logger, *rclcpp::Clock::make_shared(), 1000,
          "NMPC: %d consecutive failures, resetting solver", consecutive_failures_);
        resetSolver();
      }
    } else {
      consecutive_failures_ = 0;
    }
    
    applyConstraints(u);
    
    solution.predicted_trajectory = forwardSimulate(compensated_state, u);
    
    u_prev_ = u;
    if (u_prev_.size() > 1) {
      u_prev_.erase(u_prev_.begin());
      u_prev_.push_back(u_prev_.back());
    }
    
    if (!u.empty()) {
      solution.steering = u[0].steering;
      
      double computed_speed = current_state.v + u[0].acceleration * config_.dt;
      
      constexpr double MIN_OPERATING_SPEED = 0.5;  
      constexpr double LOW_SPEED_THRESHOLD = 0.3;  
      
      if (current_state.v < LOW_SPEED_THRESHOLD && computed_speed < MIN_OPERATING_SPEED) {
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
  
    std::vector<VehicleState> forwardSimulate(
    const VehicleState& initial_state,
    const std::vector<ControlInput>& controls) const
  {
    std::vector<VehicleState> states;
    states.reserve(controls.size() + 1);
    states.push_back(initial_state);
    
    VehicleState state = initial_state;
    
    for (const auto& u : controls) {
      double cos_yaw = std::cos(state.yaw);
      double sin_yaw = std::sin(state.yaw);
      
      double clamped_steer = std::clamp(u.steering, -config_.max_steer * 0.99, config_.max_steer * 0.99);
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
  
    std::vector<VehicleState> forwardSimulateDynamic(
    const VehicleState& initial_state,
    const std::vector<ControlInput>& controls) const
  {
    std::vector<VehicleState> states;
    states.reserve(controls.size() + 1);
    states.push_back(initial_state);
    
    VehicleState state = initial_state;
    double yaw_rate = 0.0;  
    double slip_angle = 0.0;  
    
    const double& m = config_.vehicle_mass;
    const double& Iz = config_.vehicle_inertia;
    const double& lf = config_.lf;
    const double& lr = config_.lr;
    
    for (const auto& u : controls) {
      double v = std::max(state.v, 0.1);  
      
      double delta = std::clamp(u.steering, -config_.max_steer * 0.99, config_.max_steer * 0.99);
      
      double alpha_f = delta - std::atan2(v * slip_angle + lf * yaw_rate, v);
      double alpha_r = -std::atan2(v * slip_angle - lr * yaw_rate, v);
      
      alpha_f = std::clamp(alpha_f, -0.5, 0.5);
      alpha_r = std::clamp(alpha_r, -0.5, 0.5);
      
      double Fy_f = config_.tire_Df * std::sin(config_.tire_Cf * std::atan(config_.tire_Bf * alpha_f));
      double Fy_r = config_.tire_Dr * std::sin(config_.tire_Cr * std::atan(config_.tire_Br * alpha_r));
      
      
      double a_y = (Fy_f + Fy_r) / m;
      double r_dot = (lf * Fy_f - lr * Fy_r) / Iz;
      
      double beta_dot = a_y / v - yaw_rate;
      
      double cos_yaw_beta = std::cos(state.yaw + slip_angle);
      double sin_yaw_beta = std::sin(state.yaw + slip_angle);
      
      state.x += v * cos_yaw_beta * config_.dt;
      state.y += v * sin_yaw_beta * config_.dt;
      state.yaw += yaw_rate * config_.dt;
      state.v += u.acceleration * config_.dt;
      yaw_rate += r_dot * config_.dt;
      slip_angle += beta_dot * config_.dt;
      
      yaw_rate = std::clamp(yaw_rate, -3.0, 3.0);
      slip_angle = std::clamp(slip_angle, -0.3, 0.3);
      
      state.yaw = normalizeAngle(state.yaw);
      
      state.v = std::clamp(state.v, config_.min_speed, config_.max_speed);
      
      states.push_back(state);
    }
    
    return states;
  }
  
    double computeCost(
    const std::vector<VehicleState>& states,
    const std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference) const
  {
    double cost = 0.0;
    
    for (size_t i = 0; i < states.size() && i < reference.size(); ++i) {
      double dx = states[i].x - reference[i].x;
      double dy = states[i].y - reference[i].y;
      double dyaw = normalizeAngle(states[i].yaw - reference[i].yaw);
      double dv = states[i].v - reference[i].v;
      
      double ref_cos = std::cos(reference[i].yaw);
      double ref_sin = std::sin(reference[i].yaw);
      double lateral_error = -dx * ref_sin + dy * ref_cos;
      double longitudinal_error = dx * ref_cos + dy * ref_sin;
      
      double lateral_excess = std::max(0.0, std::abs(lateral_error) - config_.lateral_tolerance);
      
      double lateral_weight_factor = 1.0;
      double heading_weight_factor = 1.0;
      if (std::abs(lateral_error) < 0.1) {
        heading_weight_factor = 2.0;
        lateral_weight_factor = 0.5;
      }
      
      cost += config_.w_pos * lateral_weight_factor * lateral_excess * lateral_excess;
      cost += config_.w_pos * 0.3 * longitudinal_error * longitudinal_error;  
      
      cost += config_.w_yaw * heading_weight_factor * dyaw * dyaw;
      
      cost += config_.w_vel * dv * dv;
    }
    
    if (!states.empty() && !reference.empty()) {
      size_t last_idx = std::min(states.size() - 1, reference.size() - 1);
      double dx_term = states[last_idx].x - reference[last_idx].x;
      double dy_term = states[last_idx].y - reference[last_idx].y;
      double dyaw_term = normalizeAngle(states[last_idx].yaw - reference[last_idx].yaw);
      cost += config_.w_terminal * (dx_term * dx_term + dy_term * dy_term);
      cost += config_.w_terminal * 0.5 * dyaw_term * dyaw_term;
    }
    
    for (size_t i = 0; i < controls.size(); ++i) {
      cost += config_.w_steer * controls[i].steering * controls[i].steering;
      cost += config_.w_accel * controls[i].acceleration * controls[i].acceleration;
      
      if (i > 0) {
        double d_steer = controls[i].steering - controls[i-1].steering;
        double d_accel = controls[i].acceleration - controls[i-1].acceleration;
        cost += config_.w_steer_rate * d_steer * d_steer;
        cost += config_.w_accel_rate * d_accel * d_accel;
      }
    }
    
    if (!controls.empty() && !u_prev_.empty()) {
      double d_steer_init = controls[0].steering - u_prev_[0].steering;
      double d_accel_init = controls[0].acceleration - u_prev_[0].acceleration;
      cost += config_.w_steer_rate * d_steer_init * d_steer_init;
      cost += config_.w_accel_rate * d_accel_init * d_accel_init;
    }
    
    return cost;
  }
  
    double computeCostWithSoftConstraints(
    const std::vector<VehicleState>& states,
    const std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference,
    double& slack_violation) const
  {
    double cost = computeCost(states, controls, reference);
    slack_violation = 0.0;
    
    for (size_t i = 0; i < states.size(); ++i) {
      double speed_excess_high = std::max(0.0, states[i].v - config_.max_speed);
      double speed_excess_low = std::max(0.0, config_.min_speed - states[i].v);
      double speed_slack = speed_excess_high + speed_excess_low;
      
      cost += config_.w_slack_speed * (speed_slack * speed_slack + 0.1 * speed_slack);
      slack_violation += speed_slack;
      
    }
    
    for (size_t i = 0; i < controls.size(); ++i) {
      double steer_excess = std::max(0.0, std::abs(controls[i].steering) - config_.max_steer);
      cost += config_.w_slack_track * steer_excess * steer_excess;
      slack_violation += steer_excess;
      
      double accel_excess_high = std::max(0.0, controls[i].acceleration - config_.max_accel);
      double accel_excess_low = std::max(0.0, config_.min_accel - controls[i].acceleration);
      double accel_slack = accel_excess_high + accel_excess_low;
      cost += config_.w_slack_speed * accel_slack * accel_slack;
      slack_violation += accel_slack;
      
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
  
    double updateControlsWithRegularization(
    const VehicleState& current_state,
    const std::vector<VehicleState>& ,
    std::vector<ControlInput>& controls,
    const std::vector<ReferencePoint>& reference,
    double lambda)
  {
    constexpr double BASE_LEARNING_RATE_STEER = 0.25;
    constexpr double BASE_LEARNING_RATE_ACCEL = 0.20;
    constexpr double GRADIENT_EPSILON = 1e-4;
    constexpr double MAX_STEP_STEER = 0.1;
    constexpr double MAX_STEP_ACCEL = 0.5;
    
    double reg_factor = 1.0 / (1.0 + lambda);
    double learning_rate_steer = BASE_LEARNING_RATE_STEER * reg_factor;
    double learning_rate_accel = BASE_LEARNING_RATE_ACCEL * reg_factor;
    
    double total_step = 0.0;
    
    std::vector<double> grad_steer(controls.size(), 0.0);
    std::vector<double> grad_accel(controls.size(), 0.0);
    
    double slack_dummy = 0.0;
    for (size_t i = 0; i < controls.size(); ++i) {
      std::vector<ControlInput> u_plus = controls;
      std::vector<ControlInput> u_minus = controls;
      u_plus[i].steering += GRADIENT_EPSILON;
      u_minus[i].steering -= GRADIENT_EPSILON;
      
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
      
      if (std::isnan(grad_steer[i]) || std::isinf(grad_steer[i])) {
        grad_steer[i] = 0.0;
      }
      
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
      
      if (std::isnan(grad_accel[i]) || std::isinf(grad_accel[i])) {
        grad_accel[i] = 0.0;
      }
    }
    
    for (size_t i = 0; i < controls.size(); ++i) {
      double step_steer = learning_rate_steer * grad_steer[i];
      double step_accel = learning_rate_accel * grad_accel[i];
      
      step_steer = std::clamp(step_steer, -MAX_STEP_STEER, MAX_STEP_STEER);
      step_accel = std::clamp(step_accel, -MAX_STEP_ACCEL, MAX_STEP_ACCEL);
      
      controls[i].steering -= step_steer;
      controls[i].acceleration -= step_accel;
      
      total_step += std::abs(step_steer) + std::abs(step_accel);
    }
    
    return total_step;
  }
  
    void applyConstraints(std::vector<ControlInput>& controls) const
  {
    for (size_t i = 0; i < controls.size(); ++i) {
      controls[i].steering = std::clamp(controls[i].steering, 
        -config_.max_steer, config_.max_steer);
      
      controls[i].acceleration = std::clamp(controls[i].acceleration,
        config_.min_accel, config_.max_accel);
      
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
  
    static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
};

}  

class NMPCEngineNode : public rclcpp::Node
{
public:
  static constexpr double MIN_CREEP_SPEED = 0.3;              
  static constexpr double BRIEF_STOP_DURATION = 0.1;          
  static constexpr double OVERTAKE_COMPLETION_TIMEOUT = 2.0;  
  static constexpr double OVERTAKE_SAFETY_MARGIN = 0.3;       
  static constexpr double DEFAULT_OBSTACLE_SPEED = 0.5;       
  
  NMPCEngineNode()
  : Node("nmpc_engine_node")
  {
    declare_parameter("prediction_horizon", 1.5);   
    declare_parameter("prediction_steps", 15);      
    declare_parameter("control_rate_hz", 50.0);
    declare_parameter("nominal_speed", 2.5);        
    declare_parameter("solver_wheelbase", 0.33);
    
    declare_parameter("w_pos", 10.0);
    declare_parameter("w_yaw", 10.0);          
    declare_parameter("w_vel", 3.0);           
    declare_parameter("w_steer", 0.5);         
    declare_parameter("w_accel", 0.3);
    declare_parameter("w_steer_rate", 600.0);  
    declare_parameter("w_accel_rate", 60.0);
    declare_parameter("w_terminal", 30.0);     
    
    declare_parameter("lateral_tolerance", 0.25);  
    
    declare_parameter("latency_compensation_sec", 0.05);  
    
    declare_parameter("levenberg_marquardt", 0.01);       
    declare_parameter("max_solver_iterations", 20);       
    
    declare_parameter("dynamic_model_threshold", 2.5);    
    declare_parameter("vehicle_mass", 3.5);               
    declare_parameter("vehicle_inertia", 0.04);           
    
    declare_parameter("max_steer", 0.6458);  
    declare_parameter("max_steer_rate", 1.8);  
    declare_parameter("max_speed", 6.0);       
    declare_parameter("max_accel", 4.0);       
    declare_parameter("min_accel", -6.0);      
    
    declare_parameter<std::string>("odom_topic", "/dual_ekf/global_odom");  
    declare_parameter<std::string>("path_topic", "/global_raceline");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("scan_topic", "/scan");  
    declare_parameter<bool>("use_dual_ekf", true);  
    
    declare_parameter("a1_threshold", 0.3);            
    declare_parameter("a2_threshold", 0.4);            
    declare_parameter("a2_urgent_threshold", 0.25);    
    declare_parameter("a2_max_steer_ratio", 1.0);      
    declare_parameter("a2_urgent_steer_ratio", 1.0);   
    declare_parameter("reverse_speed", 1.5);           
    declare_parameter("reverse_duration", 0.5);        
    declare_parameter("stop_duration", 0.3);           
    declare_parameter("a1_steer_gain", 1.0);           
    declare_parameter("a2_steer_gain", 1.5);           
    declare_parameter("a2_urgent_steer_gain", 2.0);    
    declare_parameter("enable_collision_avoidance", true);
    
    declare_parameter("opponent_detection_range", 3.0);    
    declare_parameter("opponent_following_distance", 1.0); 
    declare_parameter("overtake_path_width", 0.8);         
    declare_parameter("overtake_decision_distance", 2.5);  
    declare_parameter("enable_overtaking", true);          
    declare_parameter("opponent_speed_tracking_gain", 0.8); 
    
    declare_parameter("follow_margin", 0.05);              
    declare_parameter("v_min_follow", 0.3);                
    
    declare_parameter("overtake_boost", 0.25);             
    declare_parameter("overtake_min_commit_time", 2.0);    
    declare_parameter("overtake_completion_timeout", 3.0); 
    
    declare_parameter("overtake_opponent_width", 0.35);    
    declare_parameter("overtake_width_factor", 1.2);       
    declare_parameter("overtake_safety_margin", 0.25);     
    declare_parameter("overtake_min_longitudinal_window", 5.0); 
    
    declare_parameter("curvature_k1", 0.2);                
    declare_parameter("curvature_k2", 0.8);                
    declare_parameter("v_max_straight", 4.5);              
    declare_parameter("v_min_corner", 1.2);                
    declare_parameter("curvature_lookahead", 5);           
    declare_parameter("corner_steer_threshold", 0.25);     
    declare_parameter("corner_speed_cap", 2.5);            
    
    declare_parameter("corner_exit_wall_margin", 0.4);     
    declare_parameter("corner_exit_lateral_limit", 0.15);  
    declare_parameter("corner_exit_steer_rate_limit", 0.8);
    declare_parameter("corner_exit_transition_time", 1.0); 
    declare_parameter("enable_corner_exit_smoothing", true);
    
    declare_parameter<std::vector<double>>("overtake_zone_starts", std::vector<double>{0.0});
    declare_parameter<std::vector<double>>("overtake_zone_ends", std::vector<double>{1000.0}); 

    double prediction_horizon = get_parameter("prediction_horizon").as_double();
    prediction_horizon_ = prediction_horizon;  
    int prediction_steps = get_parameter("prediction_steps").as_int();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    nominal_speed_ = get_parameter("nominal_speed").as_double();
    max_steer_ = get_parameter("max_steer").as_double();
    
    std::string odom_topic = get_parameter("odom_topic").as_string();
    std::string path_topic = get_parameter("path_topic").as_string();
    std::string drive_topic = get_parameter("drive_topic").as_string();
    std::string scan_topic = get_parameter("scan_topic").as_string();
    use_dual_ekf_ = get_parameter("use_dual_ekf").as_bool();
    
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
    
    opponent_detection_range_ = get_parameter("opponent_detection_range").as_double();
    opponent_following_distance_ = get_parameter("opponent_following_distance").as_double();
    overtake_path_width_ = get_parameter("overtake_path_width").as_double();
    overtake_decision_distance_ = get_parameter("overtake_decision_distance").as_double();
    enable_overtaking_ = get_parameter("enable_overtaking").as_bool();
    opponent_speed_tracking_gain_ = get_parameter("opponent_speed_tracking_gain").as_double();
    
    follow_margin_ = get_parameter("follow_margin").as_double();
    v_min_follow_ = get_parameter("v_min_follow").as_double();
    overtake_boost_ = get_parameter("overtake_boost").as_double();
    overtake_min_commit_time_ = get_parameter("overtake_min_commit_time").as_double();
    overtake_completion_timeout_ = get_parameter("overtake_completion_timeout").as_double();
    
    overtake_opponent_width_ = get_parameter("overtake_opponent_width").as_double();
    overtake_width_factor_ = get_parameter("overtake_width_factor").as_double();
    overtake_safety_margin_ = get_parameter("overtake_safety_margin").as_double();
    overtake_min_longitudinal_window_ = get_parameter("overtake_min_longitudinal_window").as_double();
    
    curvature_k1_ = get_parameter("curvature_k1").as_double();
    curvature_k2_ = get_parameter("curvature_k2").as_double();
    v_max_straight_ = get_parameter("v_max_straight").as_double();
    v_min_corner_ = get_parameter("v_min_corner").as_double();
    curvature_lookahead_ = get_parameter("curvature_lookahead").as_int();
    corner_steer_threshold_ = get_parameter("corner_steer_threshold").as_double();
    corner_speed_cap_ = get_parameter("corner_speed_cap").as_double();
    
    corner_exit_wall_margin_ = get_parameter("corner_exit_wall_margin").as_double();
    corner_exit_lateral_limit_ = get_parameter("corner_exit_lateral_limit").as_double();
    corner_exit_steer_rate_limit_ = get_parameter("corner_exit_steer_rate_limit").as_double();
    corner_exit_transition_time_ = get_parameter("corner_exit_transition_time").as_double();
    enable_corner_exit_smoothing_ = get_parameter("enable_corner_exit_smoothing").as_bool();
    
    overtake_zone_starts_ = get_parameter("overtake_zone_starts").as_double_array();
    overtake_zone_ends_ = get_parameter("overtake_zone_ends").as_double_array();
    
    recovery_cooldown_duration_ = reverse_duration_ * 2.0;
    
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

    nmpc::MPCConfig cfg;
    cfg.horizon_sec = prediction_horizon;
    cfg.prediction_steps = prediction_steps;
    cfg.wheelbase = get_parameter("solver_wheelbase").as_double();
    
    cfg.w_pos = get_parameter("w_pos").as_double();
    cfg.w_yaw = get_parameter("w_yaw").as_double();
    cfg.w_vel = get_parameter("w_vel").as_double();
    cfg.w_steer = get_parameter("w_steer").as_double();
    cfg.w_accel = get_parameter("w_accel").as_double();
    cfg.w_steer_rate = get_parameter("w_steer_rate").as_double();
    cfg.w_accel_rate = get_parameter("w_accel_rate").as_double();
    cfg.w_terminal = get_parameter("w_terminal").as_double();
    
    cfg.lateral_tolerance = get_parameter("lateral_tolerance").as_double();
    
    cfg.latency_compensation_sec = get_parameter("latency_compensation_sec").as_double();
    
    cfg.levenberg_marquardt = get_parameter("levenberg_marquardt").as_double();
    cfg.max_iterations = get_parameter("max_solver_iterations").as_int();
    
    cfg.dynamic_model_threshold = get_parameter("dynamic_model_threshold").as_double();
    cfg.vehicle_mass = get_parameter("vehicle_mass").as_double();
    cfg.vehicle_inertia = get_parameter("vehicle_inertia").as_double();
    
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

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic, rclcpp::QoS(10));
    
    nmpc_trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nmpc_predicted_trajectory", rclcpp::QoS(1));
    
    nmpc_reference_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nmpc_reference_points", rclcpp::QoS(1));
    
    overtake_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/overtake_paths", rclcpp::QoS(1).transient_local());
    
    opponent_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/detected_opponent", rclcpp::QoS(1));
    
    opponent_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/opp_racecar/odom", rclcpp::QoS(10).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_opponent_odom_ = msg;
      });
    
    RCLCPP_INFO(get_logger(), "NMPC visualization: /nmpc_predicted_trajectory (GREEN), /nmpc_reference_points (BLUE)");
    RCLCPP_INFO(get_logger(), "Overtaking visualization: /overtake_paths (MAGENTA/CYAN), /detected_opponent (RED)");

    const double period_s = 1.0 / std::max(1.0, control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s),
      std::bind(&NMPCEngineNode::controlCycle, this));
    
    collision_state_start_time_ = now();
    overtake_start_time_ = now();
    corner_exit_start_time_ = now();
    
    RCLCPP_INFO(get_logger(), "NMPC control loop running at %.1f Hz", control_rate_hz_);
    RCLCPP_INFO(get_logger(), "Corner exit smoothing: %s (margin=%.2fm, lateral_limit=%.2fm, steer_limit=%.1f%%)",
                enable_corner_exit_smoothing_ ? "ENABLED" : "DISABLED",
                corner_exit_wall_margin_, corner_exit_lateral_limit_, corner_exit_steer_rate_limit_ * 100);
  }

private:
    enum class CollisionState {
    NORMAL,           
    STOPPING,         
    WAITING,          
    REVERSING,        
    COOLDOWN          
  };
  
    enum class DrivingMode {
    CRUISE,
    FOLLOW,
    OVERTAKE_CANDIDATE,
    OVERTAKE,
    OBSTACLE_STOP
  };
  
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
    
    updateObstacleDistances();
    
    OpponentInfo opponent = detectOpponent();
    
    publishOpponentVisualization(opponent);
    
    if (enable_overtaking_ && latest_path_) {
      generateAndPublishOvertakePaths();
    }
    
    
    double elapsed = (current_time - collision_state_start_time_).seconds();
    
    switch (collision_state_) {
      case CollisionState::NORMAL:
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
          ackermann_msgs::msg::AckermannDriveStamped cmd;
          cmd.header.stamp = current_time;
          cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
          cmd.drive.speed = 0.0;
          cmd.drive.steering_angle = 0.0;
          drive_pub_->publish(cmd);
          
          if (elapsed > BRIEF_STOP_DURATION) {
            collision_state_ = CollisionState::WAITING;
            collision_state_start_time_ = current_time;
            RCLCPP_INFO(get_logger(), "Stopped. Holding for %.2fs...", stop_duration_);
          }
          return;
        }
        
      case CollisionState::WAITING:
        {
          ackermann_msgs::msg::AckermannDriveStamped cmd;
          cmd.header.stamp = current_time;
          cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
          cmd.drive.speed = 0.0;
          cmd.drive.steering_angle = 0.0;
          drive_pub_->publish(cmd);
          
          if (elapsed > stop_duration_) {
            if (last_obstacle_front_dist_ < a1_threshold_ * 0.8) {
              collision_state_ = CollisionState::REVERSING;
              collision_state_start_time_ = current_time;
              RCLCPP_INFO(get_logger(), "Still blocked. Gentle reverse for %.2fs...", reverse_duration_);
            } else {
              collision_state_ = CollisionState::COOLDOWN;
              collision_state_start_time_ = current_time;
              RCLCPP_INFO(get_logger(), "Space cleared. Resuming after cooldown.");
            }
          }
          return;
        }
        
      case CollisionState::REVERSING:
        {
          ackermann_msgs::msg::AckermannDriveStamped cmd;
          cmd.header.stamp = current_time;
          cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
          cmd.drive.speed = -reverse_speed_ * 0.5;
          cmd.drive.steering_angle = 0.0;  
          drive_pub_->publish(cmd);
          
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
            "GENTLE REVERSE: speed=%.2f, steer=0.0", cmd.drive.speed);
          
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
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
              "Recovery cooldown: %.1fs remaining",
              recovery_cooldown_duration_ - elapsed);
          }
        }
        break;
    }

    nmpc::VehicleState current_state = extractState(latest_odom_);
    
    updateEgoSPosition(current_state);
    
    std::vector<nmpc::ReferencePoint> reference = buildReference(current_state);
    
    if (reference.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Empty reference trajectory");
      return;
    }
    
    auto solution = solver_.solve(current_state, reference, get_logger());
    
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
    
    
    if (enable_corner_exit_smoothing_) {
      double current_curvature = calculateCurrentCurvature(reference);
      
      bool is_corner_exit = detectCornerExit(current_curvature);
      
      if (is_corner_exit && !in_corner_exit_transition_) {
        in_corner_exit_transition_ = true;
        corner_exit_start_time_ = current_time;
        RCLCPP_INFO(get_logger(), 
          "CORNER EXIT detected: curvature %.3f -> %.3f, applying smoothing for %.1fs",
          last_curvature_, current_curvature, corner_exit_transition_time_);
      }
      
      if (in_corner_exit_transition_) {
        double elapsed = (current_time - corner_exit_start_time_).seconds();
        double transition_factor = std::min(1.0, elapsed / corner_exit_transition_time_);
        
        applyCornerExitSmoothing(solution, reference, current_state, transition_factor);
        
        if (elapsed >= corner_exit_transition_time_) {
          in_corner_exit_transition_ = false;
          RCLCPP_INFO(get_logger(), "Corner exit transition complete");
        }
      }
      
      last_curvature_ = current_curvature;
    }
    
    
    bool in_overtake_zone = isInOvertakeZone();
    
    updateDrivingMode(opponent, in_overtake_zone, current_time);
    
    switch (driving_mode_) {
      case DrivingMode::CRUISE:
        // Normal NMPC behavior - no additional speed modification
        break;
        
      case DrivingMode::FOLLOW:
        {
          // FOLLOW mode: keep NMPC raceline-based speed, only log state.
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "FOLLOW: distance=%.2fm, keeping NMPC speed=%.2f (opponent=%.2f)",
            opponent.distance, solution.speed, opponent.speed);
        }
        break;
        
      case DrivingMode::OVERTAKE_CANDIDATE:
        {
          // Do not slow down aggressively in candidate phase; just log.
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "OVERTAKE_CANDIDATE: distance=%.2fm, eval only (NMPC speed=%.2f)",
            opponent.distance, solution.speed);
        }
        break;
        
      case DrivingMode::OVERTAKE:
        {
          // Preserve original lateral offset behavior and mild speed boost.
          double lateral_offset = overtake_left_side_ ? overtake_path_width_ : -overtake_path_width_;
          
          for (auto& ref : reference) {
            double cos_yaw = std::cos(ref.yaw);
            double sin_yaw = std::sin(ref.yaw);
            ref.x += lateral_offset * (-sin_yaw);
            ref.y += lateral_offset * cos_yaw;
          }
          
          double v_overtake = std::min(
            opponent.speed + overtake_boost_,
            v_max_straight_  
          );
          solution.speed = std::max(solution.speed, v_overtake);
          
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "OVERTAKE %s: offset=%.2fm, speed=%.2f (boost=%.2f above opponent)",
            overtake_left_side_ ? "LEFT" : "RIGHT", lateral_offset, 
            solution.speed, solution.speed - opponent.speed);
        }
        break;
        
      case DrivingMode::OBSTACLE_STOP:
        solution.speed = 0.0;
        solution.steering = 0.0;
        break;
    }
    
    [[maybe_unused]] auto unused_check_a2 = &NMPCEngineNode::checkA2Zone;
    
    // Corner speed cap based on steering magnitude (safety layer):
    // If steering angle is large (cornering), clamp speed so corners are taken slower.
    {
      double steer_abs = std::abs(solution.steering);
      if (steer_abs > corner_steer_threshold_) {
        double old_speed = solution.speed;
        solution.speed = std::min(solution.speed, corner_speed_cap_);
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
          "Corner speed cap: |steer|=%.3f > %.3f, speed=%.2f -> %.2f",
          steer_abs, corner_steer_threshold_, old_speed, solution.speed);
      }
    }
    
    publishNMPCVisualization(solution.predicted_trajectory, reference);
    
    publishDriveCommand(solution);
    
    const char* model_type = (current_state.v > 2.5) ? "DYN" : "KIN";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,  
      "MPC[%s]: state(%.2f, %.2f, %.1f°, %.2f m/s) -> cmd(steer=%.3f, speed=%.2f) [%d iters, cost=%.1f]",
      model_type, current_state.x, current_state.y, current_state.yaw * 180.0 / M_PI, current_state.v,
      solution.steering, solution.speed, solution.iterations, solution.cost);
  }
  
    void updateObstacleDistances()
  {
    if (!enable_collision_avoidance_ || !scan_received_ || !latest_scan_) {
      last_obstacle_front_dist_ = 10.0;
      last_obstacle_left_dist_ = 10.0;
      last_obstacle_right_dist_ = 10.0;
      last_obstacle_angle_ = 0.0;
      return;
    }
    
    constexpr double FRONT_SECTOR_HALF_ANGLE = M_PI / 3.0;   
    constexpr double SIDE_INNER = M_PI / 6.0;               
    constexpr double SIDE_OUTER = M_PI / 2.0;               
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
      
      if (angle >= -FRONT_SECTOR_HALF_ANGLE && angle <= FRONT_SECTOR_HALF_ANGLE) {
        if (range < min_front) {
          min_front = range;
          front_angle = angle;
        }
      }
      
      if (angle >= SIDE_INNER && angle <= SIDE_OUTER) {
        if (range < min_left) {
          min_left = range;
        }
      }
      
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
  
    bool checkA1Zone()
  {
    if (!enable_collision_avoidance_ || !scan_received_) {
      is_in_a1_zone_ = false;
      return false;
    }
    
    is_in_a1_zone_ = (last_obstacle_front_dist_ < a1_threshold_);
    
    if (is_in_a1_zone_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "A1 Zone! Front: %.2fm (threshold: %.2fm) - triggering reverse",
        last_obstacle_front_dist_, a1_threshold_);
    }
    
    return is_in_a1_zone_;
  }
  
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
  
    struct OpponentInfo {
    bool is_detected = false;
    bool is_ahead = false;
    double distance = 10.0;    
    double speed = 0.0;        
    double x = 0.0;            
    double y = 0.0;            
    double relative_angle = 0.0; 
  };
  
    OpponentInfo detectOpponent()
  {
    OpponentInfo opponent;
    
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
      
      if (opponent.distance < opponent_detection_range_) {
        opponent.is_detected = true;
        
        double cos_yaw = std::cos(-ego_yaw);
        double sin_yaw = std::sin(-ego_yaw);
        double dx_vehicle = dx * cos_yaw - dy * sin_yaw;
        double dy_vehicle = dx * sin_yaw + dy * cos_yaw;
        opponent.relative_angle = std::atan2(dy_vehicle, dx_vehicle);
        
        opponent.is_ahead = (dx_vehicle > 0 && std::abs(opponent.relative_angle) < M_PI / 4.0);
        
        double vx = latest_opponent_odom_->twist.twist.linear.x;
        double vy = latest_opponent_odom_->twist.twist.linear.y;
        opponent.speed = std::sqrt(vx * vx + vy * vy);
      }
    }
    
    if (!opponent.is_detected && scan_received_ && latest_scan_) {
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
    
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    opponent_marker_pub_->publish(marker);
  }
  
    void generateAndPublishOvertakePaths()
  {
    if (!latest_path_ || latest_path_->poses.empty()) {
      return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    double cumulative_s = 0.0;
    std::vector<double> s_positions;
    s_positions.push_back(0.0);
    for (size_t i = 1; i < latest_path_->poses.size(); ++i) {
      double dx = latest_path_->poses[i].pose.position.x - latest_path_->poses[i-1].pose.position.x;
      double dy = latest_path_->poses[i].pose.position.y - latest_path_->poses[i-1].pose.position.y;
      cumulative_s += std::sqrt(dx * dx + dy * dy);
      s_positions.push_back(cumulative_s);
    }
    
    visualization_msgs::msg::Marker zone_marker;
    zone_marker.header.frame_id = "map";
    zone_marker.header.stamp = now();
    zone_marker.ns = "overtake_zones";
    zone_marker.id = 10;
    zone_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    zone_marker.action = visualization_msgs::msg::Marker::ADD;
    zone_marker.scale.x = 0.1;  
    zone_marker.color.r = 0.0f;
    zone_marker.color.g = 0.8f;  
    zone_marker.color.b = 0.2f;
    zone_marker.color.a = 0.8f;
    zone_marker.pose.orientation.w = 1.0;
    
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
        p.z = 0.15;  
        zone_marker.points.push_back(p);
      }
    }
    zone_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    marker_array.markers.push_back(zone_marker);
    
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
    left_path.color.b = 1.0f;  
    left_path.color.a = 0.7f;
    left_path.pose.orientation.w = 1.0;
    
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
    right_path.color.b = 1.0f;  
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
      
      double cos_yaw = std::cos(yaw);
      double sin_yaw = std::sin(yaw);
      
      geometry_msgs::msg::Point left_pt;
      left_pt.x = x + overtake_path_width_ * (-sin_yaw);
      left_pt.y = y + overtake_path_width_ * cos_yaw;
      left_pt.z = 0.05;
      left_path.points.push_back(left_pt);
      overtake_left_path_.push_back(left_pt);
      
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
    
    if (driving_mode_ == DrivingMode::OVERTAKE && overtake_committed_) {
      visualization_msgs::msg::Marker active_path;
      active_path.header.frame_id = "map";
      active_path.header.stamp = now();
      active_path.ns = "active_overtake_path";
      active_path.id = 20;
      active_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
      active_path.action = visualization_msgs::msg::Marker::ADD;
      active_path.scale.x = 0.12;  
      active_path.color.r = 1.0f;
      active_path.color.g = 1.0f;  
      active_path.color.b = 0.0f;
      active_path.color.a = 1.0f;
      active_path.pose.orientation.w = 1.0;
      
      const auto& active_pts = overtake_left_side_ ? overtake_left_path_ : overtake_right_path_;
      for (const auto& pt : active_pts) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.2;  
        active_path.points.push_back(p);
      }
      active_path.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(active_path);
    }
    
    overtake_path_pub_->publish(marker_array);
  }
  
    bool canStartOvertake(const OpponentInfo& opponent)
  {
    if (!enable_overtaking_ || !opponent.is_detected || !opponent.is_ahead) {
      return false;
    }
    
    if (opponent.distance > overtake_decision_distance_) {
      return false;
    }
    
    
    bool left_clear = checkOvertakeLateralClearance(true, opponent);
    bool right_clear = checkOvertakeLateralClearance(false, opponent);
    
    if (!left_clear && !right_clear) {
      return false;
    }
    
    if (!checkOvertakeLongitudinalWindow()) {
      return false;
    }
    
    return true;
  }
  
    bool checkOvertakeLateralClearance(bool left_side, [[maybe_unused]] const OpponentInfo& opponent) const
  {
    double clearance = left_side ? last_obstacle_left_dist_ : last_obstacle_right_dist_;
    
    double min_required = overtake_opponent_width_ * overtake_width_factor_ + 
                          overtake_safety_margin_ + overtake_path_width_;
    
    return clearance > min_required;
  }
  
    bool checkOvertakeLongitudinalWindow() const
  {
    if (overtake_zone_starts_.size() != overtake_zone_ends_.size()) {
      RCLCPP_WARN_ONCE(get_logger(), 
        "Overtake zone vectors have mismatched sizes: starts=%zu, ends=%zu",
        overtake_zone_starts_.size(), overtake_zone_ends_.size());
      return false;  
    }
    
    size_t num_zones = overtake_zone_starts_.size();
    for (size_t i = 0; i < num_zones; ++i) {
      if (ego_s_position_ >= overtake_zone_starts_[i] && 
          ego_s_position_ <= overtake_zone_ends_[i]) {
        double remaining = overtake_zone_ends_[i] - ego_s_position_;
        return remaining >= overtake_min_longitudinal_window_;
      }
    }
    return false;
  }
  
    bool canOvertakeOnSide(bool left_side)
  {
    double clearance = left_side ? last_obstacle_left_dist_ : last_obstacle_right_dist_;
    double min_required = overtake_opponent_width_ * overtake_width_factor_ + 
                          overtake_safety_margin_ + overtake_path_width_;
    return clearance > min_required;
  }
  
    bool shouldAbortOvertake()
  {
    if (!overtake_committed_) {
      return false;
    }
    
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
  
    bool isInOvertakeZone() const
  {
    if (overtake_zone_starts_.empty() || overtake_zone_ends_.empty()) {
      return false;
    }
    
    size_t num_zones = std::min(overtake_zone_starts_.size(), overtake_zone_ends_.size());
    for (size_t i = 0; i < num_zones; ++i) {
      if (ego_s_position_ >= overtake_zone_starts_[i] && 
          ego_s_position_ <= overtake_zone_ends_[i]) {
        return true;
      }
    }
    return false;
  }
  
    void updateDrivingMode(const OpponentInfo& opponent, bool in_overtake_zone, 
                         const rclcpp::Time& current_time)
  {
    if (collision_state_ != CollisionState::NORMAL && 
        collision_state_ != CollisionState::COOLDOWN) {
      driving_mode_ = DrivingMode::OBSTACLE_STOP;
      return;
    }
    
    double overtake_elapsed = (current_time - overtake_start_time_).seconds();
    
    if (overtake_committed_) {
      if (shouldAbortOvertake()) {
        RCLCPP_WARN(get_logger(), "OVERTAKE ABORTED: clearance became unsafe");
        overtake_committed_ = false;
        is_overtaking_ = false;
        driving_mode_ = DrivingMode::FOLLOW;
        return;
      }
      
      if (!opponent.is_ahead || !opponent.is_detected) {
        if (overtake_elapsed > overtake_completion_timeout_) {
          overtake_committed_ = false;
          is_overtaking_ = false;
          driving_mode_ = DrivingMode::CRUISE;
          RCLCPP_INFO(get_logger(), "OVERTAKE COMPLETE (passed opponent for %.1fs)", 
                      overtake_elapsed);
          return;
        }
      }
      
      driving_mode_ = DrivingMode::OVERTAKE;
      return;
    }
    
    if (!opponent.is_detected || !opponent.is_ahead) {
      driving_mode_ = DrivingMode::CRUISE;
      is_overtaking_ = false;
      return;
    }
    
    double distance = opponent.distance;
    
    if (distance > opponent_following_distance_ && distance > overtake_decision_distance_) {
      driving_mode_ = DrivingMode::CRUISE;
      return;
    }
    
    if (in_overtake_zone && enable_overtaking_) {
      bool left_clear = canOvertakeOnSide(true);
      bool right_clear = canOvertakeOnSide(false);
      bool longitudinal_ok = checkOvertakeLongitudinalWindow();
      
      if ((left_clear || right_clear) && longitudinal_ok) {
        if (driving_mode_ != DrivingMode::OVERTAKE_CANDIDATE) {
          driving_mode_ = DrivingMode::OVERTAKE_CANDIDATE;
          overtake_start_time_ = current_time;  
          RCLCPP_INFO(get_logger(), "OVERTAKE_CANDIDATE: distance=%.2fm, zone=true, left=%d, right=%d", 
                      distance, left_clear, right_clear);
        }
        
        double candidate_time = (current_time - overtake_start_time_).seconds();
        if (candidate_time > 0.5 && distance < overtake_decision_distance_) {
          overtake_committed_ = true;
          is_overtaking_ = true;
          overtake_left_side_ = left_clear;  
          overtake_start_time_ = current_time;
          driving_mode_ = DrivingMode::OVERTAKE;
          RCLCPP_WARN(get_logger(), "OVERTAKE COMMITTED! side=%s, distance=%.2fm",
                      overtake_left_side_ ? "LEFT" : "RIGHT", distance);
        }
        return;
      } else if (!longitudinal_ok) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
          "In overtake zone but longitudinal window insufficient, staying in FOLLOW");
      }
    }
    
    driving_mode_ = DrivingMode::FOLLOW;
  }
  
    void updateEgoSPosition(const nmpc::VehicleState& current_state)
  {
    if (!latest_path_ || latest_path_->poses.empty()) {
      ego_s_position_ = 0.0;
      return;
    }
    
    const auto& poses = latest_path_->poses;
    const size_t num_poses = poses.size();
    
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
    
    double s = 0.0;
    for (size_t i = 1; i <= closest_idx && i < num_poses; ++i) {
      double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
      double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
      s += std::sqrt(dx * dx + dy * dy);
    }
    
    ego_s_position_ = s;
  }
  
    double findGapCenterAngle(double lookahead_angle)
  {
    if (!scan_received_ || !latest_scan_ || latest_scan_->ranges.empty()) {
      return lookahead_angle;
    }
    
    double angle_min = latest_scan_->angle_min;
    double angle_inc = latest_scan_->angle_increment;
    int num_ranges = latest_scan_->ranges.size();
    
    constexpr double SEARCH_HALF_ANGLE = M_PI / 3.0;  
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
  
    double calculateCurrentCurvature(const std::vector<nmpc::ReferencePoint>& reference) const
  {
    if (reference.size() < 3) {
      return 0.0;
    }
    
    double total_curvature = 0.0;
    int count = 0;
    int lookahead = std::min(static_cast<int>(reference.size()) - 1, curvature_lookahead_);
    
    constexpr double MIN_SEGMENT_DISTANCE = 0.01;  
    
    for (int i = 0; i < lookahead; ++i) {
      double dx = reference[i + 1].x - reference[i].x;
      double dy = reference[i + 1].y - reference[i].y;
      double dist = std::sqrt(dx * dx + dy * dy);
      
      if (dist > MIN_SEGMENT_DISTANCE) {
        double dyaw = reference[i + 1].yaw - reference[i].yaw;
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        
        double curvature = std::abs(dyaw) / dist;
        total_curvature += curvature;
        count++;
      }
    }
    
    return (count > 0) ? total_curvature / count : 0.0;
  }
  
    bool detectCornerExit(double current_curvature) const
  {
    bool was_in_corner = last_curvature_ > curvature_k1_;
    bool now_on_straight = current_curvature < curvature_k1_;
    
    return was_in_corner && now_on_straight;
  }
  
    void applyCornerExitSmoothing(
    nmpc::MPCSolution& solution,
    const std::vector<nmpc::ReferencePoint>& reference,
    const nmpc::VehicleState& current_state,
    double transition_factor)
  {
    if (reference.empty() || !enable_corner_exit_smoothing_) {
      return;
    }
    
    constexpr double INITIAL_LIMIT_SCALE = 2.0;
    constexpr double MAX_CORRECTION_FACTOR = 0.5;
    constexpr double MAX_WALL_AVOIDANCE_FACTOR = 0.5;
    
    double steer_rate_factor = corner_exit_steer_rate_limit_ + 
                               (1.0 - corner_exit_steer_rate_limit_) * transition_factor;
    
    solution.steering *= steer_rate_factor;
    
    if (!reference.empty()) {
      double dx = current_state.x - reference[0].x;
      double dy = current_state.y - reference[0].y;
      
      double ref_cos = std::cos(reference[0].yaw);
      double ref_sin = std::sin(reference[0].yaw);
      double lateral_error = -dx * ref_sin + dy * ref_cos;
      
      double max_lateral = corner_exit_lateral_limit_ * (INITIAL_LIMIT_SCALE - transition_factor);
      
      if (std::abs(lateral_error) > max_lateral) {
        double correction_factor = MAX_CORRECTION_FACTOR * (1.0 - transition_factor);
        
        if ((lateral_error > 0 && solution.steering < 0) ||
            (lateral_error < 0 && solution.steering > 0)) {
        } else {
          solution.steering *= (1.0 - correction_factor);
        }
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 200,
          "Corner exit: lateral=%.2fm > limit=%.2fm, reducing steer by %.0f%%",
          std::abs(lateral_error), max_lateral, correction_factor * 100);
      }
    }
    
    double min_side_dist = std::min(last_obstacle_left_dist_, last_obstacle_right_dist_);
    if (min_side_dist < corner_exit_wall_margin_) {
      double wall_urgency = 1.0 - (min_side_dist / corner_exit_wall_margin_);
      
      bool wall_on_left = (last_obstacle_left_dist_ < last_obstacle_right_dist_);
      
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
  
    double computeAvoidanceSteering(double )
  {
    return 0.0;
  }
  
    nmpc::VehicleState extractState(const nav_msgs::msg::Odometry::SharedPtr& odom) const
  {
    nmpc::VehicleState state;
    state.x = odom->pose.pose.position.x;
    state.y = odom->pose.pose.position.y;
    
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    state.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    state.v = std::sqrt(
      odom->twist.twist.linear.x * odom->twist.twist.linear.x +
      odom->twist.twist.linear.y * odom->twist.twist.linear.y);
    
    return state;
  }
  
    std::vector<nmpc::ReferencePoint> buildReference(const nmpc::VehicleState& current_state) const
  {
    std::vector<nmpc::ReferencePoint> reference;
    
    if (!latest_path_ || latest_path_->poses.empty()) {
      return reference;
    }
    
    const auto& poses = latest_path_->poses;
    const size_t num_poses = poses.size();
    
    constexpr int NUM_REF_POINTS = 15;              
    constexpr double BEHIND_PENALTY = 3.0;          
    constexpr size_t MAX_SEARCH_ITERATIONS = 20;    
    constexpr double MIN_AHEAD_DISTANCE = 0.05;     
    
    double cos_yaw = std::cos(current_state.yaw);
    double sin_yaw = std::sin(current_state.yaw);
    
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    size_t search_start = (last_closest_idx_ > 50) ? last_closest_idx_ - 50 : 0;
    size_t search_end = std::min(num_poses, last_closest_idx_ + 100);
    
    for (size_t i = search_start; i < search_end; ++i) {
      double dx = poses[i].pose.position.x - current_state.x;
      double dy = poses[i].pose.position.y - current_state.y;
      double dist_sq = dx * dx + dy * dy;
      
      double ahead = dx * cos_yaw + dy * sin_yaw;
      
      double weighted_dist = dist_sq;
      if (ahead < 0.0) {
        weighted_dist *= BEHIND_PENALTY;
      }
      
      if (weighted_dist < min_dist) {
        min_dist = weighted_dist;
        closest_idx = i;
      }
    }
    
    if (min_dist > 4.0) {  
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
    
    double speed = std::max(0.5, current_state.v);
    double total_lookahead_dist = speed * prediction_horizon_;
    
    double total_path_length = 0.0;
    for (size_t i = 0; i < num_poses - 1; ++i) {
      double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
      double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
      total_path_length += std::sqrt(dx * dx + dy * dy);
    }
    double avg_point_spacing = std::max(0.05, total_path_length / std::max<size_t>(1, num_poses - 1));
    
    double dist_per_ref_point = total_lookahead_dist / NUM_REF_POINTS;
    size_t stride = std::max<size_t>(1, static_cast<size_t>(dist_per_ref_point / avg_point_spacing));
    
    stride = std::min(stride, std::max<size_t>(1, num_poses / 5));
    
    for (int i = 0; i < NUM_REF_POINTS; ++i) {
      size_t idx = (start_idx + i * stride) % num_poses;
      
      nmpc::ReferencePoint ref;
      ref.x = poses[idx].pose.position.x;
      ref.y = poses[idx].pose.position.y;
      
      double qx = poses[idx].pose.orientation.x;
      double qy = poses[idx].pose.orientation.y;
      double qz = poses[idx].pose.orientation.z;
      double qw = poses[idx].pose.orientation.w;
      ref.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      
      // Start with straight-line speed
      ref.v = v_max_straight_;
      
      if (i < NUM_REF_POINTS - 1) {
        // --- Current segment curvature ---
        size_t next_idx = (start_idx + (i + 1) * stride) % num_poses;
        double dx = poses[next_idx].pose.position.x - poses[idx].pose.position.x;
        double dy = poses[next_idx].pose.position.y - poses[idx].pose.position.y;
        
        double next_qx = poses[next_idx].pose.orientation.x;
        double next_qy = poses[next_idx].pose.orientation.y;
        double next_qz = poses[next_idx].pose.orientation.z;
        double next_qw = poses[next_idx].pose.orientation.w;
        double next_yaw = std::atan2(2.0 * (next_qw * next_qz + next_qx * next_qy), 
                                      1.0 - 2.0 * (next_qy * next_qy + next_qz * next_qz));
        
        double dyaw = next_yaw - ref.yaw;
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        dyaw = std::abs(dyaw);
        
        double seg_dist = std::sqrt(dx * dx + dy * dy);
        if (seg_dist > 0.01) {
          double curvature_now = dyaw / seg_dist;
          double k_abs = std::abs(curvature_now);

          // --- Ahead-averaged curvature for pre-brake ---
          // Look ahead minimally and blend lightly to avoid early braking/turn-in.
          constexpr int AHEAD_SEGMENTS = 1;
          double sum_k = k_abs;
          int count_k = 1;
          for (int k = 1; k <= AHEAD_SEGMENTS; ++k) {
            int ref_idx = i + k;
            if (ref_idx >= NUM_REF_POINTS - 1) {
              break;
            }
            size_t idx_k = (start_idx + ref_idx * stride) % num_poses;
            size_t next_idx_k = (start_idx + (ref_idx + 1) * stride) % num_poses;
            double dx_k = poses[next_idx_k].pose.position.x - poses[idx_k].pose.position.x;
            double dy_k = poses[next_idx_k].pose.position.y - poses[idx_k].pose.position.y;
            double dist_k = std::sqrt(dx_k * dx_k + dy_k * dy_k);
            if (dist_k <= 0.01) {
              continue;
            }

            double qx_k = poses[idx_k].pose.orientation.x;
            double qy_k = poses[idx_k].pose.orientation.y;
            double qz_k = poses[idx_k].pose.orientation.z;
            double qw_k = poses[idx_k].pose.orientation.w;
            double yaw_k = std::atan2(2.0 * (qw_k * qz_k + qx_k * qy_k),
                                      1.0 - 2.0 * (qy_k * qy_k + qz_k * qz_k));

            double qx_kn = poses[next_idx_k].pose.orientation.x;
            double qy_kn = poses[next_idx_k].pose.orientation.y;
            double qz_kn = poses[next_idx_k].pose.orientation.z;
            double qw_kn = poses[next_idx_k].pose.orientation.w;
            double yaw_kn = std::atan2(2.0 * (qw_kn * qz_kn + qx_kn * qy_kn),
                                       1.0 - 2.0 * (qy_kn * qy_kn + qz_kn * qz_kn));

            double dyaw_k = yaw_kn - yaw_k;
            while (dyaw_k > M_PI) dyaw_k -= 2.0 * M_PI;
            while (dyaw_k < -M_PI) dyaw_k += 2.0 * M_PI;

            double curv_k = std::abs(dyaw_k) / dist_k;
            sum_k += curv_k;
            count_k++;
          }

          double k_ahead_avg = (count_k > 0) ? (sum_k / static_cast<double>(count_k)) : k_abs;
          // Blend current curvature with ahead curvature to avoid braking too early
          constexpr double K_BLEND = 0.1;  // even smaller blend to delay slow/turn
          double k_eff = (1.0 - K_BLEND) * k_abs + K_BLEND * k_ahead_avg;

          // Use effective curvature to set reference speed (pre-brake before corners)
          if (k_eff < curvature_k1_) {
            ref.v = v_max_straight_;
          } else if (k_eff > curvature_k2_) {
            ref.v = v_min_corner_;
          } else {
            double t = (k_eff - curvature_k1_) / (curvature_k2_ - curvature_k1_);
            ref.v = v_max_straight_ + t * (v_min_corner_ - v_max_straight_);
          }
        }
      }
      
      reference.push_back(ref);
    }
    
    return reference;
  }
  
    void publishDriveCommand(const nmpc::MPCSolution& solution)
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";
    
    if (solution.feasible) {
      cmd.drive.steering_angle = solution.steering;
      cmd.drive.speed = solution.speed;
    } else {
      cmd.drive.steering_angle = 0.0;
      cmd.drive.speed = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "MPC solution not feasible, stopping vehicle");
    }
    
    drive_pub_->publish(cmd);
  }
  
    void publishNMPCVisualization(
    const std::vector<nmpc::VehicleState>& predicted_trajectory,
    const std::vector<nmpc::ReferencePoint>& reference)
  {
    rclcpp::Time current_time = now();
    
    if (!predicted_trajectory.empty()) {
      visualization_msgs::msg::Marker traj_marker;
      traj_marker.header.frame_id = "map";
      traj_marker.header.stamp = current_time;
      traj_marker.ns = "nmpc_prediction";
      traj_marker.id = 0;
      traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      traj_marker.action = visualization_msgs::msg::Marker::ADD;
      
      traj_marker.scale.x = 0.08;  
      
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
      
      traj_marker.lifetime = rclcpp::Duration::from_seconds(0.2);  
      nmpc_trajectory_pub_->publish(traj_marker);
    }
    
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
      
      ref_marker.lifetime = rclcpp::Duration::from_seconds(0.2);  
      nmpc_reference_pub_->publish(ref_marker);
    }
  }

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

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Odometry::SharedPtr latest_opponent_odom_;
  nav_msgs::msg::Path::SharedPtr latest_path_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  bool scan_received_{false};

  double control_rate_hz_{50.0};
  double nominal_speed_{2.0};
  bool use_dual_ekf_{true};
  double max_steer_{0.436};  
  double prediction_horizon_{1.0};  
  
  double a1_threshold_{0.3};          
  double a2_threshold_{0.4};          
  double a2_urgent_threshold_{0.25};  
  double a2_max_steer_ratio_{1.0};    
  double a2_urgent_steer_ratio_{1.0}; 
  double reverse_speed_{1.5};         
  double reverse_duration_{0.5};      
  double stop_duration_{0.3};         
  double a1_steer_gain_{1.0};         
  double a2_steer_gain_{1.5};         
  double a2_urgent_steer_gain_{2.0};  
  bool enable_collision_avoidance_{true};
  
  double opponent_detection_range_{3.0};
  double opponent_following_distance_{1.0};
  double overtake_path_width_{0.8};
  double overtake_decision_distance_{2.5};
  bool enable_overtaking_{true};
  double opponent_speed_tracking_gain_{0.8};
  
  double follow_margin_{0.05};              
  double v_min_follow_{0.3};                
  
  double overtake_boost_{0.25};             
  double overtake_min_commit_time_{2.0};    
  double overtake_completion_timeout_{3.0}; 
  
  double overtake_opponent_width_{0.35};    
  double overtake_width_factor_{1.2};       
  double overtake_safety_margin_{0.25};     
  double overtake_min_longitudinal_window_{5.0}; 
  
  double curvature_k1_{0.2};                
  double curvature_k2_{0.8};                
  double v_max_straight_{4.5};              
  double v_min_corner_{1.2};                
  int curvature_lookahead_{5};              
  double corner_steer_threshold_{0.25};
  double corner_speed_cap_{2.5};
  
  double corner_exit_wall_margin_{0.4};     
  double corner_exit_lateral_limit_{0.15};  
  double corner_exit_steer_rate_limit_{0.8};
  double corner_exit_transition_time_{1.0}; 
  bool enable_corner_exit_smoothing_{true}; 
  
  rclcpp::Time corner_exit_start_time_;     
  bool in_corner_exit_transition_{false};   
  double last_curvature_{0.0};              
  
  std::vector<double> overtake_zone_starts_;
  std::vector<double> overtake_zone_ends_;
  
  CollisionState collision_state_{CollisionState::NORMAL};
  rclcpp::Time collision_state_start_time_;
  
  bool is_in_a1_zone_{false};
  double last_obstacle_left_dist_{10.0};
  double last_obstacle_right_dist_{10.0};
  double last_obstacle_front_dist_{10.0};
  double last_obstacle_angle_{0.0};
  double recovery_cooldown_duration_{0.3};
  
  DrivingMode driving_mode_{DrivingMode::CRUISE};
  bool is_overtaking_{false};
  bool overtake_committed_{false};           
  bool overtake_left_side_{true};            
  rclcpp::Time overtake_start_time_;
  double ego_s_position_{0.0};               
  std::vector<geometry_msgs::msg::Point> overtake_left_path_;
  std::vector<geometry_msgs::msg::Point> overtake_right_path_;
  
  mutable size_t last_closest_idx_{0};

  nmpc::BicycleMPCSolver solver_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCEngineNode>());
  rclcpp::shutdown();
  return 0;
}

