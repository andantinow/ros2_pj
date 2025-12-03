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
  int max_iterations = 5;         // Maximum SQP iterations
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
    
    // Initialize control sequence with zeros
    u_prev_.resize(config_.prediction_steps);
    for (auto& u : u_prev_) {
      u.steering = 0.0;
      u.acceleration = 0.0;
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
      // Convert acceleration to speed command
      solution.speed = std::clamp(
        current_state.v + u[0].acceleration * config_.dt,
        config_.min_speed, config_.max_speed);
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
    const double learning_rate_steer = 0.1;
    const double learning_rate_accel = 0.05;
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

class NMPCEngineNode : public rclcpp::Node
{
public:
  NMPCEngineNode()
  : Node("nmpc_engine_node")
  {
    declare_parameter("prediction_horizon", 2.0);
    declare_parameter("prediction_steps", 20);
    declare_parameter("control_rate_hz", 50.0);
    declare_parameter("nominal_speed", 2.5);
    declare_parameter<std::string>("control_mode", "Pure Pursuit + PID");
    declare_parameter("solver_wheelbase", 0.33);
    // Topic parameters (configurable)
    declare_parameter<std::string>("odom_topic", "/car_state/odom_GT");
    declare_parameter<std::string>("path_topic", "/global_raceline");
    declare_parameter<std::string>("drive_topic", "/drive");

    prediction_horizon_ = get_parameter("prediction_horizon").as_double();
    prediction_steps_ = get_parameter("prediction_steps").as_int();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    nominal_speed_ = get_parameter("nominal_speed").as_double();
    control_mode_ = get_parameter("control_mode").as_string();
    solver_wheelbase_ = get_parameter("solver_wheelbase").as_double();
    
    std::string odom_topic = get_parameter("odom_topic").as_string();
    std::string path_topic = get_parameter("path_topic").as_string();
    std::string drive_topic = get_parameter("drive_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Initializing NMPC Engine...");
    RCLCPP_INFO(this->get_logger(), "Current Control Mode: %s", control_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "Topics - odom: %s, path: %s, drive: %s", 
                odom_topic.c_str(), path_topic.c_str(), drive_topic.c_str());

    AcadosConfig cfg{
      .horizon_sec = prediction_horizon_,
      .steps = static_cast<std::size_t>(std::max(1, prediction_steps_)),
      .wheelbase = solver_wheelbase_};
    solver_.initialize(cfg, this->get_logger());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(20),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(10).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = msg;
      });

    drive_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, rclcpp::QoS(10));

    const double period_s = 1.0 / std::max(1.0, control_rate_hz_);
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s));
    control_timer_ = create_wall_timer(period_ns, std::bind(&NMPCEngineNode::control_cycle, this));
  }

private:
  void control_cycle()
  {
    if (!latest_odom_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for odometry...");
      return;
    }
    if (!latest_path_ || latest_path_->poses.empty()) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for reference path...");
      return;
    }

    auto plan = build_reference_plan();
    auto warm_start = build_warm_start(plan);
    auto solution = solver_.solve(plan, warm_start, this->get_logger());
    publish_solution(solution);
  }

  ReferencePlan build_reference_plan() const
  {
    ReferencePlan plan;
    if (!latest_path_) {
      return plan;
    }

    const auto total_points = latest_path_->poses.size();
    if (total_points == 0) {
      return plan;
    }

    std::size_t steps = std::max<std::size_t>(1, static_cast<std::size_t>(prediction_steps_));
    std::size_t stride = std::max<std::size_t>(1, total_points / steps);

    for (std::size_t i = 0, idx = 0; i < steps && idx < total_points; ++i, idx += stride) {
      plan.poses.push_back(latest_path_->poses[idx].pose);
      plan.speeds.push_back(nominal_speed_);
    }

    return plan;
  }

  WarmStartSeed build_warm_start(const ReferencePlan & plan) const
  {
    WarmStartSeed seed;
    const std::size_t n = plan.poses.size();
    seed.steering_guess.assign(n, 0.0);
    seed.speed_guess = plan.speeds;
    return seed;
  }

  void publish_solution(const AcadosSolution & solution)
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd_msg;
    cmd_msg.header.stamp = this->now();
    cmd_msg.header.frame_id = latest_odom_ ? latest_odom_->child_frame_id : "base_link";

    if (solution.feasible) {
      cmd_msg.drive.speed = solution.speed;
      cmd_msg.drive.steering_angle = solution.steering;
    } else {
      cmd_msg.drive.speed = 0.0;
      cmd_msg.drive.steering_angle = 0.0;
    }

    drive_pub_->publish(cmd_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Path::SharedPtr latest_path_;

  double prediction_horizon_{2.0};
  int prediction_steps_{20};
  double control_rate_hz_{50.0};
  double nominal_speed_{2.5};
  double solver_wheelbase_{0.33};
  std::string control_mode_{"Pure Pursuit + PID"};

  AcadosSolverMock solver_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCEngineNode>());
  rclcpp::shutdown();
  return 0;
}

