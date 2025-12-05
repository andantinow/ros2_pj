/**
 * @file nmpc_lifecycle_node.cpp
 * @brief LifecycleNode-based NMPC Controller with acados integration
 * 
 * This is a refactored NMPC controller that addresses the architectural
 * criticisms from the problem statement:
 * 
 * 1. LifecycleNode Pattern (Issue 2.1):
 *    - on_configure: Initialize acados solver, allocate memory
 *    - on_activate: Start control loop
 *    - on_deactivate: Stop control, hold last command
 *    - on_cleanup: Release solver resources
 * 
 * 2. Threading/Executor (Issue 2.2):
 *    - Uses MultiThreadedExecutor-compatible callback groups
 *    - NMPC solver runs in MutuallyExclusive callback group
 *    - Sensor callbacks in Reentrant callback group
 *    - Control timer separated from subscription callbacks
 * 
 * 3. acados Integration (Issue 1.x):
 *    - Uses IAcadosNMPCSolver interface
 *    - Fallback to hand-written solver if acados unavailable
 *    - RTI scheme for predictable timing
 *    - HPIPM QP solver (not MUMPS)
 * 
 * 4. Latency Compensation (Issue 3.3):
 *    - State prediction for computation delay
 *    - RTI preparation/feedback split possible
 * 
 * @note This node should be used with MultiThreadedExecutor:
 *       auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
 *       executor->add_node(node->get_node_base_interface());
 */

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "control_pkg/acados_nmpc_interface.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace nmpc_lifecycle
{

/**
 * @brief LifecycleNode-based NMPC Controller
 * 
 * Lifecycle states:
 * - Unconfigured: Node created but not configured
 * - Inactive: Configured (solver ready) but not running
 * - Active: Control loop running
 * - Finalized: Node shutdown
 * 
 * Transition callbacks:
 * - on_configure: Setup solver, parameters, subscribers/publishers
 * - on_activate: Start control timer
 * - on_deactivate: Stop control timer, hold last command
 * - on_cleanup: Release solver resources
 * - on_shutdown: Emergency cleanup
 */
class NMPCLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NMPCLifecycleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("nmpc_lifecycle_node", options)
  {
    RCLCPP_INFO(get_logger(), "NMPC Lifecycle Node created (Unconfigured)");
    
    // Declare parameters here (before configure)
    declareParameters();
  }
  
  ~NMPCLifecycleNode() override
  {
    RCLCPP_INFO(get_logger(), "NMPC Lifecycle Node destroyed");
  }

  /**
   * @brief Lifecycle callback: Configure
   * 
   * This is where acados solver initialization should happen:
   * - Load generated code
   * - Allocate workspace memory
   * - Set default parameters
   * 
   * This matches acados usage pattern where initialization is separated
   * from the real-time control loop.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& /* state */) override
  {
    RCLCPP_INFO(get_logger(), "Configuring NMPC node...");
    
    try {
      // Create callback groups for multi-threaded execution
      createCallbackGroups();
      
      // Load parameters
      loadParameters();
      
      // Initialize solver
      if (!initializeSolver()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize NMPC solver");
        return CallbackReturn::FAILURE;
      }
      
      // Create subscribers (using reentrant callback group)
      createSubscribers();
      
      // Create publishers
      createPublishers();
      
      RCLCPP_INFO(get_logger(), "Configuration complete. Ready to activate.");
      return CallbackReturn::SUCCESS;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }
  
  /**
   * @brief Lifecycle callback: Activate
   * 
   * Start the control timer. At this point:
   * - Solver is initialized
   * - Subscribers are ready
   * - Publishers are activated
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& /* state */) override
  {
    RCLCPP_INFO(get_logger(), "Activating NMPC node...");
    
    try {
      // Reset solver state for fresh start
      solver_->reset();
      
      // Create control timer with mutually exclusive callback group
      // This ensures NMPC solve doesn't run concurrently with itself
      const double period_s = 1.0 / std::max(1.0, control_rate_hz_);
      control_timer_ = create_wall_timer(
        std::chrono::duration<double>(period_s),
        std::bind(&NMPCLifecycleNode::controlCycle, this),
        solver_callback_group_);
      
      is_active_.store(true);
      
      RCLCPP_INFO(get_logger(), "NMPC active at %.1f Hz", control_rate_hz_);
      return CallbackReturn::SUCCESS;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Activation failed: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }
  
  /**
   * @brief Lifecycle callback: Deactivate
   * 
   * Stop control loop but keep solver ready.
   * Publish zero command to stop vehicle.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /* state */) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating NMPC node...");
    
    is_active_.store(false);
    
    // Cancel timer
    if (control_timer_) {
      control_timer_->cancel();
      control_timer_.reset();
    }
    
    // Publish stop command
    publishStopCommand();
    
    RCLCPP_INFO(get_logger(), "NMPC deactivated");
    return CallbackReturn::SUCCESS;
  }
  
  /**
   * @brief Lifecycle callback: Cleanup
   * 
   * Release solver resources. After this, node returns to unconfigured state.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /* state */) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up NMPC node...");
    
    // Cancel timer if still running
    if (control_timer_) {
      control_timer_->cancel();
      control_timer_.reset();
    }
    
    // Cleanup solver (release acados workspace)
    if (solver_) {
      solver_->cleanup();
      solver_.reset();
    }
    
    // Reset subscribers/publishers
    odom_sub_.reset();
    path_sub_.reset();
    scan_sub_.reset();
    drive_pub_.reset();
    traj_pub_.reset();
    ref_pub_.reset();
    
    RCLCPP_INFO(get_logger(), "Cleanup complete");
    return CallbackReturn::SUCCESS;
  }
  
  /**
   * @brief Lifecycle callback: Shutdown
   * 
   * Emergency cleanup - called from any state.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override
  {
    RCLCPP_WARN(get_logger(), "Shutting down from state: %s", state.label().c_str());
    
    is_active_.store(false);
    
    // Ensure stop command is published
    publishStopCommand();
    
    // Cleanup
    if (control_timer_) {
      control_timer_->cancel();
      control_timer_.reset();
    }
    
    if (solver_) {
      solver_->cleanup();
      solver_.reset();
    }
    
    return CallbackReturn::SUCCESS;
  }

private:
  /**
   * @brief Declare all ROS 2 parameters
   */
  void declareParameters()
  {
    // Solver parameters
    declare_parameter("prediction_horizon", 1.5);
    declare_parameter("prediction_steps", 15);
    declare_parameter("control_rate_hz", 50.0);
    declare_parameter("nominal_speed", 2.5);
    declare_parameter("solver_wheelbase", 0.33);
    
    // MPC weights
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
    
    // Solver settings
    declare_parameter("levenberg_marquardt", 0.01);
    declare_parameter("max_solver_iterations", 20);
    declare_parameter("use_acados", true);  // Try acados first
    declare_parameter("use_rti", true);
    
    // Constraints
    declare_parameter("max_steer", 0.436);
    declare_parameter("max_steer_rate", 1.8);
    declare_parameter("max_speed", 6.0);
    declare_parameter("max_accel", 4.0);
    declare_parameter("min_accel", -6.0);
    
    // Topics
    declare_parameter<std::string>("odom_topic", "/dual_ekf/global_odom");
    declare_parameter<std::string>("path_topic", "/global_raceline");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("scan_topic", "/scan");
    
    // Collision avoidance
    declare_parameter("enable_collision_avoidance", true);
    declare_parameter("a1_threshold", 0.3);
    declare_parameter("a2_threshold", 0.8);
  }
  
  /**
   * @brief Create callback groups for multi-threaded execution
   * 
   * Issue 2.2 Fix: Proper callback group separation
   * - Sensor callbacks: Reentrant (can run in parallel)
   * - Solver/Control: MutuallyExclusive (one at a time)
   */
  void createCallbackGroups()
  {
    // Reentrant group for sensor callbacks - can run concurrently
    sensor_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    
    // MutuallyExclusive group for solver - prevents concurrent solve
    solver_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    
    RCLCPP_DEBUG(get_logger(), "Created callback groups for multi-threaded execution");
  }
  
  /**
   * @brief Load parameters from ROS 2 parameter server
   */
  void loadParameters()
  {
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    nominal_speed_ = get_parameter("nominal_speed").as_double();
    max_steer_ = get_parameter("max_steer").as_double();
    prediction_horizon_ = get_parameter("prediction_horizon").as_double();
    
    odom_topic_ = get_parameter("odom_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    drive_topic_ = get_parameter("drive_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    
    enable_collision_avoidance_ = get_parameter("enable_collision_avoidance").as_bool();
    a1_threshold_ = get_parameter("a1_threshold").as_double();
    a2_threshold_ = get_parameter("a2_threshold").as_double();
    
    RCLCPP_INFO(get_logger(), "Parameters loaded: rate=%.1f Hz, speed=%.1f m/s",
                control_rate_hz_, nominal_speed_);
  }
  
  /**
   * @brief Initialize NMPC solver
   * 
   * Issue 1.x Fix: acados integration
   * - Use factory to create appropriate solver
   * - Configure with all parameters
   * - Solver workspace allocated here (not in control loop)
   */
  bool initializeSolver()
  {
    // Build configuration from parameters
    acados_interface::NMPCConfig config;
    config.prediction_horizon_sec = get_parameter("prediction_horizon").as_double();
    config.N = get_parameter("prediction_steps").as_int();
    config.dt = config.prediction_horizon_sec / std::max(1, config.N);
    config.wheelbase = get_parameter("solver_wheelbase").as_double();
    
    // Weights
    config.w_x = 1.0;
    config.w_y = get_parameter("w_pos").as_double();
    config.w_yaw = get_parameter("w_yaw").as_double();
    config.w_v = get_parameter("w_vel").as_double();
    config.w_steering = get_parameter("w_steer").as_double();
    config.w_acceleration = get_parameter("w_accel").as_double();
    config.w_steering_rate = get_parameter("w_steer_rate").as_double();
    config.w_acceleration_rate = get_parameter("w_accel_rate").as_double();
    config.w_terminal = get_parameter("w_terminal").as_double();
    config.lateral_tolerance = get_parameter("lateral_tolerance").as_double();
    config.latency_compensation_sec = get_parameter("latency_compensation_sec").as_double();
    
    // Constraints
    config.max_steering = get_parameter("max_steer").as_double();
    config.max_steering_rate = get_parameter("max_steer_rate").as_double();
    config.max_speed = get_parameter("max_speed").as_double();
    config.max_acceleration = get_parameter("max_accel").as_double();
    config.min_acceleration = get_parameter("min_accel").as_double();
    
    // Solver settings
    config.max_sqp_iterations = get_parameter("max_solver_iterations").as_int();
    config.use_rti = get_parameter("use_rti").as_bool();
    
    // Create solver using factory (will use acados if available)
    bool prefer_acados = get_parameter("use_acados").as_bool();
    solver_ = acados_interface::createNMPCSolver(prefer_acados);
    
    if (!solver_) {
      RCLCPP_ERROR(get_logger(), "Failed to create solver");
      return false;
    }
    
    // Initialize solver
    if (!solver_->initialize(config)) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize solver");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "NMPC solver initialized:");
    RCLCPP_INFO(get_logger(), "  Horizon: %.2fs, N=%d, dt=%.3fs",
                config.prediction_horizon_sec, config.N, config.dt);
    RCLCPP_INFO(get_logger(), "  Steering rate weight: %.1f (oscillation suppression)",
                config.w_steering_rate);
    RCLCPP_INFO(get_logger(), "  Lateral tolerance: %.2fm", config.lateral_tolerance);
    RCLCPP_INFO(get_logger(), "  RTI mode: %s", config.use_rti ? "enabled" : "disabled");
    
    return true;
  }
  
  /**
   * @brief Create subscribers with appropriate callback groups
   */
  void createSubscribers()
  {
    // Subscription options for sensor callback group
    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group = sensor_callback_group_;
    
    // Odometry subscription
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_odom_ = msg;
      },
      sensor_options);
    
    // Path subscription (transient local for latched behavior)
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        latest_path_ = msg;
        if (msg && !msg->poses.empty()) {
          RCLCPP_INFO_ONCE(get_logger(), "Received path with %zu points", msg->poses.size());
        }
      },
      sensor_options);
    
    // LiDAR subscription (for collision avoidance)
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = msg;
      },
      sensor_options);
    
    RCLCPP_DEBUG(get_logger(), "Subscribers created");
  }
  
  /**
   * @brief Create publishers
   */
  void createPublishers()
  {
    // Drive command publisher
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_, rclcpp::QoS(10));
    
    // Visualization publishers
    traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nmpc_predicted_trajectory", rclcpp::QoS(1));
    ref_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nmpc_reference_points", rclcpp::QoS(1));
    
    RCLCPP_DEBUG(get_logger(), "Publishers created");
  }
  
  /**
   * @brief Main control cycle
   * 
   * This runs in the solver callback group (mutually exclusive)
   * to prevent concurrent solve operations.
   */
  void controlCycle()
  {
    if (!is_active_.load()) {
      return;
    }
    
    // Get current state (thread-safe)
    nav_msgs::msg::Odometry::SharedPtr odom;
    nav_msgs::msg::Path::SharedPtr path;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom = latest_odom_;
    }
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      path = latest_path_;
    }
    
    // Check prerequisites
    if (!odom) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, 
        "Waiting for odometry...");
      return;
    }
    if (!path || path->poses.empty()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for path...");
      return;
    }
    
    // Check collision avoidance (A1/A2 zones)
    if (checkCollisionAvoidance()) {
      return;  // Collision avoidance took over control
    }
    
    // Extract vehicle state
    acados_interface::VehicleState current_state = extractState(odom);
    
    // Build reference trajectory
    auto reference = buildReference(current_state, path);
    if (reference.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Empty reference trajectory");
      return;
    }
    
    // Set solver inputs
    solver_->setInitialState(current_state);
    solver_->setReference(reference);
    solver_->setPreviousControl(last_control_);
    
    // Solve NMPC
    auto solution = solver_->solve();
    
    // Log solver status
    if (solution.stats.status != acados_interface::SolverStatus::SUCCESS) {
      const char* status_str = "UNKNOWN";
      switch (solution.stats.status) {
        case acados_interface::SolverStatus::MAX_SQP_ITER: status_str = "MAX_ITER"; break;
        case acados_interface::SolverStatus::QP_FAILURE: status_str = "QP_FAILURE"; break;
        case acados_interface::SolverStatus::NAN_SOLUTION: status_str = "NAN"; break;
        case acados_interface::SolverStatus::TIMEOUT: status_str = "TIMEOUT"; break;
        default: break;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "NMPC: %s after %d iters, time=%.2fms",
        status_str, solution.stats.sqp_iterations, solution.stats.total_time_ms);
    }
    
    // Update last control
    last_control_ = solution.control;
    
    // Publish command
    publishDriveCommand(solution, current_state);
    
    // Publish visualization
    publishVisualization(solution.predicted_trajectory, reference);
    
    // Debug log
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "NMPC: (%.2f,%.2f,%.1f°,%.2fm/s) -> steer=%.3f, time=%.1fms",
      current_state.x, current_state.y, 
      current_state.yaw * 180.0 / M_PI, current_state.v,
      solution.control.steering, solution.stats.total_time_ms);
  }
  
  /**
   * @brief Check A1/A2 collision avoidance zones
   * @return true if collision avoidance took control
   */
  bool checkCollisionAvoidance()
  {
    if (!enable_collision_avoidance_) {
      return false;
    }
    
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan = latest_scan_;
    }
    
    if (!scan) {
      return false;
    }
    
    // Simplified A1/A2 check (full implementation would be more detailed)
    double min_front = 10.0;
    double angle_min = scan->angle_min;
    double angle_inc = scan->angle_increment;
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      double angle = angle_min + i * angle_inc;
      double range = scan->ranges[i];
      
      if (std::isnan(range) || std::isinf(range) || range < 0.03) {
        continue;
      }
      
      // Front sector (±60°)
      if (std::abs(angle) < M_PI / 3.0) {
        min_front = std::min(min_front, range);
      }
    }
    
    // A1 zone: reverse
    if (min_front < a1_threshold_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "A1 Zone! Front=%.2fm, reversing", min_front);
      
      ackermann_msgs::msg::AckermannDriveStamped cmd;
      cmd.header.stamp = now();
      cmd.header.frame_id = "base_link";
      cmd.drive.speed = -0.5;
      cmd.drive.steering_angle = 0.0;
      
      if (drive_pub_->is_activated()) {
        drive_pub_->publish(cmd);
      }
      return true;
    }
    
    return false;
  }
  
  /**
   * @brief Extract vehicle state from odometry
   */
  acados_interface::VehicleState extractState(
    const nav_msgs::msg::Odometry::SharedPtr& odom) const
  {
    acados_interface::VehicleState state;
    state.x = odom->pose.pose.position.x;
    state.y = odom->pose.pose.position.y;
    
    // Extract yaw from quaternion
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    state.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 
                          1.0 - 2.0 * (qy * qy + qz * qz));
    
    state.v = std::sqrt(
      odom->twist.twist.linear.x * odom->twist.twist.linear.x +
      odom->twist.twist.linear.y * odom->twist.twist.linear.y);
    
    return state;
  }
  
  /**
   * @brief Build reference trajectory for NMPC
   */
  std::vector<acados_interface::ReferencePoint> buildReference(
    const acados_interface::VehicleState& current_state,
    const nav_msgs::msg::Path::SharedPtr& path) const
  {
    std::vector<acados_interface::ReferencePoint> reference;
    
    if (!path || path->poses.empty()) {
      return reference;
    }
    
    const auto& poses = path->poses;
    const size_t num_poses = poses.size();
    constexpr int NUM_REF_POINTS = 15;
    
    // Find closest point ahead
    double cos_yaw = std::cos(current_state.yaw);
    double sin_yaw = std::sin(current_state.yaw);
    
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < num_poses; ++i) {
      double dx = poses[i].pose.position.x - current_state.x;
      double dy = poses[i].pose.position.y - current_state.y;
      double dist = dx * dx + dy * dy;
      double ahead = dx * cos_yaw + dy * sin_yaw;
      
      // Prefer points ahead
      double weighted = (ahead < 0) ? dist * 3.0 : dist;
      if (weighted < min_dist) {
        min_dist = weighted;
        closest_idx = i;
      }
    }
    
    // Calculate stride for horizon coverage
    double speed = std::max(0.5, current_state.v);
    double lookahead_dist = speed * prediction_horizon_;
    
    // Estimate path spacing
    double path_spacing = 0.1;  // Default estimate
    if (num_poses > 1) {
      double dx = poses[1].pose.position.x - poses[0].pose.position.x;
      double dy = poses[1].pose.position.y - poses[0].pose.position.y;
      path_spacing = std::max(0.05, std::sqrt(dx * dx + dy * dy));
    }
    
    size_t stride = std::max<size_t>(1, 
      static_cast<size_t>(lookahead_dist / NUM_REF_POINTS / path_spacing));
    
    // Build reference
    for (int i = 0; i < NUM_REF_POINTS; ++i) {
      size_t idx = (closest_idx + i * stride) % num_poses;
      
      acados_interface::ReferencePoint ref;
      ref.x = poses[idx].pose.position.x;
      ref.y = poses[idx].pose.position.y;
      
      double qz = poses[idx].pose.orientation.z;
      double qw = poses[idx].pose.orientation.w;
      ref.yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
      ref.v = nominal_speed_;
      
      reference.push_back(ref);
    }
    
    return reference;
  }
  
  /**
   * @brief Publish drive command
   */
  void publishDriveCommand(
    const acados_interface::NMPCSolution& solution,
    const acados_interface::VehicleState& current_state)
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    
    if (solution.feasible) {
      cmd.drive.steering_angle = solution.control.steering;
      
      // Compute speed from acceleration
      double dt = solver_->getConfig().dt;
      double computed_speed = current_state.v + solution.control.acceleration * dt;
      
      // Minimum speed to get moving
      if (current_state.v < 0.3 && computed_speed < 0.5) {
        computed_speed = 0.5;
      }
      
      cmd.drive.speed = std::clamp(computed_speed, 
        solver_->getConfig().min_speed, solver_->getConfig().max_speed);
    } else {
      // Not feasible - stop
      cmd.drive.steering_angle = 0.0;
      cmd.drive.speed = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "NMPC solution not feasible, stopping");
    }
    
    if (drive_pub_->is_activated()) {
      drive_pub_->publish(cmd);
    }
  }
  
  /**
   * @brief Publish stop command
   */
  void publishStopCommand()
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    
    if (drive_pub_ && drive_pub_->is_activated()) {
      drive_pub_->publish(cmd);
      RCLCPP_INFO(get_logger(), "Stop command published");
    }
  }
  
  /**
   * @brief Publish visualization markers
   */
  void publishVisualization(
    const std::vector<acados_interface::VehicleState>& trajectory,
    const std::vector<acados_interface::ReferencePoint>& reference)
  {
    auto current_time = now();
    
    // Predicted trajectory (green line)
    if (!trajectory.empty() && traj_pub_->is_activated()) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = current_time;
      marker.ns = "nmpc_prediction";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.08;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.pose.orientation.w = 1.0;
      
      for (const auto& state : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = state.x;
        p.y = state.y;
        p.z = 0.1;
        marker.points.push_back(p);
      }
      
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      traj_pub_->publish(marker);
    }
    
    // Reference points (blue spheres)
    if (!reference.empty() && ref_pub_->is_activated()) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = current_time;
      marker.ns = "nmpc_reference";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.7f;
      marker.pose.orientation.w = 1.0;
      
      for (const auto& ref : reference) {
        geometry_msgs::msg::Point p;
        p.x = ref.x;
        p.y = ref.y;
        p.z = 0.05;
        marker.points.push_back(p);
      }
      
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      ref_pub_->publish(marker);
    }
  }

  // ========== Member Variables ==========
  
  // Callback groups for multi-threaded execution
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr solver_callback_group_;
  
  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  
  // Publishers (lifecycle publishers)
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr ref_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Solver
  std::unique_ptr<acados_interface::IAcadosNMPCSolver> solver_;
  
  // Thread-safe state storage
  std::mutex state_mutex_;
  std::mutex path_mutex_;
  std::mutex scan_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Path::SharedPtr latest_path_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  
  // Control state
  acados_interface::ControlInput last_control_;
  std::atomic<bool> is_active_{false};
  
  // Parameters
  double control_rate_hz_{50.0};
  double nominal_speed_{2.5};
  double max_steer_{0.436};
  double prediction_horizon_{1.5};
  std::string odom_topic_;
  std::string path_topic_;
  std::string drive_topic_;
  std::string scan_topic_;
  bool enable_collision_avoidance_{true};
  double a1_threshold_{0.3};
  double a2_threshold_{0.8};
};

}  // namespace nmpc_lifecycle

/**
 * @brief Main entry point
 * 
 * Uses MultiThreadedExecutor for proper callback group handling.
 * This allows sensor callbacks to run in parallel with each other,
 * while the NMPC solver runs exclusively (no concurrent solve).
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create node
  auto node = std::make_shared<nmpc_lifecycle::NMPCLifecycleNode>();
  
  // Use MultiThreadedExecutor for callback group support
  // Issue 2.2 Fix: Proper executor for real-time performance
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  
  RCLCPP_INFO(node->get_logger(), 
    "NMPC Lifecycle Node started with MultiThreadedExecutor");
  RCLCPP_INFO(node->get_logger(),
    "Use lifecycle transitions to configure and activate:");
  RCLCPP_INFO(node->get_logger(),
    "  ros2 lifecycle set /nmpc_lifecycle_node configure");
  RCLCPP_INFO(node->get_logger(),
    "  ros2 lifecycle set /nmpc_lifecycle_node activate");
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
