
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct AcadosConfig
{
  double horizon_sec;
  std::size_t steps;
  double wheelbase;
};

struct ReferencePlan
{
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> speeds;
};

struct WarmStartSeed
{
  std::vector<double> steering_guess;
  std::vector<double> speed_guess;
};

struct AcadosSolution
{
  double steering = 0.0;
  double speed = 0.0;
  bool feasible = false;
};

class AcadosSolverMock
{
public:
  bool initialize(const AcadosConfig & config, const rclcpp::Logger & logger)
  {
    config_ = config;
    RCLCPP_INFO(
      logger, "Acados mock configured | horizon: %.2fs, steps: %zu, wheelbase: %.2f",
      config_.horizon_sec, config_.steps, config_.wheelbase);
    return true;
  }

  AcadosSolution solve(
    const ReferencePlan & plan, const WarmStartSeed & seed, const rclcpp::Logger & logger) const
  {
    if (plan.poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        logger, *rclcpp::Clock::make_shared(), 2000,
        "NMPC reference plan empty. Skipping solve step.");
      return {};
    }

    AcadosSolution solution;
    solution.feasible = true;
    solution.speed = plan.speeds.empty() ? 0.0 : plan.speeds.front();
    solution.steering = seed.steering_guess.empty() ? 0.0 : seed.steering_guess.front();
    return solution;
  }

private:
  AcadosConfig config_;
};
}  // namespace

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

    prediction_horizon_ = get_parameter("prediction_horizon").as_double();
    prediction_steps_ = get_parameter("prediction_steps").as_int();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    nominal_speed_ = get_parameter("nominal_speed").as_double();
    control_mode_ = get_parameter("control_mode").as_string();
    solver_wheelbase_ = get_parameter("solver_wheelbase").as_double();

    RCLCPP_INFO(this->get_logger(), "Initializing NMPC Engine...");
    RCLCPP_INFO(this->get_logger(), "Current Control Mode: %s", control_mode_.c_str());

    AcadosConfig cfg{
      .horizon_sec = prediction_horizon_,
      .steps = static_cast<std::size_t>(std::max(1, prediction_steps_)),
      .wheelbase = solver_wheelbase_};
    solver_.initialize(cfg, this->get_logger());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(20),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_raceline", rclcpp::QoS(10).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = msg;
      });

    drive_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", rclcpp::QoS(10));

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

