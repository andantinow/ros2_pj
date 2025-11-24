#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vehicle_model_msgs/msg/adaptive_vehicle_model.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <deque>
#include <cmath>
#include <cstdint>
#include <memory>
#include <algorithm>

class EstimatorNode : public rclcpp::Node {
public:
  EstimatorNode() : Node("estimator_node") {
    declare_parameter<double>("mu_init", 1.0);
    declare_parameter<double>("Cf_init", 15000.0);
    declare_parameter<double>("Cr_init", 15000.0);
    declare_parameter<double>("mu_alpha", 0.001);
    declare_parameter<double>("Cf_alpha", 0.001);
    declare_parameter<bool>("gate_param_update_on_straight", true);
    declare_parameter<double>("kappa_gate_threshold", 0.01);

    mu_ = get_parameter("mu_init").as_double();
    Cf_ = get_parameter("Cf_init").as_double();
    Cr_ = get_parameter("Cr_init").as_double();
    mu_nominal_ = mu_;
    Cf_nominal_ = Cf_;
    Cr_nominal_ = Cr_;
    mu_alpha_ = get_parameter("mu_alpha").as_double();
    Cf_alpha_ = get_parameter("Cf_alpha").as_double();
    gate_on_straight_ = get_parameter("gate_param_update_on_straight").as_bool();
    kappa_gate_th_ = get_parameter("kappa_gate_threshold").as_double();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/ego_racecar/odom", 20,
      std::bind(&EstimatorNode::odomCb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/sim/imu", 50,
      std::bind(&EstimatorNode::imuCb, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/perfect_odom", 10);
    model_pub_ = create_publisher<vehicle_model_msgs::msg::AdaptiveVehicleModel>("/adaptive_vehicle_model", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&EstimatorNode::update, this));
    RCLCPP_INFO(get_logger(), "EstimatorNode started");
  }
private:
  std::deque<nav_msgs::msg::Odometry> odom_buf_;
  std::deque<sensor_msgs::msg::Imu> imu_buf_;
  double mu_, Cf_, Cr_;
  double mu_nominal_{1.0};
  double Cf_nominal_{15000.0};
  double Cr_nominal_{15000.0};
  double mu_alpha_, Cf_alpha_;
  bool gate_on_straight_;
  double kappa_gate_th_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<vehicle_model_msgs::msg::AdaptiveVehicleModel>::SharedPtr model_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_buf_.push_back(*msg);
    if (odom_buf_.size() > 50) odom_buf_.pop_front();
  }
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_buf_.push_back(*msg);
    if (imu_buf_.size() > 200) imu_buf_.pop_front();
  }

  void update() {
    if (odom_buf_.empty() || imu_buf_.empty()) {
      RCLCPP_DEBUG(get_logger(), "No odom in buffer to publish /perfect_odom");
      return;
    }

    auto odom = odom_buf_.back();
    auto imu = imu_buf_.back();

    int64_t ns = this->now().nanoseconds();
    odom.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    odom.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    if (odom.header.frame_id.empty()) odom.header.frame_id = "odom";
    if (odom.child_frame_id.empty()) odom.child_frame_id = "base_link";
    odom_pub_->publish(odom);

    const double vx = odom.twist.twist.linear.x;
    const double vy = odom.twist.twist.linear.y;
    const double speed = std::hypot(vx, vy);
    const double yaw_rate = imu.angular_velocity.z;
    const double lat_acc = imu.linear_acceleration.y;
    const double kappa_est = (speed > 0.1) ? yaw_rate / speed : 0.0;

    bool allow_update = true;
    if (gate_on_straight_ && std::abs(kappa_est) < kappa_gate_th_) {
      allow_update = false;
    }

    if (allow_update) {
      const double mu_meas = std::clamp(std::abs(lat_acc) / 9.81, 0.05, 2.5);
      mu_ = (1.0 - mu_alpha_) * mu_ + mu_alpha_ * mu_meas;
      const double stiffness_scale = std::clamp(mu_ / std::max(0.1, mu_nominal_), 0.2, 2.0);
      const double target_cf = Cf_nominal_ * stiffness_scale;
      const double target_cr = Cr_nominal_ * stiffness_scale;
      Cf_ = (1.0 - Cf_alpha_) * Cf_ + Cf_alpha_ * target_cf;
      Cr_ = (1.0 - Cf_alpha_) * Cr_ + Cf_alpha_ * target_cr;
    }

    vehicle_model_msgs::msg::AdaptiveVehicleModel model_msg;
    model_msg.stamp = odom.header.stamp;
    model_msg.frame_id = "base_link";
    model_msg.mu = mu_;
    model_msg.cf = Cf_;
    model_msg.cr = Cr_;
    model_pub_->publish(model_msg);
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion &q){
    double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
    double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }
};

// Add main so the target links correctly
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
