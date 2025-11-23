#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vehicle_model_msgs/msg/adaptive_vehicle_model.hpp>
#include <deque>
#include <cmath>
#include <string>

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
  double mu_alpha_, Cf_alpha_;
  bool gate_on_straight_;
  double kappa_gate_th_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<vehicle_model_msgs::msg::AdaptiveVehicleModel>::SharedPtr model_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // keep incoming odom in buffer
    odom_buf_.push_back(*msg);
    if (odom_buf_.size() > 50) odom_buf_.pop_front();
  }
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_buf_.push_back(*msg);
    if (imu_buf_.size() > 200) imu_buf_.pop_front();
  }

  void update() {
    // publish latest odom as perfect_odom (ensure header.stamp is valid)
    if (!odom_buf_.empty()) {
      auto out = odom_buf_.back();
      // IMPORTANT: set a valid timestamp here so TF/RViz/tf2 see a valid time
      out.header.stamp = this->now().to_msg();
      // Optionally set frame_id and child_frame_id consistently if needed
      if (out.header.frame_id.empty()) out.header.frame_id = "odom";
      if (out.child_frame_id.empty()) out.child_frame_id = "base_link";

      odom_pub_->publish(out);
      RCLCPP_DEBUG(get_logger(), "Published /perfect_odom (stamp %u.%u)", out.header.stamp.sec, out.header.stamp.nanosec);
    } else {
      RCLCPP_DEBUG(get_logger(), "No odom in buffer to publish /perfect_odom");
    }

    // (여기에 기존 estimator 계산 로직이 들어갈 수 있음)
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion &q){
    double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
    double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }
};
