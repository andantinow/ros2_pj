#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vehicle_model_msgs/msg/adaptive_vehicle_model.hpp>
#include <deque>
#include <cmath>

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
    odom_buf_.push_back(*msg);
    if (odom_buf_.size() > 50) odom_buf_.pop_front();
  }
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_buf_.push_back(*msg);
    if (imu_buf_.size() > 200) imu_buf_.pop_front();
  }
  static double yawFromQuat(const geometry_msgs::msg::Quaternion &q){
    double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
    double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }
  static double normalizeAngle(double a){ while(a>M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; }

  void update() {
    if (odom_buf_.empty()) return;
    nav_msgs::msg::Odometry fused = odom_buf_.back();
    static bool has_prev=false; static double vx_prev=0.0, vy_prev=0.0;
    const double alpha_v=0.2;
    double vx=fused.twist.twist.linear.x;
    double vy=fused.twist.twist.linear.y;
    if (has_prev) {
      vx = alpha_v*vx + (1-alpha_v)*vx_prev;
      vy = alpha_v*vy + (1-alpha_v)*vy_prev;
    }
    fused.twist.twist.linear.x = vx;
    fused.twist.twist.linear.y = vy;
    vx_prev=vx; vy_prev=vy; has_prev=true;

    double speed = std::hypot(vx, vy);
    double kappa_est=0.0;
    if (speed > 0.1 && odom_buf_.size()>=2) {
      const auto &prev = odom_buf_[odom_buf_.size()-2];
      const auto &p1 = prev.pose.pose.position;
      const auto &p2 = fused.pose.pose.position;
      double dx=p2.x-p1.x, dy=p2.y-p1.y;
      double yaw1=yawFromQuat(prev.pose.pose.orientation);
      double yaw2=yawFromQuat(fused.pose.pose.orientation);
      double dpsi=normalizeAngle(yaw2 - yaw1);
      double ds=std::hypot(dx,dy) + 1e-6;
      kappa_est = dpsi/ds;
    }

    double ay = speed*speed * std::abs(kappa_est);
    const double g=9.80665;
    double mu_obs = std::min(2.0, ay / g);

    bool gate = gate_on_straight_ && (std::abs(kappa_est) < kappa_gate_th_);
    if (!gate){
      mu_ = (1.0 - mu_alpha_)*mu_ + mu_alpha_*mu_obs;
      double Cf_obs = 15000.0 + 2000.0 * std::min(1.0, ay/(0.8*g));
      Cf_ = (1.0 - Cf_alpha_)*Cf_ + Cf_alpha_*Cf_obs;
    }

    odom_pub_->publish(fused);

    vehicle_model_msgs::msg::AdaptiveVehicleModel model;
    model.stamp = now();
    model.frame_id = "base_link";
    model.mu = mu_;
    model.cf = Cf_;
    model.cr = Cr_;
    model_pub_->publish(model);
  }
};

int main(int argc,char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
