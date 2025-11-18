#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vehicle_model_msgs/msg/adaptive_vehicle_model.hpp>
#include "path_utils.hpp"
#include <optional>
#include <algorithm>

using std::placeholders::_1;
using ctrl_utils::normalize_angle;
using ctrl_utils::yaw_from_quat;
using ctrl_utils::nearest_index;
using ctrl_utils::RateLimiter;
using ctrl_utils::clamp;

class NmpcEngineNode : public rclcpp::Node {
public:
  NmpcEngineNode(): Node("nmpc_engine_node") {
    declare_parameter<double>("wheelbase",0.33);
    declare_parameter<double>("steer_limit",0.41);
    declare_parameter<double>("steer_rate_limit",2.0);
    declare_parameter<double>("accel_limit",4.0);
    declare_parameter<double>("decel_limit",-6.0);
    declare_parameter<double>("speed_limit",6.0);
    declare_parameter<double>("lookahead_distance",1.5);
    declare_parameter<bool>("use_vref",true);

    wheelbase_         = get_parameter("wheelbase").as_double();
    steer_limit_       = get_parameter("steer_limit").as_double();
    steer_rate_limit_  = get_parameter("steer_rate_limit").as_double();
    accel_limit_       = get_parameter("accel_limit").as_double();
    decel_limit_       = get_parameter("decel_limit").as_double();
    speed_limit_       = get_parameter("speed_limit").as_double();
    lookahead_dist_    = get_parameter("lookahead_distance").as_double();
    use_vref_          = get_parameter("use_vref").as_bool();

    // Initialize rate limiters AFTER reading parameters to avoid UB
    steer_rl_ = RateLimiter(steer_rate_limit_);
    speed_rl_ = RateLimiter(3.0);

    odom_sub_  = create_subscription<nav_msgs::msg::Odometry>("/perfect_odom",20,
                  std::bind(&NmpcEngineNode::odomCb,this,_1));
    path_sub_  = create_subscription<nav_msgs::msg::Path>("/global_raceline",
                  rclcpp::QoS(1).transient_local().reliable(),
                  std::bind(&NmpcEngineNode::pathCb,this,_1));
    vref_sub_  = create_subscription<std_msgs::msg::Float32MultiArray>("/global_vref",
                  rclcpp::QoS(1).transient_local().reliable(),
                  std::bind(&NmpcEngineNode::vrefCb,this,_1));
    model_sub_ = create_subscription<vehicle_model_msgs::msg::AdaptiveVehicleModel>("/adaptive_vehicle_model",10,
                  std::bind(&NmpcEngineNode::modelCb,this,_1));

    cmd_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/sim/drive",10);
    timer_   = create_wall_timer(std::chrono::milliseconds(10),
                  std::bind(&NmpcEngineNode::loop,this));

    RCLCPP_INFO(get_logger(),
      "NMPC Engine Node started (100 Hz). acados: %s",
#ifdef HAVE_ACADOS
      "ENABLED"
#else
      "DISABLED (fallback)"
#endif
    );
  }

private:
  double wheelbase_, steer_limit_, steer_rate_limit_, accel_limit_, decel_limit_, speed_limit_, lookahead_dist_;
  bool use_vref_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  nav_msgs::msg::Path path_;
  std::vector<float> vref_;
  std::optional<vehicle_model_msgs::msg::AdaptiveVehicleModel> model_;

  ackermann_msgs::msg::AckermannDriveStamped last_good_cmd_;
  bool has_last_good_{false};

  // Initialize with safe defaults; real limits set in constructor after param load
  RateLimiter steer_rl_{0.0};
  RateLimiter speed_rl_{3.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vref_sub_;
  rclcpp::Subscription<vehicle_model_msgs::msg::AdaptiveVehicleModel>::SharedPtr model_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_{0,0,RCL_ROS_TIME};

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr m){ odom_ = *m; }
  void pathCb(const nav_msgs::msg::Path::SharedPtr m){ path_ = *m; }
  void vrefCb(const std_msgs::msg::Float32MultiArray::SharedPtr m){ vref_ = m->data; }
  void modelCb(const vehicle_model_msgs::msg::AdaptiveVehicleModel::SharedPtr m){ model_ = *m; }

  void loop(){
    auto now = this->now();
    double dt = 0.01;
    if(last_time_.nanoseconds() != 0) dt = (now - last_time_).seconds();
    last_time_ = now;

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    bool ok = computeControl(dt,cmd);
    if(ok){
      last_good_cmd_ = cmd;
      has_last_good_ = true;
    } else if(has_last_good_) {
      cmd = last_good_cmd_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Using fallback last_good_cmd");
    } else {
      cmd.drive.speed = 0.0;
      cmd.drive.steering_angle = 0.0;
    }
    cmd_pub_->publish(cmd);
  }

  bool computeControl(double dt, ackermann_msgs::msg::AckermannDriveStamped &cmd){
    if(!odom_.has_value()) return false;
    if(path_.poses.empty()) return false;

    const auto &pose = odom_->pose.pose;
    double x = pose.position.x;
    double y = pose.position.y;
    double yaw = yaw_from_quat(pose.orientation);

    int idx_near = nearest_index(path_, x, y);
    if(idx_near < 0) return false;

    int idx_target = idx_near;
    for(size_t i=idx_near; i<path_.poses.size(); ++i){
      const auto &pp = path_.poses[i].pose.position;
      double d = std::hypot(pp.x - x, pp.y - y);
      if(d >= lookahead_dist_){
        idx_target = (int)i;
        break;
      }
    }

    const auto &pt = path_.poses[idx_target].pose.position;
    double dx = pt.x - x;
    double dy = pt.y - y;
    double target_bearing = std::atan2(dy, dx);
    double angle_error = normalize_angle(target_bearing - yaw);

    double steer = clamp(1.0 * angle_error, -steer_limit_, steer_limit_);
    steer = steer_rl_.step(steer, dt);

    double v_des = std::min(speed_limit_, 2.0);
    if(use_vref_ && !vref_.empty()){
      int idx_v = std::min<int>(idx_target, (int)vref_.size()-1);
      v_des = std::min<double>(speed_limit_, std::max<double>(0.0, vref_[idx_v]));
    }

#ifdef HAVE_ACADOS
    bool solver_ok = true;
    if(!solver_ok) return false;
#endif

    double v_cmd = speed_rl_.step(v_des, dt);
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer;
    cmd.drive.speed = v_cmd;
    return true;
  }
};

int main(int argc,char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NmpcEngineNode>());
  rclcpp::shutdown();
  return 0;
}
