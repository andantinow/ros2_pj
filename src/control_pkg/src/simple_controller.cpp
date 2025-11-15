#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

class SimpleController : public rclcpp::Node {
public:
    SimpleController() : Node("simple_controller") {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/car_state/odom", 10,
            std::bind(&SimpleController::odom_callback, this, std::placeholders::_1));
        
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
        
        RCLCPP_INFO(this->get_logger(), "Controller initialized");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double yaw = 2.0 * std::atan2(qz, qw);
        
        double speed = std::sqrt(vx * vx + vy * vy);
        
        double steering_angle = 0.0;
        double target_speed = 1.0;
        
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = target_speed;
        
        drive_pub_->publish(drive_msg);
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}

