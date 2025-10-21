#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp" // Changed to Odometry (Capital 'O')
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher() : Node("pose_publisher"), theta_(0.0)
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/pose", 10);
        timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&PosePublisher::publish_pose, this));
        RCLCPP_INFO(this->get_logger(), "Localization Publisher Node started, publishing to /localization/pose.");
    }

private:
    void publish_pose()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "base_link";
        
        double radius = 1.0;
        double speed = 0.5; // rad/s
        theta_ += speed * 0.1; 

        odom_msg.pose.pose.position.x = 5.0 + radius * std::cos(theta_);
        odom_msg.pose.pose.position.y = radius * std::sin(theta_);

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_ + M_PI/2.0); 
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), "Publishing Pose (X:%.2f, Y:%.2f)", 
                    odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
        publisher_->publish(odom_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    double theta_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisher>());
    rclcpp::shutdown();
    return 0;
}
