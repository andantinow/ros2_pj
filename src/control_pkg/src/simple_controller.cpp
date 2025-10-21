#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp" // Corrected Odometry
#include "nav_msgs/msg/path.hpp"     // Corrected Path
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h> 

using std::placeholders::_1;

class SimpleController : public rclcpp::Node
{
public:
    SimpleController() : Node("controller_node")
    {
        // 1. Subscribers (Odom and Path)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/pose", 10, std::bind(&SimpleController::pose_callback, this, _1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/path", 10, std::bind(&SimpleController::path_callback, this, _1));

        // 2. Publisher (Control Command)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Controller Node started, ready for control logic.");
    }

private:
    nav_msgs::msg::Odometry current_pose_;
    nav_msgs::msg::Path current_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Utility function to convert Quaternion to Yaw
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = *msg;
        if (!current_path_.poses.empty()) {
            control_callback(); 
        }
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        current_path_ = *msg;
        if (!current_path_.poses.empty()) {
            control_callback();
        }
    }

    void control_callback()
    {
        if (current_path_.poses.empty()) {
            return;
        }

        // --- [Core Control Logic: Simplified Pure Pursuit] ---
        double x = current_pose_.pose.pose.position.x;
        double y = current_pose_.pose.pose.position.y;
        double current_yaw = get_yaw_from_quaternion(current_pose_.pose.pose.orientation);

        // Target Waypoint (First point in path)
        double target_x = current_path_.poses[0].pose.position.x;
        double target_y = current_path_.poses[0].pose.position.y;

        // Calculate Angular Error
        double dx = target_x - x;
        double dy = target_y - y;
        double absolute_angle = std::atan2(dy, dx);
        double angle_to_target = absolute_angle - current_yaw;

        // Angle normalization (-PI ~ PI)
        if (angle_to_target > M_PI) {
            angle_to_target -= 2 * M_PI;
        } else if (angle_to_target < -M_PI) {
            angle_to_target += 2 * M_PI;
        }

        // Calculate Twist Command
        double Kp_angular = 1.0; 
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 2.0; 
        twist_msg.angular.z = Kp_angular * angle_to_target; 

        cmd_vel_pub_->publish(twist_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}
