#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // F1TENTH output standard
#include "nav_msgs/msg/odometry.hpp" // /sim/ego_racecar/odom subscription
#include "nav_msgs/msg/path.hpp"     
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h> 

using std::placeholders::_1;
using namespace std::chrono_literals;

class SimpleController : public rclcpp::Node
{
public:
    SimpleController() : Node("controller_node")
    {
        // 1. Subscriber (F1TENTH Odom)
        // TOPIC CHANGED: /localization/pose -> /sim/ego_racecar/odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sim/ego_racecar/odom", 10, std::bind(&SimpleController::pose_callback, this, _1));
        
        // 2. Subscriber (Path)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/path", 10, std::bind(&SimpleController::path_callback, this, _1));

        // 3. Publisher: Publishes the F1TENTH standard command
        // The topic will be remapped to /sim/drive in the launch file.
        cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/sim/drive", 10); // TOPIC CHANGED: /cmd_vel -> /sim/drive

        RCLCPP_INFO(this->get_logger(), "Controller Node started, publishing to /sim/drive (remapped).");
    }

private:
    nav_msgs::msg::Odometry current_pose_;
    nav_msgs::msg::Path current_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;

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
            // Stop robot if no path is received
            ackermann_msgs::msg::AckermannDriveStamped stop_msg;
            stop_msg.drive.speed = 0.0;
            cmd_pub_->publish(stop_msg);
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

        // Calculate Ackermann Command (P-Controller for steering)
        double Kp_angular = 1.0; 
        double steering_angle = Kp_angular * angle_to_target; 

        // Clamp steering angle to F1TENTH limits (e.g., -0.41 to 0.41 radians)
        if (steering_angle > 0.41) steering_angle = 0.41;
        if (steering_angle < -0.41) steering_angle = -0.41;

        // 5. Create and Publish Ackermann Message
        ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
        double desired_speed = 2.0; 
        
        ackermann_msg.drive.speed = desired_speed; 
        ackermann_msg.drive.steering_angle = steering_angle; 

        cmd_pub_->publish(ackermann_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing Ackermann: Speed=%.2f, Steering=%.2f",
            ackermann_msg.drive.speed, ackermann_msg.drive.steering_angle);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}
