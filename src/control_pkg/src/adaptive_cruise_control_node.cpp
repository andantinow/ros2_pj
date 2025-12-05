/**
 * @file adaptive_cruise_control_node.cpp
 * @brief Adaptive Cruise Control (ACC) with velocity estimation for F1TENTH
 * 
 * This node implements time-headway based adaptive cruise control that synchronizes
 * with the leading vehicle's velocity. It uses LiDAR clustering to detect the lead
 * vehicle and estimates relative velocity through distance differentiation.
 * 
 * Control Law:
 *   d_desired = d_min + tau_gap * v_ego
 *   v_cmd = v_ego + K_p * (d - d_desired) + K_d * v_rel
 * 
 * Where:
 *   - d_desired: Target following distance (increases with speed)
 *   - tau_gap: Time headway constant (seconds)
 *   - v_rel: Estimated relative velocity (negative = closing)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

namespace control_pkg
{

class AdaptiveCruiseControlNode : public rclcpp::Node
{
public:
    AdaptiveCruiseControlNode()
    : Node("adaptive_cruise_control_node")
    {
        // Parameter declarations
        this->declare_parameter<double>("min_safety_dist", 0.8);
        this->declare_parameter<double>("time_headway", 0.4);
        this->declare_parameter<double>("kp_dist", 1.2);
        this->declare_parameter<double>("kd_vel", 0.1);
        this->declare_parameter<double>("max_speed", 6.0);
        this->declare_parameter<double>("min_speed", 0.0);
        this->declare_parameter<int>("scan_window_deg", 20);
        this->declare_parameter<double>("free_range_threshold", 6.0);
        this->declare_parameter<double>("min_valid_range", 0.1);
        this->declare_parameter<bool>("enable_visualization", true);
        this->declare_parameter<double>("default_dt", 0.1);
        
        // Visualization color parameters
        this->declare_parameter<double>("marker_color_r", 1.0);
        this->declare_parameter<double>("marker_color_g", 0.5);
        this->declare_parameter<double>("marker_color_b", 0.0);
        this->declare_parameter<double>("marker_color_a", 0.8);

        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive/acc", 10);
        target_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/acc/target_speed", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/acc/target_marker", 10);

        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&AdaptiveCruiseControlNode::scanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&AdaptiveCruiseControlNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Adaptive Cruise Control Node initialized");
    }

private:
    double current_ego_speed_ = 0.0;
    double prev_target_dist_ = std::numeric_limits<double>::max();
    rclcpp::Time last_time_;
    bool first_loop_ = true;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_speed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_ego_speed_ = msg->twist.twist.linear.x;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. Perception: Find the closest obstacle in the forward region of interest
        double min_dist = std::numeric_limits<double>::max();
        double min_angle = 0.0;

        int scan_window_deg = this->get_parameter("scan_window_deg").as_int();
        double scan_window_rad = scan_window_deg * M_PI / 180.0;
        double min_valid_range = this->get_parameter("min_valid_range").as_double();

        // Find indices for the scan window
        int center_idx = msg->ranges.size() / 2;
        int window_steps = static_cast<int>(scan_window_rad / msg->angle_increment);

        for (int i = center_idx - window_steps; i <= center_idx + window_steps; ++i) {
            if (i < 0 || i >= static_cast<int>(msg->ranges.size())) {
                continue;
            }

            float range = msg->ranges[i];
            if (std::isfinite(range) && range > min_valid_range) {
                if (range < min_dist) {
                    min_dist = range;
                    min_angle = msg->angle_min + i * msg->angle_increment;
                }
            }
        }

        // 2. Planning: Determine target speed based on lead vehicle presence
        double target_speed = this->get_parameter("max_speed").as_double();
        double free_range_threshold = this->get_parameter("free_range_threshold").as_double();

        if (min_dist > free_range_threshold) {
            // No obstacle in range - free driving mode
            prev_target_dist_ = min_dist;
            first_loop_ = true;
        } else {
            // 3. Control: ACC logic with time-headway policy
            double min_safety_dist = this->get_parameter("min_safety_dist").as_double();
            double time_headway = this->get_parameter("time_headway").as_double();

            // Calculate desired following distance (increases with speed)
            double d_desired = min_safety_dist + (time_headway * current_ego_speed_);

            // Calculate distance error
            double dist_error = min_dist - d_desired;

            // Estimate relative velocity (distance derivative)
            rclcpp::Time now = this->now();
            double dt = (now - last_time_).seconds();
            if (dt <= 0.0) {
                dt = this->get_parameter("default_dt").as_double();
            }

            double relative_vel = 0.0;
            if (!first_loop_) {
                relative_vel = (min_dist - prev_target_dist_) / dt;
            }

            // PD control: speed adjustment based on distance error and relative velocity
            double kp = this->get_parameter("kp_dist").as_double();
            double kd = this->get_parameter("kd_vel").as_double();

            // speed_adjustment = Kp * distance_error + Kd * relative_velocity
            // If closing (relative_vel < 0), we reduce speed
            double speed_adjustment = (kp * dist_error) + (kd * relative_vel);
            target_speed = current_ego_speed_ + speed_adjustment;

            // Update state
            prev_target_dist_ = min_dist;
            last_time_ = now;
            first_loop_ = false;

            // Visualize target point
            if (this->get_parameter("enable_visualization").as_bool()) {
                publishTargetMarker(min_dist, min_angle);
            }

            RCLCPP_DEBUG(this->get_logger(),
                "ACC: dist=%.2f, d_des=%.2f, err=%.2f, v_rel=%.2f, v_cmd=%.2f",
                min_dist, d_desired, dist_error, relative_vel, target_speed);
        }

        // 4. Actuation: Clamp and publish
        double min_speed = this->get_parameter("min_speed").as_double();
        double max_speed = this->get_parameter("max_speed").as_double();
        target_speed = std::clamp(target_speed, min_speed, max_speed);

        publishDriveCommand(target_speed);
        publishTargetSpeed(target_speed);
    }

    void publishDriveCommand(double speed)
    {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.drive.speed = speed;
        // Steering is handled by a separate path follower node
        msg.drive.steering_angle = 0.0;
        drive_pub_->publish(msg);
    }

    void publishTargetSpeed(double speed)
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = speed;
        target_speed_pub_->publish(msg);
    }

    void publishTargetMarker(double distance, double angle)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "acc_target";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position in base_link frame
        marker.pose.position.x = distance * std::cos(angle);
        marker.pose.position.y = distance * std::sin(angle);
        marker.pose.position.z = 0.2;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        // Configurable color for tracked target marker
        marker.color.r = static_cast<float>(this->get_parameter("marker_color_r").as_double());
        marker.color.g = static_cast<float>(this->get_parameter("marker_color_g").as_double());
        marker.color.b = static_cast<float>(this->get_parameter("marker_color_b").as_double());
        marker.color.a = static_cast<float>(this->get_parameter("marker_color_a").as_double());

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_pub_->publish(marker);
    }
};

}  // namespace control_pkg

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control_pkg::AdaptiveCruiseControlNode>());
    rclcpp::shutdown();
    return 0;
}
