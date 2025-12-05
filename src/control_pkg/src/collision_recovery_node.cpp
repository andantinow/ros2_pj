/**
 * @file collision_recovery_node.cpp
 * @brief Deterministic collision recovery FSM for F1TENTH vehicle
 * 
 * This node implements a Stop-Wait-Reverse finite state machine for collision recovery.
 * It takes highest priority in the Ackermann MUX to override all other drive commands
 * during emergency situations.
 * 
 * States:
 *   IDLE            - Normal operation, monitoring for collision
 *   EMERGENCY_STOP  - Immediate stop after collision detection
 *   WAIT            - Wait for system stabilization
 *   BLIND_REVERSE   - Open-loop reverse (sensor-independent)
 *   RECOVERY_COMPLETE - Return control to main planner
 */

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

namespace control_pkg
{

enum class RecoveryState
{
    IDLE,
    EMERGENCY_STOP,
    WAIT,
    BLIND_REVERSE,
    RECOVERY_COMPLETE
};

class CollisionRecoveryNode : public rclcpp::Node
{
public:
    CollisionRecoveryNode()
    : Node("collision_recovery_node")
    {
        // Parameter declarations with defaults
        this->declare_parameter<double>("jerk_threshold", 18.0);
        this->declare_parameter<double>("min_proximity", 0.15);
        this->declare_parameter<double>("wait_duration", 1.0);
        this->declare_parameter<double>("reverse_duration", 2.0);
        this->declare_parameter<double>("reverse_speed", -1.5);
        this->declare_parameter<double>("emergency_stop_duration", 0.5);
        this->declare_parameter<int>("scan_window", 30);
        this->declare_parameter<double>("imu_dt", 0.01);

        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_recovery", 10);
        recovery_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/recovery/active", 10);

        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&CollisionRecoveryNode::imuCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&CollisionRecoveryNode::scanCallback, this, std::placeholders::_1));

        // 50Hz control loop timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CollisionRecoveryNode::controlLoop, this));

        current_state_ = RecoveryState::IDLE;
        RCLCPP_INFO(this->get_logger(), "Collision Recovery Node initialized - monitoring for collisions");
    }

private:
    RecoveryState current_state_;
    rclcpp::Time state_start_time_;
    double last_accel_x_ = 0.0;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_active_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (current_state_ != RecoveryState::IDLE) {
            return;
        }

        double current_accel = msg->linear_acceleration.x;
        double imu_dt = this->get_parameter("imu_dt").as_double();
        
        // Calculate jerk as change in acceleration over time
        double jerk = std::abs(current_accel - last_accel_x_) / imu_dt;
        last_accel_x_ = current_accel;

        double jerk_threshold = this->get_parameter("jerk_threshold").as_double();
        if (jerk > jerk_threshold) {
            RCLCPP_WARN(this->get_logger(), "COLLISION DETECTED via IMU! Jerk: %.2f m/s^3", jerk);
            transitionTo(RecoveryState::EMERGENCY_STOP);
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (current_state_ != RecoveryState::IDLE) {
            return;
        }

        // Calculate minimum distance in front of the vehicle
        double min_front_dist = std::numeric_limits<double>::max();
        size_t center_idx = msg->ranges.size() / 2;
        int window = this->get_parameter("scan_window").as_int();

        size_t start_idx = (center_idx > static_cast<size_t>(window)) ? center_idx - window : 0;
        size_t end_idx = std::min(center_idx + window, msg->ranges.size() - 1);

        for (size_t i = start_idx; i <= end_idx; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > 0.01) {
                min_front_dist = std::min(min_front_dist, static_cast<double>(msg->ranges[i]));
            }
        }

        double min_proximity = this->get_parameter("min_proximity").as_double();
        if (min_front_dist < min_proximity) {
            RCLCPP_WARN(this->get_logger(), "PROXIMITY TRIGGER! Distance: %.3f m", min_front_dist);
            transitionTo(RecoveryState::EMERGENCY_STOP);
        }
    }

    void transitionTo(RecoveryState new_state)
    {
        current_state_ = new_state;
        state_start_time_ = this->now();

        // Log state transitions for debugging with type-safe conversion
        RCLCPP_INFO(this->get_logger(), "State transition: %s", stateToString(new_state));
    }

    const char* stateToString(RecoveryState state) const
    {
        switch (state) {
            case RecoveryState::IDLE: return "IDLE";
            case RecoveryState::EMERGENCY_STOP: return "EMERGENCY_STOP";
            case RecoveryState::WAIT: return "WAIT";
            case RecoveryState::BLIND_REVERSE: return "BLIND_REVERSE";
            case RecoveryState::RECOVERY_COMPLETE: return "RECOVERY_COMPLETE";
            default: return "UNKNOWN";
        }
    }

    void controlLoop()
    {
        // Publish recovery active status
        auto active_msg = std_msgs::msg::Bool();
        active_msg.data = (current_state_ != RecoveryState::IDLE);
        recovery_active_pub_->publish(active_msg);

        if (current_state_ == RecoveryState::IDLE) {
            return;
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";

        double elapsed = (this->now() - state_start_time_).seconds();

        switch (current_state_) {
            case RecoveryState::EMERGENCY_STOP:
                drive_msg.drive.speed = 0.0;
                drive_msg.drive.steering_angle = 0.0;
                drive_pub_->publish(drive_msg);

                if (elapsed > this->get_parameter("emergency_stop_duration").as_double()) {
                    transitionTo(RecoveryState::WAIT);
                }
                break;

            case RecoveryState::WAIT:
                drive_msg.drive.speed = 0.0;
                drive_msg.drive.steering_angle = 0.0;
                drive_pub_->publish(drive_msg);

                if (elapsed > this->get_parameter("wait_duration").as_double()) {
                    transitionTo(RecoveryState::BLIND_REVERSE);
                }
                break;

            case RecoveryState::BLIND_REVERSE:
                // Open-loop reverse - ignoring sensor data for escape
                drive_msg.drive.speed = this->get_parameter("reverse_speed").as_double();
                drive_msg.drive.steering_angle = 0.0;  // Straight reverse is most reliable
                drive_pub_->publish(drive_msg);

                if (elapsed > this->get_parameter("reverse_duration").as_double()) {
                    transitionTo(RecoveryState::RECOVERY_COMPLETE);
                }
                break;

            case RecoveryState::RECOVERY_COMPLETE:
                RCLCPP_INFO(this->get_logger(), "Recovery complete - resuming normal operation");
                current_state_ = RecoveryState::IDLE;
                break;

            default:
                break;
        }
    }
};

}  // namespace control_pkg

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control_pkg::CollisionRecoveryNode>());
    rclcpp::shutdown();
    return 0;
}
