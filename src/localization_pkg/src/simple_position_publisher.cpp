#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "sensor_msgs/msg/laser_scan.hpp" // Lidar data header

using std::placeholders::_1;
using namespace std::chrono_literals;

class F1TenthOdomSubscriber : public rclcpp::Node
{
public:
    F1TenthOdomSubscriber()
    : Node("f1tenth_odom_subscriber"), count_(0)
    {
        // 1. Subscriber: Subscribes to the actual F1TENTH vehicle's Odom topic
        // TOPIC: /sim/ego_racecar/odom (F1TENTH standard for vehicle pose)
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sim/ego_racecar/odom", 10, std::bind(&F1TenthOdomSubscriber::odom_callback, this, _1));

        // 2. Subscriber: Subscribes to the Lidar Scan data
        // TOPIC: /sim/scan (F1TENTH Lidar data)
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/sim/scan", 10, std::bind(&F1TenthOdomSubscriber::scan_callback, this, _1));

        // 3. Publisher: Publishes the PoseStamped format for internal use (Planning/Control input)
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
        
        RCLCPP_INFO(this->get_logger(), "F1Tenth Odom and Lidar Subscribers started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert Odometry to PoseStamped for internal pipeline compatibility
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map"; // Standard map frame
        pose_msg.pose = msg->pose.pose;

        pose_publisher_->publish(pose_msg);

        // Debug log to confirm Odom reception
        RCLCPP_INFO_ONCE(this->get_logger(), "Successfully received first Odom message.");
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // [TODO]: The Adaptive Estimation (Pillar 2) logic will go here.
        // It will process Lidar data (msg->ranges) to estimate tire grip or map distance.
        RCLCPP_INFO_ONCE(this->get_logger(), "Lidar data received successfully. Ready for EKF/MHE integration.");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<F1TenthOdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
