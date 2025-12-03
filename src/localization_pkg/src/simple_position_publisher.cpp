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
        // Declare topic parameters (configurable)
        declare_parameter<std::string>("odom_topic", "/car_state/odom_GT");
        declare_parameter<std::string>("scan_topic", "/scan");
        declare_parameter<std::string>("pose_topic", "/localization/pose");
        declare_parameter<std::string>("output_frame", "map");
        
        std::string odom_topic = get_parameter("odom_topic").as_string();
        std::string scan_topic = get_parameter("scan_topic").as_string();
        std::string pose_topic = get_parameter("pose_topic").as_string();
        output_frame_ = get_parameter("output_frame").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Subscribing to odom: %s, scan: %s", odom_topic.c_str(), scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing pose to: %s with frame: %s", pose_topic.c_str(), output_frame_.c_str());

        // 1. Subscriber: Subscribes to the actual F1TENTH vehicle's Odom topic
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&F1TenthOdomSubscriber::odom_callback, this, _1));

        // 2. Subscriber: Subscribes to the Lidar Scan data
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&F1TenthOdomSubscriber::scan_callback, this, _1));

        // 3. Publisher: Publishes the PoseStamped format for internal use (Planning/Control input)
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "F1Tenth Odom and Lidar Subscribers started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert Odometry to PoseStamped for internal pipeline compatibility
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = output_frame_;
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

    // Member variables
    std::string output_frame_{"map"};  // Default output frame for pose messages
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<F1TenthOdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
