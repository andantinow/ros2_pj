#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp" 

using std::placeholders::_1;

class F1TenthOdomSubscriber : public rclcpp::Node
{
public:
    F1TenthOdomSubscriber() : Node("f1tenth_odom_subscriber")
    {
        // 1. Subscriber: Subscribes to the actual F1TENTH vehicle's Odom topic
        // TOPIC CHANGED: /localization/pose -> /sim/ego_racecar/odom
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sim/ego_racecar/odom", 10, std::bind(&F1TenthOdomSubscriber::odom_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Localization package (Subscriber) is now waiting for /sim/ego_racecar/odom.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // This confirms that data is successfully flowing from the F1TENTH simulator.
        RCLCPP_INFO_ONCE(this->get_logger(), "Successfully received first Odom message from F1TENTH system.");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<F1TenthOdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
