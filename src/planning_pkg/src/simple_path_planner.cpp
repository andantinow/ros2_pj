#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"             
#include "nav_msgs/msg/odometry.hpp" // Corrected: Odometry is the input type
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include <vector>

using namespace std::chrono_literals;

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        // Subscriber: Subscribes to the vehicle pose from F1TENTH system
        // TOPIC: /sim/ego_racecar/odom (Using Odometry message type)
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sim/ego_racecar/odom", 10, std::bind(&PathPlanner::odom_callback, this, std::placeholders::_1));

        // Publisher: Publishes the optimal path to the control node
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planning/path", 10);

        // Timer to periodically publish the path
        timer_ = this->create_wall_timer(
            2000ms, 
            std::bind(&PathPlanner::publish_path_fixed, this)); 
        
        RCLCPP_INFO(this->get_logger(), "Path Planner Node started, subscribing to /sim/ego_racecar/odom.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // [TODO]: The A* or RRT algorithm will calculate a dynamic path here.
        // The Pose data is available at: msg->pose.pose.position.x etc.
    }
    
    void publish_path_fixed()
    {
        // [Core Logic] Publishes a fixed path for control node to follow
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "world";

        // Fixed 2 Waypoints for testing: (6.0, 1.0) -> (5.0, 0.0)
        auto start_pose = geometry_msgs::msg::PoseStamped();
        start_pose.header = path_msg.header;
        start_pose.pose.position.x = 6.0; start_pose.pose.position.y = 1.0;
        path_msg.poses.push_back(start_pose);
        
        auto end_pose = geometry_msgs::msg::PoseStamped();
        end_pose.header = path_msg.header;
        end_pose.pose.position.x = 5.0; end_pose.pose.position.y = 0.0;
        path_msg.poses.push_back(end_pose);

        path_publisher_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "New fixed path published.");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
