#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"             
#include "nav_msgs/msg/odometry.hpp" // Added for subscription
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include <vector>

using namespace std::chrono_literals;

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        // 구독자: /localization/pose 토픽 구독 (Odometry type)
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/pose", 10, std::bind(&PathPlanner::odom_callback, this, std::placeholders::_1));

        // 발행자: /planning/path 토픽 발행 (Path type)
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planning/path", 10);

        // Path를 주기적으로 발행하기 위한 타이머 (2초)
        timer_ = this->create_wall_timer(
            2000ms, 
            std::bind(&PathPlanner::publish_path_fixed, this)); 

        RCLCPP_INFO(this->get_logger(), "Path Planner Node started, publishing to /planning/path.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // [TODO]: 로봇 위치를 받아 동적으로 경로를 계산하는 로직 삽입 예정
    }
    
    void publish_path_fixed()
    {
        // [Core Logic] 고정된 2개의 Waypoint를 가진 Path 메시지 발행
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "world";

        // Waypoint 1: (6.0, 1.0)
        auto start_pose = geometry_msgs::msg::PoseStamped(); // Corrected PoseStamped
        start_pose.header = path_msg.header;
        start_pose.pose.position.x = 6.0;
        start_pose.pose.position.y = 1.0;
        path_msg.poses.push_back(start_pose);
        
        // Waypoint 2: (5.0, 0.0)
        auto end_pose = geometry_msgs::msg::PoseStamped(); // Corrected PoseStamped
        end_pose.header = path_msg.header;
        end_pose.pose.position.x = 5.0;
        end_pose.pose.position.y = 0.0;
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
