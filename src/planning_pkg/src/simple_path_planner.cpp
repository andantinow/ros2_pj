// 파일: ~/ros2_ws/src/planning_pkg/src/simple_path_planner.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "nav_msgs/msg/path.hpp"             
#include <tf2/LinearMath/Quaternion.h>       
#include <vector>

using namespace std::chrono_literals;

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("simple_path_planner")
    {
        // 구독자: /odom 토픽 구독
        odom_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/odom", 10, std::bind(&PathPlanner::odom_callback, this, std::placeholders::_1));

        // 발행자: /path 토픽 발행
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        RCLCPP_INFO(this->get_logger(), "Path Planner 노드 시작됨. /odom 대기 및 /path 발행 준비.");
    }

private:
    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // **[핵심] /odom 메시지를 받을 때마다 고정된 경로를 발행 (A* 알고리즘 대신 단순 경로)**
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "world";

        // 고정된 3개의 목표 지점 (순수 테스트용 경로)
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        
        // 지점 1: (5, 0)
        geometry_msgs::msg::PoseStamped p1;
        p1.header = path_msg.header;
        p1.pose.position.x = 5.0; p1.pose.position.y = 0.0;
        poses.push_back(p1);

        // 지점 2: (5, 5)
        geometry_msgs::msg::PoseStamped p2;
        p2.header = path_msg.header;
        p2.pose.position.x = 5.0; p2.pose.position.y = 5.0;
        poses.push_back(p2);

        // 지점 3: (0, 5)
        geometry_msgs::msg::PoseStamped p3;
        p3.header = path_msg.header;
        p3.pose.position.x = 0.0; p3.pose.position.y = 5.0;
        poses.push_back(p3);

        path_msg.poses = poses;

        path_publisher_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "New Path published with %zu waypoints.", path_msg.poses.size());
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
