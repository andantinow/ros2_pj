// 파일: ~/ros2_ws/src/control_pkg/src/simple_controller.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // /odom 구독
#include "nav_msgs/msg/path.hpp"             // /path 구독
#include "geometry_msgs/msg/twist.hpp"       // /cmd_vel 발행
#include <cmath> 

using std::placeholders::_1;

class SimpleController : public rclcpp::Node
{
public:
    SimpleController() : Node("simple_controller_node")
    {
        // 1. 구독자 생성: /odom (현재 위치)
        odom_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/odom", 10, std::bind(&SimpleController::odom_callback, this, _1));

        // 2. 구독자 생성: /path (목표 경로)
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&SimpleController::path_callback, this, _1));

        // 3. 발행자 생성: /cmd_vel (속도 명령)
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Controller 노드 시작됨. 제어 명령 준비.");
    }

private:
    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        publish_control_command();
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg->poses.empty()) {
            target_pose_ = msg->poses[0]; 
            RCLCPP_INFO(this->get_logger(), "Target Updated: X=%.2f, Y=%.2f", target_pose_.pose.position.x, target_pose_.pose.position.y);
        } else {
            // 경로가 비어있으면 로봇 정지
            stop_robot();
        }
    }

    void publish_control_command()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // [임시 제어 로직] 
        // 목표 지점에 도달했는지 확인 (원점 5, 0으로 가정)
        double dx = target_pose_.pose.position.x - current_pose_.pose.position.x;
        double dy = target_pose_.pose.position.y - current_pose_.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 3.0) { // 원점 주변에 오면 정지 (테스트 용도)
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Target Reached (Distance < 3.0). Stopping.");
            return;
        }

        twist_msg.linear.x = 0.5;  // 전진 속도
        twist_msg.angular.z = 0.0; // 회전 속도 (나중에 Pure Pursuit 로직으로 대체)

        cmd_vel_publisher_->publish(twist_msg);
    }
    
    void stop_robot()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(twist_msg);
    }

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped target_pose_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}
