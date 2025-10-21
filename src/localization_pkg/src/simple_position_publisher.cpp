#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include <cmath> 
#include <tf2/LinearMath/Quaternion.h> // 쿼터니언 변환
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 쿼터니언 변환 헤더

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher() : Node("pose_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/odom", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&PosePublisher::publish_pose, this));
    start_time_ = this->now(); // 시작 시간 기록
    RCLCPP_INFO(this->get_logger(), "Localization Publisher Node Started.");
  }

private:
  void publish_pose()
  {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    auto elapsed_seconds = this->now().seconds() - start_time_.seconds();
    double radius = 2.0; 
    double angular_velocity = 0.5; 
    double theta = angular_velocity * elapsed_seconds;

    // 1. 헤더 설정
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "world";

    // 2. 위치 (Position): 원형 궤적
    pose_msg.pose.position.x = radius * std::cos(theta);
    pose_msg.pose.position.y = radius * std::sin(theta);
    
    // 3. 방향 (Orientation): 이동 방향으로 설정
    tf2::Quaternion q;
    // 진행 방향은 원의 접선 방향 (theta + 90도)
    q.setRPY(0, 0, theta + M_PI_2); 
    pose_msg.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(), "Publishing Pose (X:%.2f, Y:%.2f) count %zu", 
                pose_msg.pose.position.x, pose_msg.pose.position.y, count_++);
    publisher_->publish(pose_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}
