#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // Message type for position and orientation

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher()
  : Node("position_publisher"), count_(0) // Initialize node with name "position_publisher"
  {
    // Create Publisher on the "odom" topic with PoseStamped message type
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom", 10);
    
    // Create Timer to call publish_pose() every 100 milliseconds (0.1s)
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&PosePublisher::publish_pose, this));
  }

private:
  void publish_pose()
  {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    
    // 1. Set Header (Time and Frame ID)
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "world"; // The coordinate frame origin

    // 2. Set Position data (x=1.0, y=0.0)
    pose_msg.pose.position.x = 1.0; // 1 meter on the X-axis
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    
    // 3. Set Orientation (Quaternion) - No rotation (w=1.0)
    pose_msg.pose.orientation.w = 1.0; 

    // 4. Log and Publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing Pose %zu", count_++);
    publisher_->publish(pose_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Run the node (starts the infinite loop)
  rclcpp::spin(std::make_shared<PosePublisher>()); 
  rclcpp::shutdown();
  return 0;
}
