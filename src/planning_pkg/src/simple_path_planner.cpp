#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"             
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planning_pkg/path_utils.hpp"
#include <vector>
#include <string>

using namespace std::chrono_literals;

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        // Declare parameters
        declare_parameter<std::string>("odom_topic", "/car_state/odom_GT");
        declare_parameter<std::string>("path_topic", "/planning/path");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<double>("path_spacing", 0.1);  // meters between path points
        declare_parameter<bool>("enable_smoothing", true);
        declare_parameter<int>("smoothing_window", 5);
        declare_parameter<bool>("publish_fixed_path", true);
        declare_parameter<double>("publish_rate", 1.0);  // Hz

        // Get parameters
        std::string odom_topic = get_parameter("odom_topic").as_string();
        std::string path_topic = get_parameter("path_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        path_spacing_ = get_parameter("path_spacing").as_double();
        enable_smoothing_ = get_parameter("enable_smoothing").as_bool();
        smoothing_window_ = get_parameter("smoothing_window").as_int();
        publish_fixed_path_ = get_parameter("publish_fixed_path").as_bool();
        double publish_rate = get_parameter("publish_rate").as_double();

        // Subscriber: Subscribes to the vehicle pose
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&PathPlanner::odom_callback, this, std::placeholders::_1));

        // Publisher: Publishes the optimal path to the control node
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);

        // Timer to periodically publish the path
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period, 
            std::bind(&PathPlanner::publish_path, this)); 
        
        RCLCPP_INFO(this->get_logger(), 
                    "Path Planner Node started. Odom: %s, Path: %s, Frame: %s",
                    odom_topic.c_str(), path_topic.c_str(), frame_id_.c_str());
        
        if (publish_fixed_path_) {
            generate_fixed_path();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = msg;
        if (!odom_received_) {
            RCLCPP_INFO(this->get_logger(), 
                        "Received first odometry. Position: (%.2f, %.2f)",
                        msg->pose.pose.position.x, msg->pose.pose.position.y);
            odom_received_ = true;
        }
        // [TODO]: The A* or RRT algorithm will calculate a dynamic path here.
        // The Pose data is available at: msg->pose.pose.position.x etc.
    }
    
    void generate_fixed_path()
    {
        // Generate a simple straight path for testing
        fixed_path_.header.frame_id = frame_id_;
        fixed_path_.poses.clear();
        
        // Create path from (0.5, 1.25) to (10.0, 1.25) with specified spacing
        double start_x = 0.5, start_y = 1.25;
        double end_x = 10.0, end_y = 1.25;
        double dx = end_x - start_x;
        double dy = end_y - start_y;
        double length = std::sqrt(dx * dx + dy * dy);
        int num_points = static_cast<int>(length / path_spacing_) + 1;
        
        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id_;
            pose.pose.position.x = start_x + t * dx;
            pose.pose.position.y = start_y + t * dy;
            pose.pose.position.z = 0.0;
            // Orientation: pointing along path direction
            double yaw = std::atan2(dy, dx);
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            fixed_path_.poses.push_back(pose);
        }
        
        // Apply smoothing if enabled
        if (enable_smoothing_ && fixed_path_.poses.size() >= 3) {
            planning_pkg::smooth_path(fixed_path_, smoothing_window_);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "Generated fixed path with %zu points, length: %.2f m",
                    fixed_path_.poses.size(), planning_pkg::path_length(fixed_path_));
    }
    
    void publish_path()
    {
        if (!publish_fixed_path_ || fixed_path_.poses.empty()) {
            return;
        }
        
        // Update timestamp
        fixed_path_.header.stamp = this->now();
        for (auto & pose : fixed_path_.poses) {
            pose.header.stamp = fixed_path_.header.stamp;
        }
        
        // Validate path before publishing
        if (planning_pkg::validate_path(fixed_path_)) {
            path_publisher_->publish(fixed_path_);
            RCLCPP_DEBUG(this->get_logger(), 
                        "Published path with %zu points", fixed_path_.poses.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Path validation failed, not publishing");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Path fixed_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::string frame_id_{"map"};
    double path_spacing_{0.1};
    bool enable_smoothing_{true};
    int smoothing_window_{5};
    bool publish_fixed_path_{true};
    bool odom_received_{false};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
