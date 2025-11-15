#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <fstream>
#include <vector>
#include <utility>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::vector<std::pair<double,double>> loadWaypoints(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open waypoint file");
    }
    json j;
    ifs >> j;
    std::vector<std::pair<double,double>> waypoints;
    for (auto& marker : j["markers"]) {
        double x = marker["pose"]["position"]["x"];
        double y = marker["pose"]["position"]["y"];
        waypoints.emplace_back(x, y);
    }
    return waypoints;
}

class SimpleController : public rclcpp::Node {
public:
    SimpleController() : Node("simple_controller") {
        std::string waypoint_path = "/home/misys/forza_ws/race_stack/stack_master/maps/small_hall/global_waypoints.json";
        try {
            waypoints_ = loadWaypoints(waypoint_path);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
        }

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/car_state/odom", 10,
            std::bind(&SimpleController::odom_callback, this, std::placeholders::_1));

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        RCLCPP_INFO(this->get_logger(), "Controller initialized");
    }

private:
    std::vector<std::pair<double, double>> waypoints_;

    std::pair<double,double> findLookaheadPoint(const std::vector<std::pair<double,double>>& path,
                                                double x, double y, double lookahead_dist) {
        for (const auto& wp : path) {
            double dist = std::hypot(wp.first - x, wp.second - y);
            if (dist >= lookahead_dist) {
                return wp;
            }
        }
        return path.back();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double yaw = 2.0 * std::atan2(qz, qw);

        auto lookahead_point = findLookaheadPoint(waypoints_, x, y, 1.5);

        double ld_x = lookahead_point.first;
        double ld_y = lookahead_point.second;

        double alpha = std::atan2(ld_y - y, ld_x - x) - yaw;
        while (alpha > M_PI) alpha -= 2*M_PI;
        while (alpha < -M_PI) alpha += 2*M_PI;

        double L = 0.33;
        double steering_angle = std::atan2(2.0 * L * std::sin(alpha) / 1.5, 1.0);

        double target_speed = 1.0;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = target_speed;
        drive_pub_->publish(drive_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>());
    rclcpp::shutdown();
    return 0;
}

