#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

/**
 * @brief ScanProcessorNode processes LiDAR scan data for localization
 * 
 * This node subscribes to laser scan data and extracts useful features
 * for state estimation and localization, including:
 * - Obstacle detection and distance estimation
 * - Wall detection for corridor following
 * - Feature points for scan matching
 */
class ScanProcessorNode : public rclcpp::Node {
public:
    ScanProcessorNode() : Node("scan_processor_node") {
        // Declare parameters
        declare_parameter<std::string>("scan_topic", "/scan");
        declare_parameter<std::string>("processed_scan_topic", "/scan/processed");
        declare_parameter<std::string>("obstacle_topic", "/scan/obstacles");
        declare_parameter<double>("min_range", 0.1);
        declare_parameter<double>("max_range", 10.0);
        declare_parameter<double>("obstacle_threshold", 0.5);  // meters
        declare_parameter<int>("min_cluster_size", 3);
        declare_parameter<double>("cluster_tolerance", 0.1);  // meters
        declare_parameter<bool>("publish_covariance", true);
        declare_parameter<double>("scan_covariance", 0.05);
        
        // Get parameters
        std::string scan_topic = get_parameter("scan_topic").as_string();
        std::string processed_topic = get_parameter("processed_scan_topic").as_string();
        std::string obstacle_topic = get_parameter("obstacle_topic").as_string();
        min_range_ = get_parameter("min_range").as_double();
        max_range_ = get_parameter("max_range").as_double();
        obstacle_threshold_ = get_parameter("obstacle_threshold").as_double();
        min_cluster_size_ = get_parameter("min_cluster_size").as_int();
        cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
        publish_covariance_ = get_parameter("publish_covariance").as_bool();
        scan_covariance_ = get_parameter("scan_covariance").as_double();
        
        RCLCPP_INFO(get_logger(), "ScanProcessorNode initializing...");
        RCLCPP_INFO(get_logger(), "  Subscribing to: %s", scan_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Publishing processed scan to: %s", processed_topic.c_str());
        RCLCPP_INFO(get_logger(), "  Range filter: [%.2f, %.2f] m", min_range_, max_range_);
        
        // Create subscriber for raw laser scan
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(),
            std::bind(&ScanProcessorNode::scanCallback, this, std::placeholders::_1));
        
        // Create publishers
        processed_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            processed_topic, 10);
        
        obstacle_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            obstacle_topic, 10);
        
        feature_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/scan/feature_pose", 10);
        
        RCLCPP_INFO(get_logger(), "ScanProcessorNode initialized successfully");
    }

private:
    // Parameters
    double min_range_;
    double max_range_;
    double obstacle_threshold_;
    int min_cluster_size_;
    double cluster_tolerance_;
    bool publish_covariance_;
    double scan_covariance_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr processed_scan_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr feature_pose_pub_;
    
    /**
     * @brief Process incoming laser scan data
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Filter and process the scan
        auto processed_scan = filterScan(msg);
        
        // Extract features from the scan
        auto features = extractFeatures(processed_scan);
        
        // Detect obstacles
        auto obstacles = detectObstacles(processed_scan);
        
        // Publish processed scan
        processed_scan_pub_->publish(*processed_scan);
        
        // Publish obstacles as Float64MultiArray [angle, distance, ...]
        std_msgs::msg::Float64MultiArray obstacle_msg;
        obstacle_msg.data = obstacles;
        obstacle_pub_->publish(obstacle_msg);
        
        // Publish feature pose if enabled
        if (publish_covariance_ && !features.empty()) {
            publishFeaturePose(msg->header, features);
        }
        
        RCLCPP_DEBUG(get_logger(), "Processed scan: %zu valid ranges, %zu obstacles detected",
                     countValidRanges(processed_scan), obstacles.size() / 2);
    }
    
    /**
     * @brief Filter laser scan by removing invalid ranges
     */
    sensor_msgs::msg::LaserScan::SharedPtr filterScan(
            const sensor_msgs::msg::LaserScan::SharedPtr& input) {
        auto output = std::make_shared<sensor_msgs::msg::LaserScan>(*input);
        
        for (size_t i = 0; i < output->ranges.size(); ++i) {
            float range = output->ranges[i];
            
            // Filter out-of-range measurements
            if (std::isnan(range) || std::isinf(range) ||
                range < min_range_ || range > max_range_) {
                output->ranges[i] = std::numeric_limits<float>::infinity();
            }
        }
        
        return output;
    }
    
    /**
     * @brief Extract feature points from laser scan for matching
     * @return Vector of (angle, range) pairs representing significant features
     */
    std::vector<std::pair<double, double>> extractFeatures(
            const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        std::vector<std::pair<double, double>> features;
        
        if (scan->ranges.size() < 3) {
            return features;
        }
        
        // Find corners and significant range discontinuities
        for (size_t i = 1; i < scan->ranges.size() - 1; ++i) {
            float prev = scan->ranges[i - 1];
            float curr = scan->ranges[i];
            float next = scan->ranges[i + 1];
            
            // Skip invalid measurements
            if (std::isinf(prev) || std::isinf(curr) || std::isinf(next)) {
                continue;
            }
            
            // Detect range discontinuity (potential corner or edge)
            double diff_prev = std::abs(curr - prev);
            double diff_next = std::abs(curr - next);
            
            if (diff_prev > cluster_tolerance_ || diff_next > cluster_tolerance_) {
                double angle = scan->angle_min + i * scan->angle_increment;
                features.emplace_back(angle, curr);
            }
        }
        
        return features;
    }
    
    /**
     * @brief Detect obstacles within threshold distance
     * @return Vector of [angle1, distance1, angle2, distance2, ...]
     */
    std::vector<double> detectObstacles(
            const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        std::vector<double> obstacles;
        
        // Cluster nearby points that are close to the robot
        std::vector<std::vector<size_t>> clusters;
        std::vector<size_t> current_cluster;
        
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            
            if (!std::isinf(range) && range < obstacle_threshold_) {
                if (current_cluster.empty()) {
                    current_cluster.push_back(i);
                } else {
                    // Check if this point is close to the previous point in angle
                    size_t prev_idx = current_cluster.back();
                    double prev_angle = scan->angle_min + prev_idx * scan->angle_increment;
                    double curr_angle = scan->angle_min + i * scan->angle_increment;
                    
                    double prev_x = scan->ranges[prev_idx] * std::cos(prev_angle);
                    double prev_y = scan->ranges[prev_idx] * std::sin(prev_angle);
                    double curr_x = range * std::cos(curr_angle);
                    double curr_y = range * std::sin(curr_angle);
                    
                    double dist = std::hypot(curr_x - prev_x, curr_y - prev_y);
                    
                    if (dist < cluster_tolerance_) {
                        current_cluster.push_back(i);
                    } else {
                        if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
                            clusters.push_back(current_cluster);
                        }
                        current_cluster.clear();
                        current_cluster.push_back(i);
                    }
                }
            } else {
                if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
                    clusters.push_back(current_cluster);
                }
                current_cluster.clear();
            }
        }
        
        // Don't forget the last cluster
        if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
            clusters.push_back(current_cluster);
        }
        
        // Compute centroid for each cluster
        for (const auto& cluster : clusters) {
            double sum_x = 0.0, sum_y = 0.0;
            for (size_t idx : cluster) {
                double angle = scan->angle_min + idx * scan->angle_increment;
                double range = scan->ranges[idx];
                sum_x += range * std::cos(angle);
                sum_y += range * std::sin(angle);
            }
            double centroid_x = sum_x / cluster.size();
            double centroid_y = sum_y / cluster.size();
            
            double centroid_angle = std::atan2(centroid_y, centroid_x);
            double centroid_distance = std::hypot(centroid_x, centroid_y);
            
            obstacles.push_back(centroid_angle);
            obstacles.push_back(centroid_distance);
        }
        
        return obstacles;
    }
    
    /**
     * @brief Publish feature-based pose estimate with covariance
     */
    void publishFeaturePose(const std_msgs::msg::Header& header,
                            const std::vector<std::pair<double, double>>& features) {
        if (features.empty()) {
            return;
        }
        
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = header;
        pose_msg.header.frame_id = "base_link";
        
        // Robot is at origin in its own frame
        pose_msg.pose.pose.position.x = 0.0;
        pose_msg.pose.pose.position.y = 0.0;
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation.w = 1.0;
        
        // Set covariance based on number of features (more features = lower covariance)
        double cov = scan_covariance_ / std::max(1.0, std::sqrt(static_cast<double>(features.size())));
        
        // Covariance matrix (6x6, row-major)
        std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);
        pose_msg.pose.covariance[0] = cov;   // xx
        pose_msg.pose.covariance[7] = cov;   // yy
        pose_msg.pose.covariance[35] = cov;  // yaw-yaw
        
        feature_pose_pub_->publish(pose_msg);
    }
    
    /**
     * @brief Count valid (non-inf) ranges in scan
     */
    size_t countValidRanges(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        size_t count = 0;
        for (const auto& range : scan->ranges) {
            if (!std::isinf(range) && !std::isnan(range)) {
                ++count;
            }
        }
        return count;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
